/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/trajectory_generator_client.hpp>

using namespace iii_drone::control;
using namespace iii_drone::adapters;
using namespace iii_drone::utils;
using namespace iii_drone::configuration;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

TrajectoryGeneratorClient::TrajectoryGeneratorClient(
    rclcpp_lifecycle::LifecycleNode * node,
    ParameterBundle::SharedPtr parameters,
    rclcpp::CallbackGroup::SharedPtr callback_group
) : node_(node),
    parameters_(parameters) {

    RCLCPP_DEBUG(node_->get_logger(), "TrajectoryGeneratorClient::TrajectoryGeneratorClient(): Initializing.");

    // callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_ = callback_group;

    // Create service client
    client_ = node_->create_client<iii_drone_interfaces::srv::ComputeReferenceTrajectory>(
        "/control/trajectory_generator/compute_reference_trajectory",
        rmw_qos_profile_services_default,
        callback_group_
    );

    // Wait for service
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(node_->get_logger(), "Compute reference trajectory service not available, waiting again...");
    }

    Reset();

}

TrajectoryGeneratorClient::~TrajectoryGeneratorClient() {

    RCLCPP_DEBUG(node_->get_logger(), "TrajectoryGeneratorClient::~TrajectoryGeneratorClient(): Destructing.");

}

void TrajectoryGeneratorClient::Reset(const State & state) {

    RCLCPP_DEBUG(
        node_->get_logger(), 
        "TrajectoryGeneratorClient::Reset(): Resetting trajectory generator client."
    );

    Cancel();

    Reference reference(
        state.position(),
        state.yaw()
    );

    reference_trajectory_adapter_history_ = std::make_shared<History<ReferenceTrajectoryAdapter>>(1);

    reference_trajectory_adapter_history_->Store(ReferenceTrajectoryAdapter(reference));

}

void TrajectoryGeneratorClient::Cancel() {

    busy_ = false;
    done_ = false;

}

Reference TrajectoryGeneratorClient::ComputeReference(
    const State & state,
    const Reference & reference,
    bool set_reference,
    bool reset,
    trajectory_mode_t trajectory_mode,
    bool use_mpc
) {

    Reference ref_out;

    if (reset) {

        Reset(state);

    }

    if (use_mpc && parameters_->GetParameter("generate_trajectories_asynchronously_with_delay").as_bool()) {

        if (reset) {

            ref_out = GetReferenceTrajectory().references()[0];

        } else {

            rclcpp::Time wait_start_time = node_->now();

            while(!done()) {

                rclcpp::sleep_for(std::chrono::milliseconds(parameters_->GetParameter("generate_trajectories_poll_period_ms").as_int()));

            }

            ref_out = GetReferenceTrajectory().references()[1];

        }

        ComputeReferenceTrajectoryAsync(
            state,
            reference,
            set_reference,
            reset,
            trajectory_mode,
            use_mpc
        );

    } else {

        ComputeReferenceTrajectoryBlocking(
            state,
            reference,
            set_reference,
            reset,
            trajectory_mode,
            parameters_->GetParameter("generate_trajectories_poll_period_ms").as_int(),
            use_mpc
        );

        ref_out = GetReferenceTrajectory().references()[0];

    }

    // RCLCPP_DEBUG(
    //     node_->get_logger(), 
    //     "TrajectoryGeneratorClient::ComputeReference(): Reference computed: %f, %f, %f, %f.",
    //     ref_out.position()[0],
    //     ref_out.position()[1],
    //     ref_out.position()[2],
    //     ref_out.yaw()
    // );

    return ref_out;

}

void TrajectoryGeneratorClient::ComputeReferenceTrajectoryAsync(
    const State & state,
    const Reference & reference,
    bool set_reference,
    bool reset,
    trajectory_mode_t trajectory_mode,
    bool use_mpc
) {

    if (busy()) {

        std::string error_message = "TrajectoryGeneratorClient::ComputeReferenceTrajectoryAsync(): Trajectory generator is busy.";

        RCLCPP_ERROR(node_->get_logger(), error_message.c_str());

        throw std::runtime_error(error_message);

    }

    // Set busy flag
    busy_ = true;

    // Set done flag
    done_ = false;

    // Create request
    auto request = std::make_shared<iii_drone_interfaces::srv::ComputeReferenceTrajectory::Request>();

    // Set request
    request->state = StateAdapter(state).ToMsg();
    request->reference = ReferenceAdapter(reference).ToMsg();

    request->set_reference = set_reference;
    request->reset = reset;

    request->trajectory_mode.mode = trajectory_mode;

    request->use_mpc = use_mpc;

    // Send request
    auto future = client_->async_send_request(
        request,
        std::bind(
            &TrajectoryGeneratorClient::serviceResultCallback,
            this, 
            std::placeholders::_1
        )
    );

    future_ = future.future;

}

void TrajectoryGeneratorClient::ComputeReferenceTrajectoryBlocking(
    const State & state,
    const Reference & reference,
    bool set_reference,
    bool reset,
    trajectory_mode_t trajectory_mode,
    unsigned int poll_period_ms,
    bool use_mpc
) {

    ComputeReferenceTrajectoryAsync(
        state, 
        reference, 
        set_reference, 
        reset, 
        trajectory_mode,
        use_mpc
    );

    rclcpp::Time wait_start_time = node_->now();

    while (!done()) {

        rclcpp::sleep_for(std::chrono::milliseconds(poll_period_ms));

        if (node_->now() - wait_start_time > rclcpp::Duration::from_nanoseconds(parameters_->GetParameter("generate_trajectories_timeout_ms").as_int() * 1e6)) {

            std::string error_message = "TrajectoryGeneratorClient::ComputeReferenceTrajectoryBlocking(): Timeout while waiting for blocking trajectory generation.";

            RCLCPP_FATAL(node_->get_logger(), error_message.c_str());

            throw std::runtime_error(error_message);

        }

    }

}

ReferenceTrajectory TrajectoryGeneratorClient::GetReferenceTrajectory() const {

    ReferenceTrajectory ref_traj = (*reference_trajectory_adapter_history_)[0].reference_trajectory();

    return ref_traj;

}

bool TrajectoryGeneratorClient::busy() const {

    return busy_;

}

bool TrajectoryGeneratorClient::done() const {

    return done_;

}

void TrajectoryGeneratorClient::serviceResultCallback(
    rclcpp::Client<iii_drone_interfaces::srv::ComputeReferenceTrajectory>::SharedFuture future
) {

    // Get response
    auto response = future.get();

    // Check if response is valid
    if (!response) {

        std::string error_message = "TrajectoryGeneratorClient::serviceResultCallback(): Service response is invalid.";

        RCLCPP_ERROR(node_->get_logger(), error_message.c_str());

        throw std::runtime_error(error_message);

    }

    // Create reference trajectory adapter
    ReferenceTrajectoryAdapter reference_trajectory_adapter(response->reference_trajectory);

    // Add reference trajectory adapter to history
    reference_trajectory_adapter_history_->Store(reference_trajectory_adapter);

    // Set busy flag
    busy_ = false;

    // Set done flag
    done_ = true;

}