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
    rclcpp::Node * node,
    ParameterBundle::SharedPtr parameters
) : node_(node),
    parameters_(parameters) {

    RCLCPP_DEBUG(node_->get_logger(), "TrajectoryGeneratorClient::TrajectoryGeneratorClient(): Initializing.");

    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

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

    // RCLCPP_DEBUG(node_->get_logger(), "TrajectoryGeneratorClient::TrajectoryGeneratorClient(): Initialized.");

}

void TrajectoryGeneratorClient::Reset(const State & state) {

    // RCLCPP_DEBUG(node_->get_logger(), "TrajectoryGeneratorClient::Reset(): Resetting.");

    Cancel();

    Reference reference(
        state.position(),
        state.yaw()
    );

    reference_trajectory_adapter_history_ = std::make_shared<History<ReferenceTrajectoryAdapter>>(1);

    reference_trajectory_adapter_history_->Store(ReferenceTrajectoryAdapter(reference));

}

void TrajectoryGeneratorClient::Cancel() {

    // RCLCPP_DEBUG(node_->get_logger(), "TrajectoryGeneratorClient::Cancel(): Canceling.");

    client_->clear_on_new_response_callback();

    busy_ = false;
    done_ = false;

}

Reference TrajectoryGeneratorClient::ComputeReference(
    const State & state,
    const Reference & reference,
    bool set_reference,
    bool reset,
    MPC_mode_t mpc_mode
) {

    // RCLCPP_DEBUG(
    //     node_->get_logger(), 
    //     "TrajectoryGeneratorClient::ComputeReference(): Computing reference trajectory with set_reference = %s, reset = %d, mpc_mode = %s.", 
    //     set_reference ? "true" : "false", 
    //     reset ? "true" : "false", 
    //     mpc_mode
    // );

    Reference ref_out;

    // if (busy()) {

    //     std::string error_message = "TrajectoryGeneratorClient::ComputeReference(): Trajectory generator client is busy, cannot compute trajectory.";

    //     RCLCPP_FATAL(node_->get_logger(), error_message.c_str());

    //     throw std::runtime_error(error_message);

    // }

    if (reset) {

        Reset(state);

    }

    if (parameters_->GetParameter("generate_trajectories_asynchronously_with_delay").as_bool()) {

        // RCLCPP_DEBUG(node_->get_logger(), "TrajectoryGeneratorClient::ComputeReference(): Computing reference trajectory asynchronously.");

        if (reset) {

            // RCLCPP_DEBUG(node_->get_logger(), "TrajectoryGeneratorClient::ComputeReference(): Yielding passthrough state reference on reset.");

            ref_out = GetReferenceTrajectory().references()[0];

        } else {

            // RCLCPP_DEBUG(node_->get_logger(), "TrajectoryGeneratorClient::ComputeReference(): Waiting for asynchronous trajectory generation to finish.");

            rclcpp::Time wait_start_time = node_->now();

            while(!done()) {

                rclcpp::sleep_for(std::chrono::milliseconds(parameters_->GetParameter("generate_trajectories_poll_period_ms").as_int()));

                // if (node_->now() - wait_start_time > rclcpp::Duration::from_nanoseconds(parameters_->GetParameter("generate_trajectories_timeout_ms").as_int() * 1e6)) {

                //     std::string error_message = "TrajectoryGeneratorClient::ComputeReference(): Timeout while waiting for asynchronous trajectory generation.";

                //     RCLCPP_FATAL(node_->get_logger(), error_message.c_str());

                //     throw std::runtime_error(error_message);

                // }

            }

            // RCLCPP_DEBUG(node_->get_logger(), "TrajectoryGeneratorClient::ComputeReference(): Asynchronous trajectory generation finished.");

            ref_out = GetReferenceTrajectory().references()[1];

        }

        // RCLCPP_DEBUG(node_->get_logger(), "TrajectoryGeneratorClient::ComputeReference(): Starting new asynchronous trajectory generation.");

        ComputeReferenceTrajectoryAsync(
            state,
            reference,
            set_reference,
            reset,
            mpc_mode
        );

    } else {

        // RCLCPP_DEBUG(node_->get_logger(), "TrajectoryGeneratorClient::ComputeReference(): Computing reference trajectory blocking.");

        ComputeReferenceTrajectoryBlocking(
            state,
            reference,
            set_reference,
            reset,
            mpc_mode,
            parameters_->GetParameter("generate_trajectories_poll_period_ms").as_int()
        );

        // RCLCPP_DEBUG(node_->get_logger(), "TrajectoryGeneratorClient::ComputeReference(): Reference trajectory computed.");

        ref_out = GetReferenceTrajectory().references()[0];

    }

    // RCLCPP_DEBUG(node_->get_logger(), "TrajectoryGeneratorClient::ComputeReference(): Computed reference:");
    // RCLCPP_DEBUG(node_->get_logger(), "TrajectoryGeneratorClient::ComputeReference(): position: [%f, %f, %f]", ref_out.position()[0], ref_out.position()[1], ref_out.position()[2]);
    // RCLCPP_DEBUG(node_->get_logger(), "TrajectoryGeneratorClient::ComputeReference(): yaw: %f", ref_out.yaw());

    return ref_out;

}

void TrajectoryGeneratorClient::ComputeReferenceTrajectoryAsync(
    const State & state,
    const Reference & reference,
    bool set_reference,
    bool reset,
    MPC_mode_t mpc_mode
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

    request->mpc_mode.mode = mpc_mode;

    // RCLCPP_DEBUG(
    //     node_->get_logger(), 
    //     "TrajectoryGeneratorClient::ComputeReferenceTrajectoryAsync(): Sending request with:");
    // RCLCPP_DEBUG(
    //     node_->get_logger(), 
    //     "TrajectoryGeneratorClient::ComputeReferenceTrajectoryAsync(): state: [%f, %f, %f, %f, %f, %f]", 
    //     state.position()[0], state.position()[1], state.position()[2], state.velocity()[0], state.velocity()[1], state.velocity()[2]
    // );
    // RCLCPP_DEBUG(
    //     node_->get_logger(), 
    //     "TrajectoryGeneratorClient::ComputeReferenceTrajectoryAsync(): reference: [%f, %f, %f, %f]", 
    //     reference.position()[0], reference.position()[1], reference.position()[2], reference.yaw()
    // );
    // RCLCPP_DEBUG(
    //     node_->get_logger(), 
    //     "TrajectoryGeneratorClient::ComputeReferenceTrajectoryAsync(): set_reference: %s", 
    //     set_reference ? "true" : "false"
    // );
    // RCLCPP_DEBUG(
    //     node_->get_logger(), 
    //     "TrajectoryGeneratorClient::ComputeReferenceTrajectoryAsync(): reset: %s", 
    //     reset ? "true" : "false"
    // );
    // RCLCPP_DEBUG(
    //     node_->get_logger(), 
    //     "TrajectoryGeneratorClient::ComputeReferenceTrajectoryAsync(): mpc_mode: %d", 
    //     mpc_mode
    // );

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
    MPC_mode_t mpc_mode,
    unsigned int poll_period_ms
) {

    ComputeReferenceTrajectoryAsync(
        state, 
        reference, 
        set_reference, 
        reset, 
        mpc_mode
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

    // RCLCPP_DEBUG(node_->get_logger(), "TrajectoryGeneratorClient::GetReferenceTrajectory(): Returning reference trajectory:");
    // RCLCPP_DEBUG(node_->get_logger(), "TrajectoryGeneratorClient::GetReferenceTrajectory(): size: %d", ref_traj.references().size());
    
    // for (size_t i = 0; i < ref_traj.references().size(); i++) {

    //     RCLCPP_DEBUG(node_->get_logger(), "TrajectoryGeneratorClient::GetReferenceTrajectory(): reference %d:", i);
    //     RCLCPP_DEBUG(node_->get_logger(), "TrajectoryGeneratorClient::GetReferenceTrajectory(): position: [%f, %f, %f]", ref_traj.references()[i].position()[0], ref_traj.references()[i].position()[1], ref_traj.references()[i].position()[2]);
    //     RCLCPP_DEBUG(node_->get_logger(), "TrajectoryGeneratorClient::GetReferenceTrajectory(): yaw: %f", ref_traj.references()[i].yaw());

    // }

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

    // RCLCPP_DEBUG(node_->get_logger(), "TrajectoryGeneratorClient::serviceResultCallback(): Service result callback.");

    // Get response
    auto response = future.get();

    // Check if response is valid
    if (!response) {

        std::string error_message = "TrajectoryGeneratorClient::serviceResultCallback(): Service response is invalid.";

        RCLCPP_ERROR(node_->get_logger(), error_message.c_str());

        throw std::runtime_error(error_message);

    }

    // RCLCPP_DEBUG(node_->get_logger(), "TrajectoryGeneratorClient::serviceResultCallback(): Received response:");

    // for (size_t i = 0; i < response->reference_trajectory.references.size(); i++) {

    //     RCLCPP_DEBUG(node_->get_logger(), "TrajectoryGeneratorClient::serviceResultCallback(): reference %d:", i);
    //     RCLCPP_DEBUG(node_->get_logger(), "TrajectoryGeneratorClient::serviceResultCallback(): position: [%f, %f, %f]", response->reference_trajectory.references[i].position.x, response->reference_trajectory.references[i].position.y, response->reference_trajectory.references[i].position.z);
    //     RCLCPP_DEBUG(node_->get_logger(), "TrajectoryGeneratorClient::serviceResultCallback(): yaw: %f", response->reference_trajectory.references[i].yaw);

    // }

    // Create reference trajectory adapter
    ReferenceTrajectoryAdapter reference_trajectory_adapter(response->reference_trajectory);

    // Add reference trajectory adapter to history
    reference_trajectory_adapter_history_->Store(reference_trajectory_adapter);

    // Set busy flag
    busy_ = false;

    // Set done flag
    done_ = true;

}