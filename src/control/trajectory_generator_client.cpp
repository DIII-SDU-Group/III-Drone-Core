/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/trajectory_generator_client.hpp>

#include <cmath>
#include <limits>

using namespace iii_drone::control;
using namespace iii_drone::adapters;
using namespace iii_drone::utils;
using namespace iii_drone::configuration;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

TrajectoryGeneratorClient::TrajectoryGeneratorClient(
    rclcpp_lifecycle::LifecycleNode * node,
    Configuration::SharedPtr parameters,
    rclcpp::CallbackGroup::SharedPtr callback_group
) : node_(node),
    configuration_(parameters) {

    RCLCPP_DEBUG(node_->get_logger(), "TrajectoryGeneratorClient::TrajectoryGeneratorClient(): Initializing.");

    // callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_ = callback_group;

    // Create service client
    client_ = node_->create_client<iii_drone_interfaces::srv::ComputeReferenceTrajectory>(
        "/control/trajectory_generator/compute_reference_trajectory",
        rclcpp::ServicesQoS(),
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

    // A response can still arrive for a request sent before destruction.
    callback_lifetime_.Close();

    RCLCPP_DEBUG(node_->get_logger(), "TrajectoryGeneratorClient::~TrajectoryGeneratorClient(): Destructing.");

}

void TrajectoryGeneratorClient::Reset(const State & state) {

    RCLCPP_DEBUG(
        node_->get_logger(), 
        "TrajectoryGeneratorClient::Reset(): Resetting trajectory generator client."
    );

    Reference reference(state);

    auto history = std::make_shared<History<ReferenceTrajectoryAdapter>>(1);
    history->Store(ReferenceTrajectoryAdapter(reference));

    std::lock_guard<std::mutex> lock(trajectory_state_mutex_);
    ++trajectory_request_generation_;
    reference_trajectory_adapter_history_ = std::move(history);
    last_mpc_submission_stamp_ns_.reset();
    initial_plan_ready_ = false;
    last_request_success_ = true;
    last_error_message_ = std::string();
    busy_ = false;
    done_ = false;

}

void TrajectoryGeneratorClient::Cancel() {

    std::lock_guard<std::mutex> lock(trajectory_state_mutex_);
    ++trajectory_request_generation_;
    last_mpc_submission_stamp_ns_.reset();
    initial_plan_ready_ = false;
    last_request_success_ = true;
    last_error_message_ = std::string();
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

    if (use_mpc && configuration_->GetParameter("/control/maneuver_controller/generate_trajectories_asynchronously_with_delay").as_bool()) {

        bool initial_plan_ready;
        std::string startup_error;
        {
            std::lock_guard<std::mutex> lock(trajectory_state_mutex_);
            initial_plan_ready = initial_plan_ready_;
            if (done_ && !last_request_success_) {
                startup_error = last_error_message_.Load();
            }
            if (initial_plan_ready_) {
                if (!reference_trajectory_adapter_history_ ||
                    reference_trajectory_adapter_history_->empty()) {
                    throw std::runtime_error(
                        "TrajectoryGeneratorClient::ComputeReference(): ready trajectory history is empty"
                    );
                }
                const ReferenceTrajectory latest_trajectory =
                    (*reference_trajectory_adapter_history_)[0].reference_trajectory();
                ref_out = latest_trajectory.references().size() > 1 ?
                    latest_trajectory.references()[1] :
                    latest_trajectory.references()[0];
            } else {
                if (!reference_trajectory_adapter_history_ ||
                    reference_trajectory_adapter_history_->empty()) {
                    throw std::runtime_error(
                        "TrajectoryGeneratorClient::ComputeReference(): initialization seed is unavailable"
                    );
                }
                // Keep the moving-state reset seed in history for the solver,
                // but publish only a position/yaw hold until this generation
                // has a successful plan. Use the seed's original pose/stamp
                // even if a later state arrives while planning is pending.
                const Reference seed =
                    (*reference_trajectory_adapter_history_)[0].reference_trajectory().references()[0];
                ref_out = seed.CopyWithNans();
            }
        }

        if (!startup_error.empty()) {
            throw std::runtime_error(startup_error);
        }

        if (!initial_plan_ready && !done()) {
            RCLCPP_DEBUG(
                node_->get_logger(),
                "TrajectoryGeneratorClient::ComputeReference(): Asynchronous trajectory is still pending. Reusing initialization hold."
            );
        }

        if (!busy()) {
            if (!use_mpc || admitMpcPlanningRequest(state)) {
                ComputeReferenceTrajectoryAsync(
                    state,
                    reference,
                    set_reference,
                    reset,
                    trajectory_mode,
                    use_mpc
                );
            } else {
                RCLCPP_DEBUG(
                    node_->get_logger(),
                    "TrajectoryGeneratorClient::ComputeReference(): MPC planning is limited to the configured control step; reusing the cached trajectory."
                );
            }
        } else {
            RCLCPP_DEBUG(
                node_->get_logger(),
                "TrajectoryGeneratorClient::ComputeReference(): Previous asynchronous trajectory request is still busy; not submitting a new request."
            );
        }

    } else {

        if (!busy()) {
            if (!use_mpc || admitMpcPlanningRequest(state)) {
                ComputeReferenceTrajectoryBlocking(
                    state,
                    reference,
                    set_reference,
                    reset,
                    trajectory_mode,
                    configuration_->GetParameter("/control/maneuver_controller/generate_trajectories_poll_period_ms").as_int(),
                    use_mpc
                );
            } else {
                RCLCPP_DEBUG(
                    node_->get_logger(),
                    "TrajectoryGeneratorClient::ComputeReference(): MPC planning is limited to the configured control step; reusing the cached trajectory."
                );
            }
        } else {
            RCLCPP_WARN(
                node_->get_logger(),
                "TrajectoryGeneratorClient::ComputeReference(): Previous blocking trajectory request is still busy; reusing latest reference trajectory."
            );
        }

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

Reference TrajectoryGeneratorClient::ComputeReference(
    const Reference & start_reference,
    const Reference & reference,
    bool set_reference,
    bool reset,
    trajectory_mode_t trajectory_mode
) {

    if (reset) {
        auto history = std::make_shared<History<ReferenceTrajectoryAdapter>>(1);
        history->Store(ReferenceTrajectoryAdapter(start_reference));
        std::lock_guard<std::mutex> lock(trajectory_state_mutex_);
        ++trajectory_request_generation_;
        reference_trajectory_adapter_history_ = std::move(history);
        last_mpc_submission_stamp_ns_.reset();
        initial_plan_ready_ = false;
        last_request_success_ = true;
        last_error_message_ = std::string();
        busy_ = false;
        done_ = false;
    }

    if (!busy()) {
        ComputeReferenceTrajectoryBlocking(
            start_reference,
            reference,
            set_reference,
            reset,
            trajectory_mode,
            configuration_->GetParameter("/control/maneuver_controller/generate_trajectories_poll_period_ms").as_int()
        );
    } else {
        RCLCPP_WARN(
            node_->get_logger(),
            "TrajectoryGeneratorClient::ComputeReference(): Previous blocking trajectory request is still busy; reusing latest reference trajectory."
        );
    }

    if (!last_request_success_) {
        throw std::runtime_error(last_error_message_.Load());
    }

    return GetReferenceTrajectory().references()[0];

}

bool TrajectoryGeneratorClient::admitMpcPlanningRequest(const State & state) {
    const double control_step_s = configuration_->GetParameter("/control/dt").as_double();
    const long double control_step_ns = std::round(
        static_cast<long double>(control_step_s) * 1.0e9L
    );
    if (
        !std::isfinite(control_step_s) || control_step_s <= 0.0 ||
        control_step_ns < 1.0L ||
        control_step_ns > static_cast<long double>(std::numeric_limits<std::int64_t>::max())
    ) {
        throw std::runtime_error(
            "TrajectoryGeneratorClient requires a positive finite /control/dt"
        );
    }

    const std::int64_t state_stamp_ns = state.stamp().nanoseconds();
    std::lock_guard<std::mutex> lock(trajectory_state_mutex_);
    if (!last_mpc_submission_stamp_ns_) {
        last_mpc_submission_stamp_ns_ = state_stamp_ns;
        return true;
    }

    const long double elapsed_ns = static_cast<long double>(state_stamp_ns)
        - static_cast<long double>(*last_mpc_submission_stamp_ns_);
    if (elapsed_ns < 0.0L || elapsed_ns >= control_step_ns) {
        // A backwards source-clock jump establishes a fresh timeline rather
        // than blocking submissions until the old clock value is reached.
        last_mpc_submission_stamp_ns_ = state_stamp_ns;
        return true;
    }
    return false;
}

void TrajectoryGeneratorClient::ComputeReferenceTrajectoryAsync(
    const State & state,
    const Reference & reference,
    bool set_reference,
    bool reset,
    trajectory_mode_t trajectory_mode,
    bool use_mpc
) {

    std::uint64_t request_generation;
    {
        std::lock_guard<std::mutex> lock(trajectory_state_mutex_);
        if (busy_) {

            std::string error_message = "TrajectoryGeneratorClient::ComputeReferenceTrajectoryAsync(): Trajectory generator is busy.";

            RCLCPP_ERROR(node_->get_logger(), error_message.c_str());

            throw std::runtime_error(error_message);

        }

        busy_ = true;
        done_ = false;
        last_request_success_ = true;
        last_error_message_ = "";
        request_generation = ++trajectory_request_generation_;
    }

    // Create request
    auto request = std::make_shared<iii_drone_interfaces::srv::ComputeReferenceTrajectory::Request>();

    // Set request
    request->state = StateAdapter(state).ToMsg();
    request->reference = ReferenceAdapter(reference).ToMsg();
    request->use_start_reference = false;
    request->start_reference = ReferenceAdapter(Reference(state)).ToMsg();

    request->set_reference = set_reference;
    request->reset = reset;

    request->trajectory_mode.mode = trajectory_mode;

    request->use_mpc = use_mpc;

    // Send request
    auto future = client_->async_send_request(
        request,
        [this, request_generation, lifetime = callback_lifetime_.token()](
            rclcpp::Client<iii_drone_interfaces::srv::ComputeReferenceTrajectory>::SharedFuture response_future
        ) {
            const auto alive = lifetime.Enter();
            if (!alive.owns_lock()) return;
            serviceResultCallback(response_future, request_generation);
        }
    );

    {
        std::lock_guard<std::mutex> lock(trajectory_state_mutex_);
        if (request_generation == trajectory_request_generation_) {
            future_ = future.future;
        }
    }

}

void TrajectoryGeneratorClient::ComputeReferenceTrajectoryAsync(
    const Reference & start_reference,
    const Reference & reference,
    bool set_reference,
    bool reset,
    trajectory_mode_t trajectory_mode
) {

    std::uint64_t request_generation;
    {
        std::lock_guard<std::mutex> lock(trajectory_state_mutex_);
        if (busy_) {

        std::string error_message = "TrajectoryGeneratorClient::ComputeReferenceTrajectoryAsync(): Trajectory generator is busy.";

        RCLCPP_ERROR(node_->get_logger(), error_message.c_str());

        throw std::runtime_error(error_message);

        }

        busy_ = true;
        done_ = false;
        last_request_success_ = true;
        last_error_message_ = "";
        request_generation = ++trajectory_request_generation_;
    }

    auto request = std::make_shared<iii_drone_interfaces::srv::ComputeReferenceTrajectory::Request>();

    request->state = StateAdapter(State(
        start_reference.position(),
        start_reference.velocity(),
        start_reference.yaw(),
        iii_drone::types::vector_t(0.0, 0.0, start_reference.yaw_rate()),
        start_reference.stamp()
    )).ToMsg();
    request->reference = ReferenceAdapter(reference).ToMsg();
    request->use_start_reference = true;
    request->start_reference = ReferenceAdapter(start_reference).ToMsg();

    request->set_reference = set_reference;
    request->reset = reset;
    request->trajectory_mode.mode = trajectory_mode;
    request->use_mpc = false;

    auto future = client_->async_send_request(
        request,
        [this, request_generation, lifetime = callback_lifetime_.token()](
            rclcpp::Client<iii_drone_interfaces::srv::ComputeReferenceTrajectory>::SharedFuture response_future
        ) {
            const auto alive = lifetime.Enter();
            if (!alive.owns_lock()) return;
            serviceResultCallback(response_future, request_generation);
        }
    );

    {
        std::lock_guard<std::mutex> lock(trajectory_state_mutex_);
        if (request_generation == trajectory_request_generation_) {
            future_ = future.future;
        }
    }

}

bool TrajectoryGeneratorClient::ComputeReferenceTrajectoryBlocking(
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

        if (node_->now() - wait_start_time > rclcpp::Duration::from_nanoseconds(configuration_->GetParameter("/control/maneuver_controller/generate_trajectories_timeout_ms").as_int() * 1e6)) {

            RCLCPP_WARN(
                node_->get_logger(),
                "TrajectoryGeneratorClient::ComputeReferenceTrajectoryBlocking(): Timed out waiting for blocking trajectory generation after %ld ms. Reusing latest reference trajectory.",
                configuration_->GetParameter("/control/maneuver_controller/generate_trajectories_timeout_ms").as_int()
            );

            return false;

        }

    }

    if (!last_request_success_) {
        throw std::runtime_error(last_error_message_.Load());
    }

    return true;

}

bool TrajectoryGeneratorClient::ComputeReferenceTrajectoryBlocking(
    const Reference & start_reference,
    const Reference & reference,
    bool set_reference,
    bool reset,
    trajectory_mode_t trajectory_mode,
    unsigned int poll_period_ms
) {

    ComputeReferenceTrajectoryAsync(
        start_reference,
        reference,
        set_reference,
        reset,
        trajectory_mode
    );

    rclcpp::Time wait_start_time = node_->now();

    while (!done()) {

        rclcpp::sleep_for(std::chrono::milliseconds(poll_period_ms));

        if (node_->now() - wait_start_time > rclcpp::Duration::from_nanoseconds(configuration_->GetParameter("/control/maneuver_controller/generate_trajectories_timeout_ms").as_int() * 1e6)) {

            RCLCPP_WARN(
                node_->get_logger(),
                "TrajectoryGeneratorClient::ComputeReferenceTrajectoryBlocking(): Timed out waiting for blocking trajectory generation after %ld ms. Reusing latest reference trajectory.",
                configuration_->GetParameter("/control/maneuver_controller/generate_trajectories_timeout_ms").as_int()
            );

            return false;

        }

    }

    if (!last_request_success_) {
        throw std::runtime_error(last_error_message_.Load());
    }

    return true;

}

ReferenceTrajectory TrajectoryGeneratorClient::GetReferenceTrajectory() const {

    History<ReferenceTrajectoryAdapter>::SharedPtr history;
    {
        std::lock_guard<std::mutex> lock(trajectory_state_mutex_);
        history = reference_trajectory_adapter_history_;
    }
    if (!history || history->empty()) {
        throw std::runtime_error("TrajectoryGeneratorClient::GetReferenceTrajectory(): no trajectory is available");
    }
    ReferenceTrajectory ref_traj = (*history)[0].reference_trajectory();

    return ref_traj;

}

bool TrajectoryGeneratorClient::busy() const {

    return busy_;

}

bool TrajectoryGeneratorClient::done() const {

    return done_;

}

bool TrajectoryGeneratorClient::lastRequestSucceeded() const {

    return last_request_success_;

}

std::string TrajectoryGeneratorClient::lastErrorMessage() const {

    return last_error_message_.Load();

}

void TrajectoryGeneratorClient::serviceResultCallback(
    rclcpp::Client<iii_drone_interfaces::srv::ComputeReferenceTrajectory>::SharedFuture future,
    std::uint64_t request_generation
) {

    // Get response
    auto response = future.get();

    // Check if response is valid
    if (!response) {

        std::string error_message = "TrajectoryGeneratorClient::serviceResultCallback(): Service response is invalid.";

        RCLCPP_ERROR(node_->get_logger(), error_message.c_str());

        throw std::runtime_error(error_message);

    }

    std::lock_guard<std::mutex> lock(trajectory_state_mutex_);
    if (request_generation != trajectory_request_generation_) {
        RCLCPP_DEBUG(
            node_->get_logger(),
            "TrajectoryGeneratorClient::serviceResultCallback(): Ignoring superseded trajectory response generation %lu (active %lu).",
            static_cast<unsigned long>(request_generation),
            static_cast<unsigned long>(trajectory_request_generation_)
        );
        return;
    }

    const bool has_references = !response->reference_trajectory.references.empty();
    const bool response_success = response->success && has_references;
    last_request_success_ = response_success;
    last_error_message_ = response->success && !has_references ?
        "TrajectoryGeneratorClient::serviceResultCallback(): Successful response contained no references." :
        (!response_success && response->error_message.empty() ?
            "TrajectoryGeneratorClient::serviceResultCallback(): Trajectory generation failed." :
            response->error_message);
    if (response_success) {
        initial_plan_ready_ = true;

        // A failed or empty replan must not overwrite this generation's last
        // usable trajectory.
        ReferenceTrajectoryAdapter reference_trajectory_adapter(response->reference_trajectory);
        reference_trajectory_adapter_history_->Store(reference_trajectory_adapter);
    }

    busy_ = false;
    done_ = true;

}
