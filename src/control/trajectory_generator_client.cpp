/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/trajectory_generator_client.hpp>

using namespace iii_drone::control;
using namespace iii_drone::adapters;
using namespace iii_drone::utils;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

TrajectoryGeneratorClient::TrajectoryGeneratorClient(rclcpp::Node * node)
    : node_(node)
{

    // Create service client
    client_ = node_->create_client<iii_drone_interfaces::srv::ComputeReferenceTrajectory>(
        "/control/trajectory_generator/compute_reference_trajectory"
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

void TrajectoryGeneratorClient::Reset(const State & state) {

    Cancel();

    Reference reference(
        state.position(),
        state.yaw()
    );

    reference_trajectory_adapter_history_ = History<ReferenceTrajectoryAdapter>(1);

    reference_trajectory_adapter_history_ = ReferenceTrajectoryAdapter(reference);

}

void TrajectoryGeneratorClient::Cancel() {

    client_->clear_on_new_response_callback();

    busy_ = false;
    done_ = false;

}

void TrajectoryGeneratorClient::ComputeReferenceTrajectoryAsync(
    const State & state,
    const Reference & reference,
    bool set_reference,
    bool reset,
    MPC_mode_t mpc_mode
) {

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

    while (!done()) {

        rclcpp::sleep_for(std::chrono::milliseconds(poll_period_ms));

    }

}

ReferenceTrajectory TrajectoryGeneratorClient::GetReferenceTrajectory() const {

    return reference_trajectory_adapter_history_[0].reference_trajectory();

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
    reference_trajectory_adapter_history_.Store(reference_trajectory_adapter);

    // Set busy flag
    busy_ = false;

    // Set done flag
    done_ = true;

}