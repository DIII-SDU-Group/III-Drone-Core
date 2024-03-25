/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/maneuver/maneuver_reference_client.hpp>

using namespace iii_drone::control::maneuver;
using namespace iii_drone::utils;
using namespace iii_drone::types;
using namespace iii_drone::adapters;
using namespace iii_drone::adapters::px4;
using namespace iii_drone::control;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

ManeuverReferenceClient::ManeuverReferenceClient(
    rclcpp::Node * node,
    History<adapters::px4::VehicleOdometryAdapter>::SharedPtr vehicle_odometry_adapter_history,
    bool use_nans_when_hovering,
    int max_failed_attempts_during_maneuver,
    int get_reference_timeout_ms,
    std::function<void()> on_fail_during_maneuver
) : node_(node),
    vehicle_odometry_adapter_history_(vehicle_odometry_adapter_history),
    reference_mode_(reference_mode_t::PASSTHROUGH),
    reference_(Reference()),
    use_nans_when_hovering_(use_nans_when_hovering),
    max_failed_attempts_during_maneuver_(max_failed_attempts_during_maneuver),
    get_reference_timeout_ms_(get_reference_timeout_ms),
    on_fail_during_maneuver_(on_fail_during_maneuver) {
    
    get_reference_client_ = node_->create_client<iii_drone_interfaces::srv::GetReference>("/control/maneuver_controller/get_reference");

}

void ManeuverReferenceClient::UpdateReference() {

    if (reference_mode_.Load() == reference_mode_t::MANEUVER) {
        return;
    }

    if (vehicle_odometry_adapter_history_->empty()) {
        return;
    }
    
    if (use_nans_when_hovering_) {

        State state = (*vehicle_odometry_adapter_history_)[0].ToState();

        std::lock_guard<std::mutex> lock(reference_mutex_);

        reference_ = Reference(
            state.position(),
            state.yaw(),
            vector_t::Constant(NAN),
            NAN,
            vector_t::Constant(NAN),
            NAN,
            state.stamp()
        );

    } else {

        ReferenceAdapter reference_adapter = ReferenceAdapter((*vehicle_odometry_adapter_history_)[0].ToState()); // Implement nans parameter

        std::lock_guard<std::mutex> lock(reference_mutex_);

        reference_ = reference_adapter.reference();

    }
}

void ManeuverReferenceClient::SetReferenceModePassthrough() {

    if (reference_mode_.Load() == reference_mode_t::MANEUVER) {
        RCLCPP_WARN(node_->get_logger(), "ManeuverReferenceClient::SetReferenceModePassthrough(): Cannot set reference mode to PASSTHROUGH while in MANEUVER mode.");
        return;
    }

    reference_mode_.Store(reference_mode_t::PASSTHROUGH);

}

void ManeuverReferenceClient::SetReferenceModeHover() {

    if (reference_mode_.Load() == reference_mode_t::MANEUVER) {
        RCLCPP_WARN(node_->get_logger(), "ManeuverReferenceClient::SetReferenceModeHover(): Cannot set reference mode to HOVER while in MANEUVER mode.");
        return;
    }

    reference_mode_.Store(reference_mode_t::HOVER);

    UpdateReference();

}

void ManeuverReferenceClient::StartManeuver() {

    if (reference_mode_.Load() == reference_mode_t::MANEUVER) {
        RCLCPP_WARN(node_->get_logger(), "ManeuverReferenceClient::StartManeuver(): Cannot start maneuver while already in MANEUVER mode.");
        return;
    }

    reference_mode_.Store(reference_mode_t::MANEUVER);

}

void ManeuverReferenceClient::StopManeuver() {

    if (reference_mode_.Load() != reference_mode_t::MANEUVER) {
        RCLCPP_WARN(node_->get_logger(), "ManeuverReferenceClient::StopManeuver(): Cannot stop maneuver while not in MANEUVER mode.");
        return;
    }

    reference_mode_.Store(reference_mode_t::HOVER);

    UpdateReference();

}

Reference ManeuverReferenceClient::GetReference(double dt) {

    static int failed_attempts = 0;
    Reference reference;

    switch(reference_mode_.Load()) {
        case reference_mode_t::PASSTHROUGH:

            failed_attempts = 0;
            reference = vehicle_odometry_adapter_history_->empty() ? Reference() : Reference((*vehicle_odometry_adapter_history_)[0].ToState());
            break;

        case reference_mode_t::HOVER: {

            failed_attempts = 0;

            std::lock_guard<std::mutex> lock(reference_mutex_);

            reference = reference_;
            break;

        }
        case reference_mode_t::MANEUVER: {

            if (!get_reference_client_->wait_for_service(std::chrono::nanoseconds(static_cast<int64_t>(5e6)))) {
                RCLCPP_ERROR(node_->get_logger(), "ManeuverReferenceClient::GetReference(): Service not available within 5 milliseconds.");
                reference = vehicle_odometry_adapter_history_->empty() ? Reference() : Reference((*vehicle_odometry_adapter_history_)[0].ToState());
                break;
            }

            if (!get_reference_client_->service_is_ready()) {
                RCLCPP_ERROR(node_->get_logger(), "ManeuverReferenceClient::GetReference(): Service not ready.");
                reference = vehicle_odometry_adapter_history_->empty() ? Reference() : Reference((*vehicle_odometry_adapter_history_)[0].ToState());
                break;
            }

            bool finished = false;
            std::shared_ptr<iii_drone_interfaces::srv::GetReference::Response> response;

            auto result_callback = [&finished, &response](const rclcpp::Client<iii_drone_interfaces::srv::GetReference>::SharedFuture future) {

                response = future.get();
                finished = true;

            };

            auto request = std::make_shared<iii_drone_interfaces::srv::GetReference::Request>();

            rclcpp::Time start_time = rclcpp::Clock().now();

            auto result = get_reference_client_->async_send_request(
                request,
                result_callback
            );

            rclcpp::Rate rate(1e6);

            bool failed = false;

            while(rclcpp::ok() && !finished) {

                if ((rclcpp::Clock().now() - start_time).nanoseconds() > get_reference_timeout_ms_ * 1e6) {
                    failed = true;
                    break;
                }

                rate.sleep();

            }

            std::lock_guard<std::mutex> lock(reference_mutex_);


            if (failed || !response->is_valid) {

                failed_attempts++;

                if (failed_attempts > max_failed_attempts_during_maneuver_) {

                    RCLCPP_ERROR(
                        node_->get_logger(), 
                        "ManeuverReferenceClient::GetReference(): Failed to acquire valid reference after %d attempts. Calling on failed callback and returning passthrough reference.", 
                        max_failed_attempts_during_maneuver_
                    );
                    on_fail_during_maneuver_();

                } else {

                    RCLCPP_WARN(node_->get_logger(), "ManeuverReferenceClient::GetReference(): Failed to acquire valid reference. Using passthrough reference.");

                }

                reference = vehicle_odometry_adapter_history_->empty() ? Reference() : Reference((*vehicle_odometry_adapter_history_)[0].ToState());
                break;

            } 

            reference = ReferenceAdapter(response->reference).reference();

            break;

        }
    }

    return reference_;

}