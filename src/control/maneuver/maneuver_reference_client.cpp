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
using namespace iii_drone::configuration;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

ManeuverReferenceClient::ManeuverReferenceClient(
    rclcpp::Node * node,
    History<adapters::px4::VehicleOdometryAdapter>::SharedPtr vehicle_odometry_adapter_history,
    ParameterBundle::SharedPtr parameters,
    std::function<void()> on_fail_during_maneuver
) : node_(node),
    vehicle_odometry_adapter_history_(vehicle_odometry_adapter_history),
    reference_mode_(reference_mode_t::PASSTHROUGH),
    reference_(Reference()),
    parameters_(parameters),
    on_fail_during_maneuver_(on_fail_during_maneuver) {

    get_reference_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    
    get_reference_client_ = node_->create_client<iii_drone_interfaces::srv::GetReference>(
        "/control/maneuver_controller/get_reference",
        rmw_qos_profile_services_default,
        get_reference_cb_group_
    );

}

void ManeuverReferenceClient::UpdateReference() {

    if (reference_mode_.Load() == reference_mode_t::MANEUVER) {
        return;
    }

    if (vehicle_odometry_adapter_history_->empty()) {
        return;
    }

    State state = (*vehicle_odometry_adapter_history_)[0].ToState();

    std::lock_guard<std::mutex> lock(reference_mutex_);
    
    if (parameters_->GetParameter("use_nans_when_hovering").as_bool()) {

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

        reference_ = Reference(
            state.position(),
            state.yaw(),
            vector_t::Zero(),
            0,
            vector_t::Zero(),
            0,
            state.stamp()
        );

    }
}

void ManeuverReferenceClient::SetReference(Reference reference) {

    if (reference_mode_.Load() == reference_mode_t::MANEUVER) {
        RCLCPP_WARN(node_->get_logger(), "ManeuverReferenceClient::SetReference(): Cannot set reference while in MANEUVER mode.");
        return;
    }

    std::lock_guard<std::mutex> lock(reference_mutex_);

    reference_ = reference;

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

    reference_mode_.Store(reference_mode_t::MANEUVER);

    std::lock_guard<std::mutex> lock(stop_maneuver_timer_mutex_);

    if (stop_maneuver_timer_ != nullptr) {
        stop_maneuver_timer_->cancel();
    }

}

void ManeuverReferenceClient::StopManeuver() {

    if (reference_mode_.Load() != reference_mode_t::MANEUVER) {
        RCLCPP_WARN(node_->get_logger(), "ManeuverReferenceClient::StopManeuver(): Cannot stop maneuver while not in MANEUVER mode.");
        return;
    }

    reference_mode_.Store(reference_mode_t::HOVER);

    UpdateReference();

    std::lock_guard<std::mutex> lock(stop_maneuver_timer_mutex_);

    if (stop_maneuver_timer_ != nullptr) {
        stop_maneuver_timer_->cancel();
        stop_maneuver_timer_ = nullptr;
    }

}

void ManeuverReferenceClient::StopManeuver(Reference reference) {

    if (reference_mode_.Load() != reference_mode_t::MANEUVER) {
        RCLCPP_WARN(node_->get_logger(), "ManeuverReferenceClient::StopManeuver(): Cannot stop maneuver while not in MANEUVER mode.");
        return;
    }

    reference_mode_.Store(reference_mode_t::HOVER);

    SetReference(reference);

    std::lock_guard<std::mutex> lock(stop_maneuver_timer_mutex_);

    if (stop_maneuver_timer_ != nullptr) {
        stop_maneuver_timer_->cancel();
        stop_maneuver_timer_ = nullptr;
    }

}

void ManeuverReferenceClient::StopManeuverAfterTimeout(int timeout_ms) {

    if (reference_mode_.Load() != reference_mode_t::MANEUVER) {
        RCLCPP_WARN(node_->get_logger(), "ManeuverReferenceClient::StopManeuverAfterTimeout(): Cannot stop maneuver while not in MANEUVER mode.");
        return;
    }

    std::lock_guard<std::mutex> lock(stop_maneuver_timer_mutex_);

    if (stop_maneuver_timer_ != nullptr && !stop_maneuver_timer_->is_canceled()) {
        RCLCPP_WARN(node_->get_logger(), "ManeuverReferenceClient::StopManeuverAfterTimeout(): Timer already running. Resetting timer.");
        stop_maneuver_timer_->cancel();
        stop_maneuver_timer_ = nullptr;
    }

    stop_maneuver_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(timeout_ms),
        [this]() -> void {
            StopManeuver();
        }
    );

}

void ManeuverReferenceClient::StopManeuverAfterTimeout(
    Reference reference, 
    int timeout_ms
) {

    if (reference_mode_.Load() != reference_mode_t::MANEUVER) {
        RCLCPP_WARN(node_->get_logger(), "ManeuverReferenceClient::StopManeuverAfterTimeout(): Cannot stop maneuver while not in MANEUVER mode.");
        return;
    }

    std::lock_guard<std::mutex> lock(stop_maneuver_timer_mutex_);

    if (stop_maneuver_timer_ != nullptr && !stop_maneuver_timer_->is_canceled()) {
        RCLCPP_WARN(node_->get_logger(), "ManeuverReferenceClient::StopManeuverAfterTimeout(): Timer already running. Resetting timer.");
        stop_maneuver_timer_->cancel();
        stop_maneuver_timer_ = nullptr;
    }

    stop_maneuver_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(timeout_ms),
        [this, reference]() -> void {
            StopManeuver(reference);
        }
    );

}

Reference ManeuverReferenceClient::GetReference(double) {

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

            // // RCLCPP_DEBUG(node_->get_logger(), "ManeuverReferenceClient::GetReference(): Sending request to maneuver controller.");

            bool finished = false;
            std::shared_ptr<iii_drone_interfaces::srv::GetReference::Response> response;
            auto request = std::make_shared<iii_drone_interfaces::srv::GetReference::Request>();

            auto result = get_reference_client_->async_send_request(
                request
            );

            bool failed = false;

            // // RCLCPP_DEBUG(node_->get_logger(), "ManeuverReferenceClient::GetReference(): Waiting for response for %d milliseconds.", get_reference_timeout_ms_);

            result.wait();
            // if (result.wait_for(std::chrono::milliseconds(get_reference_timeout_ms_)) == std::future_status::ready) {
                // // RCLCPP_DEBUG(node_->get_logger(), "ManeuverReferenceClient::GetReference(): Received response.");
                response = result.get();
            // } else {
            //     // RCLCPP_DEBUG(node_->get_logger(), "ManeuverReferenceClient::GetReference(): Timeout while waiting for response.");
            //     failed = true;
            // }

            std::lock_guard<std::mutex> lock(reference_mutex_);

            if (failed || !response->is_valid) {

                if (!failed && !response->is_valid) {
                    RCLCPP_WARN(node_->get_logger(), "ManeuverReferenceClient::GetReference(): Received invalid reference.");
                }

                failed_attempts++;

                if (failed_attempts >= parameters_->GetParameter("max_failed_attempts_during_maneuver").as_int()) {

                    RCLCPP_ERROR(
                        node_->get_logger(), 
                        "ManeuverReferenceClient::GetReference(): Failed to acquire valid reference after %d attempts. Calling on failed callback and returning passthrough reference.", 
                        failed_attempts
                    );
                    on_fail_during_maneuver_();

                } else {

                    RCLCPP_WARN(node_->get_logger(), "ManeuverReferenceClient::GetReference(): Failed to acquire valid reference. Using passthrough reference.");

                }

                // // RCLCPP_DEBUG(node_->get_logger(), "ManeuverReferenceClient::GetReference(): Failed attempts: %d", failed_attempts);
                // // RCLCPP_DEBUG(node_->get_logger(), "ManeuverReferenceClient::GetReference(): Returning passthrough reference.");

                reference = vehicle_odometry_adapter_history_->empty() ? Reference() : Reference((*vehicle_odometry_adapter_history_)[0].ToState());
                break;

            } 

            reference = ReferenceAdapter(response->reference).reference();

            // RCLCPP_DEBUG(node_->get_logger(), "ManeuverReferenceClient::GetReference(): Received valid reference:");
            // RCLCPP_DEBUG(node_->get_logger(), "ManeuverReferenceClient::GetReference(): position: [%f, %f, %f]", reference.position().x(), reference.position().y(), reference.position().z());
            // RCLCPP_DEBUG(node_->get_logger(), "ManeuverReferenceClient::GetReference(): yaw: %f", reference.yaw());

            break;

        }
    }

    return reference;

}