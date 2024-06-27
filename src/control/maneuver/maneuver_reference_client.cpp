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
    rclcpp_lifecycle::LifecycleNode * node,
    History<adapters::px4::VehicleOdometryAdapter>::SharedPtr vehicle_odometry_adapter_history,
    ParameterBundle::SharedPtr parameters
) : node_(node),
    vehicle_odometry_adapter_history_(vehicle_odometry_adapter_history),
    reference_mode_(reference_mode_t::PASSTHROUGH),
    reference_(Reference()),
    parameters_(parameters) {

    get_reference_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    
    get_reference_client_ = node_->create_client<iii_drone_interfaces::srv::GetReference>(
        "/control/maneuver_controller/get_reference",
        rmw_qos_profile_services_default,
        get_reference_cb_group_
    );

    reference_mode_publisher_ = node_->create_publisher<iii_drone_interfaces::msg::StringStamped>(
        "maneuver_reference_client/reference_mode",
        10
    );

}

void ManeuverReferenceClient::UpdateReference(bool force) {

    if (!force && isManeuverMode()) {
        RCLCPP_WARN(node_->get_logger(), "ManeuverReferenceClient::UpdateReference(): Cannot update reference while in MANEUVER mode, returning.");
        return;
    }

    if (vehicle_odometry_adapter_history_->empty()) {
        RCLCPP_WARN(node_->get_logger(), "ManeuverReferenceClient::UpdateReference(): Vehicle odometry adapter history is empty, returning.");
        return;
    }

    RCLCPP_DEBUG(
        node_->get_logger(), 
        "ManeuverReferenceClient::UpdateReference(): Updating hover reference with current state."
    );

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

    if (isManeuverMode()) {
        RCLCPP_WARN(node_->get_logger(), "ManeuverReferenceClient::SetReference(): Cannot set reference while in a maneuver is active.");
        return;
    }

    RCLCPP_DEBUG(
        node_->get_logger(), 
        "ManeuverReferenceClient::SetReference(): Setting reference."
    );

    std::lock_guard<std::mutex> lock(reference_mutex_);

    reference_ = reference;

}

void ManeuverReferenceClient::SetReferenceModePassthrough() {

    auto reference_mode = reference_mode_.Load();

    if(isManeuverMode(reference_mode)) {
        RCLCPP_WARN(node_->get_logger(), "ManeuverReferenceClient::SetReferenceModePassthrough(): Cannot set reference mode to PASSTHROUGH while in a maneuver mode.");
        return;
    }

    if (reference_mode_.Load() == reference_mode_t::PASSTHROUGH) {
        return;
    }

    RCLCPP_DEBUG(
        node_->get_logger(), 
        "ManeuverReferenceClient::SetReferenceModePassthrough(): Setting reference mode to PASSTHROUGH."
    );

    reference_mode_.Store(reference_mode_t::PASSTHROUGH);

}

void ManeuverReferenceClient::SetReferenceModeHover(bool force) {

    auto reference_mode = reference_mode_.Load();

    if (!force && isManeuverMode(reference_mode)) {
        RCLCPP_WARN(node_->get_logger(), "ManeuverReferenceClient::SetReferenceModeHover(): Cannot set reference mode to HOVER while in a maneuver mode.");
        return;
    }

    UpdateReference(true);

    if (reference_mode == reference_mode_t::HOVER) {
        return;
    }

    RCLCPP_DEBUG(
        node_->get_logger(), 
        "ManeuverReferenceClient::SetReferenceModeHover(): Setting reference mode to HOVER."
    );

    reference_mode_.Store(reference_mode_t::HOVER);

}

bool ManeuverReferenceClient::StartManeuver() {

    auto reference_mode = reference_mode_.Load();

    if (reference_mode == WAIT_FOR_MANEUVER_START || reference_mode == MANEUVER) {
        RCLCPP_ERROR(node_->get_logger(), "ManeuverReferenceClient::StartManeuver(): Cannot start maneuver while a maneuver mode is already active");
        return false;
    }

    if (reference_mode == WAIT_FOR_MANEUVER_STOP) {
        RCLCPP_DEBUG(node_->get_logger(), "ManeuverReferenceClient::StartManeuver(): Stopping currently waiting maneuver.");
        stopManeuverPrematurely();
    }

    RCLCPP_DEBUG(
        node_->get_logger(), 
        "ManeuverReferenceClient::StartManeuver(): Starting maneuver."
    );

    if (reference_mode == PASSTHROUGH) {
        UpdateReference();
    } else if (reference_mode != HOVER && reference_mode != WAIT_FOR_MANEUVER_STOP) {
        RCLCPP_ERROR(node_->get_logger(), "ManeuverReferenceClient::StartManeuver(): Reference mode is not PASSTHROUGH or HOVER and not a MANEUVER mode.");
        return false;
    }

    reference_mode_.Store(reference_mode_t::WAIT_FOR_MANEUVER_START);

    if (*stop_maneuver_timer_ != nullptr) {
        (*stop_maneuver_timer_)->cancel();
    }

    maneuver_start_time_.Store(rclcpp::Clock().now());

    return true;

}

void ManeuverReferenceClient::StopManeuver() {

    if (!isManeuverMode()) {
        RCLCPP_WARN(node_->get_logger(), "ManeuverReferenceClient::StopManeuver(): Cannot stop maneuver while a maneuver mode is not active.");
        return;
    }

    RCLCPP_DEBUG(
        node_->get_logger(), 
        "ManeuverReferenceClient::StopManeuver(): Stopping maneuver."
    );

    reference_mode_.Store(reference_mode_t::HOVER);

    UpdateReference();

    if (*stop_maneuver_timer_ != nullptr) {
        (*stop_maneuver_timer_)->cancel();
        stop_maneuver_timer_.Store(nullptr);
    }

}

void ManeuverReferenceClient::StopManeuver(Reference reference) {

    if (!isManeuverMode()) {
        RCLCPP_WARN(node_->get_logger(), "ManeuverReferenceClient::StopManeuver(Reference): Cannot stop maneuver while a maneuver mode is not active.");
        return;
    }

    RCLCPP_DEBUG(
        node_->get_logger(), 
        "ManeuverReferenceClient::StopManeuver(Reference): Stopping maneuver with given reference."
    );

    reference_mode_.Store(reference_mode_t::HOVER);

    SetReference(reference);

    if (*stop_maneuver_timer_ != nullptr) {
        (*stop_maneuver_timer_)->cancel();
        stop_maneuver_timer_.Store(nullptr);
    }

}

void ManeuverReferenceClient::StopManeuverAfterTimeout(int timeout_ms) {

    if (!isManeuverMode()) {
        RCLCPP_WARN(node_->get_logger(), "ManeuverReferenceClient::StopManeuverAfterTimeout(): Cannot stop maneuver while a maneuver mode is not active.");
        return;
    }

    RCLCPP_DEBUG(
        node_->get_logger(), 
        "ManeuverReferenceClient::StopManeuverAfterTimeout(): Stopping maneuver after %d milliseconds.", 
        timeout_ms
    );

    if (*stop_maneuver_timer_ != nullptr && !(*stop_maneuver_timer_)->is_canceled()) {
        RCLCPP_WARN(node_->get_logger(), "ManeuverReferenceClient::StopManeuverAfterTimeout(): Timer already running. Resetting timer.");
        (*stop_maneuver_timer_)->cancel();
        stop_maneuver_timer_.Store(nullptr);
    }

    RCLCPP_DEBUG(
        node_->get_logger(), 
        "ManeuverReferenceClient::StopManeuverAfterTimeout(): Storing mode WAIT_FOR_MANEUVER_STOP."
    );
    reference_mode_.Store(WAIT_FOR_MANEUVER_STOP);

    stop_maneuver_timer_callback_ = [this]() -> void {
        RCLCPP_DEBUG(node_->get_logger(), "ManeuverReferenceClient::StopManeuverAfterTimeout(): Timer expired. Stopping maneuver.");
        StopManeuver();
    };

    stop_maneuver_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(timeout_ms),
        [this]() -> void {
            (*stop_maneuver_timer_callback_)();
        }
    );

}

void ManeuverReferenceClient::StopManeuverAfterTimeout(
    Reference reference, 
    int timeout_ms
) {

    if (!isManeuverMode()) {
        RCLCPP_WARN(node_->get_logger(), "ManeuverReferenceClient::StopManeuverAfterTimeout(Reference): Cannot stop maneuver while not in MANEUVER mode.");
        return;
    }

    RCLCPP_DEBUG(
        node_->get_logger(), 
        "ManeuverReferenceClient::StopManeuverAfterTimeout(Reference): Stopping maneuver after %d milliseconds with given reference.", 
        timeout_ms
    );

    if (*stop_maneuver_timer_ != nullptr && !(*stop_maneuver_timer_)->is_canceled()) {
        RCLCPP_WARN(node_->get_logger(), "ManeuverReferenceClient::StopManeuverAfterTimeout(Reference): Timer already running. Resetting timer.");
        (*stop_maneuver_timer_)->cancel();
        stop_maneuver_timer_.Store(nullptr);
    }

    RCLCPP_DEBUG(
        node_->get_logger(), 
        "ManeuverReferenceClient::StopManeuverAfterTimeout(): Storing mode WAIT_FOR_MANEUVER_STOP."
    );
    reference_mode_.Store(WAIT_FOR_MANEUVER_STOP);

    stop_maneuver_timer_callback_ = [this, reference]() -> void {
        RCLCPP_DEBUG(node_->get_logger(), "ManeuverReferenceClient::StopManeuverAfterTimeout(Reference): Timer expired. Stopping maneuver with given reference.");
        StopManeuver(reference);
    };

    stop_maneuver_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(timeout_ms),
        [this, reference]() -> void {
            (*stop_maneuver_timer_callback_)();
        }
    );

}

Reference ManeuverReferenceClient::GetReference(
    double,
    std::function<void()> on_fail_during_maneuver
) {

    static int failed_attempts = 0;
    Reference reference;

    iii_drone_interfaces::msg::StringStamped reference_mode_msg;

    switch(reference_mode_.Load()) {
        case reference_mode_t::PASSTHROUGH:

            failed_attempts = 0;
            reference = vehicle_odometry_adapter_history_->empty() ? Reference() : Reference((*vehicle_odometry_adapter_history_)[0].ToState());

            reference_mode_msg.data = "passthrough";

            break;

        case reference_mode_t::HOVER: {

            failed_attempts = 0;

            std::lock_guard<std::mutex> lock(reference_mutex_);

            reference = reference_;

            reference_mode_msg.data = "hover";

            break;

        }
        case WAIT_FOR_MANEUVER_START: {
    
            failed_attempts = 0;

            rclcpp::Duration elapsed_time_since_start = rclcpp::Clock().now() - *maneuver_start_time_;

            int elapsed_ms = elapsed_time_since_start.nanoseconds() / 1e6;

            if (elapsed_ms > parameters_->GetParameter("wait_for_maneuver_start_timeout_ms").as_int()) {

                RCLCPP_ERROR(
                    node_->get_logger(), 
                    "ManeuverReferenceClient::GetReference(): WAIT_FOR_MANEUVER_START: Timeout while waiting for maneuver start after %d milliseconds. Calling on fail callback and switch to HOVER mode.", 
                    elapsed_ms
                );
                on_fail_during_maneuver();

                SetReferenceModeHover(true);

                reference_mode_msg.data = "hover";
                break;

            }
            
            bool success = getReferenceFromServer(reference);

            if (!success) {

                RCLCPP_DEBUG(node_->get_logger(), "ManeuverReferenceClient::GetReference(): WAIT_FOR_MANEUVER_START: Reference is not yet valid, returning hover reference.");

                reference = reference_;

                reference_mode_msg.data = "wait_for_maneuver_start";

                break;

            }

            if (reference_mode_.Load() == reference_mode_t::WAIT_FOR_MANEUVER_START) {

                RCLCPP_DEBUG(node_->get_logger(), "ManeuverReferenceClient::GetReference(): WAIT_FOR_MANEUVER_START: Reference is valid, switching to MANEUVER mode.");

                reference_mode_.Store(reference_mode_t::MANEUVER);

                reference_mode_msg.data = "maneuver";

            } else if (reference_mode_.Load() == reference_mode_t::WAIT_FOR_MANEUVER_STOP) {

                RCLCPP_DEBUG(node_->get_logger(), "ManeuverReferenceClient::GetReference(): WAIT_FOR_MANEUVER_START: Mode switched to WAIT_FOR_MANEUVER_STOP while waiting.");

                reference_mode_msg.data = "wait_for_maneuver_stop";

            } else {

                RCLCPP_ERROR(node_->get_logger(), "ManeuverReferenceClient::GetReference(): WAIT_FOR_MANEUVER_START: Mode switched to %d while waiting.", reference_mode_.Load());

                reference_mode_msg.data = "error";

            }

            break;

        }
        case reference_mode_t::MANEUVER: {

            bool success = getReferenceFromServer(reference);

            // Check if mode is hovering:
            if (reference_mode_.Load() == reference_mode_t::HOVER) {
                RCLCPP_DEBUG(node_->get_logger(), "ManeuverReferenceClient::GetReference(): MANEUVER: Mode switched to HOVER while waiting, returning hover reference");
                reference = reference_;
                reference_mode_msg.data = "hover";
                break;
            }

            if (!success) {

                failed_attempts++;

                if (failed_attempts >= parameters_->GetParameter("max_failed_attempts_during_maneuver").as_int()) {

                    RCLCPP_ERROR(
                        node_->get_logger(), 
                        "ManeuverReferenceClient::GetReference(): MANEUVER: Failed to acquire valid reference after %d attempts. Calling on failed callback and setting mode to HOVER.", 
                        failed_attempts
                    );

                    on_fail_during_maneuver();

                    SetReferenceModeHover(true);

                    reference = reference_;

                    reference_mode_msg.data = "hover";

                    break;

                }

                RCLCPP_WARN(node_->get_logger(), "ManeuverReferenceClient::GetReference(): MANEUVER: Failed to acquire valid reference. Using nan reference.");

                reference_mode_msg.data = "nan";

                break;

            } 

            reference_mode_msg.data = "maneuver";

            break;

        }
        case WAIT_FOR_MANEUVER_STOP: {

            failed_attempts = 0;

            bool success = getReferenceFromServer(reference);

            if (!success) {

                RCLCPP_WARN(node_->get_logger(), "ManeuverReferenceClient::GetReference(): WAIT_FOR_MANEUVER_STOP: Reference is not valid, switching to HOVER mode prematurely.");

                stopManeuverPrematurely();

                reference = reference_;

                reference_mode_msg.data = "hover";

                break;

            }

            reference_mode_msg.data = "wait_for_maneuver_stop";

        }
    }

    reference_mode_msg.stamp = rclcpp::Clock().now();

    reference_mode_publisher_->publish(reference_mode_msg);

    return reference;

}

void ManeuverReferenceClient::stopManeuverPrematurely() {

    (*stop_maneuver_timer_)->cancel();
    (*stop_maneuver_timer_callback_)();
    
}

bool ManeuverReferenceClient::isManeuverMode() {

    auto reference_mode = reference_mode_.Load();

    return isManeuverMode(reference_mode);

}

bool ManeuverReferenceClient::isManeuverMode(reference_mode_t reference_mode) {

    switch(reference_mode) {
        case WAIT_FOR_MANEUVER_START:
        case MANEUVER:
        case WAIT_FOR_MANEUVER_STOP:
            return true;
        default:
            return false;
    }

}

bool ManeuverReferenceClient::getReferenceFromServer(Reference & reference) {

    Reference nan_ref(
        point_t::Constant(NAN),
        NAN,
        vector_t::Constant(NAN),
        NAN,
        vector_t::Constant(NAN),
        NAN
    );

    if (!get_reference_client_->wait_for_service(std::chrono::nanoseconds(static_cast<int64_t>(5e6)))) {
        RCLCPP_ERROR(node_->get_logger(), "ManeuverReferenceClient::getReferenceFromServer(): Service not available within 5 milliseconds, using nan reference.");
        reference = nan_ref;
        return false;

    }

    if (!get_reference_client_->service_is_ready()) {
        RCLCPP_ERROR(node_->get_logger(), "ManeuverReferenceClient::getReferenceFromServer(): Service not ready, using nan reference.");
        reference = nan_ref;
        return false;
    }

    std::shared_ptr<iii_drone_interfaces::srv::GetReference::Response> response;
    auto request = std::make_shared<iii_drone_interfaces::srv::GetReference::Request>();

    auto result = get_reference_client_->async_send_request(
        request
    );

    bool failed = false;

    // result.wait();

    int get_reference_timeout_ms = parameters_->GetParameter("get_reference_timeout_ms").as_int();

    if (result.wait_for(std::chrono::milliseconds(get_reference_timeout_ms)) == std::future_status::ready) {
        // RCLCPP_DEBUG(node_->get_logger(), "ManeuverReferenceClient::GetReference(): Received response.");
        response = result.get();
    } else {
        RCLCPP_DEBUG(node_->get_logger(), "ManeuverReferenceClient::GetReference(): Timeout while waiting for response.");
        failed = true;
    }

    if (failed) {

        RCLCPP_WARN(node_->get_logger(), "ManeuverReferenceClient::getReferenceFromServer(): Timeout while waiting for response.");
        reference = nan_ref;
        return false;
    
    }

    if(!response->is_valid) {

        RCLCPP_WARN(node_->get_logger(), "ManeuverReferenceClient::getReferenceFromServer(): Received invalid reference.");
        reference = nan_ref;
        return false;

    }

    reference = ReferenceAdapter(response->reference).reference();

    return true;

}