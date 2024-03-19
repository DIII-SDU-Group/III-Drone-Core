/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/maneuver/hover_on_cable_maneuver_server.hpp>

using namespace iii_drone::control::maneuver;
using namespace iii_drone::control;
using namespace iii_drone::types;

/*****************************************************************************/
// Implementation:
/*****************************************************************************/

HoverOnCableManeuverServer::HoverOnCableManeuverServer(
    rclcpp::Node * node,
    CombinedDroneAwarenessHandler::SharedPtr combined_drone_awareness_handler,
    const std::string & action_name,
    unsigned int wait_for_execute_poll_ms,
    unsigned int evaluate_done_poll_ms,
    double default_target_z_velocity,
    double default_target_yaw_rate
) : ManeuverServer(
        node,
        combined_drone_awareness_handler,
        action_name,
        wait_for_execute_poll_ms,
        evaluate_done_poll_ms
    ),
    has_on_fail_callback_(false),
    target_z_velocity_(default_target_z_velocity),
    target_yaw_rate_(default_target_yaw_rate) {
    

    createServer<HoverOnCable>();

}

bool HoverOnCableManeuverServer::CanExecuteManeuver(
    const Maneuver & maneuver,
    const combined_drone_awareness_t & drone_awareness
) const {

    if (maneuver.maneuver_type() != MANEUVER_TYPE_HOVER_ON_CABLE) {
        return false;
    }

    hover_on_cable_maneuver_params_t params(maneuver.maneuver_params());

    if (params.target_z_velocity < 0.0) {
        return false;
    }
    
    if (!validateAwareness(drone_awareness)) {
        return false;
    }

    return true;

}

combined_drone_awareness_t HoverOnCableManeuverServer::ExpectedAwarenessAfterExecution(const Maneuver & maneuver) {

    hover_on_cable_maneuver_params_t params(maneuver.maneuver_params());

    combined_drone_awareness_t awareness_after;

    awareness_after.armed = true;
    awareness_after.offboard = true;
    awareness_after.target_adapter = iii_drone::adapters::TargetAdapter(
        iii_drone::adapters::TARGET_TYPE_CABLE,
        0,
        "",
        iii_drone::types::transform_matrix_t()
    );
    awareness_after.target_position_known = true;
    awareness_after.drone_location = DRONE_LOCATION_ON_CABLE;

    return awareness_after;

}

void HoverOnCableManeuverServer::RegisterOnFailCallback(std::function<void()> on_fail_callback) {

    on_fail_callback_ = on_fail_callback;
    has_on_fail_callback_ = true;

}

bool HoverOnCableManeuverServer::Update(
    double target_z_velocity,
    double target_yaw_rate
) {

    if (target_z_velocity < 0.0) {
        return false;
    }
    
    target_z_velocity_ = target_z_velocity;
    target_yaw_rate_ = target_yaw_rate;

    return true;

}

Reference HoverOnCableManeuverServer::GetReference(const State &) {
    
    if (!validateAwareness(awareness_handler()->combined_drone_awareness())) {

        on_fail_callback_();

    }

    return Reference(
        {NAN,NAN,NAN},
        NAN,
        vector_t(NAN,NAN,target_z_velocity_),
        target_yaw_rate_,
        {NAN,NAN,NAN},
        NAN
    );

}

maneuver_type_t HoverOnCableManeuverServer::maneuver_type() const {

    return MANEUVER_TYPE_HOVER_ON_CABLE;

}

void HoverOnCableManeuverServer::startExecution(Maneuver &maneuver) {

    if (!has_on_fail_callback_) {

        throw std::runtime_error("No on fail callback registered for HoverOnCableManeuverServer");

    }

    hover_on_cable_maneuver_params_t params(maneuver.maneuver_params());

    target_z_velocity_ = params.target_z_velocity;
    target_yaw_rate_ = params.target_yaw_rate;

}

bool HoverOnCableManeuverServer::canCancel() {

    return false;

}

Reference HoverOnCableManeuverServer::computeReference(const State &state) {

    return GetReference(state);

}

bool HoverOnCableManeuverServer::hasSucceeded(Maneuver &) {

    return true;

}

bool HoverOnCableManeuverServer::hasFailed(Maneuver &) {

    return false;

}

void HoverOnCableManeuverServer::publishFeedback(Maneuver &) {

    // No feedback to publish

}

void HoverOnCableManeuverServer::publishResultAndFinalize(
    Maneuver &maneuver,
    maneuver_result_type_t maneuver_result_type
) {

    auto goal_handle = std::static_pointer_cast<rclcpp_action::ServerGoalHandle<HoverOnCable>>(maneuver.goal_handle());
    auto result = std::make_shared<HoverOnCable::Result>();

    switch (maneuver_result_type){

        case MANEUVER_RESULT_TYPE_SUCCEED:
            goal_handle->succeed(result);
            break;
        case MANEUVER_RESULT_TYPE_ABORT:
            goal_handle->abort(result);
            awareness_handler()->ClearTarget();
            break;
        case MANEUVER_RESULT_TYPE_CANCEL:
            goal_handle->canceled(result);
            awareness_handler()->ClearTarget();
            break;
    }

}

void HoverOnCableManeuverServer::registerReferenceCallbackOnSuccess(const Maneuver &) {
    
    registerCallback(
        std::bind(
            &HoverOnCableManeuverServer::GetReference,
            this,
            std::placeholders::_1
        )
    );

}

bool HoverOnCableManeuverServer::validateAwareness(combined_drone_awareness_t drone_awareness) const {

    if (!drone_awareness.offboard) {
        return false;
    }

    if (!drone_awareness.armed) {
        return false;
    }

    if (!drone_awareness.on_cable()) {
        return false;
    }

    return true;

}