/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/maneuver/hover_maneuver_server.hpp>

using namespace iii_drone::control::maneuver;
using namespace iii_drone::control;
using namespace iii_drone::types;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

HoverManeuverServer::HoverManeuverServer(
    rclcpp_lifecycle::LifecycleNode * node,
    CombinedDroneAwarenessHandler::SharedPtr combined_drone_awareness_handler,
    const std::string & action_name,
    unsigned int wait_for_execute_poll_ms,
    unsigned int evaluate_done_poll_ms,
    bool use_nans
) : ManeuverServer(
    node, 
    combined_drone_awareness_handler, 
    action_name, 
    wait_for_execute_poll_ms, 
    evaluate_done_poll_ms
),  use_nans_(use_nans) {

    createServer<Hover>();

}

bool HoverManeuverServer::CanExecuteManeuver(
    const Maneuver & maneuver,
    const combined_drone_awareness_t & drone_awareness
) const {

    if (maneuver.maneuver_type() != MANEUVER_TYPE_HOVER) {

        RCLCPP_DEBUG(node()->get_logger(), "HoverManeuverServer::CanExecuteManeuver(): Maneuver is not of type hover, cannot execute");

        return false;

    }

    if (!drone_awareness.offboard) {

        RCLCPP_DEBUG(node()->get_logger(), "HoverManeuverServer::CanExecuteManeuver(): Drone is not in offboard mode, cannot execute");

        return false;

    }

    if (!drone_awareness.armed) {

        RCLCPP_DEBUG(node()->get_logger(), "HoverManeuverServer::CanExecuteManeuver(): Drone is not armed, cannot execute");

        return false;

    }

    if (!drone_awareness.in_flight()) {

        RCLCPP_DEBUG(node()->get_logger(), "HoverManeuverServer::CanExecuteManeuver(): Drone is not in flight, cannot execute");

        return false;

    }

    //RCLCPP_DEBUG(node()->get_logger(), "HoverManeuverServer::CanExecuteManeuver(): Hover maneuver can be executed");

    return true;

}

combined_drone_awareness_t HoverManeuverServer::ExpectedAwarenessAfterExecution(const Maneuver & ) {

    State state = awareness_handler()->GetState();

    State expected_state(
        state.position(),
        vector_t::Zero(),
        0.0,
        vector_t::Zero()
    );

    combined_drone_awareness_t awareness_after;
    awareness_after.armed = true;
    awareness_after.offboard = true;
    awareness_after.target_adapter = iii_drone::adapters::TargetAdapter();
    awareness_after.target_position_known = false;
    awareness_after.drone_location = DRONE_LOCATION_IN_FLIGHT;
    awareness_after.state = expected_state;

    return awareness_after;

}

void HoverManeuverServer::Update(const State & hover_state) {

    point_t position = hover_state.position();
    double yaw = hover_state.yaw();

    vector_t velocity;
    vector_t acceleration;
    double yaw_rate;
    double yaw_acceleration;

    if(use_nans_) {
        velocity = vector_t(NAN, NAN, NAN);
        acceleration = vector_t(NAN, NAN, NAN);
        yaw_rate = NAN;
        yaw_acceleration = NAN;
    } else {
        velocity = vector_t(0.0, 0.0, 0.0);
        acceleration = vector_t(0.0, 0.0, 0.0);
        yaw_rate = 0.0;
        yaw_acceleration = 0.0;
    }

    hover_reference_ = iii_drone::control::Reference(
        position,
        yaw,
        velocity,
        yaw_rate,
        acceleration,
        yaw_acceleration
    );

}

void HoverManeuverServer::Update(const Reference & hover_reference) {

    point_t position = hover_reference.position();
    double yaw = hover_reference.yaw();

    vector_t velocity;
    vector_t acceleration;
    double yaw_rate;
    double yaw_acceleration;

    if(use_nans_) {
        velocity = vector_t(NAN, NAN, NAN);
        acceleration = vector_t(NAN, NAN, NAN);
        yaw_rate = NAN;
        yaw_acceleration = NAN;
    } else {
        velocity = vector_t(0.0, 0.0, 0.0);
        acceleration = vector_t(0.0, 0.0, 0.0);
        yaw_rate = 0.0;
        yaw_acceleration = 0.0;
    }

    hover_reference_ = iii_drone::control::Reference(
        position,
        yaw,
        velocity,
        yaw_rate,
        acceleration,
        yaw_acceleration
    );

}

iii_drone::control::Reference HoverManeuverServer::GetReference(const iii_drone::control::State & ) {

    //RCLCPP_DEBUG(node()->get_logger(), "HoverManeuverServer::GetReference(): Returning hover reference");

    hover_reference_ = hover_reference_->CopyWithNewStamp();

    return hover_reference_;

}

maneuver_type_t HoverManeuverServer::maneuver_type() const {
    return MANEUVER_TYPE_HOVER;
}

void HoverManeuverServer::startExecution(Maneuver & maneuver) {

    hover_maneuver_params_t params(maneuver.maneuver_params());

    Update(awareness_handler()->GetState());

    awareness_handler()->ClearTarget();

    hover_duration_s_ = params.duration_s;

    sustain_action_ = params.sustain_action;

    hover_start_time_ = rclcpp::Clock().now();

    RCLCPP_DEBUG(
        node()->get_logger(), 
        "HoverManeuverServer::startExecution(): Starting hover maneuver for %f seconds at time %f", 
        (float)hover_duration_s_,
        hover_start_time_->seconds()
    );

}

bool HoverManeuverServer::canCancel() {
    return true;
}

iii_drone::control::Reference HoverManeuverServer::computeReference(const iii_drone::control::State & state) {

    return GetReference(state);

}

bool HoverManeuverServer::hasSucceeded(Maneuver & ) {

    if (!sustain_action_) {

        RCLCPP_DEBUG(node()->get_logger(), "HoverManeuverServer::hasSucceeded(): Hover maneuver not sustained, succeeding immediately");

        return true;

    }

    rclcpp::Duration elapsed_time = rclcpp::Clock().now() - (rclcpp::Time)hover_start_time_;
    
    if (elapsed_time.seconds() >= (float)hover_duration_s_) {

        RCLCPP_DEBUG(node()->get_logger(), "HoverManeuverServer::hasSucceeded(): Hover maneuver has succeeded after %f seconds", (float)hover_duration_s_);

        return true;
    }

    return false;

}

bool HoverManeuverServer::hasFailed(Maneuver & ) {
    return false;
}

void HoverManeuverServer::publishResultAndFinalize(
    Maneuver & maneuver,
    maneuver_result_type_t maneuver_result_type
) {

    auto result = std::make_shared<Hover::Result>();
    auto goal_handle = std::static_pointer_cast<rclcpp_action::ServerGoalHandle<Hover>>(maneuver.goal_handle());

    switch (maneuver_result_type) {
        case MANEUVER_RESULT_TYPE_SUCCEED:
            goal_handle->succeed(result);
            break;
        case MANEUVER_RESULT_TYPE_ABORT:
            goal_handle->abort(result);
            break;
        case MANEUVER_RESULT_TYPE_CANCEL:
            goal_handle->canceled(result);
            break;
    }

}

void HoverManeuverServer::registerReferenceCallbackOnSuccess(const Maneuver &) {
    
    registerCallback(
        std::bind(
            &HoverManeuverServer::GetReference,
            this,
            std::placeholders::_1
        )
    );

}