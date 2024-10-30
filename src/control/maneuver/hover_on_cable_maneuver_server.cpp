/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/maneuver/hover_on_cable_maneuver_server.hpp>

using namespace iii_drone::control::maneuver;
using namespace iii_drone::control;
using namespace iii_drone::types;
using namespace iii_drone::adapters;
using namespace iii_drone::configuration;

/*****************************************************************************/
// Implementation:
/*****************************************************************************/

HoverOnCableManeuverServer::HoverOnCableManeuverServer(
    rclcpp_lifecycle::LifecycleNode * node,
    CombinedDroneAwarenessHandler::SharedPtr combined_drone_awareness_handler,
    const std::string & action_name,
    unsigned int wait_for_execute_poll_ms,
    unsigned int evaluate_done_poll_ms,
    ParameterBundle::SharedPtr parameters
) : ManeuverServer(
        node,
        combined_drone_awareness_handler,
        action_name,
        wait_for_execute_poll_ms,
        evaluate_done_poll_ms
    ),
    parameters_(parameters),
    has_on_fail_callback_(false) {

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
        RCLCPP_WARN(
            node()->get_logger(),
            "HoverOnCableManeuverServer::CanExecuteManeuver(): target_z_velocity must be non-negative."
        );
        return false;
    }

    if (drone_awareness.target_adapter.target_type() != TARGET_TYPE_CABLE) {
        RCLCPP_WARN(
            node()->get_logger(),
            "HoverOnCableManeuverServer::CanExecuteManeuver(): target_type must be TARGET_TYPE_CABLE."
        );
        return false;
    }

    if (drone_awareness.target_adapter.reference_frame_id() != parameters_->GetParameter("gripper_frame_id").as_string()) {
        RCLCPP_WARN(
            node()->get_logger(),
            "HoverOnCableManeuverServer::CanExecuteManeuver(): reference_frame_id must be gripper_frame_id."
        );
        return false;
    }

    if (!drone_awareness.target_adapter.target_transform().isApprox(transform_matrix_t::Identity())) {
        RCLCPP_WARN(
            node()->get_logger(),
            "HoverOnCableManeuverServer::CanExecuteManeuver(): target_transform gripper to cable must be identity."
        );
        return false;
    }

    if (drone_awareness.target_adapter.target_id() != params.target_cable_id) {
        RCLCPP_WARN(
            node()->get_logger(),
            "HoverOnCableManeuverServer::CanExecuteManeuver(): target_cable_id must be target id of target adapter."
        );
        return false;
    }

    if (!validateAwareness(drone_awareness)) {
        RCLCPP_WARN(
            node()->get_logger(),
            "HoverOnCableManeuverServer::CanExecuteManeuver(): validateAwareness failed."
        );
        return false;
    }

    return true;

}

combined_drone_awareness_t HoverOnCableManeuverServer::ExpectedAwarenessAfterExecution(const Maneuver & maneuver) {

    hover_on_cable_maneuver_params_t params(maneuver.maneuver_params());

    TargetAdapter target_adapter = iii_drone::adapters::TargetAdapter(
        iii_drone::adapters::TARGET_TYPE_CABLE,
        params.target_cable_id,
        parameters_->GetParameter("gripper_frame_id").as_string(),
        iii_drone::types::transform_matrix_t::Identity()
    );

    State target_state = awareness_handler()->ComputeTargetState(target_adapter);

    combined_drone_awareness_t awareness_after;

    awareness_after.armed = true;
    awareness_after.offboard = true;
    awareness_after.target_adapter = target_adapter;
    awareness_after.target_position_known = true;
    awareness_after.drone_location = DRONE_LOCATION_ON_CABLE;
    awareness_after.state = target_state;

    return awareness_after;

}

void HoverOnCableManeuverServer::RegisterOnFailCallback(std::function<void()> on_fail_callback) {

    on_fail_callback_ = on_fail_callback;
    has_on_fail_callback_ = true;

}

bool HoverOnCableManeuverServer::Update(
    int target_cable_id,
    double target_z_velocity,
    double target_yaw_rate
) {

    if (target_z_velocity < 0.0) {
        return false;
    }

    int prev_target_cable_id = target_cable_id_;
    target_cable_id_ = target_cable_id;

    if(!validateAwareness(awareness_handler()->combined_drone_awareness())) {
        target_cable_id_ = prev_target_cable_id;
        return false;
    }

    TargetAdapter target_adapter = iii_drone::adapters::TargetAdapter(
        iii_drone::adapters::TARGET_TYPE_CABLE,
        target_cable_id,
        parameters_->GetParameter("gripper_frame_id").as_string(),
        iii_drone::types::transform_matrix_t::Identity()
    );

    awareness_handler()->SetTarget(target_adapter);
    
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

    if (!Update(
        params.target_cable_id,
        params.target_z_velocity,
        params.target_yaw_rate
    )) {

        std::string msg = "HoverOnCableManeuverServer::startExecution(): Failed to update target.";

        RCLCPP_FATAL(node()->get_logger(), msg.c_str());

        throw std::runtime_error(msg);

    }

    hover_duration_s_ = params.duration_s;

    sustain_action_ = params.sustain_action;

    hover_start_time_ = rclcpp::Clock().now();

}

bool HoverOnCableManeuverServer::canCancel() {

    return true;

}

Reference HoverOnCableManeuverServer::computeReference(const State &state) {

    return GetReference(state);

}

bool HoverOnCableManeuverServer::hasSucceeded(Maneuver &) {

    if (!sustain_action_) {
        return true;
    }

    if (rclcpp::Clock().now() - hover_start_time_ >= rclcpp::Duration::from_seconds(hover_duration_s_)) {
        return true;
    }

    return false;

}

bool HoverOnCableManeuverServer::hasFailed(Maneuver &) {

    return false;

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
        RCLCPP_WARN(node()->get_logger(), "HoverByObjectManeuverServer::validateAwareness(): Drone is not on cable.");
        return false;
    }

    if (!drone_awareness.has_target()) {
        RCLCPP_WARN(node()->get_logger(), "HoverByObjectManeuverServer::validateAwareness(): Drone does not have target.");
        return false;
    }

    if (drone_awareness.target_adapter.target_type() != TARGET_TYPE_CABLE) {
        RCLCPP_WARN(node()->get_logger(), "HoverByObjectManeuverServer::validateAwareness(): Target type is not cable.");
        return false;
    }

    if (drone_awareness.target_adapter.target_id() != target_cable_id_) {
        RCLCPP_WARN(node()->get_logger(), "HoverByObjectManeuverServer::validateAwareness(): Target id is not stored target id.");
        return false;
    }

    if (drone_awareness.target_adapter.reference_frame_id() != parameters_->GetParameter("gripper_frame_id").as_string()) {
        RCLCPP_WARN(node()->get_logger(), "HoverByObjectManeuverServer::validateAwareness(): Reference frame id is not gripper.");
        return false;
    }

    if (!drone_awareness.target_position_known) {
        RCLCPP_WARN(node()->get_logger(), "HoverByObjectManeuverServer::validateAwareness(): Target position is not known.");
        return false;
    }

    return true;

}