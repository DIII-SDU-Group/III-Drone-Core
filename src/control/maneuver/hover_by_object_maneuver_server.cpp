/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/maneuver/hover_by_object_maneuver_server.hpp>

using namespace iii_drone::control::maneuver;
using namespace iii_drone::control;
using namespace iii_drone::adapters;
using namespace iii_drone::types;
using namespace iii_drone::math;

/*****************************************************************************/
// Impementation:
/*****************************************************************************/

HoverByObjectManeuverServer::HoverByObjectManeuverServer(
    rclcpp::Node * node,
    CombinedDroneAwarenessHandler::SharedPtr combined_drone_awareness_handler,
    const std::string & action_name,
    unsigned int wait_for_execute_poll_ms,
    unsigned int evaluate_done_poll_ms,
    bool use_nans,
    double max_euc_dist
) : ManeuverServer(
    node, 
    combined_drone_awareness_handler, 
    action_name, 
    wait_for_execute_poll_ms, 
    evaluate_done_poll_ms
),  has_target_(false),
    has_on_fail_callback_(false),
    use_nans_(use_nans), 
    max_euc_dist_(max_euc_dist) {

    createServer<HoverByObject>();

}

bool HoverByObjectManeuverServer::CanExecuteManeuver(
    const Maneuver & maneuver,
    const combined_drone_awareness_t & drone_awareness
) const {

    if (maneuver.maneuver_type() != MANEUVER_TYPE_HOVER_BY_OBJECT) {
        return false;
    }

    hover_by_object_maneuver_params_t params(maneuver.maneuver_params());

    if (params.target_adapter.target_type() != TARGET_TYPE_CABLE) {
        return false;
    }

    if (!drone_awareness.offboard) {
        return false;
    }

    if (!drone_awareness.armed) {
        return false;
    }

    if (!drone_awareness.in_flight()) {
        return false;
    }

    if (!drone_awareness.has_target()) {
        return false;
    }

    if (drone_awareness.target_adapter.target_type() != TARGET_TYPE_CABLE) {
        return false;
    }

    transform_matrix_t target_transform;
    
    try {

        target_transform = awareness_handler()->ComputeTargetTransform(target_adapter_);

    } catch (const std::runtime_error &e) {

        return false;

    }

    if (!validateTargetTransform(
        target_transform,
        drone_awareness.state
    )) {

        return false;

    }

    return true;

}

combined_drone_awareness_t HoverByObjectManeuverServer::ExpectedAwarenessAfterExecution(const Maneuver & ) {

    combined_drone_awareness_t awareness_after;
    awareness_after.armed = true;
    awareness_after.offboard = true;
    awareness_after.target_adapter = target_adapter_;
    awareness_after.target_position_known = true;
    awareness_after.drone_location = DRONE_LOCATION_IN_FLIGHT;

    return awareness_after;

}

void HoverByObjectManeuverServer::RegisterOnFailCallback(std::function<void()> on_fail_callback) {

    on_fail_callback_ = on_fail_callback;
    has_on_fail_callback_ = true;

}

bool HoverByObjectManeuverServer::Update(const iii_drone::adapters::TargetAdapter &target_adapter) {

    transform_matrix_t target_transform;
    
    try {

        target_transform = awareness_handler()->ComputeTargetTransform(target_adapter);

    } catch (const std::runtime_error &e) {

        has_target_ = false;

        return false;

    }

    if (!validateTargetTransform(
        target_transform,
        awareness_handler()->GetState()
    )) {

        has_target_ = false;

        return false;

    }

    has_target_ = true;

    target_adapter_ = target_adapter;

    return true;

}

iii_drone::control::Reference HoverByObjectManeuverServer::GetReference(const iii_drone::control::State & state) {

    if (!has_on_fail_callback_) {

        throw std::runtime_error("No on fail callback registered for HoverByObjectManeuverServer");

    }

    if (!has_target_) {

        throw std::runtime_error("No target set for HoverByObjectManeuverServer");

    }

    transform_matrix_t target_transform;
    
    try {

        target_transform = awareness_handler()->ComputeTargetTransform(target_adapter_);

    } catch (const std::runtime_error &e) {

        has_target_ = false;

    }

    if (!validateTargetTransform(
        target_transform,
        awareness_handler()->GetState()
    )) {

        has_target_ = false;

    }

    vector_t position;
    double yaw;
    vector_t velocity;
    double yaw_rate;
    vector_t acceleration;
    double yaw_acceleration;

    if (has_target_) {

        pose_t pose = poseFromTransformMatrix(target_transform);

        position = pose.position;
        yaw = quatToEul(pose.orientation)(2);
        
    } else {

        on_fail_callback_();

        position = state.position();
        yaw = state.yaw();

    }

    if (use_nans_) {

        velocity = vector_t::Constant(NAN);
        yaw_rate = NAN;
        acceleration = vector_t::Constant(NAN);
        yaw_acceleration = NAN;

    } else {

        velocity = vector_t::Zero();
        yaw_rate = 0;
        acceleration = vector_t::Zero();
        yaw_acceleration = 0;

    }

    return iii_drone::control::Reference(
        position,
        yaw,
        velocity,
        yaw_rate,
        acceleration,
        yaw_acceleration
    );

}

maneuver_type_t HoverByObjectManeuverServer::maneuver_type() const {

    return MANEUVER_TYPE_HOVER_BY_OBJECT;

}

void HoverByObjectManeuverServer::startExecution(Maneuver & maneuver) {

    if (!has_on_fail_callback_) {

        throw std::runtime_error("No on fail callback registered for HoverByObjectManeuverServer");

    }

    hover_by_object_maneuver_params_t params(maneuver.maneuver_params());

    target_adapter_ = params.target_adapter;
    has_target_ = true;

}

bool HoverByObjectManeuverServer::canCancel() {

    return true;

}

iii_drone::control::Reference HoverByObjectManeuverServer::computeReference(const iii_drone::control::State & state) {

    return GetReference(state);

}

bool HoverByObjectManeuverServer::hasSucceeded(Maneuver & ) {

    transform_matrix_t target_transform;
    
    try {

        target_transform = awareness_handler()->ComputeTargetTransform(target_adapter_);

    } catch (const std::runtime_error &e) {

        has_target_ = false;

    }

    return validateTargetTransform(
        target_transform,
        awareness_handler()->GetState()
    );

}

bool HoverByObjectManeuverServer::hasFailed(Maneuver & maneuver) {

    return !hasSucceeded(maneuver);

}

void HoverByObjectManeuverServer::publishFeedback(Maneuver & ) {

    // No feedback to publish

}

void HoverByObjectManeuverServer::publishResultAndFinalize(
    Maneuver & maneuver,
    maneuver_result_type_t maneuver_result_type
) {

    auto goal_handle = std::static_pointer_cast<rclcpp_action::ServerGoalHandle<HoverByObject>>(maneuver.goal_handle());
    auto result = std::make_shared<HoverByObject::Result>();

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

void HoverByObjectManeuverServer::registerReferenceCallbackOnSuccess(const Maneuver &) {

    registerCallback(
        std::bind(
            &HoverByObjectManeuverServer::GetReference,
            this,
            std::placeholders::_1
        )
    );

}

bool HoverByObjectManeuverServer::validateTargetTransform(
    const iii_drone::types::transform_matrix_t &target_transform, 
    const iii_drone::control::State &state
) const {

    vector_t target_position = poseFromTransformMatrix(target_transform).position;
    vector_t drone_position = state.position();

    if ((target_position - drone_position).norm() > max_euc_dist_) {

        return false;

    }

    return true;    

}