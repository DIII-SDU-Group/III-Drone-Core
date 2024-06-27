/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/maneuver/fly_to_object_maneuver_server.hpp>

using namespace iii_drone::control::maneuver;
using namespace iii_drone::control;
using namespace iii_drone::types;
using namespace iii_drone::math;
using namespace iii_drone::adapters;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

FlyToObjectManeuverServer::FlyToObjectManeuverServer(
    rclcpp_lifecycle::LifecycleNode * node,
    CombinedDroneAwarenessHandler::SharedPtr combined_drone_awareness_handler,
    const std::string & action_name,
    unsigned int wait_for_execute_poll_ms,
    unsigned int evaluate_done_poll_ms,
    iii_drone::configuration::ParameterBundle::SharedPtr parameters,
    iii_drone::control::TrajectoryGeneratorClient::SharedPtr trajectory_generator_client
) : ManeuverServer(
    node,
    combined_drone_awareness_handler,
    action_name,
    wait_for_execute_poll_ms,
    evaluate_done_poll_ms
),  parameters_(parameters),
    trajectory_generator_client_(trajectory_generator_client) {

    createServer<FlyToObject>();

}

bool FlyToObjectManeuverServer::CanExecuteManeuver(
    const Maneuver & maneuver,
    const combined_drone_awareness_t & drone_awareness
) const {

    if (maneuver.maneuver_type() != MANEUVER_TYPE_FLY_TO_OBJECT) {
        RCLCPP_WARN(node()->get_logger(), "FlyToObjectManeuverServer::CanExecuteManeuver(): Maneuver type is not MANEUVER_TYPE_FLY_TO_OBJECT, returning false.");
        return false;
    }

    fly_to_object_maneuver_params_t params(maneuver.maneuver_params());

    if (!validateAwarenessAndParameters(
        drone_awareness,
        params
    )) {
        RCLCPP_WARN(node()->get_logger(), "FlyToObjectManeuverServer::CanExecuteManeuver(): Awareness and parameters are not valid, returning false.");
        return false;
    }

    return true;

}

combined_drone_awareness_t FlyToObjectManeuverServer::ExpectedAwarenessAfterExecution(const Maneuver & maneuver) {

    fly_to_object_maneuver_params_t params(maneuver.maneuver_params());

    TargetAdapter target_adapter = params.target_adapter;

    State target_state = awareness_handler()->ComputeTargetState(target_adapter);

    combined_drone_awareness_t awareness_after;

    awareness_after.armed = true;
    awareness_after.offboard = true;
    awareness_after.target_adapter = target_adapter;
    awareness_after.target_position_known = true;
    awareness_after.drone_location = DRONE_LOCATION_IN_FLIGHT;
    awareness_after.state = target_state;

    return awareness_after;

}

maneuver_type_t FlyToObjectManeuverServer::maneuver_type() const {
    return MANEUVER_TYPE_FLY_TO_OBJECT;
}

void FlyToObjectManeuverServer::startExecution(Maneuver & maneuver) {

    RCLCPP_INFO(node()->get_logger(), "FlyToObjectManeuverServer::startExecution(): Starting execution of maneuver.");

    auto cda_handler = awareness_handler();

    fly_to_object_maneuver_params_t params(maneuver.maneuver_params());

    target_adapter_ = params.target_adapter;

    RCLCPP_DEBUG(
        node()->get_logger(), 
        "FlyToObjectManeuverServer::startExecution(): Starting execution of maneuver with target type %d, target id %d, and target frame id %s.",
        target_adapter_->target_type(),
        target_adapter_->target_id(),
        target_adapter_->reference_frame_id().c_str()
    );

    if (trajectory_generator_client_->busy()) {

        std::string error_message = "FlyToObjectManeuverServer::startExecution(): Trajectory generator client is busy, cannot start execution of maneuver.";

        RCLCPP_FATAL(node()->get_logger(), error_message.c_str());

        throw std::runtime_error(error_message);

    }

    first_iteration_ = true;
    has_failed_ = false;

    cda_handler->SetTarget(target_adapter_);

}

bool FlyToObjectManeuverServer::canCancel() {
    return true;
}

Reference FlyToObjectManeuverServer::computeReference(const State & state) {

    Reference target_reference;
    
    try {

        target_reference = getUpdatedTargetReference(state);

    } catch (const std::runtime_error &e) {

        has_failed_ = true;
        return Reference(state);

    }

    bool reset = first_iteration_;
    bool set_reference = true;

    Reference ref = trajectory_generator_client_->ComputeReference(
        state,
        target_reference,
        set_reference,
        reset,
        trajectory_mode_t::positional,
        parameters_->GetParameter("use_mpc").as_bool()
    );

    if (first_iteration_) {

        first_iteration_ = false;

    }

    return ref;

}

bool FlyToObjectManeuverServer::hasSucceeded(Maneuver & maneuver) {

    auto cda_handler = awareness_handler();

    State state = cda_handler->GetState();

    Reference target_reference;
    
    try {
        target_reference = getUpdatedTargetReference(state);
    } catch (const std::runtime_error &e) {
        RCLCPP_ERROR(node()->get_logger(), "FlyToObjectManeuverServer::hasSucceeded(): Failed to get updated target reference, exception: %s", e.what());
        has_failed_ = true;
        return false;
    }

    if (hasFailed(maneuver)) {
        return false;
    }

    Eigen::Vector4d euc_pos = {
        state.position()[0], 
        state.position()[1], 
        state.position()[2], 
        state.yaw()
    };

    Eigen::Vector4d target_euc_pos = {
        target_reference.position()[0], 
        target_reference.position()[1], 
        target_reference.position()[2], 
        target_reference.yaw()
    };

    double distance = (euc_pos - target_euc_pos).norm();

    return distance < parameters_->GetParameter("reached_position_euclidean_distance_threshold").as_double();

}

bool FlyToObjectManeuverServer::hasFailed(Maneuver &) {

    auto cda_handler = awareness_handler();

    bool is_in_flight_or_on_current_cable = cda_handler->in_flight() || (
        cda_handler->on_cable() && 
        cda_handler->on_cable_id() == target_adapter_->target_id() &&
        target_adapter_->target_type() == TARGET_TYPE_CABLE
    );

    if (!is_in_flight_or_on_current_cable) {
        RCLCPP_WARN(node()->get_logger(), "FlyToObjectManeuverServer::hasFailed(): Drone is not in flight, and not on the target cable, returning true.");
        return true;
    }

    if (!cda_handler->offboard()) {
        RCLCPP_WARN(node()->get_logger(), "FlyToObjectManeuverServer::hasFailed(): Drone is not in offboard mode, returning true.");
        return true;
    }

    if (!cda_handler->armed()) {
        RCLCPP_WARN(node()->get_logger(), "FlyToObjectManeuverServer::hasFailed(): Drone is not armed, returning true.");
        return true;
    }

    if (cda_handler->target_adapter() != *target_adapter_) {
        RCLCPP_WARN(node()->get_logger(), "FlyToObjectManeuverServer::hasFailed(): Target adapter has changed, returning true.");
        return true;
    }

    if (!cda_handler->target_position_known()) {
        RCLCPP_WARN(node()->get_logger(), "FlyToObjectManeuverServer::hasFailed(): Target position is not known, returning true.");
        return true;
    }

    if (has_failed_) {
        RCLCPP_WARN(node()->get_logger(), "FlyToObjectManeuverServer::hasFailed(): The has_failed_ flag is set, returning true.");
        return true;
    }

    return false;

}

std::shared_ptr<void> FlyToObjectManeuverServer::getFeedback(Maneuver &) {

    auto feedback = std::make_shared<iii_drone_interfaces::action::FlyToObject::Feedback>();

    ReferenceTrajectory reference_trajectory;

    try {
        reference_trajectory = trajectory_generator_client_->GetReferenceTrajectory();
    } catch (const std::runtime_error &e) {
        RCLCPP_WARN(node()->get_logger(), "FlyToObjectManeuverServer::getFeedback(): Failed to get reference trajectory, exception: %s", e.what());
        return std::static_pointer_cast<void>(feedback);
    }

    ReferenceTrajectoryAdapter reference_trajectory_adapter(reference_trajectory);

    State state = awareness_handler()->GetState();
    Reference target_reference;
    
    try {
        target_reference = getUpdatedTargetReference(state);
    } catch (const std::runtime_error &e) {
        RCLCPP_ERROR(node()->get_logger(), "FlyToObjectManeuverServer::getFeedback(): Failed to get updated target reference, exception: %s", e.what());
        has_failed_ = true;
        return std::static_pointer_cast<void>(feedback);
    }

    feedback->planned_path = reference_trajectory_adapter.ToPathMsg(parameters_->GetParameter("world_frame_id").as_string());
    feedback->vehicle_pose = StateAdapter(awareness_handler()->GetState()).ToPoseStampedMsg(parameters_->GetParameter("world_frame_id").as_string());
    feedback->distance_vehicle_to_target = (target_reference.position() - state.position()).norm();

    return std::static_pointer_cast<void>(feedback);

}

void FlyToObjectManeuverServer::publishResultAndFinalize(
    Maneuver & maneuver,
    maneuver_result_type_t maneuver_result_type
) {

    auto result = std::make_shared<iii_drone_interfaces::action::FlyToObject::Result>();
    auto goal_handle = std::static_pointer_cast<GoalHandleFlyToObject>(maneuver.goal_handle());

    Reference target_reference;

    try {
        target_reference = getUpdatedTargetReference(awareness_handler()->GetState());
    } catch (const std::runtime_error &e) {
        RCLCPP_ERROR(node()->get_logger(), "FlyToObjectManeuverServer::publishResultAndFinalize(): Failed to get updated target reference, exception: %s", e.what());
        maneuver_result_type = MANEUVER_RESULT_TYPE_ABORT;
    }


    switch (maneuver_result_type) {
        case MANEUVER_RESULT_TYPE_SUCCEED:
            result->success = true;
            result->target_reference = ReferenceAdapter(target_reference).ToMsg();
            goal_handle->succeed(result);
            break;
        case MANEUVER_RESULT_TYPE_ABORT:
            result->success = false;
            goal_handle->abort(result);
            awareness_handler()->ClearTarget();
            break;
        case MANEUVER_RESULT_TYPE_CANCEL:
            result->success = false;
            goal_handle->canceled(result);
            awareness_handler()->ClearTarget();
            break;
    }

}

void FlyToObjectManeuverServer::registerReferenceCallbackOnSuccess(const Maneuver &) {

    auto registered_hover_by_object_maneuver = registered_maneuvers().find(MANEUVER_TYPE_HOVER_BY_OBJECT);

    std::shared_ptr<HoverByObjectManeuverServer> hover_by_object_maneuver_server = std::static_pointer_cast<HoverByObjectManeuverServer>(registered_hover_by_object_maneuver->second);

    if (hover_by_object_maneuver_server->Update(target_adapter_)) {

        registerCallback(
            std::bind(
                &HoverByObjectManeuverServer::GetReference,
                hover_by_object_maneuver_server,
                std::placeholders::_1
            )
        );

        return;

    }

    RCLCPP_ERROR(node()->get_logger(), "FlyToObjectManeuverServer::registerReferenceCallbackOnSuccess(): Failed to register hover by object reference callback on success, registering hover maneuver instead.");

    auto registered_hover_maneuver = registered_maneuvers().find(MANEUVER_TYPE_HOVER);

    std::shared_ptr<HoverManeuverServer> hover_maneuver_server = std::static_pointer_cast<HoverManeuverServer>(registered_hover_maneuver->second);

    hover_maneuver_server->Update(awareness_handler()->GetState());

    registerCallback(
        std::bind(
            &HoverManeuverServer::GetReference,
            hover_maneuver_server,
            std::placeholders::_1
        )
    );

}

Reference FlyToObjectManeuverServer::getUpdatedTargetReference(const iii_drone::control::State &) {

    auto cda_handler = awareness_handler();

    return Reference(cda_handler->ComputeTargetState(target_adapter_));

}

bool FlyToObjectManeuverServer::validateAwarenessAndParameters(
    const combined_drone_awareness_t & drone_awareness,
    const fly_to_object_maneuver_params_t & params
) const {

    RCLCPP_DEBUG(node()->get_logger(), "FlyToObjectManeuverServer::validateAwarenessAndParameters(): Validating awareness and parameters.");

    auto cda_handler = awareness_handler();

    if (params.target_adapter.target_type() != TARGET_TYPE_CABLE) {
        RCLCPP_WARN(node()->get_logger(), "FlyToObjectManeuverServer::validateAwarenessAndParameters(): Target type is not TARGET_TYPE_CABLE, returning false.");
        return false;
    }

    if (!drone_awareness.offboard) {
        RCLCPP_WARN(node()->get_logger(), "FlyToObjectManeuverServer::validateAwarenessAndParameters(): Drone is not in offboard mode, returning false.");
        return false;
    }

    if (!drone_awareness.armed) {
        RCLCPP_WARN(node()->get_logger(), "FlyToObjectManeuverServer::validateAwarenessAndParameters(): Drone is not armed, returning false.");
        return false;
    }

    bool is_in_flight_or_on_current_cable = drone_awareness.in_flight() || (
        drone_awareness.on_cable() && 
        drone_awareness.on_cable_id == params.target_adapter.target_id() &&
        params.target_adapter.target_type() == TARGET_TYPE_CABLE
    );

    if (!is_in_flight_or_on_current_cable) {
        RCLCPP_WARN(
            node()->get_logger(), 
            "FlyToObjectManeuverServer::validateAwarenessAndParameters(): Drone is not in flight, and not on the target cable, returning false."
        );
        return false;
    }

    transform_matrix_t target_transform;
    
    try {

        target_transform = awareness_handler()->ComputeTargetTransform(params.target_adapter);

    } catch (const std::runtime_error &e) {

        RCLCPP_WARN(node()->get_logger(), "FlyToObjectManeuverServer::validateAwarenessAndParameters(): Failed to compute target transform, returning false. Exception: %s", e.what());

        return false;

    }

    point_t target_position_in_world_frame = target_transform.block<3, 1>(0, 3);

    bool target_position_valid = target_position_in_world_frame[2] - cda_handler->ground_altitude_estimate() >= parameters_->GetParameter("minimum_target_altitude").as_double();

    if (!target_position_valid) {
        RCLCPP_WARN(node()->get_logger(), "FlyToObjectManeuverServer::validateAwarenessAndParameters(): Target position is not valid, returning false.");
        return false;
    }

    return true;

}