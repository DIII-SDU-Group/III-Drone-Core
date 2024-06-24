/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/maneuver/fly_to_position_maneuver_server.hpp>

using namespace iii_drone::control::maneuver;
using namespace iii_drone::control;
using namespace iii_drone::types;
using namespace iii_drone::math;
using namespace iii_drone::adapters;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

FlyToPositionManeuverServer::FlyToPositionManeuverServer(
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

    createServer<iii_drone_interfaces::action::FlyToPosition>();

}

bool FlyToPositionManeuverServer::CanExecuteManeuver(
    const Maneuver & maneuver,
    const combined_drone_awareness_t & drone_awareness
) const {

    if (maneuver.maneuver_type() != MANEUVER_TYPE_FLY_TO_POSITION) {
        RCLCPP_WARN(
            node()->get_logger(),
            "FlyToPositionManeuverServer::CanExecuteManeuver(): Maneuver type is not FlyToPosition"
        );
        return false;
    }

    if (!drone_awareness.in_flight()) {
        RCLCPP_WARN(
            node()->get_logger(),
            "FlyToPositionManeuverServer::CanExecuteManeuver(): Drone is not in flight"
        );
        return false;
    }

    if (!drone_awareness.offboard) {
        RCLCPP_WARN(
            node()->get_logger(),
            "FlyToPositionManeuverServer::CanExecuteManeuver(): Drone is not in offboard mode"
        );
        return false;
    }

    fly_to_position_maneuver_params_t fly_to_position_maneuver_params(maneuver.maneuver_params());

    bool maneuver_params_valid = validateManeuverParameters(fly_to_position_maneuver_params);

    if (!maneuver_params_valid) {
        RCLCPP_WARN(
            node()->get_logger(),
            "FlyToPositionManeuverServer::CanExecuteManeuver(): Maneuver parameters are invalid"
        );
        return false;
    }

    return true;

}

combined_drone_awareness_t FlyToPositionManeuverServer::ExpectedAwarenessAfterExecution(const Maneuver &maneuver) {

    combined_drone_awareness_t awareness_after;

    awareness_after.armed = true;
    awareness_after.offboard = true;
    awareness_after.target_adapter = TargetAdapter();
    awareness_after.target_position_known = false;
    awareness_after.drone_location = DRONE_LOCATION_IN_FLIGHT;

    fly_to_position_maneuver_params_t maneuver_params(maneuver.maneuver_params());

    auto cda_handler = awareness_handler();

    point_t target_position_in_world_frame = maneuver_params.transform_target_position(
        parameters_->GetParameter("world_frame_id").as_string(),
        cda_handler->tf_buffer()
    );

    quaternion_t target_orientation = eulToQuat(
        euler_angles_t(
            0.0, 
            0.0, 
            maneuver_params.transform_target_yaw(
                parameters_->GetParameter("world_frame_id").as_string(),
                cda_handler->tf_buffer()
            )
        )
    );

    awareness_after.state = State(
        target_position_in_world_frame,
        vector_t(0,0,0),
        target_orientation,
        vector_t(0,0,0)
    );

    return awareness_after;

}

maneuver_type_t FlyToPositionManeuverServer::maneuver_type() const {
    return MANEUVER_TYPE_FLY_TO_POSITION;
}

void FlyToPositionManeuverServer::startExecution(Maneuver & maneuver) {

    RCLCPP_INFO(
        node()->get_logger(),
        "FlyToPositionManeuverServer::startExecution(): Starting execution of maneuver."
    );

    auto cda_handler = awareness_handler();

    fly_to_position_maneuver_params_t fly_to_position_maneuver_params(maneuver.maneuver_params());

    State state = cda_handler->GetState();

    // RCLCPP_DEBUG(node()->get_logger(), "FlyToPositionManeuverServer::startExecution(): State: %f, %f, %f, %f", state.position()[0], state.position()[1], state.position()[2], state.yaw());

    point_t target_point = fly_to_position_maneuver_params.target_position;

    // RCLCPP_DEBUG(node()->get_logger(), "FlyToPositionManeuverServer::startExecution(): Target point: %f, %f, %f", target_point[0], target_point[1], target_point[2]);

    point_t target_position_in_world_frame = fly_to_position_maneuver_params.transform_target_position(
        parameters_->GetParameter("world_frame_id").as_string(),
        cda_handler->tf_buffer()
    );

    // RCLCPP_DEBUG(node()->get_logger(), "FlyToPositionManeuverServer::startExecution(): Target position in world frame: %f, %f, %f", target_position_in_world_frame[0], target_position_in_world_frame[1], target_position_in_world_frame[2]);

    double yaw = fly_to_position_maneuver_params.target_yaw;

    // RCLCPP_DEBUG(node()->get_logger(), "FlyToPositionManeuverServer::startExecution(): Yaw: %f", yaw);

    double target_yaw_in_world_frame = fly_to_position_maneuver_params.transform_target_yaw(
        parameters_->GetParameter("world_frame_id").as_string(),
        cda_handler->tf_buffer()
    );

    // RCLCPP_DEBUG(node()->get_logger(), "FlyToPositionManeuverServer::startExecution(): Target yaw in world frame: %f", target_yaw_in_world_frame);

    target_reference_ = iii_drone::control::Reference(
        target_position_in_world_frame,
        target_yaw_in_world_frame
    );

    if (trajectory_generator_client_->busy()) {

        std::string error_message = "FlyToPositionManeuverServer::startExecution(): Trajectory generator client is busy, cannot start execution of maneuver.";

        RCLCPP_FATAL(node()->get_logger(), error_message.c_str());

        throw std::runtime_error(error_message);

    }

    first_iteration_ = true;

    cda_handler->ClearTarget();

}

bool FlyToPositionManeuverServer::canCancel() {
    return true;
}

Reference FlyToPositionManeuverServer::computeReference(const State & state) {

    bool reset, set_reference;
    reset = set_reference = first_iteration_;

    Reference ref = trajectory_generator_client_->ComputeReference(
        state,
        target_reference_,
        set_reference,
        reset,
        MPC_mode_t::positional
    );

    if (first_iteration_) {

        first_iteration_ = false;

    }

    return ref;

}

bool FlyToPositionManeuverServer::hasSucceeded(Maneuver &) {

    auto cda_handler = awareness_handler();

    State state = cda_handler->GetState();

    Eigen::Vector4d euc_pos = {
        state.position()[0], 
        state.position()[1], 
        state.position()[2], 
        state.yaw()
    };

    Eigen::Vector4d target_euc_pos = {
        target_reference_->position()[0], 
        target_reference_->position()[1], 
        target_reference_->position()[2], 
        target_reference_->yaw()
    };

    double distance = (state.position() - target_reference_->position()).norm();
    // double distance = (euc_pos - target_euc_pos).norm();

    // RCLCPP_DEBUG(node()->get_logger(), "FlyToPositionManeuverServer::hasSucceeded(): Euc pos: %f, %f, %f, %f, target euc pos: %f, %f, %f, %f, distance: %f", euc_pos[0], euc_pos[1], euc_pos[2], euc_pos[3], target_euc_pos[0], target_euc_pos[1], target_euc_pos[2], target_euc_pos[3], distance);

    return distance < parameters_->GetParameter("reached_position_euclidean_distance_threshold").as_double();

}

bool FlyToPositionManeuverServer::hasFailed(Maneuver &) {

    auto cda_handler = awareness_handler();

    if (!cda_handler->in_flight()) {
        RCLCPP_WARN(
            node()->get_logger(),
            "FlyToPositionManeuverServer::hasFailed(): Drone is not in flight"
        );
        return true;
    }

    if (!cda_handler->offboard()) {
        RCLCPP_WARN(
            node()->get_logger(),
            "FlyToPositionManeuverServer::hasFailed(): Drone is not in offboard mode"
        );
        return true;
    }

    if (!cda_handler->armed()) {
        RCLCPP_WARN(
            node()->get_logger(),
            "FlyToPositionManeuverServer::hasFailed(): Drone is not armed"
        );
        return true;
    }

    return false;

}

std::shared_ptr<void> FlyToPositionManeuverServer::getFeedback(Maneuver &) {

    auto feedback = std::make_shared<iii_drone_interfaces::action::FlyToPosition::Feedback>();

    ReferenceTrajectory reference_trajectory = trajectory_generator_client_->GetReferenceTrajectory();

    ReferenceTrajectoryAdapter reference_trajectory_adapter(reference_trajectory);

    feedback->planned_path = reference_trajectory_adapter.ToPathMsg(parameters_->GetParameter("world_frame_id").as_string());
    feedback->vehicle_pose = StateAdapter(awareness_handler()->GetState()).ToPoseStampedMsg(parameters_->GetParameter("world_frame_id").as_string());

    return std::static_pointer_cast<void>(feedback);

}

void FlyToPositionManeuverServer::publishResultAndFinalize(
    Maneuver & maneuver,
    maneuver_result_type_t maneuver_result_type
) {

    auto result = std::make_shared<iii_drone_interfaces::action::FlyToPosition::Result>();
    auto goal_handle = std::static_pointer_cast<GoalHandleFlyToPosition>(maneuver.goal_handle());

    switch (maneuver_result_type) {
        case MANEUVER_RESULT_TYPE_SUCCEED:
            result->success = true;
            result->target_reference = ReferenceAdapter(target_reference_).ToMsg();
            goal_handle->succeed(result);
            break;
        case MANEUVER_RESULT_TYPE_ABORT:
            result->success = false;
            goal_handle->abort(result);
            break;
        case MANEUVER_RESULT_TYPE_CANCEL:
            result->success = false;
            goal_handle->canceled(result);
            break;
    }

}

void FlyToPositionManeuverServer::registerReferenceCallbackOnSuccess(const Maneuver &) {

    auto registered_hover_maneuver = registered_maneuvers().find(MANEUVER_TYPE_HOVER);

    std::shared_ptr<HoverManeuverServer> hover_maneuver_server = std::static_pointer_cast<HoverManeuverServer>(registered_hover_maneuver->second);

    hover_maneuver_server->Update(target_reference_);

    registerCallback(
        std::bind(
            &HoverManeuverServer::GetReference,
            hover_maneuver_server,
            std::placeholders::_1
        )
    );

}

bool FlyToPositionManeuverServer::validateManeuverParameters(const fly_to_position_maneuver_params_t & maneuver_params) const {

    auto cda_handler = awareness_handler();

    point_t target_position_in_world_frame = maneuver_params.transform_target_position(
        parameters_->GetParameter("world_frame_id").as_string(),
        cda_handler->tf_buffer()
    );

    bool target_position_valid = target_position_in_world_frame[2] - cda_handler->ground_altitude_estimate() >= parameters_->GetParameter("minimum_target_altitude").as_double();

    return target_position_valid;

}