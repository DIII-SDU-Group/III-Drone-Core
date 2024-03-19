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
    rclcpp::Node * node,
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
        return false;
    }

    auto cda_handler = awareness_handler();

    fly_to_object_maneuver_params_t params(maneuver.maneuver_params());

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

    transform_matrix_t target_transform;
    
    try {

        target_transform = awareness_handler()->ComputeTargetTransform(target_adapter_);

    } catch (const std::runtime_error &e) {

        return false;

    }

    point_t target_position_in_world_frame = target_transform.block<3, 1>(0, 3);

    bool target_position_valid = target_position_in_world_frame[2] - cda_handler->ground_altitude_estimate() >= parameters_->GetParameter("minimum_target_altitude").as_double();

    return target_position_valid;

}

combined_drone_awareness_t FlyToObjectManeuverServer::ExpectedAwarenessAfterExecution(const Maneuver & maneuver) {

    fly_to_object_maneuver_params_t params(maneuver.maneuver_params());

    TargetAdapter target_adapter = params.target_adapter;

    if (target_adapter.target_type() != TARGET_TYPE_CABLE) {
        throw std::runtime_error("FlyToObjectManeuverServer::ExpectedAwarenessAfterExecution(): Target type is not TARGET_TYPE_CABLE.");
    }

    combined_drone_awareness_t awareness_after;

    awareness_after.armed = true;
    awareness_after.offboard = true;
    awareness_after.target_adapter = target_adapter;
    awareness_after.target_position_known = true;
    awareness_after.drone_location = DRONE_LOCATION_IN_FLIGHT;

    return awareness_after;

}

maneuver_type_t FlyToObjectManeuverServer::maneuver_type() const {
    return MANEUVER_TYPE_FLY_TO_OBJECT;
}

void FlyToObjectManeuverServer::startExecution(Maneuver & maneuver) {

    auto cda_handler = awareness_handler();

    fly_to_object_maneuver_params_t params(maneuver.maneuver_params());

    target_adapter_ = params.target_adapter;

    cda_handler->SetTarget(target_adapter_);

    if (trajectory_generator_client_->busy()) {

        std::string error_message = "FlyToObjectManeuverServer::startExecution(): Trajectory generator client is busy, cannot start execution of maneuver.";

        RCLCPP_FATAL(node()->get_logger(), error_message.c_str());

        throw std::runtime_error(error_message);

    }

    first_iteration_ = true;
    has_failed_ = false;

}

bool FlyToObjectManeuverServer::canCancel() {
    return true;
}

Reference FlyToObjectManeuverServer::computeReference(const State & state) {

    Reference target_reference = getUpdatedTargetReference(state);

    Reference ref;

    if (parameters_->GetParameter("generate_trajectories_asynchronously_with_delay").as_bool()) {

        if (first_iteration_) {

            if (trajectory_generator_client_->busy()) {

                std::string error_message = "FlyToObjectManeuverServer::computeReference(): Trajectory generator client is busy on first iteration, cannot start execution of maneuver.";

                RCLCPP_FATAL(node()->get_logger(), error_message.c_str());

                throw std::runtime_error(error_message);

            }

            trajectory_generator_client_->Reset(state);

            ref = trajectory_generator_client_->GetReferenceTrajectory().references()[0];

            trajectory_generator_client_->ComputeReferenceTrajectoryAsync(
                state,
                target_reference,
                true,
                true,
                MPC_mode_t::positional
            );

            first_iteration_ = false;

        } else {

            while (!trajectory_generator_client_->done()) {

                rclcpp::sleep_for(std::chrono::milliseconds(parameters_->GetParameter("generate_trajectories_poll_period_ms").as_int()));

            }

            ref = trajectory_generator_client_->GetReferenceTrajectory().references()[1];

            trajectory_generator_client_->ComputeReferenceTrajectoryAsync(
                state,
                target_reference,
                true,
                false,
                MPC_mode_t::positional
            );

        }

    } else {

        if (trajectory_generator_client_->busy()) {

            std::string error_message = "FlyToObjectManeuverServer::computeReference(): Trajectory generator client is busy on first iteration, cannot start execution of maneuver.";

            RCLCPP_FATAL(node()->get_logger(), error_message.c_str());

            throw std::runtime_error(error_message);

        }

        bool first_it = first_iteration_;

        if (first_it) {

            trajectory_generator_client_->Reset(state);

            first_iteration_ = false;

        }

        trajectory_generator_client_->ComputeReferenceTrajectoryBlocking(
            state,
            target_reference,
            true,
            first_it,
            MPC_mode_t::positional,
            parameters_->GetParameter("generate_trajectories_poll_period_ms").as_int()
        );

        ref = trajectory_generator_client_->GetReferenceTrajectory().references()[0];

    }

    return ref;

}

bool FlyToObjectManeuverServer::hasSucceeded(Maneuver & maneuver) {

    auto cda_handler = awareness_handler();

    State state = cda_handler->GetState();

    Reference target_reference = getUpdatedTargetReference(state);

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

    return !cda_handler->in_flight() 
        || !cda_handler->offboard()
        || !cda_handler->armed()
        || cda_handler->target_adapter().target_type() != TARGET_TYPE_CABLE
        || !cda_handler->target_position_known()
        || has_failed_;

}

void FlyToObjectManeuverServer::publishFeedback(Maneuver & maneuver) {

    auto feedback = std::make_shared<iii_drone_interfaces::action::FlyToObject::Feedback>();

    ReferenceTrajectory reference_trajectory = trajectory_generator_client_->GetReferenceTrajectory();
    ReferenceTrajectoryAdapter reference_trajectory_adapter(reference_trajectory);

    State state = awareness_handler()->GetState();
    Reference target_reference = getUpdatedTargetReference(state);

    feedback->planned_path = reference_trajectory_adapter.ToPathMsg(parameters_->GetParameter("world_frame_id").as_string());
    feedback->vehicle_pose = StateAdapter(awareness_handler()->GetState()).ToPoseStampedMsg(parameters_->GetParameter("world_frame_id").as_string());
    feedback->distance_vehicle_to_target = (target_reference.position() - state.position()).norm();

    auto goal_handle = std::static_pointer_cast<GoalHandleFlyToObject>(maneuver.goal_handle());

    goal_handle->publish_feedback(feedback);

}

void FlyToObjectManeuverServer::publishResultAndFinalize(
    Maneuver & maneuver,
    maneuver_result_type_t maneuver_result_type
) {

    auto result = std::make_shared<iii_drone_interfaces::action::FlyToObject::Result>();
    auto goal_handle = std::static_pointer_cast<GoalHandleFlyToObject>(maneuver.goal_handle());

    switch (maneuver_result_type) {
        case MANEUVER_RESULT_TYPE_SUCCEED:
            result->success = true;
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

    hover_by_object_maneuver_server->Update(target_adapter_);

    registerCallback(
        std::bind(
            &HoverByObjectManeuverServer::GetReference,
            hover_by_object_maneuver_server,
            std::placeholders::_1
        )
    );

}

Reference FlyToObjectManeuverServer::getUpdatedTargetReference(const iii_drone::control::State & state) {

    auto cda_handler = awareness_handler();

    transform_matrix_t target_transform;

    try {

        target_transform = cda_handler->ComputeTargetTransform(target_adapter_);

    } catch (const std::runtime_error &e) {

        has_failed_ = true;

        return Reference(
            state.position(),
            state.yaw()
        );

    }

    point_t target_position_in_world_frame = target_transform.block<3, 1>(0, 3);
    quaternion_t target_orientation_in_world_frame = quaternionFromTransformMatrix(target_transform);
    double target_yaw = quatToEul(target_orientation_in_world_frame)[2];

    return Reference(
        target_position_in_world_frame,
        target_yaw
    );

}