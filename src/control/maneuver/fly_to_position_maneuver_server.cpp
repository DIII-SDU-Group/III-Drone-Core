/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/maneuver/fly_to_position_maneuver_server.hpp>

using namespace iii_drone::control::maneuver;
using namespace iii_drone::control;
using namespace iii_drone::types;
using namespace iii_drone::adapters;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

FlyToPositionManeuverServer::FlyToPositionManeuverServer(
    rclcpp::Node * node,
    CombinedDroneAwarenessHandler::SharedPtr combined_drone_awareness_handler,
    const std::string & action_name,
    unsigned int wait_for_execute_poll_ms,
    unsigned int evaluate_done_poll_ms,
    FlyToPositionManeuverServerParameters::SharedPtr parameters,
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
        return false;
    }

    auto cda_handler = awareness_handler();

    std::shared_ptr<fly_to_position_maneuver_params_t> fly_to_position_maneuver_params = std::static_pointer_cast<fly_to_position_maneuver_params_t>(maneuver.maneuver_params());

    point_t target_position_in_world_frame = fly_to_position_maneuver_params->transform_target_position(
        parameters_->world_frame_id(),
        cda_handler->tf_buffer()
    );

    bool target_position_valid = target_position_in_world_frame[2] - cda_handler->ground_altitude_estimate() >= parameters_->minimum_target_altitude();

    return drone_awareness.in_flight() && 
        drone_awareness.offboard &&
        target_position_valid;

}

combined_drone_awareness_t FlyToPositionManeuverServer::ExpectedAwarenessAfterExecution(const Maneuver &) {

    combined_drone_awareness_t awareness_after;

    awareness_after.armed = true;
    awareness_after.offboard = true;
    awareness_after.target_adapter = TargetAdapter();
    awareness_after.target_position_known = false;
    awareness_after.drone_location = DRONE_LOCATION_IN_FLIGHT;

    return awareness_after;

}

maneuver_type_t FlyToPositionManeuverServer::maneuver_type() const {
    return MANEUVER_TYPE_FLY_TO_POSITION;
}

void FlyToPositionManeuverServer::startExecution(Maneuver & maneuver) {

    auto cda_handler = awareness_handler();

    std::shared_ptr<fly_to_position_maneuver_params_t> fly_to_position_maneuver_params = std::static_pointer_cast<fly_to_position_maneuver_params_t>(maneuver.maneuver_params());

    point_t target_position_in_world_frame = fly_to_position_maneuver_params->transform_target_position(
        parameters_->world_frame_id(),
        cda_handler->tf_buffer()
    );

    target_reference_ = iii_drone::control::Reference(
        target_position_in_world_frame,
        fly_to_position_maneuver_params->target_yaw
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

    Reference ref;

    if (parameters_->generate_trajectories_asynchronously_with_delay()) {

        if (first_iteration_) {

            if (trajectory_generator_client_->busy()) {

                std::string error_message = "FlyToPositionManeuverServer::computeReference(): Trajectory generator client is busy on first iteration, cannot start execution of maneuver.";

                RCLCPP_FATAL(node()->get_logger(), error_message.c_str());

                throw std::runtime_error(error_message);

            }

            trajectory_generator_client_->Reset(state);

            ref = trajectory_generator_client_->GetReferenceTrajectory().references()[0];

            trajectory_generator_client_->ComputeReferenceTrajectoryAsync(
                state,
                target_reference_,
                true,
                true,
                MPC_mode_t::positional
            );

            first_iteration_ = false;

        } else {

            while (!trajectory_generator_client_->done()) {

                rclcpp::sleep_for(std::chrono::milliseconds(parameters_->generate_trajectories_poll_period_ms()));

            }

            ref = trajectory_generator_client_->GetReferenceTrajectory().references()[1];

            trajectory_generator_client_->ComputeReferenceTrajectoryAsync(
                state,
                target_reference_,
                false,
                false,
                MPC_mode_t::positional
            );

        }

    } else {

        if (trajectory_generator_client_->busy()) {

            std::string error_message = "FlyToPositionManeuverServer::computeReference(): Trajectory generator client is busy on first iteration, cannot start execution of maneuver.";

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
            target_reference_,
            first_it,
            first_it,
            MPC_mode_t::positional,
            parameters_->generate_trajectories_poll_period_ms()
        );

        ref = trajectory_generator_client_->GetReferenceTrajectory().references()[0];

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

    double distance = (euc_pos - target_euc_pos).norm();

    return distance < parameters_->reached_position_euclidean_distance_threshold();

}

bool FlyToPositionManeuverServer::hasFailed(Maneuver &) {

    auto cda_handler = awareness_handler();

    return !cda_handler->in_flight() 
        || !cda_handler->offboard()
        || !cda_handler->armed();

}

void FlyToPositionManeuverServer::publishFeedback(Maneuver & maneuver) {

    auto feedback = std::make_shared<iii_drone_interfaces::action::FlyToPosition::Feedback>();

    ReferenceTrajectory reference_trajectory = trajectory_generator_client_->GetReferenceTrajectory();

    ReferenceTrajectoryAdapter reference_trajectory_adapter(reference_trajectory);

    feedback->planned_path = reference_trajectory_adapter.ToPathMsg(parameters_->world_frame_id());
    feedback->vehicle_pose = StateAdapter(awareness_handler()->GetState()).ToPoseStampedMsg(parameters_->world_frame_id());

    auto goal_handle = std::static_pointer_cast<GoalHandleFlyToPosition>(maneuver.goal_handle());

    goal_handle->publish_feedback(feedback);

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