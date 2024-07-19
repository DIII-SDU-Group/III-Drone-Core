/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/maneuver/cable_takeoff_maneuver_server.hpp>

using namespace iii_drone::control::maneuver;
using namespace iii_drone::control;
using namespace iii_drone::types;
using namespace iii_drone::math;
using namespace iii_drone::utils;
using namespace iii_drone::adapters;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

CableTakeoffManeuverServer::CableTakeoffManeuverServer(
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

    createServer<iii_drone_interfaces::action::CableTakeoff>();

}

bool CableTakeoffManeuverServer::CanExecuteManeuver(
    const Maneuver & maneuver,
    const combined_drone_awareness_t & drone_awareness
) const {

    cable_takeoff_maneuver_params_t cable_takeoff_maneuver_params(maneuver.maneuver_params());

    if (maneuver.maneuver_type() != MANEUVER_TYPE_CABLE_TAKEOFF) {
        return false;
    }

    if (!drone_awareness.armed) {
        return false;
    }

    if (!drone_awareness.offboard) {
        return false;
    }

    if (!drone_awareness.on_cable()) {
        return false;
    }

    if (!drone_awareness.has_target()) {
        return false;
    }

    if (drone_awareness.target_adapter.target_type() != TARGET_TYPE_CABLE) {
        return false;
    }

    if (drone_awareness.target_adapter.target_id() != cable_takeoff_maneuver_params.target_cable_id) {
        return false;
    }

    if(cable_takeoff_maneuver_params.target_cable_distance < parameters_->GetParameter("cable_takeoff_min_target_cable_distance").as_double()) {
        return false;
    }

    if (cable_takeoff_maneuver_params.target_cable_distance > parameters_->GetParameter("cable_takeoff_max_target_cable_distance").as_double()) {
        return false;
    }

    return true;

}

combined_drone_awareness_t CableTakeoffManeuverServer::ExpectedAwarenessAfterExecution(const Maneuver & maneuver) {

    cable_takeoff_maneuver_params_t cable_takeoff_maneuver_params(maneuver.maneuver_params());

    transform_matrix_t target_transform = cable_takeoff_maneuver_params.get_target_transform();

    TargetAdapter target_adapter = TargetAdapter(
        TARGET_TYPE_CABLE,
        cable_takeoff_maneuver_params.target_cable_id,
        parameters_->GetParameter("drone_frame_id").as_string(),
        target_transform
    );

    State target_state;
    
    try {
    
        target_state = awareness_handler()->ComputeTargetState(target_adapter);

    } catch (const std::runtime_error &e) {

        State current_state = awareness_handler()->GetState();

        target_state = State(
            current_state.position() - vector_t(0, 0, cable_takeoff_maneuver_params.target_cable_distance),
            vector_t::Zero(),
            current_state.yaw(),
            vector_t::Zero()
        );

    }

    combined_drone_awareness_t awareness_after;

    awareness_after.armed = true;
    awareness_after.offboard = true;
    awareness_after.target_position_known = true;
    awareness_after.drone_location = DRONE_LOCATION_ON_CABLE;
    awareness_after.target_adapter = target_adapter;
    awareness_after.state = target_state;

    return awareness_after;

}

maneuver_type_t CableTakeoffManeuverServer::maneuver_type() const {
    return MANEUVER_TYPE_CABLE_TAKEOFF;
}

void CableTakeoffManeuverServer::startExecution(Maneuver & maneuver) {

    auto cda_handler = awareness_handler();

    cable_takeoff_maneuver_params_t cable_takeoff_maneuver_params(maneuver.maneuver_params());

    transform_matrix_t target_transform = cable_takeoff_maneuver_params.get_target_transform();

    target_adapter_ = TargetAdapter(
        TARGET_TYPE_CABLE,
        cable_takeoff_maneuver_params.target_cable_id,
        parameters_->GetParameter("drone_frame_id").as_string(),
        target_transform
    );

    start_state_ = cda_handler->GetState();

    if (trajectory_generator_client_->busy()) {

        std::string error_message = "CableTakeoffManeuverServer::startExecution(): Trajectory generator client is busy, cannot start execution of maneuver.";

        RCLCPP_FATAL(node()->get_logger(), error_message.c_str());

        throw std::runtime_error(error_message);

    }

    first_iteration_ = true;
    has_failed_ = false;

    cda_handler->SetTarget(target_adapter_);

}

bool CableTakeoffManeuverServer::canCancel() {
    
    return false;

}

Reference CableTakeoffManeuverServer::computeReference(const State & state) {

    Reference target_reference = getUpdatedTargetReference(
        state,
        true
    );

    bool reset = first_iteration_;
    bool set_reference = true;

    first_iteration_ = false;

    Reference ref;
    
    try {

        ref = trajectory_generator_client_->ComputeReference(
            state,
            target_reference,
            set_reference,
            reset,
            trajectory_mode_t::cable_takeoff,
            parameters_->GetParameter("use_mpc").as_bool()
        );

    } catch (const std::runtime_error &e) {

        RCLCPP_ERROR(node()->get_logger(), "CableTakeoffManeuverServer::computeReference(): %s", e.what());

        has_failed_ = true;

        ref = Reference(
            state,
            true,
            true
        );

    }

    return ref;

}

bool CableTakeoffManeuverServer::hasSucceeded(Maneuver &) {

    auto cda_handler = awareness_handler();

    State state = cda_handler->GetState();

    Reference target_reference = getUpdatedTargetReference(state);

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

bool CableTakeoffManeuverServer::hasFailed(Maneuver &) {

    auto cda_handler = awareness_handler();

    Reference target_reference = getUpdatedTargetReference(cda_handler->GetState());

    return !(cda_handler->in_flight() || cda_handler->on_cable())
        || !cda_handler->offboard()
        || !cda_handler->armed()
        || cda_handler->target_adapter() != target_adapter_;

}

std::shared_ptr<void> CableTakeoffManeuverServer::getFeedback(Maneuver &) {

    ReferenceTrajectory reference_trajectory = trajectory_generator_client_->GetReferenceTrajectory();

    ReferenceTrajectoryAdapter reference_trajectory_adapter(reference_trajectory);

    State state = awareness_handler()->GetState();

    Reference target_reference = getUpdatedTargetReference(state);

    auto feedback = std::make_shared<iii_drone_interfaces::action::CableTakeoff::Feedback>();

    feedback->planned_path = reference_trajectory_adapter.ToPathMsg(parameters_->GetParameter("world_frame_id").as_string());
    feedback->vehicle_pose = StateAdapter(awareness_handler()->GetState()).ToPoseStampedMsg(parameters_->GetParameter("world_frame_id").as_string());
    feedback->distance_vehicle_to_cable = (state.position() - target_reference.position()).norm();

    return std::static_pointer_cast<void>(feedback);

}

void CableTakeoffManeuverServer::publishResultAndFinalize(
    Maneuver & maneuver,
    maneuver_result_type_t maneuver_result_type
) {

    auto result = std::make_shared<iii_drone_interfaces::action::CableTakeoff::Result>();
    auto goal_handle = std::static_pointer_cast<GoalHandleCableTakeoff>(maneuver.goal_handle());

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

void CableTakeoffManeuverServer::registerReferenceCallbackOnSuccess(const Maneuver &) {

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

Reference CableTakeoffManeuverServer::getUpdatedTargetReference(
    const iii_drone::control::State &,
    bool compute
) {

    static Atomic<Reference> target_reference;

    if (compute || first_iteration_) {

        auto cda_handler = awareness_handler();

        transform_matrix_t target_transform;

        try {

            target_transform = cda_handler->ComputeTargetTransform(target_adapter_);

            target_reference = Reference(
                target_transform.block<3, 1>(0, 3),
                quatToEul(matToQuat(target_transform.block<3, 3>(0, 0)))[2]
            );

        } catch (const std::runtime_error &e) {

            State target_state(
                start_state_->position() - target_adapter_->target_transform().block<3, 1>(0, 3),
                vector_t::Zero(),
                start_state_->yaw(),
                vector_t::Zero()
            );

            target_reference = Reference(
                target_state.position(),
                target_state.yaw()
            );

        }
    }

    return target_reference;

}
