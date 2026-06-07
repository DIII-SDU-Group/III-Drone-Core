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
    iii_drone::configuration::Configuration::SharedPtr parameters,
    iii_drone::control::TrajectoryGeneratorClient::SharedPtr trajectory_generator_client
) : ManeuverServer(
    node,
    combined_drone_awareness_handler,
    action_name,
    wait_for_execute_poll_ms,
    evaluate_done_poll_ms
),  configuration_(parameters),
    trajectory_generator_client_(trajectory_generator_client) {

    createServer<iii_drone_interfaces::action::CableTakeoff>();

}

bool CableTakeoffManeuverServer::CanExecuteManeuver(
    const Maneuver & maneuver,
    const iii_drone::adapters::CombinedDroneAwarenessAdapter & drone_awareness
) const {

    cable_takeoff_maneuver_params_t cable_takeoff_maneuver_params(maneuver.maneuver_params());

    if (maneuver.maneuver_type() != MANEUVER_TYPE_CABLE_TAKEOFF) {
        RCLCPP_DEBUG(
            node()->get_logger(),
            "CableTakeoffManeuverServer::CanExecuteManeuver(): Maneuver type is not MANEUVER_TYPE_CABLE_TAKEOFF."
        );
        return false;
    }

    if (!drone_awareness.armed()) {
        RCLCPP_DEBUG(
            node()->get_logger(),
            "CableTakeoffManeuverServer::CanExecuteManeuver(): Drone is not armed."
        );
        return false;
    }

    if (!drone_awareness.offboard()) {
        RCLCPP_DEBUG(
            node()->get_logger(),
            "CableTakeoffManeuverServer::CanExecuteManeuver(): Drone is not offboard."
        );
        return false;
    }

    if (!drone_awareness.gripper_open()) {
        RCLCPP_DEBUG(
            node()->get_logger(),
            "CableTakeoffManeuverServer::CanExecuteManeuver(): Gripper is not open."
        );
        return false;
    }

    if (!drone_awareness.on_cable() && !drone_awareness.in_flight()) {
        RCLCPP_DEBUG(
            node()->get_logger(),
            "CableTakeoffManeuverServer::CanExecuteManeuver(): Drone is neither on cable nor in flight."
        );
        return false;
    }

    if (!drone_awareness.has_target()) {
        RCLCPP_DEBUG(
            node()->get_logger(),
            "CableTakeoffManeuverServer::CanExecuteManeuver(): Drone does not have a target."
        );
        return false;
    }

    if (drone_awareness.target_adapter().target_type() != TARGET_TYPE_CABLE) {
        RCLCPP_DEBUG(
            node()->get_logger(),
            "CableTakeoffManeuverServer::CanExecuteManeuver(): Target type is not TARGET_TYPE_CABLE."
        );
        return false;
    }

    if (drone_awareness.target_adapter().target_id() != cable_takeoff_maneuver_params.target_cable_id) {
        RCLCPP_DEBUG(
            node()->get_logger(),
            "CableTakeoffManeuverServer::CanExecuteManeuver(): Requested target id %d differs from active target id %d; accepting because cable takeoff freezes a local clearance target.",
            cable_takeoff_maneuver_params.target_cable_id,
            drone_awareness.target_adapter().target_id()
        );
    }

    if(cable_takeoff_maneuver_params.target_cable_distance < configuration_->GetParameter("/control/maneuver_controller/cable_takeoff_min_target_cable_distance").as_double()) {
        RCLCPP_DEBUG(
            node()->get_logger(),
            "CableTakeoffManeuverServer::CanExecuteManeuver(): target_cable_distance %.3f is below minimum %.3f.",
            cable_takeoff_maneuver_params.target_cable_distance,
            configuration_->GetParameter("/control/maneuver_controller/cable_takeoff_min_target_cable_distance").as_double()
        );
        return false;
    }

    if (cable_takeoff_maneuver_params.target_cable_distance > configuration_->GetParameter("/control/maneuver_controller/cable_takeoff_max_target_cable_distance").as_double()) {
        RCLCPP_DEBUG(
            node()->get_logger(),
            "CableTakeoffManeuverServer::CanExecuteManeuver(): target_cable_distance %.3f is above maximum %.3f.",
            cable_takeoff_maneuver_params.target_cable_distance,
            configuration_->GetParameter("/control/maneuver_controller/cable_takeoff_max_target_cable_distance").as_double()
        );
        return false;
    }

    RCLCPP_DEBUG(
        node()->get_logger(),
        "CableTakeoffManeuverServer::CanExecuteManeuver(): Cable takeoff maneuver can be executed."
    );

    return true;

}

iii_drone::adapters::CombinedDroneAwarenessAdapter CableTakeoffManeuverServer::ExpectedAwarenessAfterExecution(const Maneuver & maneuver) {

    cable_takeoff_maneuver_params_t cable_takeoff_maneuver_params(maneuver.maneuver_params());

    transform_matrix_t target_transform = cable_takeoff_maneuver_params.get_target_transform();

    TargetAdapter target_adapter = TargetAdapter(
        TARGET_TYPE_CABLE,
        cable_takeoff_maneuver_params.target_cable_id,
        configuration_->GetParameter("/tf/cable_gripper_frame_id").as_string(),
        target_transform
    );

    State current_state = awareness_handler()->GetState();
    State target_state(
        current_state.position() - vector_t(0, 0, cable_takeoff_maneuver_params.target_cable_distance),
        vector_t::Zero(),
        current_state.yaw(),
        vector_t::Zero()
    );

    iii_drone::adapters::CombinedDroneAwarenessAdapter awareness_after;

    awareness_after.armed() = true;
    awareness_after.offboard() = true;
    awareness_after.target_position_known() = true;
    awareness_after.drone_location() = DRONE_LOCATION_IN_FLIGHT;
    awareness_after.target_adapter() = target_adapter;
    awareness_after.state() = target_state;

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
        // configuration_->GetParameter("/tf/drone_frame_id").as_string(),
        configuration_->GetParameter("/tf/cable_gripper_frame_id").as_string(),
        target_transform
        // transform_matrix_t::Identity()
    );

    start_state_ = cda_handler->GetState();

    if (trajectory_generator_client_->busy()) {

        std::string error_message = "CableTakeoffManeuverServer::startExecution(): Trajectory generator client is busy, cannot start execution of maneuver.";

        RCLCPP_FATAL(node()->get_logger(), error_message.c_str());

        throw std::runtime_error(error_message);

    }

    first_iteration_ = true;
    has_failed_ = false;
    abort_because_gripper_closed_ = false;
    start_on_cable_ = cda_handler->on_cable();
    in_flight_since_.reset();
    started_at_ = std::chrono::steady_clock::now();
    last_distance_improvement_at_ = started_at_;
    best_distance_to_target_ = std::numeric_limits<double>::infinity();

    cda_handler->SetTarget(target_adapter_);

    target_reference_ = Reference(
        start_state_->position() - vector_t(0, 0, cable_takeoff_maneuver_params.target_cable_distance),
        start_state_->yaw()
    );

    const Reference frozen_target_reference = target_reference_;

    RCLCPP_INFO(
        node()->get_logger(),
        "CableTakeoffManeuverServer::startExecution(): Frozen takeoff clearance target position=[%.3f, %.3f, %.3f] yaw=%.3f target_id=%d distance=%.3f",
        frozen_target_reference.position()[0],
        frozen_target_reference.position()[1],
        frozen_target_reference.position()[2],
        frozen_target_reference.yaw(),
        cable_takeoff_maneuver_params.target_cable_id,
        cable_takeoff_maneuver_params.target_cable_distance
    );

}

bool CableTakeoffManeuverServer::canCancel() {
    
    return true;

}

Reference CableTakeoffManeuverServer::computeReference(const State & state) {

    Reference target_reference = getUpdatedTargetReference(
        state,
        false
    );

    const bool use_mpc = configuration_->GetParameter("/control/maneuver_controller/cable_takeoff_use_mpc").as_bool();
    bool reset = first_iteration_;
    bool set_reference = true;
    bool compute_with_mpc = use_mpc;

    Reference ref;
    
    try {

        ref = trajectory_generator_client_->ComputeReference(
            state,
            target_reference,
            set_reference,
            reset,
            trajectory_mode_t::cable_takeoff,
            compute_with_mpc
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

    if (first_iteration_) {
        first_iteration_ = false;
    }

    return ref;

}

bool CableTakeoffManeuverServer::hasSucceeded(Maneuver &) {

    auto cda_handler = awareness_handler();

    if (!cda_handler->in_flight()) {
        in_flight_since_.reset();
        return false;
    }

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

    const bool reached = distance < configuration_->GetParameter("/control/maneuver_controller/reached_position_euclidean_distance_threshold").as_double();
    if (distance + 0.05 < best_distance_to_target_) {
        best_distance_to_target_ = distance;
        last_distance_improvement_at_ = std::chrono::steady_clock::now();
        RCLCPP_DEBUG(
            node()->get_logger(),
            "CableTakeoffManeuverServer::hasSucceeded(): target distance improved to %.3f m.",
            distance
        );
    }

    if (!reached) {
        in_flight_since_.reset();
        return false;
    }

    const auto now = std::chrono::steady_clock::now();
    if (!in_flight_since_.has_value()) {
        in_flight_since_ = now;
        return false;
    }

    const auto stable_duration = now - in_flight_since_.value();
    if (stable_duration < std::chrono::milliseconds(1500)) {
        return false;
    }

    RCLCPP_INFO(
        node()->get_logger(),
        "CableTakeoffManeuverServer::hasSucceeded(): in-flight and target reached for %.2f seconds.",
        std::chrono::duration<double>(stable_duration).count()
    );

    return true;

}

bool CableTakeoffManeuverServer::hasFailed(Maneuver &) {

    auto cda_handler = awareness_handler();

    if (has_failed_) {
        RCLCPP_WARN(
            node()->get_logger(),
            "CableTakeoffManeuverServer::hasFailed(): Internal maneuver failure flag is set."
        );
        return true;
    }

    if (!cda_handler->gripper_open()) {
        RCLCPP_WARN(
            node()->get_logger(),
            "CableTakeoffManeuverServer::hasFailed(): Gripper is closed."
        );
        abort_because_gripper_closed_ = true;
        return true;
    }

    if (!cda_handler->offboard()) {
        RCLCPP_WARN(
            node()->get_logger(),
            "CableTakeoffManeuverServer::hasFailed(): Drone is not offboard."
        );
        return true;
    }

    if (!cda_handler->armed()) {
        RCLCPP_WARN(
            node()->get_logger(),
            "CableTakeoffManeuverServer::hasFailed(): Drone is not armed."
        );
        return true;
    }

    if (cda_handler->target_adapter() != target_adapter_) {
        RCLCPP_WARN(
            node()->get_logger(),
            "CableTakeoffManeuverServer::hasFailed(): Target adapter does not match."
        );
        return true;
    }

    const auto now = std::chrono::steady_clock::now();
    if (started_at_.has_value() && last_distance_improvement_at_.has_value()) {
        const auto elapsed = now - started_at_.value();
        const auto since_improvement = now - last_distance_improvement_at_.value();
        if (
            elapsed > std::chrono::seconds(10) &&
            since_improvement > std::chrono::seconds(5)
        ) {
            const State state = cda_handler->GetState();
            const Reference target_reference = getUpdatedTargetReference(state);
            const double distance = (state.position() - target_reference.position()).norm();
            RCLCPP_WARN(
                node()->get_logger(),
                "CableTakeoffManeuverServer::hasFailed(): Target distance stalled. distance=%.3f best_distance=%.3f elapsed=%.2f since_improvement=%.2f start_on_cable=%s",
                distance,
                best_distance_to_target_,
                std::chrono::duration<double>(elapsed).count(),
                std::chrono::duration<double>(since_improvement).count(),
                start_on_cable_.Load() ? "true" : "false"
            );
            return true;
        }
    }

    return false;

}

std::shared_ptr<void> CableTakeoffManeuverServer::getFeedback(Maneuver &) {

    ReferenceTrajectory reference_trajectory = trajectory_generator_client_->GetReferenceTrajectory();

    ReferenceTrajectoryAdapter reference_trajectory_adapter(reference_trajectory);

    State state = awareness_handler()->GetState();

    Reference target_reference = getUpdatedTargetReference(state);

    auto feedback = std::make_shared<iii_drone_interfaces::action::CableTakeoff::Feedback>();

    feedback->planned_path = reference_trajectory_adapter.ToPathMsg(configuration_->GetParameter("/tf/world_frame_id").as_string());
    feedback->vehicle_pose = StateAdapter(awareness_handler()->GetState()).ToPoseStampedMsg(configuration_->GetParameter("/tf/world_frame_id").as_string());
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
            if (abort_because_gripper_closed_) {
                RCLCPP_INFO(
                    node()->get_logger(),
                    "CableTakeoffManeuverServer::publishResultAndFinalize(): Preserving cable target because abort was caused by closed gripper."
                );
            } else {
                awareness_handler()->ClearTarget();
            }
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

    hover_maneuver_server->Update(target_reference_);

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

    (void) compute;

    return target_reference_;

}
