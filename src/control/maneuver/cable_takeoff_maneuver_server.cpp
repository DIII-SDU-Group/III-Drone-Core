/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/maneuver/cable_takeoff_maneuver_server.hpp>

#include <cmath>

using namespace iii_drone::control::maneuver;
using namespace iii_drone::control;
using namespace iii_drone::types;
using namespace iii_drone::math;
using namespace iii_drone::utils;
using namespace iii_drone::adapters;

namespace {

double CableTakeoffPoseNorm(const State & state, const Reference & reference) {
    const Eigen::Vector4d state_position_and_yaw = {
        state.position()[0], state.position()[1], state.position()[2], state.yaw()
    };
    const Eigen::Vector4d target_position_and_yaw = {
        reference.position()[0], reference.position()[1], reference.position()[2], reference.yaw()
    };
    return (state_position_and_yaw - target_position_and_yaw).norm();
}

// A retry of an aborted airborne takeoff follows the Delay decorator within
// about a second; a stale target from an earlier cycle is never resumed.
constexpr auto kDepartureRetryWindow = std::chrono::seconds(10);

// The streamed takeoff command has settled on the frozen target.
constexpr double kFinalReferencePositionToleranceM = 1.0e-2;
constexpr double kFinalReferenceYawToleranceRad = 1.0e-2;
constexpr double kFinalReferenceVelocityToleranceMps = 1.0e-2;
constexpr double kFinalReferenceAccelerationToleranceMps2 = 5.0e-2;

}  // namespace

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

    if (!awareness_handler()->state_available()) {
        RCLCPP_DEBUG(
            node()->get_logger(),
            "CableTakeoffManeuverServer::CanExecuteManeuver(): Vehicle state is incomplete; waiting for PX4 odometry."
        );
        return false;
    }

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

    // An aborted airborne takeoff cleared its cable target. Its immediate
    // retry may resume the same frozen departure target; nothing else may.
    const bool departure_retry = !drone_awareness.has_target() &&
        drone_awareness.in_flight() &&
        retryDepartureTarget(
            cable_takeoff_maneuver_params.target_cable_id, drone_awareness.state()).has_value();

    if (!drone_awareness.has_target() && !departure_retry) {
        RCLCPP_DEBUG(
            node()->get_logger(),
            "CableTakeoffManeuverServer::CanExecuteManeuver(): Drone does not have a target."
        );
        return false;
    }

    if (departure_retry) {
        RCLCPP_DEBUG(
            node()->get_logger(),
            "CableTakeoffManeuverServer::CanExecuteManeuver(): Accepting airborne retry of the aborted departure target."
        );
    } else if (drone_awareness.target_adapter().target_type() != TARGET_TYPE_CABLE) {
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
    {
        std::lock_guard<std::mutex> lock(terminal_hold_mutex_);
        terminal_hold_.reset();
    }
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

    // After an aborted airborne attempt the vehicle has already departed; a
    // new target below its current position would descend another full
    // clearance distance. Resume the original frozen departure target.
    const auto resumed_target = start_on_cable_.Load() ? std::nullopt :
        retryDepartureTarget(cable_takeoff_maneuver_params.target_cable_id, start_state_.Load());
    {
        std::lock_guard<std::mutex> lock(departure_target_mutex_);
        departure_target_.reset();
        if (resumed_target) target_reference_ = *resumed_target;
        execution_departure_target_ = DepartureTarget{
            target_reference_.Load(), cable_takeoff_maneuver_params.target_cable_id,
            cable_takeoff_maneuver_params.target_cable_distance, {}};
    }
    if (resumed_target) {
        RCLCPP_INFO(
            node()->get_logger(),
            "CableTakeoffManeuverServer::startExecution(): Retrying airborne departure toward the previous frozen clearance target."
        );
    }

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

bool CableTakeoffManeuverServer::rebaseExecution(
    const State & stopped_state,
    std::string & reason
) {
    if (trajectory_generator_client_->busy()) {
        reason = "cable-takeoff trajectory generator is busy";
        return false;
    }
    first_iteration_ = true;
    has_failed_ = false;
    abort_because_gripper_closed_ = false;
    in_flight_since_.reset();
    started_at_ = std::chrono::steady_clock::now();
    last_distance_improvement_at_ = started_at_;
    best_distance_to_target_ = CableTakeoffPoseNorm(stopped_state, *target_reference_);
    reason = "replanned remaining cable takeoff from stopped state";
    return true;
}

Reference CableTakeoffManeuverServer::computeReference(const State & state) {

    if (const auto hold = terminalHold()) return hold->GetReference();

    Reference target_reference = getUpdatedTargetReference(
        state,
        false
    );

    const bool initialize_trajectory = first_iteration_;

    Reference ref;
    
    try {

        // Cable takeoff always builds one jerk-bounded quintic from the
        // measured moving state and samples that fixed segment on subsequent
        // ticks. The state-feedback MPC path is deliberately unavailable: it
        // has no hard jerk bound, leaves the cable at up to ~1.4 m/s, and its
        // restarts open at the MPC acceleration limit, which exceeds the
        // reference continuity envelope from rest.
        ref = trajectory_generator_client_->ComputeReference(
            Reference(state),
            target_reference,
            initialize_trajectory,
            initialize_trajectory,
            trajectory_mode_t::cable_takeoff
        );

    } catch (const std::runtime_error &e) {

        RCLCPP_ERROR(node()->get_logger(), "CableTakeoffManeuverServer::computeReference(): %s", e.what());

        has_failed_ = true;

        ref = Reference(state);

    }

    if (first_iteration_) {
        first_iteration_ = false;
    }

    // Once the airborne command has settled on the frozen target, a bounded
    // terminal correction removes PX4's steady position offset (the same
    // mechanism FlyToPosition uses). Its offset stays well clear of the cable.
    const double yaw_error = std::abs(std::atan2(
        std::sin(ref.yaw() - target_reference.yaw()),
        std::cos(ref.yaw() - target_reference.yaw())));
    if (!has_failed_ && awareness_handler()->in_flight() &&
        (ref.position() - target_reference.position()).norm() <= kFinalReferencePositionToleranceM &&
        yaw_error <= kFinalReferenceYawToleranceRad &&
        ref.velocity().allFinite() &&
        ref.velocity().norm() <= kFinalReferenceVelocityToleranceMps &&
        ref.acceleration().allFinite() &&
        ref.acceleration().norm() <= kFinalReferenceAccelerationToleranceMps2) {
        auto limits = TerminalPositionTrackingController::Limits{};
        limits.arrival_tolerance_m = configuration_->GetParameter(
            "/control/maneuver_controller/cable_takeoff_reached_pose_norm_threshold").as_double();
        {
            std::lock_guard<std::mutex> lock(departure_target_mutex_);
            if (execution_departure_target_) {
                limits.max_offset_m = std::min(limits.max_offset_m,
                    0.5 * execution_departure_target_->target_cable_distance);
            }
        }
        auto hold = std::make_shared<TerminalTrackingHold>(
            target_reference, awareness_handler(), node()->get_clock(),
            TerminalTrackingHold::Clearance{}, 0.0, limits);
        {
            std::lock_guard<std::mutex> lock(terminal_hold_mutex_);
            terminal_hold_ = hold;
        }
        auto hover = std::static_pointer_cast<HoverManeuverServer>(
            registered_maneuvers().at(MANEUVER_TYPE_HOVER));
        hover->AdoptTerminalHold(hold, current_maneuver().Load().requestIdentity());
        RCLCPP_INFO(node()->get_logger(),
            "CableTakeoff terminal tracking started: pose error %.3f, correction authority %.3f m",
            CableTakeoffPoseNorm(state, target_reference), limits.max_offset_m);
        return hold->GetReference();
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

    const double distance = CableTakeoffPoseNorm(state, target_reference);
    const double target_norm_threshold = configuration_->GetParameter(
        "/control/maneuver_controller/cable_takeoff_reached_pose_norm_threshold"
    ).as_double();
    const bool reached = distance < target_norm_threshold;
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

    if (const auto hold = terminalHold();
        hold && hold->phase() != TerminalTrackingHold::Phase::Tracking) {
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
        "CableTakeoffManeuverServer::hasSucceeded(): in-flight and target reached for %.2f seconds (pose_norm=%.3f threshold=%.3f).",
        std::chrono::duration<double>(stable_duration).count(),
        distance,
        target_norm_threshold
    );

    return true;

}

bool CableTakeoffManeuverServer::hasFailed(Maneuver &) {

    auto cda_handler = awareness_handler();

    const auto hold = terminalHold();
    if (hold && (hold->phase() == TerminalTrackingHold::Phase::Degraded ||
                 hold->phase() == TerminalTrackingHold::Phase::Unrecoverable)) {
        RCLCPP_ERROR(node()->get_logger(),
            "CableTakeoff terminal tracking failed: %s", hold->failureReason().c_str());
        return true;
    }

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
        logNotOffboard("CableTakeoffManeuverServer::hasFailed(): Drone is not offboard.");
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
    // An active terminal correction has its own bounded convergence and
    // authority-exhaustion failures; the nominal stall rule does not apply.
    if (!hold && started_at_.has_value() && last_distance_improvement_at_.has_value()) {
        const auto elapsed = now - started_at_.value();
        const auto since_improvement = now - last_distance_improvement_at_.value();
        if (
            elapsed > std::chrono::seconds(10) &&
            since_improvement > std::chrono::seconds(5)
        ) {
            const State state = cda_handler->GetState();
            const Reference target_reference = getUpdatedTargetReference(state);
            const double distance = CableTakeoffPoseNorm(state, target_reference);
            const double target_norm_threshold = configuration_->GetParameter(
                "/control/maneuver_controller/cable_takeoff_reached_pose_norm_threshold"
            ).as_double();
            RCLCPP_WARN(
                node()->get_logger(),
                "CableTakeoffManeuverServer::hasFailed(): Target pose norm stalled. distance=%.3f best_distance=%.3f threshold=%.3f elapsed=%.2f since_improvement=%.2f start_on_cable=%s",
                distance,
                best_distance_to_target_,
                target_norm_threshold,
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

    {
        // Only a started airborne attempt that aborted may be resumed, and
        // only by an immediate retry (see retryDepartureTarget()).
        std::lock_guard<std::mutex> lock(departure_target_mutex_);
        departure_target_.reset();
        if (maneuver_result_type == MANEUVER_RESULT_TYPE_ABORT &&
            execution_departure_target_ && !abort_because_gripper_closed_.Load() &&
            awareness_handler()->in_flight()) {
            departure_target_ = execution_departure_target_;
            departure_target_->aborted_at = std::chrono::steady_clock::now();
        }
        execution_departure_target_.reset();
    }

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

void CableTakeoffManeuverServer::registerReferenceCallbackOnSuccess(const Maneuver & maneuver) {

    // registered_maneuvers() returns a copy: take the server within this
    // statement, never an iterator into the temporary map.
    std::shared_ptr<HoverManeuverServer> hover_maneuver_server = std::static_pointer_cast<HoverManeuverServer>(
        registered_maneuvers().at(MANEUVER_TYPE_HOVER));

    if (const auto hold = terminalHold()) {
        hover_maneuver_server->AdoptTerminalHold(hold, maneuver.requestIdentity());
    } else {
        hover_maneuver_server->Update(target_reference_);
    }

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

std::shared_ptr<TerminalTrackingHold> CableTakeoffManeuverServer::terminalHold() const {
    std::lock_guard<std::mutex> lock(terminal_hold_mutex_);
    return terminal_hold_;
}

std::optional<Reference> CableTakeoffManeuverServer::retryDepartureTarget(
    int target_cable_id,
    const State & state
) const {
    std::lock_guard<std::mutex> lock(departure_target_mutex_);
    if (!departure_target_ || departure_target_->target_cable_id != target_cable_id ||
        std::chrono::steady_clock::now() - departure_target_->aborted_at > kDepartureRetryWindow ||
        !state.position().allFinite() ||
        (state.position() - departure_target_->reference.position()).norm() >
            departure_target_->target_cable_distance) {
        return std::nullopt;
    }
    return departure_target_->reference;
}
