#include <iii_drone_core/control/maneuver/follow_waypoint_path_maneuver_server.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>

#include <iii_drone_core/adapters/combined_drone_awareness_adapter.hpp>
#include <iii_drone_core/adapters/reference_adapter.hpp>
#include <iii_drone_core/adapters/reference_trajectory_adapter.hpp>
#include <iii_drone_core/adapters/state_adapter.hpp>
#include <iii_drone_core/adapters/target_adapter.hpp>
#include <iii_drone_core/control/maneuver/hover_maneuver_server.hpp>
#include <iii_drone_core/utils/math.hpp>

using namespace iii_drone::adapters;
using namespace iii_drone::control;
using namespace iii_drone::control::maneuver;
using namespace iii_drone::math;
using namespace iii_drone::types;

namespace {

bool isFlightCapable(const CombinedDroneAwarenessAdapter & awareness) {
    return awareness.in_flight() || (
        awareness.armed() && awareness.on_cable() && awareness.gripper_open()
    );
}

double shortestYawError(double current_yaw, double target_yaw) {
    return std::atan2(std::sin(target_yaw - current_yaw), std::cos(target_yaw - current_yaw));
}

}  // namespace

void WaypointPathTerminalStopProof::reset() {
    settled_since_.reset();
}

bool WaypointPathTerminalStopProof::observe(
    bool path_finished_and_at_target,
    const State & measured_state,
    const ControlledCancellationConfig & config,
    Clock::time_point now
) {
    const auto velocity = measured_state.velocity();
    const auto angular_velocity = measured_state.angular_velocity();
    if (
        !path_finished_and_at_target ||
        !velocity.allFinite() ||
        !angular_velocity.allFinite() ||
        velocity.norm() > config.velocity_threshold_m_s ||
        std::abs(angular_velocity(2)) > config.yaw_rate_threshold_rad_s
    ) {
        reset();
        return false;
    }
    if (!settled_since_.has_value()) {
        settled_since_ = now;
    }
    return std::chrono::duration<double>(now - *settled_since_).count() >= config.settle_time_s;
}

FollowWaypointPathManeuverServer::FollowWaypointPathManeuverServer(
    rclcpp_lifecycle::LifecycleNode * node,
    CombinedDroneAwarenessHandler::SharedPtr awareness_handler,
    const std::string & action_name,
    unsigned int wait_for_execute_poll_ms,
    unsigned int evaluate_done_poll_ms,
    iii_drone::configuration::Configuration::SharedPtr configuration
) : ManeuverServer(
        node,
        awareness_handler,
        action_name,
        wait_for_execute_poll_ms,
        evaluate_done_poll_ms
    ),
    configuration_(configuration) {
    createServer<FollowWaypointPath>();
}

void FollowWaypointPathManeuverServer::RegisterAppliedRestReferenceCallback(
    std::function<bool(const std::string &, const Reference &)> callback
) {
    applied_rest_reference_ = std::move(callback);
}

std::shared_ptr<TerminalTrackingHold> FollowWaypointPathManeuverServer::createTerminalHold(
    const Reference & nominal, const std::string & request_identity, bool quiescent
) {
    TerminalPositionTrackingController::Limits limits;
    limits.arrival_tolerance_m = configuration_->GetParameter(
        "/control/maneuver_controller/reached_position_euclidean_distance_threshold"
    ).as_double();
    auto hold = std::make_shared<TerminalTrackingHold>(
        nominal, awareness_handler(), node()->get_clock(),
        TerminalTrackingHold::Clearance{}, 0.0, limits);
    if (quiescent && !hold->RequestQuiescence()) {
        throw std::runtime_error("waypoint terminal hold could not be frozen at its rest endpoint");
    }
    auto hover = std::static_pointer_cast<HoverManeuverServer>(
        registered_maneuvers().at(MANEUVER_TYPE_HOVER));
    hover->AdoptTerminalHold(hold, request_identity);
    return hold;
}

bool FollowWaypointPathManeuverServer::CanExecuteManeuver(
    const Maneuver & maneuver,
    const CombinedDroneAwarenessAdapter & awareness
) const {
    if (maneuver.maneuver_type() != MANEUVER_TYPE_FOLLOW_WAYPOINT_PATH) {
        return false;
    }
    if (!isFlightCapable(awareness) || !awareness.offboard()) {
        RCLCPP_WARN(node()->get_logger(), "FollowWaypointPath: drone is not ready for position flight");
        return false;
    }
    return validateManeuverParameters(
        follow_waypoint_path_maneuver_params_t(maneuver.maneuver_params())
    );
}

CombinedDroneAwarenessAdapter FollowWaypointPathManeuverServer::ExpectedAwarenessAfterExecution(
    const Maneuver & maneuver
) {
    const auto params = follow_waypoint_path_maneuver_params_t(maneuver.maneuver_params());
    const auto waypoints = transformWaypoints(params);
    CombinedDroneAwarenessAdapter awareness_after;
    awareness_after.armed() = true;
    awareness_after.offboard() = true;
    awareness_after.target_adapter() = TargetAdapter();
    awareness_after.target_position_known() = false;
    awareness_after.drone_location() = DRONE_LOCATION_IN_FLIGHT;
    awareness_after.state() = State(
        waypoints.back().position,
        vector_t::Zero(),
        eulToQuat(euler_angles_t(0.0, 0.0, waypoints.back().yaw)),
        vector_t::Zero()
    );
    return awareness_after;
}

maneuver_type_t FollowWaypointPathManeuverServer::maneuver_type() const {
    return MANEUVER_TYPE_FOLLOW_WAYPOINT_PATH;
}

void FollowWaypointPathManeuverServer::startExecution(Maneuver & maneuver) {
    {
        std::lock_guard<std::mutex> lock(plan_mutex_);
        terminal_stop_proof_.reset();
        terminal_motion_proof_.reset();
        cancellation_motion_proof_.reset();
        terminal_hold_.reset();
        cancellation_proof_started_.reset();
        cancellation_proof_failed_ = false;
    }
    const auto params = follow_waypoint_path_maneuver_params_t(maneuver.maneuver_params());
    const State state = awareness_handler()->GetState();
    const auto terminal_seed = consumeTerminalStartReference(maneuver.requestIdentity());
    try {
        const auto waypoints = transformWaypoints(params);
        const auto constraints = resolveConstraints(params);
        WaypointPathPlan new_plan = planner_.plan(
            terminal_seed.value_or(Reference(state)),
            waypoints,
            params.repeat,
            params.repeat_from_index,
            constraints
        );
        std::lock_guard<std::mutex> lock(plan_mutex_);
        plan_ = std::move(new_plan);
        active_waypoints_ = waypoints;
        active_constraints_ = constraints;
        active_repeat_ = params.repeat;
        active_repeat_from_index_ = params.repeat_from_index;
        active_sample_ = plan_.sample(0.0);
        execution_start_time_ = node()->now();
        has_failed_ = false;
        RCLCPP_INFO(
            node()->get_logger(),
            "FollowWaypointPath: planned prefix %.2fs, loop %.2fs, repeat=%s",
            plan_.prefixDurationS(),
            plan_.loopDurationS(),
            params.repeat ? "true" : "false"
        );
    } catch (const std::exception & error) {
        has_failed_ = true;
        RCLCPP_ERROR(node()->get_logger(), "FollowWaypointPath planning failed: %s", error.what());
    }
    awareness_handler()->ClearTarget();
}

bool FollowWaypointPathManeuverServer::canCancel() {
    return true;
}

std::optional<ControlledCancellationConfig>
FollowWaypointPathManeuverServer::controlledCancellationConfig() const {
    return controlledCancellationConfigFrom(configuration_);
}

bool FollowWaypointPathManeuverServer::controlledCancellationFailure() const {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    return cancellation_proof_failed_;
}

bool FollowWaypointPathManeuverServer::controlledCancellationComplete(
    const ControlledCancellationConfig & config
) {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    if (!controlledCancellationProfileComplete()) {
        cancellation_motion_proof_.reset();
        return false;
    }
    if (!cancellation_proof_started_) {
        cancellation_proof_started_ = std::chrono::steady_clock::now();
    }
    const auto rest = controlledCancellationFinalReference();
    const auto owner = current_maneuver().Load().requestIdentity();
    const bool exact_applied_rest = rest && applied_rest_reference_ &&
        applied_rest_reference_(owner, *rest);
    const auto measured = awareness_handler()->GetMeasuredOdometry();
    const bool proved = exact_applied_rest && measured &&
        cancellation_motion_proof_.observe(true, *measured, config, node()->now());
    if (proved) {
        try {
            terminal_hold_ = createTerminalHold(*rest, owner, true);
            RCLCPP_INFO(node()->get_logger(),
                "FollowWaypointPath cancellation retained finite rest after estimated-motion proof");
            return true;
        } catch (const std::exception & error) {
            RCLCPP_ERROR(node()->get_logger(),
                "FollowWaypointPath cancellation could not retain its rest command: %s", error.what());
            cancellation_proof_failed_ = true;
            return false;
        }
    }
    if (!exact_applied_rest || !measured) cancellation_motion_proof_.reset();
    if (std::chrono::steady_clock::now() - *cancellation_proof_started_ >
        std::chrono::seconds(10)) {
        cancellation_proof_failed_ = true;
        RCLCPP_ERROR(node()->get_logger(),
            "FollowWaypointPath cancellation could not prove a current applied rest command and estimated stop");
    }
    return false;
}

bool FollowWaypointPathManeuverServer::rebaseExecution(
    const State & stopped_state,
    std::string & reason
) {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    terminal_stop_proof_.reset();
    terminal_motion_proof_.reset();
    cancellation_motion_proof_.reset();
    terminal_hold_.reset();
    cancellation_proof_started_.reset();
    cancellation_proof_failed_ = false;
    if (active_waypoints_.empty()) {
        reason = "waypoint path has no remaining objective";
        return false;
    }

    const std::size_t next = std::min<std::size_t>(
        active_sample_.waypoint_index,
        active_waypoints_.size() - 1
    );
    std::vector<WaypointPathWaypoint> remaining;
    bool repeat = false;
    uint32_t repeat_from = 0;
    if (active_repeat_) {
        const std::size_t loop_begin = std::min<std::size_t>(
            active_repeat_from_index_, active_waypoints_.size() - 1
        );
        for (std::size_t i = next; i < active_waypoints_.size(); ++i) {
            remaining.push_back(active_waypoints_[i]);
        }
        for (std::size_t i = loop_begin; i < next; ++i) {
            remaining.push_back(active_waypoints_[i]);
        }
        repeat = remaining.size() >= 2;
    } else {
        remaining.assign(active_waypoints_.begin() + next, active_waypoints_.end());
    }
    if (remaining.empty()) {
        reason = "waypoint path completed while stopping";
        return false;
    }

    try {
        plan_ = planner_.plan(
            Reference(stopped_state), remaining, repeat, repeat_from, active_constraints_
        );
        active_waypoints_ = std::move(remaining);
        active_repeat_ = repeat;
        active_repeat_from_index_ = repeat_from;
        active_sample_ = plan_.sample(0.0);
        execution_start_time_ = node()->now();
        has_failed_ = false;
        reason = "replanned remaining waypoint path from stopped state";
        return true;
    } catch (const std::exception & error) {
        reason = std::string("waypoint replan failed: ") + error.what();
        return false;
    }
}

Reference FollowWaypointPathManeuverServer::computeReference(const State & state) {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    if (has_failed_ || plan_.prefix.empty()) {
        return Reference(state);
    }
    if (terminal_hold_) return terminal_hold_->GetReference();
    const double elapsed_s = (node()->now() - execution_start_time_).seconds();
    active_sample_ = plan_.sample(elapsed_s);
    if (!active_repeat_ && elapsed_s >= plan_.prefixDurationS() &&
        !plan_.prefix.references.empty()) {
        const Reference nominal = plan_.prefix.references.back();
        if (nominal.position().allFinite() && nominal.velocity().allFinite() &&
            nominal.acceleration().allFinite() &&
            nominal.velocity().norm() <= 1.0e-5 &&
            nominal.acceleration().norm() <= 1.0e-5 &&
            std::isfinite(nominal.yaw_rate()) && std::abs(nominal.yaw_rate()) <= 1.0e-5 &&
            std::isfinite(nominal.yaw_acceleration()) &&
            std::abs(nominal.yaw_acceleration()) <= 1.0e-5) {
            try {
                terminal_hold_ = createTerminalHold(
                    nominal, current_maneuver().Load().requestIdentity(), false);
                RCLCPP_INFO(node()->get_logger(),
                    "FollowWaypointPath terminal tracking started: nominal error %.3f m, correction authority 0.4 m",
                    (state.position() - nominal.position()).norm());
                return terminal_hold_->GetReference();
            } catch (const std::exception & error) {
                has_failed_ = true;
                RCLCPP_ERROR(node()->get_logger(),
                    "FollowWaypointPath terminal tracking failed to start: %s", error.what());
            }
        }
    }
    return active_sample_.reference.CopyWithNewStamp(node()->now());
}

bool FollowWaypointPathManeuverServer::terminalBlocked(
    const char * reason, double position_error, double yaw_error
) {
    const auto now = std::chrono::steady_clock::now();
    if (!terminal_blocked_since_) {
        terminal_blocked_since_ = now;
    }
    const double blocked_s = std::chrono::duration<double>(now - *terminal_blocked_since_).count();
    if (blocked_s >= 30.0 && now - terminal_block_logged_at_ >= std::chrono::seconds(30)) {
        terminal_block_logged_at_ = now;
        RCLCPP_INFO(node()->get_logger(),
            "FollowWaypointPath terminal phase has not succeeded for %.0f s: %s "
            "(position error %.3f m, yaw error %.3f rad, hold phase %d)",
            blocked_s, reason, position_error, yaw_error,
            terminal_hold_ ? static_cast<int>(terminal_hold_->phase()) : -1);
    }
    return false;
}

bool FollowWaypointPathManeuverServer::hasSucceeded(Maneuver & maneuver) {
    const auto params = follow_waypoint_path_maneuver_params_t(maneuver.maneuver_params());
    if (params.repeat) {
        return false;
    }
    std::lock_guard<std::mutex> lock(plan_mutex_);
    if (has_failed_ || plan_.prefix.references.empty()) {
        terminal_stop_proof_.reset();
        terminal_motion_proof_.reset();
        terminal_blocked_since_.reset();
        return false;
    }
    if ((node()->now() - execution_start_time_).seconds() < plan_.prefixDurationS()) {
        terminal_stop_proof_.reset();
        terminal_motion_proof_.reset();
        terminal_blocked_since_.reset();
        return false;
    }
    const State state = awareness_handler()->GetState();
    const Reference target = plan_.prefix.references.back();
    if (!terminal_hold_ && target.velocity().allFinite() &&
        target.acceleration().allFinite() &&
        target.velocity().norm() <= 1.0e-5 &&
        target.acceleration().norm() <= 1.0e-5) {
        terminal_motion_proof_.reset();
        return false;
    }
    const double position_error = (state.position() - target.position()).norm();
    const double yaw_error = std::abs(shortestYawError(state.yaw(), target.yaw()));
    const bool terminal_pose_reached =
        position_error < configuration_->GetParameter(
            "/control/maneuver_controller/reached_position_euclidean_distance_threshold"
        ).as_double() &&
        yaw_error < configuration_->GetParameter(
            "/control/maneuver_controller/reached_yaw_error_threshold"
        ).as_double();
    if (!terminal_hold_) {
        // Preserve the previous completion contract for a planner endpoint
        // that was not a stationary nominal reference.
        const bool stopped = terminal_stop_proof_.observe(
            terminal_pose_reached, state,
            controlledCancellationConfigFrom(configuration_),
            WaypointPathTerminalStopProof::Clock::now());
        if (!stopped) {
            return terminalBlocked(
                terminal_pose_reached ? "stop proof settling" : "terminal pose not reached",
                position_error, yaw_error);
        }
        terminal_blocked_since_.reset();
        return true;
    }
    if (terminal_hold_->phase() != TerminalTrackingHold::Phase::Tracking) {
        terminal_motion_proof_.reset();
        return terminalBlocked("terminal hold not tracking", position_error, yaw_error);
    }
    if (!terminal_pose_reached) {
        terminal_motion_proof_.reset();
        terminal_hold_->ResumeTracking();
        return terminalBlocked("terminal pose not reached", position_error, yaw_error);
    }
    terminal_hold_->RequestQuiescence();
    if (!terminal_hold_->isQuiescent()) {
        terminal_motion_proof_.reset();
        return terminalBlocked("terminal hold not quiescent", position_error, yaw_error);
    }
    if (!applied_rest_reference_ ||
        !applied_rest_reference_(maneuver.requestIdentity(), terminal_hold_->lastCommand())) {
        terminal_motion_proof_.reset();
        return terminalBlocked("rest reference not applied by the consumer", position_error, yaw_error);
    }
    const auto measured = awareness_handler()->GetMeasuredOdometry();
    if (!measured) {
        terminal_motion_proof_.reset();
        return terminalBlocked("no measured odometry", position_error, yaw_error);
    }
    if (!terminal_motion_proof_.observe(
            true, *measured, controlledCancellationConfigFrom(configuration_), node()->now())) {
        return terminalBlocked("motion proof settling", position_error, yaw_error);
    }
    terminal_blocked_since_.reset();
    return true;
}

bool FollowWaypointPathManeuverServer::hasFailed(Maneuver &) {
    const auto awareness = awareness_handler()->adapter();
    {
        std::lock_guard<std::mutex> lock(plan_mutex_);
        if (terminal_hold_ &&
            (terminal_hold_->phase() == TerminalTrackingHold::Phase::Degraded ||
             terminal_hold_->phase() == TerminalTrackingHold::Phase::Unrecoverable)) {
            RCLCPP_ERROR(node()->get_logger(),
                "FollowWaypointPath terminal tracking failed: %s",
                terminal_hold_->failureReason().c_str());
            return true;
        }
    }
    return
        has_failed_ ||
        !isFlightCapable(awareness) ||
        !awareness.offboard() ||
        !awareness.armed();
}

std::shared_ptr<void> FollowWaypointPathManeuverServer::getFeedback(Maneuver &) {
    auto feedback = std::make_shared<FollowWaypointPath::Feedback>();
    std::lock_guard<std::mutex> lock(plan_mutex_);
    const auto preview = plan_.previewReferences();
    std::vector<Reference> reduced_preview;
    const std::size_t stride = std::max<std::size_t>(1, preview.size() / 500);
    for (std::size_t index = 0; index < preview.size(); index += stride) {
        reduced_preview.push_back(preview[index]);
    }
    if (!preview.empty() && (reduced_preview.empty() ||
        (reduced_preview.back().position() - preview.back().position()).norm() > 1.0e-6)) {
        reduced_preview.push_back(preview.back());
    }
    feedback->planned_path = ReferenceTrajectoryAdapter(
        ReferenceTrajectory(reduced_preview)
    ).ToPathMsg(configuration_->GetParameter("/tf/world_frame_id").as_string());
    feedback->vehicle_pose = StateAdapter(awareness_handler()->GetState()).ToPoseStampedMsg(
        configuration_->GetParameter("/tf/world_frame_id").as_string()
    );
    feedback->active_waypoint_index = active_sample_.waypoint_index;
    feedback->active_primitive_index = active_sample_.primitive_index;
    feedback->path_progress = static_cast<float>(active_sample_.progress);
    return feedback;
}

void FollowWaypointPathManeuverServer::publishResultAndFinalize(
    Maneuver & maneuver,
    maneuver_result_type_t result_type
) {
    auto result = std::make_shared<FollowWaypointPath::Result>();
    auto goal_handle = std::static_pointer_cast<GoalHandle>(maneuver.goal_handle());
    {
        std::lock_guard<std::mutex> lock(plan_mutex_);
        result->final_waypoint_index = active_sample_.waypoint_index;
        result->target_reference = ReferenceAdapter(
            result_type == MANEUVER_RESULT_TYPE_CANCEL
                ? controlledCancellationFinalReference().value_or(active_sample_.reference)
                : active_sample_.reference
        ).ToMsg();
    }
    result->success = result_type == MANEUVER_RESULT_TYPE_SUCCEED;
    if (result_type == MANEUVER_RESULT_TYPE_SUCCEED) {
        result->reason = "path completed";
        goal_handle->succeed(result);
    } else if (result_type == MANEUVER_RESULT_TYPE_CANCEL) {
        result->reason = "path canceled after controlled stop";
        goal_handle->canceled(result);
    } else {
        result->reason = "path aborted";
        goal_handle->abort(result);
    }
}

void FollowWaypointPathManeuverServer::registerReferenceCallbackOnSuccess(const Maneuver & maneuver) {
    const auto hover = std::static_pointer_cast<HoverManeuverServer>(
        registered_maneuvers().at(MANEUVER_TYPE_HOVER)
    );
    Reference final_reference;
    {
        std::lock_guard<std::mutex> lock(plan_mutex_);
        final_reference = active_sample_.reference;
    }
    std::shared_ptr<TerminalTrackingHold> hold;
    {
        std::lock_guard<std::mutex> lock(plan_mutex_);
        hold = terminal_hold_;
    }
    if (hold) hover->AdoptTerminalHold(hold, maneuver.requestIdentity());
    else hover->Update(final_reference);
    registerCallback(std::bind(&HoverManeuverServer::GetReference, hover, std::placeholders::_1));
}

bool FollowWaypointPathManeuverServer::validateManeuverParameters(
    const follow_waypoint_path_maneuver_params_t & params
) const {
    if (params.waypoints.empty()) {
        RCLCPP_WARN(node()->get_logger(), "FollowWaypointPath requires at least one waypoint");
        return false;
    }
    if (params.repeat && (
        params.repeat_from_index >= params.waypoints.size() ||
        params.waypoints.size() - params.repeat_from_index < 2
    )) {
        RCLCPP_WARN(node()->get_logger(), "FollowWaypointPath has invalid repeat_from_index");
        return false;
    }
    try {
        const auto waypoints = transformWaypoints(params);
        const double minimum_altitude = configuration_->GetParameter(
            "/control/maneuver_controller/minimum_target_altitude"
        ).as_double();
        const double ground = awareness_handler()->ground_altitude_estimate();
        for (const auto & waypoint : waypoints) {
            if (waypoint.position.z() - ground < minimum_altitude) {
                RCLCPP_WARN(node()->get_logger(), "FollowWaypointPath waypoint is below minimum altitude");
                return false;
            }
        }
        if (
            params.repeat &&
            waypoints[params.repeat_from_index].transition != WaypointTransition::Blend
        ) {
            RCLCPP_WARN(node()->get_logger(), "FollowWaypointPath repeat seam must be blended");
            return false;
        }
        (void)resolveConstraints(params);
    } catch (const std::exception & error) {
        RCLCPP_WARN(node()->get_logger(), "FollowWaypointPath parameters invalid: %s", error.what());
        return false;
    }
    return true;
}

std::vector<WaypointPathWaypoint> FollowWaypointPathManeuverServer::transformWaypoints(
    const follow_waypoint_path_maneuver_params_t & params
) const {
    const std::string world_frame = configuration_->GetParameter("/tf/world_frame_id").as_string();
    const double default_radius = configuration_->GetParameter(
        "/control/maneuver_controller/fly_to_position_blend_radius"
    ).as_double();
    std::vector<WaypointPathWaypoint> result;
    result.reserve(params.waypoints.size());

    for (const auto & waypoint : params.waypoints) {
        geometry_msgs::msg::PointStamped point;
        point.header.frame_id = params.frame_id;
        point.header.stamp = node()->now();
        point.point = waypoint.position;
        const auto transformed_point = params.frame_id == world_frame
            ? point
            : awareness_handler()->tf_buffer()->transform(point, world_frame);

        geometry_msgs::msg::QuaternionStamped orientation;
        orientation.header = point.header;
        orientation.quaternion = quaternionMsgFromQuaternion(
            eulToQuat(euler_angles_t(0.0, 0.0, waypoint.yaw))
        );
        const auto transformed_orientation = params.frame_id == world_frame
            ? orientation
            : awareness_handler()->tf_buffer()->transform(orientation, world_frame);

        if (
            waypoint.transition_mode != iii_drone_interfaces::msg::Waypoint::TRANSITION_STOP &&
            waypoint.transition_mode != iii_drone_interfaces::msg::Waypoint::TRANSITION_BLEND
        ) {
            throw std::invalid_argument("unknown waypoint transition mode");
        }
        result.push_back({
            pointFromPointMsg(transformed_point.point),
            quatToEul(quaternionFromQuaternionMsg(transformed_orientation.quaternion))(2),
            waypoint.transition_mode == iii_drone_interfaces::msg::Waypoint::TRANSITION_BLEND
                ? WaypointTransition::Blend
                : WaypointTransition::Stop,
            waypoint.blend_radius_m > 0.0F ? waypoint.blend_radius_m : default_radius,
            waypoint.speed_limit_m_s,
        });
    }
    return result;
}

WaypointPathConstraints FollowWaypointPathManeuverServer::resolveConstraints(
    const follow_waypoint_path_maneuver_params_t & params
) const {
    WaypointPathConstraints result;
    result.nominal_speed_m_s = params.nominal_speed_m_s > 0.0F
        ? params.nominal_speed_m_s
        : configuration_->GetParameter(
            "/control/trajectory_interpolator/interpolation_max_velocity_m_s"
        ).as_double();
    result.max_acceleration_m_s2 = params.max_acceleration_m_s2 > 0.0F
        ? params.max_acceleration_m_s2
        : configuration_->GetParameter(
            "/control/trajectory_interpolator/interpolation_max_acceleration_m_s2"
        ).as_double();
    result.max_jerk_m_s3 = params.max_jerk_m_s3 > 0.0F
        ? params.max_jerk_m_s3
        : configuration_->GetParameter(
            "/control/trajectory_interpolator/interpolation_max_jerk_m_s3"
        ).as_double();
    if (
        result.nominal_speed_m_s <= 0.0 ||
        result.max_acceleration_m_s2 <= 0.0 ||
        result.max_jerk_m_s3 <= 0.0
    ) {
        throw std::invalid_argument("path dynamics limits must be positive");
    }
    return result;
}
