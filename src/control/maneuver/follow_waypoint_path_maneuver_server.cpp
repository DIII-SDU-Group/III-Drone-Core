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
    const auto params = follow_waypoint_path_maneuver_params_t(maneuver.maneuver_params());
    const State state = awareness_handler()->GetState();
    try {
        const auto waypoints = transformWaypoints(params);
        const auto constraints = resolveConstraints(params);
        WaypointPathPlan new_plan = planner_.plan(
            Reference(state),
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

bool FollowWaypointPathManeuverServer::rebaseExecution(
    const State & stopped_state,
    std::string & reason
) {
    std::lock_guard<std::mutex> lock(plan_mutex_);
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
    const double elapsed_s = (node()->now() - execution_start_time_).seconds();
    active_sample_ = plan_.sample(elapsed_s);
    return active_sample_.reference.CopyWithNewStamp(node()->now());
}

bool FollowWaypointPathManeuverServer::hasSucceeded(Maneuver & maneuver) {
    const auto params = follow_waypoint_path_maneuver_params_t(maneuver.maneuver_params());
    if (params.repeat || has_failed_) {
        return false;
    }
    std::lock_guard<std::mutex> lock(plan_mutex_);
    if ((node()->now() - execution_start_time_).seconds() < plan_.prefixDurationS()) {
        return false;
    }
    const State state = awareness_handler()->GetState();
    const Reference target = plan_.prefix.references.back();
    return
        (state.position() - target.position()).norm() < configuration_->GetParameter(
            "/control/maneuver_controller/reached_position_euclidean_distance_threshold"
        ).as_double() &&
        std::abs(shortestYawError(state.yaw(), target.yaw())) < configuration_->GetParameter(
            "/control/maneuver_controller/reached_yaw_error_threshold"
        ).as_double();
}

bool FollowWaypointPathManeuverServer::hasFailed(Maneuver &) {
    const auto awareness = awareness_handler()->adapter();
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

void FollowWaypointPathManeuverServer::registerReferenceCallbackOnSuccess(const Maneuver &) {
    const auto hover = std::static_pointer_cast<HoverManeuverServer>(
        registered_maneuvers().at(MANEUVER_TYPE_HOVER)
    );
    Reference final_reference;
    {
        std::lock_guard<std::mutex> lock(plan_mutex_);
        final_reference = active_sample_.reference;
    }
    hover->Update(final_reference);
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
