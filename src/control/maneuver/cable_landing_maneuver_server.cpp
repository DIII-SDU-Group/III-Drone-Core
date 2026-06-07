/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/maneuver/cable_landing_maneuver_server.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

using namespace iii_drone::control::maneuver;
using namespace iii_drone::control;
using namespace iii_drone::types;
using namespace iii_drone::math;
using namespace iii_drone::utils;
using namespace iii_drone::adapters;

namespace {

double shortestYawError(double current_yaw, double target_yaw) {
    return std::atan2(std::sin(target_yaw - current_yaw), std::cos(target_yaw - current_yaw));
}

double shortestCableAxisYawError(double current_yaw, double target_yaw) {
    double error = shortestYawError(current_yaw, target_yaw);
    if (error > M_PI_2) {
        error -= M_PI;
    } else if (error < -M_PI_2) {
        error += M_PI;
    }
    return error;
}

double clampMagnitude(double value, double limit) {
    const double abs_limit = std::abs(limit);
    return std::clamp(value, -abs_limit, abs_limit);
}

}  // namespace

/*****************************************************************************/
// Implementation
/*****************************************************************************/

CableLandingManeuverServer::CableLandingManeuverServer(
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

    createServer<iii_drone_interfaces::action::CableLanding>();

}

bool CableLandingManeuverServer::CanExecuteManeuver(
    const Maneuver & maneuver,
    const iii_drone::adapters::CombinedDroneAwarenessAdapter & drone_awareness
) const {

    RCLCPP_DEBUG(node()->get_logger(), "CableLandingManeuverServer::CanExecuteManeuver()");

    cable_landing_maneuver_params_t cable_landing_maneuver_params(maneuver.maneuver_params());

    if (maneuver.maneuver_type() != MANEUVER_TYPE_CABLE_LANDING) {
        RCLCPP_WARN(node()->get_logger(), "CableLandingManeuverServer::CanExecuteManeuver(): Maneuver type is not MANEUVER_TYPE_CABLE_LANDING.");
        return false;
    }

    if (!drone_awareness.armed()) {
        RCLCPP_WARN(node()->get_logger(), "CableLandingManeuverServer::CanExecuteManeuver(): Drone is not armed.");
        return false;
    }

    if (!drone_awareness.offboard()) {
        RCLCPP_WARN(node()->get_logger(), "CableLandingManeuverServer::CanExecuteManeuver(): Drone is not offboard.");
        return false;
    }

    if (!drone_awareness.in_flight()) {
        RCLCPP_WARN(node()->get_logger(), "CableLandingManeuverServer::CanExecuteManeuver(): Drone is not in flight.");
        return false;
    }

    if (!drone_awareness.has_target()) {
        RCLCPP_WARN(node()->get_logger(), "CableLandingManeuverServer::CanExecuteManeuver(): Drone does not have a target.");
        return false;
    }

    if (drone_awareness.target_adapter().target_type() != TARGET_TYPE_CABLE) {
        RCLCPP_WARN(node()->get_logger(), "CableLandingManeuverServer::CanExecuteManeuver(): Target type is not TARGET_TYPE_CABLE.");
        return false;
    }

    if (drone_awareness.target_adapter().target_id() != cable_landing_maneuver_params.target_cable_id) {
        RCLCPP_WARN(
            node()->get_logger(),
            "CableLandingManeuverServer::CanExecuteManeuver(): Requested target id %d differs from active target id %d; accepting active cable target after cable reacquisition.",
            cable_landing_maneuver_params.target_cable_id,
            drone_awareness.target_adapter().target_id()
        );
    }

    if (!drone_awareness.target_position_known()) {
        if (line_pid_has_last_cable_pose_) {
            const vector_t delta_to_locked_target = line_pid_last_cable_pose_world_.position - drone_awareness.state().position();
            const quaternion_t target_quat_world = line_pid_last_cable_pose_world_.orientation;
            const quaternion_t quat_drone_to_target = quatMultiply(
                quatInv(drone_awareness.state().quaternion()),
                target_quat_world
            );
            const double axis_yaw_error = shortestCableAxisYawError(0.0, quatToEul(quat_drone_to_target)(2));
            const double max_initial_distance = configuration_->GetParameter(
                "/control/maneuver_controller/cable_landing_max_initial_distance_error"
            ).as_double();
            const double max_initial_yaw = configuration_->GetParameter(
                "/control/maneuver_controller/cable_landing_max_initial_yaw_error"
            ).as_double();

            if (delta_to_locked_target.norm() <= max_initial_distance && std::abs(axis_yaw_error) <= max_initial_yaw) {
                RCLCPP_WARN(
                    node()->get_logger(),
                    "CableLandingManeuverServer::CanExecuteManeuver(): Target position is not currently known; accepting retry using locked cable pose. distance=%.3f axis_yaw_error=%.3f",
                    delta_to_locked_target.norm(),
                    axis_yaw_error
                );
                return true;
            }

            RCLCPP_WARN(
                node()->get_logger(),
                "CableLandingManeuverServer::CanExecuteManeuver(): Locked cable pose is too far for retry. distance=%.3f threshold=%.3f axis_yaw_error=%.3f threshold=%.3f",
                delta_to_locked_target.norm(),
                max_initial_distance,
                axis_yaw_error,
                max_initial_yaw
            );
        } else {
            RCLCPP_WARN(node()->get_logger(), "CableLandingManeuverServer::CanExecuteManeuver(): Target position is not known.");
        }
        return false;
    }

    CombinedDroneAwarenessHandler::SharedPtr cda_handler = awareness_handler();

    transform_matrix_t current_target_transform = drone_awareness.target_adapter().target_transform();

    vector_t current_target_translation = current_target_transform.block<3, 1>(0, 3);

    geometry_msgs::msg::Vector3Stamped current_target_translation_drone_to_cable;
    current_target_translation_drone_to_cable.vector = vectorMsgFromVector(current_target_translation);
    current_target_translation_drone_to_cable.header.frame_id = drone_awareness.target_adapter().reference_frame_id();

    try {
        current_target_translation_drone_to_cable = cda_handler->tf_buffer()->transform(
            current_target_translation_drone_to_cable,
            configuration_->GetParameter("/tf/drone_frame_id").as_string()
        );
    } catch (const tf2::TransformException &e) {
        RCLCPP_WARN(node()->get_logger(), "CableLandingManeuverServer::CanExecuteManeuver(): Could not transform translation.");
        return false;
    }

    current_target_translation = vectorFromVectorMsg(current_target_translation_drone_to_cable.vector);

    if (current_target_translation(0) != 0 || current_target_translation(1) != 0) {
        RCLCPP_WARN(node()->get_logger(), "CableLandingManeuverServer::CanExecuteManeuver(): Target translation is not 0,0,0.");
        return false;
    }

    if (current_target_translation(2) < configuration_->GetParameter("/control/maneuver_controller/cable_landing_min_z_distance").as_double()) {
        RCLCPP_WARN(node()->get_logger(), "CableLandingManeuverServer::CanExecuteManeuver(): Target translation is less than cable_landing_min_z_distance.");
        return false;
    }

    if (current_target_translation(2) > configuration_->GetParameter("/control/maneuver_controller/cable_landing_max_z_distance").as_double()) {
        RCLCPP_WARN(node()->get_logger(), "CableLandingManeuverServer::CanExecuteManeuver(): Target translation is greater than cable_landing_max_z_distance.");
        return false;
    }

    quaternion_t current_target_quat = matToQuat(current_target_transform.block<3, 3>(0, 0));

    geometry_msgs::msg::QuaternionStamped current_target_quat_gripper_to_cable_msg;
    current_target_quat_gripper_to_cable_msg.quaternion = quaternionMsgFromQuaternion(current_target_quat);
    current_target_quat_gripper_to_cable_msg.header.frame_id = drone_awareness.target_adapter().reference_frame_id();

    try {
        current_target_quat_gripper_to_cable_msg = cda_handler->tf_buffer()->transform(
            current_target_quat_gripper_to_cable_msg,
            configuration_->GetParameter("/tf/cable_gripper_frame_id").as_string()
        );
    } catch (const tf2::TransformException &e) {
        RCLCPP_WARN(node()->get_logger(), "CableLandingManeuverServer::CanExecuteManeuver(): Could not transform quaternion.");
        return false;
    }

    // quaternion_t current_target_quat_drone_to_cable = quaternionFromQuaternionMsg(current_target_quat_drone_to_cable_msg.quaternion);

    // quaternion_t target_quat_drone_to_cable = quaternionFromQuaternionMsg(target_quat_drone_to_cable.quaternion);

    // quaternion_t target_transform_quat = matToQuat(target_transform.block<3, 3>(0, 0));

    // if (target_quat_drone_to_cable_quat != target_transform_quat) {
    //     return false;
    // }

    quaternion_t current_target_quat_gripper_to_cable = quaternionFromQuaternionMsg(current_target_quat_gripper_to_cable_msg.quaternion);

    // Check if current_target_quat_gripper_to_cable is 1,0,0,0 with a small tolerance:
    if ((current_target_quat_gripper_to_cable - quaternion_t(1, 0, 0, 0)).norm() > 1e3) {
        RCLCPP_WARN(node()->get_logger(), "CableLandingManeuverServer::CanExecuteManeuver(): Target quaternion is not 1,0,0,0.");
        return false;
    }

    vector_t pos_world_to_drone = drone_awareness.state().position();

    transform_matrix_t current_target_transform_world_to_drone;
    try {
        current_target_transform_world_to_drone = cda_handler->ComputeTargetTransform(drone_awareness.target_adapter());
    } catch (const std::runtime_error &e) {
        RCLCPP_WARN(node()->get_logger(), "CableLandingManeuverServer::CanExecuteManeuver(): Could not compute target transform.");
        return false;
    }

    vector_t current_target_pos_world_to_drone = current_target_transform_world_to_drone.block<3, 1>(0, 3);

    vector_t p_drone_to_target = current_target_pos_world_to_drone - pos_world_to_drone;

    if (p_drone_to_target.norm() > configuration_->GetParameter("/control/maneuver_controller/cable_landing_max_initial_distance_error").as_double()) {
        RCLCPP_WARN(node()->get_logger(), "CableLandingManeuverServer::CanExecuteManeuver(): Initial distance error is too large.");
        return false;
    }

    // geometry_msgs::msg::QuaternionStamped current_world_to_target_quat;
    // current_world_to_target_quat.quaternion = quaternionMsgFromQuaternion(current_target_pose.orientation);
    // current_world_to_target_quat.header.frame_id = drone_awareness.target_adapter.reference_frame_id();

    // geometry_msgs::msg::QuaternionStamped current_world_to_target_quat = cda_handler->tf_buffer()->transform(
    //     current_target_pose.orientation,
    //     configuration_->GetParameter("/tf/drone_frame_id").as_string()
    // );

    quaternion_t current_target_quat_world_to_drone = matToQuat(current_target_transform_world_to_drone.block<3, 3>(0, 0));
    quaternion_t quat_world_to_drone = drone_awareness.state().quaternion();

    quaternion_t quat_drone_to_target = quatMultiply(
        quatInv(quat_world_to_drone),
        current_target_quat_world_to_drone
    );

    euler_angles_t eul_drone_to_target = quatToEul(quat_drone_to_target);

    const double initial_axis_yaw_error = shortestCableAxisYawError(0.0, eul_drone_to_target(2));
    if (abs(initial_axis_yaw_error) > configuration_->GetParameter("/control/maneuver_controller/cable_landing_max_initial_yaw_error").as_double()) {
        RCLCPP_WARN(
            node()->get_logger(),
            "CableLandingManeuverServer::CanExecuteManeuver(): Initial yaw axis error is too large. raw_yaw_error=%.4f axis_yaw_error=%.4f threshold=%.4f",
            eul_drone_to_target(2),
            initial_axis_yaw_error,
            configuration_->GetParameter("/control/maneuver_controller/cable_landing_max_initial_yaw_error").as_double()
        );
        return false;
    }

    RCLCPP_DEBUG(node()->get_logger(), "CableLandingManeuverServer::CanExecuteManeuver(): Cable landing maneuver can be executed.");

    return true;

}

iii_drone::adapters::CombinedDroneAwarenessAdapter CableLandingManeuverServer::ExpectedAwarenessAfterExecution(const Maneuver & maneuver) {

    cable_landing_maneuver_params_t cable_landing_maneuver_params(maneuver.maneuver_params());
    int effective_target_cable_id = cable_landing_maneuver_params.target_cable_id;
    const auto current_awareness = awareness_handler()->adapter();

    if (
        current_awareness.has_target() &&
        current_awareness.target_adapter().target_type() == TARGET_TYPE_CABLE &&
        current_awareness.target_adapter().target_id() != cable_landing_maneuver_params.target_cable_id
    ) {
        effective_target_cable_id = current_awareness.target_adapter().target_id();
        RCLCPP_WARN(
            node()->get_logger(),
            "CableLandingManeuverServer::ExpectedAwarenessAfterExecution(): using active target id %d instead of requested id %d after cable reacquisition.",
            effective_target_cable_id,
            cable_landing_maneuver_params.target_cable_id
        );
    }

    TargetAdapter target_adapter = TargetAdapter(
        TARGET_TYPE_CABLE,
        effective_target_cable_id,
        configuration_->GetParameter("/tf/cable_gripper_frame_id").as_string(),
        transform_matrix_t::Identity()
    );

    transform_matrix_t target_transform = awareness_handler()->ComputeTargetTransform(target_adapter);

    State state(
        target_transform.block<3, 1>(0, 3),
        vector_t::Zero(),
        matToQuat(target_transform.block<3, 3>(0, 0)),
        vector_t::Zero()
    );

    iii_drone::adapters::CombinedDroneAwarenessAdapter awareness_after;

    awareness_after.armed() = true;
    awareness_after.offboard() = true;
    awareness_after.target_position_known() = true;
    awareness_after.drone_location() = DRONE_LOCATION_ON_CABLE;
    awareness_after.target_adapter() = target_adapter;
    awareness_after.state() = state;

    return awareness_after;

}

maneuver_type_t CableLandingManeuverServer::maneuver_type() const {
    return MANEUVER_TYPE_CABLE_LANDING;
}

void CableLandingManeuverServer::startExecution(Maneuver & maneuver) {

    RCLCPP_INFO(
        node()->get_logger(),
        "CableLandingManeuverServer::startExecution(): Starting execution of maneuver."
    );

    auto cda_handler = awareness_handler();

    cable_landing_maneuver_params_t cable_landing_maneuver_params(maneuver.maneuver_params());
    int effective_target_cable_id = cable_landing_maneuver_params.target_cable_id;
    const auto current_awareness = cda_handler->adapter();

    if (
        current_awareness.has_target() &&
        current_awareness.target_adapter().target_type() == TARGET_TYPE_CABLE &&
        current_awareness.target_adapter().target_id() != cable_landing_maneuver_params.target_cable_id
    ) {
        effective_target_cable_id = current_awareness.target_adapter().target_id();
        RCLCPP_WARN(
            node()->get_logger(),
            "CableLandingManeuverServer::startExecution(): using active target id %d instead of requested id %d after cable reacquisition.",
            effective_target_cable_id,
            cable_landing_maneuver_params.target_cable_id
        );
    }

    target_adapter_ = TargetAdapter(
        TARGET_TYPE_CABLE,
        effective_target_cable_id,
        configuration_->GetParameter("/tf/cable_gripper_frame_id").as_string(),
        transform_matrix_t::Identity()
    );

    RCLCPP_DEBUG(
        node()->get_logger(),
        "CableLandingManeuverServer::startExecution(): target_cable_id=%d reference_frame=%s target_transform=identity",
        target_adapter_->target_id(),
        target_adapter_->reference_frame_id().c_str()
    );

    if (trajectory_generator_client_->busy()) {

        std::string error_message = "CableLandingManeuverServer::startExecution(): Trajectory generator client is busy, cannot start execution of maneuver.";

        RCLCPP_FATAL(node()->get_logger(), error_message.c_str());

        throw std::runtime_error(error_message);

    }

    const bool had_locked_pose = line_pid_has_last_cable_pose_;
    const pose_t previous_locked_pose = line_pid_last_cable_pose_world_;

    first_iteration_ = true;
    has_failed_ = false;
    line_pid_initialized_ = false;
    line_pid_target_lock_initialized_ = false;
    line_pid_has_last_cable_pose_ = false;
    line_pid_along_pid_ = PidState();
    line_pid_cross_pid_ = PidState();
    line_pid_yaw_pid_ = PidState();
    gripper_v_gate_violation_active_ = false;

    if (had_locked_pose) {
        line_pid_last_cable_pose_world_ = previous_locked_pose;
        line_pid_has_last_cable_pose_ = true;
        RCLCPP_WARN(
            node()->get_logger(),
            "CableLandingManeuverServer::startExecution(): Preserving locked cable pose for retry: [%.3f, %.3f, %.3f]",
            previous_locked_pose.position(0),
            previous_locked_pose.position(1),
            previous_locked_pose.position(2)
        );
    }

    cda_handler->SetTarget(target_adapter_);

}

bool CableLandingManeuverServer::canCancel() {
    State state = awareness_handler()->GetState();
    Reference target_reference = getUpdatedTargetReference(state);

    return !isWithinSafetyZone(
        state,
        target_reference
    );
}

Reference CableLandingManeuverServer::computeReference(const State & state) {

    const std::string controller_type = configuration_->GetParameter(
        "/control/maneuver_controller/cable_landing_controller_type"
    ).as_string();

    if (controller_type == "line_pid") {
        return computeLinePidReference(state);
    }

    if (controller_type != "mpc" && controller_type != "trajectory_generator") {
        RCLCPP_WARN_THROTTLE(
            node()->get_logger(),
            *node()->get_clock(),
            1000,
            "CableLandingManeuverServer::computeReference(): Unknown controller_type='%s', falling back to trajectory generator.",
            controller_type.c_str()
        );
    }

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
            trajectory_mode_t::cable_landing,
            configuration_->GetParameter("/control/maneuver_controller/cable_landing_use_mpc").as_bool()
        );

        Reference truncated_ref = truncateReferenceWithinSafetyZone(
            ref,
            state,
            target_reference
        );

        return truncated_ref;

    } catch (const std::runtime_error &e) {

        RCLCPP_WARN(
            node()->get_logger(),
            "CableLandingManeuverServer::computeReference(): Could not compute reference."
        );

        has_failed_ = true;

        return Reference(
            state,
            true,
            true
        );

    }
    
}

Reference CableLandingManeuverServer::computeLinePidReference(const State & state) {

    auto cda_handler = awareness_handler();

    pose_t cable_pose;
    if (!getStableCablePose(cable_pose)) {
        has_failed_ = true;
        return Reference(state, true, true);
    }

    const Reference target_reference = getUpdatedTargetReference(state, true);
    if (has_failed_) {
        return Reference(state, true, true);
    }
    first_iteration_ = false;

    point_t gripper_position_world;
    if (!getGripperPositionInWorld(gripper_position_world)) {
        has_failed_ = true;
        return Reference(state, true, true);
    }

    const double raw_cable_pose_yaw = quatToEul(cable_pose.orientation)(2);
    const double target_yaw = target_reference.yaw();
    vector_t cable_direction_world(
        std::cos(target_yaw),
        std::sin(target_yaw),
        0.0
    );
    if (cable_direction_world.norm() < 1e-6) {
        cable_direction_world = vector_t::UnitX();
    } else {
        cable_direction_world.normalize();
    }

    if (line_pid_initialized_) {
        if (cable_direction_world.dot(line_pid_cable_direction_world_) < 0.0) {
            cable_direction_world = -cable_direction_world;
        }
    } else if (std::abs(shortestYawError(state.yaw(), std::atan2(cable_direction_world(1), cable_direction_world(0)))) > M_PI_2) {
        cable_direction_world = -cable_direction_world;
    }

    line_pid_cable_direction_world_ = cable_direction_world;

    const vector_t world_up = vector_t::UnitZ();
    vector_t cable_cross_world = world_up.cross(cable_direction_world);
    if (cable_cross_world.norm() < 1e-6) {
        cable_cross_world = vector_t::UnitY();
    } else {
        cable_cross_world.normalize();
    }

    vector_t target_point_gripper;
    if (!getTargetPointInCableGripperFrame(target_point_gripper)) {
        has_failed_ = true;
        return Reference(state, true, true);
    }

    vector_t gripper_y_axis_world = cable_cross_world;
    try {
        const auto world_T_gripper_msg = awareness_handler()->tf_buffer()->lookupTransform(
            configuration_->GetParameter("/tf/world_frame_id").as_string(),
            configuration_->GetParameter("/tf/cable_gripper_frame_id").as_string(),
            tf2::TimePointZero
        );
        const transform_matrix_t world_T_gripper = transformMatrixFromTransformMsg(world_T_gripper_msg.transform);
        gripper_y_axis_world = world_T_gripper.block<3, 3>(0, 0).col(1);
        if (gripper_y_axis_world.norm() < 1e-6) {
            gripper_y_axis_world = cable_cross_world;
        } else {
            gripper_y_axis_world.normalize();
        }
    } catch (const tf2::TransformException & e) {
        RCLCPP_WARN(
            node()->get_logger(),
            "CableLandingManeuverServer::computeLinePidReference(): Could not transform gripper axis to world: %s",
            e.what()
        );
        has_failed_ = true;
        return Reference(state, true, true);
    }

    if (!line_pid_initialized_) {
        const vector_t cable_to_gripper = gripper_position_world - cable_pose.position;
        line_pid_anchor_point_world_ = cable_pose.position + cable_to_gripper.dot(cable_direction_world) * cable_direction_world;
        line_pid_position_reference_world_ = state.position();
        line_pid_last_stamp_ = state.stamp();
        line_pid_last_cable_pose_world_ = cable_pose;
        line_pid_has_last_cable_pose_ = true;
        line_pid_target_lock_initialized_ = true;
        line_pid_initialized_ = true;

        RCLCPP_INFO(
            node()->get_logger(),
            "CableLandingManeuverServer::computeLinePidReference(): Initialized line PID anchor=[%.3f, %.3f, %.3f] gripper=[%.3f, %.3f, %.3f] cable=[%.3f, %.3f, %.3f] cable_yaw=%.3f",
            line_pid_anchor_point_world_(0),
            line_pid_anchor_point_world_(1),
            line_pid_anchor_point_world_(2),
            gripper_position_world(0),
            gripper_position_world(1),
            gripper_position_world(2),
            cable_pose.position(0),
            cable_pose.position(1),
            cable_pose.position(2),
            std::atan2(cable_direction_world(1), cable_direction_world(0))
        );

        RCLCPP_DEBUG_THROTTLE(
            node()->get_logger(),
            *node()->get_clock(),
            1000,
            "CableLandingManeuverServer::computeLinePidReference(): Cable yaw source comparison: target_reference_yaw=%.3f raw_cable_pose_yaw=%.3f",
            target_yaw,
            raw_cable_pose_yaw
        );
    }

    double dt = (state.stamp() - line_pid_last_stamp_).seconds();
    const double default_dt = configuration_->GetParameter(
        "/control/maneuver_controller/maneuver_execution_period_ms"
    ).as_int() / 1000.0;
    const double max_dt = configuration_->GetParameter(
        "/control/maneuver_controller/cable_landing_line_pid_max_dt_s"
    ).as_double();
    if (!std::isfinite(dt) || dt <= 0.0) {
        dt = default_dt;
    }
    dt = std::clamp(dt, 1e-3, max_dt);
    line_pid_last_stamp_ = state.stamp();

    const double along_error = (line_pid_anchor_point_world_ - gripper_position_world).dot(cable_direction_world);
    const double gripper_v_gate_center_y = configuration_->GetParameter(
        "/control/maneuver_controller/cable_landing_gripper_v_gate_center_y"
    ).as_double();
    const double cross_error = target_point_gripper(1) - gripper_v_gate_center_y;

    const double along_velocity = computePidOutput(
        line_pid_along_pid_,
        along_error,
        dt,
        configuration_->GetParameter("/control/maneuver_controller/cable_landing_line_pid_along_kp").as_double(),
        configuration_->GetParameter("/control/maneuver_controller/cable_landing_line_pid_along_ki").as_double(),
        configuration_->GetParameter("/control/maneuver_controller/cable_landing_line_pid_along_kd").as_double(),
        configuration_->GetParameter("/control/maneuver_controller/cable_landing_line_pid_along_integral_limit").as_double(),
        configuration_->GetParameter("/control/maneuver_controller/cable_landing_line_pid_max_along_velocity").as_double()
    );

    const double cross_velocity = computePidOutput(
        line_pid_cross_pid_,
        cross_error,
        dt,
        configuration_->GetParameter("/control/maneuver_controller/cable_landing_line_pid_cross_kp").as_double(),
        configuration_->GetParameter("/control/maneuver_controller/cable_landing_line_pid_cross_ki").as_double(),
        configuration_->GetParameter("/control/maneuver_controller/cable_landing_line_pid_cross_kd").as_double(),
        configuration_->GetParameter("/control/maneuver_controller/cable_landing_line_pid_cross_integral_limit").as_double(),
        configuration_->GetParameter("/control/maneuver_controller/cable_landing_line_pid_max_cross_velocity").as_double()
    );

    const double continuous_target_yaw = std::atan2(cable_direction_world(1), cable_direction_world(0));
    const double yaw_error = shortestYawError(state.yaw(), continuous_target_yaw);
    const double yaw_rate = computePidOutput(
        line_pid_yaw_pid_,
        yaw_error,
        dt,
        configuration_->GetParameter("/control/maneuver_controller/cable_landing_line_pid_yaw_kp").as_double(),
        configuration_->GetParameter("/control/maneuver_controller/cable_landing_line_pid_yaw_ki").as_double(),
        configuration_->GetParameter("/control/maneuver_controller/cable_landing_line_pid_yaw_kd").as_double(),
        configuration_->GetParameter("/control/maneuver_controller/cable_landing_line_pid_yaw_integral_limit").as_double(),
        configuration_->GetParameter("/control/maneuver_controller/cable_landing_line_pid_max_yaw_rate").as_double()
    );

    const double configured_ascent_velocity = configuration_->GetParameter(
        "/control/maneuver_controller/cable_landing_line_pid_ascent_velocity"
    ).as_double();
    const double ascent_cross_error_threshold = std::max(
        configuration_->GetParameter("/control/maneuver_controller/cable_landing_gripper_v_gate_half_width_at_reference_z").as_double(),
        0.15
    );
    const double ascent_velocity = std::abs(cross_error) <= ascent_cross_error_threshold
        ? configured_ascent_velocity
        : 0.0;
    if (ascent_velocity == 0.0) {
        RCLCPP_WARN_THROTTLE(
            node()->get_logger(),
            *node()->get_clock(),
            1000,
            "CableLandingManeuverServer::computeLinePidReference(): Holding ascent until cross error is centered. cross_error=%.4f threshold=%.4f configured_ascent_velocity=%.4f",
            cross_error,
            ascent_cross_error_threshold,
            configured_ascent_velocity
        );
    }

    vector_t velocity_world =
        static_cast<float>(along_velocity) * cable_direction_world +
        static_cast<float>(cross_velocity) * gripper_y_axis_world +
        static_cast<float>(ascent_velocity) * world_up;

    line_pid_position_reference_world_ += static_cast<float>(dt) * velocity_world;

    point_t position_reference = line_pid_position_reference_world_;
    position_reference(2) = std::numeric_limits<float>::quiet_NaN();

    vector_t velocity_reference = vector_t::Constant(std::numeric_limits<float>::quiet_NaN());
    velocity_reference(2) = static_cast<float>(ascent_velocity);

    vector_t acceleration_reference = vector_t::Constant(std::numeric_limits<float>::quiet_NaN());

    RCLCPP_DEBUG_THROTTLE(
        node()->get_logger(),
        *node()->get_clock(),
        1000,
        "CableLandingManeuverServer::computeLinePidReference(): dt=%.3f errors[along=%.4f cross=%.4f yaw=%.4f] velocities[along=%.4f cross=%.4f z=%.4f yaw=%.4f] position_ref=[%.4f, %.4f, nan] cable=[%.4f, %.4f, %.4f] gripper=[%.4f, %.4f, %.4f] target_gripper=[%.4f, %.4f, %.4f] gripper_y_axis_world=[%.4f, %.4f, %.4f]",
        dt,
        along_error,
        cross_error,
        yaw_error,
        along_velocity,
        cross_velocity,
        ascent_velocity,
        yaw_rate,
        position_reference(0),
        position_reference(1),
        cable_pose.position(0),
        cable_pose.position(1),
        cable_pose.position(2),
        gripper_position_world(0),
        gripper_position_world(1),
        gripper_position_world(2),
        target_point_gripper(0),
        target_point_gripper(1),
        target_point_gripper(2),
        gripper_y_axis_world(0),
        gripper_y_axis_world(1),
        gripper_y_axis_world(2)
    );

    return Reference(
        position_reference,
        continuous_target_yaw,
        velocity_reference,
        yaw_rate,
        acceleration_reference,
        std::numeric_limits<double>::quiet_NaN(),
        state.stamp()
    );

}

bool CableLandingManeuverServer::getGripperPositionInWorld(
    iii_drone::types::point_t & gripper_position_world
) const {

    try {
        const auto world_T_gripper = awareness_handler()->tf_buffer()->lookupTransform(
            configuration_->GetParameter("/tf/world_frame_id").as_string(),
            configuration_->GetParameter("/tf/cable_gripper_frame_id").as_string(),
            tf2::TimePointZero
        );

        gripper_position_world = vectorFromTransformMsg(world_T_gripper.transform);
        return true;
    } catch (const tf2::TransformException & e) {
        RCLCPP_WARN(
            node()->get_logger(),
            "CableLandingManeuverServer::getGripperPositionInWorld(): Could not transform gripper to world: %s",
            e.what()
        );
        return false;
    }

}

bool CableLandingManeuverServer::isCablePoseConsistentWithLock(
    const iii_drone::types::point_t & cable_position_world,
    double & orthogonal_distance
) const {

    if (!line_pid_target_lock_initialized_) {
        orthogonal_distance = 0.0;
        return true;
    }

    vector_t cable_direction_world = line_pid_cable_direction_world_;
    if (cable_direction_world.norm() < 1e-6) {
        cable_direction_world = vector_t::UnitX();
    } else {
        cable_direction_world.normalize();
    }

    const vector_t delta = cable_position_world - line_pid_anchor_point_world_;
    const vector_t orthogonal_delta = delta - delta.dot(cable_direction_world) * cable_direction_world;
    orthogonal_distance = orthogonal_delta.norm();

    double max_orthogonal_distance = 0.20;
    try {
        max_orthogonal_distance = configuration_->GetParameter(
            "/control/maneuver_controller/cable_landing_target_lock_max_orthogonal_distance"
        ).as_double();
    } catch (const std::runtime_error & e) {
        RCLCPP_WARN_THROTTLE(
            node()->get_logger(),
            *node()->get_clock(),
            5000,
            "CableLandingManeuverServer::isCablePoseConsistentWithLock(): Missing target lock parameter, using default %.3f m: %s",
            max_orthogonal_distance,
            e.what()
        );
    }

    return orthogonal_distance <= max_orthogonal_distance;

}

bool CableLandingManeuverServer::getStableCablePose(
    iii_drone::types::pose_t & cable_pose_world
) {

    auto cda_handler = awareness_handler();
    double orthogonal_distance = 0.0;

    try {
        pose_t candidate_pose = cda_handler->GetPoseOfTarget(target_adapter_);
        if (isCablePoseConsistentWithLock(candidate_pose.position, orthogonal_distance)) {
            cable_pose_world = candidate_pose;
            line_pid_last_cable_pose_world_ = candidate_pose;
            line_pid_has_last_cable_pose_ = true;
            return true;
        }

        RCLCPP_WARN_THROTTLE(
            node()->get_logger(),
            *node()->get_clock(),
            1000,
            "CableLandingManeuverServer::getStableCablePose(): target id %d jumped %.3f m from locked conductor; attempting physical-line reacquisition.",
            target_adapter_->target_id(),
            orthogonal_distance
        );
    } catch (const std::runtime_error & e) {
        RCLCPP_WARN_THROTTLE(
            node()->get_logger(),
            *node()->get_clock(),
            1000,
            "CableLandingManeuverServer::getStableCablePose(): Target id %d is not visible: %s",
            target_adapter_->target_id(),
            e.what()
        );
    }

    if (line_pid_target_lock_initialized_) {
        try {
            PowerlineAdapter powerline_adapter = cda_handler->GetPowerlineAdapter();
            powerline_adapter.Transform(
                configuration_->GetParameter("/tf/world_frame_id").as_string(),
                cda_handler->tf_buffer()
            );

            const auto lines = powerline_adapter.single_line_adapters();
            double best_distance = std::numeric_limits<double>::infinity();
            SingleLineAdapter best_line;
            bool has_best_line = false;

            for (const auto & line : lines) {
                double line_distance = 0.0;
                if (!isCablePoseConsistentWithLock(line.position(), line_distance)) {
                    if (line_distance < best_distance) {
                        best_distance = line_distance;
                    }
                    continue;
                }
                if (!has_best_line || line_distance < best_distance) {
                    best_line = line;
                    best_distance = line_distance;
                    has_best_line = true;
                }
            }

            if (has_best_line) {
                const TargetAdapter current_target = target_adapter_.Load();
                if (best_line.id() != current_target.target_id()) {
                    TargetAdapter reacquired_target(
                        TARGET_TYPE_CABLE,
                        best_line.id(),
                        current_target.reference_frame_id(),
                        current_target.target_transform()
                    );
                    target_adapter_ = reacquired_target;
                    cda_handler->SetTarget(reacquired_target);

                    RCLCPP_WARN(
                        node()->get_logger(),
                        "CableLandingManeuverServer::getStableCablePose(): Reacquired locked physical conductor as detected line id %d (previous id %d, orthogonal distance %.3f m).",
                        best_line.id(),
                        current_target.target_id(),
                        best_distance
                    );
                }

                cable_pose_world = poseFromPose(best_line.position(), best_line.quaternion());
                line_pid_last_cable_pose_world_ = cable_pose_world;
                line_pid_has_last_cable_pose_ = true;
                return true;
            }

            RCLCPP_WARN_THROTTLE(
                node()->get_logger(),
                *node()->get_clock(),
                1000,
                "CableLandingManeuverServer::getStableCablePose(): No detected line matched locked conductor; best orthogonal distance %.3f m.",
                best_distance
            );
        } catch (const std::runtime_error & e) {
            RCLCPP_WARN_THROTTLE(
                node()->get_logger(),
                *node()->get_clock(),
                1000,
                "CableLandingManeuverServer::getStableCablePose(): Could not inspect current powerline for target reacquisition: %s",
                e.what()
            );
        }
    }

    if (line_pid_has_last_cable_pose_) {
        cable_pose_world = line_pid_last_cable_pose_world_;
        RCLCPP_WARN_THROTTLE(
            node()->get_logger(),
            *node()->get_clock(),
            1000,
            "CableLandingManeuverServer::getStableCablePose(): Continuing with last locked cable pose while perception target is unavailable or inconsistent."
        );
        return true;
    }

    return false;

}

double CableLandingManeuverServer::computePidOutput(
    PidState & pid_state,
    double error,
    double dt,
    double kp,
    double ki,
    double kd,
    double integral_limit,
    double output_limit
) const {

    pid_state.integral += error * dt;
    pid_state.integral = clampMagnitude(pid_state.integral, integral_limit);

    double derivative = 0.0;
    if (pid_state.has_previous_error && dt > 0.0) {
        derivative = (error - pid_state.previous_error) / dt;
    }

    pid_state.previous_error = error;
    pid_state.has_previous_error = true;

    return clampMagnitude(kp * error + ki * pid_state.integral + kd * derivative, output_limit);

}

bool CableLandingManeuverServer::hasSucceeded(Maneuver &) {

    auto cda_handler = awareness_handler();

    if (configuration_->GetParameter("/control/maneuver_controller/use_gripper_status_condition").as_bool()) {

        if (!cda_handler->gripper_open()) {
            RCLCPP_INFO(
                node()->get_logger(),
                "CableLandingManeuverServer::hasSucceeded(): Gripper is closed, succeeded."
            );
            return true;
        }

        return false;

    }

    State state = cda_handler->GetState();

    Reference target_reference = getUpdatedTargetReference(state);

    const double position_distance = (state.position() - target_reference.position()).norm();
    const double yaw_error = std::abs(shortestCableAxisYawError(state.yaw(), target_reference.yaw()));
    const double distance = std::hypot(position_distance, yaw_error);

    if(distance < configuration_->GetParameter("/control/maneuver_controller/cable_landing_reached_position_euclidean_distance_threshold").as_double()) {
        RCLCPP_INFO(
            node()->get_logger(),
            "CableLandingManeuverServer::hasSucceeded(): Reached position Euclidean distance threshold, succeeded."
        );
        return true;
    }

    return false;

}

bool CableLandingManeuverServer::hasFailed(Maneuver &) {

    auto cda_handler = awareness_handler();

    Reference target_reference = getUpdatedTargetReference(cda_handler->GetState());

    if (cda_handler->on_cable()) {
        if (cda_handler->on_cable_id() != target_adapter_->target_id()) {
            RCLCPP_WARN(
                node()->get_logger(),
                "CableLandingManeuverServer::hasFailed(): Drone is on cable but the on cable id is wrong."
            );
            return true;
        }

        return false;
    }

    // return !cda_handler->in_flight() 
    //     || !cda_handler->offboard()
    //     || !cda_handler->armed()
    //     || cda_handler->target_adapter() != target_adapter_
    //     || !cda_handler->target_position_known()
    //     || has_failed_
    //     || !isWithinSafetyMargins(
    //         cda_handler->GetState(),
    //         target_reference
    //     );

    if (!cda_handler->in_flight()) {
        RCLCPP_WARN(
            node()->get_logger(),
            "CableLandingManeuverServer::hasFailed(): Drone is not in flight and not on cable."
        );
        return true;
    }

    if (!cda_handler->offboard()) {
        RCLCPP_WARN(
            node()->get_logger(),
            "CableLandingManeuverServer::hasFailed(): Drone is not offboard."
        );
        return true;
    }

    if (!cda_handler->armed()) {
        RCLCPP_WARN(
            node()->get_logger(),
            "CableLandingManeuverServer::hasFailed(): Drone is not armed."
        );
        return true;
    }

    if (cda_handler->target_adapter() != target_adapter_) {
        RCLCPP_WARN(
            node()->get_logger(),
            "CableLandingManeuverServer::hasFailed(): Target adapter does not match."
        );
        return true;
    }

    if (!cda_handler->target_position_known() && !line_pid_has_last_cable_pose_) {
        RCLCPP_WARN(
            node()->get_logger(),
            "CableLandingManeuverServer::hasFailed(): Target position is not known."
        );
        return true;
    }

    if (!cda_handler->target_position_known() && line_pid_has_last_cable_pose_) {
        RCLCPP_WARN_THROTTLE(
            node()->get_logger(),
            *node()->get_clock(),
            1000,
            "CableLandingManeuverServer::hasFailed(): Target position is not currently known; continuing with locked cable pose."
        );
    }

    if (has_failed_) {
        RCLCPP_WARN(
            node()->get_logger(),
            "CableLandingManeuverServer::hasFailed(): Has failed flag is set."
        );
        return true;
    }

    if (!isWithinSafetyMargins(
        cda_handler->GetState(),
        target_reference
    )) {
        RCLCPP_WARN(
            node()->get_logger(),
            "CableLandingManeuverServer::hasFailed(): Drone is not within safety margins."
        );
        return true;
    }

    return false;

}

std::shared_ptr<void> CableLandingManeuverServer::getFeedback(Maneuver &) {

    auto feedback = std::make_shared<iii_drone_interfaces::action::CableLanding::Feedback>();

    ReferenceTrajectory reference_trajectory;

    try {
        reference_trajectory = trajectory_generator_client_->GetReferenceTrajectory();
    } catch (const std::runtime_error &e) {
        return std::static_pointer_cast<void>(feedback);
    }

    ReferenceTrajectoryAdapter reference_trajectory_adapter(reference_trajectory);

    State state = awareness_handler()->GetState();

    Reference target_reference = getUpdatedTargetReference(state);

    std::string world_frame_id = configuration_->GetParameter("/tf/world_frame_id").as_string();

    feedback->planned_path = reference_trajectory_adapter.ToPathMsg(world_frame_id);
    feedback->vehicle_pose = StateAdapter(awareness_handler()->GetState()).ToPoseStampedMsg(world_frame_id);
    feedback->distance_vehicle_to_cable = (state.position() - target_reference.position()).norm();

    return std::static_pointer_cast<void>(feedback);

}

void CableLandingManeuverServer::publishResultAndFinalize(
    Maneuver & maneuver,
    maneuver_result_type_t maneuver_result_type
) {

    auto result = std::make_shared<iii_drone_interfaces::action::CableLanding::Result>();
    auto goal_handle = std::static_pointer_cast<GoalHandleCableLanding>(maneuver.goal_handle());

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
            awareness_handler()->ClearTarget();
            break;
    }

}

void CableLandingManeuverServer::registerReferenceCallbackOnSuccess(const Maneuver &) {

    auto registered_hover_on_cable_maneuver = registered_maneuvers().find(MANEUVER_TYPE_HOVER_ON_CABLE);

    std::shared_ptr<HoverOnCableManeuverServer> hover_on_cable_maneuver_server = std::static_pointer_cast<HoverOnCableManeuverServer>(registered_hover_on_cable_maneuver->second);

    hover_on_cable_maneuver_server->Update(
        target_adapter_->target_id(),
        configuration_->GetParameter("/control/maneuver_controller/hover_on_cable_default_z_velocity").as_double(),
        configuration_->GetParameter("/control/maneuver_controller/hover_on_cable_default_yaw_rate").as_double()
    );

    registerCallback(
        std::bind(
            &HoverOnCableManeuverServer::GetReference,
            hover_on_cable_maneuver_server,
            std::placeholders::_1
        )
    );

}

Reference CableLandingManeuverServer::getUpdatedTargetReference(
    const iii_drone::control::State & state,
    bool compute
) {

    static Atomic<Reference> target_reference;

    if (compute || first_iteration_) {

        auto cda_handler = awareness_handler();

        transform_matrix_t target_transform;

        auto targetTransformFromCablePose = [this, cda_handler](
            const pose_t & cable_pose_world
        ) -> transform_matrix_t {
            const TargetAdapter current_target = target_adapter_.Load();
            const transform_matrix_t w_T_c = createTransformMatrix(
                cable_pose_world.position,
                cable_pose_world.orientation
            );
            const transform_matrix_t ref_T_c = current_target.target_transform();
            const transform_matrix_t w_T_ref = w_T_c * ref_T_c.inverse();

            try {
                const auto ref_T_drone_msg = cda_handler->tf_buffer()->lookupTransform(
                    current_target.reference_frame_id(),
                    configuration_->GetParameter("/tf/drone_frame_id").as_string(),
                    tf2::TimePointZero
                );
                const transform_matrix_t ref_T_drone = transformMatrixFromTransformMsg(
                    ref_T_drone_msg.transform
                );
                return w_T_ref * ref_T_drone;
            } catch (const tf2::TransformException & e) {
                throw std::runtime_error(
                    "CableLandingManeuverServer::getUpdatedTargetReference(): "
                    "could not transform cached cable pose through target reference frame: " +
                    std::string(e.what())
                );
            }
        };

        try {

            try {
                target_transform = cda_handler->ComputeTargetTransform(target_adapter_);
            } catch (const std::runtime_error & e) {
                if (!line_pid_has_last_cable_pose_) {
                    throw;
                }
                RCLCPP_WARN_THROTTLE(
                    node()->get_logger(),
                    *node()->get_clock(),
                    1000,
                    "CableLandingManeuverServer::getUpdatedTargetReference(): Live target transform unavailable, using cached cable pose fallback: %s",
                    e.what()
                );
                target_transform = targetTransformFromCablePose(line_pid_last_cable_pose_world_);
            }

        } catch (const std::runtime_error &e) {

            RCLCPP_WARN_THROTTLE(
                node()->get_logger(),
                *node()->get_clock(),
                1000,
                "CableLandingManeuverServer::getUpdatedTargetReference(): Target is not visible."
            );

            has_failed_ = true;

            return Reference(
                state.position(),
                state.yaw()
            );

        }

        double raw_target_yaw = quatToEul(matToQuat(target_transform.block<3, 3>(0, 0)))[2];
        if (line_pid_initialized_) {
            vector_t locked_direction = line_pid_cable_direction_world_;
            locked_direction(2) = 0.0;
            if (locked_direction.norm() > 1e-6) {
                locked_direction.normalize();
                raw_target_yaw = std::atan2(locked_direction(1), locked_direction(0));
            }
        }
        const double target_yaw = state.yaw() + shortestCableAxisYawError(state.yaw(), raw_target_yaw);

        target_reference = Reference(
            target_transform.block<3, 1>(0, 3),
            target_yaw,
            vector_t(0,0,configuration_->GetParameter("/control/maneuver_controller/cable_landing_target_upwards_velocity").as_double())
        );

        RCLCPP_DEBUG_THROTTLE(
            node()->get_logger(),
            *node()->get_clock(),
            1000,
            "CableLandingManeuverServer::getUpdatedTargetReference(): target_id=%d reference=[%.3f, %.3f, %.3f, raw_yaw=%.3f, axis_yaw=%.3f] upwards_velocity=%.3f",
            target_adapter_->target_id(),
            target_reference->position()(0),
            target_reference->position()(1),
            target_reference->position()(2),
            raw_target_yaw,
            target_reference->yaw(),
            target_reference->velocity()(2)
        );

    }

    return target_reference;

}

bool CableLandingManeuverServer::isWithinSafetyZone(
    const iii_drone::control::State & state,
    const iii_drone::control::Reference & target_reference
) const {

    point_t drone_pos = state.position();
    point_t target_pos = target_reference.position();

    bool is_within_safety_zone = (drone_pos - target_pos).norm() <= configuration_->GetParameter("/control/maneuver_controller/cable_landing_safety_zone_radius").as_double();

    return is_within_safety_zone;

}

bool CableLandingManeuverServer::isWithinSafetyMargins(
    const iii_drone::control::State & state,
    const iii_drone::control::Reference & target_reference
) const {

    vector_t target_point_gripper;
    if (
        !getTargetPointInCableGripperFrame(target_point_gripper)
        || !isTargetWithinGripperVGate(target_point_gripper)
    ) {
        return false;
    }

    float cable_landing_safety_margin_max_xy_position_error = configuration_->GetParameter("/control/maneuver_controller/cable_landing_safety_margin_max_xy_position_error").as_double();
    float cable_landing_safety_margin_max_xy_velocity = configuration_->GetParameter("/control/maneuver_controller/cable_landing_safety_margin_max_xy_velocity").as_double();
    float cable_landing_safety_margin_max_yaw_error = configuration_->GetParameter("/control/maneuver_controller/cable_landing_safety_margin_max_yaw_error").as_double();
    float cable_landing_safety_margin_max_yaw_rate = configuration_->GetParameter("/control/maneuver_controller/cable_landing_safety_margin_max_yaw_rate").as_double();

    float cable_landing_safety_margin_max_negative_vertical_distance = configuration_->GetParameter("/control/maneuver_controller/cable_landing_safety_margin_max_negative_vertical_distance").as_double();
    float cable_landing_safety_margin_cone_slope = configuration_->GetParameter("/control/maneuver_controller/cable_landing_safety_margin_cone_slope").as_double();
    float cable_landing_safety_margin_cone_tolerance = configuration_->GetParameter("/control/maneuver_controller/cable_landing_safety_margin_cone_tolerance").as_double();

    point_t drone_pos = state.position();
    point_t target_pos = target_reference.position();

    vector_t cable_direction_xy(
        std::cos(target_reference.yaw()),
        std::sin(target_reference.yaw()),
        0.0
    );

    if (cable_direction_xy.norm() < 1e-6) {
        cable_direction_xy = vector_t::UnitX();
    } else {
        cable_direction_xy.normalize();
    }

    point_t drone_pos_xy = drone_pos;
    drone_pos_xy(2) = 0;

    point_t target_pos_xy = target_pos;
    target_pos_xy(2) = 0;

    vector_t xy_error_vector = drone_pos_xy - target_pos_xy;
    const double along_cable_position_error = xy_error_vector.dot(cable_direction_xy);
    vector_t perpendicular_position_error_vector = xy_error_vector - along_cable_position_error * cable_direction_xy;
    const double perpendicular_position_error = perpendicular_position_error_vector.norm();

    (void)cable_landing_safety_margin_max_xy_position_error;
    (void)along_cable_position_error;
    // Replaced by the cable-gripper-frame V gate. Keep the perpendicular error
    // calculation for safety cone history and debug context, but do not fail on it.

    float z_distance = drone_pos(2) - target_pos(2);

    if (z_distance > cable_landing_safety_margin_max_negative_vertical_distance) {

        RCLCPP_WARN_THROTTLE(
            node()->get_logger(),
            *node()->get_clock(),
            1000,
            "CableLandingManeuverServer::isWithinSafetyMargins(): Above-target distance violated: z_distance=%.4f threshold=%.4f drone_z=%.4f target_z=%.4f",
            z_distance,
            cable_landing_safety_margin_max_negative_vertical_distance,
            drone_pos(2),
            target_pos(2)
        );
        return false;

    }

    if (!isWithinSafetyZone(
        state,
        target_reference
    )) {
        const double distance = (state.position() - target_reference.position()).norm();
        const double threshold = configuration_->GetParameter(
            "/control/maneuver_controller/cable_landing_safety_zone_radius"
        ).as_double();
        RCLCPP_WARN_THROTTLE(
            node()->get_logger(),
            *node()->get_clock(),
            1000,
            "CableLandingManeuverServer::isWithinSafetyMargins(): Outside near-cable safety zone; deferring tight lateral/yaw checks: distance=%.4f threshold=%.4f drone=[%.4f, %.4f, %.4f] target=[%.4f, %.4f, %.4f]",
            distance,
            threshold,
            state.position()(0),
            state.position()(1),
            state.position()(2),
            target_reference.position()(0),
            target_reference.position()(1),
            target_reference.position()(2)
        );
        return true;
    }

    z_distance = abs(z_distance);

    (void)z_distance;
    (void)perpendicular_position_error;
    (void)cable_landing_safety_margin_cone_tolerance;
    (void)cable_landing_safety_margin_cone_slope;
    // Safety cone replaced by the cable-gripper-frame V gate.

    vector_t drone_vel = state.velocity();

    vector_t drone_vel_xy = drone_vel;
    drone_vel_xy(2) = 0;

    const double along_cable_velocity = drone_vel_xy.dot(cable_direction_xy);
    vector_t perpendicular_velocity_vector = drone_vel_xy - along_cable_velocity * cable_direction_xy;
    const double perpendicular_velocity = perpendicular_velocity_vector.norm();

    if (perpendicular_velocity > cable_landing_safety_margin_max_xy_velocity) {

        RCLCPP_WARN_THROTTLE(
            node()->get_logger(),
            *node()->get_clock(),
            1000,
            "CableLandingManeuverServer::isWithinSafetyMargins(): Perpendicular velocity violated: perp_velocity=%.4f along_velocity=%.4f threshold=%.4f velocity=[%.4f, %.4f, %.4f] cable_dir_xy=[%.4f, %.4f]",
            perpendicular_velocity,
            along_cable_velocity,
            cable_landing_safety_margin_max_xy_velocity,
            drone_vel(0),
            drone_vel(1),
            drone_vel(2),
            cable_direction_xy(0),
            cable_direction_xy(1)
        );
        return false;

    }

    quaternion_t drone_quat = state.quaternion();
    quaternion_t target_quat = eulToQuat(
        euler_angles_t(
            0, 
            0, 
            target_reference.yaw()
        )
    );
    quaternion_t drone_quat_inv = quatInv(drone_quat);
    quaternion_t quat_error = quatMultiply(
        drone_quat_inv,
        target_quat
    );
    euler_angles_t eul_error = quatToEul(quat_error);
    double yaw_error = shortestCableAxisYawError(0.0, eul_error(2));

    if (abs(yaw_error) > cable_landing_safety_margin_max_yaw_error) {

        RCLCPP_WARN_THROTTLE(
            node()->get_logger(),
            *node()->get_clock(),
            1000,
            "CableLandingManeuverServer::isWithinSafetyMargins(): Yaw axis error violated: raw_yaw_error=%.4f axis_yaw_error=%.4f threshold=%.4f target_yaw=%.4f",
            eul_error(2),
            yaw_error,
            cable_landing_safety_margin_max_yaw_error,
            target_reference.yaw()
        );
        return false;

    }

    vector_t drone_angular_vel = state.angular_velocity();

    const double yaw_rate = drone_angular_vel(2);

    if (abs(yaw_rate) > cable_landing_safety_margin_max_yaw_rate) {

        RCLCPP_WARN_THROTTLE(
            node()->get_logger(),
            *node()->get_clock(),
            1000,
            "CableLandingManeuverServer::isWithinSafetyMargins(): Yaw rate violated: yaw_rate=%.4f threshold=%.4f angular_velocity=[%.4f, %.4f, %.4f]",
            yaw_rate,
            cable_landing_safety_margin_max_yaw_rate,
            drone_angular_vel(0),
            drone_angular_vel(1),
            drone_angular_vel(2)
        );
        return false;

    }

    return true;

}

bool CableLandingManeuverServer::getTargetPointInCableGripperFrame(
    iii_drone::types::vector_t & target_point_gripper
) const {

    auto cda_handler = awareness_handler();

    geometry_msgs::msg::PoseStamped target_pose_world;
    target_pose_world.header.frame_id = configuration_->GetParameter("/tf/world_frame_id").as_string();

    try {
        pose_t target_pose;
        if (line_pid_has_last_cable_pose_) {
            target_pose = line_pid_last_cable_pose_world_;
        } else {
            target_pose = cda_handler->GetPoseOfTarget(target_adapter_);
        }
        target_pose_world.pose.position.x = target_pose.position(0);
        target_pose_world.pose.position.y = target_pose.position(1);
        target_pose_world.pose.position.z = target_pose.position(2);
        target_pose_world.pose.orientation.w = 1.0;
    } catch (const std::runtime_error & e) {
        RCLCPP_WARN_THROTTLE(
            node()->get_logger(),
            *node()->get_clock(),
            1000,
            "CableLandingManeuverServer::getTargetPointInCableGripperFrame(): Could not get target pose: %s",
            e.what()
        );
        return false;
    }

    try {
        const auto target_pose_gripper = cda_handler->tf_buffer()->transform(
            target_pose_world,
            configuration_->GetParameter("/tf/cable_gripper_frame_id").as_string()
        );

        target_point_gripper = vector_t(
            target_pose_gripper.pose.position.x,
            target_pose_gripper.pose.position.y,
            target_pose_gripper.pose.position.z
        );
    } catch (const tf2::TransformException & e) {
        RCLCPP_WARN_THROTTLE(
            node()->get_logger(),
            *node()->get_clock(),
            1000,
            "CableLandingManeuverServer::getTargetPointInCableGripperFrame(): Could not transform target point to gripper frame: %s",
            e.what()
        );
        return false;
    }

    return true;

}

bool CableLandingManeuverServer::isTargetWithinGripperVGate(
    const iii_drone::types::vector_t & target_point_gripper
) const {

    const double apex_z = configuration_->GetParameter(
        "/control/maneuver_controller/cable_landing_gripper_v_gate_apex_z"
    ).as_double();
    const double reference_z = configuration_->GetParameter(
        "/control/maneuver_controller/cable_landing_gripper_v_gate_reference_z"
    ).as_double();
    const double half_width_at_reference_z = configuration_->GetParameter(
        "/control/maneuver_controller/cable_landing_gripper_v_gate_half_width_at_reference_z"
    ).as_double();
    const double center_y = configuration_->GetParameter(
        "/control/maneuver_controller/cable_landing_gripper_v_gate_center_y"
    ).as_double();
    const double violation_grace_s = configuration_->GetParameter(
        "/control/maneuver_controller/cable_landing_gripper_v_gate_violation_grace_s"
    ).as_double();

    const double height = reference_z - apex_z;
    if (height <= 0.0 || half_width_at_reference_z <= 0.0) {
        RCLCPP_WARN(
            node()->get_logger(),
            "CableLandingManeuverServer::isTargetWithinGripperVGate(): Invalid V gate configuration: apex_z=%.4f reference_z=%.4f half_width_at_reference_z=%.4f",
            apex_z,
            reference_z,
            half_width_at_reference_z
        );
        return false;
    }

    const double z_above_apex = target_point_gripper(2) - apex_z;
    const double lateral_y = target_point_gripper(1) - center_y;
    const double allowed_lateral_y = z_above_apex * half_width_at_reference_z / height;

    const bool violated = z_above_apex < 0.0 || std::abs(lateral_y) > allowed_lateral_y;
    if (violated) {
        const auto now = node()->now();
        if (violation_grace_s > 0.0) {
            if (!gripper_v_gate_violation_active_) {
                gripper_v_gate_violation_started_ = now;
                gripper_v_gate_violation_active_ = true;
            }

            const double violation_duration_s = (now - gripper_v_gate_violation_started_).seconds();
            if (violation_duration_s < violation_grace_s) {
                RCLCPP_WARN_THROTTLE(
                    node()->get_logger(),
                    *node()->get_clock(),
                    1000,
                    "CableLandingManeuverServer::isTargetWithinGripperVGate(): Gripper V gate violated within grace: duration=%.3f grace=%.3f target_gripper=[%.4f, %.4f, %.4f] lateral_y=%.4f allowed_lateral_y=%.4f",
                    violation_duration_s,
                    violation_grace_s,
                    target_point_gripper(0),
                    target_point_gripper(1),
                    target_point_gripper(2),
                    lateral_y,
                    allowed_lateral_y
                );
                return true;
            }
        }

        RCLCPP_WARN_THROTTLE(
            node()->get_logger(),
            *node()->get_clock(),
            1000,
            "CableLandingManeuverServer::isTargetWithinGripperVGate(): Gripper V gate violated: target_gripper=[%.4f, %.4f, %.4f] apex_z=%.4f center_y=%.4f reference_z=%.4f half_width_at_reference_z=%.4f z_above_apex=%.4f lateral_y=%.4f allowed_lateral_y=%.4f grace=%.3f",
            target_point_gripper(0),
            target_point_gripper(1),
            target_point_gripper(2),
            apex_z,
            center_y,
            reference_z,
            half_width_at_reference_z,
            z_above_apex,
            lateral_y,
            allowed_lateral_y,
            violation_grace_s
        );
        return false;
    }

    gripper_v_gate_violation_active_ = false;

    RCLCPP_DEBUG_THROTTLE(
        node()->get_logger(),
        *node()->get_clock(),
        1000,
        "CableLandingManeuverServer::isTargetWithinGripperVGate(): Gripper V gate ok: target_gripper=[%.4f, %.4f, %.4f] lateral_y=%.4f allowed_lateral_y=%.4f",
        target_point_gripper(0),
        target_point_gripper(1),
        target_point_gripper(2),
        lateral_y,
        allowed_lateral_y
    );

    return true;

}

Reference CableLandingManeuverServer::truncateReferenceWithinSafetyZone(
    const iii_drone::control::Reference & reference,
    const iii_drone::control::State & state,
    const iii_drone::control::Reference & target_reference
) const {

    if ((state.position() - target_reference.position()).norm() > configuration_->GetParameter("/control/maneuver_controller/cable_landing_reference_truncate_radius").as_double()) {

        return reference;

    }

    return Reference(
        point_t::Constant(NAN),
        NAN,
        reference.velocity(),
        reference.yaw_rate(),
        reference.acceleration(),
        reference.yaw_acceleration(),
        reference.stamp()
    );

}
