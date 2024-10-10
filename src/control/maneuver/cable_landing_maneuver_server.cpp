/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/maneuver/cable_landing_maneuver_server.hpp>

using namespace iii_drone::control::maneuver;
using namespace iii_drone::control;
using namespace iii_drone::types;
using namespace iii_drone::math;
using namespace iii_drone::utils;
using namespace iii_drone::adapters;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

CableLandingManeuverServer::CableLandingManeuverServer(
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

    createServer<iii_drone_interfaces::action::CableLanding>();

}

bool CableLandingManeuverServer::CanExecuteManeuver(
    const Maneuver & maneuver,
    const combined_drone_awareness_t & drone_awareness
) const {

    RCLCPP_DEBUG(node()->get_logger(), "CableLandingManeuverServer::CanExecuteManeuver()");

    cable_landing_maneuver_params_t cable_landing_maneuver_params(maneuver.maneuver_params());

    if (maneuver.maneuver_type() != MANEUVER_TYPE_CABLE_LANDING) {
        RCLCPP_WARN(node()->get_logger(), "CableLandingManeuverServer::CanExecuteManeuver(): Maneuver type is not MANEUVER_TYPE_CABLE_LANDING.");
        return false;
    }

    if (!drone_awareness.armed) {
        RCLCPP_WARN(node()->get_logger(), "CableLandingManeuverServer::CanExecuteManeuver(): Drone is not armed.");
        return false;
    }

    if (!drone_awareness.offboard) {
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

    if (drone_awareness.target_adapter.target_type() != TARGET_TYPE_CABLE) {
        RCLCPP_WARN(node()->get_logger(), "CableLandingManeuverServer::CanExecuteManeuver(): Target type is not TARGET_TYPE_CABLE.");
        return false;
    }

    if (drone_awareness.target_adapter.target_id() != cable_landing_maneuver_params.target_cable_id) {
        RCLCPP_WARN(node()->get_logger(), "CableLandingManeuverServer::CanExecuteManeuver(): Target ID does not match.");
        return false;
    }

    if (!drone_awareness.target_position_known) {
        RCLCPP_WARN(node()->get_logger(), "CableLandingManeuverServer::CanExecuteManeuver(): Target position is not known.");
        return false;
    }

    CombinedDroneAwarenessHandler::SharedPtr cda_handler = awareness_handler();

    transform_matrix_t current_target_transform = drone_awareness.target_adapter.target_transform();

    vector_t current_target_translation = current_target_transform.block<3, 1>(0, 3);

    geometry_msgs::msg::Vector3Stamped current_target_translation_drone_to_cable;
    current_target_translation_drone_to_cable.vector = vectorMsgFromVector(current_target_translation);
    current_target_translation_drone_to_cable.header.frame_id = drone_awareness.target_adapter.reference_frame_id();

    try {
        current_target_translation_drone_to_cable = cda_handler->tf_buffer()->transform(
            current_target_translation_drone_to_cable,
            parameters_->GetParameter("drone_frame_id").as_string()
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

    if (current_target_translation(2) < parameters_->GetParameter("cable_landing_min_z_distance").as_double()) {
        RCLCPP_WARN(node()->get_logger(), "CableLandingManeuverServer::CanExecuteManeuver(): Target translation is less than cable_landing_min_z_distance.");
        return false;
    }

    if (current_target_translation(2) > parameters_->GetParameter("cable_landing_max_z_distance").as_double()) {
        RCLCPP_WARN(node()->get_logger(), "CableLandingManeuverServer::CanExecuteManeuver(): Target translation is greater than cable_landing_max_z_distance.");
        return false;
    }

    quaternion_t current_target_quat = matToQuat(current_target_transform.block<3, 3>(0, 0));

    geometry_msgs::msg::QuaternionStamped current_target_quat_gripper_to_cable_msg;
    current_target_quat_gripper_to_cable_msg.quaternion = quaternionMsgFromQuaternion(current_target_quat);
    current_target_quat_gripper_to_cable_msg.header.frame_id = drone_awareness.target_adapter.reference_frame_id();

    try {
        current_target_quat_gripper_to_cable_msg = cda_handler->tf_buffer()->transform(
            current_target_quat_gripper_to_cable_msg,
            parameters_->GetParameter("gripper_frame_id").as_string()
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

    vector_t pos_world_to_drone = drone_awareness.state.position();

    transform_matrix_t current_target_transform_world_to_drone;
    try {
        current_target_transform_world_to_drone = cda_handler->ComputeTargetTransform(drone_awareness.target_adapter);
    } catch (const std::runtime_error &e) {
        RCLCPP_WARN(node()->get_logger(), "CableLandingManeuverServer::CanExecuteManeuver(): Could not compute target transform.");
        return false;
    }

    vector_t current_target_pos_world_to_drone = current_target_transform_world_to_drone.block<3, 1>(0, 3);

    vector_t p_drone_to_target = current_target_pos_world_to_drone - pos_world_to_drone;

    if (p_drone_to_target.norm() > parameters_->GetParameter("cable_landing_max_initial_distance_error").as_double()) {
        RCLCPP_WARN(node()->get_logger(), "CableLandingManeuverServer::CanExecuteManeuver(): Initial distance error is too large.");
        return false;
    }

    // geometry_msgs::msg::QuaternionStamped current_world_to_target_quat;
    // current_world_to_target_quat.quaternion = quaternionMsgFromQuaternion(current_target_pose.orientation);
    // current_world_to_target_quat.header.frame_id = drone_awareness.target_adapter.reference_frame_id();

    // geometry_msgs::msg::QuaternionStamped current_world_to_target_quat = cda_handler->tf_buffer()->transform(
    //     current_target_pose.orientation,
    //     parameters_->GetParameter("drone_frame_id").as_string()
    // );

    quaternion_t current_target_quat_world_to_drone = matToQuat(current_target_transform_world_to_drone.block<3, 3>(0, 0));
    quaternion_t quat_world_to_drone = drone_awareness.state.quaternion();

    quaternion_t quat_drone_to_target = quatMultiply(
        quatInv(quat_world_to_drone),
        current_target_quat_world_to_drone
    );

    euler_angles_t eul_drone_to_target = quatToEul(quat_drone_to_target);

    if (abs(eul_drone_to_target(2)) > parameters_->GetParameter("cable_landing_max_initial_yaw_error").as_double()) {
        RCLCPP_WARN(node()->get_logger(), "CableLandingManeuverServer::CanExecuteManeuver(): Initial yaw error is too large.");
        return false;
    }

    RCLCPP_DEBUG(node()->get_logger(), "CableLandingManeuverServer::CanExecuteManeuver(): Cable landing maneuver can be executed.");

    return true;

}

combined_drone_awareness_t CableLandingManeuverServer::ExpectedAwarenessAfterExecution(const Maneuver & maneuver) {

    cable_landing_maneuver_params_t cable_landing_maneuver_params(maneuver.maneuver_params());

    TargetAdapter target_adapter = TargetAdapter(
        TARGET_TYPE_CABLE,
        cable_landing_maneuver_params.target_cable_id,
        parameters_->GetParameter("gripper_frame_id").as_string(),
        transform_matrix_t::Identity()
    );

    transform_matrix_t target_transform = awareness_handler()->ComputeTargetTransform(target_adapter);

    State state(
        target_transform.block<3, 1>(0, 3),
        vector_t::Zero(),
        matToQuat(target_transform.block<3, 3>(0, 0)),
        vector_t::Zero()
    );

    combined_drone_awareness_t awareness_after;

    awareness_after.armed = true;
    awareness_after.offboard = true;
    awareness_after.target_position_known = true;
    awareness_after.drone_location = DRONE_LOCATION_ON_CABLE;
    awareness_after.target_adapter = target_adapter;
    awareness_after.state = state;

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

    target_adapter_ = TargetAdapter(
        TARGET_TYPE_CABLE,
        cable_landing_maneuver_params.target_cable_id,
        parameters_->GetParameter("gripper_frame_id").as_string(),
        transform_matrix_t::Identity()
    );

    if (trajectory_generator_client_->busy()) {

        std::string error_message = "CableLandingManeuverServer::startExecution(): Trajectory generator client is busy, cannot start execution of maneuver.";

        RCLCPP_FATAL(node()->get_logger(), error_message.c_str());

        throw std::runtime_error(error_message);

    }

    first_iteration_ = true;
    has_failed_ = false;

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
            parameters_->GetParameter("use_mpc").as_bool()
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

bool CableLandingManeuverServer::hasSucceeded(Maneuver &) {

    auto cda_handler = awareness_handler();

    if (parameters_->GetParameter("use_gripper_status_condition").as_bool()) {

        if (!cda_handler->gripper_open()) {
            RCLCPP_INFO(
                node()->get_logger(),
                "CableLandingManeuverServer::hasSucceeded(): Gripper is closed, succeeded."
            );
            return true;
        }

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

    if(distance < parameters_->GetParameter("reached_position_euclidean_distance_threshold").as_double()) {
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
        if (!cda_handler->on_cable()) {
            RCLCPP_WARN(
                node()->get_logger(),
                "CableLandingManeuverServer::hasFailed(): Drone is not in flight and not on cable."
            );
            return true;
        } else if (cda_handler->on_cable_id() != target_adapter_->target_id()) {
            RCLCPP_WARN(
                node()->get_logger(),
                "CableLandingManeuverServer::hasFailed(): Drone is on cable but the on cable id is wrong."
            );
            return true;
        }
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

    if (!cda_handler->target_position_known()) {
        RCLCPP_WARN(
            node()->get_logger(),
            "CableLandingManeuverServer::hasFailed(): Target position is not known."
        );
        return true;
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

    std::string world_frame_id = parameters_->GetParameter("world_frame_id").as_string();

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
            awareness_handler()->ClearTarget();
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
        parameters_->GetParameter("hover_on_cable_default_z_velocity").as_double(),
        parameters_->GetParameter("hover_on_cable_default_yaw_rate").as_double()
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

        try {

            target_transform = cda_handler->ComputeTargetTransform(target_adapter_);

        } catch (const std::runtime_error &e) {

            RCLCPP_WARN(
                node()->get_logger(),
                "CableLandingManeuverServer::getUpdatedTargetReference(): Target is not visible."
            );

            has_failed_ = true;

            return Reference(
                state.position(),
                state.yaw()
            );

        }

        target_reference = Reference(
            target_transform.block<3, 1>(0, 3),
            quatToEul(matToQuat(target_transform.block<3, 3>(0, 0)))[2]
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

    bool is_within_safety_zone = (drone_pos - target_pos).norm() <= parameters_->GetParameter("cable_landing_safety_zone_radius").as_double();

    return is_within_safety_zone;

}

bool CableLandingManeuverServer::isWithinSafetyMargins(
    const iii_drone::control::State & state,
    const iii_drone::control::Reference & target_reference
) const {

    if (!isWithinSafetyZone(
        state, 
        target_reference
    )) {
        return true;
    }

    float cable_landing_safety_margin_max_xy_position_error = parameters_->GetParameter("cable_landing_safety_margin_max_xy_position_error").as_double();
    float cable_landing_safety_margin_max_xy_velocity = parameters_->GetParameter("cable_landing_safety_margin_max_xy_velocity").as_double();
    float cable_landing_safety_margin_max_yaw_error = parameters_->GetParameter("cable_landing_safety_margin_max_yaw_error").as_double();
    float cable_landing_safety_margin_max_yaw_rate = parameters_->GetParameter("cable_landing_safety_margin_max_yaw_rate").as_double();

    float cable_landing_safety_margin_max_negative_vertical_distance = parameters_->GetParameter("cable_landing_safety_margin_max_negative_vertical_distance").as_double();
    float cable_landing_safety_margin_cone_slope = parameters_->GetParameter("cable_landing_safety_margin_cone_slope").as_double();
    float cable_landing_safety_margin_cone_tolerance = parameters_->GetParameter("cable_landing_safety_margin_cone_tolerance").as_double();

    point_t drone_pos = state.position();
    point_t target_pos = target_reference.position();

    point_t drone_pos_xy = drone_pos;
    drone_pos_xy(2) = 0;

    point_t target_pos_xy = target_pos;
    target_pos_xy(2) = 0;

    if ((drone_pos_xy - target_pos_xy).norm() > cable_landing_safety_margin_max_xy_position_error) {

        RCLCPP_WARN(node()->get_logger(), "CableLandingManeuverServer::isWithinSafetyMargins(): XY position error is too large.");
        return false;

    }

    float z_distance = drone_pos(2) - target_pos(2);

    if (z_distance > cable_landing_safety_margin_max_negative_vertical_distance) {

        RCLCPP_WARN(node()->get_logger(), "CableLandingManeuverServer::isWithinSafetyMargins(): Drone is too much above the target.");
        return false;

    }

    z_distance = abs(z_distance);

    float xy_distance = (drone_pos_xy - target_pos_xy).norm();

    if (z_distance + cable_landing_safety_margin_cone_tolerance < xy_distance * cable_landing_safety_margin_cone_slope) {

        RCLCPP_WARN(node()->get_logger(), "CableLandingManeuverServer::isWithinSafetyMargins(): Drone is outside the safety cone.");
        return false;

    }

    vector_t drone_vel = state.velocity();

    vector_t drone_vel_xy = drone_vel;
    drone_vel_xy(2) = 0;

    if (drone_vel_xy.norm() > cable_landing_safety_margin_max_xy_velocity) {

        RCLCPP_WARN(node()->get_logger(), "CableLandingManeuverServer::isWithinSafetyMargins(): XY velocity is too large.");
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
    double yaw_error = eul_error(2);

    if (abs(yaw_error) > cable_landing_safety_margin_max_yaw_error) {

        RCLCPP_WARN(node()->get_logger(), "CableLandingManeuverServer::isWithinSafetyMargins(): Yaw error is too large.");
        return false;

    }

    vector_t drone_angular_vel = state.angular_velocity();

    if (abs(drone_angular_vel(2)) > cable_landing_safety_margin_max_yaw_rate) {

        RCLCPP_WARN(node()->get_logger(), "CableLandingManeuverServer::isWithinSafetyMargins(): Yaw rate is too large.");
        return false;

    }

    return true;

}

Reference CableLandingManeuverServer::truncateReferenceWithinSafetyZone(
    const iii_drone::control::Reference & reference,
    const iii_drone::control::State & state,
    const iii_drone::control::Reference & target_reference
) const {

    if ((state.position() - target_reference.position()).norm() > parameters_->GetParameter("cable_landing_reference_truncate_radius").as_double()) {

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