/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/maneuver/maneuver_types.hpp>

using namespace iii_drone::control::maneuver;
using namespace iii_drone::control;
using namespace iii_drone::types;
using namespace iii_drone::math;
using namespace iii_drone::adapters;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

fly_to_position_maneuver_params_t::fly_to_position_maneuver_params_t() {
    frame_id = "";
    target_position = point_t();
    target_yaw = 0.0;
}

fly_to_position_maneuver_params_t::fly_to_position_maneuver_params_t(
    const std::string frame_id,
    const point_t target_position,
    const float target_yaw
) {
    this->frame_id = frame_id;
    this->target_position = target_position;
    this->target_yaw = target_yaw;
}

fly_to_position_maneuver_params_t::fly_to_position_maneuver_params_t(std::shared_ptr<void> params) {
    fly_to_position_maneuver_params_t * params_ptr = static_cast<fly_to_position_maneuver_params_t *>(params.get());
    frame_id = params_ptr->frame_id;
    target_position = params_ptr->target_position;
    target_yaw = params_ptr->target_yaw;
}

point_t fly_to_position_maneuver_params_t::transform_target_position(
    const std::string target_frame_id,
    tf2_ros::Buffer::SharedPtr tf_buffer
) const {

    if (frame_id == target_frame_id) {
        return target_position;
    }

    geometry_msgs::msg::PointStamped target_position_stamped;
    target_position_stamped.header.frame_id = frame_id;
    target_position_stamped.point = pointMsgFromPoint(target_position);

    geometry_msgs::msg::PointStamped transformed_target_position_stamped;

    try {
        transformed_target_position_stamped = tf_buffer->transform(target_position_stamped, target_frame_id);
    } catch (tf2::TransformException & ex) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "%s", ex.what());
        throw ex;
    }

    return pointFromPointMsg(transformed_target_position_stamped.point);

}

double fly_to_position_maneuver_params_t::transform_target_yaw(
    const std::string target_frame_id,
    tf2_ros::Buffer::SharedPtr tf_buffer
) const {

    if (frame_id == target_frame_id) {
        return target_yaw;
    }

    geometry_msgs::msg::QuaternionStamped target_orientation_stamped;
    target_orientation_stamped.header.frame_id = frame_id;
    target_orientation_stamped.quaternion = quaternionMsgFromQuaternion(eulToQuat(euler_angles_t(0, 0, target_yaw)));

    geometry_msgs::msg::QuaternionStamped transformed_target_orientation_stamped;

    try {
        transformed_target_orientation_stamped = tf_buffer->transform(target_orientation_stamped, target_frame_id);
    } catch (tf2::TransformException & ex) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "%s", ex.what());
        throw ex;
    }

    return quatToEul(quaternionFromQuaternionMsg(transformed_target_orientation_stamped.quaternion))(2);

}

cable_landing_maneuver_params_t::cable_landing_maneuver_params_t() : target_cable_id(0.0) { }

cable_landing_maneuver_params_t::cable_landing_maneuver_params_t(int target_cable_id) : target_cable_id(target_cable_id) { }

cable_landing_maneuver_params_t::cable_landing_maneuver_params_t(std::shared_ptr<void> params) {
    cable_landing_maneuver_params_t * params_ptr = static_cast<cable_landing_maneuver_params_t *>(params.get());
    target_cable_id = params_ptr->target_cable_id;
}

cable_takeoff_maneuver_params_t::cable_takeoff_maneuver_params_t() { 

    target_cable_id = 0;
    target_cable_distance = 0.0;

}

cable_takeoff_maneuver_params_t::cable_takeoff_maneuver_params_t(
    int target_cable_id,
    float target_cable_distance
) : target_cable_id(target_cable_id),
    target_cable_distance(target_cable_distance) { }

cable_takeoff_maneuver_params_t::cable_takeoff_maneuver_params_t(std::shared_ptr<void> params) {
    cable_takeoff_maneuver_params_t * params_ptr = static_cast<cable_takeoff_maneuver_params_t *>(params.get());
    target_cable_id = params_ptr->target_cable_id;
    target_cable_distance = params_ptr->target_cable_distance;
}

transform_matrix_t cable_takeoff_maneuver_params_t::get_target_transform() const {
    
    transform_matrix_t target_transform = transform_matrix_t::Identity();

    target_transform.block<3, 1>(0, 3) = vector_t(0, 0, target_cable_distance);

    return target_transform;

}

fly_to_object_maneuver_params_t::fly_to_object_maneuver_params_t() : target_adapter(TargetAdapter()) { }

fly_to_object_maneuver_params_t::fly_to_object_maneuver_params_t(TargetAdapter target_adapter) : target_adapter(target_adapter) { }

fly_to_object_maneuver_params_t::fly_to_object_maneuver_params_t(std::shared_ptr<void> params) {
    fly_to_object_maneuver_params_t * params_ptr = static_cast<fly_to_object_maneuver_params_t *>(params.get());
    target_adapter = params_ptr->target_adapter;
}

hover_maneuver_params_t::hover_maneuver_params_t() { }

hover_maneuver_params_t::hover_maneuver_params_t(double duration_s) {
    this->duration_s = duration_s;
 }

hover_maneuver_params_t::hover_maneuver_params_t(std::shared_ptr<void> params) {
    hover_maneuver_params_t * params_ptr = static_cast<hover_maneuver_params_t *>(params.get());
    duration_s = params_ptr->duration_s;
}

hover_by_object_maneuver_params_t::hover_by_object_maneuver_params_t() : target_adapter(TargetAdapter()) { }

hover_by_object_maneuver_params_t::hover_by_object_maneuver_params_t(
    TargetAdapter target_adapter,
    double duration_s
) {
    this->target_adapter = target_adapter;
    this->duration_s = duration_s;
}

hover_by_object_maneuver_params_t::hover_by_object_maneuver_params_t(std::shared_ptr<void> params) {
    hover_by_object_maneuver_params_t * params_ptr = static_cast<hover_by_object_maneuver_params_t *>(params.get());
    target_adapter = params_ptr->target_adapter;
    duration_s = params_ptr->duration_s;
}

hover_on_cable_maneuver_params_t::hover_on_cable_maneuver_params_t() {
    target_cable_id = -1;
    target_z_velocity = 0.0;
    target_yaw_rate = 0.0;
}

hover_on_cable_maneuver_params_t::hover_on_cable_maneuver_params_t(
    int target_cable_id,
    double target_z_velocity,
    double target_yaw_rate,
    double duration_s
) {
    this->target_cable_id = target_cable_id;
    this->target_z_velocity = target_z_velocity;
    this->target_yaw_rate = target_yaw_rate;
    this->duration_s = duration_s;
}

hover_on_cable_maneuver_params_t::hover_on_cable_maneuver_params_t(std::shared_ptr<void> params) {
    hover_on_cable_maneuver_params_t * params_ptr = static_cast<hover_on_cable_maneuver_params_t *>(params.get());
    target_cable_id = params_ptr->target_cable_id;
    target_z_velocity = params_ptr->target_z_velocity;
    target_yaw_rate = params_ptr->target_yaw_rate;
    duration_s = params_ptr->duration_s;
}