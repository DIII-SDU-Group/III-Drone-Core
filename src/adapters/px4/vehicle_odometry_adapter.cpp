/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/adapters/px4/vehicle_odometry_adapter.hpp>

/*****************************************************************************/
// Implementation
/*****************************************************************************/

using namespace iii_drone::adapters::px4;
using namespace iii_drone::types;
using namespace iii_drone::math;

VehicleOdometryAdapter::VehicleOdometryAdapter() { 

    stamp_ = rclcpp::Time(0);

}

VehicleOdometryAdapter::VehicleOdometryAdapter(const px4_msgs::msg::VehicleOdometry & vehicle_odometry_msg) {

    UpdateFromMsg(vehicle_odometry_msg);

}

void VehicleOdometryAdapter::UpdateFromMsg(const px4_msgs::msg::VehicleOdometry & vehicle_odometry_msg) {

    stamp_ = rclcpp::Time(vehicle_odometry_msg.timestamp * 1000);

    pose_frame_ = (pose_frame_t)vehicle_odometry_msg.pose_frame;
    position_ = {
        vehicle_odometry_msg.position[0], 
        -vehicle_odometry_msg.position[1], 
        -vehicle_odometry_msg.position[2]
    };

    quaternion_t px4_q = {
        vehicle_odometry_msg.q[0], 
        vehicle_odometry_msg.q[1], 
        vehicle_odometry_msg.q[2], 
        vehicle_odometry_msg.q[3]
    };
    euler_angles_t euler_orientation = quatToEul(px4_q);

    euler_orientation = {
        euler_orientation[0], 
        -euler_orientation[1], 
        -euler_orientation[2]
    };

    quaternion_ = eulToQuat(euler_orientation);

    velocity_frame_ = (velocity_frame_t)vehicle_odometry_msg.velocity_frame;

    velocity_ = {
        vehicle_odometry_msg.velocity[0], 
        -vehicle_odometry_msg.velocity[1], 
        -vehicle_odometry_msg.velocity[2]
    };
    
    angular_velocity_ = {
        vehicle_odometry_msg.angular_velocity[0], 
        -vehicle_odometry_msg.angular_velocity[1], 
        -vehicle_odometry_msg.angular_velocity[2]
    };

    position_variance_ = {
        vehicle_odometry_msg.position_variance[0], 
        vehicle_odometry_msg.position_variance[1], 
        vehicle_odometry_msg.position_variance[2]
    };
    
    orientation_variance_ = {
        vehicle_odometry_msg.orientation_variance[0], 
        vehicle_odometry_msg.orientation_variance[1], 
        vehicle_odometry_msg.orientation_variance[2]
    };
    
    velocity_variance_ = {
        vehicle_odometry_msg.velocity_variance[0], 
        vehicle_odometry_msg.velocity_variance[1], 
        vehicle_odometry_msg.velocity_variance[2]
    };

}

// iii::control::State VehicleOdometryAdapter::ToState() const {

//     return iii::control::State(
//         position_,
//         velocity_,
//         quaternion_,
//         angular_velocity_
//     );

// }

const geometry_msgs::msg::TransformStamped VehicleOdometryAdapter::ToTransformStamped(
    const std::string & drone_frame_id,
    const std::string & world_frame_id
) const {

    geometry_msgs::msg::TransformStamped msg{};

    msg.header.stamp = stamp_;
    msg.header.frame_id = world_frame_id;
    msg.child_frame_id = drone_frame_id;

    msg.transform = transformMsgFromTransform(
        (vector_t)position_,
        quaternion_
    );

    return msg;

}

const rclcpp::Time & VehicleOdometryAdapter::stamp() const {

    return stamp_;

}

const pose_frame_t & VehicleOdometryAdapter::pose_frame() const {

    return pose_frame_;

}

const point_t & VehicleOdometryAdapter::position() const {

    return position_;

}

const quaternion_t & VehicleOdometryAdapter::quaternion() const {

    return quaternion_;

}

const velocity_frame_t & VehicleOdometryAdapter::velocity_frame() const {

    return velocity_frame_;

}

const vector_t & VehicleOdometryAdapter::velocity() const {

    return velocity_;

}

const vector_t & VehicleOdometryAdapter::angular_velocity() const {

    return angular_velocity_;

}

const Eigen::Vector3d & VehicleOdometryAdapter::position_variance() const {

    return position_variance_;

}

const Eigen::Vector3d & VehicleOdometryAdapter::orientation_variance() const {

    return orientation_variance_;

}

const Eigen::Vector3d & VehicleOdometryAdapter::velocity_variance() const {

    return velocity_variance_;

}