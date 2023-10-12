/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "iii_drone_core/utils/drone_frame_broadcaster_node/drone_frame_broadcaster_node.hpp"

using namespace iii_drone::utils::drone_frame_broadcaster_node;
using namespace iii_drone::math;
using namespace iii_drone::types;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

DroneFrameBroadcasterNode::DroneFrameBroadcasterNode(
    const std::string & node_name, 
    const std::string & node_namespace,
    const rclcpp::NodeOptions & options
) : Node(
    node_name, 
    node_namespace,
    options
) {

    // Params
    this->declare_parameter<std::string>("drone_frame_id", "drone");
    this->declare_parameter<std::string>("world_frame_id", "world");

    this->get_parameter("drone_frame_id", drone_frame_id_);
    this->get_parameter("world_frame_id", world_frame_id_);


    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    std::ostringstream stream;

    rclcpp::QoS sub_qos(rclcpp::KeepLast(1));
    sub_qos.transient_local();
    sub_qos.best_effort();

    subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", 
        sub_qos,
        std::bind(&DroneFrameBroadcasterNode::odometryCallback, this, std::placeholders::_1));

    R_NED_to_body_frame = eulToR(orientation_t(M_PI, 0, 0));

}

void DroneFrameBroadcasterNode::odometryCallback(const std::shared_ptr<px4_msgs::msg::VehicleOdometry> msg) {

    rclcpp::Time now = this->get_clock()->now();
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = now;
    t.header.frame_id = world_frame_id_;
    t.child_frame_id = drone_frame_id_;

    point_t position(
        msg->position[0],
        msg->position[1], 
        msg->position[2]
    );

    position = R_NED_to_body_frame * position;

    quat_t quat(
        msg->q[0],
        msg->q[1],
        msg->q[2],
        msg->q[3]
    );


    orientation_t eul = quatToEul(quat);
    eul(1) = -eul(1);                       // Dirty hack
    eul(2) = -eul(2);
    quat = eulToQuat(eul);

    t.transform.translation.x = position(0);
    t.transform.translation.y = position(1);
    t.transform.translation.z = position(2);

    t.transform.rotation.w = quat(0);
    t.transform.rotation.x = quat(1);
    t.transform.rotation.y = quat(2);
    t.transform.rotation.z = quat(3);

    orientation_t roll_eul(M_PI, 0, 0);
    quat_t roll_quat = eulToQuat(roll_eul);
    quat = quatMultiply(quat, roll_quat);

    // Send the transformation
    tf_broadcaster_->sendTransform(t);

    // RCLCPP debug published transform
    RCLCPP_DEBUG(this->get_logger(), "Published transform: %s -> %s", t.header.frame_id.c_str(), t.child_frame_id.c_str());

}

int main(int argc, char * argv[]) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<iii_drone::utils::drone_frame_broadcaster_node::DroneFrameBroadcasterNode>());
    rclcpp::shutdown();
    return 0;

}