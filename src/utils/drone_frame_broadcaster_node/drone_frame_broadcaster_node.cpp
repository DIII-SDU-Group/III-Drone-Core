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
), configurator_(this) {

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

    R_NED_to_body_frame = eulToMat(euler_angles_t(M_PI, 0, 0));

}

void DroneFrameBroadcasterNode::odometryCallback(const std::shared_ptr<px4_msgs::msg::VehicleOdometry> msg) {

    iii_drone::adapters::px4::VehicleOdometryAdapter adapter(*msg);

    geometry_msgs::msg::TransformStamped t = adapter.ToTransformStamped(
        configurator_.GetParameter("drone_frame_id").as_string(),
        configurator_.GetParameter("world_frame_id").as_string()
    );

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