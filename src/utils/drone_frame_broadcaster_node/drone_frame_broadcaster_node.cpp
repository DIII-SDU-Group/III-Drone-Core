/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "iii_drone_core/utils/drone_frame_broadcaster_node/drone_frame_broadcaster_node.hpp"

using namespace iii_drone::utils::drone_frame_broadcaster_node;
using namespace iii_drone::math;
using namespace iii_drone::types;

namespace {

void DeclareManagedParameters(iii_drone::configuration::Configurator<rclcpp::Node> & configurator)
{
    configurator.DeclareParameter("/tf/drone_frame_id", rclcpp::ParameterType::PARAMETER_STRING);
    configurator.DeclareParameter("/tf/world_frame_id", rclcpp::ParameterType::PARAMETER_STRING);
}

}  // namespace

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
// ), configurator_(this) {
) {

    RCLCPP_DEBUG(this->get_logger(), "DroneFrameBroadcasterNode::DroneFrameBroadcasterNode(): Constructor");

    configurator_ = std::make_shared<iii_drone::configuration::Configurator<rclcpp::Node>>(this, "drone_frame_broadcaster");
    DeclareManagedParameters(*configurator_);
    configurator_->validate();

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    std::ostringstream stream;

    is_alive_publisher_ = this->create_publisher<std_msgs::msg::Header>(
        "is_alive", 
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().best_effort()
    );

    last_alive_pub_time_ = this->get_clock()->now();
    publishIsAlive();
    is_alive_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&DroneFrameBroadcasterNode::publishIsAlive, this)
    );

    rclcpp::SensorDataQoS sub_qos;

    subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", 
        sub_qos,
        std::bind(&DroneFrameBroadcasterNode::odometryCallback, this, std::placeholders::_1));

    R_NED_to_body_frame = eulToMat(euler_angles_t(M_PI, 0, 0));

    RCLCPP_INFO(this->get_logger(), "DroneFrameBroadcasterNode::DroneFrameBroadcasterNode(): Initialized");

}

void DroneFrameBroadcasterNode::odometryCallback(const std::shared_ptr<px4_msgs::msg::VehicleOdometry> msg) {

    RCLCPP_DEBUG(this->get_logger(), "DroneFrameBroadcasterNode::odometryCallback(): Received odometry message");

    iii_drone::adapters::px4::VehicleOdometryAdapter adapter(*msg);

    geometry_msgs::msg::TransformStamped t = adapter.ToTransformStamped(
        configurator_->GetParameter("/tf/drone_frame_id").as_string(),
        configurator_->GetParameter("/tf/world_frame_id").as_string()
    );

    // PX4 timestamps are boot-relative; TF needs ROS time so buffers reset correctly on sim/PX4 restarts.
    t.header.stamp = this->get_clock()->now();

    // Send the transformation
    tf_broadcaster_->sendTransform(t);

    // RCLCPP debug published transform
    RCLCPP_DEBUG(this->get_logger(), "Published transform: %s -> %s", t.header.frame_id.c_str(), t.child_frame_id.c_str());

}

void DroneFrameBroadcasterNode::publishIsAlive() {
    std_msgs::msg::Header header;
    header.stamp = this->get_clock()->now();
    is_alive_publisher_->publish(header);
    last_alive_pub_time_ = header.stamp;

    RCLCPP_DEBUG(this->get_logger(), "DroneFrameBroadcasterNode::publishIsAlive(): Published heartbeat");
}

int main(int argc, char * argv[]) {

    rclcpp::init(argc, argv);

    auto multi_threaded_executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto node = std::make_shared<DroneFrameBroadcasterNode>();
    RCLCPP_DEBUG(node->get_logger(), "DroneFrameBroadcasterNode::main(): Node created");
    multi_threaded_executor->add_node(node);
    multi_threaded_executor->spin();
    rclcpp::shutdown();
    return 0;

}
