/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/perception/pl_dir_computer_node/pl_dir_computer_node.hpp>

using namespace iii_drone::perception::pl_dir_computer_node;
using namespace iii_drone::types;
using namespace iii_drone::math;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

PowerlineDirectionComputerNode::PowerlineDirectionComputerNode(
    const std::string & node_name, 
    const std::string & node_namespace,
    const rclcpp::NodeOptions & options
) : rclcpp::Node(
    node_name, 
    node_namespace,
    options
),  configurator_(this), 
    pl_direction_(configurator_.powerline_direction_parameters()) {

    pl_angle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/perception/hough_transformer/cable_yaw_angle", 
        10, 
        [this](const std_msgs::msg::Float32::SharedPtr msg) {
            pl_direction_.Update(msg->data);
        }
    );

    pl_sub_ = this->create_subscription<iii_drone_interfaces::msg::Powerline>(
        "/perception/pl_mapper/powerline", 
        10, 
        std::bind(
            &PowerlineDirection::UpdatePowerlineAdapter, 
            &pl_direction_,
            std::placeholders::_1
        )
    );

    pl_direction_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "powerline_direction_pose", 
        10
    );

    pl_direction_quat_pub_ = this->create_publisher<geometry_msgs::msg::QuaternionStamped>(
        "powerline_direction_quat", 
        10
    );

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    std::chrono::milliseconds sleep_ms(configurator_.init_sleep_time_ms());
	rclcpp::Rate rate(sleep_ms);
	rate.sleep();

    std::chrono::milliseconds odom_callback_ms(configurator_.odometry_callback_period_ms());
    drone_tf_timer_ = this->create_wall_timer(
        odom_callback_ms, 
        std::bind(
            &PowerlineDirectionComputerNode::odometryCallback, 
            this
        )
    );

    RCLCPP_DEBUG(this->get_logger(), "Initialized PowerlineDirectionComputerNode");

}

void PowerlineDirectionComputerNode::odometryCallback() {

    RCLCPP_DEBUG(this->get_logger(), "Fetching odometry transform");

    geometry_msgs::msg::TransformStamped tf;

    try {

        tf = tf_buffer_->lookupTransform(
            configurator_.drone_frame_id(),
            configurator_.world_frame_id(), 
            tf2::TimePointZero
        );

    } catch(tf2::TransformException & ex) {

        RCLCPP_WARN(this->get_logger(), "Could not get odometry transform, frame world to drone");
        return;

    }

    quaternion_t quat = quaternionFromTransformMsg(tf.transform);

    pl_direction_.Predict(quat);

    publishPowerlineDirection();

}

void PowerlineDirectionComputerNode::publishPowerlineDirection() {

    RCLCPP_DEBUG(this->get_logger(), "Publishing powerline direction");

    geometry_msgs::msg::PoseStamped pose_msg = pl_direction_.ToPoseStampedMsg(configurator_.drone_frame_id());

    pl_direction_pose_pub_->publish(pose_msg);
    pl_direction_quat_pub_->publish(pl_direction_.ToQuaternionStampedMsg(configurator_.drone_frame_id()));

}

int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);

    auto node = std::make_shared<PowerlineDirectionComputerNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;

}
