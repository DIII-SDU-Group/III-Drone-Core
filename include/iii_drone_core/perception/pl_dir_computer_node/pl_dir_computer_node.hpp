#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <string>
#include <iostream>
#include <chrono>
#include <mutex>
#include <fstream>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <std_msgs/msg/float32.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/msg/powerline.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/math.hpp>
#include <iii_drone_core/utils/types.hpp>

#include <iii_drone_core/configuration/configurator.hpp>

#include <iii_drone_core/perception/powerline_direction.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace perception {
namespace pl_dir_computer_node {

    /**
     * @brief Node for kalman filtering the powerline direction based on hough transform output.
    */
    class PowerlineDirectionComputerNode : public rclcpp::Node {
    public:
    explicit
        /**
         * @brief Construct a new Powerline Direction Computer Node object
         * 
         * @param node_name Name of the node
         * @param node_namespace Namespace of the node
         * @param options Node options
         */
        PowerlineDirectionComputerNode(
            const std::string & node_name="pl_dir_computer", 
            const std::string & node_namespace="/perception/pl_dir_computer",
            const rclcpp::NodeOptions & options = rclcpp::NodeOptions()
        );

    private:
        /**
         * @brief Configurator object
         */
        iii_drone::configuration::Configurator configurator_;

        /**
         * @brief Powerline direction object
         */
        PowerlineDirection pl_direction_;

        /**
         * @brief Subscription object to powerline yaw angle
         */
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pl_angle_sub_;

        /**
         * @brief Subscription object to powerline topic
         */
        rclcpp::Subscription<iii_drone_interfaces::msg::Powerline>::SharedPtr pl_sub_;

        /**
         * @brief Estimated powerline direction pose publisher
         */
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pl_direction_pose_pub_;

        /**
         * @brief Estimated powerline direction quaternion publisher.
         */
        rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr pl_direction_quat_pub_;

        std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        rclcpp::TimerBase::SharedPtr drone_tf_timer_{nullptr};

        /**
         * @brief Callback function for the odometry timer
         * 
         * @return void
         */
        void odometryCallback();

        /**
         * @brief Publishes the estimated powerline direction
         * 
         * @return void
         */
        void publishPowerlineDirection();

    };

} // namespace pl_dir_computer_node
} // namespace perception
} // namespace iii_drone

/*****************************************************************************/
// Main
/*****************************************************************************/

int main(int argc, char *argv[]);
