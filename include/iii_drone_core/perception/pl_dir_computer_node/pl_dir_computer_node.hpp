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

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/*****************************************************************************/
// III-Drone-Interfaces:

#include "iii_drone_interfaces/msg/powerline_direction.hpp"
#include "iii_drone_interfaces/msg/powerline.hpp"

/*****************************************************************************/
// III-Drone-Core:

#include "iii_drone_core/utils/math.hpp"
#include "iii_drone_core/utils/types.hpp"

#include "iii_drone_core/perception/pl_dir_computer_node/pl_dir_computer_node_configurator.hpp"

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
        PowerlineDirectionComputerConfigurator configurator_;

        /**
         * @brief Struct for holding Kalman filtering data
         */
        typedef struct {

            float state_est;
            float var_est;

        } kf_est_t;

        /**
         * @brief Subscription object to PowerlineDirection topic
         */
        rclcpp::Subscription<iii_drone_interfaces::msg::PowerlineDirection>::SharedPtr pl_direction_sub_;

        /**
         * @brief Subscription object to powerline topic
         */
        rclcpp::Subscription<iii_drone_interfaces::msg::Powerline>::SharedPtr pl_sub_;

        /**
         * @brief Estimated powerline direction publisher
         */
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pl_direction_pub_;

        std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        rclcpp::TimerBase::SharedPtr drone_tf_timer_{nullptr};

        bool received_angle = false;
        bool received_first_quat = false;
        bool received_second_quat = false;

        iii_drone::types::quat_t drone_quat_, last_drone_quat_;

        iii_drone_interfaces::msg::Powerline pl_;

        iii_drone::types::quat_t pl_direction_;

        float W_pl_yaw_ = 0.;

        kf_est_t pl_angle_est[3];

        std::mutex direction_mutex_;
        std::mutex kf_mutex_;
        std::mutex pl_mutex_;

        /**
         * @brief Callback function for the odometry timer
         * 
         * @return void
         */
        void odometryCallback();

        /**
         * @brief Callback function for the powerline direction subscription
         * 
         * @param msg Message containing the powerline direction
         * 
         * @return void
         */
        void plDirectionCallback(const iii_drone_interfaces::msg::PowerlineDirection::SharedPtr msg);

        /**
         * @brief Callback function for the powerline subscription
         * 
         * @param msg Message containing the powerline
         * 
         * @return void
         */
        void plCallback(const iii_drone_interfaces::msg::Powerline::SharedPtr msg);

        /**
         * @brief Performs the predition step of the Kalman filter
         * 
         * @return void
         */
        void predict();

        /**
         * @brief Performs the update step of the Kalman filter
         * 
         * @param angle Angle to update the Kalman filter with
         * 
         * @return void
         */
        void update(float angle);

        /**
         * @brief Publishes the estimated powerline direction
         * 
         * @return void
         */
        void publishPowerlineDirection();

        /**
         * @brief Maps the angle into continuous range based on current angle
         * 
         * @param curr_angle Current angle
         * @param new_angle New angle
         * 
         * @return float Mapped angle
         */
        float mapAngle(
            float curr_angle, 
            float new_angle
        );

        /**
         * @brief Maps the angle into continuous range based on current angle
         * 
         * @param curr_angle Current angle
         * @param new_angle New angle
         * 
         * @return float Mapped angle
         */
        float mapAngle2(
            float curr_angle, 
            float new_angle
        );

        /**
         * @brief Maps the angle back into the correct range after having performed the Kalman filter
         * 
         * @param angle Angle to map
         * 
         * @return float Back mapped angle
         */
        float backmapAngle(float angle);

        /**
         * @brief Checks if any cable is in the field of view
         * 
         * @return bool True if any cable is in the field of view
         */
        bool anyCableInFOV();

    };

} // namespace pl_dir_computer_node
} // namespace perception
} // namespace iii_drone

/*****************************************************************************/
// Main
/*****************************************************************************/

int main(int argc, char *argv[]);
