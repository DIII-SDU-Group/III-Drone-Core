#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <string>
#include <iostream>
#include <chrono>
#include <mutex>
#include <fstream>

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

#include "iii_interfaces/msg/powerline_direction.hpp"
#include "iii_interfaces/msg/powerline.hpp"

#include "geometry.h"

using namespace std::chrono_literals;

/*****************************************************************************/
// Defines
/*****************************************************************************/

typedef struct {

    float state_est;
    float var_est;

} kf_est_t;

/*****************************************************************************/
// Class
/*****************************************************************************/

class PowerlineDirectionComputerNode : public rclcpp::Node {
public:
explicit
    PowerlineDirectionComputerNode(const std::string & node_name="pl_dir_computer", const std::string & node_namespace="/pl_dir_computer");

private:
    rclcpp::Subscription<iii_interfaces::msg::PowerlineDirection>::SharedPtr pl_direction_sub_;
    rclcpp::Subscription<iii_interfaces::msg::Powerline>::SharedPtr pl_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pl_direction_pub_;

    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::TimerBase::SharedPtr drone_tf_timer_{nullptr};

    std::string world_frame_id_, drone_frame_id_, mmwave_frame_id_;

    bool received_angle = false;
    bool received_first_quat = false;
    bool received_second_quat = false;

    quat_t drone_quat_, last_drone_quat_;

    iii_interfaces::msg::Powerline pl_;

    float r_, q_;

    quat_t pl_direction_;

    float W_pl_yaw_ = 0.;

    kf_est_t pl_angle_est[3];

    std::mutex direction_mutex_;
    std::mutex kf_mutex_;
    std::mutex pl_mutex_;

    void odometryCallback();
    void plDirectionCallback(const iii_interfaces::msg::PowerlineDirection::SharedPtr msg);

    void plCallback(const iii_interfaces::msg::Powerline::SharedPtr msg);

    void predict();
    void update(float angle);

    void publishPowerlineDirection();

    float mapAngle(float curr_angle, float new_angle);
    float mapAngle2(float curr_angle, float new_angle);
    float backmapAngle(float angle);

    bool anyCableInFOV();

    //std::ofstream file;
    //std::mutex file_mutex;

};

/*****************************************************************************/
// Main
/*****************************************************************************/

int main(int argc, char *argv[]);
