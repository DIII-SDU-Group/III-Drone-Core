#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <mutex>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "iii_drone_interfaces/msg/powerline_direction.hpp"
#include "iii_drone_interfaces/msg/powerline.hpp"
#include "iii_drone_interfaces/msg/control_state.hpp"

#include "iii_drone_core/perception/powerline.hpp"
#include "iii_drone_core/utils/math.hpp"
#include "iii_drone_core/utils/types.hpp"

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace perception {
namespace pl_mapper_node {

class PowerlineMapperNode : public rclcpp::Node {
public:
explicit
    PowerlineMapperNode(
        const std::string & node_name="pl_mapper", 
        const std::string & node_namespace="/perception/pl_mapper",
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions()
    );

private:
    bool simulation_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pl_direction_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mmwave_sub_;

    rclcpp::Subscription<iii_drone_interfaces::msg::ControlState>::SharedPtr control_state_sub_;

    rclcpp::Publisher<iii_drone_interfaces::msg::Powerline>::SharedPtr powerline_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_est_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_points_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr projected_points_pub_;

    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::TimerBase::SharedPtr drone_tf_timer_{nullptr};

    std::string world_frame_id_, drone_frame_id_, mmwave_frame_id_;

    int max_lines_;

    float r_, q_;

    int alive_cnt_low_thresh_, alive_cnt_high_thresh_, alive_cnt_ceiling_;

    float matching_line_max_dist_;

    Powerline powerline_;

    iii_drone::types::rotation_matrix_t R_drone_to_mmw;
    iii_drone::types::vector_t v_drone_to_mmw;
    iii_drone::types::quat_t pl_direction_; 

    std::mutex control_state_mutex_;
    iii_drone_interfaces::msg::ControlState control_state_;

    void controlStateCallback(const iii_drone_interfaces::msg::ControlState::SharedPtr msg);
    void odometryCallback();
    void mmWaveCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void plDirectionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void publishPowerline();
    void publishPoints(
        std::vector<iii_drone::types::point_t> points, 
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub
    );

};

} // namespace pl_mapper_node
} // namespace perception
} // namespace iii_drone

/*****************************************************************************/
// Main
/*****************************************************************************/

int main(int argc, char *argv[]);
