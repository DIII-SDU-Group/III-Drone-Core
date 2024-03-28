#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <mutex>
#include <shared_mutex>
#include <memory>
#include <atomic>

/*****************************************************************************/
// ROS2:

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
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/msg/powerline.hpp>
#include <iii_drone_interfaces/msg/pl_mapper_command.hpp>

#include <iii_drone_interfaces/srv/pl_mapper_command.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/math.hpp>
#include <iii_drone_core/utils/types.hpp>
#include <iii_drone_core/utils/atomic.hpp>

#include <iii_drone_core/configuration/configurator.hpp>

#include <iii_drone_core/perception/powerline.hpp>

#include <iii_drone_core/adapters/powerline_adapter.hpp>
#include <iii_drone_core/adapters/single_line_adapter.hpp>
#include <iii_drone_core/adapters/projection_plane_adapter.hpp>
#include <iii_drone_core/adapters/point_cloud_adapter.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace perception {
namespace pl_mapper_node {

/**
 * @brief Class for computing position of powerlines from mmWave data fused with powerline direction and odometry,
 * using Kalman filtering.
*/
class PowerlineMapperNode : public rclcpp::Node {
public:
explicit
    /**
     * @brief Constructor
     * 
     * @param node_name Name of the node, default is "pl_mapper"
     * @param node_namespace Namespace of the node, default is "/perception/pl_mapper"
     * @param options Node options, default is rclcpp::NodeOptions()
    */
    PowerlineMapperNode(
        const std::string & node_name="pl_mapper", 
        const std::string & node_namespace="/perception/pl_mapper",
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions()
    );

private:
    /**
     * @brief Configurator for the node
    */
    iii_drone::configuration::Configurator configurator_;

    /**
     * @brief Subscriber for powerline direction
    */
    rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr pl_direction_sub_;

    /**
     * @brief Subscriber for mmwave point cloud data
    */
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mmwave_sub_;

    /**
     * @brief Publisher for powerline message
    */
    rclcpp::Publisher<iii_drone_interfaces::msg::Powerline>::SharedPtr powerline_pub_;

    /**
     * @brief Publisher for estimated powerline points
    */
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_est_pub_;

    /**
     * @brief Publisher for transformed mmwave points
    */
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_points_pub_;

    /**
     * @brief Publisher for mmwave points projected onto plane
    */
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr projected_points_pub_;

    /**
     * @brief PL-mapper state type.
     */
    enum pl_mapper_state_t {
        pl_mapper_state_idle,
        pl_mapper_state_paused,
        pl_mapper_state_running
    };

    /**
     * @brief Current state of the PL-mapper.
     */
    iii_drone::utils::Atomic<pl_mapper_state_t> pl_mapper_state_;

    /**
     * @brief PL-mapper command service.
     */
    rclcpp::Service<iii_drone_interfaces::srv::PLMapperCommand>::SharedPtr pl_mapper_command_srv_;

    /**
     * @brief PL-mapper command service callback.
     * 
     * @param request Request message
     * @param response Response message
     * 
     * @return void
     */
    void plMapperCommandCallback(
        const std::shared_ptr<iii_drone_interfaces::srv::PLMapperCommand::Request> request,
        std::shared_ptr<iii_drone_interfaces::srv::PLMapperCommand::Response> response
    );

    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    /**
     * @brief Timer for fetching drone transform
    */
    rclcpp::TimerBase::SharedPtr drone_tf_timer_{nullptr};

    /**
     * @brief The powerline object holding the tracked powerlines
    */
    Powerline::SharedPtr powerline_;

    /**
     * @brief Rotation matrix from drone to mmWave frame
    */
    iii_drone::types::rotation_matrix_t R_drone_to_mmw_;

    /**
     * @brief Translation vector from drone to mmWave frame
    */
    iii_drone::types::vector_t v_drone_to_mmw_;

    /**
     * @brief Direction of detected powerline
    */
    iii_drone::types::quaternion_t pl_direction_;

    /**
     * @brief Callback for fetching drone transform
     * 
     * @return void
    */
    void odometryCallback();

    /**
     * @brief Callback for mmwave data topic
     * 
     * @param msg Message containing the mmwave point cloud data
     * 
     * @return void
    */
    void mmWaveCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    /**
     * @brief Callback for powerline direction topic
     * 
     * @param msg Message containing the powerline direction
     * 
     * @return void
    */
    void plDirectionCallback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg);

    /**
     * @brief Publishes the powerline message
     * 
     * @param powerline Powerline object
     * 
     * @return void
    */
    void publishPowerline(const Powerline & powerline) const;

    /**
     * @brief Publishes point cloud data on a given publisher
     * 
     * @param points Vector of points to publish
     * @param pub Publisher to publish on
     * 
     * @return void
    */
    void publishPoints(
        std::vector<iii_drone::types::point_t> points, 
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub
    ) const;

};

} // namespace pl_mapper_node
} // namespace perception
} // namespace iii_drone

/*****************************************************************************/
// Main
/*****************************************************************************/

int main(int argc, char *argv[]);
