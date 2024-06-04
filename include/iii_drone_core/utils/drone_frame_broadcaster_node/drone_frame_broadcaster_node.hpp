#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <memory>
#include <string>

#include <iii_drone_core/utils/types.hpp>
#include <iii_drone_core/utils/math.hpp>

#include <iii_drone_configuration/configurator.hpp>

#include <iii_drone_core/adapters/px4/vehicle_odometry_adapter.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace utils {
namespace drone_frame_broadcaster_node {

    class DroneFrameBroadcasterNode : public rclcpp::Node {
    public:
    explicit
        DroneFrameBroadcasterNode(
            const std::string & node_name="drone_frame_broadcaster", 
            const std::string & node_namespace="/drone_frame_broadcaster",
            const rclcpp::NodeOptions & options = rclcpp::NodeOptions()
        );

    private:
        void odometryCallback(const std::shared_ptr<px4_msgs::msg::VehicleOdometry> msg);

        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        iii_drone::types::rotation_matrix_t R_NED_to_body_frame;

        iii_drone::configuration::Configurator<rclcpp::Node>::SharedPtr configurator_;

    };

} // namespace drone_frame_broadcaster_node
} // namespace utils
} // namespace iii_drone

/*****************************************************************************/
// Main
/*****************************************************************************/

int main(int argc, char * argv[]);