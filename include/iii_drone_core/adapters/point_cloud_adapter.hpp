#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <vector>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/types.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace adapters {

    /**
     * @brief Adapter class for PointCloud2 messages.
     */
    class PointCloudAdapter {
    public:
        /**
         * @brief Default constructor.
         */
        PointCloudAdapter();

        /**
         * @brief Constructor.
         * 
         * @param stamp Timestamp of the message.
         * @param frame_id Frame ID of the message.
         * @param points Point cloud data.
         */
        PointCloudAdapter(
            const rclcpp::Time & stamp,
            const std::string & frame_id,
            const std::vector<iii_drone::types::point_t> & points
        );

        /**
         * @brief Constructor from ROS2 message.
         * 
         * @param msg ROS2 message.
         */
        PointCloudAdapter(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        /**
         * @brief Convert to ROS2 message.
         * 
         * @return ROS2 message.
         */
        const sensor_msgs::msg::PointCloud2 ToMsg() const;

        /**
         * @brief Timestamp getter.
         */
        const rclcpp::Time & stamp() const;

        /**
         * @brief Frame ID getter.
         */
        const std::string & frame_id() const;

        /**
         * @brief Point cloud data getter.
         */
        const std::vector<iii_drone::types::point_t> & points() const;

    private:
        /**
         * @brief Timestamp of the message.
         */
        rclcpp::Time stamp_;

        /**
         * @brief Frame ID of the message.
         */
        std::string frame_id_;

        /**
         * @brief Point cloud data.
         */
        std::vector<iii_drone::types::point_t> points_;

        /**
         * @brief Reads points from a ROS2 message.
         * 
         * @param msg ROS2 message.
         */
        const std::vector<iii_drone::types::point_t> readPoints(const sensor_msgs::msg::PointCloud2 msg) const;

    };

} // namespace adapters
} // namespace iii_drone