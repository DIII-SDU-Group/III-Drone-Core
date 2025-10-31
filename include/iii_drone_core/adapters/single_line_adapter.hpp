#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <tf2_ros/buffer.h>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/types.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/msg/single_line.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace adapters {

    /**
     * @brief SingleLine message adapter class
     */
    class SingleLineAdapter {
    public:

        /**
         * @brief Default constructor.
         */
        SingleLineAdapter();

        /**
         * @brief Constructor from msg.
         * 
         * @param msg The SingleLine
         */
        SingleLineAdapter(const iii_drone_interfaces::msg::SingleLine& msg);

        /**
         * @brief Constructor from members.
         * 
         * @param stamp The timestamp.
         * @param frame_id The frame id.
         * @param id The id of the line.
         * @param position The position of the line.
         * @param projected_position The position of the line projected onto the projection plane.
         * @param quaternion The quaternion of the line.
         * @param in_fov Whether the line is in the FOV.
         */
       SingleLineAdapter(
            const rclcpp::Time & stamp,
            const std::string & frame_id,
            const int & id,
            const iii_drone::types::point_t & position,
            const iii_drone::types::point_t & projected_position,
            const iii_drone::types::quaternion_t & quaternion,
            const bool & in_fov
        );

        /**
         * @brief Constructor from pose msg.
         * 
         * @param stamp The timestamp.
         * @param frame_id The frame id.
         * @param id The id of the line.
         * @param msg The pose msg.
         * @param in_fov Whether the line is in the FOV.
         */
        SingleLineAdapter(
            const rclcpp::Time & stamp,
            const std::string & frame_id,
            const int & id,
            const geometry_msgs::msg::Pose & msg,
            const bool & in_fov
        );

        /**
         * @brief Constructor from PoseStamped msg.
         * 
         * @param id The id of the line.
         * @param msg The PoseStamped msg.
         * @param in_fov Whether the line is in the FOV.
         */
        SingleLineAdapter(
            const int & id,
            const geometry_msgs::msg::PoseStamped & msg,
            const bool & in_fov
        );

        /**
         * @brief Converts to SingleLine msg.
         */
        const iii_drone_interfaces::msg::SingleLine ToMsg() const;

        /**
         * @brief Converts to Pose msg.
         */
        const geometry_msgs::msg::Pose ToPoseMsg() const;

        /**
         * @brief Converts to PoseStamped msg.
         * 
         * @param frame_id The frame id of the PoseStamped.
         */
        const geometry_msgs::msg::PoseStamped ToPoseStampedMsg() const;

        void Transform(const std::string & target_frame_id, std::shared_ptr<tf2_ros::Buffer> tf_buffer);

        /**
         * @brief Timestamp getter.
         */
        const rclcpp::Time & stamp() const;

        /**
         * @brief Frame id getter.
         */
        const std::string & frame_id() const;

        /**
         * @brief Id getter.
         */
        const int & id() const;

        /**
         * @brief position getter.
         */
        const iii_drone::types::point_t & position() const;

        /**
         * @brief Projected position getter.
         */
        const iii_drone::types::point_t & projected_position() const;

        /**
         * @brief Quaternion getter.
         */
        const iii_drone::types::quaternion_t & quaternion() const;

        /**
         * @brief In FOV getter.
         */
        const bool & in_fov() const;

    private:
        /**
        * @brief The timestamp.
        */
        rclcpp::Time stamp_;

        /**
         * @brief The frame id.
         */
        std::string frame_id_;

        /**
        * @brief The id of the line.
        */
        int id_;

        /**
        * @brief The position of the line.
        */
        iii_drone::types::point_t position_;

        /**
         * @brief The position of the line projected onto the projection plane.
         */
        iii_drone::types::point_t projected_position_;

        /**
        * @brief The quaternion of the line.
        */
        iii_drone::types::quaternion_t quaternion_;

        /**
        * @brief Whether the line is in the FOV.
        */
        bool in_fov_;

    };

} // namespace adapters
} // namespace iii_drone