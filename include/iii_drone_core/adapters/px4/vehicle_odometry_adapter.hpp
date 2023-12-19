#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

/*****************************************************************************/
// Eigen:

#include <Eigen/Dense>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

/*****************************************************************************/
// PX4:

#include <px4_msgs/msg/vehicle_odometry.hpp>

/*****************************************************************************/
// III-Drone-Core:

// #include <iii/iii.hpp>
// #include <iii/control/state.hpp>

#include <iii_drone_core/utils/types.hpp>
#include <iii_drone_core/utils/math.hpp>

/*****************************************************************************/
// Package includes:

/*****************************************************************************/
// Enums
/*****************************************************************************/

namespace iii_drone {
namespace adapters {
namespace px4 {

    /**
     * @brief PX4 odometry pose frame enum type
     */
    typedef enum {
        POSE_FRAME_UNKNOWN = 0,
        POSE_FRAME_LOCAL_NED = 1,
        POSE_FRAME_LOCAL_FRD = 2
    } pose_frame_t;

    /**
     * @brief PX4 odometry velocity frame enum type
     */
    typedef enum {
        VELOCITY_FRAME_UNKNOWN = 0,
        VELOCITY_FRAME_LOCAL_NED = 1,
        VELOCITY_FRAME_LOCAL_FRD = 2,
        VELOCITY_FRAME_BODY_FRD = 3
    } velocity_frame_t;

}}}

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace adapters {
namespace px4 {

    /**
     * @brief PX4 odometry adapter class
     */
    class VehicleOdometryAdapter {
    public:
        /**
         * @brief Default constructor
        */
        VehicleOdometryAdapter();

        /* * 
         * @brief Constructor
         *
         * @param vehicle_odometry_msg PX4 odometry message
         */
        VehicleOdometryAdapter(const px4_msgs::msg::VehicleOdometry & vehicle_odometry_msg);

        /**
         * @brief Updates the object from a new PX4 odometry message
         *
         * @param vehicle_odometry_msg PX4 odometry message
         * 
         * @return void
         */
        void UpdateFromMsg(const px4_msgs::msg::VehicleOdometry & vehicle_odometry_msg);

        // /**
        //  * @brief Converts the object to a iii::control::State object
        //  *
        //  * @return iii::control::State object
        //  */
        // iii::control::State ToState() const;

        /**
         * @brief Converts the object to a geometry_msgs::msg::TransformStamped object.
         * 
         * @param drone_frame_id The drone frame id.
         * @param world_frame_id The world frame id.
         * 
         * @return geometry_msgs::msg::TransformStamped object
         */
        const geometry_msgs::msg::TransformStamped ToTransformStamped(
            const std::string & drone_frame_id,
            const std::string & world_frame_id
        ) const;

        /**
         * @brief Timestamp getter
         *
         * @return ROS2 timestamp
         */
        const rclcpp::Time & stamp() const;

        /**
         * @brief Pose frame getter
         *
         * @return Pose frame
         */
        const pose_frame_t & pose_frame() const;

        /**
         * @brief Position getter
         *
         * @return Position
         */
        const iii_drone::types::point_t & position() const;

        /**
         * @brief Quaternion getter
         *
         * @return Quaternion
         */
        const iii_drone::types::quaternion_t & quaternion() const;

        /**
         * @brief Velocity frame getter
         *
         * @return Velocity frame
         */
        const velocity_frame_t & velocity_frame() const;

        /**
         * @brief Velocity getter
         *
         * @return Velocity
         */
        const iii_drone::types::vector_t & velocity() const;

        /**
         * @brief Angular velocity getter
         *
         * @return Angular velocity
         */
        const iii_drone::types::vector_t & angular_velocity() const;

        /**
         * @brief Position variance getter
         *
         * @return Position variance
         */
        const Eigen::Vector3d & position_variance() const;

        /**
         * @brief Orientation variance getter
         *
         * @return Orientation variance
         */
        const Eigen::Vector3d & orientation_variance() const;

        /**
         * @brief Velocity variance getter
         *
         * @return Velocity variance
         */
        const Eigen::Vector3d & velocity_variance() const;

    private:
        /**
         * @brief ROS2 timestamp
         */
        rclcpp::Time stamp_;

        /**
         * @brief PX4 odometry pose frame
         */
        pose_frame_t pose_frame_;

        /**
         * @brief PX4 odometry position
         */
        iii_drone::types::point_t position_;

        /**
         * @brief PX4 odometry quaternion
         */
        iii_drone::types::quaternion_t quaternion_;

        /**
         * @brief PX4 odometry velocity frame
         */
        velocity_frame_t velocity_frame_;

        /**
         * @brief PX4 odometry velocity
         */
        iii_drone::types::vector_t velocity_;

        /**
         * @brief PX4 odometry angular velocity
         */
        iii_drone::types::vector_t angular_velocity_;

        /**
         * @brief PX4 odometry position variance
         */
        Eigen::Vector3d position_variance_;

        /**
         * @brief PX4 odometry orientation variance
         */
        Eigen::Vector3d orientation_variance_;

        /**
         * @brief PX4 odometry velocity variance
         */
        Eigen::Vector3d velocity_variance_;

    };
}}}