#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Standard includes:

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

/*****************************************************************************/
// Eigen:

#include <Eigen/Geometry>
#include <Eigen/Dense>

/*****************************************************************************/
// PX4:

#include <px4_msgs/msg/trajectory_setpoint.hpp>

/*****************************************************************************/
// III:

#include <iii/iii.hpp>
#include <iii/math.hpp>
#include <iii/control/reference.hpp>
#include <iii/control/state.hpp>

/*****************************************************************************/
// Package includes:

#include <iii_ros2/control/flight_control_reference.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_ros2 {
namespace px4 {

    /*
    * @brief PX4 trajectory setpoint adapter class
    */
    class TrajectorySetpoint {
    public:
        /*
        * @brief Default constructor
        */
        TrajectorySetpoint();

        /*
        * @brief Constructor
        *
        * @param stamp ROS2 time stamp
        */
        TrajectorySetpoint(const rclcpp::Time & stamp);

        /*
        * @brief Constructor from iii::control::Reference
        *
        * @param stamp ROS2 time stamp
        * @param reference iii::control::Reference
        */
        TrajectorySetpoint(
            const rclcpp::Time & stamp,
            const iii::control::Reference & reference
        );

        /*
        * @brief Constructor from iii_ros2::control::FlightControlReference
        *
        * @param stamp ROS2 time stamp
        * @param flight_control_reference iii_ros2::control::FlightControlReference
        */
        TrajectorySetpoint(
            const rclcpp::Time & stamp,
            const iii_ros2::control::FlightControlReference & flight_control_reference
        );

        /*
        * @brief Constructor from iii::control::State
        *
        * @param stamp ROS2 time stamp
        * @param state iii::control::State
        */
        TrajectorySetpoint(
            const rclcpp::Time & stamp,
            const iii::control::State & state
        );

        /**
         * @brief Sets velocity and acceleration to NAN
         * 
         * @return void
         */
        void OnlyPosition();

        /*
        * @brief Convert the object to a PX4 trajectory setpoint message
        *
        * @return PX4 trajectory setpoint message
        */
        px4_msgs::msg::TrajectorySetpoint ToMsg() const;

        /*
        * @brief Timestamp getter
        *
        * @return ROS2 time stamp
        */
        const rclcpp::Time & stamp() const;

        /*
        * @brief Position getter
        *
        * @return Setpoint position
        */
        const iii::types::position_t & position() const;

        /*
        * @brief Velocity getter
        *
        * @return Setpoint velocity
        */
        const iii::types::velocity_t & velocity() const;

        /*
        * @brief Acceleration getter
        *
        * @return Setpoint acceleration
        */
        const iii::types::acceleration_t & acceleration() const;

        /*
        * @brief Yaw getter
        *
        * @return Setpoint yaw
        */
        const double & yaw() const;

        /*
        * @brief Yaw speed getter
        *
        * @return Setpoint yaw speed
        */
        const double & yawspeed() const;

    private:
        /*
        * @brief Initialize the object
        *
        * @param stamp ROS2 time stamp
        * @param position Setpoint position
        * @param velocity Setpoint velocity
        * @param acceleration Setpoint acceleration
        * @param yaw Setpoint yaw
        * @param yawspeed Setpoint yaw speed
        * 
        * @return void
        */
        void init(
            const rclcpp::Time & stamp,
            const iii::types::position_t & position,
            const iii::types::velocity_t & velocity,
            const iii::types::acceleration_t & acceleration,
            const double & yaw,
            const double & yawspeed
        );

        /*
        * @brief ROS2 time stamp
        */
        rclcpp::Time stamp_;

        /*
        * @brief Setpoint position
        */
        iii::types::position_t position_;

        /*
        * @brief Setpoint velocity
        */
        iii::types::velocity_t velocity_;

        /*
        * @brief Setpoint acceleration
        */
        iii::types::acceleration_t acceleration_;

        /*
        * @brief Setpoint yaw
        */
        double yaw_;

        /*
        * @brief Setpoint yaw speed
        */
        double yawspeed_;


    };
}}