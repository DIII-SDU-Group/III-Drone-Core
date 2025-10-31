#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

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
// III-Drone-Core:

#include <iii_drone_core/utils/types.hpp>
#include <iii_drone_core/utils/math.hpp>

#include <iii_drone_core/control/reference.hpp>
#include <iii_drone_core/control/state.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace adapters {
namespace px4 {

    /*
    * @brief PX4 trajectory setpoint adapter class
    */
    class TrajectorySetpointAdapter {
    public:
        /*
        * @brief Default constructor
        */
        TrajectorySetpointAdapter();

        /*
        * @brief Constructor
        *
        * @param stamp ROS2 time stamp
        */
        TrajectorySetpointAdapter(const rclcpp::Time & stamp);

        /*
        * @brief Constructor from iii::control::Reference
        *
        * @param reference iii_drone::control::Reference
        */
        TrajectorySetpointAdapter(const iii_drone::control::Reference & reference);

        /*
        * @brief Constructor from iii_drone::control::State
        *
        * @param state iii_drone::control::State
        */
        TrajectorySetpointAdapter(const iii_drone::control::State & state);

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
        const iii_drone::types::point_t & position() const;

        /*
        * @brief Velocity getter
        *
        * @return Setpoint velocity
        */
        const iii_drone::types::vector_t & velocity() const;

        /*
        * @brief Acceleration getter
        *
        * @return Setpoint acceleration
        */
        const iii_drone::types::vector_t & acceleration() const;

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
            const iii_drone::types::point_t & position,
            const iii_drone::types::vector_t & velocity,
            const iii_drone::types::vector_t & acceleration,
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
        iii_drone::types::point_t position_;

        /*
        * @brief Setpoint velocity
        */
        iii_drone::types::vector_t velocity_;

        /*
        * @brief Setpoint acceleration
        */
        iii_drone::types::vector_t acceleration_;

        /*
        * @brief Setpoint yaw
        */
        double yaw_;

        /*
        * @brief Setpoint yaw speed
        */
        double yawspeed_;


    };
}}}