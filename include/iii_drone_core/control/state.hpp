#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/types.hpp>
#include <iii_drone_core/utils/math.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {

namespace control {

    /**
     * Class representing a drone state.
     */
    class State {
    public:
        /**
         * Constructor.
         *
         * @param position The position.
         * @param velocity The velocity.
         * @param quaternion The quaternion.
         * @param angular_velocity The angular velocity.
         * @param stamp The timestamp, defaults to now.
         */
        State(
            const iii_drone::types::point_t position,
            const iii_drone::types::vector_t velocity,
            const iii_drone::types::quaternion_t quaternion,
            const iii_drone::types::vector_t angular_velocity,
            const rclcpp::Time stamp = rclcpp::Clock().now()
        );

        /**
         * Constructor.
         *
         * @param position The position.
         * @param velocity The velocity.
         * @param yaw The yaw.
         * @param angular_velocity The angular velocity.
         * @param stamp The timestamp, defaults to now.
         */
        State(
            const iii_drone::types::point_t position,
            const iii_drone::types::vector_t velocity,
            double yaw,
            const iii_drone::types::vector_t angular_velocity,
            const rclcpp::Time stamp = rclcpp::Clock().now()
        );

        /**
         * Default constructor for empty object.
         */
        State();

        /**
         * @brief Sets velocities to NAN.
         * 
         * @return State&
         */
        State& ApplyNans();

        /**
         * Getter for the position.
         *
         * @return The position.
         */
        const iii_drone::types::point_t position() const;

        /**
         * Getter for the velocity.
         *
         * @return The velocity.
         */
        const iii_drone::types::vector_t velocity() const;

        /**
         * Getter for the quaternion.
         *
         * @return The quaternion.
         */
        const iii_drone::types::quaternion_t quaternion() const;

        /**
         * Getter for the yaw.
         * 
         * @return The yaw.
         */
        double yaw() const;

        /**
         * Getter for the angular velocity.
         *
         * @return The angular velocity.
         */
        const iii_drone::types::vector_t angular_velocity() const;

        /**
         * Getter for the timestamp.
         */
        const rclcpp::Time stamp() const;

    private:
        /**
         * The position.
         */
        iii_drone::types::point_t position_;

        /**
         * The velocity.
         */
        iii_drone::types::vector_t velocity_;

        /**
         * The quaternion.
         */
        iii_drone::types::quaternion_t quaternion_;

        /**
         * The yaw.
         */
        double yaw_;

        /**
         * The angular velocity.
         */
        iii_drone::types::vector_t angular_velocity_;

        /**
         * The timestamp.
         */
        rclcpp::Time stamp_;
    };

} // namespace iii_drone::control
} // namespace iii
