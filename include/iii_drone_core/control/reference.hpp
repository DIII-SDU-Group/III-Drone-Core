#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/types.hpp>
#include <iii_drone_core/utils/math.hpp>

#include <iii_drone_core/control/state.hpp>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace control {

    /**
     * Class representing a control reference position and yaw.
     */
    class Reference {
    public:
        /**
         * Constructor.
         *
         * @param position The reference position.
         * @param yaw The reference yaw.
         * @param velocity The reference velocity, defaults to zero.
         * @param yaw_rate The reference yaw rate, defaults to zero.
         * @param acceleration The reference acceleration, defaults to zero.
         * @param yaw_acceleration The reference yaw acceleration, defaults to zero.
         * @param stamp The timestamp, defaults to now.
         */
        Reference(
            const iii_drone::types::point_t position,
            double yaw,
            const iii_drone::types::vector_t velocity = iii_drone::types::vector_t::Zero(),
            double yaw_rate = 0.0,
            const iii_drone::types::vector_t acceleration = iii_drone::types::vector_t::Zero(),
            double yaw_acceleration = 0.0,
            const rclcpp::Time stamp = rclcpp::Clock().now()
        );

        /**
         * Default constructor for empty object.
         */
        Reference();

        /**
         * @brief Constructor from state.
         *
         * @param state The state object.
         */
        Reference(const State& state);

        /**
         * @brief Copies the object with a new timestamp.
         * 
         * @param stamp The new timestamp.
         * 
         * @return The copied object with the new timestamp.
         */
        Reference CopyWithNewStamp(const rclcpp::Time stamp = rclcpp::Clock().now()) const;

        /**
         * Getter for the reference position.
         *
         * @return The reference position.
         */
        const iii_drone::types::point_t position() const;

        /**
         * Getter for the reference yaw.
         *
         * @return The reference yaw.
         */
        double yaw() const;

        /**
         * Getter for the reference velocity.
         *
         * @return The reference velocity.
         */
        const iii_drone::types::vector_t velocity() const;

        /**
         * Getter for the reference yaw rate.
         *
         * @return The reference yaw rate.
         */
        double yaw_rate() const;

        /**
         * Getter for the reference acceleration.
         *
         * @return The reference acceleration.
         */
        const iii_drone::types::vector_t acceleration() const;

        /**
         * Getter for the reference yaw acceleration.
         *
         * @return The reference yaw acceleration.
         */
        double yaw_acceleration() const;

        /**
         * Getter for the timestamp.
         */
        const rclcpp::Time stamp() const;

    private:
        /**
         * The reference position.
         */
        iii_drone::types::point_t position_;

        /**
         * The reference yaw.
         */
        double yaw_;

        /**
         * The reference velocity.
         */
        iii_drone::types::vector_t velocity_;

        /**
         * The reference yaw rate.
         */
        double yaw_rate_;

        /**
         * The reference acceleration.
         */
        iii_drone::types::vector_t acceleration_;

        /**
         * The reference yaw acceleration.
         */
        double yaw_acceleration_;

        /**
         * The timestamp.
         */
        rclcpp::Time stamp_;
    };

} // namespace iii_drone::control
} // namespace iii_drone