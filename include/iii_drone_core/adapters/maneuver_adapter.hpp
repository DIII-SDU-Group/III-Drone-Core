#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

/*****************************************************************************/
// Std:

#include <shared_mutex>
#include <mutex>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/control/maneuver/maneuver_types.hpp>
#include <iii_drone_core/control/maneuver/maneuver.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/msg/maneuver.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace adapters {

    class ManeuverAdapter {
    public:
        /**
         * @brief Default constructor.
         */
        ManeuverAdapter();

        /**
         * @brief Copy constructor.
         * 
         * @param other The other ManeuverAdapter.
         */
        ManeuverAdapter(const ManeuverAdapter &other);

        /**
         * @brief Constructor from III-Drone-Interfaces::Maneuver msg.
         * 
         * @param msg The III-Drone-Interfaces::Maneuver msg.
         */
        ManeuverAdapter(const iii_drone_interfaces::msg::Maneuver &msg);

        /**
         * @brief Constructor from Manuever.
         * 
         * @param maneuver The maneuver.
         */
        ManeuverAdapter(const iii_drone::control::maneuver::Maneuver &maneuver);

        /**
         * @brief Convert to msg.
         * 
         * @return The III-Drone-Interfaces::Maneuver msg.
         */
        iii_drone_interfaces::msg::Maneuver ToMsg() const;

        /**
         * @brief Maneuver type getter.
         * 
         * @return The maneuver type.
         */
        iii_drone::control::maneuver::maneuver_type_t maneuver_type() const;

        /**
         * @brief Start time getter.
         */
        rclcpp::Time start_time() const;

        /**
         * @brief Termianted getter.
         */
        bool terminated() const;

        /**
         * @brief Success getter.
         */
        bool success() const;

        /**
         * @brief Assignment operator.
         * 
         * @param other The other ManeuverAdapter.
         * 
         * @return The assigned ManeuverAdapter.
         */
        ManeuverAdapter &operator=(const ManeuverAdapter &other);

    private:
        /**
         * @brief The maneuver type.
         */
        iii_drone::control::maneuver::maneuver_type_t maneuver_type_;

        /**
         * @brief The start time.
         */
        rclcpp::Time start_time_;

        /**
         * @brief The terminated state.
         */
        bool terminated_;

        /**
         * @brief The success state.
         */
        bool success_;

    };

} // namespace adapters
} // namespace iii_drone