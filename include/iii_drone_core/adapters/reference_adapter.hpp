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

#include <iii_drone_core/control/reference.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/msg/reference.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {

namespace adapters {

    /**
     * @brief Class for adapting the III-Drone-Core::Reference to the III-Drone-Interfaces::Reference.
     */
    class ReferenceAdapter {
    public:
        /**
         * @brief Default constructor.
         */
        ReferenceAdapter();

        /**
         * @brief Constructor from III-Drone-Core::Reference.
         * 
         * @param reference The III-Drone-Core::Reference.
         */
        ReferenceAdapter(const iii_drone::control::Reference &reference);

        /**
         * @brief Constructor from III-Drone-Interfaces::Reference msg.
         * 
         * @param reference The III-Drone-Interfaces::Reference msg.
         */
        ReferenceAdapter(const iii_drone_interfaces::msg::Reference &reference);

        /**
         * @brief Constructor from III-Drone-Interfaces::Reference msg shared pointer.
         * 
         * @param reference The III-Drone-Interfaces::Reference msg shared pointer.
         */
        ReferenceAdapter(const iii_drone_interfaces::msg::Reference::SharedPtr &reference);

        /**
         * @brief Converts to msg.
         * 
         * @return The III-Drone-Interfaces::Reference msg.
         */
        iii_drone_interfaces::msg::Reference ToMsg() const;

        /**
         * @brief Reference getter.
         * 
         * @return The III-Drone-Core::Reference.
         */
        iii_drone::control::Reference reference() const;

    private:
        iii_drone::control::Reference reference_;

    };

} // namespace adapters

} // namespace iii_drone