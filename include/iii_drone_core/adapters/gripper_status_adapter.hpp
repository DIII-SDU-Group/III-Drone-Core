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
// III-Drone-Interfaces:

#include <iii_drone_interfaces/msg/gripper_status.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace adapters {

    class GripperStatusAdapter {
    public:
        /**
         * @brief Default constructor.
         */
        GripperStatusAdapter();

        /**
         * @brief Copy constructor.
         * 
         * @param other The other GripperStatusAdapter.
         */
        GripperStatusAdapter(const GripperStatusAdapter &other);

        /**
         * @brief Constructor from III-Drone-Interfaces::GripperStatus msg.
         * 
         * @param msg The III-Drone-Interfaces::GripperStatus msg.
         */
        GripperStatusAdapter(const iii_drone_interfaces::msg::GripperStatus &msg);

        /**
         * @brief Update the adapter from a III-Drone-Interfaces::GripperStatus msg.
         * 
         * @param msg The III-Drone-Interfaces::GripperStatus msg.
         */
        void UpdateFromMsg(const iii_drone_interfaces::msg::GripperStatus &msg);

        /**
         * @brief Is open getter.
         * 
         * @return True if the gripper is open, false otherwise.
         */
        bool open() const;

        /**
         * @brief Assignment operator.
         * 
         * @param other The other GripperStatusAdapter.
         * 
         * @return The assigned GripperStatusAdapter.
         */
        GripperStatusAdapter &operator=(const GripperStatusAdapter &other);

    private:
        /**
         * @brief The mutex.
         */
        mutable std::shared_mutex mutex_;

        /**
         * @brief The open state of the gripper.
         */
        bool open_;

    };

} // namespace adapters
} // namespace iii_drone