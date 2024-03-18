#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/types.hpp>
#include <iii_drone_core/utils/math.hpp>

#include <iii_drone_core/control/state.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/msg/state.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {

namespace adapters {

    /**
     * @brief Class for adapting the III-Drone-Core::State to the III-Drone-Interfaces::State.
     */
    class StateAdapter {
    public:
        /**
         * @brief Default constructor.
         */
        StateAdapter();

        /**
         * @brief Constructor from III-Drone-Core::State.
         * 
         * @param state The III-Drone-Core::State.
         */
        StateAdapter(const iii_drone::control::State &state);

        /**
         * @brief Constructor from III-Drone-Interfaces::State msg.
         * 
         * @param state The III-Drone-Interfaces::State msg.
         */
        StateAdapter(const iii_drone_interfaces::msg::State &msg);

        /**
         * @brief Constructor from III-Drone-Interfaces::State msg shared pointer.
         * 
         * @param msg The III-Drone-Interfaces::State msg shared pointer.
         */
        StateAdapter(const iii_drone_interfaces::msg::State::SharedPtr &msg);

        /**
         * @brief Converts to msg.
         * 
         * @return The III-Drone-Interfaces::State msg.
         */
        iii_drone_interfaces::msg::State ToMsg() const;

        /**
         * @brief Converts to pose stamped msg.
         * 
         * @param frame_id The frame id.
         * 
         * @return The geometry_msgs::msg::PoseStamped msg.
         */
        geometry_msgs::msg::PoseStamped ToPoseStampedMsg(std::string frame_id) const;

        /**
         * @brief State getter.
         * 
         * @return The III-Drone-Core::State.
         */
        iii_drone::control::State state() const;

    private:
        iii_drone::control::State state_;

    };

} // namespace adapters

} // namespace iii_drone