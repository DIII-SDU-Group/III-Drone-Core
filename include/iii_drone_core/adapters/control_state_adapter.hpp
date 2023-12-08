#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/msg/control_state.hpp>

/*****************************************************************************/
// Defines
/*****************************************************************************/

namespace iii_drone {
namespace adapters {

typedef enum {
    CONTROL_STATE_INIT=0,
    CONTROL_STATE_ON_GROUND_NON_OFFBOARD=1,
    CONTROL_STATE_IN_FLIGHT_NON_OFFBOARD=2,
    CONTROL_STATE_ARMING=3,
    CONTROL_STATE_SETTING_OFFBOARD=4,
    CONTROL_STATE_TAKING_OFF=5,
    CONTROL_STATE_HOVERING=6,
    CONTROL_STATE_LANDING=7,
    CONTROL_STATE_IN_POSITIONAL_FLIGHT=8,
    CONTROL_STATE_DURING_CABLE_LANDING=9,
    CONTROL_STATE_ON_CABLE_ARMED=10,
    CONTROL_STATE_DURING_CABLE_TAKEOFF=11,
    CONTROL_STATE_HOVERING_UNDER_CABLE=12,
    CONTROL_STATE_FLYING_ALONG_CABLE=13,
    CONTROL_STATE_DISARMING_ON_CABLE=14,
    CONTROL_STATE_ON_CABLE_DISARMED=15,
    CONTROL_STATE_ARMING_ON_CABLE=16,
    CONTROL_STATE_SETTING_OFFBOARD_ON_CABLE=17
} control_state_t;

} // namespace adapters
} // namespace iii_drone

/*****************************************************************************/
// Implementation
/*****************************************************************************/

namespace iii_drone {
namespace adapters {

    /**
     * @brief ControlStateAdapter class for control state.
     */
    class ControlStateAdapter {
    public:
        /**
         * @brief Default constructor.
         */
        ControlStateAdapter();

        /**
         * @brief Constructor from control state type.
         * 
         * @param control_state Control state.
         */
        ControlStateAdapter(const control_state_t & control_state);

        /**
         * @brief Constructor from control state message.
         * 
         * @param msg Control state message.
         */
        ControlStateAdapter(const iii_drone_interfaces::msg::ControlState & msg);

        /**
         * @brief Updates from message.
         * 
         * @param msg Control state message.
         */
        void UpdateFromMsg(const iii_drone_interfaces::msg::ControlState & msg);

        /**
         * @brief Converts to message.
         * 
         * @return Control state message.
         */
        iii_drone_interfaces::msg::ControlState ToMsg() const;

        /**
         * @brief Control state getter.
         */
        const control_state_t & control_state() const;

        /**
         * @brief Timestamp getter.
         */
        const rclcpp::Time & stamp() const;

    private:
        /**
         * @brief Control state.
         */
        control_state_t control_state_;

        /**
         * @brief Timestamp.
         */
        rclcpp::Time stamp_;

    };

} // namespace adapters
} // namespace iii_drone