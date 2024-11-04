#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <vector>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/types.hpp>

#include <iii_drone_core/control/state.hpp>

#include <iii_drone_core/adapters/target_adapter.hpp>
#include <iii_drone_core/adapters/state_adapter.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/msg/combined_drone_awareness.hpp>

/*****************************************************************************/
// Enum
/*****************************************************************************/

namespace iii_drone {
namespace adapters {

    typedef enum {
        DRONE_LOCATION_UNKNOWN = 0,
        DRONE_LOCATION_ON_GROUND = 1,
        DRONE_LOCATION_IN_FLIGHT = 2,
        DRONE_LOCATION_ON_CABLE = 3
    } drone_location_t;

} // namespace adapters
} // namespace iii_drone

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace adapters {

    class CombinedDroneAwarenessAdapter {
    public:
        CombinedDroneAwarenessAdapter();

        /**
         * @brief Constructor from message.
         * 
         * @param msg Message
         */
        CombinedDroneAwarenessAdapter(const iii_drone_interfaces::msg::CombinedDroneAwareness & msg);

        /**
         * @brief Constructor from members.
         * 
         * @param state State
         * @param armed Armed
         * @param offboard Offboard
         * @param target_adapter Target adapter
         * @param target_position_known Target position known
         * @param drone_location Drone location
         * @param on_cable_id On cable id
         * @param ground_altitude_estimate Ground altitude estimate
         * @param ground_altitude_estimate_amsl Ground altitude estimate amsl
         * @param gripper_open
         */
        CombinedDroneAwarenessAdapter(
            const iii_drone::control::State & state,
            bool armed,
            bool offboard,
            const iii_drone::adapters::TargetAdapter & target_adapter,
            bool target_position_known,
            drone_location_t drone_location,
            int on_cable_id,
            float ground_altitude_estimate,
            float ground_altitude_estimate_amsl,
            bool gripper_open
        );

        void UpdateFromMsg(const iii_drone_interfaces::msg::CombinedDroneAwareness & msg);

        const iii_drone_interfaces::msg::CombinedDroneAwareness ToMsg() const;

        const iii_drone::control::State state() const;

        iii_drone::control::State & state();

        bool armed() const;

        bool & armed();

        bool offboard() const;

        bool & offboard();

        bool has_target() const;

        const iii_drone::adapters::TargetAdapter target_adapter() const;

        iii_drone::adapters::TargetAdapter & target_adapter();

        bool target_position_known() const;

        bool & target_position_known();

        drone_location_t drone_location() const;

        drone_location_t & drone_location();

        bool on_ground() const;

        bool in_flight() const;

        bool on_cable() const;

        int on_cable_id() const;

        int & on_cable_id();

        float ground_altitude_estimate() const;

        float & ground_altitude_estimate();

        float ground_altitude_estimate_amsl() const;

        float & ground_altitude_estimate_amsl();

        bool gripper_open() const;

        bool & gripper_open();

    private:
        iii_drone::control::State state_;

        bool armed_;

        bool offboard_;

        iii_drone::adapters::TargetAdapter target_adapter_;

        bool target_position_known_;

        drone_location_t drone_location_;

        int on_cable_id_;

        float ground_altitude_estimate_;

        float ground_altitude_estimate_amsl_;

        bool gripper_open_;

    };

} // namespace adapters
} // namespace iii_drone