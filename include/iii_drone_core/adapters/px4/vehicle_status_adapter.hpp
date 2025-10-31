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
// PX4:

#include <px4_msgs/msg/vehicle_status.hpp>

/*****************************************************************************/
// Package includes:

/*****************************************************************************/
// Enums
/*****************************************************************************/

namespace iii_drone {
namespace adapters {
namespace px4 {

    /*
    * @brief Arming state type
    */
    typedef enum {
        ARMING_STATE_INIT = 0,
        ARMING_STATE_STANDBY = 1,
        ARMING_STATE_ARMED = 2,
        ARMING_STATE_STANDBY_ERROR = 3,
        ARMING_STATE_SHUTDOWN = 4,
        ARMING_STATE_IN_AIR_RESTORE = 5,
        ARMING_STATE_MAX = 6
    } arming_state_t;

    /*
    * @brief Arm/disarm reason type
    */
    typedef enum {
        ARM_DISARM_REASON_TRANSITION_TO_STANDBY = 0,
        ARM_DISARM_REASON_RC_STICK = 1,
        ARM_DISARM_REASON_RC_SWITCH = 2,
        ARM_DISARM_REASON_COMMAND_INTERNAL = 3,
        ARM_DISARM_REASON_COMMAND_EXTERNAL = 4,
        ARM_DISARM_REASON_MISSION_START = 5,
        ARM_DISARM_REASON_SAFETY_BUTTON = 6,
        ARM_DISARM_REASON_AUTO_DISARM_LAND = 7,
        ARM_DISARM_REASON_AUTO_DISARM_PREFLIGHT = 8,
        ARM_DISARM_REASON_KILL_SWITCH = 9,
        ARM_DISARM_REASON_LOCKDOWN = 10,
        ARM_DISARM_REASON_FAILURE_DETECTOR = 11,
        ARM_DISARM_REASON_SHUTDOWN = 12,
        ARM_DISARM_REASON_UNIT_TEST = 13
    } arm_disarm_reason_t;

    /*
    * @brief Navigation state type
    */
    typedef enum {
        NAVIGATION_STATE_MANUAL = 0,
        NAVIGATION_STATE_ALTCTL = 1,
        NAVIGATION_STATE_POSCTL = 2,
        NAVIGATION_STATE_AUTO_MISSION = 3,
        NAVIGATION_STATE_AUTO_LOITER = 4,
        NAVIGATION_STATE_AUTO_RTL = 5,
        NAVIGATION_STATE_UNUSED_3 = 8,
        NAVIGATION_STATE_UNUSED = 9,
        NAVIGATION_STATE_AUTO_ACRO = 10,
        NAVIGATION_STATE_AUTO_UNUSED1 = 11,
        NAVIGATION_STATE_AUTO_DESCEND = 12,
        NAVIGATION_STATE_TERMINATION = 13,
        NAVIGATION_STATE_OFFBOARD = 14,
        NAVIGATION_STATE_STAB = 15,
        NAVIGATION_STATE_UNUSED2 = 16,
        NAVIGATION_STATE_AUTO_TAKEOFF = 17,
        NAVIGATION_STATE_AUTO_LAND = 18,
        NAVIGATION_STATE_AUTO_FOLLOW_TARGET = 19,
        NAVIGATION_STATE_AUTO_PRECLAND = 20,
        NAVIGATION_STATE_ORBIT = 21,
        NAVIGATION_STATE_VTOL_TAKEOFF = 22,
        NAVIGATION_STATE_MAX = 23
    } navigation_state_t;

    /*
    * @brief Failure detector status type
    */
    typedef enum {
        FAILURE_NONE = 0,
        FAILURE_ROLL = 1,
        FAILURE_PITCH = 2,
        FAILURE_ALT = 4,
        FAILURE_EXT = 8,
        FAILURE_ARM_ESC = 16,
        FAILURE_ARM_BATTERY = 32,
        FAILURE_IMBALANCED_PROP = 64,
        FAILURE_MOTOR = 128
    } vehicle_status_failure_t;

    /*
    * @brief HIL state type
    */
    typedef enum {
        HIL_STATE_OFF = 0,
        HIL_STATE_ON = 1
    } hil_state_t;

    /*
    * @brief Vehicle type type
    */
    typedef enum {
        VEHICLE_TYPE_UNKNOWN = 0,
        VEHICLE_TYPE_ROTARY_WING = 1,
        VEHICLE_TYPE_FIXED_WING = 2,
        VEHICLE_TYPE_ROVER = 3,
        VEHICLE_TYPE_AIRSHIP = 4
    } vehicle_type_t;

}}}

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace adapters {
namespace px4 {

    /* 
    * @brief PX4 vehicle status adapter class
    */
    class VehicleStatusAdapter {
    public:
        /*
        * @brief Default constructor
        */
        VehicleStatusAdapter();

        /*
        * @brief Constructor from PX4 vehicle status message
        *
        * @param vehicle_status_msg PX4 vehicle status message
        */
        VehicleStatusAdapter(const px4_msgs::msg::VehicleStatus& vehicle_status_msg);

        /*
        * @brief Updates from a PX4 vehicle status message
        *
        * @param vehicle_status_msg PX4 vehicle status message
        * 
        * @return void
        */ 
        void UpdateFromMsg(const px4_msgs::msg::VehicleStatus & vehicle_status_msg);

        /*
        * @brief Timestamp getter
        *
        * @return ROS2 time stamp
        */
        const rclcpp::Time & stamp() const;

        /*
        * @brief Armed time getter
        *
        * @return Armed timestamp
        */
        const rclcpp::Time & armed_time() const;

        /*
        * @brief Takeoff time getter
        *
        * @return Takeoff timestamp
        */
        const rclcpp::Time & takeoff_time() const;

        /*
        * @brief Arming state getter
        *
        * @return Arming state
        */
        const arming_state_t & arming_state() const;

        /*
        * @brief Latest arming reason getter
        *
        * @return Latest arming reason
        */
        const arm_disarm_reason_t & latest_arming_reason() const;
        
        /*
        * @brief Latest disarming reason getter
        *
        * @return Latest disarming reason
        */
        const arm_disarm_reason_t & latest_disarming_reason() const;

        /*
        * @brief Navigation state timestamp getter
        *
        * @return Navigation state timestamp
        */
        const rclcpp::Time & nav_state_timestamp() const;

        /*
        * @brief Navigation state user intention getter
        *
        * @return Navigation state user intention
        */
        const navigation_state_t & nav_state_user_intention() const;

        /*
        * @brief Navigation state getter
        *
        * @return Navigation state
        */
        const navigation_state_t & nav_state() const;

        /*
        * @brief Failure detector status getter
        *
        * @return Failure detector status
        */
        const std::vector<vehicle_status_failure_t> & failure_detector_status() const;

        /*
        * @brief HIL state getter
        *
        * @return HIL state
        */
        const hil_state_t & hil_state() const;

        /*
        * @brief Vehicle type getter
        *
        * @return Vehicle type
        */
        const vehicle_type_t & vehicle_type() const;

        /*
        * @brief Failsafe getter
        *
        * @return Failsafe
        */
        const bool & failsafe() const;

        /*
        * @brief Failsafe and user took over getter
        *
        * @return Failsafe and user took over
        */
        const bool & failsafe_and_user_took_over() const;

        /*
        * @brief GCS connection lost getter
        *
        * @return GCS connection lost
        */
        const bool & gcs_connection_lost() const;

        /*
        * @brief GCS connection lost counter getter
        *
        * @return GCS connection lost counter
        */
        const uint8_t & gcs_connection_lost_counter() const;

        /*
        * @brief High latency data link lost getter
        *
        * @return High latency data link lost
        */
        const bool & high_latency_data_link_lost() const;

        /*
        * @brief System type getter
        *
        * @return System type
        */
        const uint8_t & system_type() const;

        /*
        * @brief System ID getter
        *
        * @return System ID
        */
        const uint8_t & system_id() const;

        /*
        * @brief Component ID getter
        *
        * @return Component ID
        */
        const uint8_t & component_id() const;

        /*
        * @brief Safety button available getter
        *
        * @return Safety button available
        */
        const bool & safety_button_available() const;

        /*
        * @brief Safety off getter
        *
        * @return Safety off
        */
        const bool & safety_off() const;

        /*
        * @brief Power input valid getter
        *
        * @return Power input valid
        */
        const bool & power_input_valid() const;

        /*
        * @brief USB connected getter
        *
        * @return USB connected
        */
        const bool & usb_connected() const;

        /*
        * @brief Pre-flight checks pass getter
        *
        * @return Pre-flight checks pass
        */
        const bool & pre_flight_checks_pass() const;

    private:
        /*
        * @brief Timestamp for last update
        */
        rclcpp::Time stamp_;

        /*
        * @brief Timestamp for arming
        */
        rclcpp::Time armed_time_;
        /*
        * @brief Timestamp for takeoff
        */
        rclcpp::Time takeoff_time_;

        /*
        * @brief Arming state
        */
        arming_state_t arming_state_;

        /*
        * @brief Latest arming reason
        */
        arm_disarm_reason_t latest_arming_reason_;
        /*
        * @brief Latest disarming reason
        */
        arm_disarm_reason_t latest_disarming_reason_;

        /*
        * @brief Navigation state timestamp
        */
        rclcpp::Time nav_state_timestamp_;

        /*
        * @brief Navigation state user intention
        */
        navigation_state_t nav_state_user_intention_;
        /*
        * @brief Navigation state
        */
        navigation_state_t nav_state_;

        /*
        * @brief Failure detector status
        */
        std::vector<vehicle_status_failure_t> failure_detector_status_;

        /*
        * @brief HIL state
        */
        hil_state_t hil_state_;

        /*
        * @brief Vehicle type
        */
        vehicle_type_t vehicle_type_;

        /*
        * @brief Failsafe
        */
        bool failsafe_;
        /*
        * @brief Failsafe and user took over
        */
        bool failsafe_and_user_took_over_;

        /*
        * @brief GCS connection lost
        */
        bool gcs_connection_lost_;
        /*
        * @brief GCS connection lost counter
        */
        uint8_t gcs_connection_lost_counter_;
        /*
        * @brief High latency data link lost
        */
        bool high_latency_data_link_lost_;

        /*
        * @brief System type
        */
        uint8_t system_type_;
        /*
        * @brief System ID
        */
        uint8_t system_id_;
        /*
        * @brief Component ID
        */
        uint8_t component_id_;

        /*
        * @brief Safety button available
        */
        bool safety_button_available_;
        /*
        * @brief Safety off
        */
        bool safety_off_;

        /*
        * @brief Power input valid
        */
        bool power_input_valid_;
        /*
        * @brief USB connected
        */
        bool usb_connected_;

        /*
        * @brief Pre-flight checks pass
        */
        bool pre_flight_checks_pass_;

    };
}}}