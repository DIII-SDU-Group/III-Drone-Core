#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// III-Drone-Configuration:

#include <iii_drone_configuration/configurator.hpp>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {

namespace control {

namespace flight_controller_node {

    /**
     * @brief Class for handling parameters for FlightController.
    */
    class FlightControllerConfigurator : public configuration::Configurator {

    public:
        /**
         * @brief Constructor
         *
         * @param node Reference to the handling node
         * @param after_parameter_change_callback Callback function called after successful parameter change, default is nullptr
         */
        FlightControllerConfigurator(
            rclcpp::Node *node,
            std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback = nullptr
        );

        /**
         * @brief Constructor
         * 
         * @param node Reference to the handling node
         * @param qos QoS profile for the parameter event subscription
         * @param after_parameter_change_callback Callback function called after successful parameter change, default is nullptr
         */
        FlightControllerConfigurator(
            rclcpp::Node *node, 
            const rclcpp::QoS & qos,
            std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback = nullptr
        );

        /**
         * @brief Whether the drone should always be armed for debug purposes
         * 
         * @return bool The value of the parameter
        */
        bool always_armed_for_debug() const;

        /**
         * @brief The altitude threshold for considering the drone landed
         * 
         * @return float The value of the parameter
        */
        float landed_altitude_threshold() const;

        /**
         * @brief Whether the drone should use ground altitude offset
         * 
         * @return bool The value of the parameter
        */
        bool use_ground_altitude_offset() const;

        /**
         * @brief The threshold for considering the drone reached the target position
         * 
         * @return float The value of the parameter
        */
        float reached_position_euclidean_distance_threshold() const;

        /**
         * @brief The minimum altitude to be used when the target altitude is below it
         * 
         * @return float The value of the parameter
        */
        float minimum_target_altitude() const;

        /**
         * @brief The distance threshold to consider the target position reached when the drone is following a cable
         * 
         * @return float The value of the parameter
        */
        float target_cable_fixed_position_distance_threshold() const;

        /**
         * @brief The distance threshold to consider the target position reached when the drone is following a cable
         * 
         * @return float The value of the parameter
        */
        float target_cable_safety_margin_distance_threshold() const;

        /**
         * @brief The maximum euclidean distance between the drone and the target position to consider the target position reached when the drone is following a cable
         * 
         * @return float The value of the parameter
        */
        float target_cable_safety_margin_max_euc_distance() const;

        /**
         * @brief The maximum euclidean velocity between the drone and the target position to consider the target position reached when the drone is following a cable
         * 
         * @return float The value of the parameter
        */
        float target_cable_safety_margin_max_euc_velocity() const;

        /**
         * @brief The maximum euclidean acceleration between the drone and the target position to consider the target position reached when the drone is following a cable
         * 
         * @return float The value of the parameter
        */
        float target_cable_safety_margin_max_euc_acceleration() const;

        /**
         * @brief The maximum yaw distance between the drone and the target position to consider the target position reached when the drone is following a cable
         * 
         * @return float The value of the parameter
        */
        float target_cable_safety_margin_max_yaw_distance() const;

        /**
         * @brief The maximum yaw velocity between the drone and the target position to consider the target position reached when the drone is following a cable
         * 
         * @return float The value of the parameter
        */
        float target_cable_safety_margin_max_yaw_velocity() const;

        /**
         * @brief The distance threshold to truncate the setpoint when the drone is following a cable
         * 
         * @return float The value of the parameter
        */
        float target_cable_set_point_truncate_distance_threshold() const;

        /**
         * @brief The maximum euclidean distance between the drone and the powerline to consider the drone landed on the powerline when the drone is not in offboard mode
         * 
         * @return float The value of the parameter
        */
        float landed_on_powerline_non_offboard_max_euc_distance() const;

        /**
         * @brief Whether the drone should always hover in offboard mode
         * 
         * @return bool The value of the parameter
        */
        bool always_hover_in_offboard() const;

        /**
         * @brief The control mode to use when the drone is on the cable
         * 
         * @return std::string The value of the parameter
        */
        const std::string on_cable_control_mode() const;

        /**
         * @brief The upwards thrust to apply when the drone is on the cable
         * 
         * @return float The value of the parameter
        */
        float on_cable_upwards_thrust() const;

        /**
         * @brief The upwards velocity to apply when the drone is on the cable
         * 
         * @return float The value of the parameter
        */
        float on_cable_upwards_velocity() const;

        /**
         * @brief Whether to use the gripper status condition when the drone is on the cable
         * 
         * @return bool The value of the parameter
        */
        bool use_gripper_status_condition() const;

        /**
         * @brief Whether to perform cable takeoff followed by hovering under cable on aborted cable landing
         * 
         * @return bool The value of the parameter
        */
        bool hover_under_cable_on_aborted_cable_landing() const;

        /**
         * @brief The maximum descend distance to disarm the drone when the drone is on the cable
         * 
         * @return float The value of the parameter
        */
        float disarming_on_cable_max_descend_distance() const;

        /**
         * @brief The disarm mode to use when the drone is on the cable
         * 
         * @return std::string The value of the parameter
        */
        const std::string disarm_on_cable_mode() const;

        /**
         * @brief The time to decrease the thrust to zero when disarming the drone on the cable
         * 
         * @return float The value of the parameter
        */
        float disarm_on_cable_thrust_decrease_time_s() const;

        /**
         * @brief The time to wait for disarm after the thrust is zero when disarming the drone on the cable
         * 
         * @return float The value of the parameter
        */
        float disarm_on_cable_thrust_wait_for_disarm_time_s() const;

        /**
         * @brief The timeout to terminate the flight when disarming the drone on the cable
         * 
         * @return float The value of the parameter
        */
        float disarm_on_cable_flight_termination_timeout_s() const;

        /**
         * @brief The time to spool up the motor when arming the drone on the cable
         * 
         * @return float The value of the parameter
        */
        float arming_on_cable_spool_up_time_s() const;

        /**
         * @brief Whether to update the home position
         * 
         * @return bool The value of the parameter
        */
        bool update_home_position() const;

        /**
         * @brief The timeout for arming the drone
         * 
         * @return int The value of the parameter
        */
        int arm_cnt_timeout() const;

        /**
         * @brief The timeout for offboard mode
         * 
         * @return int The value of the parameter
        */
        int offboard_cnt_timeout() const;

        /**
         * @brief The timeout for landing
         * 
         * @return int The value of the parameter
        */
        int land_cnt_timeout() const;

        /**
         * @brief The timeout for following a cable
         * 
         * @return int The value of the parameter
        */
        int target_cable_cnt_timeout() const;

        /**
         * @brief The timeout for disarming the drone on the cable
         * 
         * @return int The value of the parameter
        */
        int disarm_on_cable_cnt_timeout() const;

        /**
         * @brief The time step for the trajectory controller
         * 
         * @return float The value of the parameter
        */
        float dt() const;

        /**
         * @brief Whether to use Cartesian PID control for the trajectory controller
         * 
         * @return bool The value of the parameter
        */
        bool use_cartesian_PID() const;

        /**
         * @brief The proportional gain for the Cartesian PID controller
         * 
         * @return float The value of the parameter
        */
        float cartesian_PID_Kp() const;

        /**
         * @brief The integral gain for the Cartesian PID controller
         * 
         * @return float The value of the parameter
        */
        float cartesian_PID_Ki() const;

        /**
         * @brief The derivative gain for the Cartesian PID controller
         * 
         * @return float The value of the parameter
        */
        float cartesian_PID_Kd() const;

        /**
         * @brief The error threshold to reset the integral term for the Cartesian PID controller
         * 
         * @return float The value of the parameter
        */
        float cartesian_PID_integral_reset_error_threshold() const;

        /**
         * @brief The number of points used in the MPC trajectory
         * 
         * @return int The value of the parameter
        */
        int MPC_N() const;

        /**
         * @brief Whether to use state feedback for the MPC
         * 
         * @return bool The value of the parameter
        */
        bool MPC_use_state_feedback() const;

        /**
         * @brief The distance threshold to use direct target setpoint
         * 
         * @return float The value of the parameter
        */
        float direct_target_setpoint_dist_threshold() const;

        /**
         * @brief The upwards velocity to use for cable landing target
         * 
         * @return float The value of the parameter
        */
        float cable_landing_target_upwards_velocity() const;

        /**
         * @brief Get the drone frame ID
         *
         * @return Drone frame ID
         */
        const std::string drone_frame_id() const;

        /**
         * @brief Get the world frame ID
         *
         * @return World frame ID
         */
        const std::string world_frame_id() const;

        /**
         * @brief Get the cable gripper frame ID
         * 
         * @return Cable gripper frame ID
         */
        const std::string cable_gripper_frame_id() const;

        /**
         * @brief Get the mmWave frame ID
         * 
         * @return mmWave frame ID
         */
        const std::string mmwave_frame_id() const;

    private:
        /**
         * @brief Declares the node specific parameters
         * 
         * @return void
         */
        void declareNodeParameters();

    };

} // namespace flight_controller_node
} // namespace control
} // namespace iii_drone
