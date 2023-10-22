/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "iii_drone_core/control/trajectory_controller_node/trajectory_controller_node_configurator.hpp"

using namespace iii_drone::control::trajectory_controller_node;
using namespace iii_drone::configuration;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

TrajectoryControllerConfigurator::TrajectoryControllerConfigurator(
    rclcpp::Node *node,
    std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback
) : Configurator(
        node,
        after_parameter_change_callback
    ), TFConfigurator(
        node,
        after_parameter_change_callback
    ) {

    declareNodeParameters();

}

TrajectoryControllerConfigurator::TrajectoryControllerConfigurator(
    rclcpp::Node *node, 
    const rclcpp::QoS & qos,
    std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback
) : Configurator(
        node, 
        qos,
        after_parameter_change_callback
    ), TFConfigurator(
        node,
        qos,
        after_parameter_change_callback
    ) {

    declareNodeParameters();

}

const bool TrajectoryControllerConfigurator::always_armed_for_debug() const {

    return GetParameter("/control/trajectory_controller/always_armed_for_debug").as_bool();

}

const float TrajectoryControllerConfigurator::landed_altitude_threshold() const {

    return GetParameter("/control/trajectory_controller/landed_altitude_threshold").as_double();

}

const bool TrajectoryControllerConfigurator::use_ground_altitude_offset() const {

    return GetParameter("/control/trajectory_controller/use_ground_altitude_offset").as_bool();

}

const float TrajectoryControllerConfigurator::reached_position_euclidean_distance_threshold() const {

    return GetParameter("/control/trajectory_controller/reached_position_euclidean_distance_threshold").as_double();

}

const float TrajectoryControllerConfigurator::minimum_target_altitude() const {

    return GetParameter("/control/trajectory_controller/minimum_target_altitude").as_double();

}

const float TrajectoryControllerConfigurator::target_cable_fixed_position_distance_threshold() const {

    return GetParameter("/control/trajectory_controller/target_cable_fixed_position_distance_threshold").as_double();

}

const float TrajectoryControllerConfigurator::target_cable_safety_margin_distance_threshold() const {

    return GetParameter("/control/trajectory_controller/target_cable_safety_margin_distance_threshold").as_double();

}

const float TrajectoryControllerConfigurator::target_cable_safety_margin_max_euc_distance() const {

    return GetParameter("/control/trajectory_controller/target_cable_safety_margin_max_euc_distance").as_double();

}

const float TrajectoryControllerConfigurator::target_cable_safety_margin_max_euc_velocity() const {

    return GetParameter("/control/trajectory_controller/target_cable_safety_margin_max_euc_velocity").as_double();

}

const float TrajectoryControllerConfigurator::target_cable_safety_margin_max_euc_acceleration() const {

    return GetParameter("/control/trajectory_controller/target_cable_safety_margin_max_euc_acceleration").as_double();

}

const float TrajectoryControllerConfigurator::target_cable_safety_margin_max_yaw_distance() const {

    return GetParameter("/control/trajectory_controller/target_cable_safety_margin_max_yaw_distance").as_double();

}

const float TrajectoryControllerConfigurator::target_cable_safety_margin_max_yaw_velocity() const {

    return GetParameter("/control/trajectory_controller/target_cable_safety_margin_max_yaw_velocity").as_double();

}

const float TrajectoryControllerConfigurator::target_cable_set_point_truncate_distance_threshold() const {

    return GetParameter("/control/trajectory_controller/target_cable_set_point_truncate_distance_threshold").as_double();

}

const float TrajectoryControllerConfigurator::landed_on_powerline_non_offboard_max_euc_distance() const {

    return GetParameter("/control/trajectory_controller/landed_on_powerline_non_offboard_max_euc_distance").as_double();

}

const bool TrajectoryControllerConfigurator::always_hover_in_offboard() const {

    return GetParameter("/control/trajectory_controller/always_hover_in_offboard").as_bool();

}

const std::string TrajectoryControllerConfigurator::on_cable_control_mode() const {

    return GetParameter("/control/trajectory_controller/on_cable_control_mode").as_string();

}

const float TrajectoryControllerConfigurator::on_cable_upwards_thrust() const {

    return GetParameter("/control/trajectory_controller/on_cable_upwards_thrust").as_double();

}

const float TrajectoryControllerConfigurator::on_cable_upwards_velocity() const {

    return GetParameter("/control/trajectory_controller/on_cable_upwards_velocity").as_double();

}

const bool TrajectoryControllerConfigurator::hover_under_cable_on_aborted_cable_landing() const {

    return GetParameter("/control/trajectory_controller/hover_under_cable_on_aborted_cable_landing").as_bool();

}

const bool TrajectoryControllerConfigurator::use_gripper_status_condition() const {

    return GetParameter("/control/trajectory_controller/use_gripper_status_condition").as_bool();

}

const float TrajectoryControllerConfigurator::disarming_on_cable_max_descend_distance() const {

    return GetParameter("/control/trajectory_controller/disarming_on_cable_max_descend_distance").as_double();

}

const std::string TrajectoryControllerConfigurator::disarm_on_cable_mode() const {

    return GetParameter("/control/trajectory_controller/disarm_on_cable_mode").as_string();

}

const float TrajectoryControllerConfigurator::disarm_on_cable_thrust_decrease_time_s() const {

    return GetParameter("/control/trajectory_controller/disarm_on_cable_thrust_decrease_time_s").as_double();

}

const float TrajectoryControllerConfigurator::disarm_on_cable_thrust_wait_for_disarm_time_s() const {

    return GetParameter("/control/trajectory_controller/disarm_on_cable_thrust_wait_for_disarm_time_s").as_double();

}

const float TrajectoryControllerConfigurator::disarm_on_cable_flight_termination_timeout_s() const {

    return GetParameter("/control/trajectory_controller/disarm_on_cable_flight_termination_timeout_s").as_double();

}

const float TrajectoryControllerConfigurator::arming_on_cable_spool_up_time_s() const {

    return GetParameter("/control/trajectory_controller/arming_on_cable_spool_up_time_s").as_double();

}

const bool TrajectoryControllerConfigurator::update_home_position() const {

    return GetParameter("/control/trajectory_controller/update_home_position").as_bool();

}

const int TrajectoryControllerConfigurator::arm_cnt_timeout() const {

    return GetParameter("/control/trajectory_controller/arm_cnt_timeout").as_int();

}

const int TrajectoryControllerConfigurator::offboard_cnt_timeout() const {

    return GetParameter("/control/trajectory_controller/offboard_cnt_timeout").as_int();

}

const int TrajectoryControllerConfigurator::land_cnt_timeout() const {

    return GetParameter("/control/trajectory_controller/land_cnt_timeout").as_int();

}

const int TrajectoryControllerConfigurator::target_cable_cnt_timeout() const {

    return GetParameter("/control/trajectory_controller/target_cable_cnt_timeout").as_int();

}

const int TrajectoryControllerConfigurator::disarm_on_cable_cnt_timeout() const {

    return GetParameter("/control/trajectory_controller/disarm_on_cable_cnt_timeout").as_int();

}

const float TrajectoryControllerConfigurator::dt() const {

    return GetParameter("/control/trajectory_controller/dt").as_double();

}

const bool TrajectoryControllerConfigurator::use_cartesian_PID() const {

    return GetParameter("/control/trajectory_controller/use_cartesian_PID").as_bool();

}

const float TrajectoryControllerConfigurator::cartesian_PID_Kp() const {

    return GetParameter("/control/trajectory_controller/cartesian_PID_Kp").as_double();

}

const float TrajectoryControllerConfigurator::cartesian_PID_Ki() const {

    return GetParameter("/control/trajectory_controller/cartesian_PID_Ki").as_double();

}

const float TrajectoryControllerConfigurator::cartesian_PID_Kd() const {

    return GetParameter("/control/trajectory_controller/cartesian_PID_Kd").as_double();

}

const float TrajectoryControllerConfigurator::cartesian_PID_integral_reset_error_threshold() const {

    return GetParameter("/control/trajectory_controller/cartesian_PID_integral_reset_error_threshold").as_double();

}

const int TrajectoryControllerConfigurator::MPC_N() const {

    return GetParameter("/control/trajectory_controller/MPC_N").as_int();

}

const bool TrajectoryControllerConfigurator::MPC_use_state_feedback() const {

    return GetParameter("/control/trajectory_controller/MPC_use_state_feedback").as_bool();

}

const float TrajectoryControllerConfigurator::direct_target_setpoint_dist_threshold() const {

    return GetParameter("/control/trajectory_controller/direct_target_setpoint_dist_threshold").as_double();

}

const float TrajectoryControllerConfigurator::cable_landing_target_upwards_velocity() const {

    return GetParameter("/control/trajectory_controller/cable_landing_target_upwards_velocity").as_double();

}

void TrajectoryControllerConfigurator::declareNodeParameters() {

    DeclareParameter<bool>("/control/trajectory_controller/always_armed_for_debug");
    DeclareParameter<float>("/control/trajectory_controller/landed_altitude_threshold");
    DeclareParameter<bool>("/control/trajectory_controller/use_ground_altitude_offset");
    DeclareParameter<float>("/control/trajectory_controller/reached_position_euclidean_distance_threshold");
    DeclareParameter<float>("/control/trajectory_controller/minimum_target_altitude");
    DeclareParameter<float>("/control/trajectory_controller/target_cable_fixed_position_distance_threshold");
    DeclareParameter<float>("/control/trajectory_controller/target_cable_safety_margin_distance_threshold");
    DeclareParameter<float>("/control/trajectory_controller/target_cable_safety_margin_max_euc_distance");
    DeclareParameter<float>("/control/trajectory_controller/target_cable_safety_margin_max_euc_velocity");
    DeclareParameter<float>("/control/trajectory_controller/target_cable_safety_margin_max_euc_acceleration");
    DeclareParameter<float>("/control/trajectory_controller/target_cable_safety_margin_max_yaw_distance");
    DeclareParameter<float>("/control/trajectory_controller/target_cable_safety_margin_max_yaw_velocity");
    DeclareParameter<float>("/control/trajectory_controller/target_cable_set_point_truncate_distance_threshold");
    DeclareParameter<float>("/control/trajectory_controller/landed_on_powerline_non_offboard_max_euc_distance");
    DeclareParameter<bool>("/control/trajectory_controller/always_hover_in_offboard");
    DeclareParameter<std::string>("/control/trajectory_controller/on_cable_control_mode");
    DeclareParameter<float>("/control/trajectory_controller/on_cable_upwards_thrust");
    DeclareParameter<float>("/control/trajectory_controller/on_cable_upwards_velocity");
    DeclareParameter<bool>("/control/trajectory_controller/use_gripper_status_condition");
    DeclareParameter<bool>("/control/trajectory_controller/hover_under_cable_on_aborted_cable_landing");
    DeclareParameter<float>("/control/trajectory_controller/disarming_on_cable_max_descend_distance");
    DeclareParameter<std::string>("/control/trajectory_controller/disarm_on_cable_mode");
    DeclareParameter<float>("/control/trajectory_controller/disarm_on_cable_thrust_decrease_time_s");
    DeclareParameter<float>("/control/trajectory_controller/disarm_on_cable_thrust_wait_for_disarm_time_s");
    DeclareParameter<float>("/control/trajectory_controller/disarm_on_cable_flight_termination_timeout_s");
    DeclareParameter<float>("/control/trajectory_controller/arming_on_cable_spool_up_time_s");
    DeclareParameter<bool>("/control/trajectory_controller/update_home_position");
    DeclareParameter<int>("/control/trajectory_controller/arm_cnt_timeout");
    DeclareParameter<int>("/control/trajectory_controller/offboard_cnt_timeout");
    DeclareParameter<int>("/control/trajectory_controller/land_cnt_timeout");
    DeclareParameter<int>("/control/trajectory_controller/target_cable_cnt_timeout");
    DeclareParameter<int>("/control/trajectory_controller/disarm_on_cable_cnt_timeout");
    DeclareParameter<float>("/control/trajectory_controller/dt");
    DeclareParameter<bool>("/control/trajectory_controller/use_cartesian_PID");
    DeclareParameter<float>("/control/trajectory_controller/cartesian_PID_Kp");
    DeclareParameter<float>("/control/trajectory_controller/cartesian_PID_Ki");
    DeclareParameter<float>("/control/trajectory_controller/cartesian_PID_Kd");
    DeclareParameter<float>("/control/trajectory_controller/cartesian_PID_integral_reset_error_threshold");
    DeclareParameter<int>("/control/trajectory_controller/MPC_N");
    DeclareParameter<bool>("/control/trajectory_controller/MPC_use_state_feedback");
    DeclareParameter<float>("/control/trajectory_controller/direct_target_setpoint_dist_threshold");
    DeclareParameter<float>("/control/trajectory_controller/cable_landing_target_upwards_velocity");

}
