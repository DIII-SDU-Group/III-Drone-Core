/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "iii_drone_core/control/flight_controller_node/flight_controller_node_configurator.hpp"

using namespace iii_drone::control::flight_controller_node;
using namespace iii_drone::configuration;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

FlightControllerConfigurator::FlightControllerConfigurator(
    rclcpp::Node *node,
    std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback
) : Configurator(
        node,
        std::bind(
            &FlightControllerConfigurator::updateBehavorStateParametersObjectCallback,
            this,
            std::placeholders::_1
        )
    ),
    after_parameter_change_callback_(after_parameter_change_callback) {

    declareNodeParameters();
    initBehaviorStateParameters();

}

FlightControllerConfigurator::FlightControllerConfigurator(
    rclcpp::Node *node, 
    const rclcpp::QoS & qos,
    std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback
) : Configurator(
        node, 
        qos,
        std::bind(
            &FlightControllerConfigurator::updateBehavorStateParametersObjectCallback,
            this,
            std::placeholders::_1
        )
    ),
    after_parameter_change_callback_(after_parameter_change_callback) {

    declareNodeParameters();
    initBehaviorStateParameters();

}

bool FlightControllerConfigurator::always_armed_for_debug() const {

    return GetParameter("/control/flight_controller/always_armed_for_debug").as_bool();

}

float FlightControllerConfigurator::landed_altitude_threshold() const {

    return GetParameter("/control/flight_controller/landed_altitude_threshold").as_double();

}

bool FlightControllerConfigurator::use_ground_altitude_offset() const {

    return GetParameter("/control/flight_controller/use_ground_altitude_offset").as_bool();

}

float FlightControllerConfigurator::reached_position_euclidean_distance_threshold() const {

    return GetParameter("/control/flight_controller/reached_position_euclidean_distance_threshold").as_double();

}

float FlightControllerConfigurator::minimum_target_altitude() const {

    return GetParameter("/control/flight_controller/minimum_target_altitude").as_double();

}

float FlightControllerConfigurator::target_cable_fixed_position_distance_threshold() const {

    return GetParameter("/control/flight_controller/target_cable_fixed_position_distance_threshold").as_double();

}

float FlightControllerConfigurator::target_cable_safety_margin_distance_threshold() const {

    return GetParameter("/control/flight_controller/target_cable_safety_margin_distance_threshold").as_double();

}

float FlightControllerConfigurator::target_cable_safety_margin_max_euc_distance() const {

    return GetParameter("/control/flight_controller/target_cable_safety_margin_max_euc_distance").as_double();

}

float FlightControllerConfigurator::target_cable_safety_margin_max_euc_velocity() const {

    return GetParameter("/control/flight_controller/target_cable_safety_margin_max_euc_velocity").as_double();

}

float FlightControllerConfigurator::target_cable_safety_margin_max_euc_acceleration() const {

    return GetParameter("/control/flight_controller/target_cable_safety_margin_max_euc_acceleration").as_double();

}

float FlightControllerConfigurator::target_cable_safety_margin_max_yaw_distance() const {

    return GetParameter("/control/flight_controller/target_cable_safety_margin_max_yaw_distance").as_double();

}

float FlightControllerConfigurator::target_cable_safety_margin_max_yaw_velocity() const {

    return GetParameter("/control/flight_controller/target_cable_safety_margin_max_yaw_velocity").as_double();

}

float FlightControllerConfigurator::target_cable_set_point_truncate_distance_threshold() const {

    return GetParameter("/control/flight_controller/target_cable_set_point_truncate_distance_threshold").as_double();

}

float FlightControllerConfigurator::landed_on_powerline_non_offboard_max_euc_distance() const {

    return GetParameter("/control/flight_controller/landed_on_powerline_non_offboard_max_euc_distance").as_double();

}

bool FlightControllerConfigurator::always_hover_in_offboard() const {

    return GetParameter("/control/flight_controller/always_hover_in_offboard").as_bool();

}

const std::string FlightControllerConfigurator::on_cable_control_mode() const {

    return GetParameter("/control/flight_controller/on_cable_control_mode").as_string();

}

float FlightControllerConfigurator::on_cable_upwards_thrust() const {

    return GetParameter("/control/flight_controller/on_cable_upwards_thrust").as_double();

}

float FlightControllerConfigurator::on_cable_upwards_velocity() const {

    return GetParameter("/control/flight_controller/on_cable_upwards_velocity").as_double();

}

bool FlightControllerConfigurator::hover_under_cable_on_aborted_cable_landing() const {

    return GetParameter("/control/flight_controller/hover_under_cable_on_aborted_cable_landing").as_bool();

}

bool FlightControllerConfigurator::use_gripper_status_condition() const {

    return GetParameter("/control/flight_controller/use_gripper_status_condition").as_bool();

}

float FlightControllerConfigurator::disarming_on_cable_max_descend_distance() const {

    return GetParameter("/control/flight_controller/disarming_on_cable_max_descend_distance").as_double();

}

const std::string FlightControllerConfigurator::disarm_on_cable_mode() const {

    return GetParameter("/control/flight_controller/disarm_on_cable_mode").as_string();

}

float FlightControllerConfigurator::disarm_on_cable_thrust_decrease_time_s() const {

    return GetParameter("/control/flight_controller/disarm_on_cable_thrust_decrease_time_s").as_double();

}

float FlightControllerConfigurator::disarm_on_cable_thrust_wait_for_disarm_time_s() const {

    return GetParameter("/control/flight_controller/disarm_on_cable_thrust_wait_for_disarm_time_s").as_double();

}

float FlightControllerConfigurator::disarm_on_cable_flight_termination_timeout_s() const {

    return GetParameter("/control/flight_controller/disarm_on_cable_flight_termination_timeout_s").as_double();

}

float FlightControllerConfigurator::arming_on_cable_spool_up_time_s() const {

    return GetParameter("/control/flight_controller/arming_on_cable_spool_up_time_s").as_double();

}

bool FlightControllerConfigurator::update_home_position() const {

    return GetParameter("/control/flight_controller/update_home_position").as_bool();

}

int FlightControllerConfigurator::arm_cnt_timeout() const {

    return GetParameter("/control/flight_controller/arm_cnt_timeout").as_int();

}

int FlightControllerConfigurator::offboard_cnt_timeout() const {

    return GetParameter("/control/flight_controller/offboard_cnt_timeout").as_int();

}

int FlightControllerConfigurator::land_cnt_timeout() const {

    return GetParameter("/control/flight_controller/land_cnt_timeout").as_int();

}

int FlightControllerConfigurator::target_cable_cnt_timeout() const {

    return GetParameter("/control/flight_controller/target_cable_cnt_timeout").as_int();

}

int FlightControllerConfigurator::disarm_on_cable_cnt_timeout() const {

    return GetParameter("/control/flight_controller/disarm_on_cable_cnt_timeout").as_int();

}

float FlightControllerConfigurator::dt() const {

    return GetParameter("/control/flight_controller/dt").as_double();

}

bool FlightControllerConfigurator::use_cartesian_PID() const {

    return GetParameter("/control/flight_controller/use_cartesian_PID").as_bool();

}

float FlightControllerConfigurator::cartesian_PID_Kp() const {

    return GetParameter("/control/flight_controller/cartesian_PID_Kp").as_double();

}

float FlightControllerConfigurator::cartesian_PID_Ki() const {

    return GetParameter("/control/flight_controller/cartesian_PID_Ki").as_double();

}

float FlightControllerConfigurator::cartesian_PID_Kd() const {

    return GetParameter("/control/flight_controller/cartesian_PID_Kd").as_double();

}

float FlightControllerConfigurator::cartesian_PID_integral_reset_error_threshold() const {

    return GetParameter("/control/flight_controller/cartesian_PID_integral_reset_error_threshold").as_double();

}

int FlightControllerConfigurator::MPC_N() const {

    return GetParameter("/control/flight_controller/MPC_N").as_int();

}

bool FlightControllerConfigurator::MPC_use_state_feedback() const {

    return GetParameter("/control/flight_controller/MPC_use_state_feedback").as_bool();

}

float FlightControllerConfigurator::direct_target_setpoint_dist_threshold() const {

    return GetParameter("/control/flight_controller/direct_target_setpoint_dist_threshold").as_double();

}

float FlightControllerConfigurator::cable_landing_target_upwards_velocity() const {

    return GetParameter("/control/flight_controller/cable_landing_target_upwards_velocity").as_double();

}

const std::string FlightControllerConfigurator::drone_frame_id() const {
    
    return GetParameter("/tf/drone_frame_id").as_string();

}

const std::string FlightControllerConfigurator::world_frame_id() const {
    
    return GetParameter("/tf/world_frame_id").as_string();

}

const std::string FlightControllerConfigurator::cable_gripper_frame_id() const {
    
    return GetParameter("/tf/cable_gripper_frame_id").as_string();

}

const std::string FlightControllerConfigurator::mmwave_frame_id() const {
    
    return GetParameter("/tf/mmwave_frame_id").as_string();

}

const std::shared_ptr<const iii_drone::control::BehaviorStateParameters> FlightControllerConfigurator::behavior_state_parameters() const {

    return behavior_state_parameters_;

}

void FlightControllerConfigurator::declareNodeParameters() {

    DeclareParameter<bool>("/control/flight_controller/always_armed_for_debug");
    DeclareParameter<float>("/control/flight_controller/landed_altitude_threshold");
    DeclareParameter<bool>("/control/flight_controller/use_ground_altitude_offset");
    DeclareParameter<float>("/control/flight_controller/reached_position_euclidean_distance_threshold");
    DeclareParameter<float>("/control/flight_controller/minimum_target_altitude");
    DeclareParameter<float>("/control/flight_controller/target_cable_fixed_position_distance_threshold");
    DeclareParameter<float>("/control/flight_controller/target_cable_safety_margin_distance_threshold");
    DeclareParameter<float>("/control/flight_controller/target_cable_safety_margin_max_euc_distance");
    DeclareParameter<float>("/control/flight_controller/target_cable_safety_margin_max_euc_velocity");
    DeclareParameter<float>("/control/flight_controller/target_cable_safety_margin_max_euc_acceleration");
    DeclareParameter<float>("/control/flight_controller/target_cable_safety_margin_max_yaw_distance");
    DeclareParameter<float>("/control/flight_controller/target_cable_safety_margin_max_yaw_velocity");
    DeclareParameter<float>("/control/flight_controller/target_cable_set_point_truncate_distance_threshold");
    DeclareParameter<float>("/control/flight_controller/landed_on_powerline_non_offboard_max_euc_distance");
    DeclareParameter<bool>("/control/flight_controller/always_hover_in_offboard");
    DeclareParameter<std::string>("/control/flight_controller/on_cable_control_mode");
    DeclareParameter<float>("/control/flight_controller/on_cable_upwards_thrust");
    DeclareParameter<float>("/control/flight_controller/on_cable_upwards_velocity");
    DeclareParameter<bool>("/control/flight_controller/use_gripper_status_condition");
    DeclareParameter<bool>("/control/flight_controller/hover_under_cable_on_aborted_cable_landing");
    DeclareParameter<float>("/control/flight_controller/disarming_on_cable_max_descend_distance");
    DeclareParameter<std::string>("/control/flight_controller/disarm_on_cable_mode");
    DeclareParameter<float>("/control/flight_controller/disarm_on_cable_thrust_decrease_time_s");
    DeclareParameter<float>("/control/flight_controller/disarm_on_cable_thrust_wait_for_disarm_time_s");
    DeclareParameter<float>("/control/flight_controller/disarm_on_cable_flight_termination_timeout_s");
    DeclareParameter<float>("/control/flight_controller/arming_on_cable_spool_up_time_s");
    DeclareParameter<bool>("/control/flight_controller/update_home_position");
    DeclareParameter<int>("/control/flight_controller/arm_cnt_timeout");
    DeclareParameter<int>("/control/flight_controller/offboard_cnt_timeout");
    DeclareParameter<int>("/control/flight_controller/land_cnt_timeout");
    DeclareParameter<int>("/control/flight_controller/target_cable_cnt_timeout");
    DeclareParameter<int>("/control/flight_controller/disarm_on_cable_cnt_timeout");
    DeclareParameter<float>("/control/flight_controller/dt");
    DeclareParameter<bool>("/control/flight_controller/use_cartesian_PID");
    DeclareParameter<float>("/control/flight_controller/cartesian_PID_Kp");
    DeclareParameter<float>("/control/flight_controller/cartesian_PID_Ki");
    DeclareParameter<float>("/control/flight_controller/cartesian_PID_Kd");
    DeclareParameter<float>("/control/flight_controller/cartesian_PID_integral_reset_error_threshold");
    DeclareParameter<int>("/control/flight_controller/MPC_N");
    DeclareParameter<bool>("/control/flight_controller/MPC_use_state_feedback");
    DeclareParameter<float>("/control/flight_controller/direct_target_setpoint_dist_threshold");
    DeclareParameter<float>("/control/flight_controller/cable_landing_target_upwards_velocity");

    DeclareParameter<std::string>("/tf/drone_frame_id");
    DeclareParameter<std::string>("/tf/world_frame_id");
    DeclareParameter<std::string>("/tf/cable_gripper_frame_id");
    DeclareParameter<std::string>("/tf/mmwave_frame_id");

}

void FlightControllerConfigurator::initBehaviorStateParameters() {

    behavior_state_parameters_ = std::make_shared<iii_drone::control::BehaviorStateParameters>(
        GetParameter("/control/flight_controller/landed_altitude_threshold").as_double(),
        GetParameter("/control/flight_controller/on_cable_max_euc_distance").as_double(),
        GetParameter("/control/flight_controller/minimum_target_altitude").as_double(),
        GetParameter("/tf/cable_gripper_frame_id").as_string(),
        GetParameter("/tf/drone_frame_id").as_string(),
        GetParameter("/tf/world_frame_id").as_string()
    );

    behavior_state_parameters_initialized_ = true;

}

void FlightControllerConfigurator::updateBehavorStateParametersObjectCallback(const rclcpp::Parameter & parameter) {

    if (parameter.get_name() == "/control/flight_controller/landed_altitude_threshold") {

        behavior_state_parameters_->landed_altitude_threshold() = parameter.as_double();

    } else if (parameter.get_name() == "/control/flight_controller/on_cable_max_euc_distance") {

        behavior_state_parameters_->on_cable_max_euc_distance() = parameter.as_double();

    } else if (parameter.get_name() == "/control/flight_controller/minimum_target_altitude") {

        behavior_state_parameters_->minimum_target_altitude() = parameter.as_double();

    } else if (parameter.get_name() == "/tf/cable_gripper_frame_id") {

        behavior_state_parameters_->cable_gripper_frame_id() = parameter.as_string();

    } else if (parameter.get_name() == "/tf/drone_frame_id") {

        behavior_state_parameters_->drone_frame_id() = parameter.as_string();

    } else if (parameter.get_name() == "/tf/world_frame_id") {

        behavior_state_parameters_->world_frame_id() = parameter.as_string();

    }

    after_parameter_change_callback_(parameter);

}
