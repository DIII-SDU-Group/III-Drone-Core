/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/maneuver_controller_node/maneuver_controller_node_configurator.hpp>

using namespace iii_drone::control::maneuver_controller_node;
using namespace iii_drone::control::maneuver;
using namespace iii_drone::control;

/*****************************************************************************/
// Implementation:
/*****************************************************************************/

ManeuverControllerNodeConfigurator::ManeuverControllerNodeConfigurator(
    rclcpp::Node *node,
    std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback
) : Configurator(
        node,
        std::bind(
            &ManeuverControllerNodeConfigurator::updateParametersObjectsCallback,
            this,
            std::placeholders::_1
        )
    ),
    after_parameter_change_callback_(after_parameter_change_callback) {
    
    declareNodeParameters();
    initParametersObjects();

}

ManeuverControllerNodeConfigurator::ManeuverControllerNodeConfigurator(
    rclcpp::Node *node, 
    const rclcpp::QoS & qos,
    std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback
) : Configurator(
        node,
        qos,
        std::bind(
            &ManeuverControllerNodeConfigurator::updateParametersObjectsCallback,
            this,
            std::placeholders::_1
        )
    ),
    after_parameter_change_callback_(after_parameter_change_callback) {
    
    declareNodeParameters();
    initParametersObjects();

}

const std::shared_ptr<CombinedDroneAwarenessHandlerParameters> ManeuverControllerNodeConfigurator::combined_drone_awareness_handler_parameters() const {

    return combined_drone_awareness_handler_parameters_;

}

const std::shared_ptr<ManeuverSchedulerParameters> ManeuverControllerNodeConfigurator::maneuver_scheduler_parameters() const {

    return maneuver_scheduler_parameters_;

}

const std::shared_ptr<FlyToPositionManeuverServerParameters> ManeuverControllerNodeConfigurator::fly_to_position_maneuver_server_parameters() const {

    return fly_to_position_maneuver_server_parameters_;

}

const std::shared_ptr<FlyToObjectManeuverServerParameters> ManeuverControllerNodeConfigurator::fly_to_object_maneuver_server_parameters() const {

    return fly_to_object_maneuver_server_parameters_;

}

int ManeuverControllerNodeConfigurator::ground_estimate_window_size() const {

    return GetParameter("/control/maneuver_controller/ground_estimate_window_size").as_int();

}

int ManeuverControllerNodeConfigurator::ground_estimate_update_period_ms() const {

    return GetParameter("/control/maneuver_controller/ground_estimate_update_period_ms").as_int();

}

double ManeuverControllerNodeConfigurator::landed_altitude_threshold() const {

    return GetParameter("/control/maneuver_controller/landed_altitude_threshold").as_double();

}

double ManeuverControllerNodeConfigurator::on_cable_max_euc_distance() const {

    return GetParameter("/control/maneuver_controller/on_cable_max_euc_distance").as_double();

}

bool ManeuverControllerNodeConfigurator::fail_on_unable_to_locate() const {

    return GetParameter("/control/maneuver_controller/fail_on_unable_to_locate").as_bool();

}

std::string ManeuverControllerNodeConfigurator::cable_gripper_frame_id() const {

    return GetParameter("/tf/cable_gripper_frame_id").as_string();

}

std::string ManeuverControllerNodeConfigurator::drone_frame_id() const {

    return GetParameter("/tf/drone_frame_id").as_string();

}

std::string ManeuverControllerNodeConfigurator::world_frame_id() const {

    return GetParameter("/tf/world_frame_id").as_string();

}

std::string ManeuverControllerNodeConfigurator::ground_frame_id() const {

    return GetParameter("/tf/ground_frame_id").as_string();

}

double ManeuverControllerNodeConfigurator::maneuver_register_update_timeout_s() const {

    return GetParameter("/control/maneuver_controller/maneuver_register_update_timeout_s").as_double();

}

double ManeuverControllerNodeConfigurator::maneuver_start_timeout_s() const {

    return GetParameter("/control/maneuver_controller/maneuver_start_timeout_s").as_double();

}

unsigned int ManeuverControllerNodeConfigurator::maneuver_queue_size() const {

    return GetParameter("/control/maneuver_controller/maneuver_queue_size").as_int();

}

unsigned int ManeuverControllerNodeConfigurator::maneuver_execution_period_ms() const {

    return GetParameter("/control/maneuver_controller/maneuver_execution_period_ms").as_int();

}

bool ManeuverControllerNodeConfigurator::use_nans_when_hovering() const {

    return GetParameter("/control/maneuver_controller/use_nans_when_hovering").as_bool();

}

double ManeuverControllerNodeConfigurator::no_maneuver_idle_cnt_s() const {

    return GetParameter("/control/maneuver_controller/no_maneuver_idle_cnt_s").as_double();

}

double ManeuverControllerNodeConfigurator::hover_by_object_max_euc_dist() const {

    return GetParameter("/control/maneuver_controller/hover_by_object_max_euc_dist").as_double();

}

double ManeuverControllerNodeConfigurator::hover_on_cable_default_z_velocity() const {

    return GetParameter("/control/maneuver_controller/hover_on_cable_default_z_velocity").as_double();

}

double ManeuverControllerNodeConfigurator::hover_on_cable_default_yaw_rate() const {

    return GetParameter("/control/maneuver_controller/hover_on_cable_default_yaw_rate").as_double();

}

std::vector<int64_t> ManeuverControllerNodeConfigurator::px4_offboard_mode_ids() const {

    return GetParameter("/px4/px4_offboard_mode_ids").as_integer_array();

}

unsigned int ManeuverControllerNodeConfigurator::maneuver_wait_for_execute_poll_ms() const {

    return (unsigned int)GetParameter("/control/maneuver_controller/maneuver_wait_for_execute_poll_ms").as_int();

}

unsigned int ManeuverControllerNodeConfigurator::maneuver_evaluate_done_poll_ms() const {

    return (unsigned int)GetParameter("/control/maneuver_controller/maneuver_evaluate_done_poll_ms").as_int();

}

double ManeuverControllerNodeConfigurator::reached_position_euclidean_distance_threshold() const {

    return GetParameter("/control/maneuver_controller/reached_position_euclidean_distance_threshold").as_double();

}

double ManeuverControllerNodeConfigurator::minimum_target_altitude() const {

    return GetParameter("/control/maneuver_controller/minimum_target_altitude").as_double();

}

bool ManeuverControllerNodeConfigurator::generate_trajectories_asynchronously_with_delay() const {

    return GetParameter("/control/maneuver_controller/generate_trajectories_asynchronously_with_delay").as_bool();

}

unsigned int ManeuverControllerNodeConfigurator::generate_trajectories_poll_period_ms() const {

    return (unsigned int)GetParameter("/control/maneuver_controller/generate_trajectories_poll_period_ms").as_int();

}

void ManeuverControllerNodeConfigurator::declareNodeParameters() {

    DeclareParameter<int>("/control/maneuver_controller/ground_estimate_window_size");
    DeclareParameter<int>("/control/maneuver_controller/ground_estimate_update_period_ms");
    DeclareParameter<double>("/control/maneuver_controller/landed_altitude_threshold");
    DeclareParameter<double>("/control/maneuver_controller/on_cable_max_euc_distance");
    DeclareParameter<bool>("/control/maneuver_controller/fail_on_unable_to_locate");
    DeclareParameter<double>("/control/maneuver_controller/maneuver_register_update_timeout_s");
    DeclareParameter<double>("/control/maneuver_controller/maneuver_start_timeout_s");
    DeclareParameter<int>("/control/maneuver_controller/maneuver_queue_size");
    DeclareParameter<int>("/control/maneuver_controller/maneuver_execution_period_ms");
    DeclareParameter<bool>("/control/maneuver_controller/use_nans_when_hovering");
    DeclareParameter<double>("/control/maneuver_controller/no_maneuver_idle_cnt_s");
    DeclareParameter<double>("/control/maneuver_controller/hover_by_object_max_euc_dist");
    DeclareParameter<double>("/control/maneuver_controller/hover_on_cable_default_z_velocity");
    DeclareParameter<double>("/control/maneuver_controller/hover_on_cable_default_yaw_rate");
    DeclareParameter<int>("/control/maneuver_controller/maneuver_wait_for_execute_poll_ms");
    DeclareParameter<int>("/control/maneuver_controller/maneuver_evaluate_done_poll_ms");
    DeclareParameter<double>("/control/maneuver_controller/reached_position_euclidean_distance_threshold");
    DeclareParameter<double>("/control/maneuver_controller/minimum_target_altitude");
    DeclareParameter<bool>("/control/maneuver_controller/generate_trajectories_asynchronously_with_delay");
    DeclareParameter<int>("/control/maneuver_controller/generate_trajectories_poll_period_ms");

    DeclareParameter<std::string>("/tf/cable_gripper_frame_id");
    DeclareParameter<std::string>("/tf/drone_frame_id");
    DeclareParameter<std::string>("/tf/world_frame_id");
    DeclareParameter<std::string>("/tf/ground_frame_id");

    DeclareParameter<std::vector<int>>("/px4/px4_offboard_mode_ids");

}

void ManeuverControllerNodeConfigurator::initParametersObjects() {

    combined_drone_awareness_handler_parameters_ = std::make_shared<CombinedDroneAwarenessHandlerParameters>(
        ground_estimate_window_size(),
        ground_estimate_update_period_ms(),
        landed_altitude_threshold(),
        on_cable_max_euc_distance(),
        fail_on_unable_to_locate(),
        cable_gripper_frame_id(),
        drone_frame_id(),
        world_frame_id(),
        ground_frame_id()
    );

    maneuver_scheduler_parameters_ = std::make_shared<ManeuverSchedulerParameters>(
        maneuver_register_update_timeout_s(),
        maneuver_start_timeout_s(),
        maneuver_queue_size(),
        maneuver_execution_period_ms(),
        use_nans_when_hovering(),
        no_maneuver_idle_cnt_s(),
        hover_by_object_max_euc_dist(),
        hover_on_cable_default_z_velocity(),
        hover_on_cable_default_yaw_rate()
    );

    fly_to_position_maneuver_server_parameters_ = std::make_shared<FlyToPositionManeuverServerParameters>(
        reached_position_euclidean_distance_threshold(),
        minimum_target_altitude(),
        generate_trajectories_asynchronously_with_delay(),
        generate_trajectories_poll_period_ms(),
        world_frame_id()
    );

    fly_to_object_maneuver_server_parameters_ = std::make_shared<FlyToObjectManeuverServerParameters>(
        reached_position_euclidean_distance_threshold(),
        minimum_target_altitude(),
        generate_trajectories_asynchronously_with_delay(),
        generate_trajectories_poll_period_ms(),
        world_frame_id()
    );

}

void ManeuverControllerNodeConfigurator::updateParametersObjectsCallback(const rclcpp::Parameter & parameter) {

    std::string param_name = parameter.get_name();

    if (param_name == "/control/maneuver_controller/landed_altitude_threshold") {

        combined_drone_awareness_handler_parameters_->landed_altitude_threshold() = parameter.as_double();

    } else if (param_name == "/control/maneuver_controller/on_cable_max_euc_distance") {

        combined_drone_awareness_handler_parameters_->on_cable_max_euc_distance() = parameter.as_double();

    } else if (param_name == "/control/maneuver_controller/hover_by_object_max_euc_dist") {

        maneuver_scheduler_parameters_->hover_by_object_max_euc_dist() = parameter.as_double();

    } else if (param_name == "/control/maneuver_controller/hover_by_object_max_euc_dist") {

        maneuver_scheduler_parameters_->hover_by_object_max_euc_dist() = parameter.as_double();

    } else if (param_name == "/control/maneuver_controller/hover_on_cable_default_z_velocity") {

        maneuver_scheduler_parameters_->hover_on_cable_default_z_velocity() = parameter.as_double();

    } else if (param_name == "/control/maneuver_controller/hover_on_cable_default_yaw_rate") {

        maneuver_scheduler_parameters_->hover_on_cable_default_yaw_rate() = parameter.as_double();

    } else if (param_name == "/control/maneuver_controller/reached_position_euclidean_distance_threshold") {

        fly_to_position_maneuver_server_parameters_->reached_position_euclidean_distance_threshold() = parameter.as_double();
        fly_to_object_maneuver_server_parameters_->reached_position_euclidean_distance_threshold() = parameter.as_double();

    } else if (param_name == "/control/maneuver_controller/minimum_target_altitude") {

        fly_to_position_maneuver_server_parameters_->minimum_target_altitude() = parameter.as_double();
        fly_to_object_maneuver_server_parameters_->minimum_target_altitude() = parameter.as_double();

    }

    after_parameter_change_callback_(parameter);

}