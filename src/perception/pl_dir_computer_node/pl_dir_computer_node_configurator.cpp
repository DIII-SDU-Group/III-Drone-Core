/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "iii_drone_core/perception/pl_dir_computer_node/pl_dir_computer_node_configurator.hpp"

using namespace iii_drone::perception::pl_dir_computer_node;
using namespace iii_drone::configuration;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

PowerlineDirectionComputerConfigurator::PowerlineDirectionComputerConfigurator(
    rclcpp::Node *node,
    std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback
) : Configurator(
        node,
        after_parameter_change_callback
    ) {

    declareNodeParameters();

}

PowerlineDirectionComputerConfigurator::PowerlineDirectionComputerConfigurator(
    rclcpp::Node *node, 
    const rclcpp::QoS & qos,
    std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback
) : Configurator(
        node, 
        qos,
        after_parameter_change_callback
    ) {

    declareNodeParameters();

}

const float PowerlineDirectionComputerConfigurator::kf_r() const {

    return GetParameter("/perception/pl_dir_computer/kf_r").as_double();

}

const float PowerlineDirectionComputerConfigurator::kf_q() const {

    return GetParameter("/perception/pl_dir_computer/kf_q").as_double();

}

const int PowerlineDirectionComputerConfigurator::init_sleep_time_ms() const {

    return GetParameter("/perception/pl_dir_computer/init_sleep_time_ms").as_int();

}

const int PowerlineDirectionComputerConfigurator::odometry_callback_period_ms() const {

    return GetParameter("/perception/pl_dir_computer/odometry_callback_period_ms").as_int();

}

const float PowerlineDirectionComputerConfigurator::min_point_dist() const {

    return GetParameter("/perception/pl_mapper/min_point_dist").as_double();

}

const float PowerlineDirectionComputerConfigurator::max_point_dist() const {

    return GetParameter("/perception/pl_mapper/max_point_dist").as_double();

}

const float PowerlineDirectionComputerConfigurator::view_cone_slope() const {

    return GetParameter("/perception/pl_mapper/view_cone_slope").as_double();

}

const std::string PowerlineDirectionComputerConfigurator::drone_frame_id() const {
    
    return GetParameter("/tf/drone_frame_id").as_string();

}

const std::string PowerlineDirectionComputerConfigurator::world_frame_id() const {
    
    return GetParameter("/tf/world_frame_id").as_string();

}

const std::string PowerlineDirectionComputerConfigurator::cable_gripper_frame_id() const {
    
    return GetParameter("/tf/cable_gripper_frame_id").as_string();

}

const std::string PowerlineDirectionComputerConfigurator::mmwave_frame_id() const {
    
    return GetParameter("/tf/mmwave_frame_id").as_string();

}

void PowerlineDirectionComputerConfigurator::declareNodeParameters() {

    DeclareParameter<float>("/perception/pl_dir_computer/kf_r");
    DeclareParameter<float>("/perception/pl_dir_computer/kf_q");
    DeclareParameter<int>("/perception/pl_dir_computer/init_sleep_time_ms");
    DeclareParameter<int>("/perception/pl_dir_computer/odometry_callback_period_ms");

    DeclareParameter<float>("/perception/pl_mapper/min_point_dist");
    DeclareParameter<float>("/perception/pl_mapper/max_point_dist");
    DeclareParameter<float>("/perception/pl_mapper/view_cone_slope");

    DeclareParameter<std::string>("/tf/drone_frame_id");
    DeclareParameter<std::string>("/tf/world_frame_id");
    DeclareParameter<std::string>("/tf/cable_gripper_frame_id");
    DeclareParameter<std::string>("/tf/mmwave_frame_id");

}
