#include "iii_drone_core/configuration/tf/tf_configurator.hpp"

using namespace iii_drone::configuration;

TFConfigurator::TFConfigurator(
    rclcpp::Node *node,
    std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback
) : Configurator(
    node,
    after_parameter_change_callback
) {

    declareTFParameters();

}

TFConfigurator::TFConfigurator(
    rclcpp::Node *node, 
    const rclcpp::QoS & qos,
    std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback
) : Configurator(
    node, 
    qos,
    after_parameter_change_callback
) {

    declareTFParameters();

}

const std::string TFConfigurator::drone_frame_id() const {
    
    return GetParameter("/tf/drone_frame_id").as_string();

}

const std::string TFConfigurator::world_frame_id() const {
    
    return GetParameter("/tf/world_frame_id").as_string();

}

const std::string TFConfigurator::cable_gripper_frame_id() const {
    
    return GetParameter("/tf/cable_gripper_frame_id").as_string();

}

const std::string TFConfigurator::mmwave_frame_id() const {
    
    return GetParameter("/tf/mmwave_frame_id").as_string();

}

void TFConfigurator::declareTFParameters() {

    DeclareParameter<std::string>("/tf/drone_frame_id");
    DeclareParameter<std::string>("/tf/world_frame_id");
    DeclareParameter<std::string>("/tf/cable_gripper_frame_id");
    DeclareParameter<std::string>("/tf/mmwave_frame_id");

}