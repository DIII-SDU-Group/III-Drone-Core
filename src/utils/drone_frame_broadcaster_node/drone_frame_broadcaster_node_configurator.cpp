/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "iii_drone_core/utils/drone_frame_broadcaster_node/drone_frame_broadcaster_node_configurator.hpp"

using namespace iii_drone::utils::drone_frame_broadcaster_node;
using namespace iii_drone::configuration;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

DroneFrameBroadcasterConfigurator::DroneFrameBroadcasterConfigurator(
    rclcpp::Node *node,
    std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback
) : Configurator(
        node,
        after_parameter_change_callback
    ) {

    declareNodeParameters();

    RCLCPP_INFO(node_->get_logger(), "Test 102");

}

DroneFrameBroadcasterConfigurator::DroneFrameBroadcasterConfigurator(
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

const std::string DroneFrameBroadcasterConfigurator::drone_frame_id() const {
    
    return GetParameter("/tf/drone_frame_id").as_string();

}

const std::string DroneFrameBroadcasterConfigurator::world_frame_id() const {
    
    return GetParameter("/tf/world_frame_id").as_string();

}

void DroneFrameBroadcasterConfigurator::declareNodeParameters() {

    DeclareParameter<std::string>("/tf/drone_frame_id");
    DeclareParameter<std::string>("/tf/world_frame_id");

}
