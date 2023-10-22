/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "iii_drone_core/perception/hough_transformer_node/hough_transformer_node_configurator.hpp"

using namespace iii_drone::perception::hough_transformer_node;
using namespace iii_drone::configuration;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

HoughTransformerConfigurator::HoughTransformerConfigurator(
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

HoughTransformerConfigurator::HoughTransformerConfigurator(
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

const int HoughTransformerConfigurator::canny_low_threshold() const {

    return GetParameter("/perception/hough_transformer/canny_low_threshold").as_int();

}

const int HoughTransformerConfigurator::canny_ratio() const {

    return GetParameter("/perception/hough_transformer/canny_ratio").as_int();

}

const int HoughTransformerConfigurator::canny_kernel_size() const {

    return GetParameter("/perception/hough_transformer/canny_kernel_size").as_int();

}

const int HoughTransformerConfigurator::n_lines_include() const {

    return GetParameter("/perception/hough_transformer/n_lines_include").as_int();

}

void HoughTransformerConfigurator::declareNodeParameters() {

    DeclareParameter<int>("/perception/hough_transformer/canny_low_threshold");
    DeclareParameter<int>("/perception/hough_transformer/canny_ratio");
    DeclareParameter<int>("/perception/hough_transformer/canny_kernel_size");
    DeclareParameter<int>("/perception/hough_transformer/n_lines_include");

}
