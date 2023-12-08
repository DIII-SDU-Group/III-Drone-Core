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
        std::bind(
            &HoughTransformerConfigurator::houghTransformerConfiguratorParameterChangeCallback,
            this,
            std::placeholders::_1
        )
    ) {

    after_parameter_change_callback_ = after_parameter_change_callback;

    declareNodeParameters();
    initHoughTransformerParameters();

}

HoughTransformerConfigurator::HoughTransformerConfigurator(
    rclcpp::Node *node, 
    const rclcpp::QoS & qos,
    std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback
) : Configurator(
        node, 
        qos,
        std::bind(
            &HoughTransformerConfigurator::houghTransformerConfiguratorParameterChangeCallback,
            this,
            std::placeholders::_1
        )
    ) {

    after_parameter_change_callback_ = after_parameter_change_callback;

    declareNodeParameters();
    initHoughTransformerParameters();

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

const std::string HoughTransformerConfigurator::drone_frame_id() const {
    
    return GetParameter("/tf/drone_frame_id").as_string();

}

const std::string HoughTransformerConfigurator::world_frame_id() const {
    
    return GetParameter("/tf/world_frame_id").as_string();

}

const std::string HoughTransformerConfigurator::cable_gripper_frame_id() const {
    
    return GetParameter("/tf/cable_gripper_frame_id").as_string();

}

const std::string HoughTransformerConfigurator::mmwave_frame_id() const {
    
    return GetParameter("/tf/mmwave_frame_id").as_string();

}

std::shared_ptr<iii_drone::perception::HoughTransformerParameters> HoughTransformerConfigurator::hough_transformer_parameters() const {

    return hough_transformer_parameters_;

}

void HoughTransformerConfigurator::declareNodeParameters() {

    DeclareParameter<std::string>("/tf/drone_frame_id");
    DeclareParameter<std::string>("/tf/world_frame_id");
    DeclareParameter<std::string>("/tf/cable_gripper_frame_id");
    DeclareParameter<std::string>("/tf/mmwave_frame_id");

    DeclareParameter<int>("/perception/hough_transformer/canny_low_threshold");
    DeclareParameter<int>("/perception/hough_transformer/canny_ratio");
    DeclareParameter<int>("/perception/hough_transformer/canny_kernel_size");
    DeclareParameter<int>("/perception/hough_transformer/n_lines_include");

}

void HoughTransformerConfigurator::initHoughTransformerParameters() {

    hough_transformer_parameters_ = std::make_shared<iii_drone::perception::HoughTransformerParameters>(
        canny_low_threshold(),
        canny_ratio(),
        canny_kernel_size()
    );

}

void HoughTransformerConfigurator::houghTransformerConfiguratorParameterChangeCallback(const rclcpp::Parameter & parameter) {
    
    if (parameter.get_name() == "/perception/hough_transformer/canny_low_threshold") {

        hough_transformer_parameters_->canny_low_threshold() = parameter.as_int();

    } else if (parameter.get_name() == "/perception/hough_transformer/canny_ratio") {

        hough_transformer_parameters_->canny_ratio() = parameter.as_int();

    } else if (parameter.get_name() == "/perception/hough_transformer/canny_kernel_size") {

        hough_transformer_parameters_->canny_kernel_size() = parameter.as_int();

    }

    // If after_parameter_change_callback_ is not null, call it
    if (after_parameter_change_callback_) {
        after_parameter_change_callback_(parameter);
    }
}
