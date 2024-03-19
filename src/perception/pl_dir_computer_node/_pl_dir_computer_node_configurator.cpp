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
        std::bind(
            &PowerlineDirectionComputerConfigurator::powerlineDirectionComputerAfterParameterChangeCallback,
            this,
            std::placeholders::_1
        )
    ) {

    after_parameter_change_callback_ = after_parameter_change_callback;

    declareNodeParameters();

    initializeParameters();

}

PowerlineDirectionComputerConfigurator::PowerlineDirectionComputerConfigurator(
    rclcpp::Node *node, 
    const rclcpp::QoS & qos,
    std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback
) : Configurator(
        node, 
        qos,
        std::bind(
            &PowerlineDirectionComputerConfigurator::powerlineDirectionComputerAfterParameterChangeCallback,
            this,
            std::placeholders::_1
        )
    ) {

    after_parameter_change_callback_ = after_parameter_change_callback;

    declareNodeParameters();

    initializeParameters();

}

float PowerlineDirectionComputerConfigurator::kf_r() const {

    return GetParameter("/perception/pl_dir_computer/kf_r").as_double();

}

float PowerlineDirectionComputerConfigurator::kf_q() const {

    return GetParameter("/perception/pl_dir_computer/kf_q").as_double();

}

int PowerlineDirectionComputerConfigurator::init_sleep_time_ms() const {

    return GetParameter("/perception/pl_dir_computer/init_sleep_time_ms").as_int();

}

int PowerlineDirectionComputerConfigurator::odometry_callback_period_ms() const {

    return GetParameter("/perception/pl_dir_computer/odometry_callback_period_ms").as_int();

}

float PowerlineDirectionComputerConfigurator::min_point_dist() const {

    return GetParameter("/perception/pl_mapper/min_point_dist").as_double();

}

float PowerlineDirectionComputerConfigurator::max_point_dist() const {

    return GetParameter("/perception/pl_mapper/max_point_dist").as_double();

}

float PowerlineDirectionComputerConfigurator::view_cone_slope() const {

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

std::shared_ptr<iii_drone::perception::PowerlineDirectionParameters> PowerlineDirectionComputerConfigurator::powerline_direction_parameters() const {

    return powerline_direction_parameters_;

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

void PowerlineDirectionComputerConfigurator::initializeParameters() {

    powerline_direction_parameters_ = std::make_shared<iii_drone::perception::PowerlineDirectionParameters>(
        drone_frame_id(),
        kf_r(),
        kf_q()
    );

}

void PowerlineDirectionComputerConfigurator::PowerlineDirectionComputerConfigurator::powerlineDirectionComputerAfterParameterChangeCallback(const rclcpp::Parameter & parameter) {

    if (parameter.get_name() == "/perception/pl_dir_computer/kf_r") {

        powerline_direction_parameters_->kf_r() = parameter.as_double();

    } else if (parameter.get_name() == "/perception/pl_dir_computer/kf_q") {

        powerline_direction_parameters_->kf_q() = parameter.as_double();

    } else if (parameter.get_name() == "/tf/drone_frame_id") {

        powerline_direction_parameters_->drone_frame_id() = parameter.as_string();

    }

    if (after_parameter_change_callback_ != nullptr)
        after_parameter_change_callback_(parameter);

}
