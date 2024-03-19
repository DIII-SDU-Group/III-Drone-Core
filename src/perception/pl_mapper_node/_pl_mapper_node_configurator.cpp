/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "iii_drone_core/perception/pl_mapper_node/pl_mapper_node_configurator.hpp"

using namespace iii_drone::perception::pl_mapper_node;
using namespace iii_drone::configuration;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

PowerlineMapperConfigurator::PowerlineMapperConfigurator(
    rclcpp::Node *node,
    std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback
) : Configurator(
        node,
        std::bind(
            &PowerlineMapperConfigurator::plMapperOnParameterChange,
            this,
            std::placeholders::_1
        )
    ) {

    after_parameter_change_callback_ = after_parameter_change_callback;

    declareNodeParameters();

    initializeParameters();

}

PowerlineMapperConfigurator::PowerlineMapperConfigurator(
    rclcpp::Node *node, 
    const rclcpp::QoS & qos,
    std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback
) : Configurator(
        node, 
        qos,
        std::bind(
            &PowerlineMapperConfigurator::plMapperOnParameterChange,
            this,
            std::placeholders::_1
        )
    ) {

    after_parameter_change_callback_ = after_parameter_change_callback;

    declareNodeParameters();

    initializeParameters();

}

float PowerlineMapperConfigurator::kf_r() const {
    
    return GetParameter("/perception/pl_mapper/kf_r").as_double();

}

float PowerlineMapperConfigurator::kf_q() const {
    
    return GetParameter("/perception/pl_mapper/kf_q").as_double();

}

int PowerlineMapperConfigurator::alive_cnt_low_thresh() const {
    
    return GetParameter("/perception/pl_mapper/alive_cnt_low_thresh").as_int();

}

int PowerlineMapperConfigurator::alive_cnt_high_thresh() const {
    
    return GetParameter("/perception/pl_mapper/alive_cnt_high_thresh").as_int();

}

int PowerlineMapperConfigurator::alive_cnt_ceiling() const {
    
    return GetParameter("/perception/pl_mapper/alive_cnt_ceiling").as_int();

}

float PowerlineMapperConfigurator::min_point_dist() const {
    
    return GetParameter("/perception/pl_mapper/min_point_dist").as_double();

}

float PowerlineMapperConfigurator::max_point_dist() const {
    
    return GetParameter("/perception/pl_mapper/max_point_dist").as_double();

}

float PowerlineMapperConfigurator::view_cone_slope() const {
    
    return GetParameter("/perception/pl_mapper/view_cone_slope").as_double();

}

float PowerlineMapperConfigurator::strict_min_point_dist() const {
    
    return GetParameter("/perception/pl_mapper/strict_min_point_dist").as_double();

}

float PowerlineMapperConfigurator::strict_max_point_dist() const {
    
    return GetParameter("/perception/pl_mapper/strict_max_point_dist").as_double();

}

float PowerlineMapperConfigurator::strict_view_cone_slope() const {
    
    return GetParameter("/perception/pl_mapper/strict_view_cone_slope").as_double();

}

float PowerlineMapperConfigurator::matching_line_max_dist() const {
    
    return GetParameter("/perception/pl_mapper/matching_line_max_dist").as_double();

}

int PowerlineMapperConfigurator::init_sleep_time_ms() const {
    
    return GetParameter("/perception/pl_mapper/init_sleep_time_ms").as_int();

}

int PowerlineMapperConfigurator::odometry_callback_period_ms() const {
    
    return GetParameter("/perception/pl_mapper/odometry_callback_period_ms").as_int();

}

int PowerlineMapperConfigurator::max_lines() const {
    
    return GetParameter("/perception/pl_mapper/max_lines").as_int();

}

int PowerlineMapperConfigurator::inter_pos_window_size() const {
    
    return GetParameter("/perception/pl_mapper/inter_pos_window_size").as_int();

}

bool PowerlineMapperConfigurator::simulation() const {
    
    return GetParameter("/global/simulation").as_bool();

}

const std::string PowerlineMapperConfigurator::drone_frame_id() const {
    
    return GetParameter("/tf/drone_frame_id").as_string();

}

const std::string PowerlineMapperConfigurator::world_frame_id() const {
    
    return GetParameter("/tf/world_frame_id").as_string();

}

const std::string PowerlineMapperConfigurator::cable_gripper_frame_id() const {
    
    return GetParameter("/tf/cable_gripper_frame_id").as_string();

}

const std::string PowerlineMapperConfigurator::mmwave_frame_id() const {
    
    return GetParameter("/tf/mmwave_frame_id").as_string();

}

std::shared_ptr<iii_drone::perception::PowerlineParameters> PowerlineMapperConfigurator::powerline_parameters() const {
    
    return parameters_;

}

void PowerlineMapperConfigurator::declareNodeParameters() {

    DeclareParameter<float>("/perception/pl_mapper/kf_r");
    DeclareParameter<float>("/perception/pl_mapper/kf_q");
    DeclareParameter<int>("/perception/pl_mapper/alive_cnt_low_thresh");
    DeclareParameter<int>("/perception/pl_mapper/alive_cnt_high_thresh");
    DeclareParameter<int>("/perception/pl_mapper/alive_cnt_ceiling");
    DeclareParameter<float>("/perception/pl_mapper/min_point_dist");
    DeclareParameter<float>("/perception/pl_mapper/max_point_dist");
    DeclareParameter<float>("/perception/pl_mapper/view_cone_slope");
    DeclareParameter<float>("/perception/pl_mapper/strict_min_point_dist");
    DeclareParameter<float>("/perception/pl_mapper/strict_max_point_dist");
    DeclareParameter<float>("/perception/pl_mapper/strict_view_cone_slope");
    DeclareParameter<float>("/perception/pl_mapper/matching_line_max_dist");
    DeclareParameter<int>("/perception/pl_mapper/init_sleep_time_ms");
    DeclareParameter<int>("/perception/pl_mapper/odometry_callback_period_ms");
    DeclareParameter<int>("/perception/pl_mapper/max_lines");
    DeclareParameter<int>("/perception/pl_mapper/inter_pos_window_size");

    DeclareParameter<bool>("/global/simulation");

    DeclareParameter<std::string>("/tf/drone_frame_id");
    DeclareParameter<std::string>("/tf/world_frame_id");
    DeclareParameter<std::string>("/tf/cable_gripper_frame_id");
    DeclareParameter<std::string>("/tf/mmwave_frame_id");

}

void PowerlineMapperConfigurator::initializeParameters() {

    parameters_ = std::make_shared<PowerlineParameters>(
        kf_r(),
        kf_q(),
        alive_cnt_low_thresh(),
        alive_cnt_high_thresh(),
        alive_cnt_ceiling(),
        matching_line_max_dist(),
        max_lines(),
        min_point_dist(),
        max_point_dist(),
        view_cone_slope(),
        strict_min_point_dist(),
        strict_max_point_dist(),
        strict_view_cone_slope(),
        inter_pos_window_size(),
        drone_frame_id(),
        mmwave_frame_id(),
        simulation()
    );

}

void PowerlineMapperConfigurator::plMapperOnParameterChange(const rclcpp::Parameter &parameter) {

    if (parameter.get_name() == "/perception/pl_mapper/kf_r") {
        parameters_->kf_r() = parameter.as_double();
    } else if (parameter.get_name() == "/perception/pl_mapper/kf_q") {
        parameters_->kf_q() = parameter.as_double();
    } else if (parameter.get_name() == "/perception/pl_mapper/alive_cnt_low_thresh") {
        parameters_->alive_cnt_low_thresh() = parameter.as_int();
    } else if (parameter.get_name() == "/perception/pl_mapper/alive_cnt_high_thresh") {
        parameters_->alive_cnt_high_thresh() = parameter.as_int();
    } else if (parameter.get_name() == "/perception/pl_mapper/alive_cnt_ceiling") {
        parameters_->alive_cnt_ceiling() = parameter.as_int();
    } else if (parameter.get_name() == "/perception/pl_mapper/matching_line_max_dist") {
        parameters_->matching_line_max_dist() = parameter.as_double();
    } else if (parameter.get_name() == "/perception/pl_mapper/max_lines") {
        parameters_->max_lines() = parameter.as_int();
    } else if (parameter.get_name() == "/perception/pl_mapper/min_point_dist") {
        parameters_->min_point_dist() = parameter.as_double();
    } else if (parameter.get_name() == "/perception/pl_mapper/max_point_dist") {
        parameters_->max_point_dist() = parameter.as_double();
    } else if (parameter.get_name() == "/perception/pl_mapper/view_cone_slope") {
        parameters_->view_cone_slope() = parameter.as_double();
    } else if (parameter.get_name() == "/perception/pl_mapper/strict_min_point_dist") {
        parameters_->strict_min_point_dist() = parameter.as_double();
    } else if (parameter.get_name() == "/perception/pl_mapper/strict_max_point_dist") {
        parameters_->strict_max_point_dist() = parameter.as_double();
    } else if (parameter.get_name() == "/perception/pl_mapper/strict_view_cone_slope") {
        parameters_->strict_view_cone_slope() = parameter.as_double();
    } else if (parameter.get_name() == "/perception/pl_mapper/inter_pos_window_size") {
        parameters_->inter_pos_window_size() = parameter.as_int();
    }

    if (after_parameter_change_callback_ != nullptr) {
        after_parameter_change_callback_(parameter);
    }

}