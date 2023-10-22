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
        after_parameter_change_callback
    ), TFConfigurator(
        node,
        after_parameter_change_callback
    ) {

    declareNodeParameters();

}

PowerlineMapperConfigurator::PowerlineMapperConfigurator(
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

const float PowerlineMapperConfigurator::kf_r() const {
    
    return GetParameter("/perception/pl_mapper/kf_r").as_double();

}

const float PowerlineMapperConfigurator::kf_q() const {
    
    return GetParameter("/perception/pl_mapper/kf_q").as_double();

}

const int PowerlineMapperConfigurator::alive_cnt_low_thresh() const {
    
    return GetParameter("/perception/pl_mapper/alive_cnt_low_thresh").as_int();

}

const int PowerlineMapperConfigurator::alive_cnt_high_thresh() const {
    
    return GetParameter("/perception/pl_mapper/alive_cnt_high_thresh").as_int();

}

const int PowerlineMapperConfigurator::alive_cnt_ceiling() const {
    
    return GetParameter("/perception/pl_mapper/alive_cnt_ceiling").as_int();

}

const float PowerlineMapperConfigurator::min_point_dist() const {
    
    return GetParameter("/perception/pl_mapper/min_point_dist").as_double();

}

const float PowerlineMapperConfigurator::max_point_dist() const {
    
    return GetParameter("/perception/pl_mapper/max_point_dist").as_double();

}

const float PowerlineMapperConfigurator::view_cone_slope() const {
    
    return GetParameter("/perception/pl_mapper/view_cone_slope").as_double();

}

const float PowerlineMapperConfigurator::min_point_dist_strict() const {
    
    return GetParameter("/perception/pl_mapper/min_point_dist_strict").as_double();

}

const float PowerlineMapperConfigurator::max_point_dist_strict() const {
    
    return GetParameter("/perception/pl_mapper/max_point_dist_strict").as_double();

}

const float PowerlineMapperConfigurator::view_cone_slope_strict() const {
    
    return GetParameter("/perception/pl_mapper/view_cone_slope_strict").as_double();

}

const float PowerlineMapperConfigurator::matching_line_max_dist() const {
    
    return GetParameter("/perception/pl_mapper/matching_line_max_dist").as_double();

}

const int PowerlineMapperConfigurator::init_sleep_time_ms() const {
    
    return GetParameter("/perception/pl_mapper/init_sleep_time_ms").as_int();

}

const int PowerlineMapperConfigurator::odometry_callback_period_ms() const {
    
    return GetParameter("/perception/pl_mapper/odometry_callback_period_ms").as_int();

}

const int PowerlineMapperConfigurator::max_lines() const {
    
    return GetParameter("/perception/pl_mapper/max_lines").as_int();

}

const bool PowerlineMapperConfigurator::skip_predict_when_on_cable() const {
    
    return GetParameter("/perception/pl_mapper/skip_predict_when_on_cable").as_bool();

}

const int PowerlineMapperConfigurator::inter_pos_window_size() const {
    
    return GetParameter("/perception/pl_mapper/inter_pos_window_size").as_int();

}

const bool PowerlineMapperConfigurator::simulation() const {
    
    return GetParameter("/global/simulation").as_bool();

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
    DeclareParameter<float>("/perception/pl_mapper/min_point_dist_strict");
    DeclareParameter<float>("/perception/pl_mapper/max_point_dist_strict");
    DeclareParameter<float>("/perception/pl_mapper/view_cone_slope_strict");
    DeclareParameter<float>("/perception/pl_mapper/matching_line_max_dist");
    DeclareParameter<int>("/perception/pl_mapper/init_sleep_time_ms");
    DeclareParameter<int>("/perception/pl_mapper/odometry_callback_period_ms");
    DeclareParameter<int>("/perception/pl_mapper/max_lines");
    DeclareParameter<bool>("/perception/pl_mapper/skip_predict_when_on_cable");
    DeclareParameter<int>("/perception/pl_mapper/inter_pos_window_size");

    DeclareParameter<bool>("/global/simulation");

}
