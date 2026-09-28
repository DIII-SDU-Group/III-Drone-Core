#pragma once

#include <iii_drone_configuration/configurator.hpp>

#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace iii_drone::control::maneuver_controller_node::detail {

// The FTO configuration view used by the production controller registration.
inline void ConfigureFlyToObjectManeuverServer(
    iii_drone::configuration::Configurator<rclcpp_lifecycle::LifecycleNode> & configurator)
{
    using iii_drone::configuration::configuration_entry_t;
    const auto bool_t = rclcpp::ParameterType::PARAMETER_BOOL;
    const auto int_t = rclcpp::ParameterType::PARAMETER_INTEGER;
    const auto double_t = rclcpp::ParameterType::PARAMETER_DOUBLE;
    const auto string_t = rclcpp::ParameterType::PARAMETER_STRING;

    configurator.CreateConfiguration("fly_to_object_maneuver_server", {
        configuration_entry_t("/control/maneuver_controller/reached_position_euclidean_distance_threshold", double_t),
        configuration_entry_t("/control/maneuver_controller/minimum_target_altitude", double_t),
        configuration_entry_t("/control/maneuver_controller/fly_to_object_use_mpc", bool_t),
        configuration_entry_t("/control/maneuver_controller/fly_to_object_target_low_pass_time_constant_s", double_t),
        configuration_entry_t("/control/maneuver_controller/fly_to_object_target_loss_grace_s", double_t),
        configuration_entry_t("/control/maneuver_controller/reference_stream_timeout_ms", int_t),
        configuration_entry_t("/control/maneuver_controller/cable_landing_controller_type", string_t),
        configuration_entry_t("/control/maneuver_controller/controlled_cancel_max_deceleration_m_s2", double_t),
        configuration_entry_t("/control/maneuver_controller/controlled_cancel_max_jerk_m_s3", double_t),
        configuration_entry_t("/control/maneuver_controller/controlled_cancel_max_yaw_deceleration_rad_s2", double_t),
        configuration_entry_t("/control/maneuver_controller/controlled_cancel_max_yaw_jerk_rad_s3", double_t),
        configuration_entry_t("/control/maneuver_controller/controlled_cancel_velocity_threshold_m_s", double_t),
        configuration_entry_t("/control/maneuver_controller/controlled_cancel_yaw_rate_threshold_rad_s", double_t),
        configuration_entry_t("/control/maneuver_controller/controlled_cancel_settle_time_s", double_t),
        configuration_entry_t("/control/maneuver_controller/maneuver_execution_period_ms", int_t),
        configuration_entry_t("/tf/world_frame_id", string_t),
    });
}

}  // namespace iii_drone::control::maneuver_controller_node::detail
