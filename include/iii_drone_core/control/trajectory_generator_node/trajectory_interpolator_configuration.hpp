#pragma once

#include <iii_drone_configuration/configurator.hpp>

#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace iii_drone::control::trajectory_generator_node::detail {

using LifecycleConfigurator =
    iii_drone::configuration::Configurator<rclcpp_lifecycle::LifecycleNode>;

/**
 * @brief Declare trajectory-interpolator parameters and register their shared
 * production configuration view.
 */
inline void ConfigureTrajectoryInterpolator(LifecycleConfigurator & configurator)
{
    const auto double_t = rclcpp::ParameterType::PARAMETER_DOUBLE;
    const auto int_t = rclcpp::ParameterType::PARAMETER_INTEGER;

    configurator.DeclareParameter("/control/dt", double_t);
    configurator.DeclareParameter(
        "/control/trajectory_interpolator/interpolation_avg_velocity_m_s", double_t
    );
    configurator.DeclareParameter(
        "/control/trajectory_interpolator/interpolation_avg_yaw_rate_rad_s", double_t
    );
    configurator.DeclareParameter(
        "/control/trajectory_interpolator/interpolation_max_velocity_m_s", double_t
    );
    configurator.DeclareParameter(
        "/control/trajectory_interpolator/interpolation_max_acceleration_m_s2", double_t
    );
    configurator.DeclareParameter(
        "/control/trajectory_interpolator/interpolation_max_jerk_m_s3", double_t
    );
    configurator.DeclareParameter(
        "/control/trajectory_interpolator/interpolation_max_yaw_rate_rad_s", double_t
    );
    configurator.DeclareParameter(
        "/control/trajectory_interpolator/interpolation_max_yaw_acceleration_rad_s2", double_t
    );
    configurator.DeclareParameter(
        "/control/maneuver_controller/controlled_cancel_max_yaw_jerk_rad_s3", double_t
    );
    configurator.DeclareParameter(
        "/control/trajectory_interpolator/reference_trajectory_length_N", int_t
    );

    configurator.CreateConfiguration("trajectory_interpolator", {
        {"/control/trajectory_interpolator/interpolation_avg_velocity_m_s", double_t},
        {"/control/trajectory_interpolator/interpolation_avg_yaw_rate_rad_s", double_t},
        {"/control/trajectory_interpolator/interpolation_max_velocity_m_s", double_t},
        {"/control/trajectory_interpolator/interpolation_max_acceleration_m_s2", double_t},
        {"/control/trajectory_interpolator/interpolation_max_jerk_m_s3", double_t},
        {"/control/trajectory_interpolator/interpolation_max_yaw_rate_rad_s", double_t},
        {"/control/trajectory_interpolator/interpolation_max_yaw_acceleration_rad_s2", double_t},
        {"/control/maneuver_controller/controlled_cancel_max_yaw_jerk_rad_s3", double_t},
        {"/control/trajectory_interpolator/reference_trajectory_length_N", int_t},
        {"/control/dt", double_t},
    });
}

}  // namespace iii_drone::control::trajectory_generator_node::detail
