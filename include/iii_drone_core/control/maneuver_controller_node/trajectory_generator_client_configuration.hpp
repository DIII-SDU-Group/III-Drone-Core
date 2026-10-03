#pragma once

#include <iii_drone_configuration/configurator.hpp>

#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace iii_drone::control::maneuver_controller_node::detail {

using LifecycleConfigurator =
    iii_drone::configuration::Configurator<rclcpp_lifecycle::LifecycleNode>;

/**
 * @brief Declare the trajectory client parameters and create its configuration
 * view from the production registration path.
 */
inline void ConfigureTrajectoryGeneratorClient(LifecycleConfigurator & configurator)
{
    const auto bool_t = rclcpp::ParameterType::PARAMETER_BOOL;
    const auto int_t = rclcpp::ParameterType::PARAMETER_INTEGER;
    const auto double_t = rclcpp::ParameterType::PARAMETER_DOUBLE;

    configurator.DeclareParameter(
        "/control/maneuver_controller/generate_trajectories_asynchronously_with_delay",
        bool_t
    );
    configurator.DeclareParameter(
        "/control/maneuver_controller/generate_trajectories_poll_period_ms",
        int_t
    );
    configurator.DeclareParameter(
        "/control/maneuver_controller/generate_trajectories_timeout_ms",
        int_t
    );
    configurator.DeclareParameter("/control/dt", double_t);
    configurator.CreateConfiguration("trajectory_generator_client", {
        {
            "/control/maneuver_controller/generate_trajectories_asynchronously_with_delay",
            bool_t
        },
        {"/control/maneuver_controller/generate_trajectories_poll_period_ms", int_t},
        {"/control/maneuver_controller/generate_trajectories_timeout_ms", int_t},
        {"/control/dt", double_t},
    });
}

}  // namespace iii_drone::control::maneuver_controller_node::detail
