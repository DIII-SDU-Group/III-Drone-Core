#include <array>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <iii_drone_core/control/maneuver/maneuver_server.hpp>
#include <iii_drone_core/control/maneuver_controller_node/fly_to_object_configuration.hpp>
#include <iii_drone_core/control/maneuver_controller_node/trajectory_generator_client_configuration.hpp>

namespace {

class RclcppContext {
public:
    RclcppContext() : initialized_here_(!rclcpp::ok()) {
        if (initialized_here_) {
            rclcpp::init(0, nullptr);
        }
    }

    ~RclcppContext() {
        if (initialized_here_) {
            rclcpp::shutdown();
        }
    }

private:
    bool initialized_here_;
};

struct CancellationConfigProbe : iii_drone::control::maneuver::ManeuverServer {
    using ManeuverServer::controlledCancellationConfigFrom;
};

void DeclareFlyToObjectParameters(
    iii_drone::control::maneuver_controller_node::detail::LifecycleConfigurator & configurator)
{
    using Type = rclcpp::ParameterType;
    const std::array<std::pair<const char *, Type>, 16> parameters{{
        {"/control/maneuver_controller/reached_position_euclidean_distance_threshold", Type::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/minimum_target_altitude", Type::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/fly_to_object_use_mpc", Type::PARAMETER_BOOL},
        {"/control/maneuver_controller/fly_to_object_target_low_pass_time_constant_s", Type::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/fly_to_object_target_loss_grace_s", Type::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/reference_stream_timeout_ms", Type::PARAMETER_INTEGER},
        {"/control/maneuver_controller/maneuver_execution_period_ms", Type::PARAMETER_INTEGER},
        {"/tf/world_frame_id", Type::PARAMETER_STRING},
        {"/control/maneuver_controller/cable_landing_controller_type", Type::PARAMETER_STRING},
        {"/control/maneuver_controller/controlled_cancel_max_deceleration_m_s2", Type::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/controlled_cancel_max_jerk_m_s3", Type::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/controlled_cancel_max_yaw_deceleration_rad_s2", Type::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/controlled_cancel_max_yaw_jerk_rad_s3", Type::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/controlled_cancel_velocity_threshold_m_s", Type::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/controlled_cancel_yaw_rate_threshold_rad_s", Type::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/controlled_cancel_settle_time_s", Type::PARAMETER_DOUBLE},
    }};
    for (const auto & [name, type] : parameters) {
        configurator.DeclareParameter(name, type);
    }
}

TEST(ManeuverControllerConfigurationTest, TrajectoryClientViewDeclaresEveryRequiredParameter) {
    RclcppContext context;
    rclcpp::NodeOptions node_options;
    node_options.parameter_overrides({rclcpp::Parameter("/control/dt", 0.2)});
    rclcpp_lifecycle::LifecycleNode node(
        "trajectory_client_configuration_test", "/", node_options
    );
    iii_drone::control::maneuver_controller_node::detail::LifecycleConfigurator configurator(
        &node, "maneuver_controller"
    );

    iii_drone::control::maneuver_controller_node::detail::ConfigureTrajectoryGeneratorClient(
        configurator
    );

    const auto configuration = configurator.GetConfiguration("trajectory_generator_client");
    const std::array<std::pair<std::string, rclcpp::ParameterType>, 4> required_parameters{{
        {"/control/maneuver_controller/generate_trajectories_asynchronously_with_delay",
            rclcpp::ParameterType::PARAMETER_BOOL},
        {"/control/maneuver_controller/generate_trajectories_poll_period_ms",
            rclcpp::ParameterType::PARAMETER_INTEGER},
        {"/control/maneuver_controller/generate_trajectories_timeout_ms",
            rclcpp::ParameterType::PARAMETER_INTEGER},
        {"/control/dt", rclcpp::ParameterType::PARAMETER_DOUBLE},
    }};

    for (const auto & [name, expected_type] : required_parameters) {
        SCOPED_TRACE(name);
        EXPECT_TRUE(configuration->HasParameter(name));
        ASSERT_TRUE(node.has_parameter(name));
        EXPECT_EQ(configuration->GetParameter(name).get_type(), expected_type);
    }

    EXPECT_DOUBLE_EQ(configuration->GetParameter("/control/dt").as_double(), 0.2);
}

TEST(ManeuverControllerConfigurationTest, FlyToObjectViewCarriesLandingCapabilitySelector) {
    RclcppContext context;
    struct Case {
        const char * landing_controller;
        bool fly_to_object_use_mpc;
        bool object_tracking_capable;
    };
    const std::array<Case, 3> cases{{
        {"line_pid", false, true},
        {"mpc", false, false},
        {"line_pid", true, false},
    }};
    for (std::size_t i = 0; i < cases.size(); ++i) {
        const auto & scenario = cases[i];
        SCOPED_TRACE(scenario.landing_controller);
        rclcpp::NodeOptions options;
        options.parameter_overrides({
            rclcpp::Parameter(
                "/control/maneuver_controller/cable_landing_controller_type",
                scenario.landing_controller),
            rclcpp::Parameter(
                "/control/maneuver_controller/fly_to_object_use_mpc",
                scenario.fly_to_object_use_mpc),
            rclcpp::Parameter(
                "/control/maneuver_controller/reference_stream_timeout_ms", 1473),
        });
        rclcpp_lifecycle::LifecycleNode node(
            "fly_to_object_configuration_test_" + std::to_string(i), "/", options);
        iii_drone::control::maneuver_controller_node::detail::LifecycleConfigurator configurator(
            &node, "maneuver_controller");
        DeclareFlyToObjectParameters(configurator);
        iii_drone::control::maneuver_controller_node::detail::ConfigureFlyToObjectManeuverServer(
            configurator);
        const auto configuration = configurator.GetConfiguration("fly_to_object_maneuver_server");
        ASSERT_TRUE(configuration->HasParameter(
            "/control/maneuver_controller/cable_landing_controller_type"));
        const auto selector = configuration->GetParameter(
            "/control/maneuver_controller/cable_landing_controller_type");
        EXPECT_EQ(selector.get_type(), rclcpp::ParameterType::PARAMETER_STRING);
        EXPECT_EQ(selector.as_string(), scenario.landing_controller);
        ASSERT_TRUE(configuration->HasParameter(
            "/control/maneuver_controller/fly_to_object_use_mpc"));
        const bool supports_tracking = !configuration->GetParameter(
            "/control/maneuver_controller/fly_to_object_use_mpc").as_bool() &&
            selector.as_string() == "line_pid";
        EXPECT_EQ(supports_tracking, scenario.object_tracking_capable);
        ASSERT_TRUE(configuration->HasParameter(
            "/control/maneuver_controller/reference_stream_timeout_ms"));
        EXPECT_EQ(configuration->GetParameter(
            "/control/maneuver_controller/reference_stream_timeout_ms").as_int(), 1473);
    }
}

TEST(ManeuverControllerConfigurationTest, FlyToObjectViewFeedsConfiguredCancellationLimits) {
    RclcppContext context;
    const std::array<std::pair<const char *, double>, 7> values{{
        {"/control/maneuver_controller/controlled_cancel_max_deceleration_m_s2", 0.31},
        {"/control/maneuver_controller/controlled_cancel_max_jerk_m_s3", 0.72},
        {"/control/maneuver_controller/controlled_cancel_max_yaw_deceleration_rad_s2", 0.83},
        {"/control/maneuver_controller/controlled_cancel_max_yaw_jerk_rad_s3", 1.29},
        {"/control/maneuver_controller/controlled_cancel_velocity_threshold_m_s", 0.043},
        {"/control/maneuver_controller/controlled_cancel_yaw_rate_threshold_rad_s", 0.054},
        {"/control/maneuver_controller/controlled_cancel_settle_time_s", 0.37},
    }};
    std::vector<rclcpp::Parameter> overrides;
    for (const auto & [name, value] : values) {
        overrides.emplace_back(name, value);
    }
    overrides.emplace_back(
        "/control/maneuver_controller/cable_landing_controller_type", "line_pid");
    overrides.emplace_back("/control/maneuver_controller/fly_to_object_use_mpc", false);
    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides);
    rclcpp_lifecycle::LifecycleNode node("fly_to_object_cancellation_configuration_test", "/", options);
    iii_drone::control::maneuver_controller_node::detail::LifecycleConfigurator configurator(
        &node, "maneuver_controller");
    DeclareFlyToObjectParameters(configurator);
    iii_drone::control::maneuver_controller_node::detail::ConfigureFlyToObjectManeuverServer(
        configurator);
    const auto configuration = configurator.GetConfiguration("fly_to_object_maneuver_server");
    for (const auto & [name, value] : values) {
        SCOPED_TRACE(name);
        ASSERT_TRUE(configuration->HasParameter(name));
        EXPECT_DOUBLE_EQ(configuration->GetParameter(name).as_double(), value);
    }
    const auto cancellation = CancellationConfigProbe::controlledCancellationConfigFrom(
        configuration);
    EXPECT_DOUBLE_EQ(cancellation.limits.max_acceleration_m_s2, values[0].second);
    EXPECT_DOUBLE_EQ(cancellation.limits.max_jerk_m_s3, values[1].second);
    EXPECT_DOUBLE_EQ(cancellation.limits.max_yaw_acceleration_rad_s2, values[2].second);
    EXPECT_DOUBLE_EQ(cancellation.limits.max_yaw_jerk_rad_s3, values[3].second);
    EXPECT_DOUBLE_EQ(cancellation.velocity_threshold_m_s, values[4].second);
    EXPECT_DOUBLE_EQ(cancellation.yaw_rate_threshold_rad_s, values[5].second);
    EXPECT_DOUBLE_EQ(cancellation.settle_time_s, values[6].second);
}

}  // namespace
