#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <iii_drone_core/control/trajectory_generator.hpp>

namespace {

using iii_drone::configuration::Configuration;
using iii_drone::configuration::configuration_entry_t;
using iii_drone::control::Reference;
using iii_drone::control::State;
using iii_drone::control::TrajectoryGenerator;
using iii_drone::control::positional;
using iii_drone::types::point_t;
using iii_drone::types::vector_t;

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

Configuration::SharedPtr makeMpcConfiguration() {
    const std::vector<configuration_entry_t> entries{
        {"/control/trajectory_generator/MPC_use_state_feedback", rclcpp::ParameterType::PARAMETER_BOOL},
        {"/control/trajectory_generator/MPC_N", rclcpp::ParameterType::PARAMETER_INTEGER},
        {"/control/dt", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/trajectory_generator/position_MPC_vx_max", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/trajectory_generator/position_MPC_vy_max", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/trajectory_generator/position_MPC_vz_max", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/trajectory_generator/position_MPC_ax_max", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/trajectory_generator/position_MPC_ay_max", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/trajectory_generator/position_MPC_az_max", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/trajectory_generator/position_MPC_wx", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/trajectory_generator/position_MPC_wy", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/trajectory_generator/position_MPC_wz", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/trajectory_generator/position_MPC_wvx", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/trajectory_generator/position_MPC_wvy", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/trajectory_generator/position_MPC_wvz", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/trajectory_generator/position_MPC_wax", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/trajectory_generator/position_MPC_way", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/trajectory_generator/position_MPC_waz", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/trajectory_generator/position_MPC_wjx", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/trajectory_generator/position_MPC_wjy", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/trajectory_generator/position_MPC_wjz", rclcpp::ParameterType::PARAMETER_DOUBLE},
    };
    return std::make_shared<Configuration>(
        "trajectory_generator_test",
        entries,
        [](const std::string & name) {
            if (name == "/control/trajectory_generator/MPC_use_state_feedback") {
                return rclcpp::Parameter(name, true);
            }
            if (name == "/control/trajectory_generator/MPC_N") {
                return rclcpp::Parameter(name, 10);
            }
            if (name == "/control/dt") {
                return rclcpp::Parameter(name, 0.2);
            }
            if (name.ends_with("_vx_max") || name.ends_with("_vy_max") || name.ends_with("_vz_max")) {
                return rclcpp::Parameter(name, 4.0);
            }
            if (name.ends_with("_ax_max") || name.ends_with("_ay_max") || name.ends_with("_az_max")) {
                return rclcpp::Parameter(name, 2.0);
            }
            return rclcpp::Parameter(name, 1.0);
        }
    );
}

}  // namespace

TEST(TrajectoryGeneratorTest, PredictedVelocityUsesMatchingAxisAccelerationAtControlStep) {
    RclcppContext context;
    rclcpp_lifecycle::LifecycleNode node("trajectory_generator_numeric_test");
    const auto parameters = makeMpcConfiguration();
    TrajectoryGenerator generator(parameters, parameters, parameters, &node);
    const State state(
        point_t::Zero(), vector_t(0.0, 0.0, 2.0), 0.0, vector_t::Zero(), rclcpp::Time(1, 0)
    );
    const Reference target(
        point_t(0.0, 0.0, 1.0), 0.0, vector_t::Zero(), 0.0,
        vector_t::Zero(), 0.0, rclcpp::Time(1, 0)
    );

    const auto trajectory = generator.ComputeReferenceTrajectory(
        state, target, true, true, positional
    );
    ASSERT_GE(trajectory.references().size(), 3U);
    const Reference reset_seed(state);
    const double seed_to_first_mpc_acceleration_jump = (
        trajectory.references()[0].acceleration() - reset_seed.acceleration()
    ).norm();
    // The startup seed has zero acceleration. With a 2 m/s vertical state,
    // this sample's first MPC acceleration exceeds the guard's default
    // 0.75 + elapsed_s envelope at both the 50 ms producer and 200 ms MPC
    // intervals. This preserves numerical evidence for a separate startup
    // contract review without changing guard limits here.
    EXPECT_GT(seed_to_first_mpc_acceleration_jump, 0.75 + 0.05);
    EXPECT_GT(seed_to_first_mpc_acceleration_jump, 0.75 + 0.2);
    for (size_t step = 0; step + 1U < trajectory.references().size(); ++step) {
        const auto & current = trajectory.references()[step];
        const auto & next = trajectory.references()[step + 1U];
        const vector_t velocity_delta = next.velocity() - current.velocity();
        const vector_t expected_delta = current.acceleration() * 0.2;
        EXPECT_TRUE(velocity_delta.isApprox(expected_delta, 2.0e-2))
            << "step=" << step << " delta=" << velocity_delta.transpose()
            << " expected=" << expected_delta.transpose();
    }
}
