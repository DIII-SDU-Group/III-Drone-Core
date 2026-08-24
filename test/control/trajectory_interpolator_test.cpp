#include <gtest/gtest.h>

#define private public
#include <iii_drone_core/control/trajectory_interpolator.hpp>
#undef private

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

using iii_drone::configuration::Configuration;
using iii_drone::configuration::configuration_entry_t;
using iii_drone::control::Reference;
using iii_drone::control::State;
using iii_drone::control::TrajectoryInterpolator;
using iii_drone::types::point_t;
using iii_drone::types::vector_t;

namespace {

Configuration::SharedPtr makeConfiguration() {
    const std::unordered_map<std::string, rclcpp::Parameter> values{
        {"/control/trajectory_interpolator/interpolation_avg_velocity_m_s", rclcpp::Parameter("avg_velocity", 0.5)},
        {"/control/trajectory_interpolator/interpolation_avg_yaw_rate_rad_s", rclcpp::Parameter("avg_yaw_rate", 0.5)},
        {"/control/trajectory_interpolator/interpolation_max_velocity_m_s", rclcpp::Parameter("max_velocity", 1.0)},
        {"/control/trajectory_interpolator/interpolation_max_acceleration_m_s2", rclcpp::Parameter("max_acceleration", 0.5)},
        {"/control/trajectory_interpolator/interpolation_max_yaw_rate_rad_s", rclcpp::Parameter("max_yaw_rate", 0.75)},
        {"/control/trajectory_interpolator/interpolation_max_yaw_acceleration_rad_s2", rclcpp::Parameter("max_yaw_acceleration", 0.75)},
        {"/control/trajectory_interpolator/reference_trajectory_length_N", rclcpp::Parameter("trajectory_length", 10)},
        {"/control/dt", rclcpp::Parameter("dt", 0.2)},
    };

    std::vector<configuration_entry_t> entries;
    entries.reserve(values.size());
    for (const auto & [name, value] : values) {
        entries.emplace_back(name, value.get_type());
    }

    return std::make_shared<Configuration>(
        "trajectory_interpolator_test",
        std::move(entries),
        [values](const std::string & name) {
            return values.at(name);
        }
    );
}

double distanceToSegment(
    const point_t & point,
    const point_t & segment_start,
    const point_t & segment_end
) {
    const vector_t segment = segment_end - segment_start;
    const double length_squared = segment.squaredNorm();
    const double fraction = std::clamp(
        static_cast<double>((point - segment_start).dot(segment)) / length_squared,
        0.0,
        1.0
    );
    return (point - (segment_start + fraction * segment)).norm();
}

}  // namespace

TEST(TrajectoryInterpolatorTest, BlendedCornerDoesNotGenerateRunawayExcursion) {
    TrajectoryInterpolator interpolator(makeConfiguration(), nullptr);

    const point_t start_position(-1.236, -13.726, 10.619);
    const point_t target_position(10.121, 22.375, 10.619);
    const Reference start_reference(
        start_position,
        -0.455,
        vector_t(0.625, -0.182, 0.0),
        0.0,
        vector_t(-0.240, 0.070, 0.0),
        0.0
    );
    const Reference target_reference(target_position, -0.455);

    const double duration = interpolator.computeInterpolation(start_reference, target_reference);

    double maximum_deviation = 0.0;
    for (int sample = 0; sample <= 200; ++sample) {
        const double time = duration * static_cast<double>(sample) / 200.0;
        const point_t position = interpolator.positionFunction(time);
        maximum_deviation = std::max(
            maximum_deviation,
            distanceToSegment(position, start_position, target_position)
        );
    }

    EXPECT_LT(duration, 100.0);
    EXPECT_LT(maximum_deviation, 15.0);
}

TEST(TrajectoryInterpolatorTest, StateBasedPointToPointTrajectoryIgnoresResidualVelocity) {
    TrajectoryInterpolator interpolator(makeConfiguration(), nullptr);

    const point_t start_position(0.0, 0.0, 10.0);
    const point_t target_position(20.0, 0.0, 10.0);
    const State start_state(
        start_position,
        vector_t(0.1, -0.2, 0.3),
        0.0,
        vector_t(0.0, 0.0, 0.1)
    );

    interpolator.ComputeReferenceTrajectory(
        start_state,
        Reference(target_position, 0.0),
        true,
        true
    );
    const double duration = (interpolator.end_time_ - interpolator.start_time_).seconds();

    for (int sample = 0; sample <= 100; ++sample) {
        const double time = duration * static_cast<double>(sample) / 100.0;
        const point_t position = interpolator.positionFunction(time);
        EXPECT_NEAR(position.y(), 0.0, 1.0e-5);
        EXPECT_NEAR(position.z(), 10.0, 1.0e-5);
    }
}

TEST(TrajectoryInterpolatorTest, BlendedDescentIntoLongLevelLegDoesNotUndershootAltitude) {
    TrajectoryInterpolator interpolator(makeConfiguration(), nullptr);

    const point_t start_position(2.353, -8.743, 5.338);
    const point_t target_position(7.841, 17.535, 4.347);
    const Reference start_reference(
        start_position,
        -0.988,
        vector_t(-0.232, 0.054, -0.864),
        0.0,
        vector_t(0.061, -0.014, 0.366),
        0.0
    );

    const double duration = interpolator.computeInterpolation(
        start_reference,
        Reference(target_position, -0.988)
    );

    double minimum_z = start_position.z();
    for (int sample = 0; sample <= 500; ++sample) {
        const double time = duration * static_cast<double>(sample) / 500.0;
        minimum_z = std::min(
            minimum_z,
            static_cast<double>(interpolator.positionFunction(time).z())
        );
    }

    EXPECT_GE(minimum_z, target_position.z() - 0.05);
}
