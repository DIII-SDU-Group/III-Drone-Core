#include <gtest/gtest.h>

#define private public
#include <iii_drone_core/control/trajectory_interpolator.hpp>
#undef private
#include <iii_drone_core/control/trajectory_generator_client.hpp>
#include <iii_drone_core/adapters/px4/trajectory_setpoint_adapter.hpp>
#include <iii_drone_core/control/maneuver/maneuver_reference_safety_guard.hpp>
#include <iii_drone_core/control/maneuver/object_tracking_session.hpp>
#include <iii_drone_core/control/combined_drone_awareness_handler.hpp>
#include <iii_drone_core/adapters/px4/vehicle_odometry_adapter.hpp>
#include <iii_drone_core/control/trajectory_generator_node/trajectory_interpolator_configuration.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

using iii_drone::configuration::Configuration;
using iii_drone::configuration::configuration_entry_t;
using iii_drone::control::Reference;
using iii_drone::control::State;
using iii_drone::control::TrajectoryInterpolator;
using iii_drone::control::TrajectoryGeneratorClient;
using iii_drone::control::maneuver::ManeuverReferenceSafetyConfig;
using iii_drone::control::maneuver::ManeuverReferenceSafetyDecision;
using iii_drone::control::maneuver::ManeuverReferenceSafetyGuard;
using iii_drone::types::point_t;
using iii_drone::types::vector_t;

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

class ExecutorThreadGuard {
public:
    ExecutorThreadGuard(
        rclcpp::executors::MultiThreadedExecutor & executor,
        std::thread & thread
    ) : executor_(executor), thread_(thread) { }

    ~ExecutorThreadGuard() { stop(); }

    void stop() {
        executor_.cancel();
        if (thread_.joinable()) {
            thread_.join();
        }
    }

private:
    rclcpp::executors::MultiThreadedExecutor & executor_;
    std::thread & thread_;
};

Configuration::SharedPtr makeConfiguration(
    double max_velocity = 1.0,
    double max_acceleration = 0.5,
    double max_jerk = 1.0
) {
    const std::unordered_map<std::string, rclcpp::Parameter> values{
        {"/control/trajectory_interpolator/interpolation_avg_velocity_m_s", rclcpp::Parameter("avg_velocity", 0.5)},
        {"/control/trajectory_interpolator/interpolation_avg_yaw_rate_rad_s", rclcpp::Parameter("avg_yaw_rate", 0.5)},
        {"/control/trajectory_interpolator/interpolation_max_velocity_m_s", rclcpp::Parameter("max_velocity", max_velocity)},
        {"/control/trajectory_interpolator/interpolation_max_acceleration_m_s2", rclcpp::Parameter("max_acceleration", max_acceleration)},
        {"/control/trajectory_interpolator/interpolation_max_jerk_m_s3", rclcpp::Parameter("max_jerk", max_jerk)},
        {"/control/trajectory_interpolator/interpolation_max_yaw_rate_rad_s", rclcpp::Parameter("max_yaw_rate", 0.75)},
        {"/control/trajectory_interpolator/interpolation_max_yaw_acceleration_rad_s2", rclcpp::Parameter("max_yaw_acceleration", 0.75)},
        {"/control/maneuver_controller/controlled_cancel_max_yaw_jerk_rad_s3", rclcpp::Parameter("max_yaw_jerk", 1.5)},
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

struct BiasedPlantResult {
    double final_error = 0.0;
    double correction = 0.0;
    double first_arrival_s = -1.0;
    double maximum_error_after_45s = 0.0;
    double maximum_error_45_to_60s = 0.0;
    double maximum_error_105_to_140s = 0.0;
    double maximum_speed = 0.0;
    double maximum_acceleration = 0.0;
    double maximum_jerk = 0.0;
    double failure_time_s = -1.0;
    std::string failure_reason;
    bool saturated = false;
};

BiasedPlantResult runBiasedPlant(
    double duration_s,
    bool correction_enabled,
    const std::function<double(double)> & target_x,
    const std::function<double(double)> & bias_x,
    const std::function<double(double)> & position_noise,
    const std::function<double(double)> & velocity_noise
) {
    constexpr double dt = 0.05;
    constexpr double start_seconds = 100.0;
    const int last_step = static_cast<int>(duration_s / dt);
    double seconds = start_seconds;
    TrajectoryInterpolator interpolator(
        makeConfiguration(0.45, 0.5, 1.0), nullptr,
        [&] { return rclcpp::Time(static_cast<int64_t>(seconds * 1.0e9), RCL_SYSTEM_TIME); });
    double position = 0.0;
    double velocity = 0.0;
    double velocity_integral = 0.0;
    BiasedPlantResult result;
    const Reference start(point_t::Zero(), 0.0);
    const std::string request_identity = "mri1-object-test-0000000000000001";
    std::unique_ptr<iii_drone::control::maneuver::ObjectTrackingSession> session;
    if (correction_enabled) {
        session = std::make_unique<iii_drone::control::maneuver::ObjectTrackingSession>(
            [&](const Reference & seed, const Reference & target, bool reset) {
                return interpolator.ComputeBoundedPositionalTrajectory(
                    seed, target, true, reset).references().front();
            },
            start, request_identity, 1,
            rclcpp::Time(static_cast<int64_t>(start_seconds * 1.0e9), RCL_SYSTEM_TIME),
            -1.0,
            iii_drone::control::maneuver::ObjectTrackingSession::Limits{});
    }

    for (int step = 0; step <= last_step; ++step) {
        const double elapsed = step * dt;
        seconds = start_seconds + elapsed;
        const double nominal_x = target_x(elapsed);
        const Reference target(point_t(nominal_x, 0.0, 0.0), 0.0);
        Reference command;
        if (session) {
            iii_drone::control::MeasuredOdometrySnapshot measured;
            const auto stamp = rclcpp::Time(
                static_cast<int64_t>(seconds * 1.0e9), RCL_SYSTEM_TIME);
            measured.state = State(
                point_t(position + position_noise(elapsed), 0.0, 0.0),
                vector_t(velocity + bias_x(elapsed) + velocity_noise(elapsed), 0.0, 0.0),
                0.0, vector_t::Zero(), stamp);
            measured.receipt_stamp = stamp;
            measured.source_sample_timestamp_us = static_cast<uint64_t>(seconds * 1.0e6);
            std::string reason;
            const bool computed = session->Compute(
                target, measured, stamp, request_identity, 1, -1.0, 0.4, command, reason);
            if (!computed && result.failure_time_s < 0.0) {
                result.failure_time_s = elapsed;
                result.failure_reason = reason;
            }
            result.saturated = result.saturated || session->saturated();
            result.correction = session->correction().x();
            if (!computed) {
                ADD_FAILURE() << "object tracking failed at " << elapsed << "s: " << reason;
                break;
            }
        } else {
            command = interpolator.ComputeBoundedPositionalTrajectory(
                start, target, true, step == 0).references().front();
        }

        const auto px4 = iii_drone::adapters::px4::TrajectorySetpointAdapter(command).ToMsg();
        EXPECT_TRUE(command.position().allFinite());
        EXPECT_TRUE(command.velocity().allFinite());
        EXPECT_TRUE(command.acceleration().allFinite());
        EXPECT_TRUE(std::isfinite(command.yaw()));
        EXPECT_TRUE(std::isfinite(command.yaw_rate()));
        EXPECT_TRUE(std::isfinite(command.yaw_acceleration()));
        EXPECT_TRUE(std::isfinite(px4.position[0]));
        EXPECT_TRUE(std::isfinite(px4.velocity[0]));
        EXPECT_TRUE(std::isfinite(px4.acceleration[0]));
        EXPECT_LE(std::abs(px4.velocity[0]), 0.45 + 1.0e-5);
        EXPECT_LE(std::abs(px4.acceleration[0]), 0.5 + 1.0e-5);

        const double segment_duration =
            (interpolator.end_time_ - interpolator.start_time_).seconds();
        for (int substep = 0; substep <= 8; ++substep) {
            const double segment_time = std::min(dt, segment_duration) * substep / 8.0;
            result.maximum_speed = std::max(result.maximum_speed,
                static_cast<double>(interpolator.velocityFunction(segment_time).norm()));
            result.maximum_acceleration = std::max(result.maximum_acceleration,
                static_cast<double>(interpolator.accelerationFunction(segment_time).norm()));
            result.maximum_jerk = std::max(result.maximum_jerk,
                static_cast<double>(interpolator.jerkFunction(segment_time).norm()));
        }

        const double measured_error = std::abs(
            position + position_noise(elapsed) - nominal_x);
        if (result.first_arrival_s < 0.0 && measured_error < 0.1) {
            result.first_arrival_s = elapsed;
        }
        if (elapsed >= 45.0) {
            result.maximum_error_after_45s = std::max(
                result.maximum_error_after_45s, measured_error);
        }
        if (elapsed >= 45.0 && elapsed <= 60.0) {
            result.maximum_error_45_to_60s = std::max(
                result.maximum_error_45_to_60s, measured_error);
        }
        if (elapsed >= 105.0 && elapsed <= 140.0) {
            result.maximum_error_105_to_140s = std::max(
                result.maximum_error_105_to_140s, measured_error);
        }

        const double measured_position = position + position_noise(elapsed);
        const double measured_velocity = velocity + bias_x(elapsed) + velocity_noise(elapsed);
        const double desired_velocity = (px4.position[0] - measured_position) + px4.velocity[0];
        const double velocity_error = desired_velocity - measured_velocity;
        velocity_integral += 2.0 * velocity_error * dt;
        const double acceleration = 4.0 * velocity_error + velocity_integral + px4.acceleration[0];
        velocity += acceleration * dt;
        position += velocity * dt;
    }
    result.final_error = std::abs(position - target_x(duration_s));
    return result;
}

}  // namespace

TEST(TrajectoryInterpolatorTest, BoundedPlannerRepresentativeTimingAndSamples) {
    const auto configuration = makeConfiguration(0.45, 0.5, 1.0);
    const rclcpp::Time fixed_now(100, 0, RCL_SYSTEM_TIME);
    struct Case {
        const char * name;
        Reference start;
        Reference target;
    };
    const point_t approach_start(1.822f, -0.579f, 2.135f);
    const std::vector<Case> cases{
        {"representative_rest",
            Reference(approach_start, 2.947, vector_t::Zero(), 0.0,
                vector_t::Zero(), 0.0, fixed_now),
            Reference(point_t(1.809f, -0.609f, 2.102f), 2.947)},
        {"moving_mixed_axes",
            Reference(approach_start, 2.947, vector_t(-0.04f, 0.05f, 0.02f), 0.04,
                vector_t(0.01f, -0.015f, 0.005f), -0.02, fixed_now),
            Reference(point_t(2.12f, -0.30f, 2.32f), 3.147)},
        {"long_target",
            Reference(approach_start, 2.947, vector_t::Zero(), 0.0,
                vector_t::Zero(), 0.0, fixed_now),
            Reference(point_t(3.0f, -1.1f, 2.5f), 2.4)},
    };
    for (const auto & scenario : cases) {
        TrajectoryInterpolator interpolator(configuration, nullptr,
            [&] { return fixed_now; });
        std::vector<double> elapsed_ms;
        iii_drone::control::ReferenceTrajectory trajectory;
        for (int run = 0; run < 9; ++run) {
            const auto begin = std::chrono::steady_clock::now();
            trajectory = interpolator.ComputeBoundedPositionalTrajectory(
                scenario.start, scenario.target, true, true);
            const auto end = std::chrono::steady_clock::now();
            if (run > 0) {
                elapsed_ms.push_back(std::chrono::duration<double, std::milli>(
                    end - begin).count());
            }
        }
        ASSERT_EQ(trajectory.references().size(), 10U);
        std::sort(elapsed_ms.begin(), elapsed_ms.end());
        std::cout << std::setprecision(17)
            << "BENCH " << scenario.name
            << " median_ms=" << (elapsed_ms[3] + elapsed_ms[4]) / 2.0
            << " max_ms=" << elapsed_ms.back()
            << " duration_s=" << (interpolator.end_time_ - interpolator.start_time_).seconds()
            << '\n';
        for (size_t i = 0; i < trajectory.references().size(); ++i) {
            const auto & reference = trajectory.references()[i];
            ASSERT_TRUE(reference.position().allFinite());
            ASSERT_TRUE(reference.velocity().allFinite());
            ASSERT_TRUE(reference.acceleration().allFinite());
            std::cout << "BENCH_SAMPLE " << scenario.name << ' ' << i << ' '
                << reference.position().transpose() << ' '
                << reference.velocity().transpose() << ' '
                << reference.acceleration().transpose() << ' '
                << reference.yaw() << ' ' << reference.yaw_rate() << ' '
                << reference.yaw_acceleration() << '\n';
        }
    }
}

TEST(TrajectoryInterpolatorTest, BoundedDerivativeHornerMatchesMatrixOracle) {
    TrajectoryInterpolator interpolator(makeConfiguration(), nullptr);
    interpolator.q <<
        1.2, -0.4, 0.8,
        0.3, -0.2, 0.15,
        -0.08, 0.04, 0.02,
        0.025, -0.01, -0.015,
        -0.003, 0.006, 0.004,
        0.001, -0.002, 0.0005;
    interpolator.q_yaw << 2.9, -0.12, 0.06, -0.018, 0.004, -0.0007;

    for (const double t : {0.0, 0.001, 0.05, 0.2, 0.75, 1.0, 2.0}) {
        const auto actual = interpolator.boundedDerivativeSample(t);
        Eigen::Matrix<double, 1, 6> velocity_basis;
        velocity_basis << 0, 1, 2*t, 3*t*t, 4*t*t*t, 5*t*t*t*t;
        Eigen::Matrix<double, 1, 6> acceleration_basis;
        acceleration_basis << 0, 0, 2, 6*t, 12*t*t, 20*t*t*t;
        Eigen::Matrix<double, 1, 6> jerk_basis;
        jerk_basis << 0, 0, 0, 6, 24*t, 60*t*t;
        const Eigen::Matrix<double, 1, 3> expected_velocity =
            velocity_basis * interpolator.q;
        const Eigen::Matrix<double, 1, 3> expected_acceleration =
            acceleration_basis * interpolator.q;
        const Eigen::Matrix<double, 1, 3> expected_jerk =
            jerk_basis * interpolator.q;
        for (int axis = 0; axis < 3; ++axis) {
            const double scale = std::max({1.0, std::abs(expected_velocity(axis)),
                std::abs(expected_acceleration(axis)), std::abs(expected_jerk(axis))});
            EXPECT_NEAR(actual.velocity(axis),
                static_cast<float>(expected_velocity(axis)), 1.0e-6 * scale);
            EXPECT_NEAR(actual.acceleration(axis),
                static_cast<float>(expected_acceleration(axis)), 1.0e-6 * scale);
            EXPECT_NEAR(actual.jerk(axis),
                static_cast<float>(expected_jerk(axis)), 1.0e-6 * scale);
        }
        EXPECT_NEAR(actual.yaw_rate,
            (velocity_basis * interpolator.q_yaw)(0), 1.0e-12);
        EXPECT_NEAR(actual.yaw_acceleration,
            (acceleration_basis * interpolator.q_yaw)(0), 1.0e-12);
        EXPECT_NEAR(actual.yaw_jerk,
            (jerk_basis * interpolator.q_yaw)(0), 1.0e-12);
    }
}

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

TEST(TrajectoryInterpolatorTest, EndpointHoldKeepsSampleTimeAdvancing) {
    TrajectoryInterpolator interpolator(makeConfiguration(), nullptr);
    const point_t target_position(1.0, 2.0, 3.0);
    const Reference target(target_position, 0.4, vector_t::Zero(), 0.0,
        vector_t::Zero(), 0.0, rclcpp::Time(1, 0));
    interpolator.ComputeReferenceTrajectory(
        Reference(point_t::Zero(), 0.0), target, true, true);
    const double duration = (interpolator.end_time_ - interpolator.start_time_).seconds();

    for (const double after_end : {0.2, 1.0, 600.0}) {
        const double sample_time = duration + after_end;
        const auto reference = interpolator.referenceFunction(sample_time);
        EXPECT_EQ(reference.stamp().nanoseconds(),
            (interpolator.start_time_ + rclcpp::Duration::from_seconds(sample_time)).nanoseconds());
        EXPECT_TRUE(reference.position().isApprox(target_position));
        EXPECT_DOUBLE_EQ(reference.yaw(), target.yaw());
        EXPECT_TRUE(reference.velocity().isZero());
        EXPECT_TRUE(reference.acceleration().isZero());
        EXPECT_DOUBLE_EQ(reference.yaw_rate(), 0.0);
    }
    EXPECT_EQ(interpolator.reference_.stamp().nanoseconds(), target.stamp().nanoseconds());
}

TEST(TrajectoryInterpolatorTest, BoundedCableTakeoffPreservesMovingStartAndBoundsJerk) {
    TrajectoryInterpolator interpolator(makeConfiguration(0.45, 0.5, 0.5), nullptr);
    const point_t start_position(1.928281, -0.583113, 3.781651);
    const vector_t start_velocity(0.104790, 0.064297, -0.071986);
    const Reference start(
        start_position, 2.70, start_velocity, 0.05,
        vector_t::Zero(), 0.02, rclcpp::Time(100, 0)
    );
    const Reference target(point_t(1.931, -0.581, 2.277), 2.746);

    const auto trajectory = interpolator.ComputeReferenceTrajectory(
        start, target, true, true, true
    );
    ASSERT_FALSE(trajectory.references().empty());
    EXPECT_TRUE(trajectory.references()[0].position().isApprox(start_position, 1.0e-9));
    EXPECT_TRUE(trajectory.references()[0].velocity().isApprox(start_velocity, 1.0e-9));
    EXPECT_TRUE(trajectory.references()[0].acceleration().isZero(1.0e-9));
    EXPECT_NEAR(trajectory.references()[0].yaw_rate(), 0.05, 1.0e-9);
    EXPECT_NEAR(trajectory.references()[0].yaw_acceleration(), 0.02, 1.0e-9);

    const double duration = (interpolator.end_time_ - interpolator.start_time_).seconds();
    ASSERT_GT(duration, 0.0);
    EXPECT_TRUE(std::isfinite(duration));
    for (int i = 0; i <= 400; ++i) {
        const double t = duration * static_cast<double>(i) / 400.0;
        EXPECT_LE(interpolator.velocityFunction(t).norm(), 0.45 + 1.0e-6);
        EXPECT_LE(interpolator.accelerationFunction(t).norm(), 0.5 + 1.0e-6);
        EXPECT_LE(interpolator.jerkFunction(t).norm(), 0.5 + 1.0e-6);
    }

    const Reference endpoint = interpolator.referenceFunction(duration);
    EXPECT_TRUE(endpoint.position().isApprox(target.position(), 1.0e-6));
    EXPECT_TRUE(endpoint.velocity().isZero(1.0e-6));
    EXPECT_TRUE(endpoint.acceleration().isZero(1.0e-6));
    EXPECT_NEAR(endpoint.yaw_rate(), 0.0, 1.0e-6);
    EXPECT_NEAR(endpoint.yaw_acceleration(), 0.0, 1.0e-6);
    EXPECT_NEAR(endpoint.stamp().seconds(), interpolator.end_time_.seconds(), 1.0e-6);
    const double derivative_step = std::min(duration * 1.0e-4, 1.0e-4);
    EXPECT_NEAR(
        (interpolator.yawRateFunction(derivative_step) - interpolator.yawRateFunction(0.0)) /
            derivative_step,
        interpolator.yawAccelerationFunction(0.0),
        1.0e-5
    );
}

TEST(TrajectoryInterpolatorTest, BoundedPositionalCandidateRejectsVelocityBiasThroughPx4Adapter) {
    for (const double bias : {0.24, -0.24}) {
        const auto constant_target = [](double) { return 1.0; };
        const auto constant_bias = [bias](double) { return bias; };
        const auto no_noise = [](double) { return 0.0; };
        const auto baseline = runBiasedPlant(
            130.0, false, constant_target, constant_bias, no_noise, no_noise);
        const auto candidate = runBiasedPlant(
            130.0, true, constant_target, constant_bias, no_noise, no_noise);
        std::cout << "biased_plant bias=" << bias
                  << " baseline_residual=" << baseline.final_error
                  << " corrected_residual=" << candidate.final_error
                  << " delta=" << candidate.correction
                  << " corrected_first_arrival_s=" << candidate.first_arrival_s
                  << " max_error_after_45s=" << candidate.maximum_error_after_45s
                  << " session_failure=" << candidate.failure_reason
                  << " max_speed=" << candidate.maximum_speed
                  << " max_acceleration=" << candidate.maximum_acceleration
                  << " max_jerk=" << candidate.maximum_jerk << '\n';
        EXPECT_GT(baseline.final_error, 0.15);
        EXPECT_LT(candidate.final_error, 0.1);
        EXPECT_GE(candidate.first_arrival_s, 0.0);
        EXPECT_LE(candidate.first_arrival_s, 45.0);
        EXPECT_LT(candidate.maximum_error_after_45s, 0.1);
        EXPECT_LT(candidate.failure_time_s, 0.0);
        EXPECT_LE(candidate.maximum_speed, 0.45 + 1.0e-5);
        EXPECT_LE(candidate.maximum_acceleration, 0.5 + 1.0e-5);
        EXPECT_LE(candidate.maximum_jerk, 1.0 + 1.0e-5);
    }
}

TEST(TrajectoryInterpolatorTest, MovingTargetAndChangingBiasedVelocitySettleWithMatchedNoise) {
    const auto target = [](double elapsed) { return 1.0 + 0.01 * elapsed; };
    const auto bias = [](double elapsed) { return elapsed < 60.0 ? 0.24 : -0.24; };
    const auto position_noise = [](double elapsed) {
        return 0.01 * std::sin(0.71 * elapsed + 0.3);
    };
    const auto velocity_noise = [](double elapsed) {
        return 0.02 * std::sin(1.13 * elapsed + 0.8);
    };
    const auto baseline = runBiasedPlant(
        140.0, false, target, bias, position_noise, velocity_noise);
    const auto corrected = runBiasedPlant(
        140.0, true, target, bias, position_noise, velocity_noise);
    std::cout << "moving_biased_plant baseline_final_error=" << baseline.final_error
              << " corrected_final_error=" << corrected.final_error
              << " corrected_error_45_60s=" << corrected.maximum_error_45_to_60s
              << " corrected_error_105_140s=" << corrected.maximum_error_105_to_140s
              << " delta=" << corrected.correction
              << " session_failure_at_s=" << corrected.failure_time_s
              << " session_failure=" << corrected.failure_reason
              << " saturated=" << corrected.saturated
              << " max_speed=" << corrected.maximum_speed
              << " max_acceleration=" << corrected.maximum_acceleration
              << " max_jerk=" << corrected.maximum_jerk << '\n';
    EXPECT_GT(baseline.final_error, 0.15);
    EXPECT_LT(corrected.maximum_error_45_to_60s, 0.1);
    EXPECT_LT(corrected.maximum_error_105_to_140s, 0.1);
    EXPECT_LT(corrected.final_error, 0.1);
    EXPECT_LT(corrected.failure_time_s, 0.0);
    EXPECT_LE(corrected.maximum_speed, 0.45 + 1.0e-5);
    EXPECT_LE(corrected.maximum_acceleration, 0.5 + 1.0e-5);
    EXPECT_LE(corrected.maximum_jerk, 1.0 + 1.0e-5);
}

TEST(TrajectoryInterpolatorTest, ObjectTrackingAuthorityShrinksOnDuplicateAndZeroAuthorityFails) {
    using Session = iii_drone::control::maneuver::ObjectTrackingSession;
    int planner_calls = 0;
    const auto stamp = [](double seconds) {
        return rclcpp::Time(static_cast<int64_t>(seconds * 1.0e9), RCL_SYSTEM_TIME);
    };
    Session session(
        [&](const Reference &, const Reference & target, bool) {
            ++planner_calls;
            return target;
        }, Reference(point_t::Zero(), 0.0), "mri1-object-test-0000000000000001",
        1, stamp(100.0), -1.0, Session::Limits{});
    iii_drone::control::MeasuredOdometrySnapshot measured;
    measured.state = State(point_t::Zero(), vector_t::Zero(), 0.0,
        vector_t::Zero(), stamp(100.0));
    measured.receipt_stamp = stamp(100.0);
    measured.source_sample_timestamp_us = 100000000;
    Reference output;
    std::string reason;
    const Reference nominal(point_t(1.0f, 0.0f, 0.0f), 0.0);
    ASSERT_TRUE(session.Compute(nominal, measured, stamp(100.0),
        "mri1-object-test-0000000000000001", 1, -1.0, 0.4,
        output, reason)) << reason;
    measured.receipt_stamp = stamp(100.05);
    measured.source_sample_timestamp_us += 50000;
    ASSERT_TRUE(session.Compute(nominal, measured, stamp(100.05),
        "mri1-object-test-0000000000000001", 1, -1.0, 0.4,
        output, reason)) << reason;
    EXPECT_GT(session.correction().x(), 0.0f);
    const int before_duplicate = planner_calls;
    ASSERT_TRUE(session.Compute(nominal, measured, stamp(100.1),
        "mri1-object-test-0000000000000001", 1, -1.0, 0.0,
        output, reason)) << reason;
    EXPECT_EQ(planner_calls, before_duplicate + 1);
    EXPECT_NEAR(session.correction().norm(), 0.0, 1.0e-7);
    EXPECT_TRUE(session.saturated());
    for (int i = 3; i <= 62 && !session.failed(); ++i) {
        measured.receipt_stamp = stamp(100.0 + i * 0.05);
        measured.source_sample_timestamp_us = 100000000 + i * 50000;
        (void)session.Compute(nominal, measured, measured.receipt_stamp,
            "mri1-object-test-0000000000000001", 1, -1.0, 0.0,
            output, reason);
    }
    EXPECT_TRUE(session.failed());
    EXPECT_NE(session.failureReason().find("exhausted"), std::string::npos);
}

TEST(TrajectoryInterpolatorTest, AdvancingPx4SamplesWithEqualRosReceiptRemainValidForTracking) {
    using Session = iii_drone::control::maneuver::ObjectTrackingSession;
    using iii_drone::control::CombinedDroneAwarenessHandler;
    using iii_drone::adapters::px4::VehicleOdometryAdapter;
    const auto stamp = [](double seconds) {
        return rclcpp::Time(static_cast<int64_t>(seconds * 1.0e9), RCL_SYSTEM_TIME);
    };
    const std::string owner = "mri1-object-sample-time-0000000000000001";
    Session session(
        [](const Reference &, const Reference & target, bool) { return target; },
        Reference(point_t::Zero(), 0.0), owner, 1, stamp(100.0), -1.0,
        Session::Limits{});
    px4_msgs::msg::VehicleOdometry odometry;
    odometry.pose_frame = iii_drone::adapters::px4::POSE_FRAME_LOCAL_NED;
    odometry.velocity_frame = iii_drone::adapters::px4::VELOCITY_FRAME_LOCAL_NED;
    odometry.q[0] = 1.0F;
    odometry.reset_counter = 7;
    odometry.timestamp = 100000000;
    odometry.timestamp_sample = 100000000;
    const auto first = CombinedDroneAwarenessHandler::AdvanceMeasuredOdometry(
        std::nullopt, VehicleOdometryAdapter(odometry),
        odometry.timestamp_sample, stamp(100.0));
    ASSERT_TRUE(first);
    const Reference nominal(point_t(1.0F, 0.0F, 0.0F), 0.0);
    Reference output;
    std::string reason;
    ASSERT_TRUE(session.Compute(nominal, *first, stamp(100.0), owner, 1,
        -1.0, 0.4, output, reason)) << reason;

    // Separate PX4 samples may be delivered during the same ROS clock tick.
    // Receipt time still proves freshness against the emission clock; source
    // sample time identifies and times the new measured observation.
    odometry.timestamp += 50000;
    odometry.timestamp_sample += 50000;
    odometry.position[0] = 0.01F;
    const auto second = CombinedDroneAwarenessHandler::AdvanceMeasuredOdometry(
        first, VehicleOdometryAdapter(odometry),
        odometry.timestamp_sample, stamp(100.0));
    ASSERT_TRUE(second);
    ASSERT_GT(second->source_sample_timestamp_us, first->source_sample_timestamp_us);
    const double position_delta_m =
        (second->state.position() - first->state.position()).norm();
    ASSERT_GT(position_delta_m, 0.009);
    ASSERT_EQ(second->receipt_stamp.nanoseconds(), first->receipt_stamp.nanoseconds());
    ASSERT_EQ(second->reset_counter, first->reset_counter);
    RecordProperty("first_source_sample_us", first->source_sample_timestamp_us);
    RecordProperty("second_source_sample_us", second->source_sample_timestamp_us);
    RecordProperty("first_receipt_ros_ns", first->receipt_stamp.nanoseconds());
    RecordProperty("second_receipt_ros_ns", second->receipt_stamp.nanoseconds());
    RecordProperty("second_emission_ros_ns", stamp(100.05).nanoseconds());
    RecordProperty("measured_position_delta_m", position_delta_m);
    RecordProperty("reset_counter", static_cast<unsigned>(second->reset_counter));
    const bool accepted = session.Compute(nominal, *second, stamp(100.05),
        owner, 1, -1.0, 0.4, output, reason);
    RecordProperty("tracking_reason", reason);
    EXPECT_TRUE(accepted) << reason;
    EXPECT_FALSE(session.failed());
    EXPECT_TRUE(output.position().allFinite());
    EXPECT_TRUE(output.velocity().allFinite());
    EXPECT_TRUE(output.acceleration().allFinite());
}

TEST(TrajectoryInterpolatorTest, ObjectTrackingCorrectionUsesSourceIntervalDespiteReceiptJitter) {
    using Session = iii_drone::control::maneuver::ObjectTrackingSession;
    const auto stamp = [](double seconds) {
        return rclcpp::Time(static_cast<int64_t>(seconds * 1.0e9), RCL_SYSTEM_TIME);
    };
    const std::string owner = "mri1-object-sample-jitter-0000000000000001";
    const Reference target(point_t(1.0F, 0.0F, 0.0F), 0.0);
    const auto make_session = [&] {
        return Session(
            [](const Reference &, const Reference & corrected, bool) { return corrected; },
            Reference(point_t::Zero(), 0.0), owner, 1, stamp(100.0), -1.0,
            Session::Limits{});
    };
    auto equal_receipts = make_session();
    auto jittered_receipts = make_session();
    iii_drone::control::MeasuredOdometrySnapshot measured;
    measured.state = State(point_t::Zero(), vector_t::Zero(), 0.0,
        vector_t::Zero(), stamp(100.0));
    measured.receipt_stamp = stamp(100.0);
    measured.source_sample_timestamp_us = 100000000;
    measured.reset_counter = 7;
    Reference output;
    std::string reason;
    ASSERT_TRUE(equal_receipts.Compute(target, measured, stamp(100.0), owner, 1,
        -1.0, 0.4, output, reason)) << reason;
    ASSERT_TRUE(jittered_receipts.Compute(target, measured, stamp(100.0), owner, 1,
        -1.0, 0.4, output, reason)) << reason;

    measured.source_sample_timestamp_us += 50000;
    measured.state = State(point_t(0.01F, 0.0F, 0.0F), vector_t::Zero(), 0.0,
        vector_t::Zero(), stamp(100.05));
    ASSERT_TRUE(equal_receipts.Compute(target, measured, stamp(100.05), owner, 1,
        -1.0, 0.4, output, reason)) << reason;
    const auto equal_correction = equal_receipts.correction();
    measured.receipt_stamp = stamp(100.04);
    ASSERT_TRUE(jittered_receipts.Compute(target, measured, stamp(100.05), owner, 1,
        -1.0, 0.4, output, reason)) << reason;
    EXPECT_GT(equal_correction.norm(), 0.0F);
    EXPECT_TRUE(jittered_receipts.correction().isApprox(equal_correction, 1.0e-7F));

    // A duplicate source sample with a fresh receipt is not integrated twice.
    measured.receipt_stamp = stamp(100.1);
    ASSERT_TRUE(jittered_receipts.Compute(target, measured, stamp(100.1), owner, 1,
        -1.0, 0.4, output, reason)) << reason;
    EXPECT_TRUE(jittered_receipts.correction().isApprox(equal_correction, 1.0e-7F));
}

TEST(TrajectoryInterpolatorTest, ObjectTrackingRejectsOversizedSourceGapDespiteFreshReceipt) {
    using Session = iii_drone::control::maneuver::ObjectTrackingSession;
    const auto stamp = [](double seconds) {
        return rclcpp::Time(static_cast<int64_t>(seconds * 1.0e9), RCL_SYSTEM_TIME);
    };
    const std::string owner = "mri1-object-source-gap-0000000000000001";
    Session session(
        [](const Reference &, const Reference & target, bool) { return target; },
        Reference(point_t::Zero(), 0.0), owner, 1, stamp(100.0), -1.0,
        Session::Limits{});
    iii_drone::control::MeasuredOdometrySnapshot measured;
    measured.state = State(point_t::Zero(), vector_t::Zero(), 0.0,
        vector_t::Zero(), stamp(100.0));
    measured.receipt_stamp = stamp(100.0);
    measured.source_sample_timestamp_us = 100000000;
    measured.reset_counter = 7;
    const Reference target(point_t(1.0F, 0.0F, 0.0F), 0.0);
    Reference output;
    std::string reason;
    ASSERT_TRUE(session.Compute(target, measured, stamp(100.0), owner, 1,
        -1.0, 0.4, output, reason)) << reason;
    measured.source_sample_timestamp_us += 1300000;
    measured.receipt_stamp = stamp(100.05);
    EXPECT_FALSE(session.Compute(target, measured, stamp(100.05), owner, 1,
        -1.0, 0.4, output, reason));
    EXPECT_TRUE(session.failed());
    EXPECT_NE(reason.find("sample interval is discontinuous"), std::string::npos);
    EXPECT_NE(reason.find("source_interval_s=1.300000"), std::string::npos);
    EXPECT_NE(reason.find("receipt_interval_s=0.050000"), std::string::npos);
    EXPECT_NE(reason.find("source_sample_us=101300000"), std::string::npos);
    EXPECT_NE(reason.find("prior_source_sample_us=100000000"), std::string::npos);
    EXPECT_NE(reason.find("request_identity=" + owner), std::string::npos);
    EXPECT_NE(reason.find("execution_id=1"), std::string::npos);
    EXPECT_TRUE(output.position().allFinite());
    EXPECT_TRUE(output.velocity().allFinite());
    EXPECT_TRUE(output.acceleration().allFinite());
}

// HIL: PX4 odometry resumed after a 0.304 s source gap and object hover
// tracking failed. An ended gap within maximum_sample_gap_s is ridden through
// without integrating the correction across it.
TEST(TrajectoryInterpolatorTest, ObjectTrackingRidesThroughEndedSourceGapWithoutIntegrating) {
    using Session = iii_drone::control::maneuver::ObjectTrackingSession;
    const auto stamp = [](double seconds) {
        return rclcpp::Time(static_cast<int64_t>(seconds * 1.0e9), RCL_SYSTEM_TIME);
    };
    const std::string owner = "mri1-object-source-gap-0000000000000002";
    Session session(
        [](const Reference &, const Reference & target, bool) { return target; },
        Reference(point_t::Zero(), 0.0), owner, 1, stamp(100.0), -1.0,
        Session::Limits{});
    iii_drone::control::MeasuredOdometrySnapshot measured;
    measured.state = State(point_t::Zero(), vector_t::Zero(), 0.0,
        vector_t::Zero(), stamp(100.0));
    measured.receipt_stamp = stamp(100.0);
    measured.source_sample_timestamp_us = 100000000;
    measured.reset_counter = 7;
    const Reference target(point_t(1.0F, 0.0F, 0.0F), 0.0);
    Reference output;
    std::string reason;
    ASSERT_TRUE(session.Compute(target, measured, stamp(100.0), owner, 1,
        -1.0, 0.4, output, reason)) << reason;
    const auto before_gap = session.correction();

    // Commands keep being emitted during the gap with the last sample.
    ASSERT_TRUE(session.Compute(target, measured, stamp(100.2), owner, 1,
        -1.0, 0.4, output, reason)) << reason;
    measured.source_sample_timestamp_us += 304000;
    measured.receipt_stamp = stamp(100.296);
    ASSERT_TRUE(session.Compute(target, measured, stamp(100.296), owner, 1,
        -1.0, 0.4, output, reason)) << reason;
    EXPECT_FALSE(session.failed());
    EXPECT_TRUE(session.correction().isApprox(before_gap, 1.0e-7F));

    // Regular samples integrate again.
    measured.source_sample_timestamp_us += 10000;
    measured.receipt_stamp = stamp(100.306);
    ASSERT_TRUE(session.Compute(target, measured, stamp(100.306), owner, 1,
        -1.0, 0.4, output, reason)) << reason;
    EXPECT_FALSE(session.correction().isApprox(before_gap, 1.0e-7F));
}

TEST(TrajectoryInterpolatorTest, ObjectTrackingRejectsWrongOwnerAndOwnsResetOrStaleFailureStops) {
    using Session = iii_drone::control::maneuver::ObjectTrackingSession;
    const auto stamp = [](double seconds) {
        return rclcpp::Time(static_cast<int64_t>(seconds * 1.0e9), RCL_SYSTEM_TIME);
    };
    const std::string owner = "mri1-object-test-0000000000000001";
    const auto make_session = [&](double started_at) {
        return Session(
            [](const Reference &, const Reference & target, bool) { return target; },
            Reference(point_t::Zero(), 0.0), owner, 1, stamp(started_at), -1.0,
            Session::Limits{});
    };
    const Reference target(point_t(1.0, 0.0, 0.0), 0.0);

    auto reset_session = make_session(100.0);
    iii_drone::control::MeasuredOdometrySnapshot measured;
    measured.state = State(point_t::Zero(), vector_t::Zero(), 0.0,
        vector_t::Zero(), stamp(100.0));
    measured.receipt_stamp = stamp(100.0);
    measured.source_sample_timestamp_us = 100000000;
    Reference output;
    std::string reason;
    ASSERT_TRUE(reset_session.Compute(target, measured, stamp(100.0), owner, 1,
        -1.0, 0.4, output, reason)) << reason;
    measured.state = State(point_t(-0.2, 0.0, 0.0), vector_t::Zero(), 0.0,
        vector_t::Zero(), stamp(100.05));
    measured.receipt_stamp = stamp(100.05);
    measured.source_sample_timestamp_us += 50000;
    ASSERT_TRUE(reset_session.Compute(target, measured, stamp(100.05), owner, 1,
        -1.0, 0.4, output, reason)) << reason;
    const vector_t correction_after_fresh_sample = reset_session.correction();

    // A fresh receipt with a duplicate source timestamp must not advance the estimator.
    measured.receipt_stamp = stamp(100.1);
    ASSERT_TRUE(reset_session.Compute(target, measured, stamp(100.1), owner, 1,
        -1.0, 0.4, output, reason)) << reason;
    EXPECT_TRUE(reset_session.correction().isApprox(correction_after_fresh_sample, 1.0e-9));

    const Reference owned_command = reset_session.lastCommand();
    measured.receipt_stamp = stamp(100.15);
    measured.source_sample_timestamp_us += 50000;
    const bool wrong_owner_computed = reset_session.Compute(target, measured, stamp(100.15),
        "mri1-object-test-0000000000000002", 1, -1.0, 0.4, output, reason);
    EXPECT_FALSE(wrong_owner_computed);
    EXPECT_NE(reason.find("does not own"), std::string::npos);
    EXPECT_FALSE(reset_session.failed());
    EXPECT_TRUE(reset_session.owns(owner, 1));
    EXPECT_TRUE(output.position().isApprox(owned_command.position(), 1.0e-9));
    EXPECT_FALSE(reset_session.Compute(target, measured, stamp(100.15),
        owner, 2, -1.0, 0.4, output, reason));
    EXPECT_FALSE(reset_session.failed());
    EXPECT_TRUE(reset_session.owns(owner, 1));
    EXPECT_TRUE(reset_session.correction().isApprox(correction_after_fresh_sample, 1.0e-9));

    // The legitimate owner continues, but a changed odometry reset counter fails closed.
    ASSERT_TRUE(reset_session.Compute(target, measured, stamp(100.15), owner, 1,
        -1.0, 0.4, output, reason)) << reason;
    measured.receipt_stamp = stamp(100.2);
    measured.state = State(point_t(-0.2, 0.0, 0.0), vector_t::Zero(), 0.0,
        vector_t::Zero(), stamp(100.2));
    measured.source_sample_timestamp_us += 50000;
    measured.reset_counter = 1;
    EXPECT_FALSE(reset_session.Compute(target, measured, stamp(100.2), owner, 1,
        -1.0, 0.4, output, reason));
    EXPECT_TRUE(reset_session.failed());
    EXPECT_NE(reset_session.failureReason().find("reset"), std::string::npos);
    EXPECT_TRUE(output.position().allFinite());
    EXPECT_TRUE(output.velocity().allFinite());
    EXPECT_TRUE(output.acceleration().allFinite());
    EXPECT_FALSE(reset_session.Compute(target, measured, stamp(100.25), owner, 1,
        -1.0, 0.4, output, reason));
    EXPECT_TRUE(output.position().allFinite());

    auto stale_session = make_session(200.0);
    measured = iii_drone::control::MeasuredOdometrySnapshot{};
    measured.state = State(point_t::Zero(), vector_t::Zero(), 0.0,
        vector_t::Zero(), stamp(200.0));
    measured.receipt_stamp = stamp(200.0);
    measured.source_sample_timestamp_us = 200000000;
    ASSERT_TRUE(stale_session.Compute(target, measured, stamp(200.0), owner, 1,
        -1.0, 0.4, output, reason)) << reason;
    EXPECT_FALSE(stale_session.Compute(target, measured, stamp(200.3), owner, 1,
        -1.0, 0.4, output, reason));
    EXPECT_TRUE(stale_session.failed());
    EXPECT_NE(stale_session.failureReason().find("stale"), std::string::npos);
    EXPECT_TRUE(output.position().allFinite());
    EXPECT_TRUE(output.velocity().allFinite());
    EXPECT_TRUE(output.acceleration().allFinite());
}

TEST(TrajectoryInterpolatorTest, ObjectTrackingReportsOnlyOriginatingFreshnessFault) {
    using Session = iii_drone::control::maneuver::ObjectTrackingSession;
    const auto stamp = [](double seconds) {
        return rclcpp::Time(static_cast<int64_t>(seconds * 1.0e9), RCL_SYSTEM_TIME);
    };
    const std::string owner = "mri1-object-freshness-0000000000000001";
    const Reference seed(point_t::Zero(), 0.0, vector_t::Zero(), 0.0,
        vector_t::Zero(), 0.0, stamp(100.0));
    const Reference target(point_t(0.1F, 0.0F, 0.0F), 0.0);
    const auto make_session = [&](double start_s) {
        return Session(
            [](const Reference &, const Reference & nominal, bool) { return nominal; },
            seed.CopyWithNewStamp(stamp(start_s)), owner, 1, stamp(start_s),
            -1.0, Session::Limits{});
    };
    iii_drone::control::MeasuredOdometrySnapshot measured;
    measured.state = State(point_t::Zero(), vector_t::Zero(), 0.0,
        vector_t::Zero(), stamp(100.0));
    measured.receipt_stamp = stamp(100.0);
    measured.source_sample_timestamp_us = 100'000'000;
    Reference output;
    std::string reason;
    bool first_fault = true;
    auto session = make_session(100.0);
    ASSERT_TRUE(session.Compute(target, measured, stamp(100.0), owner, 1,
        -1.0, 0.4, output, reason, &first_fault)) << reason;
    EXPECT_FALSE(first_fault);

    first_fault = true;
    EXPECT_FALSE(session.Compute(target, measured, stamp(100.3), "wrong-owner", 1,
        -1.0, 0.4, output, reason, &first_fault));
    EXPECT_FALSE(first_fault);
    EXPECT_FALSE(session.failed());

    EXPECT_FALSE(session.Compute(target, measured, stamp(100.3), owner, 1,
        -1.0, 0.4, output, reason, &first_fault));
    EXPECT_TRUE(first_fault);
    EXPECT_TRUE(session.failed());
    first_fault = true;
    EXPECT_FALSE(session.Compute(target, measured, stamp(100.35), owner, 1,
        -1.0, 0.4, output, reason, &first_fault));
    EXPECT_FALSE(first_fault);

    auto transition = make_session(200.0);
    measured.state = State(point_t::Zero(), vector_t::Zero(), 0.0,
        vector_t::Zero(), stamp(200.0));
    measured.receipt_stamp = stamp(200.0);
    measured.source_sample_timestamp_us = 200'000'000;
    ASSERT_TRUE(transition.Compute(target, measured, stamp(200.0), owner, 1,
        -1.0, 0.4, output, reason, &first_fault)) << reason;
    ASSERT_TRUE(transition.RequestTransitionStop(owner, 1));
    first_fault = true;
    (void)transition.Compute(target, measured, stamp(200.3), owner, 1,
        -1.0, 0.4, output, reason, &first_fault);
    EXPECT_FALSE(first_fault);
}

TEST(TrajectoryInterpolatorTest, ObjectTrackingInitialStopChecksNonzeroAccelerationAndRejectsInfeasibleSeed) {
    using Session = iii_drone::control::maneuver::ObjectTrackingSession;
    Session::Limits limits;
    std::string reason;
    const auto stamp = rclcpp::Time(100000000000LL, RCL_SYSTEM_TIME);
    const Reference accelerating_seed(
        point_t(0.0, 0.0, 1.0), 0.0, vector_t(0.0, 0.0, 0.05), 0.0,
        vector_t(0.0, 0.0, 0.1), 0.0, stamp);
    EXPECT_TRUE(Session::CanCertifyInitialSeed(
        accelerating_seed, 0.0, limits.cancellation_config, reason)) << reason;

    const Reference infeasible_seed(
        point_t(0.0, 0.0, 0.01), 0.0, vector_t(0.0, 0.0, -0.4), 0.0,
        vector_t(0.0, 0.0, -0.2), 0.0, stamp);
    EXPECT_FALSE(Session::CanCertifyInitialSeed(
        infeasible_seed, 0.0, limits.cancellation_config, reason));
    EXPECT_FALSE(reason.empty());
}

TEST(TrajectoryInterpolatorTest, BoundedCableTakeoffFlowsThroughClientAndGuardAtFlightRates) {
    RclcppContext context;
    auto service_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
        "trajectory_generator", "/control"
    );
    auto client_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
        "bounded_takeoff_client", "/test"
    );
    TrajectoryInterpolator interpolator(makeConfiguration(0.45, 0.5, 0.5), service_node.get());
    std::mutex request_mutex;
    int request_count = 0;
    int reset_count = 0;
    int set_target_count = 0;
    point_t requested_target = point_t::Zero();
    auto service = service_node->create_service<
        iii_drone_interfaces::srv::ComputeReferenceTrajectory
    >(
        "/control/trajectory_generator/compute_reference_trajectory",
        [&interpolator, &request_mutex, &request_count, &reset_count, &set_target_count,
            &requested_target](
            const std::shared_ptr<iii_drone_interfaces::srv::ComputeReferenceTrajectory::Request> request,
            std::shared_ptr<iii_drone_interfaces::srv::ComputeReferenceTrajectory::Response> response
        ) {
            iii_drone::adapters::ReferenceAdapter start_adapter(request->start_reference);
            iii_drone::adapters::ReferenceAdapter target_adapter(request->reference);
            const auto mode = static_cast<iii_drone::control::trajectory_mode_t>(
                request->trajectory_mode.mode
            );
            auto trajectory = interpolator.ComputeReferenceTrajectory(
                start_adapter.reference(), target_adapter.reference(),
                request->set_reference, request->reset,
                mode == iii_drone::control::trajectory_mode_t::cable_takeoff
            );
            response->success = true;
            response->reference_trajectory =
                iii_drone::adapters::ReferenceTrajectoryAdapter(trajectory).ToMsg();
            std::lock_guard<std::mutex> lock(request_mutex);
            ++request_count;
            reset_count += request->reset ? 1 : 0;
            set_target_count += request->set_reference ? 1 : 0;
            requested_target = target_adapter.reference().position();
        }
    );

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(service_node->get_node_base_interface());
    executor.add_node(client_node->get_node_base_interface());
    std::thread spin_thread([&executor]() { executor.spin(); });
    ExecutorThreadGuard spinner(executor, spin_thread);

    const std::unordered_map<std::string, rclcpp::Parameter> client_values{
        {"/control/maneuver_controller/generate_trajectories_asynchronously_with_delay",
            rclcpp::Parameter("async", false)},
        {"/control/maneuver_controller/generate_trajectories_poll_period_ms",
            rclcpp::Parameter("poll_ms", 1)},
        {"/control/maneuver_controller/generate_trajectories_timeout_ms",
            rclcpp::Parameter("timeout_ms", 2000)},
        {"/control/dt", rclcpp::Parameter("dt", 0.2)},
    };
    std::vector<configuration_entry_t> client_entries;
    for (const auto & [name, value] : client_values) {
        client_entries.emplace_back(name, value.get_type());
    }
    auto client_configuration = std::make_shared<Configuration>(
        "trajectory_client_test", std::move(client_entries),
        [client_values](const std::string & name) { return client_values.at(name); }
    );
    auto client = std::make_shared<TrajectoryGeneratorClient>(
        client_node.get(), client_configuration,
        client_node->create_callback_group(rclcpp::CallbackGroupType::Reentrant)
    );

    const point_t start_position(1.928281, -0.583113, 3.781651);
    const vector_t start_velocity(0.104790, 0.064297, -0.071986);
    const Reference start(
        start_position, 2.746, start_velocity, 0.0,
        vector_t::Zero(), 0.0, rclcpp::Clock().now()
    );
    const Reference target(point_t(1.931, -0.581, 2.277), 2.746);
    ManeuverReferenceSafetyConfig guard_config;
    guard_config.max_jerk_m_s3 = 0.5;
    guard_config.max_yaw_jerk_rad_s3 = 1.5;
    ManeuverReferenceSafetyGuard guard(guard_config);
    const auto guard_start = ManeuverReferenceSafetyGuard::Clock::now();
    EXPECT_EQ(
        guard.observeReference(start.CopyWithNans(), guard_start).decision,
        ManeuverReferenceSafetyDecision::ACCEPT
    );

    std::vector<Reference> samples;
    samples.reserve(180);
    std::string guard_failure;
    const auto wall_start = std::chrono::steady_clock::now();
    const auto finish_at = wall_start + std::chrono::seconds(8);
    auto next_tick = wall_start;
    while (std::chrono::steady_clock::now() < finish_at) {
        const auto sample = client->ComputeReference(
            start, target, samples.empty(), samples.empty(),
            iii_drone::control::trajectory_mode_t::cable_takeoff
        );
        samples.push_back(sample);
        if (samples.size() % 4 == 0) {
            const auto received = ManeuverReferenceSafetyGuard::Clock::now();
            const auto evaluation = guard.observeReference(sample, received);
            if (evaluation.decision != ManeuverReferenceSafetyDecision::ACCEPT) {
                guard_failure = evaluation.reason;
                break;
            }
        }
        next_tick += std::chrono::milliseconds(50);
        std::this_thread::sleep_until(next_tick);
    }

    const Reference stopped_start(
        target.position(), target.yaw(), vector_t::Zero(), 0.0,
        vector_t::Zero(), 0.0, rclcpp::Clock().now()
    );
    const Reference rebase_start = client->ComputeReference(
        stopped_start, target, true, true,
        iii_drone::control::trajectory_mode_t::cable_takeoff
    );
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    const Reference rebase_endpoint = client->ComputeReference(
        stopped_start, target, false, false,
        iii_drone::control::trajectory_mode_t::cable_takeoff
    );

    {
        std::lock_guard<std::mutex> lock(request_mutex);
        EXPECT_EQ(request_count, static_cast<int>(samples.size()) + 2);
        EXPECT_EQ(reset_count, 2);
        EXPECT_EQ(set_target_count, 2);
        EXPECT_TRUE(requested_target.isApprox(target.position(), 1.0e-9));
    }

    client.reset();
    spinner.stop();
    (void)service;
    EXPECT_TRUE(guard_failure.empty()) << guard_failure;
    ASSERT_GT(samples.size(), 100U);
    EXPECT_FALSE(guard.faultLatched());
    EXPECT_TRUE(samples.back().position().isApprox(target.position(), 1.0e-4));
    EXPECT_TRUE(samples.back().velocity().isZero(1.0e-6));
    EXPECT_TRUE(samples.back().acceleration().isZero(1.0e-6));
    EXPECT_TRUE(rebase_start.position().isApprox(target.position(), 1.0e-6));
    EXPECT_TRUE(rebase_endpoint.position().isApprox(target.position(), 1.0e-6));
    EXPECT_TRUE(rebase_endpoint.velocity().isZero(1.0e-6));
    EXPECT_TRUE(rebase_endpoint.acceleration().isZero(1.0e-6));
}

TEST(TrajectoryInterpolatorTest, BoundedCableTakeoffRejectsNonfiniteYawWithoutNormalizationLoop) {
    TrajectoryInterpolator interpolator(makeConfiguration(0.45, 0.5, 0.5), nullptr);
    const Reference target(point_t(0.0, 0.0, 1.0), 0.0);
    const Reference infinite_yaw(
        point_t::Zero(), std::numeric_limits<double>::infinity()
    );
    const Reference nan_yaw(
        point_t::Zero(), std::numeric_limits<double>::quiet_NaN()
    );

    EXPECT_THROW(interpolator.computeInterpolation(infinite_yaw, target, true), std::runtime_error);
    EXPECT_THROW(interpolator.computeInterpolation(nan_yaw, target, true), std::runtime_error);
}

TEST(TrajectoryInterpolatorTest, ProductionConfigurationViewDeclaresJerkLimits) {
    RclcppContext context;
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("/control/dt", 0.2),
        rclcpp::Parameter("/control/trajectory_interpolator/interpolation_max_jerk_m_s3", 0.5),
        rclcpp::Parameter("/control/maneuver_controller/controlled_cancel_max_yaw_jerk_rad_s3", 1.5),
    });
    rclcpp_lifecycle::LifecycleNode node("trajectory_interpolator_configuration_test", "/", options);
    iii_drone::control::trajectory_generator_node::detail::LifecycleConfigurator configurator(
        &node, "trajectory_generator"
    );
    iii_drone::control::trajectory_generator_node::detail::ConfigureTrajectoryInterpolator(
        configurator
    );

    const auto configuration = configurator.GetConfiguration("trajectory_interpolator");
    for (const std::string & name : {
        "/control/trajectory_interpolator/interpolation_max_jerk_m_s3",
        "/control/maneuver_controller/controlled_cancel_max_yaw_jerk_rad_s3",
    }) {
        EXPECT_TRUE(configuration->HasParameter(name));
        ASSERT_TRUE(node.has_parameter(name));
    }
    EXPECT_DOUBLE_EQ(
        configuration->GetParameter("/control/trajectory_interpolator/interpolation_max_jerk_m_s3").as_double(),
        0.5
    );
    EXPECT_DOUBLE_EQ(
        configuration->GetParameter("/control/maneuver_controller/controlled_cancel_max_yaw_jerk_rad_s3").as_double(),
        1.5
    );
}

TEST(TrajectoryInterpolatorTest, ShortPositionalSegmentHonoursJerkLimit) {
    // HIL soak finding: a 13 mm FlyToPosition (positional, non-bounded mode)
    // was timed by its acceleration limit only and swung its acceleration by
    // ~0.9 m/s^2 within 0.2 s, tripping the consumer's continuity envelope.
    TrajectoryInterpolator interpolator(makeConfiguration(0.45, 0.5, 1.0), nullptr);
    const Reference start(
        point_t(5.738, 6.981, 4.132), -1.817, vector_t::Zero(), 0.0,
        vector_t::Zero(), 0.0, rclcpp::Time(100, 0)
    );
    const Reference target(point_t(5.737, 6.994, 4.138), -1.817);

    interpolator.ComputeReferenceTrajectory(start, target, true, true, false);
    const double duration = (interpolator.end_time_ - interpolator.start_time_).seconds();
    ASSERT_GT(duration, 0.0);
    const double distance = (target.position() - start.position()).norm();
    EXPECT_GE(duration, std::cbrt(60.0 * distance / 1.0) - 1.0e-9);
    for (int i = 0; i <= 400; ++i) {
        const double t = duration * static_cast<double>(i) / 400.0;
        EXPECT_LE(interpolator.jerkFunction(t).norm(), 1.0 + 1.0e-6);
        EXPECT_LE(interpolator.accelerationFunction(t).norm(), 0.5 + 1.0e-6);
    }
    // A 0.2 s consumer sample interval can never see more than jerk * dt of
    // acceleration change, inside the 0.75 + 1.0 * 0.2 envelope.
    for (double t = 0.0; t + 0.2 <= duration; t += 0.01) {
        EXPECT_LE((interpolator.accelerationFunction(t + 0.2) -
                   interpolator.accelerationFunction(t)).norm(), 0.2 + 1.0e-6);
    }
}

TEST(TrajectoryInterpolatorTest, ShortYawOnlySegmentHonoursYawJerkLimit) {
    TrajectoryInterpolator interpolator(makeConfiguration(0.45, 0.5, 1.0), nullptr);
    const Reference start(
        point_t(0.0, 0.0, 2.0), 0.0, vector_t::Zero(), 0.0,
        vector_t::Zero(), 0.0, rclcpp::Time(100, 0)
    );
    const Reference target(point_t(0.0, 0.0, 2.0), 0.02);

    interpolator.ComputeReferenceTrajectory(start, target, true, true, false);
    const double duration = (interpolator.end_time_ - interpolator.start_time_).seconds();
    EXPECT_GE(duration, std::cbrt(60.0 * 0.02 / 1.5) - 1.0e-9);
}
