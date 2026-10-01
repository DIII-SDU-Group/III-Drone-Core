#include <gtest/gtest.h>

#include <cmath>
#include <chrono>
#include <string>

#include <iii_drone_core/adapters/px4/trajectory_setpoint_adapter.hpp>
#include <iii_drone_core/control/maneuver/maneuver_reference_safety_guard.hpp>
#include <iii_drone_core/control/kinematic_stop_trajectory.hpp>
#include <iii_drone_core/control/terminal_position_tracking_controller.hpp>

using iii_drone::control::Reference;
using iii_drone::control::State;
using iii_drone::control::TerminalPositionTrackingController;
using iii_drone::control::maneuver::ManeuverReferenceSafetyDecision;
using iii_drone::control::maneuver::ManeuverReferenceSafetyGuard;
using iii_drone::control::maneuver::ManeuverReferenceSafetyConfig;
using iii_drone::adapters::px4::TrajectorySetpointAdapter;
using iii_drone::types::point_t;
using iii_drone::types::vector_t;

namespace {

rclcpp::Time rosTime(double seconds) {
    return rclcpp::Time(static_cast<int64_t>(seconds * 1.0e9), RCL_ROS_TIME);
}

struct Plant {
    double position = 0.0;
    double velocity = 0.0;
    double velocity_integral = 0.0;
};

double oneStep(Plant & plant, double position_sp, double velocity_ff, double acceleration_ff, double dt, double velocity_bias) {
    // Reduced PX4-like position-P / velocity-PI cascade. The estimator bias is
    // injected only into measured velocity; position feedback remains measured.
    constexpr double position_p = 1.0;
    constexpr double velocity_p = 4.0;
    constexpr double velocity_i = 2.0;
    const double velocity_sp = position_p * (position_sp - plant.position) + velocity_ff;
    const double error = velocity_sp - (plant.velocity + velocity_bias);
    plant.velocity_integral += velocity_i * error * dt;
    const double acceleration = velocity_p * error + plant.velocity_integral + acceleration_ff;
    plant.velocity += acceleration * dt;
    plant.position += plant.velocity * dt;
    return acceleration;
}

}  // namespace

TEST(BoundedTerminalPositionTrackingTest, RejectsStaleResetAndInvalidInputsWithoutChangingLastCommand) {
    TerminalPositionTrackingController controller(
        Reference(point_t(1.5, 0.0, 0.0), 0.0)
    );
    Reference output;
    std::string reason;
    State state(point_t::Zero(), vector_t::Zero(), 0.0, vector_t::Zero(), rosTime(1.0));
    ASSERT_TRUE(controller.Update(state, rosTime(1.0), 3, rosTime(1.0), 0.35, output, reason)) << reason;
    const Reference accepted = output;

    state = State(point_t(0.1, 0.0, 0.0), vector_t::Zero(), 0.0, vector_t::Zero(), rosTime(1.05));
    EXPECT_FALSE(controller.Update(state, rosTime(1.05), 3, rosTime(1.40), 0.35, output, reason));
    EXPECT_EQ(output.position(), accepted.position());

    EXPECT_FALSE(controller.Update(state, rosTime(1.05), 4, rosTime(1.05), 0.35, output, reason));
    EXPECT_EQ(output.position(), accepted.position());

    state = State(point_t(NAN, 0.0, 0.0), vector_t::Zero(), 0.0, vector_t::Zero(), rosTime(1.10));
    EXPECT_FALSE(controller.Update(state, rosTime(1.10), 3, rosTime(1.10), 0.35, output, reason));
    EXPECT_EQ(output.position(), accepted.position());
}

TEST(BoundedTerminalPositionTrackingTest, RejectsMismatchedClockBeforeSubtractingTimestamps) {
    TerminalPositionTrackingController controller(Reference(point_t(1.0, 0.0, 0.0), 0.0));
    Reference output;
    std::string reason;
    const State state(point_t::Zero(), vector_t::Zero(), 0.0, vector_t::Zero(), rosTime(1.0));
    ASSERT_TRUE(controller.Update(state, rosTime(1.0), 1, rosTime(1.0), 0.35, output, reason));
    const Reference previous = output;

    const rclcpp::Time system_stamp(1050000000LL, RCL_SYSTEM_TIME);
    EXPECT_FALSE(controller.Update(state, system_stamp, 1, rosTime(1.05), 0.35, output, reason));
    EXPECT_NE(reason.find("clocks differ"), std::string::npos);
    EXPECT_EQ(output.position(), previous.position());
    EXPECT_EQ(output.velocity(), previous.velocity());
    EXPECT_EQ(output.acceleration(), previous.acceleration());
}

TEST(BoundedTerminalPositionTrackingTest, ReportsSignedStaleAndFutureReceiptAge) {
    TerminalPositionTrackingController controller(Reference(point_t(1.0, 0.0, 0.0), 0.0));
    const State state(point_t::Zero(), vector_t::Zero(), 0.0, vector_t::Zero(), rosTime(1.0));
    Reference output;
    std::string reason;
    ASSERT_TRUE(controller.Update(state, rosTime(1.0), 1, rosTime(1.0),
        0.35, output, reason)) << reason;
    const Reference accepted = output;

    const auto stale_receipt = rosTime(1.0);
    const auto stale_emission = rosTime(1.30);
    EXPECT_FALSE(controller.Update(state, stale_receipt, 1, stale_emission,
        0.35, output, reason));
    EXPECT_NE(reason.find("cause=stale"), std::string::npos);
    EXPECT_NE(reason.find("age_s=0.300000"), std::string::npos);
    EXPECT_NE(reason.find("receipt_ns=" + std::to_string(stale_receipt.nanoseconds())),
        std::string::npos);
    EXPECT_NE(reason.find("emission_ns=" + std::to_string(stale_emission.nanoseconds())),
        std::string::npos);
    EXPECT_NE(reason.find("maximum_odometry_age_s=0.250000"), std::string::npos);
    EXPECT_NE(reason.find("maximum_future_stamp_s=0.020000"), std::string::npos);
    EXPECT_EQ(output.position(), accepted.position());

    const auto future_receipt = rosTime(1.35);
    const auto future_emission = rosTime(1.30);
    EXPECT_FALSE(controller.Update(state, future_receipt, 1, future_emission,
        0.35, output, reason));
    EXPECT_NE(reason.find("cause=future"), std::string::npos);
    EXPECT_NE(reason.find("age_s=-"), std::string::npos);
    EXPECT_NE(reason.find("receipt_ns=" + std::to_string(future_receipt.nanoseconds())),
        std::string::npos);
    EXPECT_NE(reason.find("emission_ns=" + std::to_string(future_emission.nanoseconds())),
        std::string::npos);
    EXPECT_EQ(output.position(), accepted.position());
}

TEST(BoundedTerminalPositionTrackingTest, ReportsOriginalReceiptAndEmissionIntervalsOnRejection) {
    const State state(point_t::Zero(), vector_t::Zero(), 0.0,
        vector_t::Zero(), rosTime(1.0));
    const auto check = [&](const rclcpp::Time & receipt, const rclcpp::Time & emission,
                           const std::string & prefix, const std::string & sample_dt,
                           const std::string & emission_dt) {
        TerminalPositionTrackingController controller(Reference(point_t(1.0, 0.0, 0.0), 0.0));
        Reference output;
        std::string reason;
        const auto original = rosTime(1.0);
        ASSERT_TRUE(controller.Update(state, original, 1, original, 0.35, output, reason)) << reason;
        const Reference accepted = output;
        EXPECT_FALSE(controller.Update(state, receipt, 1, emission, 0.35, output, reason));
        EXPECT_EQ(reason.rfind(prefix, 0), 0U) << reason;
        EXPECT_NE(reason.find("sample_dt_s=" + sample_dt), std::string::npos) << reason;
        EXPECT_NE(reason.find("emission_dt_s=" + emission_dt), std::string::npos) << reason;
        EXPECT_NE(reason.find("current_receipt_ns=" + std::to_string(receipt.nanoseconds())),
            std::string::npos) << reason;
        EXPECT_NE(reason.find("previous_receipt_ns=" + std::to_string(original.nanoseconds())),
            std::string::npos) << reason;
        EXPECT_NE(reason.find("current_emission_ns=" + std::to_string(emission.nanoseconds())),
            std::string::npos) << reason;
        EXPECT_NE(reason.find("previous_emission_ns=" + std::to_string(original.nanoseconds())),
            std::string::npos) << reason;
        EXPECT_NE(reason.find("measured_age_s="), std::string::npos) << reason;
        EXPECT_NE(reason.find("current_receipt_clock_type="), std::string::npos) << reason;
        EXPECT_NE(reason.find("previous_emission_clock_type="), std::string::npos) << reason;
        EXPECT_NE(reason.find("maximum_sample_interval_s=0.250000"), std::string::npos) << reason;
        EXPECT_EQ(output.position(), accepted.position());
        EXPECT_EQ(output.velocity(), accepted.velocity());
        EXPECT_EQ(output.acceleration(), accepted.acceleration());
    };
    check(rosTime(1.30), rosTime(1.30),
        "terminal tracking sample interval is discontinuous", "0.300000", "0.300000");
    check(rosTime(0.95), rosTime(1.05),
        "terminal tracking sample interval is discontinuous", "-0.050000", "0.050000");
    check(rosTime(0.98), rosTime(0.99),
        "terminal tracking command-emission clock is discontinuous", "-0.020000", "-0.010000");
}

// A maneuver handover pauses evaluation (SIM: 0.252 s, just over the 0.25 s
// sample-interval limit); the adopting maneuver resumes the hold, which
// re-anchors its timing once instead of rejecting the pause. Without the
// handover the same gap is still rejected, as is an older sample after it.
TEST(BoundedTerminalPositionTrackingTest, HandoverPauseIsReanchoredOnceButOdometryGapsStillFail) {
    const State state(point_t::Zero(), vector_t::Zero(), 0.0, vector_t::Zero(), rosTime(1.0));
    Reference output;
    std::string reason;

    TerminalPositionTrackingController plain(Reference(point_t(1.0, 0.0, 0.0), 0.0));
    ASSERT_TRUE(plain.Update(state, rosTime(1.0), 1, rosTime(1.0), 0.35, output, reason)) << reason;
    EXPECT_FALSE(plain.Update(state, rosTime(1.252), 1, rosTime(1.252), 0.35, output, reason));

    TerminalPositionTrackingController handed_over(Reference(point_t(1.0, 0.0, 0.0), 0.0));
    ASSERT_TRUE(handed_over.Update(state, rosTime(1.0), 1, rosTime(1.0), 0.35, output, reason)) << reason;
    handed_over.ResumeAfterHandover();
    EXPECT_TRUE(handed_over.Update(state, rosTime(1.252), 1, rosTime(1.252), 0.35, output, reason)) << reason;
    EXPECT_TRUE(handed_over.Update(state, rosTime(1.272), 1, rosTime(1.272), 0.35, output, reason)) << reason;
    // Once only: a later gap is an odometry gap again.
    EXPECT_FALSE(handed_over.Update(state, rosTime(1.6), 1, rosTime(1.6), 0.35, output, reason));

    TerminalPositionTrackingController backwards(Reference(point_t(1.0, 0.0, 0.0), 0.0));
    ASSERT_TRUE(backwards.Update(state, rosTime(1.0), 1, rosTime(1.0), 0.35, output, reason)) << reason;
    backwards.ResumeAfterHandover();
    EXPECT_FALSE(backwards.Update(state, rosTime(0.95), 1, rosTime(1.1), 0.35, output, reason));
}

TEST(BoundedTerminalPositionTrackingTest, RepeatedEmissionIsIdempotentButStillValidatesSampleAndReset) {
    const Reference nominal(point_t(1.5, 0.0, 0.0), 0.0);
    TerminalPositionTrackingController repeated(nominal);
    TerminalPositionTrackingController baseline(nominal);
    Reference output;
    Reference expected;
    std::string reason;
    const State initial(point_t::Zero(), vector_t::Zero(), 0.0, vector_t::Zero(), rosTime(1.0));
    ASSERT_TRUE(repeated.Update(initial, rosTime(1.0), 3, rosTime(1.0), 0.4, output, reason)) << reason;
    ASSERT_TRUE(baseline.Update(initial, rosTime(1.0), 3, rosTime(1.0), 0.4, expected, reason)) << reason;
    const State next(point_t::Zero(), vector_t::Zero(), 0.0, vector_t::Zero(), rosTime(1.05));
    ASSERT_TRUE(repeated.Update(next, rosTime(1.05), 3, rosTime(1.05), 0.4, output, reason)) << reason;
    ASSERT_TRUE(baseline.Update(next, rosTime(1.05), 3, rosTime(1.05), 0.4, expected, reason)) << reason;
    const Reference once = output;
    ASSERT_TRUE(repeated.Update(next, rosTime(1.05), 3, rosTime(1.05), 0.4, output, reason)) << reason;
    EXPECT_EQ(output.position(), once.position());
    EXPECT_EQ(output.velocity(), once.velocity());
    EXPECT_EQ(output.acceleration(), once.acceleration());
    EXPECT_FALSE(repeated.Update(next, rosTime(1.05), 4, rosTime(1.05), 0.4, output, reason));
    EXPECT_NE(reason.find("reset-counter"), std::string::npos);
    EXPECT_FALSE(repeated.Update(next, rosTime(1.05), 3, rosTime(1.05), 0.0, output, reason));
    EXPECT_NE(reason.find("clearance"), std::string::npos);
    const State later(point_t::Zero(), vector_t::Zero(), 0.0, vector_t::Zero(), rosTime(1.10));
    ASSERT_TRUE(repeated.Update(later, rosTime(1.10), 3, rosTime(1.10), 0.4, output, reason)) << reason;
    ASSERT_TRUE(baseline.Update(later, rosTime(1.10), 3, rosTime(1.10), 0.4, expected, reason)) << reason;
    EXPECT_NEAR((output.position() - expected.position()).norm(), 0.0, 1.0e-12);
    EXPECT_NEAR((output.velocity() - expected.velocity()).norm(), 0.0, 1.0e-12);
    EXPECT_NEAR((output.acceleration() - expected.acceleration()).norm(), 0.0, 1.0e-12);
}

TEST(BoundedTerminalPositionTrackingTest, RejectsInsufficientSafeAuthorityInsteadOfMovingTowardCable) {
    TerminalPositionTrackingController controller(
        Reference(point_t(1.5, 0.0, 0.0), 0.0)
    );
    Reference output;
    std::string reason;
    State state(point_t::Zero(), vector_t::Zero(), 0.0, vector_t::Zero(), rosTime(1.0));
    EXPECT_FALSE(controller.Update(state, rosTime(1.0), 1, rosTime(1.0), -0.01, output, reason));
    EXPECT_NE(reason.find("clearance"), std::string::npos);
}

TEST(BoundedTerminalPositionTrackingTest, ShrinkingClearanceOrInvalidSampleRetainsLastAcceptedCommand) {
    TerminalPositionTrackingController controller(
        Reference(point_t(1.5, 0.0, 0.0), 0.0)
    );
    Reference output;
    std::string reason;
    for (int tick = 0; tick < 20; ++tick) {
        const auto stamp = rosTime(1.0 + tick * 0.05);
        State state(point_t::Zero(), vector_t::Zero(), 0.0, vector_t::Zero(), stamp);
        ASSERT_TRUE(controller.Update(state, stamp, 2, stamp, 0.35, output, reason)) << reason;
    }
    const Reference last_accepted = output;
    const auto next_stamp = rosTime(2.0);
    State next_state(point_t::Zero(), vector_t::Zero(), 0.0, vector_t::Zero(), next_stamp);
    EXPECT_FALSE(controller.Update(next_state, next_stamp, 2, next_stamp, 0.01, output, reason));
    EXPECT_NE(reason.find("clearance"), std::string::npos);
    EXPECT_EQ(output.position(), last_accepted.position());
    EXPECT_EQ(output.velocity(), last_accepted.velocity());
    EXPECT_EQ(output.acceleration(), last_accepted.acceleration());
    EXPECT_FALSE(controller.Update(
        next_state,
        rosTime(1.95),
        2,
        rosTime(2.50),
        0.35,
        output,
        reason
    ));
    EXPECT_NE(reason.find("stale"), std::string::npos);
    EXPECT_EQ(output.position(), last_accepted.position());
}

TEST(BoundedTerminalPositionTrackingTest, RejectsBiasedVelocityOffsetThroughGuardAndAdapter) {
    constexpr double dt = 0.05;
    constexpr double bias = 0.24;
    constexpr double target = 1.5;
    Plant baseline;
    baseline.position = target - bias;
    for (int tick = 0; tick < 800; ++tick) {
        const double velocity_noise = 0.01 * std::sin(0.13 * tick) + 0.004 * std::sin(0.041 * tick);
        oneStep(baseline, target, 0.0, 0.0, dt, bias + velocity_noise);
    }
    EXPECT_NEAR(std::abs(baseline.position - target), 0.24, 0.04);
    const Reference nominal(point_t(target, 0.0, 0.0), 0.0);
    auto tracker = std::make_shared<TerminalPositionTrackingController>(nominal);

    ManeuverReferenceSafetyConfig guard_config;
    guard_config.max_jerk_m_s3 = 1.0;  // unchanged production guard limits
    ManeuverReferenceSafetyGuard guard(guard_config);
    Plant plant;
    plant.position = target - bias;
    Reference output;
    std::string reason;
    auto guard_time = ManeuverReferenceSafetyGuard::Clock::now();
    bool saw_correction = false;
    double previous_position_sp = target;
    double previous_velocity_sp = 0.0;
    double previous_acceleration_sp = 0.0;
    State sampled_state;
    rclcpp::Time sampled_stamp = rosTime(1.0);

    for (int tick = 0; tick <= 1200; ++tick) {
        const double t = tick * dt;
        double velocity_noise = 0.01 * std::sin(0.13 * tick) + 0.004 * std::sin(0.041 * tick);
        if (tick % 2 == 0) {  // estimator updates at 10 Hz; commands emit at 20 Hz
            const double correlated_noise = 0.004 * std::sin(0.37 * tick) + 0.002 * std::sin(0.11 * tick);
            const double measured_position = plant.position + correlated_noise;
            sampled_stamp = rosTime(t + 1.0);
            sampled_state = State(
                point_t(measured_position, 0.0, 0.0),
                vector_t(plant.velocity + bias + velocity_noise, 0.0, 0.0),
                0.0,
                vector_t::Zero(),
                sampled_stamp
            );
        } else {
            velocity_noise = 0.01 * std::sin(0.13 * (tick - 1)) + 0.004 * std::sin(0.041 * (tick - 1));
        }
        const double old_integral_norm = tracker->integralTargetOffsetNorm();
        ASSERT_TRUE(tracker->Update(sampled_state, sampled_stamp, 8, rosTime(t + 1.0), 0.35, output, reason))
            << "at t=" << t << ": " << reason;
        if (tick % 2 == 1) {
            EXPECT_DOUBLE_EQ(tracker->integralTargetOffsetNorm(), old_integral_norm)
                << "a repeated sensor timestamp must sample the profile without integrating twice";
        }

        const auto evaluation = guard.observeReference(output, guard_time);
        ASSERT_EQ(evaluation.decision, ManeuverReferenceSafetyDecision::ACCEPT)
            << "at t=" << t << ": " << evaluation.reason;
        const auto msg = TrajectorySetpointAdapter(output).ToMsg();
        // Convert the adapter's PX4 NED values back to III ROS-world axes.
        const double position_sp = msg.position[0];
        const double velocity_sp = msg.velocity[0];
        const double acceleration_sp = msg.acceleration[0];
        ASSERT_TRUE(std::isfinite(position_sp));
        ASSERT_TRUE(std::isfinite(velocity_sp));
        ASSERT_TRUE(std::isfinite(acceleration_sp));
        EXPECT_LE(std::abs(position_sp - previous_position_sp), 0.02);
        EXPECT_LE(std::abs(velocity_sp - previous_velocity_sp), 0.06);
        EXPECT_LE(std::abs(acceleration_sp - previous_acceleration_sp), 0.055);
        const point_t commanded_offset(position_sp - target, -msg.position[1], -msg.position[2]);
        const vector_t commanded_velocity(velocity_sp, -msg.velocity[1], -msg.velocity[2]);
        const vector_t commanded_acceleration(acceleration_sp, -msg.acceleration[1], -msg.acceleration[2]);
        EXPECT_LE(commanded_offset.norm(), 0.350001);
        EXPECT_LE(commanded_velocity.norm(), 0.100001);
        EXPECT_LE(commanded_acceleration.norm(), 0.200001);
        EXPECT_LE(std::abs(velocity_sp), 0.100001);
        EXPECT_LE(std::abs(acceleration_sp), 0.200001);
        EXPECT_NEAR((position_sp - previous_position_sp) / dt, velocity_sp, 0.012);
        EXPECT_NEAR((velocity_sp - previous_velocity_sp) / dt, acceleration_sp, 0.055);
        EXPECT_LE(std::abs((acceleration_sp - previous_acceleration_sp) / dt), 0.56);
        saw_correction = saw_correction || std::abs(position_sp - target) > 1e-4;

        oneStep(plant, position_sp, velocity_sp, acceleration_sp, dt, bias + velocity_noise);
        previous_position_sp = position_sp;
        previous_velocity_sp = velocity_sp;
        previous_acceleration_sp = acceleration_sp;
        guard_time += std::chrono::milliseconds(50);

    }

    EXPECT_TRUE(saw_correction);
    EXPECT_LT(std::abs(plant.position - target), 0.1);
    EXPECT_FALSE(guard.faultLatched());
}

TEST(BoundedTerminalPositionTrackingTest, RejectsNegativeBiasedVelocityOffset) {
    constexpr double dt = 0.05;
    constexpr double bias = -0.24;
    constexpr double target = -1.5;
    TerminalPositionTrackingController controller(Reference(point_t(target, 0.0, 0.0), 0.0));
    ManeuverReferenceSafetyGuard guard(ManeuverReferenceSafetyConfig{});
    Plant plant;
    plant.position = target - bias;
    Reference output;
    std::string reason;
    auto guard_time = ManeuverReferenceSafetyGuard::Clock::now();
    for (int tick = 0; tick <= 1200; ++tick) {
        const double t = tick * dt;
        const double noise = 0.01 * std::sin(0.13 * tick) + 0.004 * std::sin(0.041 * tick);
        const auto stamp = rosTime(t + 1.0);
        const State state(
            point_t(plant.position, 0.0, 0.0),
            vector_t(plant.velocity + bias + noise, 0.0, 0.0),
            0.0,
            vector_t::Zero(),
            stamp
        );
        ASSERT_TRUE(controller.Update(state, stamp, 4, stamp, 0.245, output, reason)) << reason;
        const auto guard_result = guard.observeReference(
            output,
            guard_time
        );
        ASSERT_EQ(guard_result.decision, ManeuverReferenceSafetyDecision::ACCEPT) << guard_result.reason;
        const auto msg = TrajectorySetpointAdapter(output).ToMsg();
        EXPECT_LE(controller.emittedOffsetNorm(), 0.245001);
        oneStep(plant, msg.position[0], msg.velocity[0], msg.acceleration[0], dt, bias + noise);
        guard_time += std::chrono::milliseconds(50);
    }
    EXPECT_LT(std::abs(plant.position - target), 0.1);
    EXPECT_LT(controller.emittedOffsetNorm(), 0.245);
    EXPECT_FALSE(guard.faultLatched());
}

TEST(BoundedTerminalPositionTrackingTest, ConvergedAdaptiveHoldDoesNotExpireAfterNinetySeconds) {
    constexpr double target = 0.4;
    TerminalPositionTrackingController controller(Reference(point_t(target, 0.0, 0.0), 0.0));
    Reference output;
    std::string reason;
    const State state(point_t(target, 0.0, 0.0), vector_t::Zero(), 0.0, vector_t::Zero(), rosTime(1.0));
    rclcpp::Time sample_stamp = rosTime(1.0);
    for (int tick = 0; tick <= 2500; ++tick) {
        if (tick > 0 && tick % 2 == 0) {
            sample_stamp = rosTime(1.0 + tick * 0.05);
        }
        ASSERT_TRUE(controller.Update(
            state,
            sample_stamp,
            1,
            rosTime(1.0 + tick * 0.05),
            0.35,
            output,
            reason
        )) << "at t=" << tick * 0.05 << ": " << reason;
    }
}

TEST(BoundedTerminalPositionTrackingTest, QuiescenceFinishesActiveSegmentThenResumesWithoutPvaJump) {
    constexpr double dt = 0.05;
    TerminalPositionTrackingController controller(Reference(point_t(1.5, 0.0, 0.0), 0.0));
    ManeuverReferenceSafetyGuard guard(ManeuverReferenceSafetyConfig{});
    Reference output;
    std::string reason;
    const State state(point_t(1.26, 0.0, 0.0), vector_t::Zero(), 0.0, vector_t::Zero(), rosTime(1.0));
    rclcpp::Time sample_stamp = rosTime(1.0);
    auto guard_time = ManeuverReferenceSafetyGuard::Clock::now();

    for (int tick = 0; tick <= 7; ++tick) {
        const double t = tick * dt;
        if (tick > 0 && tick % 2 == 0) sample_stamp = rosTime(1.0 + t);
        ASSERT_TRUE(controller.Update(state, sample_stamp, 6, rosTime(1.0 + t), 0.35, output, reason)) << reason;
        const auto accepted = guard.observeReference(output, guard_time);
        ASSERT_EQ(accepted.decision, ManeuverReferenceSafetyDecision::ACCEPT) << accepted.reason;
        guard_time += std::chrono::milliseconds(50);
    }
    ASSERT_GT(controller.emittedOffsetNorm(), 0.0);
    ASSERT_FALSE(controller.isQuiescent());

    controller.RequestQuiescence();
    controller.RequestQuiescence();  // idempotent even while a segment is active
    ASSERT_FALSE(controller.isQuiescent());
    const double frozen_target_norm = controller.integralTargetOffsetNorm();
    ASSERT_GT(frozen_target_norm, 0.0);

    bool reached_rest = false;
    int next_tick = 8;
    for (int tick = 8; tick < 80; ++tick) {
        const double t = tick * dt;
        if (tick % 2 == 0) sample_stamp = rosTime(1.0 + t);
        ASSERT_TRUE(controller.Update(state, sample_stamp, 6, rosTime(1.0 + t), 0.35, output, reason)) << reason;
        const auto accepted = guard.observeReference(output, guard_time);
        ASSERT_EQ(accepted.decision, ManeuverReferenceSafetyDecision::ACCEPT) << accepted.reason;
        guard_time += std::chrono::milliseconds(50);
        if (controller.isQuiescent()) {
            reached_rest = true;
            next_tick = tick + 1;
            break;
        }
    }
    ASSERT_TRUE(reached_rest);
    const Reference rest_reference = output;
    EXPECT_TRUE(rest_reference.position().allFinite());
    EXPECT_TRUE(rest_reference.velocity().allFinite());
    EXPECT_TRUE(rest_reference.acceleration().allFinite());
    EXPECT_DOUBLE_EQ(rest_reference.velocity().norm(), 0.0);
    EXPECT_DOUBLE_EQ(rest_reference.acceleration().norm(), 0.0);
    EXPECT_DOUBLE_EQ(controller.integralTargetOffsetNorm(), frozen_target_norm);
    EXPECT_GT(std::abs(controller.emittedOffsetNorm() - frozen_target_norm), 1.0e-8)
        << "the active segment must finish at its committed endpoint without queuing another segment";

    const Reference before_stale = output;
    EXPECT_FALSE(controller.Update(
        state,
        sample_stamp,
        6,
        rosTime(2.0 + next_tick * dt),
        0.35,
        output,
        reason
    ));
    EXPECT_NE(reason.find("stale"), std::string::npos);
    EXPECT_EQ(output.position(), before_stale.position());
    EXPECT_EQ(output.velocity(), before_stale.velocity());
    EXPECT_EQ(output.acceleration(), before_stale.acceleration());

    // Fresh sensor samples continue to arrive while quiescent. They must not
    // restart the integral or schedule a new segment.
    for (int i = 0; i < 40; ++i) {
        const int tick = next_tick + i;
        const double t = tick * dt;
        if (tick % 2 == 0) sample_stamp = rosTime(1.0 + t);
        ASSERT_TRUE(controller.Update(state, sample_stamp, 6, rosTime(1.0 + t), 0.35, output, reason)) << reason;
        const auto accepted = guard.observeReference(output, guard_time);
        ASSERT_EQ(accepted.decision, ManeuverReferenceSafetyDecision::ACCEPT) << accepted.reason;
        EXPECT_EQ(output.position(), rest_reference.position());
        EXPECT_EQ(output.velocity(), rest_reference.velocity());
        EXPECT_EQ(output.acceleration(), rest_reference.acceleration());
        EXPECT_DOUBLE_EQ(controller.integralTargetOffsetNorm(), frozen_target_norm);
        EXPECT_TRUE(controller.isQuiescent());
        guard_time += std::chrono::milliseconds(50);
    }

    const double before_resume_position = output.position()[0];
    const double before_resume_velocity = output.velocity()[0];
    const double before_resume_acceleration = output.acceleration()[0];
    controller.ResumeTracking();
    const double resume_time_s = (next_tick + 40) * dt;
    sample_stamp = rosTime(1.0 + resume_time_s);
    ASSERT_TRUE(controller.Update(state, sample_stamp, 6, rosTime(1.0 + resume_time_s), 0.35, output, reason)) << reason;
    const auto resumed = guard.observeReference(output, guard_time);
    EXPECT_EQ(resumed.decision, ManeuverReferenceSafetyDecision::ACCEPT) << resumed.reason;
    EXPECT_DOUBLE_EQ(output.position()[0], before_resume_position);
    EXPECT_DOUBLE_EQ(output.velocity()[0], before_resume_velocity);
    EXPECT_DOUBLE_EQ(output.acceleration()[0], before_resume_acceleration);
    EXPECT_FALSE(controller.isQuiescent());
}

TEST(BoundedTerminalPositionTrackingTest, FaultStopPreservesCertifiedBallWithoutFreshOdometry) {
    constexpr double radius = 0.09378287315655387;
    constexpr double dt = 0.05;
    const Reference nominal(point_t(1.5, 0.0, 0.0), 0.0);
    TerminalPositionTrackingController controller(nominal);
    ManeuverReferenceSafetyGuard guard(ManeuverReferenceSafetyConfig{});
    auto guard_time = ManeuverReferenceSafetyGuard::Clock::now();
    iii_drone::control::KinematicStopLimits generic_limits;
    generic_limits.max_acceleration_m_s2 = 0.2;
    generic_limits.max_jerk_m_s3 = 0.5;
    Reference output;
    std::string reason;
    int fault_tick = 0;
    for (int tick = 0; tick < 100; ++tick) {
        const auto stamp = rosTime(1.0 + tick * dt);
        const State state(point_t::Zero(), vector_t::Zero(), 0.0, vector_t::Zero(), stamp);
        ASSERT_TRUE(controller.Update(state, stamp, 6, stamp, radius, output, reason)) << reason;
        const auto accepted = guard.observeReference(output, guard_time);
        ASSERT_EQ(accepted.decision, ManeuverReferenceSafetyDecision::ACCEPT) << accepted.reason;
        const iii_drone::control::KinematicStopTrajectory generic_stop(output, generic_limits);
        if ((generic_stop.terminalReference().position() - nominal.position()).norm() > radius + 1.0e-4) {
            fault_tick = tick;
            break;
        }
        guard_time += std::chrono::milliseconds(50);
    }
    ASSERT_GT(fault_tick, 0) << "regression must expose the old generic-stop overshoot";
    const auto fault_stamp = rosTime(1.0 + fault_tick * dt);
    const Reference last_accepted = output;
    ASSERT_GT(output.velocity().norm(), 1.0e-4);
    ASSERT_TRUE(controller.ContinueCommittedStop(fault_stamp, output, reason)) << reason;
    EXPECT_EQ(output.position(), last_accepted.position());
    EXPECT_EQ(output.velocity(), last_accepted.velocity());
    EXPECT_EQ(output.acceleration(), last_accepted.acceleration());
    EXPECT_FALSE(controller.isQuiescent());
    const double frozen_integral = controller.integralTargetOffsetNorm();
    bool stopped = false;
    for (int tick = fault_tick + 1; tick < fault_tick + 500; ++tick) {
        guard_time += std::chrono::milliseconds(50);
        const Reference previous = output;
        ASSERT_TRUE(controller.ContinueCommittedStop(rosTime(1.0 + tick * dt), output, reason)) << reason;
        const auto accepted = guard.observeReference(output, guard_time);
        ASSERT_EQ(accepted.decision, ManeuverReferenceSafetyDecision::ACCEPT) << accepted.reason;
        const auto msg = TrajectorySetpointAdapter(output).ToMsg();
        EXPECT_TRUE(std::isfinite(msg.position[0]) && std::isfinite(msg.velocity[0]) && std::isfinite(msg.acceleration[0]));
        // point_t is float32; the subtraction at nominal x=1.5 quantizes
        // the reported offset by several 1e-8 m at this radius.
        EXPECT_LE((output.position() - nominal.position()).norm(), radius + 1.0e-6);
        EXPECT_LE(output.velocity().norm(), 0.1 + 1.0e-9);
        EXPECT_LE(output.acceleration().norm(), 0.2 + 1.0e-9);
        EXPECT_LE((output.acceleration() - previous.acceleration()).norm() / dt, 0.5 + 1.0e-8);
        EXPECT_DOUBLE_EQ(controller.integralTargetOffsetNorm(), frozen_integral);
        if (controller.isQuiescent()) {
            stopped = true;
            break;
        }
    }
    ASSERT_TRUE(stopped);
    EXPECT_DOUBLE_EQ(output.velocity().norm(), 0.0);
    EXPECT_DOUBLE_EQ(output.acceleration().norm(), 0.0);
    const Reference stopped_reference = output;
    controller.ResumeTracking();
    const State new_state(point_t::Zero(), vector_t::Zero(), 0.0, vector_t::Zero(), rosTime(40.0));
    EXPECT_FALSE(controller.Update(new_state, rosTime(40.0), 6, rosTime(40.0), radius, output, reason));
    EXPECT_EQ(output.position(), stopped_reference.position());
    ASSERT_TRUE(controller.ContinueCommittedStop(rosTime(41.0), output, reason));
    EXPECT_EQ(output.position(), stopped_reference.position());
    EXPECT_TRUE(controller.isQuiescent());
}

TEST(BoundedTerminalPositionTrackingTest, FaultStopRejectsClockReversalAndMismatchWithoutInventingRest) {
    TerminalPositionTrackingController controller(Reference(point_t(1.5, 0.0, 0.0), 0.0));
    Reference output;
    std::string reason;
    for (int tick = 0; tick <= 7; ++tick) {
        const auto stamp = rosTime(1.0 + tick * 0.05);
        const State state(point_t::Zero(), vector_t::Zero(), 0.0, vector_t::Zero(), stamp);
        ASSERT_TRUE(controller.Update(state, stamp, 1, stamp, 0.35, output, reason));
    }
    const Reference previous = output;
    ASSERT_GT(previous.velocity().norm(), 0.0);
    EXPECT_FALSE(controller.ContinueCommittedStop(rosTime(1.3), output, reason));
    EXPECT_FALSE(controller.ContinueCommittedStop(rclcpp::Time(1400000000LL, RCL_SYSTEM_TIME), output, reason));
    EXPECT_EQ(output.position(), previous.position());
    EXPECT_EQ(output.velocity(), previous.velocity());
    EXPECT_EQ(output.acceleration(), previous.acceleration());
    EXPECT_FALSE(controller.isQuiescent());
    ASSERT_TRUE(controller.ContinueCommittedStop(rosTime(1.35), output, reason));
    EXPECT_EQ(output.position(), previous.position());
}

TEST(BoundedTerminalPositionTrackingTest, FaultBeforeFirstMeasurementRetainsStationaryNominal) {
    const Reference nominal(point_t(1.5, 2.0, 3.0), 0.4);
    TerminalPositionTrackingController controller(nominal);
    Reference output;
    std::string reason;
    ASSERT_TRUE(controller.ContinueCommittedStop(rosTime(1.0), output, reason));
    EXPECT_EQ(output.position(), nominal.position());
    EXPECT_DOUBLE_EQ(output.yaw(), nominal.yaw());
    EXPECT_DOUBLE_EQ(output.velocity().norm(), 0.0);
    EXPECT_DOUBLE_EQ(output.acceleration().norm(), 0.0);
    EXPECT_TRUE(controller.isQuiescent());
}
