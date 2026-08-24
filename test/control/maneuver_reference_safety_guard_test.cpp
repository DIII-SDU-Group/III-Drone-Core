#include <chrono>
#include <limits>

#include <gtest/gtest.h>

#include <iii_drone_core/control/maneuver/maneuver_reference_safety_guard.hpp>

using iii_drone::control::Reference;
using iii_drone::control::maneuver::ManeuverReferenceSafetyConfig;
using iii_drone::control::maneuver::ManeuverReferenceSafetyDecision;
using iii_drone::control::maneuver::ManeuverReferenceSafetyGuard;
using iii_drone::types::point_t;
using iii_drone::types::vector_t;

namespace {

Reference makeReference(
    const point_t & position,
    const vector_t & velocity = vector_t::Zero(),
    const vector_t & acceleration = vector_t::Zero()
) {
    return Reference(position, 0.0, velocity, 0.0, acceleration, 0.0);
}

}  // namespace

TEST(ManeuverReferenceSafetyGuard, LatchesAfterReferenceDeadlineAndRejectsLateRecovery) {
    ManeuverReferenceSafetyConfig config;
    config.loss_timeout = std::chrono::milliseconds(500);
    ManeuverReferenceSafetyGuard guard(config);
    const auto start = ManeuverReferenceSafetyGuard::Clock::time_point{};

    EXPECT_EQ(
        guard.observeReference(makeReference(point_t::Zero()), start).decision,
        ManeuverReferenceSafetyDecision::ACCEPT
    );
    EXPECT_EQ(
        guard.observeMiss(start + std::chrono::milliseconds(200)).decision,
        ManeuverReferenceSafetyDecision::HOLD_LAST
    );
    EXPECT_EQ(
        guard.observeMiss(start + std::chrono::milliseconds(400)).decision,
        ManeuverReferenceSafetyDecision::HOLD_LAST
    );
    EXPECT_EQ(
        guard.observeMiss(start + std::chrono::milliseconds(600)).decision,
        ManeuverReferenceSafetyDecision::BEGIN_STOP
    );

    const auto recovery = guard.observeReference(
        makeReference(point_t(4.5, 0.0, 0.0)),
        start + std::chrono::milliseconds(620)
    );
    EXPECT_EQ(recovery.decision, ManeuverReferenceSafetyDecision::REJECT_LATCHED);
    EXPECT_TRUE(guard.faultLatched());
}

TEST(ManeuverReferenceSafetyGuard, RejectsAReferenceOutsideTheKinematicEnvelope) {
    ManeuverReferenceSafetyConfig config;
    config.loss_timeout = std::chrono::milliseconds(500);
    config.position_tolerance_m = 0.1;
    ManeuverReferenceSafetyGuard guard(config);
    const auto start = ManeuverReferenceSafetyGuard::Clock::time_point{};

    EXPECT_EQ(
        guard.observeReference(
            makeReference(point_t::Zero(), vector_t(1.0, 0.0, 0.0)),
            start
        ).decision,
        ManeuverReferenceSafetyDecision::ACCEPT
    );

    const auto discontinuity = guard.observeReference(
        makeReference(point_t(4.5, 0.0, 0.0), vector_t(1.0, 0.0, 0.0)),
        start + std::chrono::milliseconds(200)
    );
    EXPECT_EQ(discontinuity.decision, ManeuverReferenceSafetyDecision::BEGIN_STOP);
    EXPECT_GT(discontinuity.position_error_m, discontinuity.position_limit_m);
}

TEST(ManeuverReferenceSafetyGuard, AcceptsSmoothBoundedMotionAndResetClearsFault) {
    ManeuverReferenceSafetyConfig config;
    config.loss_timeout = std::chrono::milliseconds(500);
    config.position_tolerance_m = 0.01;
    config.velocity_tolerance_m_s = 0.01;
    config.acceleration_tolerance_m_s2 = 0.01;
    config.max_jerk_m_s3 = 1.0;
    ManeuverReferenceSafetyGuard guard(config);
    const auto start = ManeuverReferenceSafetyGuard::Clock::time_point{};
    const Reference initial = makeReference(
        point_t::Zero(),
        vector_t(1.0, 0.0, 0.0),
        vector_t(0.2, 0.0, 0.0)
    );
    EXPECT_EQ(
        guard.observeReference(initial, start).decision,
        ManeuverReferenceSafetyDecision::ACCEPT
    );

    const double dt = 0.2;
    const Reference smooth = makeReference(
        point_t(1.0 * dt + 0.5 * 0.2 * dt * dt, 0.0, 0.0),
        vector_t(1.0 + 0.2 * dt, 0.0, 0.0),
        vector_t(0.2, 0.0, 0.0)
    );
    const auto accepted = guard.observeReference(
        smooth,
        start + std::chrono::milliseconds(200)
    );
    EXPECT_EQ(accepted.decision, ManeuverReferenceSafetyDecision::ACCEPT)
        << accepted.reason
        << " position=" << accepted.position_error_m << "/" << accepted.position_limit_m
        << " velocity=" << accepted.velocity_error_m_s << "/" << accepted.velocity_limit_m_s
        << " acceleration=" << accepted.acceleration_error_m_s2 << "/"
        << accepted.acceleration_limit_m_s2
        << " yaw=" << accepted.yaw_error_rad << "/" << accepted.yaw_limit_rad
        << " yaw_rate=" << accepted.yaw_rate_error_rad_s << "/"
        << accepted.yaw_rate_limit_rad_s
        << " yaw_acceleration=" << accepted.yaw_acceleration_error_rad_s2 << "/"
        << accepted.yaw_acceleration_limit_rad_s2;

    EXPECT_EQ(
        guard.observeMiss(start + std::chrono::milliseconds(800)).decision,
        ManeuverReferenceSafetyDecision::BEGIN_STOP
    );
    guard.reset();
    EXPECT_FALSE(guard.faultLatched());
    EXPECT_EQ(
        guard.observeReference(initial, start + std::chrono::seconds(1)).decision,
        ManeuverReferenceSafetyDecision::ACCEPT
    );
}

TEST(ManeuverReferenceSafetyGuard, AcceptsPx4PerAxisControlSelection) {
    ManeuverReferenceSafetyConfig config;
    ManeuverReferenceSafetyGuard guard(config);
    const auto start = ManeuverReferenceSafetyGuard::Clock::time_point{};
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const Reference line_pid_reference(
        point_t(1.0, 2.0, nan),
        0.2,
        vector_t(nan, nan, 0.1),
        0.0,
        vector_t(nan, nan, nan),
        nan
    );
    EXPECT_EQ(
        guard.observeReference(line_pid_reference, start).decision,
        ManeuverReferenceSafetyDecision::ACCEPT
    );
    EXPECT_EQ(
        guard.observeReference(line_pid_reference, start + std::chrono::milliseconds(50)).decision,
        ManeuverReferenceSafetyDecision::ACCEPT
    );
}

TEST(ManeuverReferenceSafetyGuard, RejectsInfiniteOrUncontrolledAxes) {
    ManeuverReferenceSafetyConfig config;
    const auto start = ManeuverReferenceSafetyGuard::Clock::time_point{};
    const double nan = std::numeric_limits<double>::quiet_NaN();

    ManeuverReferenceSafetyGuard infinite_guard(config);
    EXPECT_EQ(
        infinite_guard.observeReference(
            Reference(
                point_t(std::numeric_limits<double>::infinity(), 0.0, 0.0), 0.0
            ),
            start
        ).decision,
        ManeuverReferenceSafetyDecision::BEGIN_STOP
    );

    ManeuverReferenceSafetyGuard uncontrolled_guard(config);
    EXPECT_EQ(
        uncontrolled_guard.observeReference(
            Reference(
                point_t(nan, 0.0, 0.0), 0.0,
                vector_t(nan, 0.0, 0.0), 0.0,
                vector_t(nan, 0.0, 0.0), 0.0
            ),
            start
        ).decision,
        ManeuverReferenceSafetyDecision::BEGIN_STOP
    );
}
