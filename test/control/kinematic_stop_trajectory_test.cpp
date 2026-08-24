#include <gtest/gtest.h>

#include <iii_drone_core/control/kinematic_stop_trajectory.hpp>

using iii_drone::control::KinematicStopLimits;
using iii_drone::control::KinematicStopTrajectory;
using iii_drone::control::Reference;
using iii_drone::types::point_t;
using iii_drone::types::vector_t;

TEST(KinematicStopTrajectory, PreservesInitialStateAndStopsAtTerminalState) {
    const Reference initial(
        point_t(2.0, -1.0, 4.0),
        0.4,
        vector_t(1.2, -0.3, 0.2),
        0.25,
        vector_t(-0.1, 0.05, 0.0),
        -0.1
    );
    const KinematicStopLimits limits{0.5, 1.0, 0.75, 1.5};
    const KinematicStopTrajectory trajectory(initial, limits);

    const Reference first = trajectory.sample(0.0);
    EXPECT_TRUE(first.position().isApprox(initial.position(), 1.0e-12));
    EXPECT_TRUE(first.velocity().isApprox(initial.velocity(), 1.0e-12));
    EXPECT_TRUE(first.acceleration().isApprox(initial.acceleration(), 1.0e-12));
    EXPECT_DOUBLE_EQ(first.yaw(), initial.yaw());
    EXPECT_DOUBLE_EQ(first.yaw_rate(), initial.yaw_rate());
    EXPECT_DOUBLE_EQ(first.yaw_acceleration(), initial.yaw_acceleration());

    const Reference terminal = trajectory.sample(trajectory.durationS() + 1.0);
    EXPECT_TRUE(terminal.velocity().isZero(1.0e-12));
    EXPECT_TRUE(terminal.acceleration().isZero(1.0e-12));
    EXPECT_DOUBLE_EQ(terminal.yaw_rate(), 0.0);
    EXPECT_DOUBLE_EQ(terminal.yaw_acceleration(), 0.0);
}

TEST(KinematicStopTrajectory, ObeysConfiguredAccelerationAndJerkLimits) {
    const Reference initial(
        point_t::Zero(), 0.0, vector_t(1.5, 0.5, -0.3), 0.4,
        vector_t(-0.2, 0.1, 0.05), -0.15
    );
    const KinematicStopLimits limits{0.6, 0.8, 0.7, 1.1};
    const KinematicStopTrajectory trajectory(initial, limits);
    const double dt = trajectory.durationS() / 2000.0;

    Reference previous = trajectory.sample(0.0);
    for (int index = 1; index <= 2000; ++index) {
        const Reference current = trajectory.sample(index * dt);
        EXPECT_LE(current.acceleration().norm(), limits.max_acceleration_m_s2 + 1.0e-9);
        EXPECT_LE(std::abs(current.yaw_acceleration()), limits.max_yaw_acceleration_rad_s2 + 1.0e-9);
        const vector_t jerk = (current.acceleration() - previous.acceleration()) / dt;
        const double yaw_jerk =
            (current.yaw_acceleration() - previous.yaw_acceleration()) / dt;
        EXPECT_LE(jerk.norm(), limits.max_jerk_m_s3 + 2.0e-3);
        EXPECT_LE(std::abs(yaw_jerk), limits.max_yaw_jerk_rad_s3 + 2.0e-3);
        previous = current;
    }
}

TEST(KinematicStopTrajectory, PreservesAnOverLimitIncomingAccelerationWithoutGrowingIt) {
    const Reference initial(
        point_t::Zero(), 0.0, vector_t::Ones(), 0.4,
        vector_t(2.0, 0.0, 0.0), 1.2
    );
    const KinematicStopLimits limits{0.5, 1.0, 0.75, 1.5};
    const KinematicStopTrajectory trajectory(initial, limits);
    const double dt = trajectory.durationS() / 2000.0;

    Reference previous = trajectory.sample(0.0);
    EXPECT_TRUE(previous.acceleration().isApprox(initial.acceleration(), 1.0e-12));
    EXPECT_DOUBLE_EQ(previous.yaw_acceleration(), initial.yaw_acceleration());
    for (int index = 1; index <= 2000; ++index) {
        const Reference current = trajectory.sample(index * dt);
        EXPECT_LE(current.acceleration().norm(), initial.acceleration().norm() + 1.0e-9);
        EXPECT_LE(
            std::abs(current.yaw_acceleration()),
            std::abs(initial.yaw_acceleration()) + 1.0e-9
        );
        EXPECT_LE(
            ((current.acceleration() - previous.acceleration()) / dt).norm(),
            limits.max_jerk_m_s3 + 2.0e-3
        );
        EXPECT_LE(
            std::abs(
                (current.yaw_acceleration() - previous.yaw_acceleration()) / dt
            ),
            limits.max_yaw_jerk_rad_s3 + 2.0e-3
        );
        previous = current;
    }
}
