#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include <iii_drone_core/control/estimated_position_stop_proof.hpp>

using iii_drone::control::ControlledCancellationConfig;
using iii_drone::control::EstimatedPositionStopProof;
using iii_drone::control::MeasuredOdometrySnapshot;
using iii_drone::control::State;
using iii_drone::types::point_t;
using iii_drone::types::vector_t;

namespace {
rclcpp::Time stamp(int milliseconds) {
    return rclcpp::Time(static_cast<int64_t>(milliseconds) * 1000000, RCL_ROS_TIME);
}
MeasuredOdometrySnapshot measured(int milliseconds, point_t position = point_t::Zero(),
    double velocity_bias = 0.24, double yaw_rate = 0.0, uint8_t reset_counter = 0) {
    return {
        State(position, vector_t(0.0, 0.0, velocity_bias), 0.0,
            vector_t(0.0, 0.0, yaw_rate), stamp(milliseconds)),
        stamp(milliseconds), static_cast<uint64_t>(milliseconds) * 1000, reset_counter
    };
}
}  // namespace

TEST(EstimatedPositionStopProofTest, StationaryJitterWithVelocityBiasNeedsFullWindowAndDwell) {
    EstimatedPositionStopProof proof;
    ControlledCancellationConfig config;
    for (int ms = 1000; ms < 2200; ms += 50) {
        EXPECT_FALSE(proof.observe(true, measured(ms, point_t(0.0002 * std::sin(ms), 0, 0)),
            config, stamp(ms)));
    }
    EXPECT_TRUE(proof.observe(true, measured(2200, point_t(0.0002 * std::sin(2200), 0, 0)),
        config, stamp(2200)));
    ASSERT_TRUE(proof.pathSpeedMS());
    EXPECT_LT(*proof.pathSpeedMS(), config.velocity_threshold_m_s);
}

TEST(EstimatedPositionStopProofTest, DetectsRealMotionEvenWithZeroReportedVelocity) {
    for (const double speed : {0.09, 0.15}) {
        EstimatedPositionStopProof proof;
        ControlledCancellationConfig config;
        for (int ms = 1000; ms <= 4000; ms += 50) {
            EXPECT_FALSE(proof.observe(true,
                measured(ms, point_t(0, 0, speed * (ms - 1000) / 1000.0), 0.0),
                config, stamp(ms)));
        }
        ASSERT_TRUE(proof.pathSpeedMS());
        EXPECT_NEAR(*proof.pathSpeedMS(), speed, 1.0e-7);
    }
}

TEST(EstimatedPositionStopProofTest, RejectsBackAndForthMotionWithZeroNetDisplacement) {
    EstimatedPositionStopProof proof;
    ControlledCancellationConfig config;
    for (int ms = 1000; ms <= 4000; ms += 50) {
        // 0.12 m/s triangle motion, 1 s period, identical ends of each window.
        const int phase_ms = (ms - 1000) % 1000;
        const double x = 0.12 * std::min(phase_ms, 1000 - phase_ms) / 1000.0;
        EXPECT_FALSE(proof.observe(true, measured(ms, point_t(x, 0, 0), 0.0), config, stamp(ms)));
    }
    ASSERT_TRUE(proof.pathSpeedMS());
    EXPECT_NEAR(*proof.pathSpeedMS(), 0.12, 1.0e-7);
}

TEST(EstimatedPositionStopProofTest, DuplicateSamplesCannotAdvanceProofAndLongGapsResetIt) {
    EstimatedPositionStopProof proof;
    ControlledCancellationConfig config;
    for (int ms = 1000; ms <= 2000; ms += 50)
        EXPECT_FALSE(proof.observe(true, measured(ms), config, stamp(ms)));
    for (int ms = 2010; ms <= 2300; ms += 10) {
        auto duplicate = measured(2000);
        duplicate.receipt_stamp = stamp(ms);  // Re-publication is not new measurement.
        EXPECT_FALSE(proof.observe(true, duplicate, config, stamp(ms)));
    }
    // A stale sample cannot certify; a gap longer than the odometry gap bound
    // (0.5 s) then restarts the dwell.
    EXPECT_FALSE(proof.observe(true, measured(2000), config, stamp(2301)));
    // The sample ending the 0.55 s gap resets; the dwell restarts after it.
    for (int ms = 2550; ms < 3800; ms += 50)
        EXPECT_FALSE(proof.observe(true, measured(ms), config, stamp(ms)));
    EXPECT_TRUE(proof.observe(true, measured(3800), config, stamp(3800)));
}

// HIL: PX4 odometry occasionally resumes after a ~0.3 s gap. A stale
// evaluation during the gap does not certify, and the ended gap does not
// restart the one-second dwell.
TEST(EstimatedPositionStopProofTest, EndedOdometryGapWithinBoundDoesNotRestartTheDwell) {
    EstimatedPositionStopProof proof;
    ControlledCancellationConfig config;
    for (int ms = 1000; ms <= 1900; ms += 50)
        EXPECT_FALSE(proof.observe(true, measured(ms), config, stamp(ms)));
    EXPECT_FALSE(proof.observe(true, measured(1900), config, stamp(2160)));  // stale, no new sample
    // The 0.3 s gap ended: the window (from 1.0 s) is complete and the
    // 0.2 s settle certifies at 2.4 s; a restarted dwell would need until 3.4 s.
    for (int ms = 2200; ms < 2400; ms += 50)
        EXPECT_FALSE(proof.observe(true, measured(ms), config, stamp(ms)));
    EXPECT_TRUE(proof.observe(true, measured(2400), config, stamp(2400)));
}

TEST(EstimatedPositionStopProofTest, InvalidGatesGapsResetsAndClocksDiscardTheFullHistory) {
    ControlledCancellationConfig config;
    for (int defect = 0; defect < 9; ++defect) {
        EstimatedPositionStopProof proof;
        for (int ms = 1000; ms <= 2100; ms += 50)
            ASSERT_FALSE(proof.observe(true, measured(ms), config, stamp(ms)));
        auto bad = measured(2150);
        bool gate = true;
        auto now = stamp(2150);
        switch (defect) {
            case 0: gate = false; break;
            case 1: bad.reset_counter = 1; break;
            case 2: bad.source_sample_timestamp_us = 2000000; break;
            case 3: bad.source_sample_timestamp_us = 2700000; break;  // > 0.5 s gap
            case 4: bad.receipt_stamp = stamp(1800); break;
            case 5: bad = measured(2150, point_t(NAN, 0, 0)); break;
            case 6: bad = measured(2150, point_t::Zero(), INFINITY); break;
            case 7: bad = measured(2150, point_t::Zero(), 0.24, 0.09); break;
            case 8: now = rclcpp::Time(2150000000LL, RCL_SYSTEM_TIME); break;
        }
        EXPECT_FALSE(proof.observe(gate, bad, config, now)) << defect;
        for (int ms = 2200; ms < 3400; ms += 50)
            EXPECT_FALSE(proof.observe(true, measured(ms), config, stamp(ms))) << defect;
        EXPECT_TRUE(proof.observe(true, measured(3400), config, stamp(3400))) << defect;
    }
}

TEST(EstimatedPositionStopProofTest, UsesActualThreeDimensionalTravelAndExactRollingBoundary) {
    EstimatedPositionStopProof proof;
    ControlledCancellationConfig config;
    const vector_t direction = vector_t(1.0, 2.0, 3.0).normalized();
    for (int ms = 1000; ms <= 4000; ms += 70) {
        const point_t position = direction * (0.079 * (ms - 1000) / 1000.0);
        const bool stopped = proof.observe(true, measured(ms, position, 0), config, stamp(ms));
        if (ms >= 2300) { EXPECT_TRUE(stopped); }
        if (proof.pathSpeedMS()) { EXPECT_NEAR(*proof.pathSpeedMS(), 0.079, 1.0e-7); }
    }
}
