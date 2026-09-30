#include <gtest/gtest.h>

#include <chrono>

#include <iii_drone_core/control/hover_thrust_meter.hpp>
#include <iii_drone_core/control/maneuver/cable_push_profile.hpp>

using namespace std::chrono_literals;
using iii_drone::control::HoverThrustMeter;
using iii_drone::control::maneuver::CablePushProfile;

namespace {

// Feeds samples every 20 ms for `duration` and returns the time after them.
HoverThrustMeter::Clock::time_point feed(
    HoverThrustMeter & meter, HoverThrustMeter::Clock::time_point t, std::chrono::milliseconds duration,
    double thrust, double speed, double vertical_speed, bool free_flight) {
    for (auto end = t + duration; t < end; t += 20ms) {
        meter.Add(t, thrust, speed, vertical_speed, free_flight);
    }
    return t;
}

}  // namespace

// In steady free flight the commanded thrust carries the vehicle's weight,
// whatever hover thrust PX4 assumes.
TEST(HoverThrustMeter, MeasuresTheThrustOfSteadyFreeFlight) {
    HoverThrustMeter meter;
    auto t = HoverThrustMeter::Clock::time_point{};
    t = feed(meter, t, 1400ms, 0.70, 0.02, 0.01, true);
    EXPECT_FALSE(meter.estimate());
    t = feed(meter, t, 400ms, 0.70, 0.02, 0.01, true);
    ASSERT_TRUE(meter.estimate());
    EXPECT_NEAR(meter.estimate()->hover_thrust, 0.70, 1e-9);
}

// Moving, climbing, held by the cable (the push) or on the ground: not a
// measurement, and the last measurement is kept.
TEST(HoverThrustMeter, IgnoresMotionTheCableAndIdleAndKeepsTheLastMeasurement) {
    HoverThrustMeter meter;
    auto t = HoverThrustMeter::Clock::time_point{};
    t = feed(meter, t, 2s, 0.70, 0.02, 0.0, true);
    t = feed(meter, t, 3s, 0.91, 0.0, 0.0, false);   // pushing against the cable
    t = feed(meter, t, 3s, 0.80, 0.30, 0.0, true);   // flying along
    t = feed(meter, t, 3s, 0.72, 0.05, 0.20, true);  // climbing
    t = feed(meter, t, 3s, 0.00, 0.0, 0.0, true);    // idle thrust
    ASSERT_TRUE(meter.estimate());
    EXPECT_NEAR(meter.estimate()->hover_thrust, 0.70, 1e-9);
}

TEST(HoverThrustMeter, AGapInSamplesRestartsTheSteadyRun) {
    HoverThrustMeter meter;
    auto t = HoverThrustMeter::Clock::time_point{};
    t = feed(meter, t, 1s, 0.70, 0.0, 0.0, true);
    t += 1s;
    t = feed(meter, t, 1s, 0.70, 0.0, 0.0, true);
    EXPECT_FALSE(meter.estimate());
}

// PX4 realizes a as px4_hover * (1 + a/g); the calibrated push makes that the
// wanted multiple of the measured hover thrust, capped below saturation.
TEST(CablePushCalibration, SizesThePushFromMeasuredAndAssumedHoverThrust) {
    constexpr double g = 9.80665;
    const auto thrust = [&](double px4_hover, double a) { return px4_hover * (1.0 + a / g); };
    // Tuned: PX4 assumes the measured value.
    EXPECT_NEAR(thrust(0.70, CablePushProfile::CalibratedAcceleration(1.3, 0.70, 0.70, 0.95, 0.2)), 0.91, 1e-9);
    // Untuned default: PX4 assumes 0.5 for a vehicle hovering at 0.7.
    EXPECT_NEAR(thrust(0.50, CablePushProfile::CalibratedAcceleration(1.3, 0.70, 0.50, 0.95, 0.2)), 0.91, 1e-9);
    // Over-estimated: fewer m/s^2, the same force.
    EXPECT_NEAR(thrust(0.80, CablePushProfile::CalibratedAcceleration(1.3, 0.70, 0.80, 0.95, 0.2)), 0.91, 1e-9);
    // A heavy vehicle would need more than the cap.
    EXPECT_NEAR(thrust(0.70, CablePushProfile::CalibratedAcceleration(1.3, 0.80, 0.70, 0.95, 0.2)), 0.95, 1e-9);
    // Never below the takeoff request.
    EXPECT_DOUBLE_EQ(CablePushProfile::CalibratedAcceleration(1.3, 0.70, 0.99, 0.95, 0.2), 0.2);
}

TEST(CablePushCalibration, RetargetingRampsToTheNewTargetAndEstablishesThere) {
    CablePushProfile::Limits limits;
    limits.takeoff_request_acceleration_m_s2 = 0.2;
    limits.jerk_m_s3 = 1.0;
    const auto t0 = CablePushProfile::Clock::time_point{};
    CablePushProfile push(3.0, limits, t0);
    push.SetTarget(8.0);
    for (auto t = t0; t <= t0 + 10s; t += 20ms) push.Update(t, CablePushProfile::Px4::kPushing);
    EXPECT_DOUBLE_EQ(push.acceleration(), 8.0);
    EXPECT_TRUE(push.established());
    push.SetTarget(-1.0);
    EXPECT_DOUBLE_EQ(push.target(), 0.2);
}
