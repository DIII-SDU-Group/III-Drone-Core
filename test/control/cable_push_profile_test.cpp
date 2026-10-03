#include <gtest/gtest.h>

#include <chrono>
#include <stdexcept>

#include <iii_drone_core/control/maneuver/cable_push_profile.hpp>

using namespace std::chrono_literals;
using iii_drone::control::maneuver::CablePushProfile;
using Px4 = CablePushProfile::Px4;

namespace {

CablePushProfile::Limits limits() {
    CablePushProfile::Limits limits;
    limits.takeoff_request_acceleration_m_s2 = 0.2;
    limits.jerk_m_s3 = 1.0;
    limits.start_timeout = 8s;
    return limits;
}

}  // namespace

// Far above the ground PX4 reports airborne as soon as it arms while its
// takeoff state machine still commands zero thrust: only airborne with thrust
// means PX4 applies the push.
TEST(CablePushProfile, Px4AppliesThePushOnlyWhenAirborneWithThrust) {
    EXPECT_EQ(CablePushProfile::Classify(true, true, 0.7), Px4::kPushing);
    EXPECT_EQ(CablePushProfile::Classify(true, true, 0.0), Px4::kNotPushing);
    EXPECT_EQ(CablePushProfile::Classify(true, false, 0.7), Px4::kNotPushing);
    EXPECT_EQ(CablePushProfile::Classify(false, true, 0.7), Px4::kUnknown);
}

// The push only requests takeoff until PX4 applies it and ramps from that
// moment, not from the maneuver start or the last sample before it.
TEST(CablePushProfile, RequestsTakeoffUntilPx4PushesThenRampsAtBoundedJerk) {
    const auto t0 = CablePushProfile::Clock::time_point{};
    CablePushProfile push(3.0, limits(), t0);

    EXPECT_DOUBLE_EQ(push.Update(t0, Px4::kNotPushing), 0.2);
    EXPECT_DOUBLE_EQ(push.Update(t0 + 2s, Px4::kNotPushing), 0.2);
    EXPECT_FALSE(push.established());
    EXPECT_FALSE(push.failed(t0 + 2s));

    EXPECT_DOUBLE_EQ(push.Update(t0 + 2500ms, Px4::kPushing), 0.2);
    EXPECT_NEAR(push.Update(t0 + 3500ms, Px4::kPushing), 1.2, 1e-9);
    EXPECT_FALSE(push.established());
    double previous = push.acceleration();
    for (auto t = t0 + 3520ms; t <= t0 + 7s; t += 20ms) {
        const double acceleration = push.Update(t, Px4::kPushing);
        EXPECT_LE(acceleration - previous, 1.0 * 0.020 + 1e-9);
        EXPECT_LE(acceleration, 3.0);
        previous = acceleration;
    }
    EXPECT_DOUBLE_EQ(push.acceleration(), 3.0);
    EXPECT_TRUE(push.established());
    EXPECT_FALSE(push.failed(t0 + 7s));
}

// Stale PX4 samples neither ramp nor fail the push; the ramp resumes only
// with fresh evidence, and the push is not established meanwhile.
TEST(CablePushProfile, UnknownPx4StateHoldsThePush) {
    const auto t0 = CablePushProfile::Clock::time_point{};
    CablePushProfile push(3.0, limits(), t0);
    push.Update(t0 + 1s, Px4::kPushing);
    push.Update(t0 + 2s, Px4::kPushing);
    EXPECT_NEAR(push.acceleration(), 1.2, 1e-9);
    push.Update(t0 + 3s, Px4::kUnknown);
    push.Update(t0 + 4s, Px4::kUnknown);
    EXPECT_NEAR(push.acceleration(), 1.2, 1e-9);
    EXPECT_FALSE(push.failed(t0 + 4s));
    push.Update(t0 + 4500ms, Px4::kPushing);
    EXPECT_NEAR(push.acceleration(), 1.2, 1e-9);
    push.Update(t0 + 5500ms, Px4::kPushing);
    EXPECT_NEAR(push.acceleration(), 2.2, 1e-9);
}

TEST(CablePushProfile, FailsWhenPx4NeverAppliesThePush) {
    const auto t0 = CablePushProfile::Clock::time_point{};
    CablePushProfile push(3.0, limits(), t0);
    push.Update(t0 + 7900ms, Px4::kNotPushing);
    EXPECT_FALSE(push.failed(t0 + 7900ms));
    push.Update(t0 + 8100ms, Px4::kUnknown);
    EXPECT_TRUE(push.failed(t0 + 8100ms));
    EXPECT_FALSE(push.pushingLost());
    EXPECT_DOUBLE_EQ(push.acceleration(), 0.2);
}

// Once PX4 applied the push, PX4 reporting landed or dropping thrust means it
// may cut thrust or disarm: the push must fail rather than let the gripper
// open.
TEST(CablePushProfile, FailsWhenPx4StopsApplyingThePush) {
    const auto t0 = CablePushProfile::Clock::time_point{};
    CablePushProfile push(3.0, limits(), t0);
    push.Update(t0 + 1s, Px4::kPushing);
    push.Update(t0 + 5s, Px4::kPushing);
    ASSERT_TRUE(push.established());
    push.Update(t0 + 5100ms, Px4::kNotPushing);
    EXPECT_FALSE(push.established());
    EXPECT_TRUE(push.pushingLost());
    EXPECT_TRUE(push.failed(t0 + 5100ms));
    push.Update(t0 + 5200ms, Px4::kPushing);
    EXPECT_FALSE(push.established());
    EXPECT_TRUE(push.failed(t0 + 5200ms));
}

TEST(CablePushProfile, RejectsInvalidTargetsAndLimits) {
    const auto t0 = CablePushProfile::Clock::time_point{};
    EXPECT_THROW(CablePushProfile(0.0, limits(), t0), std::invalid_argument);
    EXPECT_THROW(CablePushProfile(0.1, limits(), t0), std::invalid_argument);
    auto no_jerk = limits();
    no_jerk.jerk_m_s3 = 0.0;
    EXPECT_THROW(CablePushProfile(3.0, no_jerk, t0), std::invalid_argument);
}
