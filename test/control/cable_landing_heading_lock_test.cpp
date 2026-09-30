#include <gtest/gtest.h>

#include <limits>

#include <iii_drone_core/control/maneuver/cable_landing_maneuver_server.hpp>

namespace {

double shortestYawDelta(double from, double to)
{
  return std::atan2(std::sin(to - from), std::cos(to - from));
}

}  // namespace

TEST(CableLandingHeadingLockTest, KeepsCapturedWorldHeadingAcrossLiveInputChanges)
{
  iii_drone::control::maneuver::detail::CableLandingHeadingLock lock;
  ASSERT_TRUE(lock.capture(3.12, -3.13));
  ASSERT_TRUE(lock.locked());

  const double captured_yaw = lock.yawWorld();
  const auto captured_direction = lock.directionWorld();
  for (const double changed_live_yaw : {-2.8, -1.2, 0.4, 2.7}) {
    ASSERT_TRUE(lock.capture(changed_live_yaw, 0.0));
    EXPECT_NEAR(shortestYawDelta(captured_yaw, lock.yawWorld()), 0.0, 1e-12);
    EXPECT_NEAR((captured_direction - lock.directionWorld()).norm(), 0.0, 1e-12);
  }
}

TEST(CableLandingHeadingLockTest, ChoosesNearestPiEquivalentAndResetsForNextExecution)
{
  iii_drone::control::maneuver::detail::CableLandingHeadingLock lock;
  const double pi = std::acos(-1.0);
  ASSERT_TRUE(lock.capture(pi - 0.1, 0.05));
  EXPECT_NEAR(shortestYawDelta(0.05, lock.yawWorld()), -0.15, 1e-7);

  lock.reset();
  EXPECT_FALSE(lock.locked());
  ASSERT_TRUE(lock.capture(0.8, 0.7));
  EXPECT_NEAR(shortestYawDelta(0.7, lock.yawWorld()), 0.1, 2e-8);
}

TEST(CableLandingHeadingLockTest, RejectsNonFiniteInitialHeading)
{
  iii_drone::control::maneuver::detail::CableLandingHeadingLock lock;
  EXPECT_FALSE(lock.capture(std::numeric_limits<double>::quiet_NaN(), 0.0));
  EXPECT_FALSE(lock.locked());
}

// Once the conductor has entered the V gate at the capture height the gripper
// holds it; above that height, or outside the gate, it is not captured.
// SIM gate: apex -0.16 m, half width 0.18 m at 0.03 m; capture at 0.03 m.
TEST(ConductorCapturedByGripperTest, CapturedOnlyInsideTheGateAtOrBelowTheCaptureHeight)
{
  using iii_drone::control::maneuver::detail::ConductorCapturedByGripper;
  using iii_drone::types::vector_t;
  const auto captured = [](double y, double z) {
    return ConductorCapturedByGripper(vector_t(0.0, y, z), 0.03, -0.16, 0.03, 0.18, 0.0);
  };
  EXPECT_TRUE(captured(0.0, 0.03));
  EXPECT_TRUE(captured(0.10, 0.0));      // gate allows 0.152 at z = 0
  EXPECT_FALSE(captured(0.0, 0.031));    // still above the capture height
  EXPECT_FALSE(captured(0.16, 0.0));     // outside the gate
  EXPECT_FALSE(captured(0.0, -0.17));    // below the apex
  EXPECT_FALSE(ConductorCapturedByGripper(vector_t(0.0, 0.0, 0.0), 0.03, 0.05, 0.03, 0.18, 0.0));  // invalid gate
}
