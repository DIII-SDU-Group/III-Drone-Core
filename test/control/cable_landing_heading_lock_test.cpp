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

// The landing freezes its conductor estimate once the estimate reaches the
// gripper (freeze height), where the offset sensor stops seeing the conductor.
TEST(ConductorEstimateShouldFreezeTest, FreezesAtOrBelowTheFreezeHeightOnly)
{
  using iii_drone::control::maneuver::detail::ConductorEstimateShouldFreeze;
  using iii_drone::types::vector_t;
  EXPECT_TRUE(ConductorEstimateShouldFreeze(vector_t(0.0, 0.0, 0.03), 0.03));
  EXPECT_TRUE(ConductorEstimateShouldFreeze(vector_t(0.0, 0.087, -0.01), 0.03));
  EXPECT_FALSE(ConductorEstimateShouldFreeze(vector_t(0.0, 0.0, 0.031), 0.03));
  EXPECT_FALSE(ConductorEstimateShouldFreeze(vector_t(0.0, 0.0, std::nan("")), 0.03));
}
