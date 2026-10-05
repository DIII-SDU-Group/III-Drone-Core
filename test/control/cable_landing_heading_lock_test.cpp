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
// HIL: holding ascent only at the full V-gate opening let the vehicle reach
// the funnel lip 7 cm off-centre; the gate then failed the landing.
TEST(AscentHoldCrossErrorThresholdTest, HoldsWithAMarginInsideTheGateOpening)
{
  using iii_drone::control::maneuver::detail::AscentHoldCrossErrorThreshold;

  EXPECT_NEAR(AscentHoldCrossErrorThreshold(0.0581), 0.0407, 1.0e-4);
  EXPECT_LT(AscentHoldCrossErrorThreshold(0.046), 0.046);
  EXPECT_DOUBLE_EQ(AscentHoldCrossErrorThreshold(-0.01), 0.0);
}

// HIL: the conductor left the sensor's view 2 cm above the freeze height and
// its mapped estimate jumped ~10 cm before the freeze captured it.
TEST(HoldLastSeenConductorTest, HoldsOnlyNearTheConductorWhenTheSensorLostIt)
{
  using iii_drone::control::maneuver::detail::HoldLastSeenConductor;

  EXPECT_TRUE(HoldLastSeenConductor(true, true, false));
  EXPECT_FALSE(HoldLastSeenConductor(true, true, true));
  EXPECT_FALSE(HoldLastSeenConductor(false, true, false));
  EXPECT_FALSE(HoldLastSeenConductor(true, false, false));
}

// HIL soak run 20: after the loss of view near the gripper the mapper flagged
// the conductor in view again for 50 ms with an 8 cm jump; the estimate must
// stay frozen at the last one seen instead of resuming from that detection.
TEST(FreezeConductorEstimateOnLossOfViewTest, StaysFrozenWhenTheSensorReportsTheConductorAgain)
{
  using iii_drone::control::maneuver::detail::FreezeConductorEstimateOnLossOfView;

  // Approach: in view, or out of view away from the conductor, is not frozen.
  EXPECT_FALSE(FreezeConductorEstimateOnLossOfView(false, true, true, true));
  EXPECT_FALSE(FreezeConductorEstimateOnLossOfView(false, false, true, false));
  EXPECT_FALSE(FreezeConductorEstimateOnLossOfView(false, true, false, false));
  // First loss of view near the conductor freezes.
  EXPECT_TRUE(FreezeConductorEstimateOnLossOfView(false, true, true, false));
  // A later in-view sample does not unfreeze it.
  EXPECT_TRUE(FreezeConductorEstimateOnLossOfView(true, true, true, true));
}

// HIL soak run 22: the estimate froze at the loss of view ~8 cm above the
// gripper, 2.8 cm off a conductor the gripper captured and centred; the gate
// then narrowed to its capture-height width (4.6 cm) and failed the landing.
TEST(VGateHeightTest, HoldsTheWidthWhereTheEstimateFroze)
{
  using iii_drone::control::maneuver::detail::VGateHeight;
  const double nan = std::numeric_limits<double>::quiet_NaN();

  // Not frozen: the gate follows the estimate down.
  EXPECT_DOUBLE_EQ(VGateHeight(-0.01, false, 0.08, 0.03), -0.01);
  // Frozen: the width at the freeze height, not lower.
  EXPECT_DOUBLE_EQ(VGateHeight(-0.01, true, 0.08, 0.03), 0.08);
  EXPECT_DOUBLE_EQ(VGateHeight(0.10, true, 0.08, 0.03), 0.10);
  // Freeze height unknown: the capture height, as before.
  EXPECT_DOUBLE_EQ(VGateHeight(-0.01, true, nan, 0.03), 0.03);
}

TEST(ConductorEstimateShouldFreezeTest, FreezesAtOrBelowTheFreezeHeightOnly)
{
  using iii_drone::control::maneuver::detail::ConductorEstimateShouldFreeze;
  using iii_drone::types::vector_t;
  EXPECT_TRUE(ConductorEstimateShouldFreeze(vector_t(0.0, 0.0, 0.03), 0.03));
  EXPECT_TRUE(ConductorEstimateShouldFreeze(vector_t(0.0, 0.087, -0.01), 0.03));
  EXPECT_FALSE(ConductorEstimateShouldFreeze(vector_t(0.0, 0.0, 0.031), 0.03));
  EXPECT_FALSE(ConductorEstimateShouldFreeze(vector_t(0.0, 0.0, std::nan("")), 0.03));
}

// The line PID's cross velocity integrates into a position reference that
// PX4 tracks with a lag: its error must be the reference's remaining error,
// which reaches zero when the reference is on the conductor line even though
// the lagging vehicle still is not.
TEST(ReferenceCrossErrorTest, ZeroOnceTheReferenceIsOnTheLine)
{
  using iii_drone::control::maneuver::detail::ReferenceCrossError;
  using iii_drone::types::point_t;
  using iii_drone::types::vector_t;
  const vector_t cross_axis(0.0, 1.0, 0.0);
  // Vehicle 8 cm short of the line; reference still at the vehicle.
  EXPECT_NEAR(ReferenceCrossError(0.08, point_t(0, 0, 3), point_t(0, 0, NAN), cross_axis), 0.08, 1e-6);
  // Reference already 8 cm ahead: nothing left for it to move.
  EXPECT_NEAR(ReferenceCrossError(0.08, point_t(0, 0, 3), point_t(0, 0.08, NAN), cross_axis), 0.0, 1e-6);
  // Reference past the line: move it back.
  EXPECT_NEAR(ReferenceCrossError(0.08, point_t(0, 0, 3), point_t(0, 0.11, NAN), cross_axis), -0.03, 1e-6);
  // Along-line lead does not count.
  EXPECT_NEAR(ReferenceCrossError(0.08, point_t(0, 0, 3), point_t(0.5, 0, NAN), cross_axis), 0.08, 1e-6);
}
