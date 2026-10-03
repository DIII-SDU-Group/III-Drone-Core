#include <gtest/gtest.h>

#define private public
#include <iii_drone_core/control/cable_aware_trajectory_planner.hpp>
#undef private

#include <iii_drone_core/control/maneuver/maneuver_reference_safety_guard.hpp>
#include <iii_drone_core/utils/math.hpp>
#include <iii_drone_core/utils/types.hpp>

namespace {

iii_drone::adapters::PowerlineAdapter MakePowerlineWithDirection(
  const iii_drone::types::vector_t & direction)
{
  const rclcpp::Time stamp(0, 0, RCL_ROS_TIME);
  const auto identity = iii_drone::math::eulToQuat(iii_drone::types::euler_angles_t(0.0F, 0.0F, 0.0F));
  const iii_drone::adapters::SingleLineAdapter line(
    stamp,
    "world",
    1,
    iii_drone::types::point_t(0.0F, 0.0F, 0.0F),
    iii_drone::types::point_t(0.0F, 0.0F, 0.0F),
    identity,
    true);
  return iii_drone::adapters::PowerlineAdapter(
    stamp,
    {line},
    iii_drone::types::createPlane(iii_drone::types::point_t(0.0F, 0.0F, 0.0F), direction));
}

iii_drone::adapters::PowerlineAdapter MakeSingleConductorPowerline(
  const iii_drone::types::point_t & position,
  const iii_drone::types::vector_t & direction)
{
  const rclcpp::Time stamp(0, 0, RCL_ROS_TIME);
  const auto identity = iii_drone::math::eulToQuat(iii_drone::types::euler_angles_t(0.0F, 0.0F, 0.0F));
  const iii_drone::adapters::SingleLineAdapter line(
    stamp,
    "world",
    1,
    position,
    position,
    identity,
    true);
  return iii_drone::adapters::PowerlineAdapter(
    stamp,
    {line},
    iii_drone::types::createPlane(position, direction));
}

}  // namespace

TEST(CableAwareTrajectoryPlannerTest, ProjectionPlaneNormalIsCableDirection)
{
  iii_drone::control::CableAwareTrajectoryPlanner planner(nullptr, nullptr);
  const auto powerline = MakePowerlineWithDirection(iii_drone::types::vector_t(1.0F, 0.0F, 0.0F));
  const auto & line = powerline.single_line_adapters().front();

  EXPECT_NEAR(
    planner.distanceToCable(iii_drone::types::point_t(10.0F, 1.0F, 0.0F), powerline, line),
    1.0,
    1e-5);
  EXPECT_NEAR(
    planner.distanceToCable(iii_drone::types::point_t(0.0F, 1.0F, 0.0F), powerline, line),
    1.0,
    1e-5);
  EXPECT_NEAR(
    planner.distanceToCable(iii_drone::types::point_t(10.0F, 0.0F, 2.0F), powerline, line),
    2.0,
    1e-5);
}

TEST(CableAwareTrajectoryPlannerTest, VerticalCrossingPrefersLocalAStarDetour)
{
  iii_drone::control::CableAwareTrajectoryPlanner planner(nullptr, nullptr);
  const auto powerline = MakeSingleConductorPowerline(
    iii_drone::types::point_t(0.0F, 0.0F, 5.0F),
    iii_drone::types::vector_t(1.0F, 0.0F, 0.0F));

  const iii_drone::types::point_t start(0.0F, 0.0F, 0.0F);
  const iii_drone::types::point_t goal(0.0F, 0.0F, 10.0F);
  const auto path = planner.planAStarPath(start, goal, powerline);

  ASSERT_GE(path.size(), 3u);
  EXPECT_EQ(path.front(), start);
  EXPECT_EQ(path.back(), goal);

  double max_lateral_distance = 0.0;
  for (const auto & point : path) {
    max_lateral_distance = std::max(max_lateral_distance, std::abs(static_cast<double>(point.y())));
    EXPECT_TRUE(planner.pointIsSafeForPlanning(point, powerline, start, goal, false, false));
  }

  EXPECT_LT(max_lateral_distance, 2.0)
    << "The cable-aware planner should use a local A* detour, not the outside-corridor fallback.";

  const rclcpp::Time stamp(100, 0, RCL_ROS_TIME);
  const iii_drone::control::State start_state(
    start,
    iii_drone::types::vector_t::Zero(),
    0.0,
    iii_drone::types::vector_t::Zero(),
    stamp);
  const iii_drone::control::Reference goal_reference(goal, 0.0);
  const auto trajectory = planner.buildPiecewiseLinearTrajectory(path, start_state, goal_reference);
  EXPECT_TRUE(planner.trajectoryIsSafe(
    trajectory, powerline, start, goal, false, false))
    << "Piecewise fallback must preserve every validated A* edge without cutting corners.";
}

TEST(CableAwareTrajectoryPlannerTest, PlannerFallsBackWhenSmoothingMissesTerminalContract)
{
  iii_drone::control::CableAwareTrajectoryPlanner planner(nullptr, nullptr);
  const rclcpp::Time stamp(100, 0, RCL_ROS_TIME);
  const iii_drone::types::point_t start(-1.03F, -1.35F, 0.82F);
  const iii_drone::types::point_t goal(-2.60F, -11.72F, 11.13F);
  const iii_drone::control::State start_state(
    start,
    iii_drone::types::vector_t(0.12F, -0.08F, 0.03F),
    -1.0,
    iii_drone::types::vector_t::Zero(),
    stamp);
  const iii_drone::control::Reference goal_reference(goal, -1.81);
  const std::vector<iii_drone::types::point_t> waypoints{
    start,
    iii_drone::types::point_t(-1.0F, -6.7F, 6.1F),
    iii_drone::types::point_t(-2.5F, -11.6F, 11.0F),
    goal,
  };

  const auto smoothed = planner.smoothPathLeastSquares(waypoints, start_state, goal_reference);
  ASSERT_FALSE(planner.trajectoryMeetsBoundaryContract(smoothed, start_state, goal_reference));

  const auto trajectory = planner.buildPiecewiseLinearTrajectory(waypoints, start_state, goal_reference);
  ASSERT_TRUE(planner.trajectoryMeetsBoundaryContract(trajectory, start_state, goal_reference));
  ASSERT_FALSE(trajectory.references().empty());
  const auto & first = trajectory.references().front();
  const auto & last = trajectory.references().back();

  EXPECT_LT((first.position() - start).norm(), 1.0e-6);
  EXPECT_LT((first.velocity() - start_state.velocity()).norm(), 1.0e-6);
  EXPECT_LT(first.acceleration().norm(), 1.0e-6);
  EXPECT_LT((last.position() - goal).norm(), 1.0e-6);
  EXPECT_LT(last.velocity().norm(), 1.0e-6);
  EXPECT_LT(last.acceleration().norm(), 1.0e-6);
}

TEST(CableAwareTrajectoryPlannerTest, SamplingAtFractionalDurationReturnsExactTerminalReference)
{
  iii_drone::control::CableAwareTrajectoryPlanner planner(nullptr, nullptr);
  const rclcpp::Time stamp(100, 0, RCL_ROS_TIME);
  const iii_drone::types::point_t start(0.0F, 0.0F, 0.0F);
  const iii_drone::types::point_t goal(1.02F, 0.0F, 0.0F);
  const iii_drone::control::State start_state(
    start,
    iii_drone::types::vector_t::Zero(),
    0.0,
    iii_drone::types::vector_t::Zero(),
    stamp);
  const iii_drone::control::Reference goal_reference(goal, 0.0);

  planner.active_trajectory_ = planner.buildPiecewiseLinearTrajectory(
    {start, goal}, start_state, goal_reference);
  // Rate-limited start and stop make the duration longer than 1.02 m / 0.5 m/s.
  ASSERT_GT(planner.duration_s_, 1.02 / 0.5);

  const auto sampled = planner.sampleActiveTrajectory(planner.duration_s_);
  ASSERT_FALSE(sampled.references().empty());
  for (const auto & reference : sampled.references()) {
    EXPECT_LT((reference.position() - goal).norm(), 1.0e-6);
    EXPECT_LT(reference.velocity().norm(), 1.0e-6);
    EXPECT_LT(reference.acceleration().norm(), 1.0e-6);
  }
}

TEST(CableAwareTrajectoryPlannerTest, PiecewiseFallbackPassesConsumerContinuityFromMeasuredStart)
{
  // SIM ingress: the start violates clearance, LLS smoothing is rejected and
  // the piecewise-linear A* fallback is streamed after a Reference(state)
  // hold whose measured velocity points slightly against the path.
  iii_drone::control::CableAwareTrajectoryPlanner planner(nullptr, nullptr);
  const rclcpp::Time stamp(100, 0, RCL_ROS_TIME);
  const iii_drone::types::point_t start(0.0F, 0.0F, 1.0F);
  const iii_drone::types::point_t goal(3.0F, 3.0F, 1.0F);
  const iii_drone::control::State start_state(
    start,
    iii_drone::types::vector_t(-0.08F, 0.03F, 0.0F),
    0.0,
    iii_drone::types::vector_t::Zero(),
    stamp);
  const iii_drone::control::Reference goal_reference(goal, 0.5);
  const std::vector<iii_drone::types::point_t> waypoints{
    start,
    iii_drone::types::point_t(1.0F, 0.0F, 1.0F),
    iii_drone::types::point_t(3.0F, 0.0F, 1.0F),
    goal,
  };
  const auto trajectory = planner.buildPiecewiseLinearTrajectory(waypoints, start_state, goal_reference);
  ASSERT_TRUE(planner.trajectoryMeetsBoundaryContract(trajectory, start_state, goal_reference));

  // Each trajectory sample is 0.2 s apart, the 5 Hz consumer cadence.
  iii_drone::control::maneuver::ManeuverReferenceSafetyGuard guard(
    iii_drone::control::maneuver::ManeuverReferenceSafetyConfig{});
  auto received = std::chrono::steady_clock::time_point{} + std::chrono::seconds(10);
  ASSERT_EQ(guard.observeReference(iii_drone::control::Reference(start_state), received).decision,
    iii_drone::control::maneuver::ManeuverReferenceSafetyDecision::ACCEPT);
  const auto & references = trajectory.references();
  for (std::size_t i = 1; i < references.size(); ++i) {
    received += std::chrono::milliseconds(200);
    const auto evaluation = guard.observeReference(references[i], received);
    ASSERT_EQ(evaluation.decision,
      iii_drone::control::maneuver::ManeuverReferenceSafetyDecision::ACCEPT)
      << "sample " << i << ": " << evaluation.reason
      << " velocity " << evaluation.velocity_error_m_s << "/" << evaluation.velocity_limit_m_s
      << " position " << evaluation.position_error_m << "/" << evaluation.position_limit_m;
  }
  for (std::size_t i = 1; i < references.size(); ++i) {
    EXPECT_LE(references[i].velocity().norm(), 0.5 + 1.0e-6);
  }
}
