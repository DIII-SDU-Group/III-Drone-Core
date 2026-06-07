#include <gtest/gtest.h>

#define private public
#include <iii_drone_core/control/cable_aware_trajectory_planner.hpp>
#undef private

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
}
