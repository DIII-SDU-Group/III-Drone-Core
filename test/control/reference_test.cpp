#include <gtest/gtest.h>

#include <cmath>

#include <iii_drone_core/control/reference.hpp>
#include <iii_drone_core/control/reference_trajectory.hpp>
#include <iii_drone_core/control/state.hpp>

using iii_drone::control::Reference;
using iii_drone::control::ReferenceTrajectory;
using iii_drone::control::State;
using iii_drone::types::point_t;
using iii_drone::types::quaternion_t;
using iii_drone::types::vector_t;

TEST(ReferenceTest, ConstructsFromStateAndPreservesCoreFields)
{
  const auto stamp = rclcpp::Time(123456789LL);
  const State state(
    point_t(1.0f, 2.0f, 3.0f),
    vector_t(4.0f, 5.0f, 6.0f),
    0.75,
    vector_t(0.1f, 0.2f, 0.3f),
    stamp);

  const Reference reference(state);

  EXPECT_FLOAT_EQ(reference.position().x(), 1.0f);
  EXPECT_FLOAT_EQ(reference.position().y(), 2.0f);
  EXPECT_FLOAT_EQ(reference.position().z(), 3.0f);
  EXPECT_NEAR(reference.yaw(), 0.75, 1e-5);
  EXPECT_FLOAT_EQ(reference.velocity().x(), 4.0f);
  EXPECT_FLOAT_EQ(reference.velocity().y(), 5.0f);
  EXPECT_FLOAT_EQ(reference.velocity().z(), 6.0f);
  EXPECT_EQ(reference.stamp().nanoseconds(), stamp.nanoseconds());
}

TEST(ReferenceTest, CopyWithNansOnlyTouchesRequestedFields)
{
  const Reference reference(
    point_t(1.0f, 2.0f, 3.0f),
    1.25,
    vector_t(4.0f, 5.0f, 6.0f),
    0.5,
    vector_t(7.0f, 8.0f, 9.0f),
    0.6,
    rclcpp::Time(42LL));

  const Reference nan_copy = reference.CopyWithNans(true, false);

  EXPECT_TRUE(std::isnan(nan_copy.velocity().x()));
  EXPECT_TRUE(std::isnan(nan_copy.velocity().y()));
  EXPECT_TRUE(std::isnan(nan_copy.velocity().z()));
  EXPECT_TRUE(std::isnan(nan_copy.yaw_rate()));
  EXPECT_FLOAT_EQ(nan_copy.acceleration().x(), 7.0f);
  EXPECT_FLOAT_EQ(nan_copy.acceleration().y(), 8.0f);
  EXPECT_FLOAT_EQ(nan_copy.acceleration().z(), 9.0f);
  EXPECT_DOUBLE_EQ(nan_copy.yaw_acceleration(), 0.6);
  EXPECT_EQ(nan_copy.stamp().nanoseconds(), 42LL);
}

TEST(ReferenceTest, ReferenceTrajectoryConvertsStatesToReferences)
{
  std::vector<State> states;
  states.emplace_back(
    point_t(1.0f, 0.0f, 0.0f),
    vector_t(0.0f, 1.0f, 0.0f),
    0.1,
    vector_t::Zero(),
    rclcpp::Time(10LL));
  states.emplace_back(
    point_t(2.0f, 0.0f, 0.0f),
    vector_t(0.0f, 2.0f, 0.0f),
    0.2,
    vector_t::Zero(),
    rclcpp::Time(20LL));

  const ReferenceTrajectory trajectory(states);

  ASSERT_EQ(trajectory.references().size(), 2U);
  EXPECT_NEAR(trajectory.references()[0].yaw(), 0.1, 1e-5);
  EXPECT_NEAR(trajectory.references()[1].yaw(), 0.2, 1e-5);
  EXPECT_EQ(trajectory.references()[1].stamp().nanoseconds(), 20LL);
}
