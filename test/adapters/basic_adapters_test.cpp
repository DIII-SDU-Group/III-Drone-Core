#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include <nav_msgs/msg/path.hpp>

#include <iii_drone_core/adapters/projection_plane_adapter.hpp>
#include <iii_drone_core/adapters/reference_adapter.hpp>
#include <iii_drone_core/adapters/reference_trajectory_adapter.hpp>
#include <iii_drone_core/adapters/state_adapter.hpp>
#include <iii_drone_core/adapters/target_adapter.hpp>
#include <iii_drone_core/control/reference.hpp>
#include <iii_drone_core/control/reference_trajectory.hpp>
#include <iii_drone_core/control/state.hpp>
#include <iii_drone_core/utils/math.hpp>
#include <iii_drone_core/utils/types.hpp>

namespace {

constexpr float kTolerance = 1e-4F;
constexpr float kRotationTolerance = 2e-1F;

void ExpectRosTimeMatches(
  const builtin_interfaces::msg::Time & actual,
  const rclcpp::Time & expected)
{
  const int64_t total_nanoseconds = expected.nanoseconds();
  const int32_t expected_sec = static_cast<int32_t>(total_nanoseconds / 1000000000LL);
  const uint32_t expected_nanosec = static_cast<uint32_t>(total_nanoseconds % 1000000000LL);
  EXPECT_EQ(actual.sec, expected_sec);
  EXPECT_EQ(actual.nanosec, expected_nanosec);
}

void ExpectPointNear(
  const iii_drone::types::point_t & actual,
  const iii_drone::types::point_t & expected,
  float tolerance = kTolerance)
{
  EXPECT_NEAR(actual.x(), expected.x(), tolerance);
  EXPECT_NEAR(actual.y(), expected.y(), tolerance);
  EXPECT_NEAR(actual.z(), expected.z(), tolerance);
}

void ExpectVectorNear(
  const iii_drone::types::vector_t & actual,
  const iii_drone::types::vector_t & expected,
  float tolerance = kTolerance)
{
  EXPECT_NEAR(actual.x(), expected.x(), tolerance);
  EXPECT_NEAR(actual.y(), expected.y(), tolerance);
  EXPECT_NEAR(actual.z(), expected.z(), tolerance);
}

void ExpectQuaternionNear(
  const iii_drone::types::quaternion_t & actual,
  const iii_drone::types::quaternion_t & expected,
  float tolerance = kTolerance)
{
  EXPECT_NEAR(actual(0), expected(0), tolerance);
  EXPECT_NEAR(actual(1), expected(1), tolerance);
  EXPECT_NEAR(actual(2), expected(2), tolerance);
  EXPECT_NEAR(actual(3), expected(3), tolerance);
}

void ExpectTransformSemanticsNear(
  const iii_drone::types::transform_matrix_t & actual,
  const iii_drone::types::transform_matrix_t & expected,
  float tolerance = kTolerance)
{
  ExpectVectorNear(actual.block<3, 1>(0, 3), expected.block<3, 1>(0, 3), tolerance);
  EXPECT_TRUE(
    iii_drone::math::quatToMat(iii_drone::types::quaternionFromTransformMatrix(actual))
      .isApprox(
        iii_drone::math::quatToMat(iii_drone::types::quaternionFromTransformMatrix(expected)),
        kRotationTolerance));
}

iii_drone::control::Reference MakeReference(
  const iii_drone::types::point_t & position,
  double yaw,
  const rclcpp::Time & stamp)
{
  return iii_drone::control::Reference(
    position,
    yaw,
    iii_drone::types::vector_t(0.1F, 0.2F, 0.3F),
    0.4,
    iii_drone::types::vector_t(-0.1F, -0.2F, -0.3F),
    0.5,
    stamp);
}

}  // namespace

TEST(BasicAdaptersTest, StateAdapterConvertsStateToMessages)
{
  const rclcpp::Time stamp(1010LL);
  const iii_drone::types::point_t position(1.0F, 2.0F, 3.0F);
  const iii_drone::types::vector_t velocity(-1.0F, 4.0F, 5.0F);
  const iii_drone::types::quaternion_t quaternion =
    iii_drone::math::eulToQuat(iii_drone::types::euler_angles_t(0.0F, 0.0F, 0.75F));
  const iii_drone::types::vector_t angular_velocity(0.5F, -0.5F, 1.5F);

  const iii_drone::adapters::StateAdapter adapter(
    iii_drone::control::State(position, velocity, quaternion, angular_velocity, stamp));

  const auto state_msg = adapter.ToMsg();
  const auto pose_stamped = adapter.ToPoseStampedMsg("map");
  const auto round_trip = iii_drone::adapters::StateAdapter(state_msg).state();

  ExpectPointNear(round_trip.position(), position);
  ExpectVectorNear(round_trip.velocity(), velocity);
  ExpectQuaternionNear(round_trip.quaternion(), quaternion);
  ExpectVectorNear(round_trip.angular_velocity(), angular_velocity);
  EXPECT_EQ(round_trip.stamp().nanoseconds(), stamp.nanoseconds());
  EXPECT_EQ(pose_stamped.header.frame_id, "map");
  ExpectRosTimeMatches(pose_stamped.header.stamp, stamp);
}

TEST(BasicAdaptersTest, ReferenceAdapterConvertsReferenceToMessages)
{
  const rclcpp::Time stamp(2020LL);
  const auto reference = MakeReference(iii_drone::types::point_t(2.0F, -1.0F, 4.0F), 1.25, stamp);
  const iii_drone::adapters::ReferenceAdapter adapter(reference);

  const auto msg = adapter.ToMsg();
  const auto pose = adapter.ToPoseMsg();
  const auto pose_stamped = adapter.ToPoseStampedMsg("odom");
  const auto round_trip = iii_drone::adapters::ReferenceAdapter(msg).reference();

  ExpectPointNear(round_trip.position(), reference.position());
  ExpectVectorNear(round_trip.velocity(), reference.velocity());
  ExpectVectorNear(round_trip.acceleration(), reference.acceleration());
  EXPECT_DOUBLE_EQ(round_trip.yaw(), reference.yaw());
  EXPECT_DOUBLE_EQ(round_trip.yaw_rate(), reference.yaw_rate());
  EXPECT_DOUBLE_EQ(round_trip.yaw_acceleration(), reference.yaw_acceleration());
  EXPECT_EQ(adapter.stamp().nanoseconds(), stamp.nanoseconds());
  EXPECT_EQ(pose_stamped.header.frame_id, "odom");
  ExpectRosTimeMatches(pose_stamped.header.stamp, stamp);
  ExpectPointNear(iii_drone::types::pointFromPoseMsg(pose), reference.position());
}

TEST(BasicAdaptersTest, ReferenceTrajectoryAdapterConvertsTrajectories)
{
  const auto first = MakeReference(iii_drone::types::point_t(0.0F, 0.0F, 1.0F), 0.1, rclcpp::Time(11LL));
  const auto second = MakeReference(iii_drone::types::point_t(1.0F, 2.0F, 3.0F), 0.2, rclcpp::Time(22LL));
  const iii_drone::control::ReferenceTrajectory trajectory({first, second});
  const iii_drone::adapters::ReferenceTrajectoryAdapter adapter(trajectory);

  const auto msg = adapter.ToMsg();
  const auto path = adapter.ToPathMsg("world");
  const auto round_trip =
    iii_drone::adapters::ReferenceTrajectoryAdapter(std::make_shared<iii_drone_interfaces::msg::ReferenceTrajectory>(msg))
      .reference_trajectory();

  ASSERT_EQ(round_trip.references().size(), 2U);
  ExpectPointNear(round_trip.references()[0].position(), first.position());
  ExpectPointNear(round_trip.references()[1].position(), second.position());
  EXPECT_EQ(path.header.frame_id, "world");
  ExpectRosTimeMatches(path.header.stamp, first.stamp());
  ASSERT_EQ(path.poses.size(), 2U);
  ExpectPointNear(iii_drone::types::pointFromPoseMsg(path.poses[1].pose), second.position());

  const iii_drone::adapters::ReferenceTrajectoryAdapter single_reference_adapter(first);
  ASSERT_EQ(single_reference_adapter.reference_trajectory().references().size(), 1U);
}

TEST(BasicAdaptersTest, ProjectionPlaneAdapterUpdatesAndSerializesPlane)
{
  iii_drone_interfaces::msg::ProjectionPlane msg;
  msg.normal.x = 0.0;
  msg.normal.y = 1.0;
  msg.normal.z = 0.0;
  msg.point.x = 4.0;
  msg.point.y = 5.0;
  msg.point.z = 6.0;

  iii_drone::adapters::ProjectionPlaneAdapter adapter;
  adapter.UpdateFromMsg(msg);

  const auto plane = adapter.projection_plane();
  ExpectVectorNear(plane.normal, iii_drone::types::vector_t(0.0F, 1.0F, 0.0F));
  ExpectPointNear(plane.p, iii_drone::types::point_t(4.0F, 5.0F, 6.0F));

  const auto round_trip = adapter.ToMsg();
  EXPECT_FLOAT_EQ(round_trip.point.x, 4.0F);
  EXPECT_FLOAT_EQ(round_trip.normal.y, 1.0F);
}

TEST(BasicAdaptersTest, TargetAdapterSupportsCopyAssignmentAndSerialization)
{
  const auto transform = iii_drone::math::createTransformMatrix(
    iii_drone::types::vector_t(1.0F, 2.0F, 3.0F),
    iii_drone::math::eulToQuat(iii_drone::types::euler_angles_t(0.1F, 0.2F, 0.3F)));

  const iii_drone::adapters::TargetAdapter adapter(
    iii_drone::adapters::TARGET_TYPE_CABLE,
    42,
    "camera",
    transform);

  const auto msg = adapter.ToMsg();
  const iii_drone::adapters::TargetAdapter from_msg(msg);
  iii_drone::adapters::TargetAdapter assigned;
  assigned = from_msg;

  EXPECT_EQ(msg.target_type, iii_drone::adapters::TARGET_TYPE_CABLE);
  EXPECT_EQ(msg.target_id, 42);
  EXPECT_EQ(msg.reference_frame_id, "camera");
  EXPECT_TRUE(assigned == from_msg);
  EXPECT_EQ(from_msg.target_type(), iii_drone::adapters::TARGET_TYPE_CABLE);
  EXPECT_EQ(from_msg.target_id(), 42);
  EXPECT_EQ(from_msg.reference_frame_id(), "camera");
  EXPECT_FALSE(assigned != from_msg);
  ExpectVectorNear(
    iii_drone::types::vectorFromTransformMsg(msg.target_transform),
    transform.block<3, 1>(0, 3));
  ExpectTransformSemanticsNear(
    from_msg.target_transform(),
    iii_drone::types::transformMatrixFromTransformMsg(msg.target_transform));
}
