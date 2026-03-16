#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>
#include <vector>

#include <iii_drone_core/adapters/point_cloud_adapter.hpp>
#include <iii_drone_core/adapters/powerline_adapter.hpp>
#include <iii_drone_core/adapters/projection_plane_adapter.hpp>
#include <iii_drone_core/adapters/single_line_adapter.hpp>
#include <iii_drone_core/utils/math.hpp>
#include <iii_drone_core/utils/types.hpp>

namespace {

constexpr float kTolerance = 1e-4F;

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

iii_drone::adapters::SingleLineAdapter MakeLine(
  int id,
  const iii_drone::types::point_t & position,
  const iii_drone::types::point_t & projected_position,
  bool in_fov,
  const rclcpp::Time & stamp = rclcpp::Time(100LL))
{
  return iii_drone::adapters::SingleLineAdapter(
    stamp,
    "line_frame",
    id,
    position,
    projected_position,
    iii_drone::math::eulToQuat(iii_drone::types::euler_angles_t(0.0F, 0.0F, 0.25F)),
    in_fov);
}

}  // namespace

TEST(PerceptionAdaptersTest, PointCloudAdapterRoundTripsPointCloud)
{
  const std::vector<iii_drone::types::point_t> points = {
    iii_drone::types::point_t(1.0F, 2.0F, 3.0F),
    iii_drone::types::point_t(-4.0F, 5.5F, 6.5F),
  };
  const rclcpp::Time stamp(777LL);
  const iii_drone::adapters::PointCloudAdapter adapter(stamp, "sensor", points);

  const auto msg = adapter.ToMsg();
  const iii_drone::adapters::PointCloudAdapter round_trip(
    std::make_shared<sensor_msgs::msg::PointCloud2>(msg));

  EXPECT_EQ(msg.header.frame_id, "sensor");
  ExpectRosTimeMatches(msg.header.stamp, stamp);
  EXPECT_EQ(msg.width, points.size());
  ASSERT_EQ(round_trip.points().size(), points.size());
  ExpectPointNear(round_trip.points()[0], points[0]);
  ExpectPointNear(round_trip.points()[1], points[1]);
}

TEST(PerceptionAdaptersTest, SingleLineAdapterSerializesConstructorsConsistently)
{
  const rclcpp::Time stamp(888LL);
  const iii_drone::types::point_t position(1.0F, 2.0F, 3.0F);
  const iii_drone::types::point_t projected(1.5F, 2.5F, 0.0F);
  const auto quaternion =
    iii_drone::math::eulToQuat(iii_drone::types::euler_angles_t(0.1F, 0.0F, 0.2F));

  const iii_drone::adapters::SingleLineAdapter adapter(
    stamp,
    "map",
    7,
    position,
    projected,
    quaternion,
    true);

  const auto msg = adapter.ToMsg();
  const iii_drone::adapters::SingleLineAdapter from_msg(msg);

  ExpectPointNear(from_msg.position(), position);
  ExpectPointNear(from_msg.projected_position(), projected);
  ExpectQuaternionNear(from_msg.quaternion(), quaternion);
  EXPECT_EQ(from_msg.id(), 7);
  EXPECT_TRUE(from_msg.in_fov());

  geometry_msgs::msg::Pose pose_msg = adapter.ToPoseMsg();
  const iii_drone::adapters::SingleLineAdapter from_pose(stamp, "map", 8, pose_msg, false);
  ExpectPointNear(from_pose.position(), position);
  ExpectPointNear(from_pose.projected_position(), position);

  geometry_msgs::msg::PoseStamped pose_stamped = adapter.ToPoseStampedMsg();
  const iii_drone::adapters::SingleLineAdapter from_pose_stamped(9, pose_stamped, true);
  ExpectPointNear(from_pose_stamped.position(), position);
  EXPECT_EQ(from_pose_stamped.frame_id(), "map");
}

TEST(PerceptionAdaptersTest, PowerlineAdapterFiltersFindsAndSerializesLines)
{
  const auto closest_line = MakeLine(1, iii_drone::types::point_t(0.0F, 0.0F, 0.0F), iii_drone::types::point_t(0.0F, 0.0F, 0.0F), true, rclcpp::Time(50LL));
  const auto hidden_line = MakeLine(2, iii_drone::types::point_t(4.0F, 0.0F, 0.0F), iii_drone::types::point_t(4.0F, 0.0F, 0.0F), false, rclcpp::Time(60LL));
  const auto far_line = MakeLine(3, iii_drone::types::point_t(2.0F, 2.0F, 0.0F), iii_drone::types::point_t(2.0F, 2.0F, 0.0F), true, rclcpp::Time(40LL));
  const auto plane = iii_drone::types::createPlane(
    iii_drone::types::point_t(0.0F, 0.0F, 1.0F),
    iii_drone::types::vector_t(0.0F, 0.0F, 1.0F));

  const iii_drone::adapters::PowerlineAdapter adapter(
    rclcpp::Time(90LL),
    {closest_line, hidden_line, far_line},
    plane);

  const auto visible = adapter.GetVisibleLineAdapters();
  ASSERT_EQ(visible.size(), 2U);
  EXPECT_TRUE(adapter.HasLine(2));
  EXPECT_FALSE(adapter.HasLine(99));
  EXPECT_EQ(adapter.GetLine(3).id(), 3);
  EXPECT_EQ(adapter.GetClosestLine(iii_drone::types::point_t(0.2F, 0.1F, 0.0F)).id(), 1);

  const auto points = adapter.GetPoints();
  ASSERT_EQ(points.size(), 3U);
  ExpectPointNear(points[1], hidden_line.position());

  const auto msg = adapter.ToMsg();
  const iii_drone::adapters::PowerlineAdapter from_msg(msg);
  EXPECT_EQ(from_msg.stamp().nanoseconds(), 40LL);
  ASSERT_EQ(from_msg.single_line_adapters().size(), 3U);
  EXPECT_EQ(from_msg.projection_plane().p.z(), 1.0F);
}

TEST(PerceptionAdaptersTest, PowerlineAdapterThrowsForMissingLines)
{
  const iii_drone::adapters::PowerlineAdapter empty_adapter;

  EXPECT_THROW(empty_adapter.GetClosestLine(iii_drone::types::point_t::Zero()), std::runtime_error);
  EXPECT_THROW(empty_adapter.GetLine(123), std::runtime_error);
}
