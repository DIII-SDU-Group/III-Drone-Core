#include <gtest/gtest.h>

#include <iii_drone_core/utils/timestamp.hpp>

TEST(TimestampTest, DefaultConstructorInitializesStamp)
{
  const iii_drone::utils::Timestamp timestamp;
  const rclcpp::Time stamp = timestamp;

  EXPECT_GT(stamp.nanoseconds(), 0);
}

TEST(TimestampTest, AssignmentFromTimeReplacesStoredStamp)
{
  iii_drone::utils::Timestamp timestamp;
  const rclcpp::Time expected_stamp(123456789LL);

  timestamp = expected_stamp;
  const rclcpp::Time stored_stamp = timestamp;

  EXPECT_EQ(stored_stamp.nanoseconds(), expected_stamp.nanoseconds());
}

TEST(TimestampTest, CopyAssignmentCopiesOtherTimestampValue)
{
  iii_drone::utils::Timestamp lhs;
  iii_drone::utils::Timestamp rhs;
  const rclcpp::Time expected_stamp(987654321LL);

  rhs = expected_stamp;
  lhs = rhs;

  const rclcpp::Time stored_stamp = lhs;
  EXPECT_EQ(stored_stamp.nanoseconds(), expected_stamp.nanoseconds());
}

TEST(TimestampTest, UpdateRefreshesTimestamp)
{
  iii_drone::utils::Timestamp timestamp;
  const rclcpp::Time before_update = timestamp;

  timestamp.Update();
  const rclcpp::Time after_update = timestamp;

  EXPECT_GE(after_update.nanoseconds(), before_update.nanoseconds());
}
