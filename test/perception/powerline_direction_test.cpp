#include <memory>

#include <gtest/gtest.h>

#include "iii_drone_core/perception/powerline_direction.hpp"

using iii_drone::perception::PowerlineDirection;

TEST(PowerlineDirectionTest, IdentityFallbackDoesNotClaimMeasuredEstimate)
{
  PowerlineDirection direction(nullptr);

  const auto message = direction.ToQuaternionStampedMsg("drone");

  EXPECT_FALSE(direction.HasEstimate());
  EXPECT_DOUBLE_EQ(message.quaternion.x, 0.0);
  EXPECT_DOUBLE_EQ(message.quaternion.y, 0.0);
  EXPECT_DOUBLE_EQ(message.quaternion.z, 0.0);
  EXPECT_DOUBLE_EQ(message.quaternion.w, 1.0);
}

TEST(PowerlineDirectionTest, ResetLeavesEstimateUnavailable)
{
  PowerlineDirection direction(nullptr);

  direction.Reset();

  EXPECT_FALSE(direction.HasEstimate());
}
