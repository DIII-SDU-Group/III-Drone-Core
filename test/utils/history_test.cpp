#include <gtest/gtest.h>

#include <iii_drone_core/utils/history.hpp>

using iii_drone::utils::History;

TEST(HistoryTest, StoresOnlyLastCapacityValues)
{
  History<int> history(3);

  history.Store(1);
  history.Store(2);
  history.Store(3);
  history.Store(4);

  const auto values = history.Load();

  ASSERT_EQ(values.size(), 3U);
  EXPECT_EQ(values[0], 2);
  EXPECT_EQ(values[1], 3);
  EXPECT_EQ(values[2], 4);
  EXPECT_EQ(history(), 4);
  EXPECT_EQ(history[0], 4);
  EXPECT_EQ(history[-1], 3);
  EXPECT_EQ(history[-2], 2);
  EXPECT_THROW(static_cast<void>(history[-3]), std::runtime_error);
}

TEST(HistoryTest, AssignmentFromVectorKeepsMostRecentValues)
{
  History<int> history(2);

  history = std::vector<int>{1, 2, 3};

  const auto values = history.Load();

  ASSERT_EQ(values.size(), 2U);
  EXPECT_EQ(values[0], 2);
  EXPECT_EQ(values[1], 3);
}

TEST(HistoryTest, EmptyHistoryThrowsOnRead)
{
  History<int> history(1);

  EXPECT_THROW(static_cast<void>(history.Load()), std::runtime_error);
  EXPECT_THROW(static_cast<void>(history()), std::runtime_error);
}
