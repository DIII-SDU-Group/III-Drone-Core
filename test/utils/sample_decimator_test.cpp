#include <gtest/gtest.h>

#include <iii_drone_core/utils/sample_decimator.hpp>

using iii_drone::utils::SampleDecimator;

// The drone pose TF passes every second 100 Hz PX4 odometry sample (50 Hz),
// also with sample jitter of +-0.5 ms.
TEST(SampleDecimator, HalvesA100HzStreamWithJitter) {
    SampleDecimator decimator(19000);
    int passed = 0;
    uint64_t sample_us = 1'000'000;
    for (int i = 0; i < 1000; ++i) {
        sample_us += (i % 2 == 0) ? 9500 : 10500;
        passed += decimator.pass(sample_us) ? 1 : 0;
    }
    EXPECT_EQ(passed, 500);
}

TEST(SampleDecimator, ASampleTimeThatGoesBackPassesAndRestartsThePeriod) {
    SampleDecimator decimator(19000);
    EXPECT_TRUE(decimator.pass(5'000'000));
    EXPECT_FALSE(decimator.pass(5'010'000));
    EXPECT_TRUE(decimator.pass(1'000));  // PX4 restarted
    EXPECT_FALSE(decimator.pass(11'000));
    EXPECT_TRUE(decimator.pass(21'000));
}
