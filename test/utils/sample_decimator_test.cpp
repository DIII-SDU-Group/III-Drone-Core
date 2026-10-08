#include <gtest/gtest.h>

#include <iii_drone_core/utils/sample_decimator.hpp>

using iii_drone::utils::SampleDecimator;

// The drone pose TF passes every second 100 Hz PX4 odometry sample (50 Hz),
// also with sample jitter of +-0.5 ms.
TEST(SampleDecimator, HalvesA100HzStreamWithJitter) {
    SampleDecimator decimator(15000);
    int passed = 0;
    uint64_t sample_us = 1'000'000;
    for (int i = 0; i < 1000; ++i) {
        sample_us += (i % 2 == 0) ? 9500 : 10500;
        passed += decimator.pass(sample_us) ? 1 : 0;
    }
    EXPECT_EQ(passed, 500);
}

// SITL odometry samples are 8 or 16 ms apart (HIL 2026-10-05). A 19 ms
// threshold needed three 8 ms samples (24-32 ms, 37.5 Hz on average); now
// every second sample passes, at most 24 ms after the previous one (8 + 16).
TEST(SampleDecimator, PassesEverySecondSitlSample) {
    SampleDecimator decimator(15000);
    uint64_t sample_us = 1'000'000;
    uint64_t last_passed_us = sample_us;
    int passed = 0;
    EXPECT_TRUE(decimator.pass(sample_us));
    for (int i = 0; i < 1000; ++i) {
        sample_us += (i % 4 == 3) ? 16000 : 8000;
        if (decimator.pass(sample_us)) {
            EXPECT_LE(sample_us - last_passed_us, 24000u);
            last_passed_us = sample_us;
            ++passed;
        }
    }
    EXPECT_EQ(passed, 500);
}

TEST(SampleDecimator, ASampleTimeThatGoesBackPassesAndRestartsThePeriod) {
    SampleDecimator decimator(15000);
    EXPECT_TRUE(decimator.pass(5'000'000));
    EXPECT_FALSE(decimator.pass(5'010'000));
    EXPECT_TRUE(decimator.pass(1'000));  // PX4 restarted
    EXPECT_FALSE(decimator.pass(11'000));
    EXPECT_TRUE(decimator.pass(21'000));
}
