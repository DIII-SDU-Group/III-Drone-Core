#include <gtest/gtest.h>
#include <chrono>
#include <cmath>
#include <limits>

#include <iii_drone_core/control/combined_drone_awareness_handler.hpp>

using iii_drone::adapters::px4::VehicleOdometryAdapter;
using iii_drone::control::CombinedDroneAwarenessHandler;

TEST(MeasuredOdometrySnapshot, PublicationTimestampDoesNotRefreshAnOldSample) {
    px4_msgs::msg::VehicleOdometry message;
    message.pose_frame = iii_drone::adapters::px4::POSE_FRAME_LOCAL_NED;
    message.velocity_frame = iii_drone::adapters::px4::VELOCITY_FRAME_LOCAL_NED;
    message.q[0] = 1.0F;
    message.timestamp_sample = 1000;
    message.timestamp = 1100;
    auto snapshot = CombinedDroneAwarenessHandler::AdvanceMeasuredOdometry(
        std::nullopt, VehicleOdometryAdapter(message), message.timestamp_sample,
        rclcpp::Time(1'000'000'000LL, RCL_ROS_TIME));
    ASSERT_TRUE(snapshot);
    EXPECT_EQ(snapshot->source_sample_timestamp_us, 1000U);

    // A republisher may advance its publication timestamp without producing
    // a new physical odometry sample. That cannot refresh the tracker's age.
    message.timestamp = 2200;
    message.position[0] = 50.0F;
    snapshot = CombinedDroneAwarenessHandler::AdvanceMeasuredOdometry(
        snapshot, VehicleOdometryAdapter(message), message.timestamp_sample,
        rclcpp::Time(2'000'000'000LL, RCL_ROS_TIME));
    ASSERT_TRUE(snapshot);
    EXPECT_EQ(snapshot->receipt_stamp.nanoseconds(), 1'000'000'000);

    message.timestamp_sample = 900;
    snapshot = CombinedDroneAwarenessHandler::AdvanceMeasuredOdometry(
        snapshot, VehicleOdometryAdapter(message), message.timestamp_sample,
        rclcpp::Time(3'000'000'000, RCL_ROS_TIME));
    ASSERT_TRUE(snapshot);
    EXPECT_EQ(snapshot->receipt_stamp.nanoseconds(), 1'000'000'000);

    message.timestamp_sample = 1200;
    snapshot = CombinedDroneAwarenessHandler::AdvanceMeasuredOdometry(
        snapshot, VehicleOdometryAdapter(message), message.timestamp_sample,
        rclcpp::Time(4'000'000'000, RCL_ROS_TIME));
    ASSERT_TRUE(snapshot);
    EXPECT_EQ(snapshot->receipt_stamp.nanoseconds(), 4'000'000'000);
    EXPECT_EQ(snapshot->source_sample_timestamp_us, 1200U);

    message.reset_counter = 1;
    message.timestamp_sample = 100;
    snapshot = CombinedDroneAwarenessHandler::AdvanceMeasuredOdometry(
        snapshot, VehicleOdometryAdapter(message), message.timestamp_sample,
        rclcpp::Time(5'000'000'000, RCL_ROS_TIME));
    ASSERT_TRUE(snapshot);
    EXPECT_EQ(snapshot->receipt_stamp.nanoseconds(), 5'000'000'000);
    EXPECT_EQ(snapshot->reset_counter, 1U);
}

TEST(MeasuredOdometrySnapshot, RawPx4LocalPositionMatchesTfFeedbackAndFiniteCommandFrame) {
    px4_msgs::msg::VehicleOdometry message;
    message.pose_frame = message.POSE_FRAME_NED;
    message.velocity_frame = message.VELOCITY_FRAME_NED;
    message.position = {8.28F, -17.41F, -5.53F};
    message.velocity = {0.02F, -0.03F, 0.04F};
    message.q[0] = 1.0F;
    message.timestamp_sample = 1'000'000;
    message.reset_counter = 15;
    VehicleOdometryAdapter before(message);
    const auto first = CombinedDroneAwarenessHandler::AdvanceMeasuredOdometry(
        std::nullopt, before, message.timestamp_sample,
        rclcpp::Time(10'000'000'000LL, RCL_ROS_TIME));
    ASSERT_TRUE(first);
    const auto tf_before = before.ToTransformStamped("drone", "world");
    EXPECT_FLOAT_EQ(tf_before.transform.translation.x, first->state.position()(0));
    EXPECT_FLOAT_EQ(tf_before.transform.translation.y, first->state.position()(1));
    EXPECT_FLOAT_EQ(tf_before.transform.translation.z, first->state.position()(2));

    // A heading-only reset must not erase ordinary raw-world translation.
    message.reset_counter = 16;
    message.timestamp_sample = 1'050'000;
    message.position[0] += 0.02F;
    VehicleOdometryAdapter after(message);
    const auto second = CombinedDroneAwarenessHandler::AdvanceMeasuredOdometry(
        first, after, message.timestamp_sample,
        rclcpp::Time(10'050'000'000LL, RCL_ROS_TIME));
    ASSERT_TRUE(second);
    const auto tf_after = after.ToTransformStamped("drone", "world");
    const iii_drone::control::Reference finite_command(second->state);
    EXPECT_NEAR(second->state.position()(0) - first->state.position()(0), 0.02, 1.0e-6);
    EXPECT_FLOAT_EQ(tf_after.transform.translation.x, finite_command.position()(0));
    EXPECT_FLOAT_EQ(tf_after.transform.translation.y, finite_command.position()(1));
    EXPECT_FLOAT_EQ(tf_after.transform.translation.z, finite_command.position()(2));
    EXPECT_EQ(second->reset_counter, 16U); // independent rest proof observes raw reset
}

TEST(VehicleNavigationEvidence, SourceIdentityAndResetFenceNativeHold) {
    using Clock = std::chrono::steady_clock;
    using Evidence = iii_drone::control::VehicleNavigationEvidence;
    px4_msgs::msg::VehicleStatus status;
    status.timestamp = 100;
    status.nav_state_timestamp = 50;
    status.nav_state = status.NAVIGATION_STATE_EXTERNAL5;
    const auto first_receipt = Clock::now();
    auto evidence = CombinedDroneAwarenessHandler::AdvanceVehicleNavigation(
        Evidence{}, status, first_receipt, true);
    ASSERT_TRUE(evidence.latest);
    ASSERT_TRUE(evidence.last_external);
    EXPECT_EQ(evidence.last_external->source_timestamp_us, 100U);

    status.nav_state = status.NAVIGATION_STATE_AUTO_LOITER;
    status.nav_state_timestamp = 200;
    status.timestamp = 200;
    evidence = CombinedDroneAwarenessHandler::AdvanceVehicleNavigation(
        evidence, status, first_receipt + std::chrono::milliseconds(500), false);
    ASSERT_TRUE(evidence.latest);
    EXPECT_EQ(evidence.latest->nav_state, status.NAVIGATION_STATE_AUTO_LOITER);
    const auto hold_receipt = evidence.latest->receipt;
    evidence = CombinedDroneAwarenessHandler::AdvanceVehicleNavigation(
        evidence, status, first_receipt + std::chrono::seconds(2), false);
    EXPECT_EQ(evidence.latest->receipt, hold_receipt);  // duplicate is not fresh

    status.timestamp = 150;  // out of order source sample invalidates epoch
    evidence = CombinedDroneAwarenessHandler::AdvanceVehicleNavigation(
        evidence, status, first_receipt + std::chrono::seconds(3), false);
    EXPECT_FALSE(evidence.latest);
    EXPECT_FALSE(evidence.last_external);
    EXPECT_EQ(evidence.source_epoch, 1U);

    status.timestamp = 0;  // missing source stamp is never positive proof
    evidence = CombinedDroneAwarenessHandler::AdvanceVehicleNavigation(
        evidence, status, first_receipt + std::chrono::seconds(4), false);
    EXPECT_FALSE(evidence.latest);
    EXPECT_FALSE(evidence.last_external);
}

TEST(GroundAltitudeEstimateAmsl, AddsTheAmslOffsetOfTheSameInstant) {
    EXPECT_NEAR(iii_drone::control::GroundAltitudeEstimateAmsl(0.10, 45.30F, 0.12F), 45.28, 1.0e-5);
}

TEST(GroundAltitudeEstimateAmsl, UnknownAltitudeIsNan) {
    using iii_drone::control::GroundAltitudeEstimateAmsl;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();
    // A NaN AMSL altitude never compared equal to NAN, so it passed the old guard.
    EXPECT_TRUE(std::isnan(GroundAltitudeEstimateAmsl(0.10, nan, 0.12F)));
    EXPECT_TRUE(std::isnan(GroundAltitudeEstimateAmsl(0.10, infinity, 0.12F)));
    EXPECT_TRUE(std::isnan(GroundAltitudeEstimateAmsl(0.10, -infinity, 0.12F)));
    EXPECT_TRUE(std::isnan(GroundAltitudeEstimateAmsl(0.10, 0.0F, 0.12F)));
    EXPECT_TRUE(std::isnan(GroundAltitudeEstimateAmsl(0.10, 45.30F, nan)));
}
