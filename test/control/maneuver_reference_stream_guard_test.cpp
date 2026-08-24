#include <gtest/gtest.h>

#include <iii_drone_core/control/maneuver/maneuver_reference_stream_guard.hpp>

using iii_drone::control::maneuver::ManeuverReferenceStreamDecision;
using iii_drone::control::maneuver::ManeuverReferenceStreamGuard;
using Stream = iii_drone_interfaces::msg::ManeuverReferenceStream;

namespace {

Stream sample(const std::string & id, uint64_t sequence, int64_t valid_until_ns) {
    Stream message;
    message.stream_id = id;
    message.sequence = sequence;
    message.state = Stream::STATE_ACTIVE;
    message.is_valid = true;
    message.valid_until.sec = static_cast<int32_t>(valid_until_ns / 1000000000LL);
    message.valid_until.nanosec = static_cast<uint32_t>(valid_until_ns % 1000000000LL);
    return message;
}

}  // namespace

TEST(ManeuverReferenceStreamGuard, RejectsExpiredAndInvalidSamples) {
    ManeuverReferenceStreamGuard guard;
    auto message = sample("generation-a", 1, 100);
    EXPECT_EQ(guard.observe(message, rclcpp::Time(101)), ManeuverReferenceStreamDecision::Expired);
    message.is_valid = false;
    EXPECT_EQ(guard.observe(message, rclcpp::Time(99)), ManeuverReferenceStreamDecision::Invalid);
}

TEST(ManeuverReferenceStreamGuard, EvaluatesExpiryInTheSuppliedRosClockDomain) {
    ManeuverReferenceStreamGuard guard;
    const auto simulation_sample = sample(
        "generation-sim",
        1,
        436500000000LL
    );

    EXPECT_EQ(
        guard.observe(simulation_sample, rclcpp::Time(436400000000LL, RCL_ROS_TIME)),
        ManeuverReferenceStreamDecision::NewActive
    );
    EXPECT_EQ(
        guard.observe(simulation_sample, rclcpp::Time(436600000000LL, RCL_ROS_TIME)),
        ManeuverReferenceStreamDecision::Expired
    );
}

TEST(ManeuverReferenceStreamGuard, RejectsWrongGenerationAndOutOfOrderSamples) {
    ManeuverReferenceStreamGuard guard;
    EXPECT_EQ(
        guard.observe(sample("generation-a", 4, 1000), rclcpp::Time(10)),
        ManeuverReferenceStreamDecision::NewActive
    );
    guard.commitCandidate();
    EXPECT_EQ(
        guard.observe(sample("generation-b", 5, 1000), rclcpp::Time(10)),
        ManeuverReferenceStreamDecision::WrongGeneration
    );
    EXPECT_EQ(
        guard.observe(sample("generation-a", 3, 1000), rclcpp::Time(10)),
        ManeuverReferenceStreamDecision::OutOfOrder
    );
    EXPECT_EQ(
        guard.observe(sample("generation-a", 4, 1000), rclcpp::Time(10)),
        ManeuverReferenceStreamDecision::FreshHeld
    );
}

TEST(ManeuverReferenceStreamGuard, SwitchesGenerationOnlyWhenExplicitlyExpected) {
    ManeuverReferenceStreamGuard guard;
    guard.expectGeneration("generation-b");
    EXPECT_EQ(
        guard.observe(sample("generation-a", 100, 1000), rclcpp::Time(10)),
        ManeuverReferenceStreamDecision::WrongGeneration
    );
    EXPECT_EQ(
        guard.observe(sample("generation-b", 1, 1000), rclcpp::Time(10)),
        ManeuverReferenceStreamDecision::NewActive
    );
    guard.commitCandidate();
    EXPECT_EQ(guard.streamId(), "generation-b");
    EXPECT_EQ(guard.lastAppliedSequence(), 1U);
}

TEST(ManeuverReferenceStreamGuard, ExposesPreparedAndPausedControlStates) {
    ManeuverReferenceStreamGuard guard;
    auto message = sample("generation-b", 1, 1000);
    message.state = Stream::STATE_PREPARED;
    EXPECT_EQ(
        guard.observe(message, rclcpp::Time(10)),
        ManeuverReferenceStreamDecision::Prepared
    );
    message.state = Stream::STATE_PAUSED;
    EXPECT_EQ(
        guard.observe(message, rclcpp::Time(10)),
        ManeuverReferenceStreamDecision::Paused
    );
}

TEST(ManeuverReferenceStreamGuard, CommitContinuesSequenceWithinPreparedGeneration) {
    ManeuverReferenceStreamGuard guard;
    guard.expectGeneration("generation-b");

    auto prepared = sample("generation-b", 7, 1000);
    prepared.state = Stream::STATE_PREPARED;
    EXPECT_EQ(
        guard.observe(prepared, rclcpp::Time(10)),
        ManeuverReferenceStreamDecision::Prepared
    );

    EXPECT_EQ(
        guard.observe(sample("generation-b", 8, 1000), rclcpp::Time(10)),
        ManeuverReferenceStreamDecision::NewActive
    );
    guard.commitCandidate();
    EXPECT_EQ(guard.lastAppliedSequence(), 8U);
}

TEST(ManeuverReferenceStreamGuard, AllowsExactlyOneExplicitSuccessorGeneration) {
    ManeuverReferenceStreamGuard guard;
    EXPECT_EQ(
        guard.observe(sample("generation-a", 7, 1000), rclcpp::Time(10)),
        ManeuverReferenceStreamDecision::NewActive
    );
    guard.commitCandidate();

    guard.expectSuccessorGeneration();
    EXPECT_TRUE(guard.successorGenerationExpected());
    EXPECT_EQ(
        guard.observe(sample("generation-a", 7, 1000), rclcpp::Time(10)),
        ManeuverReferenceStreamDecision::FreshHeld
    );
    EXPECT_TRUE(guard.successorGenerationExpected());

    EXPECT_EQ(
        guard.observe(sample("generation-b", 1, 1000), rclcpp::Time(10)),
        ManeuverReferenceStreamDecision::NewActive
    );
    EXPECT_FALSE(guard.successorGenerationExpected());
    guard.commitCandidate();
    EXPECT_EQ(guard.streamId(), "generation-b");
    EXPECT_EQ(guard.lastAppliedSequence(), 1U);

    EXPECT_EQ(
        guard.observe(sample("generation-c", 1, 1000), rclcpp::Time(10)),
        ManeuverReferenceStreamDecision::WrongGeneration
    );
}
