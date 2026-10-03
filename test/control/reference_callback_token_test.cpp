#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <thread>
#include <vector>

#include <iii_drone_core/control/maneuver/reference_callback_token.hpp>
#include <iii_drone_core/control/maneuver/maneuver_request_identity.hpp>

#include <iii_drone_configuration/configuration.hpp>

#define private public
#include <iii_drone_core/control/maneuver/maneuver_scheduler.hpp>
#include <iii_drone_core/control/maneuver/fly_to_position_maneuver_server.hpp>
#undef private

using iii_drone::control::Reference;
using iii_drone::control::State;
using iii_drone::control::maneuver::ReferenceCallbackStruct;
using iii_drone::control::maneuver::ManeuverRequestIdentityGenerator;

constexpr char kRequestA[] = "mri1-00000000000000010000000000000001-0000000000000001";
constexpr char kRequestB[] = "mri1-00000000000000020000000000000002-0000000000000002";

namespace {

class TestHoverOnCableManeuverServer final : public iii_drone::control::maneuver::ManeuverServer {
public:
    explicit TestHoverOnCableManeuverServer(rclcpp_lifecycle::LifecycleNode * node)
    : ManeuverServer(node, nullptr, "hover_on_cable", 1, 1) {}

    bool CanExecuteManeuver(
        const iii_drone::control::maneuver::Maneuver &,
        const iii_drone::adapters::CombinedDroneAwarenessAdapter &
    ) const override {
        return true;
    }

    iii_drone::adapters::CombinedDroneAwarenessAdapter ExpectedAwarenessAfterExecution(
        const iii_drone::control::maneuver::Maneuver &
    ) override {
        return {};
    }

    iii_drone::control::maneuver::maneuver_type_t maneuver_type() const override {
        return iii_drone::control::maneuver::MANEUVER_TYPE_HOVER_ON_CABLE;
    }

    void startExecution(iii_drone::control::maneuver::Maneuver &) override {}
    bool canCancel() override { return true; }
    Reference computeReference(const State &) override { return Reference(); }
    bool hasSucceeded(iii_drone::control::maneuver::Maneuver &) override { return false; }
    bool hasFailed(iii_drone::control::maneuver::Maneuver &) override { return false; }
    void publishResultAndFinalize(
        iii_drone::control::maneuver::Maneuver &,
        maneuver_result_type_t
    ) override {}
    void registerReferenceCallbackOnSuccess(
        const iii_drone::control::maneuver::Maneuver &
    ) override {}
};

class TestableFlyToPositionManeuverServer final :
    public iii_drone::control::maneuver::FlyToPositionManeuverServer {
public:
    using FlyToPositionManeuverServer::FlyToPositionManeuverServer;

    Reference initializationReferenceForTest(const State & state) const {
        return initializationReference(state);
    }
};

}  // namespace

TEST(ManeuverRequestIdentity, ProcessEpochPreventsClockZeroRestartCollision) {
    const ManeuverRequestIdentityGenerator::Epoch first_epoch{
        0x0123456789abcdefULL,
        0xfedcba9876543210ULL,
    };
    const ManeuverRequestIdentityGenerator::Epoch restarted_epoch{
        0x1111111111111111ULL,
        0x2222222222222222ULL,
    };
    ManeuverRequestIdentityGenerator first_process(first_epoch);
    ManeuverRequestIdentityGenerator restarted_process(restarted_epoch);

    const std::string first = first_process.next();
    const std::string restarted = restarted_process.next();
    EXPECT_TRUE(iii_drone::control::maneuver::isValidManeuverRequestIdentity(first));
    EXPECT_TRUE(iii_drone::control::maneuver::isValidManeuverRequestIdentity(restarted));
    // Both counters start at one: only their process epochs distinguish a
    // simulated ROS-clock-zero restart.
    EXPECT_EQ(first.substr(38), restarted.substr(38));
    EXPECT_NE(first, restarted);
    EXPECT_NE(first.substr(5, 32), restarted.substr(5, 32));
}

TEST(ReferenceCallbackBinding, SuccessorFiniteSeedIsAtomicAndRequestFenced) {
    ReferenceCallbackStruct callback;
    const Reference seed(
        iii_drone::types::point_t(1.2, -0.4, 2.5), 0.3,
        iii_drone::types::vector_t::Zero(), 0.0,
        iii_drone::types::vector_t::Zero(), 0.0);
    const auto generation = callback.beginExecution(
        "fly_to_position", kRequestA,
        [seed](const State &) { return seed.CopyWithNewStamp(); });
    const auto pending = callback.snapshot();
    ASSERT_TRUE(pending.callback);
    EXPECT_EQ(pending.execution_id, generation);
    EXPECT_EQ(pending.request_identity, kRequestA);
    // A delayed token acquisition or planner call still leaves a finite
    // new-generation command, including velocity and acceleration channels.
    for (int poll = 0; poll < 100; ++poll) {
        const auto emitted = pending.callback(State());
        EXPECT_LT((emitted.position() - seed.position()).norm(), 1.0e-12);
        EXPECT_LT((emitted.velocity() - seed.velocity()).norm(), 1.0e-12);
        EXPECT_LT((emitted.acceleration() - seed.acceleration()).norm(), 1.0e-12);
    }
    const auto successor = callback.beginExecution("fly_to_position", kRequestB);
    EXPECT_GT(successor, generation);
    EXPECT_FALSE(callback.snapshot().callback);
    EXPECT_EQ(callback.snapshot().request_identity, kRequestB);
    EXPECT_NE(pending.execution_id, callback.snapshot().execution_id);
}

TEST(ReferenceCallbackBinding, SameIdentityReplacementRetiresCopiedCallableAfterDrain) {
    ReferenceCallbackStruct callbacks;
    std::promise<void> entered;
    auto entered_future = entered.get_future();
    std::promise<void> release;
    const auto release_future = release.get_future().share();
    const auto execution = callbacks.beginExecution("fly_to_object", kRequestA);
    callbacks.set([&](const State &) {
        entered.set_value();
        release_future.wait();
        return Reference(iii_drone::types::point_t(0.25F, 0.0F, 1.0F), 0.0);
    }, "fly_to_object", execution, kRequestA);
    const auto old = callbacks.snapshot();
    ASSERT_TRUE(old.lease);
    std::thread entered_call([&] { (void)old.callback(State()); });
    const bool entered_before_replacement =
        entered_future.wait_for(std::chrono::seconds(2)) == std::future_status::ready;
    if (!entered_before_replacement) {
        release.set_value();
        entered_call.join();
        FAIL() << "the old callback did not enter before replacement";
    }
    old.lease->requestQuiescence();
    EXPECT_FALSE(old.lease->drained());
    callbacks.set([](const State &) {
        return Reference(iii_drone::types::point_t(0.5F, 0.0F, 1.0F), 0.0);
    }, "fly_to_object", execution, kRequestA);
    const auto replacement = callbacks.snapshot();
    EXPECT_EQ(replacement.execution_id, old.execution_id);
    EXPECT_EQ(replacement.request_identity, old.request_identity);
    EXPECT_EQ(replacement.reference_provider_name, old.reference_provider_name);
    EXPECT_GT(replacement.revision, old.revision);
    EXPECT_TRUE(old.lease->retired());
    release.set_value();
    entered_call.join();
    old.lease->waitUntilDrained();
    EXPECT_THROW((void)old.callback(State()),
        iii_drone::control::maneuver::RetiredReferenceCallback);
    EXPECT_LT((replacement.callback(State()).position() -
        iii_drone::types::point_t(0.5F, 0.0F, 1.0F)).norm(), 1.0e-6);
}

TEST(ReferenceCallbackBinding, SameProviderSuccessorGenerationRetiresPriorCallable) {
    ReferenceCallbackStruct callbacks;
    const auto old_execution = callbacks.beginExecution("hover_by_object", kRequestA);
    callbacks.set([](const State &) {
        return Reference(iii_drone::types::point_t(0.25F, 0.0F, 1.0F), 0.0);
    }, "hover_by_object", old_execution, kRequestA);
    const auto old = callbacks.snapshot();
    ASSERT_TRUE(old.lease);
    const auto next_execution = callbacks.beginExecution("hover_by_object", kRequestB,
        [](const State &) {
            return Reference(iii_drone::types::point_t(0.50F, 0.0F, 1.0F), 0.0);
        });
    const auto next = callbacks.snapshot();
    EXPECT_GT(next_execution, old_execution);
    EXPECT_GT(next.revision, old.revision);
    EXPECT_EQ(next.reference_provider_name, old.reference_provider_name);
    EXPECT_EQ(next.request_identity, kRequestB);
    EXPECT_TRUE(old.lease->retired());
    EXPECT_THROW((void)old.callback(State()),
        iii_drone::control::maneuver::RetiredReferenceCallback);
    EXPECT_LT((next.callback(State()).position() -
        iii_drone::types::point_t(0.50F, 0.0F, 1.0F)).norm(), 1.0e-6);
}

TEST(ManeuverSchedulerAppliedRestProof, RequiresCurrentPublishedFiniteAngularChannels) {
    const bool initialized_here = !rclcpp::ok();
    if (initialized_here) rclcpp::init(0, nullptr);
    {
        rclcpp_lifecycle::LifecycleNode node("fwp_applied_rest_proof_test");
        auto configuration = std::make_shared<iii_drone::configuration::Configuration>(
            "fwp-applied-rest-proof-test",
            std::vector<iii_drone::configuration::configuration_entry_t>{
                {"/control/maneuver_controller/maneuver_execution_period_ms", rclcpp::ParameterType::PARAMETER_INTEGER},
                {"/control/maneuver_controller/reference_stream_timeout_ms", rclcpp::ParameterType::PARAMETER_INTEGER},
            },
            [](const std::string & name) { return rclcpp::Parameter(name, 1000); });
        auto awareness = std::make_shared<iii_drone::control::CombinedDroneAwarenessHandler>(
            configuration, std::make_shared<tf2_ros::Buffer>(node.get_clock()), &node);
        iii_drone::control::maneuver::ManeuverScheduler scheduler(
            &node, awareness, configuration,
            node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
        const Reference rest(iii_drone::types::point_t(1.0, 0.0, 2.0), 0.2,
            iii_drone::types::vector_t::Zero(), 0.0,
            iii_drone::types::vector_t::Zero(), 0.0);
        const auto execution = scheduler.reference_callback_struct_->beginExecution(
            "follow_waypoint_path", kRequestA,
            [rest](const State &) { return rest; });
        scheduler.current_reference_execution_id_.Store(execution);
        scheduler.maneuver_server_get_reference_callback_still_registered_ = true;
        auto & stream = scheduler.reference_stream_state_;
        stream.valid = true;
        stream.stream_id = "follow_waypoint_path:rest";
        stream.request_identity = kRequestA;
        stream.execution_id = execution;
        stream.sequence = 1;
        stream.recent_references.emplace_back(1, rest);
        auto ack = std::make_shared<iii_drone_interfaces::msg::ManeuverReferenceAck>();
        ack->stream_id = stream.stream_id;
        ack->last_applied_sequence = 1;
        ack->consumer_status = iii_drone_interfaces::msg::ManeuverReferenceAck::STATUS_APPLIED;
        scheduler.acknowledgeReferenceStream(ack);
        EXPECT_TRUE(scheduler.appliedFiniteRestReference(kRequestA, rest));
        EXPECT_FALSE(scheduler.appliedFiniteRestReference(kRequestB, rest));

        const Reference angular_mismatch(rest.position(), rest.yaw(), rest.velocity(),
            0.1, rest.acceleration(), 0.1);
        stream.sequence = 2;
        stream.recent_references.emplace_back(2, angular_mismatch);
        ack->last_applied_sequence = 2;
        scheduler.acknowledgeReferenceStream(ack);
        EXPECT_FALSE(scheduler.appliedFiniteRestReference(kRequestA, rest));
        stream.last_ack = std::chrono::steady_clock::now() - std::chrono::seconds(2);
        EXPECT_FALSE(scheduler.appliedFiniteRestReference(kRequestA, rest));
        scheduler.is_started_ = false;
    }
    if (initialized_here) rclcpp::shutdown();
}

TEST(ManeuverSchedulerScopedQueueClear, PreservesOtherRequestAndCurrentManeuver) {
    const bool context_was_initialized = rclcpp::ok();
    if (!context_was_initialized) {
        rclcpp::init(0, nullptr);
    }
    {
        rclcpp_lifecycle::LifecycleNode node("scoped_queue_clear_test");
        auto configuration = std::make_shared<iii_drone::configuration::Configuration>(
            "scoped-queue-clear-test",
            std::vector<iii_drone::configuration::configuration_entry_t>{
                {"/control/maneuver_controller/maneuver_execution_period_ms", rclcpp::ParameterType::PARAMETER_INTEGER},
                {"/control/maneuver_controller/reference_stream_timeout_ms", rclcpp::ParameterType::PARAMETER_INTEGER},
            },
            [](const std::string & name) { return rclcpp::Parameter(name, 100); }
        );
        auto awareness = std::make_shared<iii_drone::control::CombinedDroneAwarenessHandler>(
            configuration,
            std::make_shared<tf2_ros::Buffer>(node.get_clock()),
            &node
        );
        iii_drone::control::maneuver::ManeuverScheduler scheduler(
            &node,
            awareness,
            configuration,
            node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)
        );
        scheduler.is_started_ = true;
        scheduler.maneuver_queue_ = std::make_unique<iii_drone::control::maneuver::ManeuverQueue>();

        iii_drone::control::maneuver::Maneuver queued_a(
            iii_drone::control::maneuver::MANEUVER_TYPE_HOVER, {}
        );
        queued_a.request_identity_ = kRequestA;
        iii_drone::control::maneuver::Maneuver queued_b(
            iii_drone::control::maneuver::MANEUVER_TYPE_HOVER, {}
        );
        queued_b.request_identity_ = kRequestB;
        iii_drone::control::maneuver::Maneuver current(
            iii_drone::control::maneuver::MANEUVER_TYPE_HOVER, {}
        );
        current.request_identity_ = kRequestB;
        scheduler.current_maneuver_.Store(current);
        ASSERT_TRUE(scheduler.maneuver_queue_->Push(queued_a));
        ASSERT_TRUE(scheduler.maneuver_queue_->Push(queued_b));

        auto scoped_request = std::make_shared<iii_drone_interfaces::srv::ClearManeuverQueue::Request>();
        scoped_request->reason = "retire request A";
        scoped_request->request_identity = kRequestA;
        auto scoped_response = std::make_shared<iii_drone_interfaces::srv::ClearManeuverQueue::Response>();
        scheduler.clearManeuverQueueServiceCallback(scoped_request, scoped_response);
        EXPECT_TRUE(scoped_response->success);
        EXPECT_EQ(scoped_response->cleared_count, 1U);
        ASSERT_EQ(scheduler.maneuver_queue_->size(), 1);
        EXPECT_EQ(scheduler.maneuver_queue_->vector().front().requestIdentity(), kRequestB);
        EXPECT_EQ(scheduler.current_maneuver_.Load().requestIdentity(), kRequestB);

        auto malformed_request = std::make_shared<iii_drone_interfaces::srv::ClearManeuverQueue::Request>();
        malformed_request->reason = "must fail closed";
        malformed_request->request_identity = "legacy";
        auto malformed_response = std::make_shared<iii_drone_interfaces::srv::ClearManeuverQueue::Response>();
        scheduler.clearManeuverQueueServiceCallback(malformed_request, malformed_response);
        EXPECT_FALSE(malformed_response->success);
        EXPECT_EQ(malformed_response->cleared_count, 0U);
        EXPECT_EQ(scheduler.maneuver_queue_->size(), 1);

        auto global_request = std::make_shared<iii_drone_interfaces::srv::ClearManeuverQueue::Request>();
        global_request->reason = "explicit operator global clear";
        auto global_response = std::make_shared<iii_drone_interfaces::srv::ClearManeuverQueue::Response>();
        scheduler.clearManeuverQueueServiceCallback(global_request, global_response);
        EXPECT_TRUE(global_response->success);
        EXPECT_EQ(global_response->cleared_count, 1U);
        EXPECT_TRUE(scheduler.maneuver_queue_->empty());
        EXPECT_EQ(scheduler.current_maneuver_.Load().requestIdentity(), kRequestB);
        // This unit fixture bypasses Start() to seed the queue directly;
        // restore the lifecycle flag before Stop() runs in the destructor.
        scheduler.is_started_ = false;
    }
    if (!context_was_initialized) {
        rclcpp::shutdown();
    }
}

TEST(BlendedFlyToPosition, NearTargetWaitsForCurrentAppliedStreamAck) {
    const bool context_was_initialized = rclcpp::ok();
    if (!context_was_initialized) {
        rclcpp::init(0, nullptr);
    }
    {
        rclcpp_lifecycle::LifecycleNode node("blended_ftp_applied_ack_test");
        auto configuration = std::make_shared<iii_drone::configuration::Configuration>(
            "blended-ftp-applied-ack-test",
            std::vector<iii_drone::configuration::configuration_entry_t>{
                {"/control/maneuver_controller/maneuver_execution_period_ms", rclcpp::ParameterType::PARAMETER_INTEGER},
                {"/control/maneuver_controller/reference_stream_timeout_ms", rclcpp::ParameterType::PARAMETER_INTEGER},
                {"/control/maneuver_controller/reached_position_euclidean_distance_threshold", rclcpp::ParameterType::PARAMETER_DOUBLE},
                {"/control/maneuver_controller/fly_to_position_blend_radius", rclcpp::ParameterType::PARAMETER_DOUBLE},
                {"/control/maneuver_controller/reached_yaw_error_threshold", rclcpp::ParameterType::PARAMETER_DOUBLE},
                {"/control/maneuver_controller/fly_to_position_use_mpc", rclcpp::ParameterType::PARAMETER_BOOL},
            },
            [](const std::string & name) {
                if (name == "/control/maneuver_controller/fly_to_position_use_mpc") {
                    return rclcpp::Parameter(name, false);
                }
                if (name == "/control/maneuver_controller/maneuver_execution_period_ms" ||
                    name == "/control/maneuver_controller/reference_stream_timeout_ms") {
                    return rclcpp::Parameter(name, 100);
                }
                return rclcpp::Parameter(name, 1.0);
            }
        );
        auto awareness = std::make_shared<iii_drone::control::CombinedDroneAwarenessHandler>(
            configuration, std::make_shared<tf2_ros::Buffer>(node.get_clock()), &node
        );
        iii_drone::control::maneuver::ManeuverScheduler scheduler(
            &node, awareness, configuration,
            node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)
        );
        iii_drone::control::maneuver::FlyToPositionManeuverServer server(
            &node, awareness, "fly_to_position", 1, 1, configuration, nullptr
        );
        server.RegisterBlendReferenceAppliedCallback(
            [&scheduler](const std::string & request_identity) {
                return scheduler.blendedReferenceApplied(request_identity);
            }
        );
        server.active_blend_to_next_ = true;
        server.target_reference_ = Reference(iii_drone::types::point_t(0.382, 0.0, 0.0), 0.0);
        server.maneuver_start_time_ = node.now();
        iii_drone::control::maneuver::Maneuver goal(
            iii_drone::control::maneuver::MANEUVER_TYPE_FLY_TO_POSITION, {}
        );
        goal.request_identity_ = kRequestB;

        scheduler.beginReferenceExecution("fly_to_position", kRequestB);
        const auto first_execution = scheduler.current_reference_execution_id_.Load();
        scheduler.reference_stream_state_.valid = true;
        scheduler.reference_stream_state_.stream_id = "fly_to_position:first";
        scheduler.reference_stream_state_.request_identity = kRequestB;
        scheduler.reference_stream_state_.execution_id = first_execution;
        scheduler.reference_stream_state_.sequence = 1;

        EXPECT_FALSE(server.hasSucceeded(goal));  // Already inside 1 m, but no consumer ACK.
        EXPECT_FALSE(server.blend_completion_reference_.has_value());

        auto ack = std::make_shared<iii_drone_interfaces::msg::ManeuverReferenceAck>();
        ack->stream_id = "fly_to_position:other";
        ack->last_applied_sequence = 1;
        ack->consumer_status = iii_drone_interfaces::msg::ManeuverReferenceAck::STATUS_APPLIED;
        scheduler.acknowledgeReferenceStream(ack);
        EXPECT_FALSE(server.hasSucceeded(goal));

        ack->stream_id = "fly_to_position:first";
        ack->consumer_status = iii_drone_interfaces::msg::ManeuverReferenceAck::STATUS_ACTION_ABORT_READY;
        scheduler.acknowledgeReferenceStream(ack);
        EXPECT_FALSE(server.hasSucceeded(goal));

        scheduler.beginReferenceExecution("fly_to_position", kRequestB);
        scheduler.reference_stream_state_.valid = true;
        scheduler.reference_stream_state_.stream_id = "fly_to_position:second";
        scheduler.reference_stream_state_.request_identity = kRequestB;
        scheduler.reference_stream_state_.execution_id = scheduler.current_reference_execution_id_.Load();
        scheduler.reference_stream_state_.sequence = 1;
        scheduler.reference_stream_state_.recent_references.emplace_back(
            1, Reference(iii_drone::types::point_t::Zero(), 0.0));
        ack->stream_id = "fly_to_position:first";
        ack->consumer_status = iii_drone_interfaces::msg::ManeuverReferenceAck::STATUS_APPLIED;
        scheduler.acknowledgeReferenceStream(ack);
        EXPECT_FALSE(server.hasSucceeded(goal));  // Applied predecessor generation is stale.

        ack->stream_id = "fly_to_position:second";
        ack->last_applied_sequence = 0;
        scheduler.acknowledgeReferenceStream(ack);
        EXPECT_FALSE(server.hasSucceeded(goal));  // No reference has actually been applied.

        ack->last_applied_sequence = 1;
        scheduler.acknowledgeReferenceStream(ack);
        iii_drone::control::maneuver::Maneuver unrelated_goal(goal);
        unrelated_goal.request_identity_ = kRequestA;
        EXPECT_FALSE(server.hasSucceeded(unrelated_goal));
        EXPECT_TRUE(server.hasSucceeded(goal));
        EXPECT_TRUE(server.blend_completion_reference_.has_value());
        // An ACK for a sequence no longer represented by the producer's
        // command ring cannot refresh the transferable command proof.
        const auto prior_ack_time = scheduler.reference_stream_state_.last_ack;
        scheduler.reference_stream_state_.sequence = 2;
        scheduler.reference_stream_state_.recent_references.clear();
        ack->last_applied_sequence = 2;
        scheduler.acknowledgeReferenceStream(ack);
        EXPECT_EQ(scheduler.reference_stream_state_.last_ack_sequence, 1U);
        EXPECT_EQ(scheduler.reference_stream_state_.last_ack, prior_ack_time);
    }
    if (!context_was_initialized) {
        rclcpp::shutdown();
    }
}

TEST(BlendedFlyToPosition, InitializationUsesFreshPendingAnchorWithoutConsumingIt) {
    const bool context_was_initialized = rclcpp::ok();
    if (!context_was_initialized) {
        rclcpp::init(0, nullptr);
    }
    {
        rclcpp_lifecycle::LifecycleNode node("blended_ftp_initialization_anchor_test");
        auto configuration = std::make_shared<iii_drone::configuration::Configuration>(
            "blended-ftp-initialization-anchor-test",
            std::vector<iii_drone::configuration::configuration_entry_t>{
                {"/control/maneuver_controller/maneuver_execution_period_ms", rclcpp::ParameterType::PARAMETER_INTEGER},
                {"/control/maneuver_controller/reference_stream_timeout_ms", rclcpp::ParameterType::PARAMETER_INTEGER},
                {"/control/maneuver_controller/reached_position_euclidean_distance_threshold", rclcpp::ParameterType::PARAMETER_DOUBLE},
                {"/control/maneuver_controller/fly_to_position_blend_radius", rclcpp::ParameterType::PARAMETER_DOUBLE},
                {"/control/maneuver_controller/reached_yaw_error_threshold", rclcpp::ParameterType::PARAMETER_DOUBLE},
                {"/control/maneuver_controller/fly_to_position_use_mpc", rclcpp::ParameterType::PARAMETER_BOOL},
            },
            [](const std::string & name) {
                if (name == "/control/maneuver_controller/fly_to_position_use_mpc") {
                    return rclcpp::Parameter(name, false);
                }
                if (name == "/control/maneuver_controller/maneuver_execution_period_ms" ||
                    name == "/control/maneuver_controller/reference_stream_timeout_ms") {
                    return rclcpp::Parameter(name, 100);
                }
                return rclcpp::Parameter(name, 1.0);
            }
        );
        auto awareness = std::make_shared<iii_drone::control::CombinedDroneAwarenessHandler>(
            configuration, std::make_shared<tf2_ros::Buffer>(node.get_clock()), &node
        );
        TestableFlyToPositionManeuverServer server(
            &node, awareness, "fly_to_position", 1, 1, configuration, nullptr
        );

        const State actual_state(
            iii_drone::types::point_t(-2.0, 3.0, 4.0),
            iii_drone::types::vector_t(0.1, 0.2, 0.3),
            0.4,
            iii_drone::types::vector_t(0.01, 0.02, 0.03)
        );
        const Reference cold_hold = server.initializationReferenceForTest(actual_state);
        EXPECT_TRUE(cold_hold.position().isApprox(actual_state.position()));
        EXPECT_NEAR(cold_hold.yaw(), actual_state.yaw(), 1.0e-6);
        EXPECT_TRUE(std::isnan(cold_hold.velocity().x()));
        EXPECT_TRUE(std::isnan(cold_hold.yaw_rate()));

        const Reference planned_anchor(
            iii_drone::types::point_t(8.0, -7.0, 6.0),
            -0.75,
            iii_drone::types::vector_t(0.4, -0.3, 0.2),
            0.15,
            iii_drone::types::vector_t(-0.1, 0.2, -0.3),
            -0.05,
            node.now()
        );
        server.pending_blend_start_reference_ = planned_anchor;
        server.pending_blend_start_time_ = node.now() - rclcpp::Duration::from_seconds(0.25);

        const Reference initialization_hold =
            server.initializationReferenceForTest(actual_state);
        EXPECT_TRUE(initialization_hold.position().isApprox(planned_anchor.position()));
        EXPECT_NEAR(initialization_hold.yaw(), planned_anchor.yaw(), 1.0e-12);
        EXPECT_TRUE(std::isnan(initialization_hold.velocity().x()));
        EXPECT_TRUE(std::isnan(initialization_hold.yaw_rate()));
        ASSERT_TRUE(server.pending_blend_start_reference_.has_value());

        Reference consumed_anchor;
        ASSERT_TRUE(server.consumePendingBlendStartReference(consumed_anchor));
        EXPECT_FALSE(server.pending_blend_start_reference_.has_value());
        EXPECT_TRUE(consumed_anchor.position().isApprox(planned_anchor.position()));
        EXPECT_TRUE(consumed_anchor.velocity().isApprox(planned_anchor.velocity()));
        EXPECT_NEAR(consumed_anchor.yaw_rate(), planned_anchor.yaw_rate(), 1.0e-12);

        server.pending_blend_start_reference_ = planned_anchor;
        server.pending_blend_start_time_ = node.now() - rclcpp::Duration::from_seconds(2.1);
        const Reference expired_hold = server.initializationReferenceForTest(actual_state);
        EXPECT_TRUE(expired_hold.position().isApprox(actual_state.position()));
        EXPECT_TRUE(server.pending_blend_start_reference_.has_value());
        EXPECT_FALSE(server.consumePendingBlendStartReference(consumed_anchor));
        EXPECT_FALSE(server.pending_blend_start_reference_.has_value());

        server.pending_blend_start_reference_ = planned_anchor;
        server.pending_blend_start_time_ = node.now() + rclcpp::Duration::from_seconds(1.0);
        const Reference future_hold = server.initializationReferenceForTest(actual_state);
        EXPECT_TRUE(future_hold.position().isApprox(actual_state.position()));
        EXPECT_FALSE(server.consumePendingBlendStartReference(consumed_anchor));
        EXPECT_FALSE(server.pending_blend_start_reference_.has_value());
    }
    if (!context_was_initialized) {
        rclcpp::shutdown();
    }
}

TEST(ReferenceCallbackToken, ReplacementRetainsTheInFlightCallable)
{
    std::promise<void> entered;
    auto entered_future = entered.get_future();
    std::promise<void> release;
    auto released = release.get_future().share();
    auto captured_resource = std::make_shared<int>(42);
    std::weak_ptr<int> lifetime = captured_resource;
    ReferenceCallbackStruct callback;
    callback.set(
        [captured_resource, &entered, released](const State &) {
            // Copy everything used after the barrier before notifying the
            // setter. The test observes capture lifetime without dereferencing
            // a capture that the broken wrapper may already have destroyed.
            auto resume = released;
            entered.set_value();
            resume.wait();
            return Reference();
        },
        "initialization_hold"
    );
    captured_resource.reset();
    std::thread reader([&] { callback(State()); });
    const bool did_enter = entered_future.wait_for(std::chrono::seconds(5)) == std::future_status::ready;
    bool retained_during_call = false;
    if (did_enter) {
        callback.set([](const State &) { return Reference(); }, "planned_reference");
        retained_during_call = !lifetime.expired();
    }
    release.set_value();
    reader.join();
    ASSERT_TRUE(did_enter);
    EXPECT_TRUE(retained_during_call);
    EXPECT_TRUE(lifetime.expired());
}

TEST(ReferenceCallbackToken, UnsetCallbackFailsExplicitly)
{
    ReferenceCallbackStruct callback;
    EXPECT_THROW(callback(State()), std::runtime_error);
}

TEST(ReferenceCallbackToken, SameProviderSuccessorInvalidatesThePredecessorCallable)
{
    ReferenceCallbackStruct callback;
    bool predecessor_called = false;
    bool successor_called = false;
    callback.set(
        [&predecessor_called](const State &) {
            predecessor_called = true;
            return Reference();
        },
        "hover_on_cable"
    );

    const auto predecessor = callback.snapshot();
    const auto successor_execution_id = callback.beginExecution("hover_on_cable");
    const auto pending_successor = callback.snapshot();

    EXPECT_EQ(pending_successor.reference_provider_name, "hover_on_cable");
    EXPECT_NE(successor_execution_id, predecessor.execution_id);
    EXPECT_EQ(pending_successor.execution_id, successor_execution_id);
    EXPECT_FALSE(pending_successor.callback);
    EXPECT_THROW(callback(State()), std::runtime_error);
    EXPECT_FALSE(predecessor_called);

    callback.set(
        [&successor_called](const State &) {
            successor_called = true;
            return Reference();
        },
        "hover_on_cable"
    );
    callback(State());

    EXPECT_TRUE(successor_called);
    EXPECT_FALSE(predecessor_called);
    EXPECT_EQ(callback.snapshot().execution_id, successor_execution_id);
}

TEST(ReferenceCallbackToken, RetainedSuccessCallbackMayChangeProviderWithoutChangingExecution)
{
    ReferenceCallbackStruct callback;
    const auto execution_id = callback.beginExecution("cable_landing", kRequestA);
    callback.set(
        [](const State &) { return Reference(); }, "cable_landing", execution_id, kRequestA
    );

    callback.set(
        [](const State &) { return Reference(); }, "hover_on_cable", execution_id, kRequestA
    );
    const auto retained = callback.snapshot();

    EXPECT_EQ(retained.reference_provider_name, "hover_on_cable");
    EXPECT_EQ(retained.execution_id, execution_id);
    EXPECT_EQ(retained.request_identity, kRequestA);
    EXPECT_TRUE(retained.callback);
}

TEST(ManeuverSchedulerReferenceStream, SameProviderSuccessorRetiresPausedPredecessorBeforeTokenGrant)
{
    const bool context_was_initialized = rclcpp::ok();
    if (!context_was_initialized) {
        rclcpp::init(0, nullptr);
    }

    {
    rclcpp_lifecycle::LifecycleNode node("reference_stream_successor_test");
    auto configuration = std::make_shared<iii_drone::configuration::Configuration>(
        "scheduler-test",
        std::vector<iii_drone::configuration::configuration_entry_t>{
            {
                "/control/maneuver_controller/maneuver_execution_period_ms",
                rclcpp::ParameterType::PARAMETER_INTEGER
            },
            {
                "/control/maneuver_controller/reference_stream_timeout_ms",
                rclcpp::ParameterType::PARAMETER_INTEGER
            }
        },
        [](const std::string & name) {
            return rclcpp::Parameter(name, 100);
        }
    );
    auto awareness = std::make_shared<iii_drone::control::CombinedDroneAwarenessHandler>(
        configuration,
        std::make_shared<tf2_ros::Buffer>(node.get_clock()),
        &node
    );
    iii_drone::control::maneuver::ManeuverScheduler scheduler(
        &node,
        awareness,
        configuration,
        node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)
    );

    bool predecessor_called = false;
    scheduler.reference_callback_struct_->set(
        [&predecessor_called](const State &) {
            predecessor_called = true;
            return Reference();
        },
        "hover_on_cable"
    );
    const auto predecessor = scheduler.reference_callback_struct_->snapshot();
    scheduler.reference_stream_state_.valid = true;
    scheduler.reference_stream_state_.paused = true;
    scheduler.reference_stream_state_.stream_id = "hover_on_cable:predecessor";
    scheduler.reference_stream_state_.execution_id = predecessor.execution_id;
    scheduler.current_reference_execution_id_.Store(predecessor.execution_id);

    scheduler.beginReferenceExecution("hover_on_cable", kRequestB);
    const auto successor = scheduler.reference_callback_struct_->snapshot();

    EXPECT_FALSE(scheduler.reference_stream_state_.valid);
    EXPECT_FALSE(scheduler.reference_stream_state_.paused);
    EXPECT_NE(successor.execution_id, predecessor.execution_id);
    EXPECT_FALSE(successor.callback);
    scheduler.maneuver_server_get_reference_callback_still_registered_ = true;
    EXPECT_FALSE(scheduler.currentReferenceValid(predecessor));
    EXPECT_THROW((*scheduler.reference_callback_struct_)(State()), std::runtime_error);
    EXPECT_FALSE(predecessor_called);

    auto reference_response = std::make_shared<iii_drone_interfaces::srv::GetReference::Response>();
    EXPECT_NO_THROW(scheduler.getReferenceServiceCallback(nullptr, reference_response));
    EXPECT_FALSE(reference_response->is_valid);

    auto pause_request = std::make_shared<iii_drone_interfaces::srv::PauseReferenceStream::Request>();
    pause_request->stream_id = "hover_on_cable:predecessor";
    pause_request->last_applied_sequence = 1;
    auto pause_response = std::make_shared<iii_drone_interfaces::srv::PauseReferenceStream::Response>();
    scheduler.pauseReferenceStream(pause_request, pause_response);

    EXPECT_FALSE(pause_response->accepted);
    EXPECT_FALSE(scheduler.reference_stream_state_.valid);

    auto rebase_request = std::make_shared<iii_drone_interfaces::srv::RebaseReferenceStream::Request>();
    rebase_request->stream_id = "hover_on_cable:predecessor";
    rebase_request->last_applied_sequence = 1;
    auto rebase_response = std::make_shared<iii_drone_interfaces::srv::RebaseReferenceStream::Response>();
    scheduler.rebaseReferenceStream(rebase_request, rebase_response);

    EXPECT_FALSE(rebase_response->accepted);
    EXPECT_TRUE(rebase_response->prepared_stream_id.empty());
    EXPECT_FALSE(scheduler.reference_stream_state_.valid);

    auto stale_ack = std::make_shared<iii_drone_interfaces::msg::ManeuverReferenceAck>();
    stale_ack->stream_id = "hover_on_cable:predecessor";
    stale_ack->last_applied_sequence = 1;
    stale_ack->consumer_status = iii_drone_interfaces::msg::ManeuverReferenceAck::STATUS_APPLIED;
    scheduler.acknowledgeReferenceStream(stale_ack);

    EXPECT_FALSE(scheduler.reference_stream_state_.valid);

    auto successor_server = std::make_shared<TestHoverOnCableManeuverServer>(&node);
    scheduler.registered_maneuvers_[iii_drone::control::maneuver::MANEUVER_TYPE_HOVER_ON_CABLE] =
        successor_server;
    iii_drone::control::maneuver::Maneuver successor_maneuver(
        iii_drone::control::maneuver::MANEUVER_TYPE_HOVER_ON_CABLE,
        {}
    );
    successor_maneuver.request_identity_ = kRequestB;
    successor_maneuver.started_ = true;
    successor_maneuver.terminated_ = false;
    scheduler.current_maneuver_.Store(successor_maneuver);
    scheduler.reference_callback_struct_->set(
        [](const State &) { return Reference(); },
        "hover_on_cable",
        successor.execution_id,
        kRequestB
    );

    scheduler.publishReferenceStream();

    EXPECT_TRUE(scheduler.reference_stream_state_.valid);
    EXPECT_EQ(scheduler.reference_stream_state_.execution_id, successor.execution_id);
    EXPECT_EQ(scheduler.reference_stream_state_.request_identity, kRequestB);
    EXPECT_NE(scheduler.reference_stream_state_.stream_id, "hover_on_cable:predecessor");
    EXPECT_EQ(scheduler.reference_stream_state_.sequence, 1U);
    const auto successor_stream_id = scheduler.reference_stream_state_.stream_id;
    scheduler.rebaseReferenceStream(rebase_request, rebase_response);
    EXPECT_FALSE(rebase_response->accepted);
    EXPECT_EQ(scheduler.reference_stream_state_.stream_id, successor_stream_id);
    EXPECT_EQ(scheduler.reference_stream_state_.sequence, 1U);
    EXPECT_FALSE(scheduler.reference_stream_state_.paused);
    }

    if (!context_was_initialized) {
        rclcpp::shutdown();
    }
}
