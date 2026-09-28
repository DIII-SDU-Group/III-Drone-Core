#include <chrono>
#include <atomic>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <iii_drone_core/adapters/reference_adapter.hpp>
#include <iii_drone_core/control/maneuver/object_tracking_session.hpp>
#include <iii_drone_core/control/trajectory_generator_client.hpp>

namespace {

using iii_drone::configuration::Configuration;
using iii_drone::configuration::configuration_entry_t;
using iii_drone::control::Reference;
using iii_drone::control::State;
using iii_drone::control::TrajectoryGeneratorClient;
using iii_drone::control::positional;
using iii_drone::types::point_t;
using iii_drone::types::vector_t;
using ComputeTrajectory = iii_drone_interfaces::srv::ComputeReferenceTrajectory;

constexpr char kServiceName[] =
    "/control/trajectory_generator/compute_reference_trajectory";

class RclcppContext {
public:
    RclcppContext() : initialized_here_(!rclcpp::ok()) {
        if (initialized_here_) {
            rclcpp::init(0, nullptr);
        }
    }

    ~RclcppContext() {
        if (initialized_here_) {
            rclcpp::shutdown();
        }
    }

private:
    bool initialized_here_;
};

Configuration::SharedPtr makeConfiguration(bool asynchronous = true, double control_step_s = 0.2) {
    const std::vector<configuration_entry_t> entries{
        {
            "/control/maneuver_controller/generate_trajectories_asynchronously_with_delay",
            rclcpp::ParameterType::PARAMETER_BOOL
        },
        {"/control/dt", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/generate_trajectories_poll_period_ms",
            rclcpp::ParameterType::PARAMETER_INTEGER},
        {"/control/maneuver_controller/generate_trajectories_timeout_ms",
            rclcpp::ParameterType::PARAMETER_INTEGER},
    };
    return std::make_shared<Configuration>(
        "trajectory_generator_client_test",
        entries,
        [=](const std::string & name) {
            if (name == "/control/dt") {
                return rclcpp::Parameter(name, control_step_s);
            }
            if (name == "/control/maneuver_controller/generate_trajectories_poll_period_ms") {
                return rclcpp::Parameter(name, 1);
            }
            if (name == "/control/maneuver_controller/generate_trajectories_timeout_ms") {
                return rclcpp::Parameter(name, 1000);
            }
            return rclcpp::Parameter(name, asynchronous);
        }
    );
}

Reference responseReference(size_t request_number) {
    const double offset = static_cast<double>(request_number * 10U);
    return Reference(
        point_t(50.0 + offset, 60.0, 70.0),
        0.7,
        vector_t(1.0, 2.0, 3.0),
        0.4,
        vector_t::Zero(),
        0.0,
        rclcpp::Time(200 + static_cast<int32_t>(request_number), 0)
    );
}

bool spinUntil(
    rclcpp::executors::SingleThreadedExecutor & executor,
    const std::function<bool()> & condition
) {
    for (int attempt = 0; attempt < 100; ++attempt) {
        executor.spin_some();
        if (condition()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return condition();
}

}  // namespace

TEST(TrajectoryGeneratorClientTest, AsyncResetSeedsMovingStateAndFencesOldResponse) {
    RclcppContext context;
    rclcpp_lifecycle::LifecycleNode node("trajectory_generator_client_seed_test");
    size_t service_request_count = 0U;
    auto service = node.create_service<ComputeTrajectory>(
        kServiceName,
        [&service_request_count](
            const ComputeTrajectory::Request::SharedPtr,
            ComputeTrajectory::Response::SharedPtr response
        ) {
            ++service_request_count;
            const Reference result = responseReference(service_request_count);
            response->success = true;
            response->reference_trajectory.references.push_back(
                iii_drone::adapters::ReferenceAdapter(result).ToMsg()
            );
        }
    );
    auto callback_group = node.create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    TrajectoryGeneratorClient client(&node, makeConfiguration(), callback_group);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node.get_node_base_interface());

    const rclcpp::Time moving_stamp(123, 456);
    const State moving_state(
        point_t(1.0, -2.0, 3.0),
        vector_t(0.1, -0.2, 0.3),
        0.6,
        vector_t(0.01, -0.02, 0.4),
        moving_stamp
    );
    const Reference target(point_t(4.0, 5.0, 6.0), -0.2);

    const Reference initial_result = client.ComputeReference(
        moving_state, target, true, true, positional, true
    );
    EXPECT_TRUE(initial_result.position().isApprox(moving_state.position()));
    EXPECT_NEAR(initial_result.yaw(), moving_state.yaw(), 1.0e-6);
    EXPECT_EQ(initial_result.stamp().nanoseconds(), moving_stamp.nanoseconds());
    EXPECT_TRUE(initial_result.velocity().array().isNaN().all());
    EXPECT_TRUE(std::isnan(initial_result.yaw_rate()));
    EXPECT_TRUE(initial_result.acceleration().array().isNaN().all());
    EXPECT_TRUE(std::isnan(initial_result.yaw_acceleration()));

    // The solver request still uses the complete moving-state seed. Only the
    // published startup reference is reduced to a position/yaw hold.
    const Reference internal_seed = client.GetReferenceTrajectory().references()[0];
    EXPECT_TRUE(internal_seed.velocity().isApprox(moving_state.velocity()));
    EXPECT_NEAR(internal_seed.yaw_rate(), moving_state.angular_velocity().z(), 1.0e-12);
    EXPECT_EQ(internal_seed.stamp().nanoseconds(), moving_stamp.nanoseconds());
    EXPECT_TRUE(client.busy());

    // Reset while the service result is pending. Its response belongs to the
    // old generation and must not replace the newer moving-state seed.
    const State successor_state(
        point_t(-7.0, 8.0, 9.0),
        vector_t(-0.4, 0.5, -0.6),
        -0.9,
        vector_t(0.03, 0.02, -0.7),
        rclcpp::Time(321, 654)
    );
    client.Reset(successor_state);
    ASSERT_TRUE(spinUntil(executor, [&service_request_count] {
        return service_request_count == 1U;
    }));
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    executor.spin_some();

    const Reference after_stale_response = client.ComputeReference(
        successor_state, target, true, false, positional, true
    );
    EXPECT_TRUE(after_stale_response.position().isApprox(successor_state.position()));
    EXPECT_TRUE(after_stale_response.velocity().array().isNaN().all());
    EXPECT_TRUE(std::isnan(after_stale_response.yaw_rate()));
    EXPECT_TRUE(after_stale_response.acceleration().array().isNaN().all());
    EXPECT_TRUE(std::isnan(after_stale_response.yaw_acceleration()));

    const Reference successor_seed = client.GetReferenceTrajectory().references()[0];
    EXPECT_TRUE(successor_seed.velocity().isApprox(successor_state.velocity()));
    EXPECT_NEAR(successor_seed.yaw_rate(), successor_state.angular_velocity().z(), 1.0e-12);
    EXPECT_EQ(
        successor_seed.stamp().nanoseconds(),
        successor_state.stamp().nanoseconds()
    );

    // A subsequent public async request still completes and publishes its
    // own service result into the active generation.
    ASSERT_TRUE(spinUntil(executor, [&client, &service_request_count] {
        return service_request_count == 2U && client.done();
    }));
    const Reference completed_result = client.GetReferenceTrajectory().references()[0];
    EXPECT_NEAR(completed_result.position().x(), 70.0, 1.0e-12);

    // A subsequent replan may be pending, but the usable planned trajectory
    // remains visible instead of reverting to the startup hold.
    const State later_state(
        successor_state.position(), successor_state.velocity(), successor_state.yaw(),
        successor_state.angular_velocity(),
        successor_state.stamp() + rclcpp::Duration::from_seconds(0.2)
    );
    const Reference during_replan = client.ComputeReference(
        later_state, target, true, false, positional, true
    );
    EXPECT_NEAR(during_replan.position().x(), 70.0, 1.0e-12);
    EXPECT_TRUE(during_replan.velocity().array().isFinite().all());
    EXPECT_FALSE(std::isnan(during_replan.yaw_rate()));
    EXPECT_TRUE(client.busy());

    executor.remove_node(node.get_node_base_interface());
}

TEST(TrajectoryGeneratorClientTest, FailedEmptyReplanPreservesCacheAndResetClearsFailure) {
    RclcppContext context;
    rclcpp_lifecycle::LifecycleNode node("trajectory_generator_client_empty_response_test");
    size_t service_request_count = 0U;
    auto service = node.create_service<ComputeTrajectory>(
        kServiceName,
        [&service_request_count](
            const ComputeTrajectory::Request::SharedPtr,
            ComputeTrajectory::Response::SharedPtr response
        ) {
            ++service_request_count;
            if (service_request_count == 2U) {
                response->success = false;
                // Even an empty service error must be surfaced as a failure.
                return;
            }
            response->success = true;
            response->reference_trajectory.references.push_back(
                iii_drone::adapters::ReferenceAdapter(
                    responseReference(service_request_count)
                ).ToMsg()
            );
        }
    );
    auto callback_group = node.create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    TrajectoryGeneratorClient client(&node, makeConfiguration(), callback_group);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node.get_node_base_interface());

    const State first_state(
        point_t(1.0, 2.0, 3.0), vector_t(0.0, 0.0, 0.0), 0.2,
        vector_t(0.0, 0.0, 0.0), rclcpp::Time(10, 0)
    );
    const Reference target(point_t(4.0, 5.0, 6.0), -0.2);
    const Reference hold = client.ComputeReference(
        first_state, target, true, true, positional, true
    );
    EXPECT_TRUE(hold.velocity().array().isNaN().all());
    ASSERT_TRUE(spinUntil(executor, [&client, &service_request_count] {
        return service_request_count == 1U && client.done();
    }));

    const Reference first_plan = client.ComputeReference(
        first_state, target, true, false, positional, true
    );
    EXPECT_NEAR(first_plan.position().x(), 60.0, 1.0e-12);

    const State later_state(
        first_state.position(), first_state.velocity(), first_state.yaw(),
        first_state.angular_velocity(), first_state.stamp() + rclcpp::Duration::from_seconds(0.2)
    );
    const Reference cached_during_replan = client.ComputeReference(
        later_state, target, true, false, positional, true
    );
    EXPECT_NEAR(cached_during_replan.position().x(), 60.0, 1.0e-12);
    ASSERT_TRUE(spinUntil(executor, [&client, &service_request_count] {
        return service_request_count == 2U && client.done();
    }));

    EXPECT_THROW(
        client.ComputeReference(later_state, target, true, false, positional, true),
        std::runtime_error
    );
    const Reference preserved_plan = client.GetReferenceTrajectory().references()[0];
    EXPECT_NEAR(preserved_plan.position().x(), 60.0, 1.0e-12);

    const State recovery_state(
        point_t(-1.0, -2.0, -3.0), vector_t(0.1, 0.2, 0.3), -0.4,
        vector_t(0.0, 0.0, 0.5), rclcpp::Time(11, 0)
    );
    const Reference recovery_hold = client.ComputeReference(
        recovery_state, target, true, true, positional, true
    );
    EXPECT_TRUE(recovery_hold.velocity().array().isNaN().all());
    ASSERT_TRUE(spinUntil(executor, [&client, &service_request_count] {
        return service_request_count == 3U && client.done();
    }));
    const Reference recovered_plan = client.ComputeReference(
        recovery_state, target, true, false, positional, true
    );
    EXPECT_NEAR(recovered_plan.position().x(), 80.0, 1.0e-12);

    executor.remove_node(node.get_node_base_interface());
}

TEST(TrajectoryGeneratorClientTest, MpcRequestsFollowConfiguredControlStep) {
    RclcppContext context;
    rclcpp_lifecycle::LifecycleNode node("trajectory_generator_client_cadence_test");
    size_t service_request_count = 0U;
    auto service = node.create_service<ComputeTrajectory>(
        kServiceName,
        [&service_request_count](
            const ComputeTrajectory::Request::SharedPtr,
            ComputeTrajectory::Response::SharedPtr response
        ) {
            ++service_request_count;
            const Reference result = responseReference(service_request_count);
            response->success = true;
            response->reference_trajectory.references.push_back(
                iii_drone::adapters::ReferenceAdapter(result).ToMsg()
            );
        }
    );
    auto callback_group = node.create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    TrajectoryGeneratorClient client(&node, makeConfiguration(), callback_group);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node.get_node_base_interface());

    const Reference target(point_t(4.0, 5.0, 6.0), -0.2);
    const std::vector<int64_t> timestamps_ns{
        1'000'000'000, 1'050'000'000, 1'100'000'000, 1'150'000'000,
        1'200'000'000, 1'250'000'000, 1'300'000'000, 1'350'000'000,
        1'400'000'000,
    };
    for (size_t index = 0; index < timestamps_ns.size(); ++index) {
        const State state(
            point_t(0.0, 0.0, 1.0),
            vector_t::Zero(),
            0.0,
            vector_t::Zero(),
            rclcpp::Time(timestamps_ns[index])
        );
        const Reference output = client.ComputeReference(
            state, target, true, index == 0U, positional, true
        );
        EXPECT_TRUE(output.position().allFinite());
        ASSERT_TRUE(spinUntil(executor, [&client] { return !client.busy(); }));
    }

    EXPECT_EQ(service_request_count, 3U);
    executor.remove_node(node.get_node_base_interface());
}

TEST(TrajectoryGeneratorClientTest, ResetBackwardTimestampAndCancelReopenMpcAdmission) {
    RclcppContext context;
    rclcpp_lifecycle::LifecycleNode node("trajectory_generator_client_clock_reset_test");
    std::atomic_size_t service_request_count{0U};
    auto service = node.create_service<ComputeTrajectory>(
        kServiceName,
        [&service_request_count](
            const ComputeTrajectory::Request::SharedPtr,
            ComputeTrajectory::Response::SharedPtr response
        ) {
            const size_t request_number = ++service_request_count;
            const Reference result = responseReference(request_number);
            response->success = true;
            response->reference_trajectory.references.push_back(
                iii_drone::adapters::ReferenceAdapter(result).ToMsg()
            );
        }
    );
    auto callback_group = node.create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    TrajectoryGeneratorClient client(&node, makeConfiguration(), callback_group);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node.get_node_base_interface());

    const Reference target(point_t(4.0, 5.0, 6.0), -0.2);
    auto call = [&](int64_t stamp_ns, bool reset = false) {
        const State state(
            point_t(0.0, 0.0, 1.0), vector_t::Zero(), 0.0, vector_t::Zero(),
            rclcpp::Time(stamp_ns)
        );
        static_cast<void>(client.ComputeReference(state, target, true, reset, positional, true));
        ASSERT_TRUE(spinUntil(executor, [&client] { return !client.busy(); }));
    };

    call(2'000'000'000, true);   // First call after Reset is immediate.
    call(2'050'000'000);         // Too early.
    call(2'100'000'000, true);   // Explicit reset starts a new cadence.
    call(2'150'000'000);         // Too early after reset.
    call(1'500'000'000);         // Backwards ROS clock jump admits once and resets origin.
    call(1'550'000'000);         // Too early on the new timeline.
    client.Cancel();
    call(1'560'000'000);         // Cancellation also reopens admission.

    EXPECT_EQ(service_request_count.load(), 4U);
    executor.remove_node(node.get_node_base_interface());
}

TEST(TrajectoryGeneratorClientTest, InterpolationRequestsAreNotMpcRateLimited) {
    RclcppContext context;
    rclcpp_lifecycle::LifecycleNode node("trajectory_generator_client_interpolation_test");
    std::atomic_size_t service_request_count{0U};
    auto service = node.create_service<ComputeTrajectory>(
        kServiceName,
        [&service_request_count](
            const ComputeTrajectory::Request::SharedPtr,
            ComputeTrajectory::Response::SharedPtr response
        ) {
            const size_t request_number = ++service_request_count;
            const Reference result = responseReference(request_number);
            response->success = true;
            response->reference_trajectory.references.push_back(
                iii_drone::adapters::ReferenceAdapter(result).ToMsg()
            );
        }
    );
    auto callback_group = node.create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    TrajectoryGeneratorClient client(&node, makeConfiguration(), callback_group);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node.get_node_base_interface());
    std::thread spin_thread([&executor] { executor.spin(); });

    const rclcpp::Time stamp(3'000'000'000);
    const Reference start(point_t(0.0, 0.0, 1.0), 0.0, vector_t::Zero(), 0.0,
        vector_t::Zero(), 0.0, stamp);
    const Reference target(point_t(1.0, 0.0, 1.0), 0.0, vector_t::Zero(), 0.0,
        vector_t::Zero(), 0.0, stamp);
    static_cast<void>(client.ComputeReference(start, target, true, true, positional));
    static_cast<void>(client.ComputeReference(start, target, true, false, positional));

    executor.cancel();
    spin_thread.join();
    EXPECT_EQ(service_request_count.load(), 2U);
    executor.remove_node(node.get_node_base_interface());
}

TEST(TrajectoryGeneratorClientTest, ObjectTrackingStaleCauseThroughBoundedPositionalRpc) {
    using Session = iii_drone::control::maneuver::ObjectTrackingSession;
    using iii_drone::control::MeasuredOdometrySnapshot;
    using iii_drone::control::bounded_positional;
    RclcppContext context;
    rclcpp_lifecycle::LifecycleNode node("object_tracking_stale_rpc_test");
    std::atomic<int> service_delay_ms{270};
    std::atomic<int> bounded_requests{0};
    auto stamp = [](int64_t nanoseconds) {
        return rclcpp::Time(nanoseconds, RCL_ROS_TIME);
    };
    const Reference seed(point_t(0.0f, 0.0f, 2.0f), 0.0, vector_t::Zero(),
        0.0, vector_t::Zero(), 0.0, stamp(100'000'000'000LL));
    auto service = node.create_service<ComputeTrajectory>(
        kServiceName,
        [&](const ComputeTrajectory::Request::SharedPtr request,
            ComputeTrajectory::Response::SharedPtr response) {
            if (request->trajectory_mode.mode == bounded_positional &&
                request->use_start_reference) {
                ++bounded_requests;
            }
            const int delay = service_delay_ms.load();
            if (delay > 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(delay));
            }
            response->success = true;
            response->reference_trajectory.references.push_back(
                iii_drone::adapters::ReferenceAdapter(seed).ToMsg());
        });
    auto callback_group = node.create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    TrajectoryGeneratorClient client(&node, makeConfiguration(), callback_group);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node.get_node_base_interface());
    std::thread spin_thread([&executor] { executor.spin(); });

    const std::string owner = "mri1-object-stale-rpc-0000000000000001";
    const Reference target(point_t(0.2f, 0.0f, 2.0f), 0.0, vector_t::Zero(),
        0.0, vector_t::Zero(), 0.0, stamp(100'000'000'000LL));
    auto planner = [&](const Reference & start, const Reference & goal, bool reset) {
        return client.ComputeReference(start, goal, true, reset, bounded_positional);
    };
    auto measuredAt = [&](int64_t receipt_ns, uint64_t sample_us) {
        MeasuredOdometrySnapshot measured;
        measured.state = State(seed.position(), vector_t::Zero(), 0.0,
            vector_t::Zero(), stamp(receipt_ns));
        measured.receipt_stamp = stamp(receipt_ns);
        measured.source_sample_timestamp_us = sample_us;
        return measured;
    };
    Reference output;
    std::string reason;

    // The real blocking client call takes longer than the unchanged command
    // freshness budget, while the next measured sample itself is fresh.
    Session delayed(planner, seed, owner, 1, stamp(100'000'000'000LL),
        1.0, Session::Limits{});
    EXPECT_TRUE(delayed.Compute(target,
        measuredAt(100'000'000'000LL, 100'000'000), stamp(100'000'000'000LL),
        owner, 1, 1.0, 0.4, output, reason)) << reason;
    EXPECT_FALSE(delayed.Compute(target,
        measuredAt(100'300'000'000LL, 100'300'000), stamp(100'300'000'000LL),
        owner, 1, 1.0, 0.4, output, reason));
    EXPECT_NE(reason.find("odometry_stale=false"), std::string::npos) << reason;
    EXPECT_NE(reason.find("command_stale=true"), std::string::npos) << reason;
    EXPECT_NE(reason.find("odometry_age_s=0.000000"), std::string::npos) << reason;
    EXPECT_NE(reason.find("command_age_s=0.300000"), std::string::npos) << reason;
    const std::string planner_field = "prior_planner_rpc_ms=";
    const auto planner_field_at = reason.find(planner_field);
    EXPECT_NE(planner_field_at, std::string::npos) << reason;
    if (planner_field_at != std::string::npos) {
        EXPECT_GE(std::stod(reason.substr(planner_field_at + planner_field.size())), 250.0);
    }
    EXPECT_NE(reason.find("prior_stop_certification_ms="), std::string::npos) << reason;
    EXPECT_NE(reason.find("emission_ros_ns=100300000000"), std::string::npos) << reason;
    EXPECT_NE(reason.find("emission_clock_type=" +
        std::to_string(static_cast<int>(RCL_ROS_TIME))), std::string::npos) << reason;
    EXPECT_NE(reason.find("last_command_ros_ns=100000000000"), std::string::npos) << reason;
    EXPECT_NE(reason.find("receipt_ros_ns=100300000000"), std::string::npos) << reason;
    EXPECT_NE(reason.find("source_sample_us=100300000"), std::string::npos) << reason;
    EXPECT_NE(reason.find("reset_counter=0"), std::string::npos) << reason;

    // A duplicate PX4 sample retains its original receipt stamp; the newly
    // emitted command remains fresh when that measured age crosses 250 ms.
    service_delay_ms = 0;
    Session duplicate(planner, seed.CopyWithNewStamp(stamp(200'000'000'000LL)),
        owner, 2, stamp(200'000'000'000LL), 1.0, Session::Limits{});
    EXPECT_TRUE(duplicate.Compute(target,
        measuredAt(199'760'000'000LL, 199'760'000), stamp(200'000'000'000LL),
        owner, 2, 1.0, 0.4, output, reason)) << reason;
    EXPECT_FALSE(duplicate.Compute(target,
        measuredAt(199'760'000'000LL, 199'760'000), stamp(200'020'000'000LL),
        owner, 2, 1.0, 0.4, output, reason));
    EXPECT_NE(reason.find("odometry_stale=true"), std::string::npos) << reason;
    EXPECT_NE(reason.find("command_stale=false"), std::string::npos) << reason;
    EXPECT_NE(reason.find("odometry_age_s=0.260000"), std::string::npos) << reason;
    EXPECT_NE(reason.find("command_age_s=0.020000"), std::string::npos) << reason;
    EXPECT_EQ(bounded_requests.load(), 2);

    executor.cancel();
    spin_thread.join();
    executor.remove_node(node.get_node_base_interface());
}

TEST(TrajectoryGeneratorClientTest, BlockingMpcRequestsFollowConfiguredControlStep) {
    RclcppContext context;
    rclcpp_lifecycle::LifecycleNode node("trajectory_generator_client_blocking_cadence_test");
    std::atomic_size_t service_request_count{0U};
    auto service = node.create_service<ComputeTrajectory>(
        kServiceName,
        [&service_request_count](
            const ComputeTrajectory::Request::SharedPtr,
            ComputeTrajectory::Response::SharedPtr response
        ) {
            const size_t request_number = ++service_request_count;
            const Reference result = responseReference(request_number);
            response->success = true;
            response->reference_trajectory.references.push_back(
                iii_drone::adapters::ReferenceAdapter(result).ToMsg()
            );
        }
    );
    auto callback_group = node.create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    TrajectoryGeneratorClient client(&node, makeConfiguration(false), callback_group);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node.get_node_base_interface());
    std::thread spin_thread([&executor] { executor.spin(); });

    const Reference target(point_t(4.0, 5.0, 6.0), -0.2);
    for (size_t index = 0; index < 5U; ++index) {
        const State state(
            point_t(0.0, 0.0, 1.0), vector_t::Zero(), 0.0, vector_t::Zero(),
            rclcpp::Time(4'000'000'000 + static_cast<int64_t>(index) * 50'000'000)
        );
        const Reference output = client.ComputeReference(
            state, target, true, index == 0U, positional, true
        );
        EXPECT_TRUE(output.position().allFinite());
    }

    executor.cancel();
    spin_thread.join();
    EXPECT_EQ(service_request_count.load(), 2U);
    executor.remove_node(node.get_node_base_interface());
}
