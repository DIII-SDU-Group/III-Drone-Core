#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_ros/buffer.h>

#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <iii_drone_configuration/configuration.hpp>
#include <iii_drone_core/control/combined_drone_awareness_handler.hpp>

using namespace std::chrono_literals;
using iii_drone::control::CombinedDroneAwarenessHandler;

namespace {

rclcpp::Parameter awarenessParameter(const std::string & name) {
    if (name.rfind("/tf/", 0) == 0) return rclcpp::Parameter(name, std::string("frame"));
    if (name.find("fail_on_unable_to_locate") != std::string::npos ||
        name.find("use_gripper_status_condition") != std::string::npos) {
        return rclcpp::Parameter(name, false);
    }
    if (name.find("_ms") != std::string::npos || name.find("window_size") != std::string::npos) {
        return rclcpp::Parameter(name, 100);
    }
    return rclcpp::Parameter(name, 0.5);
}

}  // namespace

// HIL soak runs 6 and 7: maneuver_controller's odometry callback shared the node's
// default (MutuallyExclusive) group with every other awareness callback and
// kept one sample, so a ~200 ms default-group callback silently dropped the
// PX4 samples in between and the object-tracking continuity guard failed a
// FlyToObject approach. Odometry ingress must keep a continuous sample
// sequence while the default group is busy.
TEST(OdometryIngress, StaysContinuousWhileTheDefaultCallbackGroupIsBlocked) {
    const bool initialized_here = !rclcpp::ok();
    if (initialized_here) rclcpp::init(0, nullptr);
    {
        auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("odometry_ingress_starvation_test");
        std::vector<iii_drone::configuration::configuration_entry_t> entries;
        for (const char * name : {
                "/control/maneuver_controller/combined_drone_awareness_pub_period_ms",
                "/control/maneuver_controller/fail_on_unable_to_locate",
                "/control/maneuver_controller/ground_estimate_update_period_ms",
                "/control/maneuver_controller/ground_estimate_window_size",
                "/control/maneuver_controller/ground_estimate_initial_delay_s",
                "/control/maneuver_controller/landed_altitude_threshold",
                "/control/maneuver_controller/landed_altitude_threshold_on_start",
                "/control/maneuver_controller/on_cable_max_euc_distance",
                "/control/maneuver_controller/use_gripper_status_condition",
                "/tf/cable_gripper_frame_id", "/tf/drone_frame_id",
                "/tf/ground_frame_id", "/tf/world_frame_id"}) {
            entries.push_back({name, awarenessParameter(name).get_type()});
        }
        auto configuration = std::make_shared<iii_drone::configuration::Configuration>(
            "odometry-ingress-starvation-test", entries, awarenessParameter);
        auto awareness = std::make_shared<CombinedDroneAwarenessHandler>(
            configuration, std::make_shared<tf2_ros::Buffer>(node->get_clock()), node.get());
        awareness->Start();

        // Occupies every thread of the node's executor for 300 ms (HIL run 7:
        // with its own callback group the ingress still waited ~190 ms for a
        // free pool thread): the default group and a second, reentrant one.
        std::atomic<bool> start_block{false};
        std::atomic<int> blocking{0};
        std::atomic<int> blocked{0};
        auto block_once = [&](std::atomic<bool> & done) {
            if (!start_block || done.exchange(true)) return;
            ++blocking;
            ++blocked;
            std::this_thread::sleep_for(300ms);
            --blocking;
        };
        std::atomic<bool> default_done{false};
        std::atomic<bool> reentrant_done{false};
        auto default_blocker = node->create_wall_timer(5ms, [&]() { block_once(default_done); });
        auto reentrant_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        auto reentrant_blocker = node->create_wall_timer(
            5ms, [&]() { block_once(reentrant_done); }, reentrant_group);

        rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
        executor.add_node(node->get_node_base_interface());
        std::thread spinner([&executor]() { executor.spin(); });

        auto publisher_node = std::make_shared<rclcpp::Node>("odometry_ingress_publisher");
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.best_effort().transient_local();  // as PX4's uXRCE-DDS publishers
        auto publisher = publisher_node->create_publisher<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos);
        while (publisher->get_subscription_count() == 0) std::this_thread::sleep_for(10ms);

        px4_msgs::msg::VehicleOdometry odometry;
        odometry.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
        odometry.q = {1.0f, 0.0f, 0.0f, 0.0f};
        uint64_t sample_us = 1000000;
        int published_while_blocked = 0;
        // Warm up, stall the default group for 300 ms, then 60 ms more: the
        // whole stall stays inside the 64-entry ingress history.
        const auto begin = std::chrono::steady_clock::now();
        const auto end = begin + 560ms;
        while (std::chrono::steady_clock::now() < end) {
            if (std::chrono::steady_clock::now() > begin + 200ms) start_block = true;
            sample_us += 8000;  // PX4 publishes vehicle_odometry at 125 Hz
            odometry.timestamp = sample_us;
            odometry.timestamp_sample = sample_us;
            publisher->publish(odometry);
            if (blocking == 2) ++published_while_blocked;
            std::this_thread::sleep_for(8ms);
        }
        std::this_thread::sleep_for(100ms);
        ASSERT_EQ(blocked.load(), 2);
        ASSERT_GT(published_while_blocked, 20);
        ASSERT_EQ(blocking.load(), 0);

        const auto diagnostics = awareness->TryGetOdometryIngressDiagnostics();
        ASSERT_TRUE(diagnostics.available);
        ASSERT_GT(diagnostics.history_count, 32u);
        uint64_t max_interval_us = 0;
        int64_t max_entry_gap_ns = 0;
        for (size_t i = 1; i < diagnostics.history_count; ++i) {
            const auto & previous = diagnostics.history[i - 1];
            const auto & current = diagnostics.history[i];
            max_entry_gap_ns = std::max(max_entry_gap_ns,
                current.callback_entry_steady_ns - previous.callback_entry_steady_ns);
            if (current.source_sample_timestamp_us > previous.source_sample_timestamp_us) {
                max_interval_us = std::max(max_interval_us,
                    current.source_sample_timestamp_us - previous.source_sample_timestamp_us);
            }
        }
        // Best-effort loopback may drop a rare sample; a 300 ms default-group
        // stall must not.
        EXPECT_LE(max_interval_us, 40000u);
        // And they are ingested promptly, not as a late backlog.
        EXPECT_LE(max_entry_gap_ns, 40000000);

        executor.cancel();
        spinner.join();
        awareness->Stop();
    }
    if (initialized_here) rclcpp::shutdown();
}
