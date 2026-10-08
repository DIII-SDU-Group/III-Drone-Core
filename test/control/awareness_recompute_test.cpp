#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/buffer.h>

#include <iii_drone_interfaces/msg/gripper_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

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
    if (name.find("_ms") != std::string::npos) return rclcpp::Parameter(name, 100);
    if (name.find("window_size") != std::string::npos) return rclcpp::Parameter(name, 5);
    return rclcpp::Parameter(name, 0.5);
}

// A disarmed vehicle on the ground: the handler, its executor and PX4-like
// publishers of status, odometry and local position.
class GroundedAwareness {
public:
    explicit GroundedAwareness(const std::string & name) {
        node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(name);
        std::vector<iii_drone::configuration::configuration_entry_t> entries;
        for (const char * parameter : {
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
            entries.push_back({parameter, awarenessParameter(parameter).get_type()});
        }
        auto configuration = std::make_shared<iii_drone::configuration::Configuration>(
            name, entries, awarenessParameter);
        awareness_ = std::make_shared<CombinedDroneAwarenessHandler>(
            configuration, std::make_shared<tf2_ros::Buffer>(node_->get_clock()), node_.get());
        awareness_->Start();
        executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
            rclcpp::ExecutorOptions(), 2);
        executor_->add_node(node_->get_node_base_interface());
        spinner_ = std::thread([this]() { executor_->spin(); });

        publisher_node_ = std::make_shared<rclcpp::Node>(name + "_px4");
        rclcpp::QoS px4_qos(rclcpp::KeepLast(1));
        px4_qos.best_effort().transient_local();  // as PX4's uXRCE-DDS publishers
        status_ = publisher_node_->create_publisher<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status_v1", px4_qos);
        odometry_ = publisher_node_->create_publisher<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", px4_qos);
        local_position_ = publisher_node_->create_publisher<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", px4_qos);
        gripper_ = publisher_node_->create_publisher<iii_drone_interfaces::msg::GripperStatus>(
            "/payload/charger_gripper/gripper_status", 10);
        for (const auto & publisher : std::vector<rclcpp::PublisherBase *>{
                status_.get(), odometry_.get(), local_position_.get(), gripper_.get()}) {
            while (publisher->get_subscription_count() == 0) std::this_thread::sleep_for(10ms);
        }
    }

    ~GroundedAwareness() {
        executor_->cancel();
        spinner_.join();
        awareness_->Stop();
    }

    // One PX4 cycle: status, local position (reference altitude
    // reference_altitude_amsl) and odometry at the ground.
    void publishCycle(double reference_altitude_amsl) {
        sample_us_ += 10000;
        px4_msgs::msg::VehicleStatus status;
        status.timestamp = sample_us_;
        status.arming_state = px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED;
        status.nav_state = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER;
        status_->publish(status);

        px4_msgs::msg::VehicleLocalPosition local;
        local.timestamp = sample_us_;
        local.timestamp_sample = sample_us_;
        local.xy_valid = local.z_valid = local.v_xy_valid = local.v_z_valid = true;
        local.xy_global = local.z_global = true;
        local.ref_timestamp = 1000;
        local.ref_lat = 55.4;
        local.ref_lon = 10.4;
        local.ref_alt = static_cast<float>(reference_altitude_amsl);
        local_position_->publish(local);

        px4_msgs::msg::VehicleOdometry odometry;
        odometry.timestamp = sample_us_;
        odometry.timestamp_sample = sample_us_;
        odometry.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
        odometry.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
        odometry.q = {1.0f, 0.0f, 0.0f, 0.0f};
        odometry_->publish(odometry);
    }

    void publishGripper(bool open) {
        iii_drone_interfaces::msg::GripperStatus status;
        status.gripper_status = open
            ? iii_drone_interfaces::msg::GripperStatus::GRIPPER_STATUS_OPEN
            : iii_drone_interfaces::msg::GripperStatus::GRIPPER_STATUS_CLOSED;
        gripper_->publish(status);
    }

    CombinedDroneAwarenessHandler & awareness() { return *awareness_; }
    rclcpp::Node & publisher_node() { return *publisher_node_; }

private:
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    CombinedDroneAwarenessHandler::SharedPtr awareness_;
    std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    std::thread spinner_;
    rclcpp::Node::SharedPtr publisher_node_;
    rclcpp::Publisher<px4_msgs::msg::VehicleStatus>::SharedPtr status_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_;
    rclcpp::Publisher<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_;
    rclcpp::Publisher<iii_drone_interfaces::msg::GripperStatus>::SharedPtr gripper_;
    uint64_t sample_us_ = 1000000;
};

class RclcppScope {
public:
    RclcppScope() : initialized_here_(!rclcpp::ok()) {
        if (initialized_here_) rclcpp::init(0, nullptr);
    }
    ~RclcppScope() {
        if (initialized_here_) rclcpp::shutdown();
    }
private:
    bool initialized_here_;
};

}  // namespace

// The ground's AMSL altitude used to come from a 100 Hz global-position
// subscription; PX4 derives the global altitude from the local position's
// reference altitude, which the handler already receives.
TEST(AwarenessRecompute, GroundAmslIsTheEstimatePlusTheLocalReferenceAltitude) {
    RclcppScope rclcpp_scope;
    GroundedAwareness grounded("awareness_ground_amsl_test");
    const auto end = std::chrono::steady_clock::now() + 1500ms;
    while (std::chrono::steady_clock::now() < end) {
        grounded.publishCycle(488.25);
        std::this_thread::sleep_for(10ms);
    }
    const auto adapter = grounded.awareness().adapter();
    EXPECT_NEAR(adapter.ground_altitude_estimate(), 0.0, 1e-6);
    EXPECT_NEAR(adapter.ground_altitude_estimate_amsl(), 488.25, 1e-3);
}

// Every /tf listener receives each ground frame; only rviz shows it. With a
// steady estimate it is published once and then once a second, not at the
// estimate's 10 Hz (test parameters) or 20 Hz (flight parameters).
TEST(AwarenessRecompute, SteadyGroundFrameIsPublishedOnceASecond) {
    RclcppScope rclcpp_scope;
    GroundedAwareness grounded("awareness_ground_frame_test");
    std::atomic<int> ground_frames{0};
    auto subscription = grounded.publisher_node().create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", 100, [&ground_frames](const tf2_msgs::msg::TFMessage::SharedPtr message) {
            ground_frames += static_cast<int>(message->transforms.size());
        });
    rclcpp::executors::SingleThreadedExecutor listener;
    listener.add_node(grounded.publisher_node().get_node_base_interface());
    // Settle the estimate, then count over two seconds.
    auto publish_for = [&](std::chrono::milliseconds duration) {
        const auto end = std::chrono::steady_clock::now() + duration;
        while (std::chrono::steady_clock::now() < end) {
            grounded.publishCycle(488.25);
            listener.spin_some(5ms);
            std::this_thread::sleep_for(5ms);
        }
    };
    publish_for(1000ms);
    ground_frames = 0;
    publish_for(2000ms);
    EXPECT_GE(ground_frames.load(), 1);
    EXPECT_LE(ground_frames.load(), 3);
}

// Gripper status arrives at 50-100 Hz and recomputes the awareness only when
// open/closed changes: every change must still reach it.
TEST(AwarenessRecompute, GripperChangesReachTheAwareness) {
    RclcppScope rclcpp_scope;
    GroundedAwareness grounded("awareness_gripper_test");
    auto expect_gripper = [&](bool open) {
        const auto end = std::chrono::steady_clock::now() + 1000ms;
        while (std::chrono::steady_clock::now() < end) {
            grounded.publishCycle(488.25);
            grounded.publishGripper(open);
            std::this_thread::sleep_for(10ms);
            if (grounded.awareness().gripper_open() == open) return true;
        }
        return false;
    };
    EXPECT_TRUE(expect_gripper(false));
    EXPECT_TRUE(expect_gripper(true));
    EXPECT_TRUE(expect_gripper(false));
}
