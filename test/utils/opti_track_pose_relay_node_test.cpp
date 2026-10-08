#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <iii_drone_core/utils/opti_track_pose_relay_node/opti_track_pose_relay_node.hpp>

using namespace std::chrono_literals;
using namespace iii_drone::utils::opti_track_pose_relay;
using namespace iii_drone::utils::opti_track_pose_relay_node;

namespace {

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

std::string valueOf(const diagnostic_msgs::msg::DiagnosticStatus & status, const std::string & key) {
    for (const auto & entry : status.values) {
        if (entry.key == key) {
            return entry.value;
        }
    }
    return "<missing>";
}

rclcpp::NodeOptions relayOptions(std::vector<rclcpp::Parameter> overrides) {
    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides);
    return options;
}

std::string configurationError(const rclcpp::NodeOptions & options) {
    OptiTrackPoseRelayNode node("pose_relay_parameter_test", "/opti_track_test", options);
    return node.configuration_error();
}

}  // namespace

TEST(OptiTrackPoseRelayMessages, VisualOdometryIsANedPoseWithUnknownVelocity) {
    NedPose pose;
    pose.position = {1.0, -2.0, -3.0};
    pose.orientation = {0.5, 0.5, -0.5, -0.5};
    const auto message = MakeVisualOdometry(pose, 1'791'000'000'123'456ULL, 1.0e-4, 4.0e-4);

    EXPECT_EQ(message.timestamp, 1'791'000'000'123'456ULL);
    EXPECT_EQ(message.timestamp_sample, message.timestamp);
    EXPECT_EQ(message.pose_frame, px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED);
    EXPECT_FLOAT_EQ(message.position[0], 1.0F);
    EXPECT_FLOAT_EQ(message.position[1], -2.0F);
    EXPECT_FLOAT_EQ(message.position[2], -3.0F);
    EXPECT_FLOAT_EQ(message.q[0], 0.5F);
    EXPECT_FLOAT_EQ(message.q[1], 0.5F);
    EXPECT_FLOAT_EQ(message.q[2], -0.5F);
    EXPECT_FLOAT_EQ(message.q[3], -0.5F);
    EXPECT_EQ(message.velocity_frame, px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN);
    for (std::size_t i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isnan(message.velocity[i]));
        EXPECT_TRUE(std::isnan(message.angular_velocity[i]));
        EXPECT_TRUE(std::isnan(message.velocity_variance[i]));
        EXPECT_FLOAT_EQ(message.position_variance[i], 1.0e-4F);
        EXPECT_FLOAT_EQ(message.orientation_variance[i], 4.0e-4F);
    }
    EXPECT_EQ(message.reset_counter, 0);
    EXPECT_EQ(message.quality, 0);
}

TEST(OptiTrackPoseRelayMessages, OriginCommandCarriesLatitudeLongitudeAltitude) {
    const auto command = MakeSetGpsGlobalOriginCommand(55.3672, 10.4310, 20.0, 123'456, 1, 1);
    EXPECT_EQ(command.command, px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN);
    EXPECT_DOUBLE_EQ(command.param5, 55.3672);
    EXPECT_DOUBLE_EQ(command.param6, 10.4310);
    EXPECT_FLOAT_EQ(command.param7, 20.0F);
    EXPECT_FLOAT_EQ(command.param1, 0.0F);
    EXPECT_EQ(command.timestamp, 123'456u);
    EXPECT_EQ(command.target_system, 1);
    EXPECT_EQ(command.target_component, 1);
    EXPECT_EQ(command.source_system, 1);
    EXPECT_EQ(command.source_component, 1);
    EXPECT_EQ(command.confirmation, 0);
    EXPECT_TRUE(command.from_external);
}

TEST(OptiTrackPoseRelayMessages, HealthCarriesEveryValue) {
    HealthReport report;
    report.level = HealthLevel::WARN;
    report.message = "input gap 230 ms";
    report.input_rate_hz = 119.96;
    report.output_rate_hz = 50.0;
    report.last_input_age_ms = 4.04;
    report.max_input_gap_ms = 230.0;
    report.stale = false;
    report.rejected_samples = 3;
    const auto status = MakeHealthStatus(report, true, 7);

    EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
    EXPECT_EQ(status.name, "opti_track_pose_relay");
    EXPECT_EQ(status.message, "input gap 230 ms");
    const std::vector<std::pair<std::string, std::string>> expected{
        {"input_rate_hz", "120.0"},
        {"output_rate_hz", "50.0"},
        {"last_input_age_ms", "4.0"},
        {"max_input_gap_ms", "230.0"},
        {"lab_stamp_age_ms", "nan"},
        {"stale", "false"},
        {"origin_sent", "true"},
        {"rigid_body_id", "7"},
        {"rejected_samples", "3"},
    };
    ASSERT_EQ(status.values.size(), expected.size());
    for (std::size_t i = 0; i < expected.size(); ++i) {
        EXPECT_EQ(status.values[i].key, expected[i].first);
        EXPECT_EQ(status.values[i].value, expected[i].second) << expected[i].first;
    }

    report.level = HealthLevel::OK;
    EXPECT_EQ(MakeHealthStatus(report, false, 7).level, diagnostic_msgs::msg::DiagnosticStatus::OK);
    report.level = HealthLevel::ERROR;
    EXPECT_EQ(MakeHealthStatus(report, false, 7).level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
}

TEST(OptiTrackPoseRelayNode, UnsetRigidBodyIdIsAConfigurationError) {
    RclcppContext context;
    const std::string error = configurationError(rclcpp::NodeOptions());
    EXPECT_NE(error.find("/opti_track/pose_relay/rigid_body_id is not configured"), std::string::npos) << error;
}

TEST(OptiTrackPoseRelayNode, InvalidValuesAreConfigurationErrors) {
    RclcppContext context;
    const std::string error = configurationError(relayOptions({
        rclcpp::Parameter("/opti_track/pose_relay/rigid_body_id", 3),
        rclcpp::Parameter("/opti_track/pose_relay/stale_timeout_s", 0.0),
        rclcpp::Parameter("/opti_track/pose_relay/lab_ros_domain_id", 300),
    }));
    EXPECT_NE(error.find("/opti_track/pose_relay/stale_timeout_s must be in (0, 1] s"), std::string::npos)
        << error;
    EXPECT_NE(error.find("/opti_track/pose_relay/lab_ros_domain_id must be in [0, 232]"), std::string::npos)
        << error;
}

TEST(OptiTrackPoseRelayNode, WrongParameterTypeIsAConfigurationError) {
    RclcppContext context;
    const std::string error = configurationError(relayOptions({
        rclcpp::Parameter("/opti_track/pose_relay/rigid_body_id", 3),
        rclcpp::Parameter("/opti_track/pose_relay/output_rate_hz", 50),
    }));
    EXPECT_NE(error.find("/opti_track/pose_relay/output_rate_hz must be a double"), std::string::npos)
        << error;
}

TEST(OptiTrackPoseRelayNode, InvalidConfigurationKeepsTheRelayIdleAndReportsIt) {
    RclcppContext context;
    // Stays up (a supervised restart would only repeat the error), reports
    // the error, and neither relays nor claims readiness.
    auto relay = std::make_shared<OptiTrackPoseRelayNode>(
        "pose_relay_idle_test", "/opti_track_test", rclcpp::NodeOptions());
    ASSERT_FALSE(relay->configuration_error().empty());
    auto peer = std::make_shared<rclcpp::Node>("pose_relay_idle_peer");

    std::mutex mutex;
    std::vector<diagnostic_msgs::msg::DiagnosticStatus> health;
    auto health_sub = peer->create_subscription<diagnostic_msgs::msg::DiagnosticStatus>(
        kHealthTopic, rclcpp::QoS(rclcpp::KeepLast(10)),
        [&](const diagnostic_msgs::msg::DiagnosticStatus::ConstSharedPtr message) {
            std::lock_guard<std::mutex> lock(mutex);
            health.push_back(*message);
        });
    int heartbeats = 0;
    auto fresh_sub = peer->create_subscription<std_msgs::msg::Header>(
        kFreshTopic, rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
        [&](const std_msgs::msg::Header::ConstSharedPtr) {
            std::lock_guard<std::mutex> lock(mutex);
            ++heartbeats;
        });

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(relay);
    executor.add_node(peer);
    std::thread spinner([&executor]() { executor.spin(); });
    const auto deadline = std::chrono::steady_clock::now() + 10s;
    std::size_t received = 0;
    while (received < 3 && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(20ms);
        std::lock_guard<std::mutex> lock(mutex);
        received = health.size();
    }
    executor.cancel();
    spinner.join();

    std::lock_guard<std::mutex> lock(mutex);
    ASSERT_GE(health.size(), 3u);
    for (const auto & status : health) {
        EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
        EXPECT_EQ(status.name, "opti_track_pose_relay");
        EXPECT_NE(status.message.find("not relaying, invalid configuration: "
            "/opti_track/pose_relay/rigid_body_id is not configured"), std::string::npos) << status.message;
        EXPECT_EQ(valueOf(status, "stale"), "true");
        EXPECT_EQ(valueOf(status, "origin_sent"), "false");
        EXPECT_EQ(valueOf(status, "rigid_body_id"), "-1");
    }
    EXPECT_EQ(heartbeats, 0);
    // Its health publisher is discovered, so would be the others.
    EXPECT_EQ(peer->count_publishers(kVisualOdometryTopic), 0u);
    EXPECT_EQ(peer->count_publishers(kFreshTopic), 0u);
    EXPECT_EQ(peer->count_publishers("/fmu/in/vehicle_command"), 0u);
    EXPECT_EQ(peer->count_subscribers("/fmu/out/vehicle_status_v1"), 0u);
}

TEST(OptiTrackPoseRelayNode, RelaysLabPosesToPx4AndSetsTheOrigin) {
    RclcppContext context;
    const auto domain = static_cast<int64_t>(
        rclcpp::contexts::get_global_default_context()->get_domain_id());

    // The lab gateway shares the test's domain here; on the aircraft it is
    // the lab's domain, joined by the relay's second context.
    auto relay = std::make_shared<OptiTrackPoseRelayNode>(
        "pose_relay_end_to_end_test", "/opti_track_test", relayOptions({
            rclcpp::Parameter("/opti_track/pose_relay/rigid_body_id", 7),
            rclcpp::Parameter("/opti_track/pose_relay/lab_ros_domain_id", domain),
        }));
    auto peer = std::make_shared<rclcpp::Node>("pose_relay_end_to_end_peer");

    std::mutex mutex;
    std::vector<px4_msgs::msg::VehicleOdometry> odometry;
    std::vector<px4_msgs::msg::VehicleCommand> commands;
    std::vector<diagnostic_msgs::msg::DiagnosticStatus> health;
    std::vector<std_msgs::msg::Header> heartbeats;
    std::vector<std::chrono::steady_clock::time_point> heartbeat_receipts;
    std::optional<std::chrono::steady_clock::time_point> stale_receipt;
    auto odometry_sub = peer->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/in/vehicle_visual_odometry", rclcpp::QoS(rclcpp::KeepLast(100)).best_effort(),
        [&](const px4_msgs::msg::VehicleOdometry::ConstSharedPtr message) {
            std::lock_guard<std::mutex> lock(mutex);
            odometry.push_back(*message);
        });
    auto command_sub = peer->create_subscription<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
        [&](const px4_msgs::msg::VehicleCommand::ConstSharedPtr message) {
            std::lock_guard<std::mutex> lock(mutex);
            commands.push_back(*message);
        });
    auto health_sub = peer->create_subscription<diagnostic_msgs::msg::DiagnosticStatus>(
        "/opti_track/pose_relay/health", rclcpp::QoS(rclcpp::KeepLast(10)),
        [&](const diagnostic_msgs::msg::DiagnosticStatus::ConstSharedPtr message) {
            std::lock_guard<std::mutex> lock(mutex);
            health.push_back(*message);
            if (!stale_receipt && !odometry.empty() && valueOf(*message, "stale") == "true") {
                stale_receipt = std::chrono::steady_clock::now();
            }
        });
    auto fresh_sub = peer->create_subscription<std_msgs::msg::Header>(
        "/opti_track/pose_relay/fresh", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
        [&](const std_msgs::msg::Header::ConstSharedPtr message) {
            std::lock_guard<std::mutex> lock(mutex);
            heartbeats.push_back(*message);
            heartbeat_receipts.push_back(std::chrono::steady_clock::now());
        });

    auto pose_pub = peer->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/body_splitter/body_7/pose", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    rclcpp::QoS px4_out_qos(rclcpp::KeepLast(1));
    px4_out_qos.best_effort().transient_local();  // as PX4's uXRCE-DDS writers
    auto status_pub = peer->create_publisher<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status_v1", px4_out_qos);
    auto local_position_pub = peer->create_publisher<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", px4_out_qos);
    auto flags_pub = peer->create_publisher<px4_msgs::msg::EstimatorStatusFlags>(
        "/fmu/out/estimator_status_flags", px4_out_qos);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(relay);
    executor.add_node(peer);
    std::thread spinner([&executor]() { executor.spin(); });

    const auto discovered = [&]() {
        return pose_pub->get_subscription_count() > 0 && fresh_sub->get_publisher_count() > 0 &&
            odometry_sub->get_publisher_count() > 0 && command_sub->get_publisher_count() > 0 &&
            status_pub->get_subscription_count() > 0 && flags_pub->get_subscription_count() > 0 &&
            local_position_pub->get_subscription_count() > 0;
    };
    const auto discovery_deadline = std::chrono::steady_clock::now() + 10s;
    while (!discovered() && std::chrono::steady_clock::now() < discovery_deadline) {
        std::this_thread::sleep_for(10ms);
    }
    ASSERT_TRUE(discovered());

    px4_msgs::msg::VehicleStatus status;
    status.timestamp = 123'456;
    status.arming_state = px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED;
    status.system_id = 1;
    status.component_id = 1;
    px4_msgs::msg::VehicleLocalPosition local_position;
    local_position.xy_global = false;
    px4_msgs::msg::EstimatorStatusFlags flags;
    flags.cs_ev_pos = true;

    // A 90 degree yaw about the lab's Z (up) axis.
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = 1.0;
    pose.pose.position.y = 2.0;
    pose.pose.position.z = 3.0;
    pose.pose.orientation.w = std::cos(M_PI / 4.0);
    pose.pose.orientation.z = std::sin(M_PI / 4.0);

    const auto publish_start = std::chrono::steady_clock::now();
    int published = 0;
    while (std::chrono::steady_clock::now() - publish_start < 1500ms) {
        pose.header.stamp = peer->now();
        pose_pub->publish(pose);
        if (published % 12 == 0) {
            status_pub->publish(status);
            local_position_pub->publish(local_position);
            flags_pub->publish(flags);
        }
        ++published;
        std::this_thread::sleep_for(std::chrono::microseconds(8333));
    }
    const double publish_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - publish_start).count();

    // The stream stops: the relay reports it stale and stops its heartbeat.
    bool reported_stale = false;
    const auto stale_deadline = std::chrono::steady_clock::now() + 3s;
    while (!reported_stale && std::chrono::steady_clock::now() < stale_deadline) {
        std::this_thread::sleep_for(50ms);
        std::lock_guard<std::mutex> lock(mutex);
        reported_stale = !health.empty() && valueOf(health.back(), "stale") == "true" &&
            health.back().level == diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    }
    std::this_thread::sleep_for(1100ms);  // two more health periods

    executor.cancel();
    spinner.join();

    std::lock_guard<std::mutex> lock(mutex);
    EXPECT_TRUE(reported_stale);

    // Forwarded, converted to NED/FRD, at most at the output rate.
    ASSERT_GE(odometry.size(), 10u) << "published " << published << " lab poses";
    EXPECT_LE(static_cast<double>(odometry.size()), publish_s * 50.0 + 3.0);
    const double c = std::cos(M_PI / 4.0);
    const auto now_us = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
    for (const auto & message : odometry) {
        EXPECT_EQ(message.pose_frame, px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED);
        EXPECT_FLOAT_EQ(message.position[0], 1.0F);
        EXPECT_FLOAT_EQ(message.position[1], -2.0F);
        EXPECT_FLOAT_EQ(message.position[2], -3.0F);
        EXPECT_NEAR(message.q[0], c, 1.0e-6);
        EXPECT_NEAR(message.q[1], 0.0, 1.0e-6);
        EXPECT_NEAR(message.q[2], 0.0, 1.0e-6);
        EXPECT_NEAR(message.q[3], -c, 1.0e-6);
        EXPECT_EQ(message.velocity_frame, px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN);
        EXPECT_TRUE(std::isnan(message.velocity[0]));
        EXPECT_FLOAT_EQ(message.position_variance[0], 1.0e-4F);
        EXPECT_FLOAT_EQ(message.orientation_variance[2], 4.0e-4F);
        EXPECT_EQ(message.timestamp, message.timestamp_sample);
        EXPECT_NEAR(static_cast<double>(message.timestamp), now_us, 30.0e6);  // system clock
    }

    // Healthy while the stream ran.
    bool reported_fresh = false;
    for (const auto & status_message : health) {
        EXPECT_EQ(status_message.name, "opti_track_pose_relay");
        EXPECT_EQ(valueOf(status_message, "rigid_body_id"), "7");
        reported_fresh = reported_fresh || valueOf(status_message, "stale") == "false";
    }
    EXPECT_TRUE(reported_fresh);
    ASSERT_FALSE(health.empty());
    EXPECT_EQ(valueOf(health.back(), "origin_sent"), "true");

    // Readiness heartbeat at 2 Hz while forwarding, none once stale.
    EXPECT_GE(heartbeats.size(), 2u);
    EXPECT_LE(heartbeats.size(), static_cast<std::size_t>(publish_s * 2.0 + 2.0));
    for (const auto & heartbeat : heartbeats) {
        EXPECT_EQ(heartbeat.frame_id, "opti_track_pose_relay");
        EXPECT_GT(rclcpp::Time(heartbeat.stamp).nanoseconds(), 0);
    }
    ASSERT_TRUE(stale_receipt.has_value());
    for (const auto & receipt : heartbeat_receipts) {
        EXPECT_LT(receipt, *stale_receipt);
    }

    // Disarmed, no origin and vision position fusion intended: origin sent.
    ASSERT_FALSE(commands.empty());
    const auto & command = commands.front();
    EXPECT_EQ(command.command, px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN);
    EXPECT_DOUBLE_EQ(command.param5, 55.3672);
    EXPECT_DOUBLE_EQ(command.param6, 10.4310);
    EXPECT_FLOAT_EQ(command.param7, 20.0F);
    EXPECT_EQ(command.timestamp, 123'456u);
    EXPECT_EQ(command.target_system, 1);
    EXPECT_EQ(command.target_component, 1);
    EXPECT_TRUE(command.from_external);
    // At most once per 5 s.
    EXPECT_EQ(commands.size(), 1u);
}
