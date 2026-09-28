#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <memory>
#include <thread>

#define private public
#include <iii_drone_core/control/maneuver/follow_waypoint_path_maneuver_server.hpp>
#include <iii_drone_core/control/maneuver/hover_maneuver_server.hpp>
#undef private

using iii_drone::control::ControlledCancellationConfig;
using iii_drone::control::State;
using iii_drone::control::maneuver::WaypointPathTerminalStopProof;
using iii_drone::types::point_t;
using iii_drone::types::vector_t;

namespace {

State measured(double speed, double yaw_rate) {
    return State(
        point_t(1.0, 2.0, 3.0),
        vector_t(speed, 0.0, 0.0),
        0.0,
        vector_t(0.0, 0.0, yaw_rate)
    );
}

}  // namespace

namespace {

constexpr char kPathRequest[] =
    "mri1-00000000000000010000000000000001-0000000000000001";

struct PathServerFixture {
    bool initialized_here = !rclcpp::ok();
    std::unique_ptr<rclcpp_lifecycle::LifecycleNode> node;
    iii_drone::configuration::Configuration::SharedPtr config;
    iii_drone::control::CombinedDroneAwarenessHandler::SharedPtr awareness;
    std::shared_ptr<iii_drone::control::maneuver::HoverManeuverServer> hover;
    std::shared_ptr<iii_drone::control::maneuver::FollowWaypointPathManeuverServer> path;
    uint64_t sample_us = 1'000'000;

    PathServerFixture() {
        if (initialized_here) rclcpp::init(0, nullptr);
        node = std::make_unique<rclcpp_lifecycle::LifecycleNode>("fwp_terminal_server_test");
        config = std::make_shared<iii_drone::configuration::Configuration>(
            "fwp-terminal-server-test",
            std::vector<iii_drone::configuration::configuration_entry_t>{
                {"/control/maneuver_controller/reached_position_euclidean_distance_threshold",
                    rclcpp::ParameterType::PARAMETER_DOUBLE},
                {"/control/maneuver_controller/reached_yaw_error_threshold",
                    rclcpp::ParameterType::PARAMETER_DOUBLE},
                {"/control/maneuver_controller/controlled_cancel_max_deceleration_m_s2",
                    rclcpp::ParameterType::PARAMETER_DOUBLE},
                {"/control/maneuver_controller/controlled_cancel_max_jerk_m_s3",
                    rclcpp::ParameterType::PARAMETER_DOUBLE},
                {"/control/maneuver_controller/controlled_cancel_max_yaw_deceleration_rad_s2",
                    rclcpp::ParameterType::PARAMETER_DOUBLE},
                {"/control/maneuver_controller/controlled_cancel_max_yaw_jerk_rad_s3",
                    rclcpp::ParameterType::PARAMETER_DOUBLE},
                {"/control/maneuver_controller/controlled_cancel_velocity_threshold_m_s",
                    rclcpp::ParameterType::PARAMETER_DOUBLE},
                {"/control/maneuver_controller/controlled_cancel_yaw_rate_threshold_rad_s",
                    rclcpp::ParameterType::PARAMETER_DOUBLE},
                {"/control/maneuver_controller/controlled_cancel_settle_time_s",
                    rclcpp::ParameterType::PARAMETER_DOUBLE},
            },
            [](const std::string & name) {
                if (name.find("reached_position") != std::string::npos)
                    return rclcpp::Parameter(name, 0.1);
                if (name.find("settle_time") != std::string::npos)
                    return rclcpp::Parameter(name, 0.2);
                if (name.find("max_deceleration") != std::string::npos ||
                    name.find("max_yaw_deceleration") != std::string::npos)
                    return rclcpp::Parameter(name, 0.5);
                if (name.find("max_jerk") != std::string::npos)
                    return rclcpp::Parameter(name, 1.0);
                return rclcpp::Parameter(name, 0.08);
            });
        awareness = std::make_shared<iii_drone::control::CombinedDroneAwarenessHandler>(
            config, std::make_shared<tf2_ros::Buffer>(node->get_clock()), node.get());
        awareness->vehicle_status_adapter_history_ = std::make_shared<
            iii_drone::utils::History<iii_drone::adapters::px4::VehicleStatusAdapter>>(1);
        awareness->vehicle_odometry_adapter_history_ = std::make_shared<
            iii_drone::utils::History<iii_drone::adapters::px4::VehicleOdometryAdapter>>(1);
        awareness->vehicle_status_adapter_history_->Store(
            iii_drone::adapters::px4::VehicleStatusAdapter(px4_msgs::msg::VehicleStatus{}));
        px4_msgs::msg::VehicleOdometry odometry;
        odometry.pose_frame = iii_drone::adapters::px4::POSE_FRAME_LOCAL_NED;
        odometry.velocity_frame = iii_drone::adapters::px4::VELOCITY_FRAME_LOCAL_NED;
        odometry.q[0] = 1.0F;
        awareness->vehicle_odometry_adapter_history_->Store(
            iii_drone::adapters::px4::VehicleOdometryAdapter(odometry));
        hover = std::make_shared<iii_drone::control::maneuver::HoverManeuverServer>(
            node.get(), awareness, "hover", 1, 1, false);
        path = std::make_shared<iii_drone::control::maneuver::FollowWaypointPathManeuverServer>(
            node.get(), awareness, "follow_waypoint_path", 1, 1, config);
        path->registered_maneuvers_[iii_drone::control::maneuver::MANEUVER_TYPE_HOVER] = hover;
        iii_drone::control::maneuver::Maneuver maneuver(
            iii_drone::control::maneuver::MANEUVER_TYPE_FOLLOW_WAYPOINT_PATH, {});
        maneuver.request_identity_ = kPathRequest;
        auto params = std::make_shared<iii_drone::control::maneuver::follow_waypoint_path_maneuver_params_t>();
        maneuver.maneuver_params_ = params;
        path->current_maneuver_.Store(maneuver);
    }

    ~PathServerFixture() {
        path.reset();
        hover.reset();
        awareness.reset();
        node.reset();
        if (initialized_here) rclcpp::shutdown();
    }

    void sample(const point_t & position = point_t::Zero()) {
        iii_drone::control::MeasuredOdometrySnapshot measured;
        measured.state = State(position, vector_t(0.24F, 0.0F, 0.0F), 0.0,
            vector_t::Zero(), node->now());
        measured.receipt_stamp = node->now();
        measured.source_sample_timestamp_us = sample_us;
        sample_us += 50'000;
        awareness->measured_odometry_.Store(
            std::optional<iii_drone::control::MeasuredOdometrySnapshot>(measured));
    }
};

}  // namespace

TEST(FollowWaypointPathStopProofTest, MovingAtTargetCannotCompleteAndDwellStartsAfterStop) {
    WaypointPathTerminalStopProof proof;
    ControlledCancellationConfig config;
    config.velocity_threshold_m_s = 0.08;
    config.yaw_rate_threshold_rad_s = 0.08;
    config.settle_time_s = 0.2;
    const auto start = WaypointPathTerminalStopProof::Clock::time_point{};

    EXPECT_FALSE(proof.observe(false, measured(0.0, 0.0), config, start));
    EXPECT_FALSE(proof.observe(true, measured(0.09, 0.0), config, start + std::chrono::seconds(1)));
    EXPECT_FALSE(proof.observe(true, measured(0.0, 0.09), config, start + std::chrono::seconds(2)));
    EXPECT_FALSE(proof.observe(true, measured(0.08, 0.08), config, start + std::chrono::seconds(3)));
    EXPECT_FALSE(proof.observe(true, measured(0.08, 0.08), config, start + std::chrono::milliseconds(3199)));
    EXPECT_TRUE(proof.observe(true, measured(0.08, 0.08), config, start + std::chrono::milliseconds(3200)));
}

TEST(FollowWaypointPathStopProofTest, AnyViolationOrNewExecutionRestartsDwell) {
    WaypointPathTerminalStopProof proof;
    ControlledCancellationConfig config;
    config.settle_time_s = 0.2;
    const auto start = WaypointPathTerminalStopProof::Clock::time_point{};
    const auto at = [&](int milliseconds, bool terminal, const State & state) {
        return proof.observe(
            terminal, state, config, start + std::chrono::milliseconds(milliseconds)
        );
    };

    EXPECT_FALSE(at(0, true, measured(0.0, 0.0)));
    EXPECT_FALSE(at(100, false, measured(0.0, 0.0)));  // Pose or planned duration violated.
    EXPECT_FALSE(at(200, true, measured(0.0, 0.0)));
    EXPECT_FALSE(at(300, true, measured(std::numeric_limits<double>::quiet_NaN(), 0.0)));
    EXPECT_FALSE(at(400, true, measured(0.0, 0.0)));
    EXPECT_FALSE(at(500, true, measured(0.0, std::numeric_limits<double>::infinity())));
    EXPECT_FALSE(at(600, true, measured(0.0, 0.0)));
    EXPECT_FALSE(at(700, true, measured(0.09, 0.0)));
    EXPECT_FALSE(at(800, true, measured(0.0, 0.0)));
    EXPECT_TRUE(at(1000, true, measured(0.0, 0.0)));

    proof.reset();  // New path or rebase.
    EXPECT_FALSE(at(1100, true, measured(0.0, 0.0)));
    EXPECT_FALSE(at(1299, true, measured(0.0, 0.0)));
    EXPECT_TRUE(at(1300, true, measured(0.0, 0.0)));
}

TEST(FollowWaypointPathTerminalServer, NonrepeatingRestRequiresAckAndEstimatedMotionProof) {
    PathServerFixture fixture;
    const iii_drone::control::Reference nominal(point_t::Zero(), 0.0);
    iii_drone::control::WaypointPathWaypoint destination;
    destination.position = nominal.position();
    destination.transition = iii_drone::control::WaypointTransition::Stop;
    fixture.path->plan_ = fixture.path->planner_.plan(
        iii_drone::control::Reference(point_t(-1.0F, 0.0F, 0.0F), 0.0),
        {destination}, false, 0, iii_drone::control::WaypointPathConstraints{});
    ASSERT_FALSE(fixture.path->plan_.prefix.references.empty());
    const auto planned_end = fixture.path->plan_.prefix.references.back();
    EXPECT_LE(planned_end.velocity().norm(), 1.0e-5);
    EXPECT_LE(planned_end.acceleration().norm(), 1.0e-5);
    EXPECT_LE(std::abs(planned_end.yaw_rate()), 1.0e-5);
    EXPECT_LE(std::abs(planned_end.yaw_acceleration()), 1.0e-5);
    fixture.path->execution_start_time_ = fixture.node->now() -
        rclcpp::Duration::from_seconds(fixture.path->plan_.prefixDurationS() + 0.1);
    fixture.path->active_repeat_ = false;
    fixture.sample();
    const auto emitted = fixture.path->computeReference(State());
    ASSERT_TRUE(fixture.path->terminal_hold_);
    EXPECT_TRUE(emitted.position().allFinite());
    EXPECT_EQ(fixture.hover->terminalHoldBinding().request_identity, kPathRequest);

    bool applied = false;
    fixture.path->RegisterAppliedRestReferenceCallback(
        [&applied](const std::string & owner, const iii_drone::control::Reference & rest) {
            return applied && owner == kPathRequest && rest.velocity().norm() < 1.0e-5;
        });
    auto maneuver = fixture.path->current_maneuver_.Load();
    EXPECT_FALSE(fixture.path->hasSucceeded(maneuver));
    applied = true;
    bool complete = false;
    for (int tick = 0; tick < 27 && !complete; ++tick) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        fixture.sample();
        fixture.path->computeReference(State());
        complete = fixture.path->hasSucceeded(maneuver);
    }
    EXPECT_TRUE(complete);  // Raw EKF speed remained 0.24 m/s throughout.
}

TEST(FollowWaypointPathCancellationServer, MovingPathStopRetainsAckedFiniteRest) {
    PathServerFixture fixture;
    const iii_drone::control::Reference moving(
        point_t::Zero(), 0.0, vector_t(0.2F, 0.0F, 0.0F), 0.0,
        vector_t::Zero(), 0.0);
    ControlledCancellationConfig config;
    fixture.path->controlled_stop_trajectory_.emplace(moving, config.limits);
    fixture.path->controlled_stop_start_time_ = std::chrono::steady_clock::now() -
        std::chrono::seconds(10);
    const auto rest = fixture.path->controlled_stop_trajectory_->terminalReference(
        fixture.node->now());
    EXPECT_TRUE(rest.position().allFinite());
    EXPECT_NEAR(rest.velocity().norm(), 0.0, 1.0e-6);
    bool applied = false;
    fixture.path->RegisterAppliedRestReferenceCallback(
        [&applied, &rest](const std::string & owner,
            const iii_drone::control::Reference & candidate) {
            return applied && owner == kPathRequest &&
                (candidate.position() - rest.position()).norm() < 1.0e-6;
        });
    fixture.sample();
    EXPECT_FALSE(fixture.path->controlledCancellationComplete(config));
    applied = true;
    bool complete = false;
    for (int tick = 0; tick < 27 && !complete; ++tick) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        fixture.sample();
        complete = fixture.path->controlledCancellationComplete(config);
    }
    EXPECT_TRUE(complete);  // A .24 m/s biased velocity does not block measured-position proof.
    ASSERT_TRUE(fixture.path->terminal_hold_);
    EXPECT_TRUE(fixture.path->terminal_hold_->isQuiescent());
    EXPECT_EQ(fixture.hover->terminalHoldBinding().request_identity, kPathRequest);
    EXPECT_FALSE(fixture.path->controlledCancellationFailure());
}
