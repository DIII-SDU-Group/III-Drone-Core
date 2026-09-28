#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdarg>
#include <cstdio>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rcutils/logging.h>

#include <gtest/gtest.h>

#include <iii_drone_configuration/configuration.hpp>

#define private public
#include <iii_drone_core/control/maneuver/maneuver_reference_client.hpp>
#include <iii_drone_core/control/maneuver/maneuver_scheduler.hpp>
#include <iii_drone_core/control/maneuver/hover_maneuver_server.hpp>
#include <iii_drone_core/control/maneuver/cable_aware_fly_to_position_maneuver_server.hpp>
#include <iii_drone_core/control/maneuver/follow_waypoint_path_maneuver_server.hpp>
#include <iii_drone_core/control/maneuver/fly_to_position_maneuver_server.hpp>
#include <iii_drone_core/control/maneuver/fly_to_object_maneuver_server.hpp>
#include <iii_drone_core/control/maneuver/cable_landing_maneuver_server.hpp>
#undef private
#include <iii_drone_core/control/trajectory_interpolator.hpp>
#include <iii_drone_core/control/maneuver/object_tracking_session.hpp>

namespace {

using iii_drone::adapters::ReferenceAdapter;
using iii_drone::adapters::px4::VehicleOdometryAdapter;
using iii_drone::configuration::Configuration;
using iii_drone::configuration::configuration_entry_t;
using iii_drone::control::Reference;
using iii_drone::control::maneuver::ManeuverReferenceClient;
using iii_drone::types::point_t;
using iii_drone::types::vector_t;
using Stream = iii_drone_interfaces::msg::ManeuverReferenceStream;
using Ack = iii_drone_interfaces::msg::ManeuverReferenceAck;

constexpr char kRequestA[] = "mri1-00000000000000010000000000000001-0000000000000001";
constexpr char kRequestB[] = "mri1-00000000000000020000000000000002-0000000000000002";
constexpr char kRequestC[] = "mri1-00000000000000030000000000000003-0000000000000003";
constexpr char kRequestD[] = "mri1-00000000000000040000000000000004-0000000000000004";

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

Configuration::SharedPtr makeConfiguration(
    int wait_for_maneuver_start_timeout_ms = 10000,
    double reference_continuity_velocity_tolerance_m_s = 1.0,
    int reference_stream_timeout_ms = 10000,
    double object_arrival_threshold_m = 1.0,
    double object_target_filter_time_constant_s = 1.0,
    int maneuver_execution_period_ms = 10000
) {
    const std::vector<configuration_entry_t> entries{
        {"/control/maneuver_controller/maneuver_execution_period_ms", rclcpp::ParameterType::PARAMETER_INTEGER},
        {"/control/maneuver_controller/maneuver_queue_size", rclcpp::ParameterType::PARAMETER_INTEGER},
        {"/control/maneuver_controller/maneuver_publish_period_ms", rclcpp::ParameterType::PARAMETER_INTEGER},
        {"/control/maneuver_controller/ground_estimate_window_size", rclcpp::ParameterType::PARAMETER_INTEGER},
        {"/control/maneuver_controller/ground_estimate_update_period_ms", rclcpp::ParameterType::PARAMETER_INTEGER},
        {"/control/maneuver_controller/combined_drone_awareness_pub_period_ms", rclcpp::ParameterType::PARAMETER_INTEGER},
        {"/control/maneuver_controller/maneuver_start_timeout_s", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/maneuver_completion_token_acquisition_timeout_s", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/no_maneuver_idle_cnt_s", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/reference_stream_timeout_ms", rclcpp::ParameterType::PARAMETER_INTEGER},
        {"/mission/reference_loss_timeout_ms", rclcpp::ParameterType::PARAMETER_INTEGER},
        {"/control/maneuver_controller/controlled_cancel_max_jerk_m_s3", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/controlled_cancel_max_yaw_jerk_rad_s3", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/mission/reference_continuity_position_tolerance_m", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/mission/reference_continuity_velocity_tolerance_m_s", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/mission/reference_continuity_acceleration_tolerance_m_s2", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/mission/reference_continuity_yaw_tolerance_rad", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/mission/reference_continuity_yaw_rate_tolerance_rad_s", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/mission/reference_continuity_yaw_acceleration_tolerance_rad_s2", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/controlled_cancel_max_deceleration_m_s2", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/controlled_cancel_max_yaw_deceleration_rad_s2", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/controlled_cancel_velocity_threshold_m_s", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/controlled_cancel_yaw_rate_threshold_rad_s", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/controlled_cancel_settle_time_s", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/mission/reference_rebase_timeout_ms", rclcpp::ParameterType::PARAMETER_INTEGER},
        {"/mission/wait_for_maneuver_start_timeout_ms", rclcpp::ParameterType::PARAMETER_INTEGER},
        {"/mission/max_failed_attempts_during_maneuver", rclcpp::ParameterType::PARAMETER_INTEGER},
        {"/mission/use_nans_when_hovering", rclcpp::ParameterType::PARAMETER_BOOL},
        {"/mission/get_reference_timeout_ms", rclcpp::ParameterType::PARAMETER_INTEGER},
        {"/control/maneuver_controller/fly_to_object_use_mpc", rclcpp::ParameterType::PARAMETER_BOOL},
        {"/control/maneuver_controller/cable_landing_controller_type", rclcpp::ParameterType::PARAMETER_STRING},
        {"/control/maneuver_controller/minimum_target_altitude", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/tf/world_frame_id", rclcpp::ParameterType::PARAMETER_STRING},
        {"/tf/drone_frame_id", rclcpp::ParameterType::PARAMETER_STRING},
        {"/tf/cable_gripper_frame_id", rclcpp::ParameterType::PARAMETER_STRING},
        {"/control/maneuver_controller/fly_to_object_target_loss_grace_s", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/reached_position_euclidean_distance_threshold", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/fly_to_object_target_low_pass_time_constant_s", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/cable_landing_target_upwards_velocity", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/cable_landing_line_pid_max_dt_s", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/cable_landing_line_pid_ascent_velocity", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/cable_landing_gripper_v_gate_center_y", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/cable_landing_gripper_v_gate_apex_z", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/cable_landing_gripper_v_gate_reference_z", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/cable_landing_gripper_v_gate_half_width_at_reference_z", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/cable_landing_line_pid_along_kp", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/cable_landing_line_pid_along_ki", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/cable_landing_line_pid_along_kd", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/cable_landing_line_pid_along_integral_limit", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/cable_landing_line_pid_max_along_velocity", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/cable_landing_line_pid_cross_kp", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/cable_landing_line_pid_cross_ki", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/cable_landing_line_pid_cross_kd", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/cable_landing_line_pid_cross_integral_limit", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/cable_landing_line_pid_max_cross_velocity", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/cable_landing_line_pid_yaw_kp", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/cable_landing_line_pid_yaw_ki", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/cable_landing_line_pid_yaw_kd", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/cable_landing_line_pid_yaw_integral_limit", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/cable_landing_line_pid_max_yaw_rate", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/trajectory_interpolator/interpolation_avg_velocity_m_s", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/trajectory_interpolator/interpolation_avg_yaw_rate_rad_s", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/trajectory_interpolator/interpolation_max_velocity_m_s", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/trajectory_interpolator/interpolation_max_acceleration_m_s2", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/trajectory_interpolator/interpolation_max_jerk_m_s3", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/trajectory_interpolator/interpolation_max_yaw_rate_rad_s", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/trajectory_interpolator/interpolation_max_yaw_acceleration_rad_s2", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/trajectory_interpolator/reference_trajectory_length_N", rclcpp::ParameterType::PARAMETER_INTEGER},
        {"/control/dt", rclcpp::ParameterType::PARAMETER_DOUBLE},
    };
    return std::make_shared<Configuration>(
        "maneuver-reference-client-transaction-test",
        entries,
        [wait_for_maneuver_start_timeout_ms, reference_continuity_velocity_tolerance_m_s,
         reference_stream_timeout_ms, object_arrival_threshold_m,
         object_target_filter_time_constant_s, maneuver_execution_period_ms](
            const std::string & name
        ) -> rclcpp::Parameter {
            if (name == "/mission/use_nans_when_hovering") {
                return rclcpp::Parameter(name, false);
            }
            if (name == "/control/maneuver_controller/fly_to_object_use_mpc") {
                return rclcpp::Parameter(name, false);
            }
            if (name == "/control/maneuver_controller/cable_landing_controller_type") {
                return rclcpp::Parameter(name, "line_pid");
            }
            if (name == "/tf/world_frame_id") return rclcpp::Parameter(name, "world");
            if (name == "/tf/drone_frame_id") return rclcpp::Parameter(name, "drone");
            if (name == "/tf/cable_gripper_frame_id") return rclcpp::Parameter(name, "gripper");
            if (name.find("/cable_landing_line_pid_") != std::string::npos ||
                name.find("/cable_landing_gripper_v_gate_") != std::string::npos ||
                name == "/control/maneuver_controller/cable_landing_target_upwards_velocity") {
                if (name == "/control/maneuver_controller/cable_landing_line_pid_max_dt_s") {
                    return rclcpp::Parameter(name, 0.2);
                }
                if (name == "/control/maneuver_controller/cable_landing_line_pid_ascent_velocity") {
                    return rclcpp::Parameter(name, 0.2);
                }
                if (name == "/control/maneuver_controller/cable_landing_gripper_v_gate_apex_z") {
                    return rclcpp::Parameter(name, -0.16);
                }
                if (name == "/control/maneuver_controller/cable_landing_gripper_v_gate_reference_z") {
                    return rclcpp::Parameter(name, 0.03);
                }
                if (name == "/control/maneuver_controller/cable_landing_gripper_v_gate_half_width_at_reference_z") {
                    return rclcpp::Parameter(name, 0.18);
                }
                if (name == "/control/maneuver_controller/cable_landing_line_pid_max_along_velocity") {
                    return rclcpp::Parameter(name, 0.01);
                }
                if (name == "/control/maneuver_controller/cable_landing_line_pid_max_cross_velocity") {
                    return rclcpp::Parameter(name, 0.12);
                }
                if (name == "/control/maneuver_controller/cable_landing_line_pid_max_yaw_rate") {
                    return rclcpp::Parameter(name, 0.35);
                }
                if (name == "/control/maneuver_controller/cable_landing_line_pid_cross_kp") {
                    return rclcpp::Parameter(name, 0.55);
                }
                if (name == "/control/maneuver_controller/cable_landing_line_pid_yaw_kp") {
                    return rclcpp::Parameter(name, 1.20);
                }
                if (name == "/control/maneuver_controller/cable_landing_line_pid_yaw_kd") {
                    return rclcpp::Parameter(name, 0.05);
                }
                return rclcpp::Parameter(name, 0.0);
            }
            if (name == "/control/trajectory_interpolator/reference_trajectory_length_N") {
                return rclcpp::Parameter(name, 10);
            }
            if (name == "/control/dt") return rclcpp::Parameter(name, 0.2);
            if (name.find("/control/trajectory_interpolator/") == 0) {
                return rclcpp::Parameter(name, 0.5);
            }
            if (name == "/control/maneuver_controller/minimum_target_altitude") {
                return rclcpp::Parameter(name, 0.0);
            }
            if (name == "/control/maneuver_controller/reached_position_euclidean_distance_threshold") {
                return rclcpp::Parameter(name, object_arrival_threshold_m);
            }
            if (name == "/control/maneuver_controller/fly_to_object_target_low_pass_time_constant_s") {
                return rclcpp::Parameter(name, object_target_filter_time_constant_s);
            }
            if (name == "/control/maneuver_controller/maneuver_execution_period_ms") {
                return rclcpp::Parameter(name, maneuver_execution_period_ms);
            }
            if (name == "/control/maneuver_controller/fly_to_object_target_loss_grace_s") {
                return rclcpp::Parameter(name, 1.0);
            }
            if (name == "/mission/wait_for_maneuver_start_timeout_ms") {
                return rclcpp::Parameter(name, wait_for_maneuver_start_timeout_ms);
            }
            if (name == "/control/maneuver_controller/reference_stream_timeout_ms") {
                return rclcpp::Parameter(name, reference_stream_timeout_ms);
            }
            if (
                name == "/control/maneuver_controller/controlled_cancel_max_jerk_m_s3" ||
                name == "/control/maneuver_controller/controlled_cancel_max_yaw_jerk_rad_s3" ||
                name == "/mission/reference_continuity_position_tolerance_m" ||
                name == "/mission/reference_continuity_velocity_tolerance_m_s" ||
                name == "/mission/reference_continuity_acceleration_tolerance_m_s2" ||
                name == "/mission/reference_continuity_yaw_tolerance_rad" ||
                name == "/mission/reference_continuity_yaw_rate_tolerance_rad_s" ||
                name == "/mission/reference_continuity_yaw_acceleration_tolerance_rad_s2" ||
                name == "/control/maneuver_controller/controlled_cancel_max_deceleration_m_s2" ||
                name == "/control/maneuver_controller/controlled_cancel_max_yaw_deceleration_rad_s2" ||
                name == "/control/maneuver_controller/controlled_cancel_velocity_threshold_m_s" ||
                name == "/control/maneuver_controller/controlled_cancel_yaw_rate_threshold_rad_s" ||
                name == "/control/maneuver_controller/controlled_cancel_settle_time_s"
                || name == "/control/maneuver_controller/maneuver_start_timeout_s"
                || name == "/control/maneuver_controller/maneuver_completion_token_acquisition_timeout_s"
                || name == "/control/maneuver_controller/no_maneuver_idle_cnt_s"
            ) {
                if (name == "/mission/reference_continuity_velocity_tolerance_m_s") {
                    return rclcpp::Parameter(name, reference_continuity_velocity_tolerance_m_s);
                }
                return rclcpp::Parameter(name, 1.0);
            }
            return rclcpp::Parameter(name, 10000);
        }
    );
}

Stream::SharedPtr stream(
    rclcpp_lifecycle::LifecycleNode & node,
    const std::string & stream_id,
    uint64_t sequence,
    const Reference & reference,
    const std::string & request_identity = kRequestA
) {
    auto message = std::make_shared<Stream>();
    message->stream_id = stream_id;
    message->request_identity = request_identity;
    message->sequence = sequence;
    message->state = Stream::STATE_ACTIVE;
    message->is_valid = true;
    message->reference = ReferenceAdapter(reference).ToMsg();
    message->produced_at = node.now();
    message->valid_until = node.now() + rclcpp::Duration::from_seconds(30.0);
    return message;
}

Reference finiteReference(double x, double velocity_x) {
    return Reference(
        point_t(x, 0.0, 0.0),
        0.0,
        vector_t(velocity_x, 0.0, 0.0),
        0.0,
        vector_t::Zero(),
        0.0
    );
}

Reference initializationHold() {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    return Reference(
        point_t::Zero(),
        0.0,
        vector_t::Constant(nan),
        nan,
        vector_t::Constant(nan),
        nan
    );
}

Reference upwardVelocityOnlyReference() {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    return Reference(
        point_t::Constant(nan),
        nan,
        vector_t(0.0, 0.0, 2.0),
        0.0,
        vector_t::Constant(nan),
        nan
    );
}

Reference mpcReferenceWithNaNYawDerivatives(
    double vertical_velocity, double vertical_acceleration = 0.3
) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    return Reference(
        point_t(1.0, 2.0, 3.0),
        0.25,
        vector_t(0.0, 0.0, vertical_velocity),
        nan,
        vector_t(0.1, 0.2, vertical_acceleration),
        nan
    );
}

VehicleOdometryAdapter stationaryVehicleState() {
    px4_msgs::msg::VehicleOdometry odometry;
    odometry.pose_frame = iii_drone::adapters::px4::POSE_FRAME_LOCAL_NED;
    odometry.velocity_frame = iii_drone::adapters::px4::VELOCITY_FRAME_LOCAL_NED;
    odometry.q[0] = 1.0F;
    return VehicleOdometryAdapter(odometry);
}

struct ClientFixture {
    explicit ClientFixture(
        const std::string & node_name,
        int wait_for_maneuver_start_timeout_ms = 10000,
        double reference_continuity_velocity_tolerance_m_s = 1.0
    )
    : node(node_name),
      history(std::make_shared<iii_drone::utils::History<VehicleOdometryAdapter>>(4)),
      client(
          &node,
          history,
          makeConfiguration(
              wait_for_maneuver_start_timeout_ms,
              reference_continuity_velocity_tolerance_m_s
          ),
          node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)
      ) {
        history->Store(stationaryVehicleState());
        ack_observer = std::make_shared<rclcpp::Node>(node_name + "_ack_observer");
        ack_subscription = ack_observer->create_subscription<Ack>(
            client.reference_ack_publisher_->get_topic_name(),
            rclcpp::QoS(10),
            [this](const Ack::SharedPtr message) { acknowledgements.push_back(*message); }
        );
        executor.add_node(node.get_node_base_interface());
        executor.add_node(ack_observer);
    }

    ~ClientFixture() {
        executor.remove_node(ack_observer);
        executor.remove_node(node.get_node_base_interface());
    }

    void spinCallbacks() {
        executor.spin_some();
    }

    rclcpp_lifecycle::LifecycleNode node;
    rclcpp::Node::SharedPtr ack_observer;
    iii_drone::utils::History<VehicleOdometryAdapter>::SharedPtr history;
    ManeuverReferenceClient client;
    std::vector<Ack> acknowledgements;
    rclcpp::Subscription<Ack>::SharedPtr ack_subscription;
    rclcpp::executors::SingleThreadedExecutor executor;
};

void startRunningPredecessor(ClientFixture & fixture) {
    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestA));
    ASSERT_TRUE(fixture.client.ConfirmManeuverGoalHandoff(kRequestA));
    fixture.client.receiveReferenceStream(
        stream(fixture.node, "hover:g1", 100, finiteReference(0.0, 2.0), kRequestA)
    );
    fixture.client.GetReference(0.02, [] {});
    ASSERT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::MANEUVER);
    ASSERT_EQ(fixture.client.last_applied_sequence_, 100U);
    ASSERT_EQ(fixture.client.active_request_identity_, kRequestA);
}

void startPredecessor(ClientFixture & fixture) {
    startRunningPredecessor(fixture);
    fixture.client.StopManeuverAfterTimeout(30000);
    ASSERT_EQ(
        fixture.client.reference_mode_.Load(),
        ManeuverReferenceClient::WAIT_FOR_MANEUVER_STOP
    );
}

bool observedAppliedAck(
    ClientFixture & fixture,
    const std::string & stream_id,
    uint64_t sequence
) {
    for (int attempt = 0; attempt < 20; ++attempt) {
        fixture.spinCallbacks();
        const bool observed = std::any_of(
            fixture.acknowledgements.begin(),
            fixture.acknowledgements.end(),
            [&stream_id, sequence](const Ack & ack) {
                return ack.consumer_status == Ack::STATUS_APPLIED &&
                    ack.stream_id == stream_id &&
                    ack.last_applied_sequence == sequence;
            }
        );
        if (observed) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
}

bool waitForAckSubscriber(ClientFixture & fixture) {
    for (int attempt = 0; attempt < 50; ++attempt) {
        fixture.spinCallbacks();
        if (fixture.client.reference_ack_publisher_->get_subscription_count() > 0U) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
}

}  // namespace

TEST(ManeuverReferenceClientTransaction, ModeCompletionRetiresOldStreamBeforeSuccessorReads) {
    RclcppContext context;
    ClientFixture fixture("reference_mode_completion");
    const auto predecessor = fixture.client.AcquireReferenceControl();
    startRunningPredecessor(fixture);
    ASSERT_TRUE(fixture.client.ReleaseReferenceControl(predecessor));
    EXPECT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::HOVER);
    bool failed = false;
    fixture.client.GetReference(0.02, [&failed] { failed = true; });
    EXPECT_FALSE(failed);
    EXPECT_TRUE(fixture.client.active_request_identity_.empty());
    EXPECT_FALSE(fixture.client.ReleaseReferenceControl(predecessor));
}

TEST(ManeuverReferenceClientTransaction, LateModeDeactivationCannotClearSuccessorReference) {
    RclcppContext context;
    ClientFixture fixture("reference_mode_late_deactivation");
    const auto predecessor = fixture.client.AcquireReferenceControl();
    startRunningPredecessor(fixture);

    // Activation can precede the predecessor's onDeactivate() callback.
    const auto successor = fixture.client.AcquireReferenceControl();
    EXPECT_NE(successor, predecessor);
    EXPECT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::HOVER);
    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestB));
    ASSERT_TRUE(fixture.client.ConfirmManeuverGoalHandoff(kRequestB));
    EXPECT_FALSE(fixture.client.ReleaseReferenceControl(predecessor));
    fixture.client.receiveReferenceStream(
        stream(fixture.node, "successor:g1", 1, finiteReference(0.0, 0.0), kRequestB));
    fixture.client.GetReference(0.02, [] {});
    EXPECT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::MANEUVER);
    EXPECT_EQ(fixture.client.active_request_identity_, kRequestB);
    EXPECT_EQ(fixture.client.last_applied_sequence_, 1U);
    EXPECT_FALSE(fixture.client.ReleaseReferenceControl(predecessor));
    EXPECT_EQ(fixture.client.active_request_identity_, kRequestB);
    EXPECT_TRUE(fixture.client.ReleaseReferenceControl(successor));
    EXPECT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::HOVER);

    // Releasing a mode never recycles its token for a later activation.
    const auto later = fixture.client.AcquireReferenceControl();
    EXPECT_NE(later, successor);
    EXPECT_FALSE(fixture.client.ReleaseReferenceControl(successor));
    EXPECT_TRUE(fixture.client.ReleaseReferenceControl(later));
}

TEST(ManeuverReferenceClientTransaction, PositiveOwnedStopAcceptsFirstStreamBeforeExpiryAndRejectsItAfterward) {
    RclcppContext context;
    ClientFixture fixture("reference_transaction_owned_pre_release_stop", 10000, 0.5);
    ASSERT_TRUE(waitForAckSubscriber(fixture));
    fixture.client.SetReferenceModeHover(true);
    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestA));
    ASSERT_TRUE(fixture.client.ConfirmManeuverGoalHandoff(kRequestA));
    ASSERT_TRUE(fixture.client.StopManeuverGoalHandoffAfterTimeout(kRequestA, 7000));
    const auto original_stop_callback = *fixture.client.stop_maneuver_timer_callback_;
    ASSERT_TRUE(original_stop_callback);
    EXPECT_EQ(
        fixture.client.reference_mode_.Load(),
        ManeuverReferenceClient::WAIT_FOR_MANEUVER_STOP
    );

    const Reference actual((*fixture.history)[0].ToState());
    fixture.client.receiveReferenceStream(
        stream(fixture.node, "hover_on_cable:g1", 1, actual.CopyWithNans(), kRequestA)
    );
    fixture.client.GetReference(0.02, [] {});
    EXPECT_EQ(
        fixture.client.reference_mode_.Load(),
        ManeuverReferenceClient::WAIT_FOR_MANEUVER_STOP
    );
    EXPECT_EQ(fixture.client.last_applied_sequence_, 1U);

    fixture.client.receiveReferenceStream(
        stream(fixture.node, "hover_on_cable:g1", 2, upwardVelocityOnlyReference(), kRequestA)
    );
    fixture.client.GetReference(0.02, [] {});
    EXPECT_EQ(
        fixture.client.reference_mode_.Load(),
        ManeuverReferenceClient::WAIT_FOR_MANEUVER_STOP
    );
    EXPECT_EQ(fixture.client.last_applied_sequence_, 2U);
    EXPECT_EQ(fixture.client.active_request_identity_, kRequestA);
    EXPECT_TRUE(observedAppliedAck(fixture, "hover_on_cable:g1", 2U));

    original_stop_callback();
    EXPECT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::HOVER);
    EXPECT_TRUE(fixture.client.active_request_identity_.empty());
    EXPECT_FALSE(fixture.client.pending_goal_handoff_);
    EXPECT_FALSE(fixture.client.latest_stream_message_);

    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestB));
    ASSERT_TRUE(fixture.client.ConfirmManeuverGoalHandoff(kRequestB));
    fixture.client.receiveReferenceStream(
        stream(fixture.node, "hover_on_cable:g1", 3, upwardVelocityOnlyReference(), kRequestA)
    );
    fixture.client.GetReference(0.02, [] {});
    EXPECT_EQ(
        fixture.client.reference_mode_.Load(),
        ManeuverReferenceClient::WAIT_FOR_MANEUVER_START
    );
    EXPECT_TRUE(fixture.client.active_request_identity_.empty());
    EXPECT_NE(fixture.client.last_applied_sequence_, 3U);
}

TEST(ManeuverReferenceClientTransaction, OwnedStopAcceptsVelocityOnlyFirstReceivedSampleWhenInitHoldWasSkipped) {
    RclcppContext context;
    ClientFixture fixture("reference_transaction_owned_velocity_only_first", 10000, 0.5);
    ASSERT_TRUE(waitForAckSubscriber(fixture));
    fixture.client.SetReferenceModeHover(true);
    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestA));
    ASSERT_TRUE(fixture.client.ConfirmManeuverGoalHandoff(kRequestA));
    ASSERT_TRUE(fixture.client.StopManeuverGoalHandoffAfterTimeout(kRequestA, 7000));

    // Latest-value delivery can skip the scheduler's initialization hold.
    // Sequence 2 is then the first sample observed by this authorized stream.
    fixture.client.receiveReferenceStream(
        stream(fixture.node, "hover_on_cable:g1", 2, upwardVelocityOnlyReference(), kRequestA)
    );
    fixture.client.GetReference(0.02, [] {});
    EXPECT_EQ(
        fixture.client.reference_mode_.Load(),
        ManeuverReferenceClient::WAIT_FOR_MANEUVER_STOP
    );
    EXPECT_EQ(fixture.client.last_applied_sequence_, 2U);
    EXPECT_EQ(fixture.client.active_request_identity_, kRequestA);
    EXPECT_TRUE(observedAppliedAck(fixture, "hover_on_cable:g1", 2U));
    EXPECT_FALSE(fixture.client.startup_reference_policy_.firstPlannedBaselinePending());

    fixture.client.receiveReferenceStream(
        stream(fixture.node, "hover_on_cable:g1", 3, upwardVelocityOnlyReference(), kRequestA)
    );
    fixture.client.GetReference(0.02, [] {});
    EXPECT_EQ(
        fixture.client.reference_mode_.Load(),
        ManeuverReferenceClient::WAIT_FOR_MANEUVER_STOP
    );
    EXPECT_EQ(fixture.client.last_applied_sequence_, 3U);
    EXPECT_TRUE(observedAppliedAck(fixture, "hover_on_cable:g1", 3U));

    fixture.client.receiveReferenceStream(
        stream(fixture.node, "hover_on_cable:g1", 4, finiteReference(0.0, 4.1), kRequestA)
    );
    fixture.client.GetReference(0.02, [] {});
    EXPECT_EQ(
        fixture.client.reference_mode_.Load(),
        ManeuverReferenceClient::REFERENCE_LOSS_STOP
    );
    EXPECT_EQ(fixture.client.last_applied_sequence_, 3U);
    EXPECT_FALSE(std::any_of(
        fixture.acknowledgements.begin(), fixture.acknowledgements.end(),
        [](const Ack & ack) {
            return ack.consumer_status == Ack::STATUS_APPLIED &&
                ack.stream_id == "hover_on_cable:g1" && ack.last_applied_sequence == 4U;
        }
    ));
}

TEST(ManeuverReferenceClientTransaction, SuccessorAcceptsFirstMpcSampleWithOrWithoutInitializationAndKeepsGuard) {
    RclcppContext context;
    for (const bool observe_initialization : {false, true}) {
        SCOPED_TRACE(observe_initialization);
        ClientFixture fixture("reference_transaction_mpc_first_sample", 10000, 0.5);
        ASSERT_TRUE(waitForAckSubscriber(fixture));
        fixture.client.SetReferenceModeHover(true);
        ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestA));
        ASSERT_TRUE(fixture.client.ConfirmManeuverGoalHandoff(kRequestA));

        const Reference hover_on_cable = upwardVelocityOnlyReference();
        fixture.client.receiveReferenceStream(
            stream(fixture.node, "hover_on_cable:g1", 1, hover_on_cable, kRequestA)
        );
        fixture.client.GetReference(0.02, [] {});
        ASSERT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::MANEUVER);
        ASSERT_EQ(fixture.client.active_request_identity_, kRequestA);

        ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestB));
        ASSERT_TRUE(fixture.client.ConfirmManeuverGoalHandoff(kRequestB));
        ASSERT_EQ(
            fixture.client.reference_mode_.Load(),
            ManeuverReferenceClient::WAIT_FOR_MANEUVER_START
        );

        // The consumer may see or skip the temporary pose-only initialization.
        // Either way, its one startup allowance belongs to the actual MPC plan.
        if (observe_initialization) {
            fixture.client.receiveReferenceStream(
                stream(fixture.node, "cable_takeoff:g2", 1, initializationHold(), kRequestB));
            fixture.client.GetReference(0.02, [] {});
            ASSERT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::MANEUVER);
            EXPECT_TRUE(fixture.client.startup_reference_policy_.firstPlannedBaselinePending());
            EXPECT_TRUE(observedAppliedAck(fixture, "cable_takeoff:g2", 1U));
        }
        fixture.client.receiveReferenceStream(
            stream(
                fixture.node,
                "cable_takeoff:g2",
                2,
                mpcReferenceWithNaNYawDerivatives(-0.373, -1.354),
                kRequestB
            )
        );
        fixture.client.GetReference(0.02, [] {});
        EXPECT_EQ(
            fixture.client.reference_mode_.Load(),
            ManeuverReferenceClient::MANEUVER
        );
        EXPECT_EQ(fixture.client.active_request_identity_, kRequestB);
        EXPECT_EQ(fixture.client.last_applied_sequence_, 2U);
        EXPECT_TRUE(observedAppliedAck(fixture, "cable_takeoff:g2", 2U));
        EXPECT_FALSE(fixture.client.startup_reference_policy_.firstPlannedBaselinePending());

        fixture.client.receiveReferenceStream(
            stream(
                fixture.node,
                "cable_takeoff:g2",
                3,
                mpcReferenceWithNaNYawDerivatives(-1.0, -1.354),
                kRequestB
            )
        );
        fixture.client.GetReference(0.02, [] {});
        EXPECT_EQ(
            fixture.client.reference_mode_.Load(),
            ManeuverReferenceClient::REFERENCE_LOSS_STOP
        );
        ASSERT_TRUE(fixture.client.fault_stream_identity_);
        EXPECT_EQ(fixture.client.fault_stream_identity_->request_identity, kRequestB);
        EXPECT_EQ(fixture.client.fault_stream_identity_->last_applied_sequence, 2U);
        EXPECT_FALSE(std::any_of(
            fixture.acknowledgements.begin(), fixture.acknowledgements.end(),
            [](const Ack & ack) {
                return ack.consumer_status == Ack::STATUS_APPLIED &&
                    ack.stream_id == "cable_takeoff:g2" && ack.last_applied_sequence == 3U;
            }
        ));
    }
}

TEST(ManeuverReferenceStartupPolicy, OnlyAllowsOnePlannedBaselineShapePerGeneration) {
    iii_drone::control::maneuver::ManeuverReferenceStartupPolicy policy;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    policy.arm();

    EXPECT_FALSE(policy.consumeFirstPlannedBaseline(initializationHold()));
    EXPECT_TRUE(policy.firstPlannedBaselinePending());

    const Reference partial_invalid(
        point_t(nan, 0.0, nan), nan, vector_t(0.0, 0.0, 2.0), 0.0,
        vector_t::Constant(nan), nan
    );
    EXPECT_FALSE(policy.consumeFirstPlannedBaseline(partial_invalid));
    EXPECT_TRUE(policy.firstPlannedBaselinePending());

    const Reference infinite_channel(
        point_t::Constant(std::numeric_limits<double>::infinity()), nan,
        vector_t(0.0, 0.0, 2.0), 0.0, vector_t::Constant(nan), nan
    );
    EXPECT_FALSE(policy.consumeFirstPlannedBaseline(infinite_channel));
    EXPECT_TRUE(policy.firstPlannedBaselinePending());

    const Reference all_nan(
        point_t::Constant(nan), nan, vector_t::Constant(nan), nan,
        vector_t::Constant(nan), nan
    );
    EXPECT_FALSE(policy.consumeFirstPlannedBaseline(all_nan));
    EXPECT_TRUE(policy.firstPlannedBaselinePending());

    EXPECT_TRUE(policy.consumeFirstPlannedBaseline(upwardVelocityOnlyReference()));
    EXPECT_FALSE(policy.firstPlannedBaselinePending());
    EXPECT_FALSE(policy.consumeFirstPlannedBaseline(finiteReference(0.0, 4.1)));
    EXPECT_FALSE(policy.firstPlannedBaselinePending());

    policy.arm();
    EXPECT_TRUE(policy.consumeFirstPlannedBaseline(mpcReferenceWithNaNYawDerivatives(-0.373)));
    EXPECT_FALSE(policy.firstPlannedBaselinePending());
    EXPECT_FALSE(policy.consumeFirstPlannedBaseline(mpcReferenceWithNaNYawDerivatives(-1.0)));
}

TEST(ManeuverReferenceClientTransaction, LateConfirmPreservesOwnedStopForRunningPredecessor) {
    RclcppContext context;
    ClientFixture fixture("reference_transaction_owned_stop_before_confirm");
    startRunningPredecessor(fixture);
    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestB));
    ASSERT_TRUE(fixture.client.StopManeuverGoalHandoffAfterTimeout(kRequestB, 7000));
    const auto original_timer = *fixture.client.stop_maneuver_timer_;
    const auto original_stop_callback = *fixture.client.stop_maneuver_timer_callback_;
    ASSERT_NE(original_timer, nullptr);
    ASSERT_TRUE(original_stop_callback);
    EXPECT_EQ(
        fixture.client.reference_mode_.Load(),
        ManeuverReferenceClient::WAIT_FOR_MANEUVER_STOP
    );

    ASSERT_TRUE(fixture.client.ConfirmManeuverGoalHandoff(kRequestB));
    EXPECT_EQ(
        fixture.client.reference_mode_.Load(),
        ManeuverReferenceClient::WAIT_FOR_MANEUVER_STOP
    );
    EXPECT_EQ(*fixture.client.stop_maneuver_timer_, original_timer);

    original_stop_callback();
    EXPECT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::HOVER);
    EXPECT_FALSE(fixture.client.pending_goal_handoff_);
    EXPECT_TRUE(fixture.client.active_request_identity_.empty());
}

TEST(ManeuverReferenceClientTransaction, InitialAcceptedGoalExpiresWithoutFirstStreamSample) {
    RclcppContext context;
    ClientFixture fixture("reference_transaction_owned_stop_no_sample");
    fixture.client.SetReferenceModeHover(true);
    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestA));
    ASSERT_TRUE(fixture.client.ConfirmManeuverGoalHandoff(kRequestA));
    ASSERT_TRUE(fixture.client.StopManeuverGoalHandoffAfterTimeout(kRequestA, 7000));
    const auto original_stop_callback = *fixture.client.stop_maneuver_timer_callback_;
    ASSERT_TRUE(original_stop_callback);
    EXPECT_EQ(
        fixture.client.reference_mode_.Load(),
        ManeuverReferenceClient::WAIT_FOR_MANEUVER_STOP
    );
    EXPECT_TRUE(fixture.client.active_request_identity_.empty());
    ASSERT_TRUE(fixture.client.pending_goal_handoff_);
    EXPECT_EQ(fixture.client.pending_goal_handoff_->request_identity, kRequestA);
    EXPECT_FALSE(fixture.client.pending_goal_handoff_->successor_consumed);

    original_stop_callback();
    EXPECT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::HOVER);
    EXPECT_FALSE(fixture.client.pending_goal_handoff_);
    EXPECT_TRUE(fixture.client.active_request_identity_.empty());
    EXPECT_FALSE(fixture.client.latest_stream_message_);
}

TEST(ManeuverReferenceClientTransaction, NewSuccessorInvalidatesOldOwnedStopCallback) {
    RclcppContext context;
    ClientFixture fixture("reference_transaction_owned_stop_successor_fence");
    fixture.client.SetReferenceModeHover(true);
    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestA));
    ASSERT_TRUE(fixture.client.ConfirmManeuverGoalHandoff(kRequestA));
    ASSERT_TRUE(fixture.client.StopManeuverGoalHandoffAfterTimeout(kRequestA, 7000));
    const auto stale_stop_callback = *fixture.client.stop_maneuver_timer_callback_;
    ASSERT_TRUE(stale_stop_callback);

    fixture.client.receiveReferenceStream(
        stream(fixture.node, "hover_on_cable:g1", 1, finiteReference(0.0, 0.0), kRequestA)
    );
    fixture.client.GetReference(0.02, [] {});
    EXPECT_EQ(
        fixture.client.reference_mode_.Load(),
        ManeuverReferenceClient::WAIT_FOR_MANEUVER_STOP
    );
    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestB));
    EXPECT_EQ(*fixture.client.stop_maneuver_timer_, nullptr);
    ASSERT_TRUE(fixture.client.ConfirmManeuverGoalHandoff(kRequestB));
    fixture.client.receiveReferenceStream(
        stream(fixture.node, "cable_takeoff:g2", 1, finiteReference(0.0, 0.0), kRequestB)
    );
    fixture.client.GetReference(0.02, [] {});
    ASSERT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::MANEUVER);
    ASSERT_EQ(fixture.client.active_request_identity_, kRequestB);

    stale_stop_callback();
    EXPECT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::MANEUVER);
    EXPECT_EQ(fixture.client.active_request_identity_, kRequestB);
    EXPECT_EQ(fixture.client.reference_stream_guard_.streamId(), "cable_takeoff:g2");
}

TEST(ManeuverReferenceClientTransaction, ImmediateOwnedStopStillCleansUpUnstartedRequest) {
    RclcppContext context;
    ClientFixture fixture("reference_transaction_owned_stop_immediate");
    fixture.client.SetReferenceModeHover(true);
    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestA));
    ASSERT_TRUE(fixture.client.ConfirmManeuverGoalHandoff(kRequestA));
    ASSERT_TRUE(fixture.client.StopManeuverGoalHandoffAfterTimeout(kRequestA, 0));

    EXPECT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::HOVER);
    EXPECT_FALSE(fixture.client.pending_goal_handoff_);
    EXPECT_TRUE(fixture.client.active_request_identity_.empty());
    EXPECT_EQ(*fixture.client.stop_maneuver_timer_, nullptr);
    fixture.client.receiveReferenceStream(
        stream(fixture.node, "hover_on_cable:g1", 1, upwardVelocityOnlyReference(), kRequestA)
    );
    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestB));
    ASSERT_TRUE(fixture.client.ConfirmManeuverGoalHandoff(kRequestB));
    fixture.client.GetReference(0.02, [] {});
    EXPECT_EQ(
        fixture.client.reference_mode_.Load(),
        ManeuverReferenceClient::WAIT_FOR_MANEUVER_START
    );
    EXPECT_TRUE(fixture.client.active_request_identity_.empty());
}

TEST(ManeuverReferenceClientTransaction, EarlySuccessorWaitStopRetainsFirstFiniteBaselineAfterNanHold) {
    RclcppContext context;
    ClientFixture fixture("reference_transaction_hold_then_finite");
    startPredecessor(fixture);
    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestB));

    fixture.client.receiveReferenceStream(
        stream(fixture.node, "cable_takeoff:g2", 1, initializationHold(), kRequestB)
    );
    fixture.client.GetReference(0.02, [] {});
    // The predecessor is in its delayed-stop phase. A held early successor
    // does not finish that stop or establish a finite successor baseline.
    EXPECT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::WAIT_FOR_MANEUVER_STOP);
    EXPECT_TRUE(fixture.client.startup_reference_policy_.firstPlannedBaselinePending());
    EXPECT_EQ(fixture.client.last_applied_sequence_, 1U);

    fixture.client.receiveReferenceStream(
        stream(fixture.node, "cable_takeoff:g2", 2, finiteReference(0.0, 0.0), kRequestB)
    );
    fixture.client.GetReference(0.02, [] {});
    EXPECT_TRUE(fixture.client.startup_reference_policy_.firstPlannedBaselinePending() == false);
    EXPECT_EQ(fixture.client.last_applied_sequence_, 2U);

    fixture.client.receiveReferenceStream(
        stream(fixture.node, "cable_takeoff:g2", 3, finiteReference(10.0, 0.0), kRequestB)
    );
    fixture.client.GetReference(0.02, [] {});

    EXPECT_EQ(
        fixture.client.reference_mode_.Load(),
        ManeuverReferenceClient::REFERENCE_LOSS_STOP
    );
    ASSERT_TRUE(fixture.client.fault_stream_identity_);
    EXPECT_EQ(fixture.client.fault_stream_identity_->stream_id, "cable_takeoff:g2");
    EXPECT_EQ(fixture.client.fault_stream_identity_->last_applied_sequence, 2U);
}

TEST(ManeuverReferenceClientTransaction, DirectEarlySuccessorPlannerSampleDoesNotUsePredecessorEnvelope) {
    RclcppContext context;
    ClientFixture fixture("reference_transaction_direct_finite");
    startPredecessor(fixture);
    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestB));

    fixture.client.receiveReferenceStream(
        stream(fixture.node, "cable_takeoff:g2", 1, finiteReference(0.0, 0.0), kRequestB)
    );
    fixture.client.GetReference(0.02, [] {});

    EXPECT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::WAIT_FOR_MANEUVER_STOP);
    EXPECT_EQ(fixture.client.last_applied_sequence_, 1U);
    EXPECT_FALSE(fixture.client.startup_reference_policy_.firstPlannedBaselinePending());
}

TEST(ManeuverReferenceClientTransaction, OrdinaryPendingHandoffAcknowledgesHealthyPredecessorBeyondGoalLatency) {
    RclcppContext context;
    ClientFixture fixture("reference_transaction_pending_predecessor_progress");
    ASSERT_TRUE(waitForAckSubscriber(fixture));
    startPredecessor(fixture);

    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestB));
    fixture.client.receiveReferenceStream(
        stream(fixture.node, "hover:g1", 101, finiteReference(0.0, 2.0), kRequestA)
    );
    fixture.client.GetReference(0.02, [] {});

    std::this_thread::sleep_for(std::chrono::milliseconds(550));
    fixture.client.receiveReferenceStream(
        stream(fixture.node, "hover:g1", 102, finiteReference(1.2, 2.0), kRequestA)
    );
    fixture.client.GetReference(0.02, [] {});

    EXPECT_EQ(fixture.client.reference_stream_guard_.streamId(), "hover:g1");
    EXPECT_EQ(fixture.client.last_applied_sequence_, 102U);
    EXPECT_EQ(fixture.client.reference_stream_guard_.lastAppliedSequence(), 102U);
    EXPECT_TRUE(observedAppliedAck(fixture, "hover:g1", 102));
}

TEST(ManeuverReferenceClientTransaction, BlendedPendingHandoffAcknowledgesHealthyPredecessorBeyondGoalLatency) {
    RclcppContext context;
    ClientFixture fixture("reference_transaction_blended_predecessor_progress");
    ASSERT_TRUE(waitForAckSubscriber(fixture));
    startRunningPredecessor(fixture);

    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestB, true));
    fixture.client.receiveReferenceStream(
        stream(fixture.node, "hover:g1", 101, finiteReference(0.0, 2.0), kRequestA)
    );
    fixture.client.GetReference(0.02, [] {});

    std::this_thread::sleep_for(std::chrono::milliseconds(550));
    fixture.client.receiveReferenceStream(
        stream(fixture.node, "hover:g1", 102, finiteReference(1.2, 2.0), kRequestA)
    );
    fixture.client.GetReference(0.02, [] {});

    EXPECT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::MANEUVER);
    EXPECT_EQ(fixture.client.reference_stream_guard_.streamId(), "hover:g1");
    EXPECT_EQ(fixture.client.last_applied_sequence_, 102U);
    EXPECT_TRUE(observedAppliedAck(fixture, "hover:g1", 102));
}

TEST(ManeuverReferenceClientTransaction, RejectedFirstSuccessorUsesGenerationLocalSequenceZero) {
    RclcppContext context;
    ClientFixture fixture("reference_transaction_successor_identity");
    startPredecessor(fixture);
    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestB));

    fixture.client.receiveReferenceStream(
        stream(
            fixture.node,
            "cable_takeoff:g2",
            1,
            finiteReference(std::numeric_limits<double>::infinity(), 0.0),
            kRequestB
        )
    );
    fixture.client.GetReference(0.02, [] {});

    ASSERT_TRUE(fixture.client.fault_stream_identity_);
    EXPECT_EQ(fixture.client.fault_stream_identity_->stream_id, "cable_takeoff:g2");
    EXPECT_EQ(fixture.client.fault_stream_identity_->request_identity, kRequestB);
    EXPECT_EQ(fixture.client.fault_stream_identity_->last_applied_sequence, 0U);
    EXPECT_NE(
        fixture.client.fault_stream_identity_->last_applied_sequence,
        100U
    );
}

TEST(ManeuverReferenceClientTransaction, GoalAcceptanceAdoptsEarlyFiniteSuccessorWithoutRearmingG3) {
    RclcppContext context;
    ClientFixture fixture("reference_transaction_early_goal_finite");
    startPredecessor(fixture);

    ASSERT_NE(*fixture.client.stop_maneuver_timer_, nullptr);
    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestB));
    EXPECT_EQ(*fixture.client.stop_maneuver_timer_, nullptr);
    fixture.client.receiveReferenceStream(
        stream(fixture.node, "cable_takeoff:g2", 1, finiteReference(0.0, 0.0), kRequestB)
    );
    fixture.client.GetReference(0.02, [] {});
    ASSERT_TRUE(fixture.client.pending_goal_handoff_);
    EXPECT_TRUE(fixture.client.pending_goal_handoff_->successor_consumed);
    EXPECT_EQ(fixture.client.last_applied_sequence_, 1U);
    EXPECT_FALSE(fixture.client.startup_reference_policy_.firstPlannedBaselinePending());

    ASSERT_TRUE(fixture.client.ConfirmManeuverGoalHandoff(kRequestB));
    EXPECT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::MANEUVER);
    EXPECT_EQ(fixture.client.reference_stream_guard_.streamId(), "cable_takeoff:g2");
    EXPECT_EQ(fixture.client.last_applied_sequence_, 1U);

    fixture.client.receiveReferenceStream(
        stream(fixture.node, "cable_takeoff:g3", 1, finiteReference(0.0, 0.0), kRequestB)
    );
    fixture.client.GetReference(0.02, [] {});
    EXPECT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::MANEUVER);
    EXPECT_EQ(fixture.client.reference_stream_guard_.streamId(), "cable_takeoff:g2");
    EXPECT_EQ(fixture.client.last_applied_sequence_, 1U);

    fixture.client.receiveReferenceStream(
        stream(fixture.node, "hover:g1", 101, finiteReference(0.0, 2.0), kRequestA)
    );
    fixture.client.GetReference(0.02, [] {});
    EXPECT_EQ(fixture.client.reference_stream_guard_.streamId(), "cable_takeoff:g2");
    EXPECT_EQ(fixture.client.last_applied_sequence_, 1U);

    fixture.client.receiveReferenceStream(
        stream(fixture.node, "cable_takeoff:g2", 2, finiteReference(10.0, 0.0), kRequestB)
    );
    fixture.client.GetReference(0.02, [] {});
    EXPECT_EQ(
        fixture.client.reference_mode_.Load(),
        ManeuverReferenceClient::REFERENCE_LOSS_STOP
    );
}

TEST(ManeuverReferenceClientTransaction, GoalAcceptanceRetainsEarlyHoldForFirstFiniteSuccessorBaseline) {
    RclcppContext context;
    ClientFixture fixture("reference_transaction_early_goal_hold");
    startPredecessor(fixture);

    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestB));
    fixture.client.receiveReferenceStream(
        stream(fixture.node, "cable_takeoff:g2", 1, initializationHold(), kRequestB)
    );
    fixture.client.GetReference(0.02, [] {});
    ASSERT_TRUE(fixture.client.ConfirmManeuverGoalHandoff(kRequestB));
    EXPECT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::MANEUVER);
    EXPECT_TRUE(fixture.client.startup_reference_policy_.firstPlannedBaselinePending());
    EXPECT_EQ(fixture.client.last_applied_sequence_, 1U);

    fixture.client.receiveReferenceStream(
        stream(fixture.node, "cable_takeoff:g2", 2, finiteReference(0.0, 0.0), kRequestB)
    );
    fixture.client.GetReference(0.02, [] {});
    EXPECT_FALSE(fixture.client.startup_reference_policy_.firstPlannedBaselinePending());
    EXPECT_EQ(fixture.client.last_applied_sequence_, 2U);

    fixture.client.receiveReferenceStream(
        stream(fixture.node, "cable_takeoff:g2", 3, finiteReference(10.0, 0.0), kRequestB)
    );
    fixture.client.GetReference(0.02, [] {});
    EXPECT_EQ(
        fixture.client.reference_mode_.Load(),
        ManeuverReferenceClient::REFERENCE_LOSS_STOP
    );
}

TEST(ManeuverReferenceClientTransaction, GoalAcceptanceAfterSubmissionHoldsPredecessorUntilG2) {
    RclcppContext context;
    ClientFixture fixture("reference_transaction_goal_then_successor");
    startPredecessor(fixture);

    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestB));
    ASSERT_TRUE(fixture.client.ConfirmManeuverGoalHandoff(kRequestB));
    EXPECT_EQ(
        fixture.client.reference_mode_.Load(),
        ManeuverReferenceClient::WAIT_FOR_MANEUVER_START
    );

    fixture.client.receiveReferenceStream(
        stream(fixture.node, "hover:g1", 101, finiteReference(0.0, 2.0), kRequestA)
    );
    fixture.client.GetReference(0.02, [] {});
    EXPECT_EQ(
        fixture.client.reference_mode_.Load(),
        ManeuverReferenceClient::WAIT_FOR_MANEUVER_START
    );
    EXPECT_EQ(fixture.client.reference_stream_guard_.streamId(), "hover:g1");
    EXPECT_EQ(fixture.client.reference_stream_guard_.lastAppliedSequence(), 101U);

    fixture.client.receiveReferenceStream(
        stream(fixture.node, "cable_takeoff:g2", 1, finiteReference(0.0, 0.0), kRequestB)
    );
    fixture.client.GetReference(0.02, [] {});
    EXPECT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::MANEUVER);
    EXPECT_EQ(fixture.client.reference_stream_guard_.streamId(), "cable_takeoff:g2");
    EXPECT_EQ(fixture.client.last_applied_sequence_, 1U);
}

TEST(ManeuverReferenceClientTransaction, RejectedGoalRevokesPendingHandoffThroughSafeHover) {
    RclcppContext context;
    ClientFixture fixture("reference_transaction_goal_rejected");
    startPredecessor(fixture);

    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestB));
    EXPECT_EQ(*fixture.client.stop_maneuver_timer_, nullptr);
    EXPECT_TRUE(fixture.client.CancelManeuverGoalHandoff(kRequestB));

    EXPECT_FALSE(fixture.client.pending_goal_handoff_);
    EXPECT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::HOVER);
    EXPECT_TRUE(fixture.client.active_stream_id_.empty());
}

TEST(ManeuverReferenceClientTransaction, RejectedBlendedGoalBeforeAcceptanceRestoresRunningPredecessor) {
    RclcppContext context;
    ClientFixture fixture("reference_transaction_blended_rejected_before_acceptance");
    startRunningPredecessor(fixture);

    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestB, true));
    EXPECT_TRUE(fixture.client.CancelManeuverGoalHandoff(kRequestB));

    EXPECT_FALSE(fixture.client.pending_goal_handoff_);
    EXPECT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::MANEUVER);
    EXPECT_EQ(fixture.client.reference_stream_guard_.streamId(), "hover:g1");
    EXPECT_EQ(fixture.client.last_applied_sequence_, 100U);
}

TEST(ManeuverReferenceClientTransaction, BlendedCancellationRetiresCachedSuccessorAndRetainsPredecessor) {
    RclcppContext context;
    ClientFixture fixture("reference_transaction_blended_cached_successor");
    startRunningPredecessor(fixture);

    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestB, true));
    fixture.client.receiveReferenceStream(
        stream(fixture.node, "pending-b:g2", 1, finiteReference(0.0, 0.0), kRequestB)
    );
    ASSERT_TRUE(fixture.client.latest_stream_message_);
    EXPECT_EQ(fixture.client.latest_stream_message_->request_identity, kRequestB);

    EXPECT_TRUE(fixture.client.CancelManeuverGoalHandoff(kRequestB));
    EXPECT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::MANEUVER);
    EXPECT_FALSE(fixture.client.pending_goal_handoff_);
    EXPECT_FALSE(fixture.client.latest_stream_message_);
    EXPECT_EQ(fixture.client.reference_stream_guard_.streamId(), "hover:g1");
    EXPECT_EQ(fixture.client.active_request_identity_, kRequestA);

    fixture.client.receiveReferenceStream(
        stream(fixture.node, "hover:g1", 101, finiteReference(0.0, 2.0), kRequestA)
    );
    fixture.client.GetReference(0.02, [] {});
    EXPECT_EQ(fixture.client.reference_stream_guard_.streamId(), "hover:g1");
    EXPECT_EQ(fixture.client.last_applied_sequence_, 101U);
    EXPECT_EQ(fixture.client.active_request_identity_, kRequestA);
}

TEST(ManeuverReferenceClientTransaction, HaltedBlendedGoalAfterAcceptanceUsesSafeStopBeforeG2) {
    RclcppContext context;
    ClientFixture fixture("reference_transaction_blended_halted_after_acceptance");
    startRunningPredecessor(fixture);

    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestB, true));
    ASSERT_TRUE(fixture.client.ConfirmManeuverGoalHandoff(kRequestB));
    EXPECT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::MANEUVER);
    EXPECT_TRUE(fixture.client.CancelManeuverGoalHandoff(kRequestB));

    // Acceptance followed by terminal cancellation uses the immediate safe
    // stop; delayed-stop is only a predecessor's pre-terminal state.
    EXPECT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::HOVER);
    EXPECT_TRUE(fixture.client.active_stream_id_.empty());
}

TEST(ManeuverReferenceClientTransaction, LateGoalAcceptanceCannotReplaceReferenceLossStop) {
    RclcppContext context;
    ClientFixture fixture("reference_transaction_late_accept_after_fault");
    startPredecessor(fixture);

    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestB));
    EXPECT_FALSE(fixture.client.BeginManeuverGoalHandoff(kRequestB));
    fixture.client.receiveReferenceStream(
        stream(fixture.node, "cable_takeoff:g2", 1, finiteReference(0.0, 0.0), kRequestB)
    );
    fixture.client.GetReference(0.02, [] {});
    fixture.client.receiveReferenceStream(
        stream(fixture.node, "cable_takeoff:g2", 2, finiteReference(10.0, 0.0), kRequestB)
    );
    fixture.client.GetReference(0.02, [] {});

    EXPECT_EQ(
        fixture.client.reference_mode_.Load(),
        ManeuverReferenceClient::REFERENCE_LOSS_STOP
    );
    EXPECT_FALSE(fixture.client.pending_goal_handoff_);
    EXPECT_FALSE(fixture.client.ConfirmManeuverGoalHandoff(kRequestB));
    EXPECT_EQ(
        fixture.client.reference_mode_.Load(),
        ManeuverReferenceClient::REFERENCE_LOSS_STOP
    );
}

TEST(ManeuverReferenceClientTransaction, InitialCanceledCachedIdentityCannotConsumeNextGoal) {
    RclcppContext context;
    ClientFixture fixture("reference_transaction_initial_cached_identity");
    ASSERT_TRUE(waitForAckSubscriber(fixture));
    fixture.client.SetReferenceModeHover();
    ASSERT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::HOVER);

    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestA));
    fixture.client.receiveReferenceStream(
        stream(fixture.node, "initial-a:g1", 1, finiteReference(0.0, 0.0), kRequestA)
    );
    ASSERT_TRUE(fixture.client.latest_stream_message_);
    EXPECT_EQ(fixture.client.latest_stream_message_->request_identity, kRequestA);

    EXPECT_TRUE(fixture.client.CancelManeuverGoalHandoff(kRequestA));
    EXPECT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::HOVER);
    EXPECT_FALSE(fixture.client.pending_goal_handoff_);
    EXPECT_FALSE(fixture.client.latest_stream_message_);

    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestB));
    ASSERT_TRUE(fixture.client.ConfirmManeuverGoalHandoff(kRequestB));
    fixture.client.GetReference(0.02, [] {});
    fixture.spinCallbacks();
    EXPECT_TRUE(fixture.client.reference_stream_guard_.streamId().empty());
    EXPECT_EQ(fixture.client.last_applied_sequence_, 0U);
    EXPECT_FALSE(fixture.client.latest_stream_message_);
    EXPECT_FALSE(fixture.client.fault_stream_identity_);
    EXPECT_FALSE(std::any_of(
        fixture.acknowledgements.begin(),
        fixture.acknowledgements.end(),
        [](const Ack & ack) {
            return ack.stream_id == "initial-a:g1" &&
                ack.last_applied_sequence == 1U;
        }
    ));

    fixture.client.receiveReferenceStream(
        stream(fixture.node, "initial-b:g2", 1, finiteReference(0.0, 0.0), kRequestB)
    );
    fixture.client.GetReference(0.02, [] {});
    EXPECT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::MANEUVER);
    EXPECT_EQ(fixture.client.active_request_identity_, kRequestB);
    EXPECT_EQ(fixture.client.reference_stream_guard_.streamId(), "initial-b:g2");
}

TEST(ManeuverReferenceClientTransaction, CanceledRequestCannotClaimTheNextGoalGeneration) {
    RclcppContext context;
    ClientFixture fixture("reference_transaction_request_a_then_b");

    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestA));
    ASSERT_TRUE(fixture.client.ConfirmManeuverGoalHandoff(kRequestA));
    EXPECT_TRUE(fixture.client.CancelManeuverGoalHandoff(kRequestA));
    EXPECT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::HOVER);

    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestB));
    ASSERT_TRUE(fixture.client.ConfirmManeuverGoalHandoff(kRequestB));
    ASSERT_FALSE(fixture.client.latest_stream_message_);
    fixture.client.receiveReferenceStream(
        stream(fixture.node, "late-a:g2", 1, finiteReference(0.0, 0.0), kRequestA)
    );
    EXPECT_FALSE(fixture.client.latest_stream_message_);
    EXPECT_TRUE(fixture.client.reference_stream_guard_.streamId().empty());
    EXPECT_EQ(fixture.client.last_applied_sequence_, 0U);

    fixture.client.receiveReferenceStream(
        stream(fixture.node, "goal-b:g3", 1, finiteReference(0.0, 0.0), kRequestB)
    );
    fixture.client.GetReference(0.02, [] {});
    EXPECT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::MANEUVER);
    EXPECT_EQ(fixture.client.reference_stream_guard_.streamId(), "goal-b:g3");
    EXPECT_EQ(fixture.client.active_request_identity_, kRequestB);
    EXPECT_EQ(fixture.client.last_applied_sequence_, 1U);
}

TEST(ManeuverReferenceClientTransaction, StaleCancellationCannotRevokeNewerPendingHandoff) {
    RclcppContext context;
    ClientFixture fixture("reference_transaction_keyed_cancel");
    startPredecessor(fixture);

    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestB));
    EXPECT_FALSE(fixture.client.CancelManeuverGoalHandoff(kRequestA));
    ASSERT_TRUE(fixture.client.pending_goal_handoff_);
    EXPECT_EQ(fixture.client.pending_goal_handoff_->request_identity, kRequestB);
    EXPECT_EQ(
        fixture.client.reference_mode_.Load(),
        ManeuverReferenceClient::WAIT_FOR_MANEUVER_STOP
    );

    EXPECT_TRUE(fixture.client.CancelManeuverGoalHandoff(kRequestB));
    EXPECT_FALSE(fixture.client.pending_goal_handoff_);
    EXPECT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::HOVER);
}

TEST(ManeuverReferenceClientTransaction, EarlyConsumedSuccessorFailureRetiresActiveStreamAndCache) {
    RclcppContext context;
    ClientFixture fixture("reference_transaction_early_successor_failure");
    startRunningPredecessor(fixture);
    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestB));
    fixture.client.receiveReferenceStream(
        stream(fixture.node, "goal-b:g2", 1, finiteReference(0.0, 2.0), kRequestB)
    );
    fixture.client.GetReference(0.02, [] {});
    ASSERT_TRUE(fixture.client.pending_goal_handoff_);
    ASSERT_TRUE(fixture.client.pending_goal_handoff_->successor_consumed);
    ASSERT_EQ(fixture.client.active_request_identity_, kRequestB);

    EXPECT_TRUE(fixture.client.CancelManeuverGoalHandoff(kRequestB));
    EXPECT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::HOVER);
    EXPECT_TRUE(fixture.client.active_request_identity_.empty());
    EXPECT_FALSE(fixture.client.latest_stream_message_);
}

TEST(ManeuverReferenceClientTransaction, ConfirmedSuccessorTerminalRetiresOwnedStreamAndStaleAIsNoOp) {
    RclcppContext context;
    ClientFixture fixture("reference_transaction_confirmed_terminal");
    startRunningPredecessor(fixture);
    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestB));
    fixture.client.receiveReferenceStream(
        stream(fixture.node, "goal-b:g2", 1, finiteReference(0.0, 2.0), kRequestB)
    );
    fixture.client.GetReference(0.02, [] {});
    ASSERT_TRUE(fixture.client.ConfirmManeuverGoalHandoff(kRequestB));
    ASSERT_FALSE(fixture.client.pending_goal_handoff_);
    ASSERT_EQ(fixture.client.active_request_identity_, kRequestB);

    EXPECT_TRUE(fixture.client.CancelManeuverGoalHandoff(kRequestB));
    EXPECT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::HOVER);
    EXPECT_TRUE(fixture.client.active_request_identity_.empty());
    EXPECT_FALSE(fixture.client.latest_stream_message_);
    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestC));
    EXPECT_FALSE(fixture.client.CancelManeuverGoalHandoff(kRequestB));
    ASSERT_TRUE(fixture.client.pending_goal_handoff_);
    EXPECT_EQ(fixture.client.pending_goal_handoff_->request_identity, kRequestC);
}

TEST(ManeuverReferenceClientTransaction, IdentityBoundSuccessPreservesTargetReferenceAndRetiresCache) {
    RclcppContext context;
    ClientFixture fixture("reference_transaction_success_target");
    startRunningPredecessor(fixture);
    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestB));
    fixture.client.receiveReferenceStream(
        stream(fixture.node, "goal-b:g2", 1, finiteReference(0.0, 2.0), kRequestB)
    );
    fixture.client.GetReference(0.02, [] {});
    ASSERT_TRUE(fixture.client.ConfirmManeuverGoalHandoff(kRequestB));
    ASSERT_FALSE(fixture.client.pending_goal_handoff_);

    const auto final_reference = finiteReference(3.0, 0.0);
    EXPECT_TRUE(fixture.client.CompleteManeuverGoalHandoff(kRequestB, final_reference));
    EXPECT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::HOVER);
    EXPECT_EQ(fixture.client.reference_.Load().position().x(), 3.0);
    EXPECT_TRUE(fixture.client.active_request_identity_.empty());
    EXPECT_FALSE(fixture.client.latest_stream_message_);
    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestC));
    EXPECT_FALSE(fixture.client.CompleteManeuverGoalHandoff(kRequestB, finiteReference(99.0, 0.0)));
    EXPECT_EQ(fixture.client.reference_.Load().position().x(), 3.0);
}

TEST(ManeuverReferenceClientTransaction, DeferredTerminalStopDoesNotCompletePendingObjectHandoff) {
    RclcppContext context;
    ClientFixture fixture("deferred_object_result");
    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestA));
    ASSERT_TRUE(fixture.client.ConfirmManeuverGoalHandoff(kRequestA));
    auto predecessor = stream(fixture.node, "ftp:terminal", 1,
        finiteReference(0.24, 0.0), kRequestA);
    predecessor->terminal_hold_active = true;
    fixture.client.receiveReferenceStream(predecessor);
    const auto applied = fixture.client.GetReference(0.02, [] {});
    ASSERT_EQ(fixture.client.active_request_identity_, kRequestA);
    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestB));
    ASSERT_TRUE(fixture.client.ConfirmManeuverGoalHandoff(kRequestB));
    const auto nominal_object_result = finiteReference(0.0, 0.0).CopyWithNans();
    EXPECT_FALSE(fixture.client.CompleteManeuverGoalHandoff(
        kRequestB, nominal_object_result));
    ASSERT_TRUE(fixture.client.pending_goal_handoff_);
    EXPECT_EQ(fixture.client.pending_goal_handoff_->request_identity, kRequestB);
    EXPECT_EQ(fixture.client.active_request_identity_, kRequestA);
    EXPECT_LT((fixture.client.reference_.Load().position() - applied.position()).norm(), 1.0e-6);
    EXPECT_FALSE(fixture.client.BeginManeuverGoalHandoff(kRequestC));
}

TEST(ManeuverReferenceClientTransaction, MissingObjectGenerationUsesOwnedCancelWithoutNominalOverwrite) {
    RclcppContext context;
    ClientFixture fixture("missing_object_first_ack", 1);
    const auto owner = fixture.client.AcquireReferenceControl();
    ASSERT_NE(owner, 0U);
    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestA));
    ASSERT_TRUE(fixture.client.ConfirmManeuverGoalHandoff(kRequestA));
    auto predecessor = stream(fixture.node, "ftp:terminal", 1,
        finiteReference(0.24, 0.0), kRequestA);
    predecessor->terminal_hold_active = true;
    fixture.client.receiveReferenceStream(predecessor);
    const auto applied = fixture.client.GetReference(0.02, [] {});
    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestB));
    ASSERT_TRUE(fixture.client.ConfirmManeuverGoalHandoff(kRequestB));
    fixture.client.maneuver_start_time_.Store(
        rclcpp::Clock().now() - rclcpp::Duration::from_seconds(1.0));
    bool loss_callback = false;
    fixture.client.GetReference(0.02, [&] {
        loss_callback = true;
        EXPECT_TRUE(fixture.client.CancelManeuverGoalHandoff(kRequestB));
    });
    EXPECT_TRUE(loss_callback);
    EXPECT_FALSE(fixture.client.pending_goal_handoff_);
    EXPECT_EQ(fixture.client.active_request_identity_, kRequestA);
    EXPECT_LT((fixture.client.reference_.Load().position() - applied.position()).norm(), 1.0e-6);
    EXPECT_EQ(fixture.client.reference_mode_.Load(),
        ManeuverReferenceClient::WAIT_FOR_MANEUVER_START);
    int repeated_failure_callbacks = 0;
    fixture.client.GetReference(0.02, [&] { ++repeated_failure_callbacks; });
    EXPECT_EQ(repeated_failure_callbacks, 1);
    EXPECT_EQ(fixture.client.active_request_identity_, kRequestA);
    EXPECT_LT((fixture.client.reference_.Load().position() - applied.position()).norm(), 1.0e-6);
    EXPECT_TRUE(fixture.client.ReleaseReferenceControl(owner));
    EXPECT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::HOVER);
    EXPECT_TRUE(fixture.client.active_request_identity_.empty());
}

TEST(ManeuverReferenceClientTransaction, PendingHandoffRejectsNewGenerationWithPredecessorIdentity) {
    RclcppContext context;
    ClientFixture fixture("reference_transaction_predecessor_identity_generation");
    startRunningPredecessor(fixture);

    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestB));
    fixture.client.receiveReferenceStream(
        stream(fixture.node, "late-a:g2", 1, finiteReference(0.0, 0.0), kRequestA)
    );
    fixture.client.GetReference(0.02, [] {});

    EXPECT_EQ(fixture.client.reference_stream_guard_.streamId(), "hover:g1");
    EXPECT_EQ(fixture.client.reference_stream_guard_.lastAppliedSequence(), 100U);
    EXPECT_EQ(fixture.client.active_request_identity_, kRequestA);
    ASSERT_TRUE(fixture.client.pending_goal_handoff_);
    EXPECT_FALSE(fixture.client.pending_goal_handoff_->successor_consumed);

    fixture.client.receiveReferenceStream(
        stream(fixture.node, "goal-b:g3", 1, finiteReference(0.0, 0.0), kRequestB)
    );
    fixture.client.GetReference(0.02, [] {});
    EXPECT_EQ(fixture.client.reference_stream_guard_.streamId(), "goal-b:g3");
    EXPECT_EQ(fixture.client.active_request_identity_, kRequestB);
}

TEST(ManeuverReferenceClientTransaction, EmptyMalformedAndWrongRequestIdentityHaveNoIngressEffects) {
    RclcppContext context;
    ClientFixture fixture("reference_transaction_invalid_request_identity");
    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestB));
    ASSERT_TRUE(fixture.client.ConfirmManeuverGoalHandoff(kRequestB));

    fixture.client.receiveReferenceStream(
        stream(fixture.node, "invalid:empty", 1, finiteReference(0.0, 0.0), "")
    );
    fixture.client.receiveReferenceStream(
        stream(fixture.node, "invalid:legacy", 1, finiteReference(0.0, 0.0), "legacy-request")
    );
    fixture.client.receiveReferenceStream(
        stream(fixture.node, "invalid:wrong", 1, finiteReference(0.0, 0.0), kRequestC)
    );
    EXPECT_FALSE(fixture.client.latest_stream_message_);
    EXPECT_TRUE(fixture.client.reference_stream_guard_.streamId().empty());
    EXPECT_EQ(fixture.client.last_applied_sequence_, 0U);

    fixture.client.receiveReferenceStream(
        stream(fixture.node, "valid-b:g2", 1, finiteReference(0.0, 0.0), kRequestB)
    );
    fixture.client.GetReference(0.02, [] {});
    EXPECT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::MANEUVER);
    EXPECT_EQ(fixture.client.active_request_identity_, kRequestB);
}

TEST(ManeuverReferenceClientTransaction, CommittedRebasePromotesTheFaultingRequestOwner) {
    RclcppContext context;
    ClientFixture fixture("reference_transaction_rebase_request_owner");
    fixture.client.reference_mode_.Store(ManeuverReferenceClient::REFERENCE_LOSS_STOP);
    fixture.client.recovery_phase_ = ManeuverReferenceClient::RecoveryPhase::WaitActive;
    fixture.client.recovery_phase_started_ = std::chrono::steady_clock::now();
    fixture.client.prepared_reference_anchor_ = finiteReference(0.0, 0.0);
    fixture.client.active_stream_id_ = "rebased-b:g2";
    fixture.client.active_request_identity_ = kRequestA;
    fixture.client.fault_stream_identity_ = ManeuverReferenceClient::ReferenceStreamIdentity{
        "rebased-b:g2", kRequestB, 0
    };
    fixture.client.reference_stream_guard_.expectGeneration("rebased-b:g2");

    fixture.client.receiveReferenceStream(
        stream(fixture.node, "rebased-b:g2", 1, finiteReference(0.0, 0.0), kRequestB)
    );
    Reference reference;
    std::string reference_mode;
    EXPECT_TRUE(fixture.client.advanceReferenceRecovery(reference, [] {}, reference_mode));
    EXPECT_EQ(reference_mode, "maneuver_rebased");
    EXPECT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::MANEUVER);
    EXPECT_EQ(fixture.client.active_request_identity_, kRequestB);
    EXPECT_FALSE(fixture.client.fault_stream_identity_);

    fixture.client.receiveReferenceStream(
        stream(fixture.node, "late-a:g3", 1, finiteReference(0.0, 0.0), kRequestA)
    );
    EXPECT_EQ(fixture.client.reference_stream_guard_.streamId(), "rebased-b:g2");
}

TEST(ManeuverReferenceClientTransaction, StartStopInterleaveCannotCommitAPartialGeneration) {
    RclcppContext context;
    ClientFixture fixture("reference_transaction_start_stop_interleave");
    startPredecessor(fixture);
    fixture.client.receiveReferenceStream(
        stream(fixture.node, "cable_takeoff:g2", 1, finiteReference(0.0, 0.0), kRequestB)
    );

    std::unique_lock<std::recursive_mutex> transition_lock(fixture.client.transition_mutex_);
    std::promise<void> reader_started;
    std::promise<void> transition_started;
    auto reader = std::async(std::launch::async, [&fixture, &reader_started] {
        reader_started.set_value();
        fixture.client.GetReference(0.02, [] {});
    });
    auto transition = std::async(std::launch::async, [&fixture, &transition_started] {
        transition_started.set_value();
        fixture.client.StartManeuver();
        fixture.client.StopManeuver();
    });

    EXPECT_EQ(
        reader_started.get_future().wait_for(std::chrono::seconds(1)),
        std::future_status::ready
    );
    EXPECT_EQ(
        transition_started.get_future().wait_for(std::chrono::seconds(1)),
        std::future_status::ready
    );
    transition_lock.unlock();

    EXPECT_EQ(reader.wait_for(std::chrono::seconds(2)), std::future_status::ready);
    EXPECT_EQ(transition.wait_for(std::chrono::seconds(2)), std::future_status::ready);
    EXPECT_NO_THROW(reader.get());
    EXPECT_NO_THROW(transition.get());
    EXPECT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::HOVER);
    EXPECT_TRUE(fixture.client.active_stream_id_.empty());
    EXPECT_EQ(fixture.client.last_applied_sequence_, 0U);
}

TEST(ManeuverReferenceClientTransaction, ReentrantFailureCallbackReturnsWithoutTransitionLockDeadlock) {
    RclcppContext context;
    ClientFixture fixture("reference_transaction_reentrant_failure");
    fixture.client.reference_mode_.Store(ManeuverReferenceClient::REFERENCE_LOSS_STOP);
    fixture.client.recovery_phase_ = ManeuverReferenceClient::RecoveryPhase::WaitActive;
    fixture.client.recovery_phase_started_ = std::chrono::steady_clock::now();
    fixture.client.prepared_reference_anchor_ = finiteReference(0.0, 0.0);
    fixture.client.active_stream_id_ = "rebased:g3";
    fixture.client.active_request_identity_ = kRequestB;
    fixture.client.fault_stream_identity_ = ManeuverReferenceClient::ReferenceStreamIdentity{
        "rebased:g3", kRequestB, 0
    };
    fixture.client.reference_stream_guard_.expectGeneration("rebased:g3");
    fixture.client.receiveReferenceStream(
        stream(fixture.node, "rebased:g3", 1, finiteReference(10.0, 0.0), kRequestB)
    );

    auto result = std::async(std::launch::async, [&fixture] {
        Reference reference;
        std::string reference_mode;
        return fixture.client.advanceReferenceRecovery(
            reference,
            [&fixture] { fixture.client.SetReferenceModeHover(true); },
            reference_mode
        );
    });

    EXPECT_EQ(result.wait_for(std::chrono::seconds(2)), std::future_status::ready);
    EXPECT_FALSE(result.get());
    EXPECT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::HOVER);
}

void expectFailedReadCannotHoverSuccessor(bool fail_during_maneuver) {
    RclcppContext context;
    ClientFixture fixture(
        fail_during_maneuver ? "reference_failed_maneuver_successor" : "reference_failed_start_successor",
        1
    );
    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestA));
    ASSERT_TRUE(fixture.client.ConfirmManeuverGoalHandoff(kRequestA));
    if (fail_during_maneuver) {
        // Reach the legacy first-reference failure branch without an accepted
        // stream; that branch must obey the same ownership fence as timeout.
        fixture.client.reference_mode_.Store(ManeuverReferenceClient::MANEUVER);
        fixture.client.failed_attempts_ = 9999;
    } else {
        fixture.client.maneuver_start_time_.Store(
            rclcpp::Clock().now() - rclcpp::Duration::from_seconds(1.0)
        );
    }

    std::promise<void> callback_finished_cleanup;
    std::promise<void> resume_callback;
    auto resume = resume_callback.get_future().share();
    auto reader = std::async(std::launch::async, [&] {
        fixture.client.GetReference(0.02, [&] {
            EXPECT_TRUE(fixture.client.CancelManeuverGoalHandoff(kRequestA));
            callback_finished_cleanup.set_value();
            resume.wait();
        });
    });

    const bool reached = callback_finished_cleanup.get_future().wait_for(
        std::chrono::seconds(2)
    ) == std::future_status::ready;
    EXPECT_TRUE(reached);
    if (reached) {
        EXPECT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestB));
        EXPECT_TRUE(fixture.client.ConfirmManeuverGoalHandoff(kRequestB));
        fixture.client.receiveReferenceStream(
            stream(fixture.node, "successor:g2", 1, finiteReference(4.0, 0.0), kRequestB)
        );
    }
    resume_callback.set_value();
    EXPECT_EQ(reader.wait_for(std::chrono::seconds(2)), std::future_status::ready);
    reader.get();
    if (reached) {
        EXPECT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::WAIT_FOR_MANEUVER_START);
        ASSERT_TRUE(fixture.client.pending_goal_handoff_);
        EXPECT_EQ(fixture.client.pending_goal_handoff_->request_identity, kRequestB);
        ASSERT_TRUE(fixture.client.latest_stream_message_);
        EXPECT_EQ(fixture.client.latest_stream_message_->request_identity, kRequestB);
    }
}

TEST(ManeuverReferenceClientTransaction, WaitStartFailureCannotHoverSuccessorAfterCallbackCleanup) {
    expectFailedReadCannotHoverSuccessor(false);
}

TEST(ManeuverReferenceClientTransaction, FirstReferenceFailureCannotHoverSuccessorAfterCallbackCleanup) {
    expectFailedReadCannotHoverSuccessor(true);
}

TEST(ManeuverReferenceClientTransaction, FailedReadWithoutSuccessorStillHovers) {
    RclcppContext context;
    ClientFixture fixture("reference_failed_start_without_successor", 1);
    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestA));
    ASSERT_TRUE(fixture.client.ConfirmManeuverGoalHandoff(kRequestA));
    fixture.client.maneuver_start_time_.Store(
        rclcpp::Clock().now() - rclcpp::Duration::from_seconds(1.0)
    );
    bool failed = false;
    fixture.client.GetReference(0.02, [&] { failed = true; });
    EXPECT_TRUE(failed);
    EXPECT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::HOVER);
}

TEST(ManeuverReferenceClientTransaction, OldHaltCannotPoisonNewModeOwnerAfterReset) {
    RclcppContext context;
    ClientFixture fixture("terminal_retention_owner_fence");
    const auto first_owner = fixture.client.AcquireReferenceControl();
    fixture.client.active_request_identity_ = kRequestA;
    fixture.client.ResetTerminalRetentionFailure();
    EXPECT_TRUE(fixture.client.ReportTerminalRetentionFailure(kRequestA));
    EXPECT_TRUE(fixture.client.TerminalRetentionFailed());
    ASSERT_TRUE(fixture.client.ReleaseReferenceControl(first_owner));

    const auto second_owner = fixture.client.AcquireReferenceControl();
    ASSERT_NE(first_owner, second_owner);
    fixture.client.active_request_identity_ = kRequestB;
    fixture.client.ResetTerminalRetentionFailure();
    EXPECT_FALSE(fixture.client.ReportTerminalRetentionFailure(kRequestA));
    EXPECT_FALSE(fixture.client.TerminalRetentionFailed());
    EXPECT_TRUE(fixture.client.ReportTerminalRetentionFailure(kRequestB));
    EXPECT_TRUE(fixture.client.TerminalRetentionFailed());
}

TEST(ManeuverReferenceClientTransaction, NewerUnappliedTerminalSampleCannotCauseMeasuredStopFallback) {
    RclcppContext context;
    ClientFixture fixture("terminal_stop_applied_owner_test");
    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestA));
    ASSERT_TRUE(fixture.client.ConfirmManeuverGoalHandoff(kRequestA));
    auto first = stream(fixture.node, "terminal:owned", 1, finiteReference(0.2, 0.0), kRequestA);
    first->terminal_hold_active = true;
    fixture.client.receiveReferenceStream(first);
    const auto accepted = fixture.client.GetReference(0.02, [] {});
    ASSERT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::MANEUVER);
    auto newer = stream(fixture.node, "terminal:owned", 2, finiteReference(0.3, 0.0), kRequestA);
    newer->terminal_hold_active = true;
    fixture.client.receiveReferenceStream(newer);
    fixture.client.StopManeuver();
    EXPECT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::MANEUVER);
    EXPECT_LT((fixture.client.reference_.Load().position() - accepted.position()).norm(), 1.0e-6);
    EXPECT_EQ(fixture.client.last_applied_sequence_, 1U);
}

struct TerminalCompletionFixture {
    explicit TerminalCompletionFixture(const std::string & name,
                                       int ack_timeout_ms = 10000,
                                       const std::string & node_namespace = "",
                                       double object_arrival_threshold_m = 1.0,
                                       double object_target_filter_time_constant_s = 1.0,
                                       int maneuver_execution_period_ms = 10000)
    : node(name, node_namespace),
      config(makeConfiguration(10000, 1.0, ack_timeout_ms,
          object_arrival_threshold_m, object_target_filter_time_constant_s,
          maneuver_execution_period_ms)),
      awareness(std::make_shared<iii_drone::control::CombinedDroneAwarenessHandler>(
          config, std::make_shared<tf2_ros::Buffer>(node.get_clock()), &node)),
      scheduler(&node, awareness, config,
          node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)),
      hover(std::make_shared<iii_drone::control::maneuver::HoverManeuverServer>(
          &node, awareness, "hover", 1, 1, false)),
      hold(std::make_shared<iii_drone::control::maneuver::TerminalTrackingHold>(
          Reference(point_t(0.24F, 0.0F, 0.0F), 0.0), awareness,
          node.get_clock(),
          iii_drone::control::maneuver::TerminalTrackingHold::Clearance{}, 0.0)) {
        awareness->combined_drone_awareness_adapter_ = std::make_shared<
            iii_drone::utils::Atomic<iii_drone::adapters::CombinedDroneAwarenessAdapter>>();
        awareness->target_adapter_ = std::make_shared<
            iii_drone::utils::Atomic<iii_drone::adapters::TargetAdapter>>();
        scheduler.registered_maneuvers_[iii_drone::control::maneuver::MANEUVER_TYPE_HOVER] = hover;
        hover->AdoptTerminalHold(hold, kRequestA);
        iii_drone::control::MeasuredOdometrySnapshot measured;
        measured.receipt_stamp = node.now();
        measured.state = iii_drone::control::State(
            point_t(0.24F, 0.0F, 0.0F), vector_t::Zero(), 0.0,
            vector_t::Zero(), measured.receipt_stamp);
        measured.source_sample_timestamp_us = 1000000;
        awareness->measured_odometry_.Store(
            std::optional<iii_drone::control::MeasuredOdometrySnapshot>(measured));
    }

    ~TerminalCompletionFixture() {
        if (scheduler.is_started_) {
            scheduler.registered_maneuvers_.clear();
            scheduler.Stop();
        }
    }

    void observeNavigation(
        uint64_t source_us, uint64_t transition_us, uint8_t nav_state,
        std::chrono::steady_clock::time_point receipt =
            std::chrono::steady_clock::now()
    ) {
        px4_msgs::msg::VehicleStatus status;
        status.timestamp = source_us;
        status.nav_state_timestamp = transition_us;
        status.nav_state = nav_state;
        const bool external = nav_state >= status.NAVIGATION_STATE_EXTERNAL1 &&
            nav_state <= status.NAVIGATION_STATE_EXTERNAL8;
        awareness->vehicle_navigation_evidence_.Store(
            iii_drone::control::CombinedDroneAwarenessHandler::AdvanceVehicleNavigation(
                awareness->vehicle_navigation_evidence_.Load(), status, receipt, external));
    }

    void observeSourceExternal(uint64_t source_us = 1000000,
                               uint64_t transition_us = 500000) {
        observeNavigation(source_us, transition_us,
            px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_EXTERNAL5);
    }

    void observeNativeHold(uint64_t source_us = 2000000,
                           uint64_t transition_us = 2000000) {
        observeNavigation(source_us, transition_us,
            px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER);
    }

    void finishWithoutSchedulerTick(
        iii_drone::control::maneuver::maneuver_type_t type,
        const std::string & provider, bool succeeded, bool reacquire_token = true,
        const std::string & request_identity = kRequestA
    ) {
        if (type == iii_drone::control::maneuver::MANEUVER_TYPE_CABLE_AWARE_FLY_TO_POSITION) {
            source_server = std::make_shared<
                iii_drone::control::maneuver::CableAwareFlyToPositionManeuverServer>(
                    &node, awareness, provider, 1, 1, config, nullptr);
        } else {
            source_server = std::make_shared<
                iii_drone::control::maneuver::FollowWaypointPathManeuverServer>(
                    &node, awareness, provider, 1, 1, config);
        }
        scheduler.registered_maneuvers_[type] = source_server;
        scheduler.beginReferenceExecution(provider, request_identity, hold->lastCommand());
        const auto execution = scheduler.current_reference_execution_id_.Load();
        scheduler.reference_callback_struct_->set(
            [this](const iii_drone::control::State &) {
                return hold->lastCommand();
            }, provider, execution, request_identity);
        auto & stream_state = scheduler.reference_stream_state_;
        stream_state.valid = true;
        stream_state.stream_id = "terminal:completed";
        stream_state.execution_id = execution;
        stream_state.request_identity = request_identity;
        stream_state.sequence = 1;
        stream_state.recent_references.emplace_back(1, hold->lastCommand());
        auto ack = std::make_shared<Ack>();
        ack->stream_id = stream_state.stream_id;
        ack->last_applied_sequence = 1;
        ack->consumer_status = Ack::STATUS_APPLIED;
        scheduler.acknowledgeReferenceStream(ack);

        iii_drone::control::maneuver::Maneuver maneuver(type, rclcpp_action::GoalUUID{});
        maneuver.request_identity_ = request_identity;
        maneuver.started_ = true;
        maneuver.Terminate(succeeded);
        scheduler.current_maneuver_ = maneuver;
        // This is the actual token-return callback after action termination,
        // before the next 50 ms progressScheduler tick.
        if (reacquire_token) scheduler.onReferenceCallbackTokenReacquired();
    }

    std::shared_ptr<iii_drone_interfaces::srv::TerminalHoldTransfer::Response> query() {
        using Transfer = iii_drone_interfaces::srv::TerminalHoldTransfer;
        auto request = std::make_shared<Transfer::Request>();
        request->operation = Transfer::Request::OP_QUERY;
        auto response = std::make_shared<Transfer::Response>();
        scheduler.terminalHoldTransfer(request, response);
        return response;
    }

    rclcpp_lifecycle::LifecycleNode node;
    Configuration::SharedPtr config;
    iii_drone::control::CombinedDroneAwarenessHandler::SharedPtr awareness;
    iii_drone::control::maneuver::ManeuverScheduler scheduler;
    std::shared_ptr<iii_drone::control::maneuver::HoverManeuverServer> hover;
    std::shared_ptr<iii_drone::control::maneuver::TerminalTrackingHold> hold;
    std::shared_ptr<iii_drone::control::maneuver::ManeuverServer> source_server;
};

struct ClockReadSnapshotInterleave {
    ClockReadSnapshotInterleave(
        rcl_clock_t * handle,
        iii_drone::control::CombinedDroneAwarenessHandler::SharedPtr awareness,
        iii_drone::control::MeasuredOdometrySnapshot initial,
        VehicleOdometryAdapter adapter)
    : handle_(handle), original_get_now_(handle->get_now),
      original_data_(handle->data), awareness_(std::move(awareness)),
      initial_(std::move(initial)), adapter_(std::move(adapter)) {
        handle_->get_now = &ClockReadSnapshotInterleave::getNow;
        handle_->data = this;
    }

    ~ClockReadSnapshotInterleave() {
        handle_->get_now = original_get_now_;
        handle_->data = original_data_;
    }

    static rcl_ret_t getNow(void * data, rcl_time_point_value_t * now) {
        auto & self = *static_cast<ClockReadSnapshotInterleave *>(data);
        if (!self.fired_) {
            self.fired_ = true;
            // Inject the real measured-snapshot writer operation during the
            // first clock read. The caller's snapshot/clock order determines
            // whether this invocation consumes the old or new sample.
            self.awareness_->measured_odometry_.Store(
                iii_drone::control::CombinedDroneAwarenessHandler::
                    AdvanceMeasuredOdometry(
                        self.initial_, self.adapter_, 1'050'000,
                        rclcpp::Time(10'050'000'000LL, RCL_ROS_TIME)));
            *now = 10'000'000'000LL;
        } else {
            *now = 10'050'000'000LL;
        }
        return RCL_RET_OK;
    }

    rcl_clock_t * handle_;
    rcl_ret_t (* original_get_now_)(void *, rcl_time_point_value_t *);
    void * original_data_;
    iii_drone::control::CombinedDroneAwarenessHandler::SharedPtr awareness_;
    iii_drone::control::MeasuredOdometrySnapshot initial_;
    VehicleOdometryAdapter adapter_;
    bool fired_ = false;
};

struct ClockReadFreshIngressInterleave {
    ClockReadFreshIngressInterleave(
        rcl_clock_t * handle,
        iii_drone::control::CombinedDroneAwarenessHandler::SharedPtr awareness,
        px4_msgs::msg::VehicleOdometry next)
    : handle_(handle), original_get_now_(handle->get_now),
      original_data_(handle->data), awareness_(std::move(awareness)),
      next_(std::move(next)) {
        handle_->get_now = &ClockReadFreshIngressInterleave::getNow;
        handle_->data = this;
    }

    ~ClockReadFreshIngressInterleave() {
        handle_->get_now = original_get_now_;
        handle_->data = original_data_;
    }

    static rcl_ret_t getNow(void * data, rcl_time_point_value_t * now) {
        auto & self = *static_cast<ClockReadFreshIngressInterleave *>(data);
        if (!self.fired_) {
            self.fired_ = true;
            // The hold has captured the old snapshot. Publish a real newer
            // ingress sample before its emission-clock read completes.
            self.awareness_->ingestVehicleOdometry(self.next_,
                rclcpp::Time(10'300'000'000LL, RCL_ROS_TIME));
        }
        *now = 10'300'000'000LL;
        return RCL_RET_OK;
    }

    rcl_clock_t * handle_;
    rcl_ret_t (* original_get_now_)(void *, rcl_time_point_value_t *);
    void * original_data_;
    iii_drone::control::CombinedDroneAwarenessHandler::SharedPtr awareness_;
    px4_msgs::msg::VehicleOdometry next_;
    bool fired_ = false;
};

TEST(ManeuverReferenceClientTransaction, TerminalHoldAcceptsReceiptArrivingDuringClockRead) {
    RclcppContext context;
    TerminalCompletionFixture fixture("terminal_receipt_after_emission");
    const auto adapter = stationaryVehicleState();
    const auto initial = iii_drone::control::CombinedDroneAwarenessHandler::
        AdvanceMeasuredOdometry(std::nullopt, adapter, 1'000'000,
            rclcpp::Time(10'000'000'000LL, RCL_ROS_TIME));
    ASSERT_TRUE(initial);
    fixture.awareness->measured_odometry_.Store(initial);
    ClockReadSnapshotInterleave interleave(
        fixture.node.get_clock()->get_clock_handle(), fixture.awareness,
        *initial, adapter);
    const Reference command = fixture.hold->GetReference();
    EXPECT_TRUE(interleave.fired_);
    EXPECT_TRUE(command.position().allFinite());
    EXPECT_EQ(fixture.awareness->GetMeasuredOdometry()->source_sample_timestamp_us,
        1'050'000U);
    EXPECT_EQ(fixture.hold->phase(),
        iii_drone::control::maneuver::TerminalTrackingHold::Phase::Tracking)
        << fixture.hold->failureReason();
    EXPECT_TRUE(fixture.hold->failureReason().empty());
    const Reference next_command = fixture.hold->GetReference();
    EXPECT_TRUE(next_command.position().allFinite());
    EXPECT_EQ(fixture.hold->phase(),
        iii_drone::control::maneuver::TerminalTrackingHold::Phase::Tracking)
        << fixture.hold->failureReason();
}

TEST(ManeuverReferenceClientTransaction, TerminalHoldKeepsSmoothPositionThroughHeadingOnlyAggregateReset) {
    RclcppContext context;
    TerminalCompletionFixture fixture("terminal_heading_only_reset");
    auto * clock_handle = fixture.node.get_clock()->get_clock_handle();
    ASSERT_EQ(rcl_enable_ros_time_override(clock_handle), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(clock_handle, 10'000'000'000LL), RCL_RET_OK);

    fixture.awareness->vehicle_odometry_adapter_history_ = std::make_shared<
        iii_drone::utils::History<VehicleOdometryAdapter>>(2);
    fixture.awareness->measured_odometry_.Store(std::nullopt);

    px4_msgs::msg::VehicleLocalPosition local;
    local.xy_valid = local.z_valid = local.v_xy_valid = local.v_z_valid = true;
    local.xy_global = local.z_global = true;
    local.ref_timestamp = 900'000;
    local.ref_lat = 55.0;
    local.ref_lon = 10.0;
    local.ref_alt = 20.0F;
    local.xy_reset_counter = 4;
    local.z_reset_counter = 3;
    local.vxy_reset_counter = 3;
    local.vz_reset_counter = 2;
    local.heading_reset_counter = 3;
    local.timestamp_sample = 1'000'000;
    fixture.awareness->ingestVehicleLocalPosition(local, fixture.node.now());
    ASSERT_TRUE(fixture.awareness->latest_local_reset_);

    px4_msgs::msg::VehicleOdometry raw;
    raw.pose_frame = iii_drone::adapters::px4::POSE_FRAME_LOCAL_NED;
    raw.velocity_frame = iii_drone::adapters::px4::VELOCITY_FRAME_LOCAL_NED;
    raw.q[0] = 1.0F;
    raw.reset_counter = 15;
    raw.timestamp_sample = 1'000'000;
    ASSERT_TRUE(fixture.awareness->metadataMatches(
        *fixture.awareness->latest_local_reset_, raw, fixture.node.now()));
    fixture.awareness->ingestVehicleOdometry(raw, fixture.node.now());
    const auto initial = fixture.awareness->GetMeasuredOdometry();
    ASSERT_TRUE(initial);
    ASSERT_TRUE(initial->position_continuity.source_qualified);
    const Reference accepted = fixture.hold->GetReference();
    ASSERT_EQ(fixture.hold->phase(),
        iii_drone::control::maneuver::TerminalTrackingHold::Phase::Tracking)
        << fixture.hold->failureReason();

    // Start a real correction segment before the estimator heading reset.
    ASSERT_EQ(rcl_set_ros_time_override(clock_handle, 10'050'000'000LL), RCL_RET_OK);
    local.timestamp_sample = 1'050'000;
    fixture.awareness->ingestVehicleLocalPosition(local, fixture.node.now());
    raw.timestamp_sample = 1'050'000;
    fixture.awareness->ingestVehicleOdometry(raw, fixture.node.now());
    const Reference in_segment = fixture.hold->GetReference();
    ASSERT_GT(fixture.hold->controller_.integralTargetOffsetNorm(), 0.0);
    const auto before = fixture.awareness->GetMeasuredOdometry();
    ASSERT_TRUE(before);

    // A45's nearby PX4 evidence changed heading/quat reset 3->4 by
    // 0.001926 rad while position and velocity reset counters stayed fixed.
    // VehicleOdometry exposes only the aggregate 15->16 counter. Keep raw
    // position and velocity continuous and the same nominal command owner.
    constexpr double heading_change_rad = 0.00192606495693326;
    raw.q[0] = static_cast<float>(std::cos(heading_change_rad / 2.0));
    raw.q[3] = static_cast<float>(std::sin(heading_change_rad / 2.0));
    raw.reset_counter = 16;
    raw.position[0] = 0.02F;  // normal raw-world motion must not be compensated away
    raw.timestamp_sample = 1'100'000;
    local.timestamp_sample = 1'100'000;
    local.heading_reset_counter = 4;
    ASSERT_EQ(rcl_set_ros_time_override(clock_handle, 10'100'000'000LL), RCL_RET_OK);
    // Local-position metadata arrives before the aggregate odometry sample.
    fixture.awareness->ingestVehicleLocalPosition(local, fixture.node.now());
    fixture.awareness->ingestVehicleOdometry(raw, fixture.node.now());
    const auto after = fixture.awareness->GetMeasuredOdometry();
    ASSERT_TRUE(after);
    ASSERT_EQ(after->reset_counter, 16U);
    ASSERT_GT(after->source_sample_timestamp_us,
        before->source_sample_timestamp_us);
    ASSERT_NEAR(after->state.position()(0) - before->state.position()(0), 0.02, 1.0e-6);
    ASSERT_LT((after->state.velocity() - before->state.velocity()).norm(), 1.0e-6);
    ASSERT_TRUE(after->position_continuity.source_qualified);
    ASSERT_EQ(after->position_continuity.position_epoch,
        before->position_continuity.position_epoch);
    const Reference resumed = fixture.hold->GetReference();
    EXPECT_EQ(fixture.hold->phase(),
        iii_drone::control::maneuver::TerminalTrackingHold::Phase::Tracking)
        << fixture.hold->failureReason();
    EXPECT_TRUE(fixture.hold->failureReason().empty());
    EXPECT_TRUE(resumed.position().allFinite());
    EXPECT_LT((resumed.position() - in_segment.position()).norm(), 0.01);
    EXPECT_GT(fixture.hold->controller_.integralTargetOffsetNorm(), 0.0);
    EXPECT_LT((in_segment.position() - accepted.position()).norm(), 0.01);
    EXPECT_EQ(fixture.hold->nominalReference().position(),
        point_t(0.24F, 0.0F, 0.0F));
}

TEST(ManeuverReferenceClientTransaction, TerminalHoldPromotesSameSampleHeadingProofAtDuplicateEmission) {
    RclcppContext context;
    TerminalCompletionFixture fixture("terminal_same_sample_heading_proof");
    auto * clock_handle = fixture.node.get_clock()->get_clock_handle();
    ASSERT_EQ(rcl_enable_ros_time_override(clock_handle), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(clock_handle, 10'000'000'000LL), RCL_RET_OK);
    fixture.awareness->vehicle_odometry_adapter_history_ = std::make_shared<
        iii_drone::utils::History<VehicleOdometryAdapter>>(2);
    fixture.awareness->measured_odometry_.Store(std::nullopt);

    px4_msgs::msg::VehicleLocalPosition local;
    local.xy_valid = local.z_valid = local.v_xy_valid = local.v_z_valid = true;
    local.xy_global = local.z_global = true;
    local.ref_timestamp = 900'000;
    local.ref_lat = 55.0;
    local.ref_lon = 10.0;
    local.ref_alt = 20.0F;
    local.xy_reset_counter = 4;
    local.z_reset_counter = 3;
    local.vxy_reset_counter = 3;
    local.vz_reset_counter = 2;
    local.heading_reset_counter = 3;
    local.timestamp_sample = 1'000'000;
    px4_msgs::msg::VehicleOdometry raw;
    raw.pose_frame = iii_drone::adapters::px4::POSE_FRAME_LOCAL_NED;
    raw.velocity_frame = iii_drone::adapters::px4::VELOCITY_FRAME_LOCAL_NED;
    raw.q[0] = 1.0F;
    raw.reset_counter = 15;
    raw.timestamp_sample = 1'000'000;

    // The first command consumes odometry before its matching local metadata.
    fixture.awareness->ingestVehicleOdometry(raw, fixture.node.now());
    ASSERT_FALSE(fixture.awareness->GetMeasuredOdometry()->position_continuity.source_qualified);
    const Reference before = fixture.hold->GetReference();
    ASSERT_EQ(fixture.hold->phase(),
        iii_drone::control::maneuver::TerminalTrackingHold::Phase::Tracking);
    const double integral_before = fixture.hold->controller_.integralTargetOffsetNorm();

    // Qualification enriches the same source sample without a new receipt or
    // ROS emission tick; it must not integrate again or lose proof for reset16.
    fixture.awareness->ingestVehicleLocalPosition(local, fixture.node.now());
    ASSERT_TRUE(fixture.awareness->GetMeasuredOdometry()->position_continuity.source_qualified);
    const Reference duplicate = fixture.hold->GetReference();
    ASSERT_EQ(fixture.hold->phase(),
        iii_drone::control::maneuver::TerminalTrackingHold::Phase::Tracking)
        << fixture.hold->failureReason();
    EXPECT_LT((duplicate.position() - before.position()).norm(), 1.0e-7);
    EXPECT_DOUBLE_EQ(fixture.hold->controller_.integralTargetOffsetNorm(), integral_before);

    ASSERT_EQ(rcl_set_ros_time_override(clock_handle, 10'050'000'000LL), RCL_RET_OK);
    local.timestamp_sample = 1'050'000;
    local.heading_reset_counter = 4;
    fixture.awareness->ingestVehicleLocalPosition(local, fixture.node.now());
    raw.timestamp_sample = 1'050'000;
    raw.reset_counter = 16;
    fixture.awareness->ingestVehicleOdometry(raw, fixture.node.now());
    ASSERT_TRUE(fixture.awareness->GetMeasuredOdometry()->position_continuity.source_qualified);
    (void)fixture.hold->GetReference();
    EXPECT_EQ(fixture.hold->phase(),
        iii_drone::control::maneuver::TerminalTrackingHold::Phase::Tracking)
        << fixture.hold->failureReason();
}

TEST(ManeuverReferenceClientTransaction, TerminalStaleFailureRecordsCapturedAndLatestIngressOnce) {
    RclcppContext context;
    TerminalCompletionFixture fixture("terminal_stale_ingress_evidence");
    auto * clock_handle = fixture.node.get_clock()->get_clock_handle();
    ASSERT_EQ(rcl_enable_ros_time_override(clock_handle), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(clock_handle, 10'000'000'000LL), RCL_RET_OK);
    fixture.awareness->vehicle_odometry_adapter_history_ = std::make_shared<
        iii_drone::utils::History<VehicleOdometryAdapter>>(2);
    fixture.awareness->measured_odometry_.Store(std::nullopt);
    px4_msgs::msg::VehicleOdometry raw;
    raw.pose_frame = iii_drone::adapters::px4::POSE_FRAME_LOCAL_NED;
    raw.velocity_frame = iii_drone::adapters::px4::VELOCITY_FRAME_LOCAL_NED;
    raw.q[0] = 1.0F;
    raw.reset_counter = 15;
    raw.timestamp_sample = 1'000'000;
    fixture.awareness->ingestVehicleOdometry(raw, fixture.node.now());
    (void)fixture.hold->GetReference();
    ASSERT_EQ(fixture.hold->phase(),
        iii_drone::control::maneuver::TerminalTrackingHold::Phase::Tracking);
    EXPECT_TRUE(fixture.hold->last_freshness_diagnostic_.empty());

    ASSERT_EQ(rcl_set_ros_time_override(clock_handle, 10'010'000'000LL), RCL_RET_OK);
    fixture.awareness->ingestVehicleOdometry(raw, fixture.node.now()); // duplicate
    EXPECT_EQ(fixture.awareness->GetMeasuredOdometry()->receipt_stamp.nanoseconds(),
        10'000'000'000LL);
    ASSERT_EQ(rcl_set_ros_time_override(clock_handle, 10'020'000'000LL), RCL_RET_OK);
    raw.timestamp_sample = 1'020'000;
    raw.reset_counter = 16;
    fixture.awareness->ingestVehicleOdometry(raw, fixture.node.now()); // pending reset
    EXPECT_EQ(fixture.awareness->GetMeasuredOdometry()->receipt_stamp.nanoseconds(),
        10'000'000'000LL);
    const auto ingress = fixture.awareness->TryGetOdometryIngressDiagnostics();
    ASSERT_TRUE(ingress.available);
    ASSERT_EQ(ingress.history_count, 3U);
    EXPECT_TRUE(ingress.history[0].accepted);
    EXPECT_FALSE(ingress.history[1].accepted);
    EXPECT_FALSE(ingress.history[2].accepted);
    EXPECT_TRUE(ingress.history[2].pending_after);
    EXPECT_EQ(ingress.latest_source_sample_timestamp_us, 1'000'000U);
    EXPECT_EQ(ingress.latest_receipt_ros_ns, 10'000'000'000LL);
    EXPECT_LE(ingress.history[0].callback_entry_steady_ns,
        ingress.history[0].lock_acquired_steady_ns);
    EXPECT_LE(ingress.history[0].accepted_steady_ns,
        ingress.history[0].completed_steady_ns);
    std::promise<void> lock_entered;
    std::promise<void> release_lock;
    auto release_future = release_lock.get_future();
    std::thread lock_holder([&] {
        std::lock_guard<std::mutex> lock(fixture.awareness->odometry_ingest_mutex_);
        lock_entered.set_value();
        release_future.wait();
    });
    lock_entered.get_future().wait();
    const auto busy = fixture.awareness->TryGetOdometryIngressDiagnostics();
    release_lock.set_value();
    lock_holder.join();
    EXPECT_TRUE(busy.busy);
    EXPECT_FALSE(busy.available);

    ASSERT_EQ(rcl_set_ros_time_override(clock_handle, 10'300'000'000LL), RCL_RET_OK);
    (void)fixture.hold->GetReference();
    const std::string diagnostic = fixture.hold->last_freshness_diagnostic_;
    EXPECT_NE(diagnostic.find("captured_source_us=1000000"), std::string::npos);
    EXPECT_NE(diagnostic.find("captured_receipt_ros_ns=10000000000"), std::string::npos);
    EXPECT_NE(diagnostic.find("latest_source_us=1000000"), std::string::npos);
    EXPECT_NE(diagnostic.find("ingress_count=3"), std::string::npos);
    EXPECT_NE(fixture.hold->failureReason().find("stale or from the future"),
        std::string::npos);
    EXPECT_EQ(fixture.hold->failureReason().find("ingress_history"), std::string::npos);
    (void)fixture.hold->GetReference();
    EXPECT_EQ(fixture.hold->last_freshness_diagnostic_, diagnostic);
}

TEST(ManeuverReferenceClientTransaction, TerminalSampleIntervalFailureRecordsOriginalIngressOnce) {
    RclcppContext context;
    TerminalCompletionFixture fixture("terminal_interval_ingress_evidence");
    auto * clock_handle = fixture.node.get_clock()->get_clock_handle();
    ASSERT_EQ(rcl_enable_ros_time_override(clock_handle), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(clock_handle, 10'000'000'000LL), RCL_RET_OK);
    fixture.awareness->vehicle_odometry_adapter_history_ = std::make_shared<
        iii_drone::utils::History<VehicleOdometryAdapter>>(2);
    fixture.awareness->measured_odometry_.Store(std::nullopt);
    px4_msgs::msg::VehicleOdometry raw;
    raw.pose_frame = iii_drone::adapters::px4::POSE_FRAME_LOCAL_NED;
    raw.velocity_frame = iii_drone::adapters::px4::VELOCITY_FRAME_LOCAL_NED;
    raw.q[0] = 1.0F;
    raw.reset_counter = 15;
    raw.timestamp_sample = 1'000'000;
    fixture.awareness->ingestVehicleOdometry(raw, fixture.node.now());
    ASSERT_EQ(fixture.awareness->GetMeasuredOdometry()->receipt_stamp.nanoseconds(),
        10'000'000'000LL);
    (void)fixture.hold->GetReference();
    ASSERT_EQ(fixture.hold->phase(),
        iii_drone::control::maneuver::TerminalTrackingHold::Phase::Tracking);

    ASSERT_EQ(rcl_set_ros_time_override(clock_handle, 10'300'000'000LL), RCL_RET_OK);
    raw.timestamp_sample = 1'300'000;
    fixture.awareness->ingestVehicleOdometry(raw, fixture.node.now());
    const auto measured = fixture.awareness->GetMeasuredOdometry();
    ASSERT_TRUE(measured);
    ASSERT_EQ(measured->receipt_stamp.nanoseconds(), 10'300'000'000LL);
    (void)fixture.hold->GetReference();
    const auto reason = fixture.hold->failureReason();
    const auto diagnostic = fixture.hold->last_freshness_diagnostic_;
    EXPECT_EQ(reason.rfind("terminal tracking sample interval is discontinuous", 0), 0U);
    EXPECT_NE(reason.find("sample_dt_s=0.300000"), std::string::npos);
    EXPECT_NE(reason.find("previous_receipt_ns=10000000000"), std::string::npos);
    EXPECT_NE(reason.find("current_receipt_ns=10300000000"), std::string::npos);
    EXPECT_EQ(reason.find("ingress_history"), std::string::npos);
    EXPECT_NE(diagnostic.find("controller_reason=" + reason), std::string::npos);
    EXPECT_NE(diagnostic.find("captured_source_us=1300000"), std::string::npos);
    EXPECT_NE(diagnostic.find("captured_receipt_ros_ns=10300000000"), std::string::npos);
    EXPECT_NE(diagnostic.find("latest_source_us=1300000"), std::string::npos);
    EXPECT_NE(diagnostic.find("ingress_count=2"), std::string::npos);
    EXPECT_NE(diagnostic.find("ingress_history=["), std::string::npos);
    (void)fixture.hold->GetReference();
    EXPECT_EQ(fixture.hold->last_freshness_diagnostic_, diagnostic);
    EXPECT_EQ(fixture.hold->failureReason(), reason);
}

TEST(ManeuverReferenceClientTransaction, StatusSnapshotContentionDoesNotStarveOdometryIngress) {
    RclcppContext context;
    TerminalCompletionFixture fixture("status_snapshot_ingress_contention");
    fixture.awareness->Start();
    fixture.scheduler.Start();
    // The test supplies a barrier immediately before the extracted production
    // status callback. Both this timer and PX4 odometry use the node's real
    // default MutuallyExclusive group; the execution heartbeat has its own.
    fixture.scheduler.maneuver_publish_timer_->cancel();
    fixture.scheduler.current_maneuver_publisher_->on_activate();
    fixture.scheduler.maneuver_queue_publisher_->on_activate();
    auto publisher_node = std::make_shared<rclcpp::Node>("status_snapshot_px4_source");
    auto odometry_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().best_effort();
    auto odometry_publisher = publisher_node->create_publisher<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", odometry_qos);
    std::atomic<unsigned> status_messages{0};
    auto status_subscription = publisher_node->create_subscription<iii_drone_interfaces::msg::Maneuver>(
        "current_maneuver", 10,
        [&status_messages](iii_drone_interfaces::msg::Maneuver::ConstSharedPtr) {
            ++status_messages;
        });
    auto heartbeat_group = fixture.node.create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    std::atomic<unsigned> heartbeat_count{0};
    auto heartbeat = fixture.node.create_wall_timer(std::chrono::milliseconds(10),
        [&heartbeat_count] { ++heartbeat_count; }, heartbeat_group);
    std::promise<void> status_entered;
    auto status_entry = status_entered.get_future();
    std::atomic<bool> first_status{true};
    rclcpp::TimerBase::SharedPtr status_timer;
    std::unique_lock<std::shared_mutex> mutation_lock(
        fixture.scheduler.maneuver_mutex_, std::defer_lock);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
    executor.add_node(fixture.node.get_node_base_interface());
    executor.add_node(publisher_node);
    struct SpinGuard {
        rclcpp::executors::MultiThreadedExecutor & executor;
        std::unique_lock<std::shared_mutex> & mutation_lock;
        rclcpp::TimerBase::SharedPtr & status_timer;
        std::thread thread;
        void stop() {
            if (status_timer) status_timer->cancel();
            if (mutation_lock.owns_lock()) mutation_lock.unlock();
            executor.cancel();
            if (thread.joinable()) thread.join();
        }
        ~SpinGuard() { stop(); }
    } spin{executor, mutation_lock, status_timer,
        std::thread([&executor] { executor.spin(); })};
    const auto wait_for = [](const auto & ready) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
        while (!ready() && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        return ready();
    };
    ASSERT_TRUE(wait_for([&] { return odometry_publisher->get_subscription_count() > 0; }));
    px4_msgs::msg::VehicleOdometry raw;
    raw.pose_frame = iii_drone::adapters::px4::POSE_FRAME_LOCAL_NED;
    raw.velocity_frame = iii_drone::adapters::px4::VELOCITY_FRAME_LOCAL_NED;
    raw.q[0] = 1.0F;
    raw.reset_counter = 15;
    raw.timestamp_sample = 1'000'000;
    odometry_publisher->publish(raw);
    ASSERT_TRUE(wait_for([&] {
        const auto measured = fixture.awareness->GetMeasuredOdometry();
        return measured && measured->source_sample_timestamp_us == 1'000'000;
    }));

    mutation_lock.lock();
    status_timer = fixture.node.create_wall_timer(std::chrono::milliseconds(20), [&] {
        if (first_status.exchange(false)) status_entered.set_value();
        fixture.scheduler.publishManeuverStatus();
    });
    const bool callback_entered =
        status_entry.wait_for(std::chrono::seconds(1)) == std::future_status::ready;
    const unsigned heartbeat_before = heartbeat_count.load();
    raw.timestamp_sample = 1'050'000;
    odometry_publisher->publish(raw);
    const bool heartbeat_progressed = wait_for([&] {
        return heartbeat_count.load() >= heartbeat_before + 2;
    });
    const bool ingress_progressed_while_contended = wait_for([&] {
        const auto measured = fixture.awareness->GetMeasuredOdometry();
        return measured && measured->source_sample_timestamp_us == 1'050'000;
    });
    mutation_lock.unlock();
    const bool status_resumed = wait_for([&] { return status_messages.load() > 0; });
    const bool ingress_resumed = wait_for([&] {
        const auto measured = fixture.awareness->GetMeasuredOdometry();
        return measured && measured->source_sample_timestamp_us == 1'050'000;
    });
    spin.stop();

    EXPECT_TRUE(callback_entered) << "production status snapshot was not dispatched";
    EXPECT_TRUE(heartbeat_progressed) << "separate executor group did not progress";
    EXPECT_TRUE(ingress_progressed_while_contended)
        << "status snapshot blocked the default-group PX4 odometry callback";
    EXPECT_TRUE(status_resumed) << "status publication did not resume after mutation";
    EXPECT_TRUE(ingress_resumed) << "PX4 odometry did not resume after mutation";
}

TEST(ManeuverReferenceClientTransaction, TerminalStaleDiagnosticSeparatesCapturedFromNewerAcceptedSample) {
    RclcppContext context;
    TerminalCompletionFixture fixture("terminal_stale_interleaved_ingress");
    fixture.awareness->vehicle_odometry_adapter_history_ = std::make_shared<
        iii_drone::utils::History<VehicleOdometryAdapter>>(2);
    fixture.awareness->measured_odometry_.Store(std::nullopt);
    auto * clock_handle = fixture.node.get_clock()->get_clock_handle();
    ASSERT_EQ(rcl_enable_ros_time_override(clock_handle), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(clock_handle, 10'000'000'000LL), RCL_RET_OK);
    px4_msgs::msg::VehicleOdometry raw;
    raw.pose_frame = iii_drone::adapters::px4::POSE_FRAME_LOCAL_NED;
    raw.velocity_frame = iii_drone::adapters::px4::VELOCITY_FRAME_LOCAL_NED;
    raw.q[0] = 1.0F;
    raw.reset_counter = 15;
    raw.timestamp_sample = 1'000'000;
    fixture.awareness->ingestVehicleOdometry(raw, fixture.node.now());
    (void)fixture.hold->GetReference();
    ASSERT_EQ(fixture.hold->phase(),
        iii_drone::control::maneuver::TerminalTrackingHold::Phase::Tracking);

    raw.timestamp_sample = 1'300'000;
    ClockReadFreshIngressInterleave interleave(clock_handle, fixture.awareness, raw);
    (void)fixture.hold->GetReference();
    ASSERT_TRUE(interleave.fired_);
    EXPECT_NE(fixture.hold->failureReason().find("stale or from the future"),
        std::string::npos);
    const auto & diagnostic = fixture.hold->last_freshness_diagnostic_;
    EXPECT_NE(diagnostic.find("captured_source_us=1000000"), std::string::npos);
    EXPECT_NE(diagnostic.find("captured_receipt_ros_ns=10000000000"), std::string::npos);
    EXPECT_NE(diagnostic.find("latest_source_us=1300000"), std::string::npos);
    EXPECT_NE(diagnostic.find("latest_receipt_ros_ns=10300000000"), std::string::npos);
    const auto capture_marker = diagnostic.find("capture_finished_steady_ns=");
    const auto clock_marker = diagnostic.find("clock_finished_steady_ns=");
    ASSERT_NE(capture_marker, std::string::npos);
    ASSERT_NE(clock_marker, std::string::npos);
    const auto marker_value = [&diagnostic](size_t marker, const std::string & field) {
        return std::stoll(diagnostic.substr(marker + field.size()));
    };
    const auto captured_at = marker_value(capture_marker, "capture_finished_steady_ns=");
    const auto clock_finished_at = marker_value(clock_marker, "clock_finished_steady_ns=");
    const auto ingress = fixture.awareness->TryGetOdometryIngressDiagnostics();
    ASSERT_TRUE(ingress.available);
    EXPECT_LT(captured_at, ingress.latest_accepted_steady_ns);
    EXPECT_LE(ingress.latest_accepted_steady_ns, clock_finished_at);
    EXPECT_EQ(fixture.awareness->GetMeasuredOdometry()->source_sample_timestamp_us,
        1'300'000U);
}

TEST(ManeuverReferenceClientTransaction, OdometryIngressHistoryIsFixedAndKeepsLatestAcceptedReceipt) {
    RclcppContext context;
    TerminalCompletionFixture fixture("bounded_odometry_ingress_history");
    fixture.awareness->vehicle_odometry_adapter_history_ = std::make_shared<
        iii_drone::utils::History<VehicleOdometryAdapter>>(2);
    fixture.awareness->measured_odometry_.Store(std::nullopt);
    px4_msgs::msg::VehicleOdometry raw;
    raw.pose_frame = iii_drone::adapters::px4::POSE_FRAME_LOCAL_NED;
    raw.velocity_frame = iii_drone::adapters::px4::VELOCITY_FRAME_LOCAL_NED;
    raw.q[0] = 1.0F;
    raw.reset_counter = 15;
    for (uint64_t i = 0; i < 70; ++i) {
        raw.timestamp_sample = 1'000'000 + i * 10'000;
        fixture.awareness->ingestVehicleOdometry(raw,
            rclcpp::Time(10'000'000'000LL + static_cast<int64_t>(i) * 10'000'000LL,
                         RCL_ROS_TIME));
    }
    const auto ingress = fixture.awareness->TryGetOdometryIngressDiagnostics();
    ASSERT_TRUE(ingress.available);
    EXPECT_EQ(ingress.total_callbacks, 70U);
    ASSERT_EQ(ingress.history_count,
        iii_drone::control::OdometryIngressDiagnostics::history_capacity);
    EXPECT_EQ(ingress.history.front().source_sample_timestamp_us, 1'060'000U);
    EXPECT_EQ(ingress.history[ingress.history_count - 1].source_sample_timestamp_us,
        1'690'000U);
    EXPECT_EQ(ingress.latest_source_sample_timestamp_us, 1'690'000U);
    EXPECT_EQ(ingress.latest_receipt_ros_ns, 10'690'000'000LL);
}

TEST(ManeuverReferenceClientTransaction, TerminalHoldWaitsForReorderedHeadingMetadataWithoutRestamping) {
    RclcppContext context;
    TerminalCompletionFixture fixture("terminal_reordered_heading_reset");
    auto * clock_handle = fixture.node.get_clock()->get_clock_handle();
    ASSERT_EQ(rcl_enable_ros_time_override(clock_handle), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(clock_handle, 10'000'000'000LL), RCL_RET_OK);
    fixture.awareness->vehicle_odometry_adapter_history_ = std::make_shared<
        iii_drone::utils::History<VehicleOdometryAdapter>>(2);
    fixture.awareness->measured_odometry_.Store(std::nullopt);
    px4_msgs::msg::VehicleLocalPosition local;
    local.xy_valid = local.z_valid = local.v_xy_valid = local.v_z_valid = true;
    local.xy_global = local.z_global = true;
    local.ref_timestamp = 900'000;
    local.ref_lat = 55.0;
    local.ref_lon = 10.0;
    local.ref_alt = 20.0F;
    local.xy_reset_counter = 4;
    local.z_reset_counter = 3;
    local.vxy_reset_counter = 3;
    local.vz_reset_counter = 2;
    local.heading_reset_counter = 3;
    local.timestamp_sample = 1'000'000;
    fixture.awareness->ingestVehicleLocalPosition(local, fixture.node.now());
    px4_msgs::msg::VehicleOdometry raw;
    raw.pose_frame = iii_drone::adapters::px4::POSE_FRAME_LOCAL_NED;
    raw.velocity_frame = iii_drone::adapters::px4::VELOCITY_FRAME_LOCAL_NED;
    raw.q[0] = 1.0F;
    raw.reset_counter = 15;
    raw.timestamp_sample = 1'000'000;
    fixture.awareness->ingestVehicleOdometry(raw, fixture.node.now());
    const auto before = fixture.awareness->GetMeasuredOdometry();
    ASSERT_TRUE(before);
    ASSERT_TRUE(before->position_continuity.source_qualified);
    ASSERT_EQ(fixture.hold->GetReference().position().allFinite(), true);

    ASSERT_EQ(rcl_set_ros_time_override(clock_handle, 10'050'000'000LL), RCL_RET_OK);
    raw.timestamp_sample = 1'050'000;
    raw.reset_counter = 16;
    raw.position[0] = 0.02F;
    fixture.awareness->ingestVehicleOdometry(raw, fixture.node.now());
    const auto retained = fixture.awareness->GetMeasuredOdometry();
    ASSERT_TRUE(retained);
    EXPECT_EQ(retained->source_sample_timestamp_us, before->source_sample_timestamp_us);
    EXPECT_EQ(retained->receipt_stamp, before->receipt_stamp);
    ASSERT_EQ(rcl_set_ros_time_override(clock_handle, 10'080'000'000LL), RCL_RET_OK);
    local.timestamp_sample = 1'050'000;
    local.heading_reset_counter = 4;
    fixture.awareness->ingestVehicleLocalPosition(local, fixture.node.now());
    const auto after = fixture.awareness->GetMeasuredOdometry();
    ASSERT_TRUE(after);
    EXPECT_EQ(after->receipt_stamp.nanoseconds(), 10'050'000'000LL);
    EXPECT_NE(after->receipt_stamp, fixture.node.now());
    EXPECT_EQ(after->source_sample_timestamp_us, 1'050'000U);
    EXPECT_EQ(fixture.hold->GetReference().position().allFinite(), true);
    EXPECT_EQ(fixture.hold->phase(),
        iii_drone::control::maneuver::TerminalTrackingHold::Phase::Tracking)
        << fixture.hold->failureReason();
}

TEST(ManeuverReferenceClientTransaction, MissingHeadingProvenanceCannotQualifySmoothReset) {
    RclcppContext context;
    TerminalCompletionFixture fixture("terminal_missing_heading_provenance");
    auto * clock_handle = fixture.node.get_clock()->get_clock_handle();
    ASSERT_EQ(rcl_enable_ros_time_override(clock_handle), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(clock_handle, 10'000'000'000LL), RCL_RET_OK);
    fixture.awareness->vehicle_odometry_adapter_history_ = std::make_shared<
        iii_drone::utils::History<VehicleOdometryAdapter>>(2);
    fixture.awareness->measured_odometry_.Store(std::nullopt);
    px4_msgs::msg::VehicleOdometry raw;
    raw.pose_frame = iii_drone::adapters::px4::POSE_FRAME_LOCAL_NED;
    raw.velocity_frame = iii_drone::adapters::px4::VELOCITY_FRAME_LOCAL_NED;
    raw.q[0] = 1.0F;
    raw.reset_counter = 15;
    raw.timestamp_sample = 1'000'000;
    fixture.awareness->ingestVehicleOdometry(raw, fixture.node.now());
    ASSERT_TRUE(fixture.hold->GetReference().position().allFinite());
    ASSERT_EQ(rcl_set_ros_time_override(clock_handle, 10'050'000'000LL), RCL_RET_OK);
    raw.timestamp_sample = 1'050'000;
    raw.reset_counter = 16;
    fixture.awareness->ingestVehicleOdometry(raw, fixture.node.now());
    EXPECT_EQ(fixture.awareness->GetMeasuredOdometry()->reset_counter, 15U);
    ASSERT_EQ(rcl_set_ros_time_override(clock_handle, 10'300'000'000LL), RCL_RET_OK);
    fixture.hold->GetReference();
    EXPECT_NE(fixture.hold->phase(),
        iii_drone::control::maneuver::TerminalTrackingHold::Phase::Tracking);
}

TEST(ManeuverReferenceClientTransaction, PositionVelocityOriginAndSourceChangesCannotQualifyHeadingReset) {
    RclcppContext context;
    for (int variant = 0; variant < 6; ++variant) {
        TerminalCompletionFixture fixture("terminal_reset_negative_" + std::to_string(variant));
        auto * clock_handle = fixture.node.get_clock()->get_clock_handle();
        ASSERT_EQ(rcl_enable_ros_time_override(clock_handle), RCL_RET_OK);
        ASSERT_EQ(rcl_set_ros_time_override(clock_handle, 10'000'000'000LL), RCL_RET_OK);
        fixture.awareness->vehicle_odometry_adapter_history_ = std::make_shared<
            iii_drone::utils::History<VehicleOdometryAdapter>>(2);
        fixture.awareness->measured_odometry_.Store(std::nullopt);
        px4_msgs::msg::VehicleLocalPosition local;
        local.xy_valid = local.z_valid = local.v_xy_valid = local.v_z_valid = true;
        local.xy_global = local.z_global = true;
        local.ref_timestamp = 900'000;
        local.ref_lat = 55.0;
        local.ref_lon = 10.0;
        local.ref_alt = 20.0F;
        local.xy_reset_counter = 4;
        local.z_reset_counter = 3;
        local.vxy_reset_counter = 3;
        local.vz_reset_counter = 2;
        local.heading_reset_counter = 3;
        local.timestamp_sample = 1'000'000;
        fixture.awareness->ingestVehicleLocalPosition(local, fixture.node.now());
        px4_msgs::msg::VehicleOdometry raw;
        raw.pose_frame = iii_drone::adapters::px4::POSE_FRAME_LOCAL_NED;
        raw.velocity_frame = iii_drone::adapters::px4::VELOCITY_FRAME_LOCAL_NED;
        raw.q[0] = 1.0F;
        raw.reset_counter = 15;
        raw.timestamp_sample = 1'000'000;
        fixture.awareness->ingestVehicleOdometry(raw, fixture.node.now());
        ASSERT_TRUE(fixture.awareness->GetMeasuredOdometry());
        ASSERT_EQ(fixture.hold->phase(),
            iii_drone::control::maneuver::TerminalTrackingHold::Phase::Tracking);
        (void)fixture.hold->GetReference();
        ASSERT_EQ(rcl_set_ros_time_override(clock_handle, 10'050'000'000LL), RCL_RET_OK);
        raw.timestamp_sample = variant == 4 ? 900'000 : 1'050'000;
        raw.reset_counter = 16;
        local.timestamp_sample = variant == 3 ? 1'500'000 : 1'050'000;
        if (variant == 0) ++local.xy_reset_counter;
        if (variant == 1) ++local.vxy_reset_counter;
        if (variant == 2) {
            ++local.heading_reset_counter;
            ++local.ref_timestamp;
        }
        if (variant >= 3) ++local.heading_reset_counter;
        if (variant == 5)
            raw.pose_frame = iii_drone::adapters::px4::POSE_FRAME_LOCAL_FRD;
        fixture.awareness->ingestVehicleLocalPosition(local, fixture.node.now());
        fixture.awareness->ingestVehicleOdometry(raw, fixture.node.now());
        if (variant == 3) {
            // Missing source-matched metadata cannot refresh the old receipt.
            EXPECT_EQ(fixture.awareness->GetMeasuredOdometry()->reset_counter, 15U);
            ASSERT_EQ(rcl_set_ros_time_override(clock_handle, 10'300'000'000LL),
                RCL_RET_OK);
        }
        fixture.hold->GetReference();
        EXPECT_NE(fixture.hold->phase(),
            iii_drone::control::maneuver::TerminalTrackingHold::Phase::Tracking)
            << "variant=" << variant;
    }
}

TEST(ManeuverReferenceClientTransaction, HeadingCounterWrapAndDuplicateSamplesPreserveOnePositionEpoch) {
    RclcppContext context;
    TerminalCompletionFixture fixture("terminal_heading_counter_wrap");
    auto * clock_handle = fixture.node.get_clock()->get_clock_handle();
    ASSERT_EQ(rcl_enable_ros_time_override(clock_handle), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(clock_handle, 10'000'000'000LL), RCL_RET_OK);
    fixture.awareness->vehicle_odometry_adapter_history_ = std::make_shared<
        iii_drone::utils::History<VehicleOdometryAdapter>>(2);
    fixture.awareness->measured_odometry_.Store(std::nullopt);
    px4_msgs::msg::VehicleLocalPosition local;
    local.xy_valid = local.z_valid = local.v_xy_valid = local.v_z_valid = true;
    local.xy_global = local.z_global = true;
    local.ref_timestamp = 900'000;
    local.ref_lat = 55.0;
    local.ref_lon = 10.0;
    local.ref_alt = 20.0F;
    local.heading_reset_counter = 255;
    local.timestamp_sample = 1'000'000;
    fixture.awareness->ingestVehicleLocalPosition(local, fixture.node.now());
    px4_msgs::msg::VehicleOdometry raw;
    raw.pose_frame = iii_drone::adapters::px4::POSE_FRAME_LOCAL_NED;
    raw.velocity_frame = iii_drone::adapters::px4::VELOCITY_FRAME_LOCAL_NED;
    raw.q[0] = 1.0F;
    raw.reset_counter = 255;
    raw.timestamp_sample = 1'000'000;
    fixture.awareness->ingestVehicleOdometry(raw, fixture.node.now());
    const auto first = fixture.awareness->GetMeasuredOdometry();
    ASSERT_TRUE(first);
    (void)fixture.hold->GetReference();
    ASSERT_EQ(rcl_set_ros_time_override(clock_handle, 10'050'000'000LL), RCL_RET_OK);
    local.heading_reset_counter = 0;
    local.timestamp_sample = 1'050'000;
    fixture.awareness->ingestVehicleLocalPosition(local, fixture.node.now());
    raw.reset_counter = 0;
    raw.timestamp_sample = 1'050'000;
    fixture.awareness->ingestVehicleOdometry(raw, fixture.node.now());
    const auto wrapped = fixture.awareness->GetMeasuredOdometry();
    ASSERT_TRUE(wrapped);
    EXPECT_EQ(wrapped->position_continuity.position_epoch,
        first->position_continuity.position_epoch);
    EXPECT_TRUE(wrapped->position_continuity.source_qualified);
    (void)fixture.hold->GetReference();
    EXPECT_EQ(fixture.hold->phase(),
        iii_drone::control::maneuver::TerminalTrackingHold::Phase::Tracking)
        << fixture.hold->failureReason();
    fixture.awareness->ingestVehicleOdometry(raw,
        rclcpp::Time(10'200'000'000LL, RCL_ROS_TIME));
    EXPECT_EQ(fixture.awareness->GetMeasuredOdometry()->receipt_stamp,
        wrapped->receipt_stamp);
}

TEST(ManeuverReferenceClientTransaction, IsolatedOldOdometryStampPreservesQualifiedTerminalOwner) {
    RclcppContext context;
    TerminalCompletionFixture fixture("terminal_isolated_old_odometry_stamp");
    auto * clock = fixture.node.get_clock()->get_clock_handle();
    ASSERT_EQ(rcl_enable_ros_time_override(clock), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(clock, 10'000'000'000LL), RCL_RET_OK);
    fixture.awareness->vehicle_odometry_adapter_history_ = std::make_shared<
        iii_drone::utils::History<VehicleOdometryAdapter>>(2);
    fixture.awareness->measured_odometry_.Store(std::nullopt);

    constexpr uint64_t before_us = 1'790'578'918'314'347ULL;
    constexpr uint64_t old_us = 973'848'000ULL;
    constexpr uint64_t after_us = 1'790'578'918'563'959ULL;
    px4_msgs::msg::VehicleLocalPosition local;
    local.xy_valid = local.z_valid = local.v_xy_valid = local.v_z_valid = true;
    local.xy_global = local.z_global = true;
    local.ref_timestamp = 900'000;
    local.ref_lat = 55.0;
    local.ref_lon = 10.0;
    local.ref_alt = 20.0F;
    local.xy_reset_counter = 4;
    local.z_reset_counter = 3;
    local.vxy_reset_counter = 3;
    local.vz_reset_counter = 2;
    local.heading_reset_counter = 2;  // aggregate reset counter 14
    local.timestamp_sample = before_us;
    fixture.awareness->ingestVehicleLocalPosition(local, fixture.node.now());
    px4_msgs::msg::VehicleOdometry raw;
    raw.pose_frame = iii_drone::adapters::px4::POSE_FRAME_LOCAL_NED;
    raw.velocity_frame = iii_drone::adapters::px4::VELOCITY_FRAME_LOCAL_NED;
    raw.q[0] = 1.0F;
    raw.reset_counter = 14;
    raw.timestamp_sample = before_us;
    raw.timestamp = before_us;
    fixture.awareness->ingestVehicleOdometry(raw, fixture.node.now());
    const auto accepted = fixture.awareness->GetMeasuredOdometry();
    ASSERT_TRUE(accepted);
    ASSERT_TRUE(accepted->position_continuity.source_qualified);
    (void)fixture.hold->GetReference();
    ASSERT_EQ(fixture.hold->phase(),
        iii_drone::control::maneuver::TerminalTrackingHold::Phase::Tracking);

    // Exact A31 source-stamp excursion: an isolated same-counter old sample
    // must not replace the previously accepted sample or make a new epoch.
    ASSERT_EQ(rcl_set_ros_time_override(clock, 10'012'090'000LL), RCL_RET_OK);
    raw.timestamp_sample = old_us;
    raw.timestamp = old_us;
    fixture.awareness->ingestVehicleOdometry(raw, fixture.node.now());
    const auto after_outlier = fixture.awareness->GetMeasuredOdometry();
    ASSERT_TRUE(after_outlier);
    EXPECT_EQ(after_outlier->source_sample_timestamp_us,
        accepted->source_sample_timestamp_us);
    EXPECT_EQ(after_outlier->receipt_stamp, accepted->receipt_stamp);
    EXPECT_EQ(after_outlier->position_continuity.source_epoch,
        accepted->position_continuity.source_epoch);
    EXPECT_EQ(after_outlier->position_continuity.position_epoch,
        accepted->position_continuity.position_epoch);
    EXPECT_EQ((*fixture.awareness->vehicle_odometry_adapter_history_)[0]
                  .stamp().nanoseconds(), static_cast<int64_t>(before_us * 1000));

    ASSERT_EQ(rcl_set_ros_time_override(clock, 10'019'910'000LL), RCL_RET_OK);
    local.timestamp_sample = after_us;
    fixture.awareness->ingestVehicleLocalPosition(local, fixture.node.now());
    raw.timestamp_sample = after_us;
    raw.timestamp = after_us;
    fixture.awareness->ingestVehicleOdometry(raw, fixture.node.now());
    const auto successor = fixture.awareness->GetMeasuredOdometry();
    ASSERT_TRUE(successor);
    EXPECT_EQ(successor->source_sample_timestamp_us, after_us);
    EXPECT_EQ(successor->position_continuity.source_epoch,
        accepted->position_continuity.source_epoch);
    EXPECT_EQ(successor->position_continuity.position_epoch,
        accepted->position_continuity.position_epoch);
    (void)fixture.hold->GetReference();
    EXPECT_EQ(fixture.hold->phase(),
        iii_drone::control::maneuver::TerminalTrackingHold::Phase::Tracking)
        << fixture.hold->failureReason();
}

TEST(ManeuverReferenceClientTransaction, IsolatedOldLocalMetadataPreservesQualifiedTerminalOwner) {
    RclcppContext context;
    for (bool successor_odometry_first : {false, true}) {
        TerminalCompletionFixture fixture(successor_odometry_first
            ? "terminal_old_local_after_successor_odometry"
            : "terminal_old_local_before_successor_odometry");
        auto * clock = fixture.node.get_clock()->get_clock_handle();
        ASSERT_EQ(rcl_enable_ros_time_override(clock), RCL_RET_OK);
        ASSERT_EQ(rcl_set_ros_time_override(clock, 10'000'000'000LL), RCL_RET_OK);
        fixture.awareness->vehicle_odometry_adapter_history_ = std::make_shared<
            iii_drone::utils::History<VehicleOdometryAdapter>>(2);
        fixture.awareness->measured_odometry_.Store(std::nullopt);
        constexpr uint64_t before_us = 1'790'578'918'314'347ULL;
        constexpr uint64_t old_us = 973'848'000ULL;
        constexpr uint64_t after_us = 1'790'578'918'563'959ULL;
        px4_msgs::msg::VehicleLocalPosition local;
        local.xy_valid = local.z_valid = local.v_xy_valid = local.v_z_valid = true;
        local.xy_global = local.z_global = true;
        local.ref_timestamp = 900'000;
        local.ref_lat = 55.0;
        local.ref_lon = 10.0;
        local.ref_alt = 20.0F;
        local.xy_reset_counter = 4;
        local.z_reset_counter = 3;
        local.vxy_reset_counter = 3;
        local.vz_reset_counter = 2;
        local.heading_reset_counter = 2;
        local.timestamp_sample = before_us;
        fixture.awareness->ingestVehicleLocalPosition(local, fixture.node.now());
        px4_msgs::msg::VehicleOdometry raw;
        raw.pose_frame = iii_drone::adapters::px4::POSE_FRAME_LOCAL_NED;
        raw.velocity_frame = iii_drone::adapters::px4::VELOCITY_FRAME_LOCAL_NED;
        raw.q[0] = 1.0F;
        raw.reset_counter = 14;
        raw.timestamp_sample = before_us;
        raw.timestamp = before_us;
        fixture.awareness->ingestVehicleOdometry(raw, fixture.node.now());
        const auto accepted = fixture.awareness->GetMeasuredOdometry();
        ASSERT_TRUE(accepted);
        ASSERT_TRUE(accepted->position_continuity.source_qualified);
        (void)fixture.hold->GetReference();
        ASSERT_EQ(fixture.hold->phase(),
            iii_drone::control::maneuver::TerminalTrackingHold::Phase::Tracking);

        ASSERT_EQ(rcl_set_ros_time_override(clock, 10'012'090'000LL), RCL_RET_OK);
        if (successor_odometry_first) {
            raw.timestamp_sample = after_us;
            raw.timestamp = after_us;
            fixture.awareness->ingestVehicleOdometry(raw, fixture.node.now());
        }
        local.timestamp_sample = old_us;  // same counters and origin, wrong source clock
        fixture.awareness->ingestVehicleLocalPosition(local, fixture.node.now());
        const auto after_outlier = fixture.awareness->GetMeasuredOdometry();
        ASSERT_TRUE(after_outlier);
        EXPECT_EQ(after_outlier->position_continuity.source_epoch,
            accepted->position_continuity.source_epoch);
        EXPECT_EQ(after_outlier->position_continuity.position_epoch,
            accepted->position_continuity.position_epoch);
        if (!successor_odometry_first) {
            EXPECT_EQ(after_outlier->receipt_stamp, accepted->receipt_stamp);
        }

        ASSERT_EQ(rcl_set_ros_time_override(clock, 10'019'910'000LL), RCL_RET_OK);
        local.timestamp_sample = after_us;
        fixture.awareness->ingestVehicleLocalPosition(local, fixture.node.now());
        if (!successor_odometry_first) {
            raw.timestamp_sample = after_us;
            raw.timestamp = after_us;
            fixture.awareness->ingestVehicleOdometry(raw, fixture.node.now());
        }
        const auto successor = fixture.awareness->GetMeasuredOdometry();
        ASSERT_TRUE(successor);
        EXPECT_EQ(successor->source_sample_timestamp_us, after_us);
        EXPECT_EQ(successor->position_continuity.source_epoch,
            accepted->position_continuity.source_epoch);
        EXPECT_EQ(successor->position_continuity.position_epoch,
            accepted->position_continuity.position_epoch);
        (void)fixture.hold->GetReference();
        EXPECT_EQ(fixture.hold->phase(),
            iii_drone::control::maneuver::TerminalTrackingHold::Phase::Tracking)
            << fixture.hold->failureReason();
    }
}

TEST(ManeuverReferenceClientTransaction, SustainedOrChangedOldStampsStillFencePositionEpoch) {
    RclcppContext context;
    // 0: old odometry beyond the 250 ms bound, 1: old odometry with a changed
    // raw reset, 2: old local metadata with a changed basis, 3: old local
    // metadata beyond the 250 ms bound. None is an isolated stamp anomaly.
    for (int variant = 0; variant < 4; ++variant) {
        TerminalCompletionFixture fixture("terminal_old_stamp_fence_" + std::to_string(variant));
        auto * clock = fixture.node.get_clock()->get_clock_handle();
        ASSERT_EQ(rcl_enable_ros_time_override(clock), RCL_RET_OK);
        ASSERT_EQ(rcl_set_ros_time_override(clock, 10'000'000'000LL), RCL_RET_OK);
        fixture.awareness->vehicle_odometry_adapter_history_ = std::make_shared<
            iii_drone::utils::History<VehicleOdometryAdapter>>(2);
        fixture.awareness->measured_odometry_.Store(std::nullopt);
        constexpr uint64_t before_us = 1'790'578'918'314'347ULL;
        constexpr uint64_t old_us = 973'848'000ULL;
        px4_msgs::msg::VehicleLocalPosition local;
        local.xy_valid = local.z_valid = local.v_xy_valid = local.v_z_valid = true;
        local.xy_global = local.z_global = true;
        local.ref_timestamp = 900'000;
        local.ref_lat = 55.0;
        local.ref_lon = 10.0;
        local.ref_alt = 20.0F;
        local.xy_reset_counter = 4;
        local.z_reset_counter = 3;
        local.vxy_reset_counter = 3;
        local.vz_reset_counter = 2;
        local.heading_reset_counter = 2;
        local.timestamp_sample = before_us;
        fixture.awareness->ingestVehicleLocalPosition(local, fixture.node.now());
        px4_msgs::msg::VehicleOdometry raw;
        raw.pose_frame = iii_drone::adapters::px4::POSE_FRAME_LOCAL_NED;
        raw.velocity_frame = iii_drone::adapters::px4::VELOCITY_FRAME_LOCAL_NED;
        raw.q[0] = 1.0F;
        raw.reset_counter = 14;
        raw.timestamp_sample = before_us;
        raw.timestamp = before_us;
        fixture.awareness->ingestVehicleOdometry(raw, fixture.node.now());
        const auto accepted = fixture.awareness->GetMeasuredOdometry();
        ASSERT_TRUE(accepted);
        ASSERT_TRUE(accepted->position_continuity.source_qualified);

        const bool late = variant == 0 || variant == 3;
        ASSERT_EQ(rcl_set_ros_time_override(clock,
            late ? 10'300'000'000LL : 10'012'090'000LL), RCL_RET_OK);
        if (variant <= 1) {
            if (variant == 1) raw.reset_counter = 15;
            raw.timestamp_sample = old_us;
            raw.timestamp = old_us;
            fixture.awareness->ingestVehicleOdometry(raw, fixture.node.now());
        } else {
            if (variant == 2) ++local.xy_reset_counter;
            local.timestamp_sample = old_us;
            fixture.awareness->ingestVehicleLocalPosition(local, fixture.node.now());
        }
        const auto after = fixture.awareness->GetMeasuredOdometry();
        ASSERT_TRUE(after) << "variant=" << variant;
        EXPECT_NE(after->position_continuity.source_epoch,
            accepted->position_continuity.source_epoch) << "variant=" << variant;
        EXPECT_FALSE(after->position_continuity.source_qualified) << "variant=" << variant;
    }
}

TEST(ManeuverReferenceClientTransaction, ObjectIntegratorUsesOnlyQualifiedHeadingContinuity) {
    RclcppContext context;
    TerminalCompletionFixture fixture("object_heading_position_epoch");
    auto * clock_handle = fixture.node.get_clock()->get_clock_handle();
    ASSERT_EQ(rcl_enable_ros_time_override(clock_handle), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(clock_handle, 10'000'000'000LL), RCL_RET_OK);
    fixture.awareness->vehicle_odometry_adapter_history_ = std::make_shared<
        iii_drone::utils::History<VehicleOdometryAdapter>>(2);
    fixture.awareness->measured_odometry_.Store(std::nullopt);
    px4_msgs::msg::VehicleLocalPosition local;
    local.xy_valid = local.z_valid = local.v_xy_valid = local.v_z_valid = true;
    local.xy_global = local.z_global = true;
    local.ref_timestamp = 900'000;
    local.ref_lat = 55.0;
    local.ref_lon = 10.0;
    local.ref_alt = 20.0F;
    local.xy_reset_counter = 4;
    local.z_reset_counter = 3;
    local.vxy_reset_counter = 3;
    local.vz_reset_counter = 2;
    local.heading_reset_counter = 3;
    local.timestamp_sample = 1'000'000;
    px4_msgs::msg::VehicleOdometry raw;
    raw.pose_frame = iii_drone::adapters::px4::POSE_FRAME_LOCAL_NED;
    raw.velocity_frame = iii_drone::adapters::px4::VELOCITY_FRAME_LOCAL_NED;
    raw.q[0] = 1.0F;
    raw.position[2] = -1.0F;
    raw.reset_counter = 15;
    raw.timestamp_sample = 1'000'000;
    fixture.awareness->ingestVehicleOdometry(raw, fixture.node.now());
    ASSERT_FALSE(fixture.awareness->GetMeasuredOdometry()->position_continuity.source_qualified);
    const std::string owner = "mri1-object-qualified-heading-0000000000000001";
    const Reference seed(point_t(0.0F, 0.0F, 1.0F), 0.0);
    const Reference target(point_t(0.1F, 0.0F, 1.0F), 0.0);
    iii_drone::control::maneuver::ObjectTrackingSession session(
        [](const Reference &, const Reference & target, bool) { return target; },
        seed, owner, 1, fixture.node.now(), 0.0,
        iii_drone::control::maneuver::ObjectTrackingSession::Limits{});
    Reference output;
    std::string reason;
    ASSERT_TRUE(session.Compute(target, *fixture.awareness->GetMeasuredOdometry(),
        fixture.node.now(), owner, 1, 0.0, 0.4, output, reason)) << reason;
    const auto correction_before = session.correction();
    fixture.awareness->ingestVehicleLocalPosition(local, fixture.node.now());
    ASSERT_TRUE(fixture.awareness->GetMeasuredOdometry()->position_continuity.source_qualified);
    ASSERT_TRUE(session.Compute(target, *fixture.awareness->GetMeasuredOdometry(),
        fixture.node.now(), owner, 1, 0.0, 0.4, output, reason)) << reason;
    EXPECT_LT((session.correction() - correction_before).norm(), 1.0e-7);
    ASSERT_EQ(rcl_set_ros_time_override(clock_handle, 10'050'000'000LL), RCL_RET_OK);
    local.timestamp_sample = 1'050'000;
    local.heading_reset_counter = 4;
    fixture.awareness->ingestVehicleLocalPosition(local, fixture.node.now());
    raw.timestamp_sample = 1'050'000;
    raw.reset_counter = 16;
    fixture.awareness->ingestVehicleOdometry(raw, fixture.node.now());
    ASSERT_TRUE(session.Compute(target, *fixture.awareness->GetMeasuredOdometry(),
        fixture.node.now(), owner, 1, 0.0, 0.4, output, reason)) << reason;
    EXPECT_FALSE(session.failed());
    EXPECT_TRUE(session.owns(owner, 1));
    EXPECT_FALSE(session.Compute(target, *fixture.awareness->GetMeasuredOdometry(),
        fixture.node.now(), owner, 2, 0.0, 0.4, output, reason));
    EXPECT_FALSE(session.failed()); // wrong owner cannot mutate the live integrator
    ASSERT_EQ(rcl_set_ros_time_override(clock_handle, 10'100'000'000LL), RCL_RET_OK);
    local.timestamp_sample = 1'100'000;
    ++local.xy_reset_counter;
    fixture.awareness->ingestVehicleLocalPosition(local, fixture.node.now());
    raw.timestamp_sample = 1'100'000;
    raw.reset_counter = 17;
    fixture.awareness->ingestVehicleOdometry(raw, fixture.node.now());
    EXPECT_FALSE(session.Compute(target, *fixture.awareness->GetMeasuredOdometry(),
        fixture.node.now(), owner, 1, 0.0, 0.4, output, reason));
    EXPECT_TRUE(session.failed());
    EXPECT_TRUE(output.position().allFinite());
}

TEST(ManeuverReferenceClientTransaction, ReorderedResetDuplicateCannotRefreshPendingReceipt) {
    RclcppContext context;
    TerminalCompletionFixture fixture("terminal_pending_duplicate_receipt");
    auto * clock_handle = fixture.node.get_clock()->get_clock_handle();
    ASSERT_EQ(rcl_enable_ros_time_override(clock_handle), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(clock_handle, 10'000'000'000LL), RCL_RET_OK);
    fixture.awareness->vehicle_odometry_adapter_history_ = std::make_shared<
        iii_drone::utils::History<VehicleOdometryAdapter>>(2);
    fixture.awareness->measured_odometry_.Store(std::nullopt);
    px4_msgs::msg::VehicleLocalPosition local;
    local.xy_valid = local.z_valid = local.v_xy_valid = local.v_z_valid = true;
    local.xy_global = local.z_global = true;
    local.ref_timestamp = 900'000;
    local.ref_lat = 55.0;
    local.ref_lon = 10.0;
    local.ref_alt = 20.0F;
    local.xy_reset_counter = 4;
    local.z_reset_counter = 3;
    local.vxy_reset_counter = 3;
    local.vz_reset_counter = 2;
    local.heading_reset_counter = 3;
    local.timestamp_sample = 1'000'000;
    fixture.awareness->ingestVehicleLocalPosition(local, fixture.node.now());
    px4_msgs::msg::VehicleOdometry raw;
    raw.pose_frame = iii_drone::adapters::px4::POSE_FRAME_LOCAL_NED;
    raw.velocity_frame = iii_drone::adapters::px4::VELOCITY_FRAME_LOCAL_NED;
    raw.q[0] = 1.0F;
    raw.reset_counter = 15;
    raw.timestamp_sample = 1'000'000;
    fixture.awareness->ingestVehicleOdometry(raw, fixture.node.now());
    ASSERT_TRUE(fixture.awareness->GetMeasuredOdometry());
    ASSERT_EQ(rcl_set_ros_time_override(clock_handle, 10'050'000'000LL), RCL_RET_OK);
    raw.reset_counter = 16;
    raw.timestamp_sample = 1'050'000;
    fixture.awareness->ingestVehicleOdometry(raw, fixture.node.now());
    ASSERT_TRUE(fixture.awareness->pending_odometry_);
    const auto first_receipt = fixture.awareness->pending_odometry_->receipt;
    ASSERT_EQ(rcl_set_ros_time_override(clock_handle, 10'150'000'000LL), RCL_RET_OK);
    fixture.awareness->ingestVehicleOdometry(raw, fixture.node.now());
    raw.timestamp_sample = 1'040'000;
    fixture.awareness->ingestVehicleOdometry(raw, fixture.node.now());
    ASSERT_TRUE(fixture.awareness->pending_odometry_);
    EXPECT_EQ(fixture.awareness->pending_odometry_->receipt, first_receipt);
    EXPECT_EQ(fixture.awareness->pending_odometry_->message.timestamp_sample, 1'050'000U);
    local.heading_reset_counter = 4;
    local.timestamp_sample = 1'050'000;
    fixture.awareness->ingestVehicleLocalPosition(local, fixture.node.now());
    const auto accepted = fixture.awareness->GetMeasuredOdometry();
    ASSERT_TRUE(accepted);
    EXPECT_EQ(accepted->receipt_stamp, first_receipt);
    EXPECT_EQ(accepted->source_sample_timestamp_us, 1'050'000U);
    EXPECT_TRUE(accepted->position_continuity.source_qualified);
}

TEST(ManeuverReferenceClientTransaction, UnchangedAggregateWithChangedOriginFailsPositionContinuity) {
    RclcppContext context;
    TerminalCompletionFixture fixture("terminal_origin_without_aggregate");
    auto * clock_handle = fixture.node.get_clock()->get_clock_handle();
    ASSERT_EQ(rcl_enable_ros_time_override(clock_handle), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(clock_handle, 10'000'000'000LL), RCL_RET_OK);
    fixture.awareness->vehicle_odometry_adapter_history_ = std::make_shared<
        iii_drone::utils::History<VehicleOdometryAdapter>>(2);
    fixture.awareness->measured_odometry_.Store(std::nullopt);
    px4_msgs::msg::VehicleLocalPosition local;
    local.xy_valid = local.z_valid = local.v_xy_valid = local.v_z_valid = true;
    local.xy_global = local.z_global = true;
    local.ref_timestamp = 900'000;
    local.ref_lat = 55.0;
    local.ref_lon = 10.0;
    local.ref_alt = 20.0F;
    local.xy_reset_counter = 4;
    local.z_reset_counter = 3;
    local.vxy_reset_counter = 3;
    local.vz_reset_counter = 2;
    local.heading_reset_counter = 3;
    local.timestamp_sample = 1'000'000;
    fixture.awareness->ingestVehicleLocalPosition(local, fixture.node.now());
    px4_msgs::msg::VehicleOdometry raw;
    raw.pose_frame = iii_drone::adapters::px4::POSE_FRAME_LOCAL_NED;
    raw.velocity_frame = iii_drone::adapters::px4::VELOCITY_FRAME_LOCAL_NED;
    raw.q[0] = 1.0F;
    raw.reset_counter = 15;
    raw.timestamp_sample = 1'000'000;
    fixture.awareness->ingestVehicleOdometry(raw, fixture.node.now());
    const auto before = fixture.awareness->GetMeasuredOdometry();
    ASSERT_TRUE(before);
    (void)fixture.hold->GetReference();
    ASSERT_EQ(rcl_set_ros_time_override(clock_handle, 10'050'000'000LL), RCL_RET_OK);
    raw.timestamp_sample = 1'050'000; // odometry may arrive before metadata
    fixture.awareness->ingestVehicleOdometry(raw, fixture.node.now());
    local.timestamp_sample = 1'050'000;
    local.ref_lat += 0.001;
    fixture.awareness->ingestVehicleLocalPosition(local, fixture.node.now());
    const auto changed = fixture.awareness->GetMeasuredOdometry();
    ASSERT_TRUE(changed);
    EXPECT_EQ(changed->reset_counter, before->reset_counter);
    EXPECT_NE(changed->position_continuity.position_epoch,
        before->position_continuity.position_epoch);
    (void)fixture.hold->GetReference();
    EXPECT_NE(fixture.hold->phase(),
        iii_drone::control::maneuver::TerminalTrackingHold::Phase::Tracking);
}

TEST(ManeuverReferenceClientTransaction, OldHeadingMetadataCannotQualifyARecentOdometryReset) {
    RclcppContext context;
    TerminalCompletionFixture fixture("terminal_old_heading_metadata");
    auto * clock_handle = fixture.node.get_clock()->get_clock_handle();
    ASSERT_EQ(rcl_enable_ros_time_override(clock_handle), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(clock_handle, 10'000'000'000LL), RCL_RET_OK);
    fixture.awareness->vehicle_odometry_adapter_history_ = std::make_shared<
        iii_drone::utils::History<VehicleOdometryAdapter>>(2);
    fixture.awareness->measured_odometry_.Store(std::nullopt);
    px4_msgs::msg::VehicleLocalPosition local;
    local.xy_valid = local.z_valid = local.v_xy_valid = local.v_z_valid = true;
    local.xy_global = local.z_global = true;
    local.ref_timestamp = 900'000;
    local.ref_lat = 55.0;
    local.ref_lon = 10.0;
    local.ref_alt = 20.0F;
    local.xy_reset_counter = 4;
    local.z_reset_counter = 3;
    local.vxy_reset_counter = 3;
    local.vz_reset_counter = 2;
    local.heading_reset_counter = 3;
    local.timestamp_sample = 1'000'000;
    fixture.awareness->ingestVehicleLocalPosition(local, fixture.node.now());
    px4_msgs::msg::VehicleOdometry raw;
    raw.pose_frame = iii_drone::adapters::px4::POSE_FRAME_LOCAL_NED;
    raw.velocity_frame = iii_drone::adapters::px4::VELOCITY_FRAME_LOCAL_NED;
    raw.q[0] = 1.0F;
    raw.reset_counter = 15;
    raw.timestamp_sample = 1'000'000;
    fixture.awareness->ingestVehicleOdometry(raw, fixture.node.now());
    (void)fixture.hold->GetReference();
    ASSERT_EQ(rcl_set_ros_time_override(clock_handle, 10'250'000'000LL), RCL_RET_OK);
    raw.timestamp_sample = 1'250'000;
    fixture.awareness->ingestVehicleOdometry(raw, fixture.node.now());
    (void)fixture.hold->GetReference();
    ASSERT_EQ(fixture.hold->phase(),
        iii_drone::control::maneuver::TerminalTrackingHold::Phase::Tracking)
        << fixture.hold->failureReason();
    ASSERT_EQ(rcl_set_ros_time_override(clock_handle, 10'300'000'000LL), RCL_RET_OK);
    local.timestamp_sample = 1'300'000;
    local.heading_reset_counter = 4;
    fixture.awareness->ingestVehicleLocalPosition(local, fixture.node.now());
    raw.timestamp_sample = 1'300'000;
    raw.reset_counter = 16;
    fixture.awareness->ingestVehicleOdometry(raw, fixture.node.now());
    const auto classified = fixture.awareness->GetMeasuredOdometry();
    ASSERT_TRUE(classified);
    EXPECT_FALSE(classified->position_continuity.source_qualified);
    EXPECT_GT(classified->position_continuity.position_epoch, 0U);
    (void)fixture.hold->GetReference();
    EXPECT_NE(fixture.hold->phase(),
        iii_drone::control::maneuver::TerminalTrackingHold::Phase::Tracking);
}

TEST(ManeuverReferenceClientTransaction, LostOriginEvidenceFencesCurrentPositionWithoutRestamping) {
    RclcppContext context;
    TerminalCompletionFixture fixture("terminal_origin_evidence_lost");
    auto * clock_handle = fixture.node.get_clock()->get_clock_handle();
    ASSERT_EQ(rcl_enable_ros_time_override(clock_handle), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(clock_handle, 10'000'000'000LL), RCL_RET_OK);
    fixture.awareness->vehicle_odometry_adapter_history_ = std::make_shared<
        iii_drone::utils::History<VehicleOdometryAdapter>>(2);
    fixture.awareness->measured_odometry_.Store(std::nullopt);
    px4_msgs::msg::VehicleLocalPosition local;
    local.xy_valid = local.z_valid = local.v_xy_valid = local.v_z_valid = true;
    local.xy_global = local.z_global = true;
    local.ref_timestamp = 900'000;
    local.ref_lat = 55.0;
    local.ref_lon = 10.0;
    local.ref_alt = 20.0F;
    local.xy_reset_counter = 4;
    local.z_reset_counter = 3;
    local.vxy_reset_counter = 3;
    local.vz_reset_counter = 2;
    local.heading_reset_counter = 3;
    local.timestamp_sample = 1'000'000;
    fixture.awareness->ingestVehicleLocalPosition(local, fixture.node.now());
    px4_msgs::msg::VehicleOdometry raw;
    raw.pose_frame = iii_drone::adapters::px4::POSE_FRAME_LOCAL_NED;
    raw.velocity_frame = iii_drone::adapters::px4::VELOCITY_FRAME_LOCAL_NED;
    raw.q[0] = 1.0F;
    raw.reset_counter = 15;
    raw.timestamp_sample = 1'000'000;
    fixture.awareness->ingestVehicleOdometry(raw, fixture.node.now());
    const auto before = fixture.awareness->GetMeasuredOdometry();
    ASSERT_TRUE(before);
    (void)fixture.hold->GetReference();
    ASSERT_EQ(rcl_set_ros_time_override(clock_handle, 10'050'000'000LL), RCL_RET_OK);
    local.timestamp_sample = 1'050'000;
    local.xy_global = false;
    fixture.awareness->ingestVehicleLocalPosition(local, fixture.node.now());
    const auto fenced = fixture.awareness->GetMeasuredOdometry();
    ASSERT_TRUE(fenced);
    EXPECT_EQ(fenced->receipt_stamp, before->receipt_stamp);
    EXPECT_NE(fenced->position_continuity.source_epoch,
        before->position_continuity.source_epoch);
    (void)fixture.hold->GetReference();
    EXPECT_NE(fixture.hold->phase(),
        iii_drone::control::maneuver::TerminalTrackingHold::Phase::Tracking);
}

struct AcceptedObjectGoal {
    using Action = iii_drone_interfaces::action::FlyToObject;
    using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

    explicit AcceptedObjectGoal(const std::string & name)
    : server_node(std::make_shared<rclcpp::Node>(name + "_server")),
      client_node(std::make_shared<rclcpp::Node>(name + "_client")) {
        const std::string action_name = "/" + name + "/accepted_object";
        server = rclcpp_action::create_server<Action>(server_node, action_name,
            [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>) {
                return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
            },
            [](const std::shared_ptr<GoalHandle>) {
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](const std::shared_ptr<GoalHandle> accepted) { handle = accepted; });
        client = rclcpp_action::create_client<Action>(client_node, action_name);
        executor.add_node(server_node);
        executor.add_node(client_node);
    }

    ~AcceptedObjectGoal() {
        executor.remove_node(client_node);
        executor.remove_node(server_node);
    }

    std::shared_ptr<GoalHandle> accept(const std::string & request_identity,
                                       const iii_drone::adapters::TargetAdapter & target) {
        if (!client->wait_for_action_server(std::chrono::seconds(2))) return nullptr;
        Action::Goal goal;
        goal.request_identity = request_identity;
        goal.target = target.ToMsg();
        const auto future = client->async_send_goal(goal);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (std::chrono::steady_clock::now() < deadline &&
               (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready ||
                !handle)) {
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        return handle;
    }

    rclcpp::Node::SharedPtr server_node;
    rclcpp::Node::SharedPtr client_node;
    rclcpp_action::Server<Action>::SharedPtr server;
    rclcpp_action::Client<Action>::SharedPtr client;
    rclcpp::executors::SingleThreadedExecutor executor;
    std::shared_ptr<GoalHandle> handle;
};

struct AcceptedLandingGoal {
    using Action = iii_drone_interfaces::action::CableLanding;
    using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

    explicit AcceptedLandingGoal(const std::string & name)
    : server_node(std::make_shared<rclcpp::Node>(name + "_server")),
      client_node(std::make_shared<rclcpp::Node>(name + "_client")) {
        const std::string action_name = "/" + name + "/accepted_landing";
        server = rclcpp_action::create_server<Action>(server_node, action_name,
            [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>) {
                return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
            },
            [](const std::shared_ptr<GoalHandle>) {
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](const std::shared_ptr<GoalHandle> accepted) { handle = accepted; });
        client = rclcpp_action::create_client<Action>(client_node, action_name);
        executor.add_node(server_node);
        executor.add_node(client_node);
    }

    ~AcceptedLandingGoal() {
        executor.remove_node(client_node);
        executor.remove_node(server_node);
    }

    std::shared_ptr<GoalHandle> accept(const std::string & request_identity) {
        if (!client->wait_for_action_server(std::chrono::seconds(2))) return nullptr;
        Action::Goal goal;
        goal.request_identity = request_identity;
        goal.target_cable_id = 1;
        const auto future = client->async_send_goal(goal);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (std::chrono::steady_clock::now() < deadline &&
               (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready ||
                !handle)) {
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        return handle;
    }

    rclcpp::Node::SharedPtr server_node;
    rclcpp::Node::SharedPtr client_node;
    rclcpp_action::Server<Action>::SharedPtr server;
    rclcpp_action::Client<Action>::SharedPtr client;
    rclcpp::executors::SingleThreadedExecutor executor;
    std::shared_ptr<GoalHandle> handle;
};

struct AcceptedPositionGoal {
    using Action = iii_drone_interfaces::action::FlyToPosition;
    using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

    explicit AcceptedPositionGoal(const std::string & name)
    : server_node(std::make_shared<rclcpp::Node>(name + "_server")),
      client_node(std::make_shared<rclcpp::Node>(name + "_client")) {
        const std::string action_name = "/" + name + "/accepted_position";
        server = rclcpp_action::create_server<Action>(server_node, action_name,
            [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>) {
                return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
            },
            [](const std::shared_ptr<GoalHandle>) {
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](const std::shared_ptr<GoalHandle> accepted) { handle = accepted; });
        client = rclcpp_action::create_client<Action>(client_node, action_name);
        executor.add_node(server_node);
        executor.add_node(client_node);
    }

    ~AcceptedPositionGoal() {
        executor.remove_node(client_node);
        executor.remove_node(server_node);
    }

    std::shared_ptr<GoalHandle> accept(const std::string & request_identity) {
        if (!client->wait_for_action_server(std::chrono::seconds(2))) return nullptr;
        Action::Goal goal;
        goal.request_identity = request_identity;
        goal.frame_id = "world";
        goal.target_position.x = 1.0;
        goal.target_position.z = 1.5;
        const auto future = client->async_send_goal(goal);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (std::chrono::steady_clock::now() < deadline &&
               (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready ||
                !handle)) {
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        return handle;
    }

    rclcpp::Node::SharedPtr server_node;
    rclcpp::Node::SharedPtr client_node;
    rclcpp_action::Server<Action>::SharedPtr server;
    rclcpp_action::Client<Action>::SharedPtr client;
    rclcpp::executors::SingleThreadedExecutor executor;
    std::shared_ptr<GoalHandle> handle;
};

// Keep the real FlyToPosition success hook and action worker, while making the
// planner/arrival deterministic for this callback-lifetime boundary test.
class BlendedCompletionLeaseServer final
    : public iii_drone::control::maneuver::FlyToPositionManeuverServer {
public:
    using FlyToPositionManeuverServer::FlyToPositionManeuverServer;
    std::atomic<bool> allow_success{false};
    std::atomic<unsigned> compute_calls{0};
    std::function<void()> compute_hook;

private:
    void startExecution(iii_drone::control::maneuver::Maneuver &) override {
        active_blend_to_next_ = true;
        target_reference_ = Reference(point_t(0.4F, 0.0F, 1.0F), 0.0);
    }

    Reference initializationReference(const iii_drone::control::State &) const override {
        return Reference(point_t(0.4F, 0.0F, 1.0F), 0.0);
    }

    Reference computeReference(const iii_drone::control::State &) override {
        ++compute_calls;
        if (compute_hook) compute_hook();
        return Reference(point_t(0.4F, 0.0F, 1.0F), 0.0);
    }

    bool hasSucceeded(iii_drone::control::maneuver::Maneuver &) override {
        return allow_success.load();
    }

    bool hasFailed(iii_drone::control::maneuver::Maneuver &) override {
        return false;
    }

    std::shared_ptr<void> getFeedback(
        iii_drone::control::maneuver::Maneuver &) override {
        return std::make_shared<iii_drone_interfaces::action::FlyToPosition::Feedback>();
    }

    void publishResultAndFinalize(
        iii_drone::control::maneuver::Maneuver & maneuver,
        maneuver_result_type_t result_type) override {
        auto handle = std::static_pointer_cast<rclcpp_action::ServerGoalHandle<
            iii_drone_interfaces::action::FlyToPosition>>(maneuver.goal_handle());
        auto result = std::make_shared<iii_drone_interfaces::action::FlyToPosition::Result>();
        result->success = result_type == MANEUVER_RESULT_TYPE_SUCCEED;
        if (result->success) handle->succeed(result);
        else handle->abort(result);
    }
};

struct AcceptedHoverByObjectGoal {
    using Action = iii_drone_interfaces::action::HoverByObject;
    using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

    explicit AcceptedHoverByObjectGoal(const std::string & name)
    : server_node(std::make_shared<rclcpp::Node>(name + "_server")),
      client_node(std::make_shared<rclcpp::Node>(name + "_client")) {
        const std::string action_name = "/" + name + "/accepted_object_hover";
        server = rclcpp_action::create_server<Action>(server_node, action_name,
            [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>) {
                return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
            },
            [](const std::shared_ptr<GoalHandle>) {
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](const std::shared_ptr<GoalHandle> accepted) { handle = accepted; });
        client = rclcpp_action::create_client<Action>(client_node, action_name);
        executor.add_node(server_node);
        executor.add_node(client_node);
    }

    ~AcceptedHoverByObjectGoal() {
        executor.remove_node(client_node);
        executor.remove_node(server_node);
    }

    std::shared_ptr<GoalHandle> accept(const std::string & request_identity,
                                      const iii_drone::adapters::TargetAdapter & target) {
        if (!client->wait_for_action_server(std::chrono::seconds(2))) return nullptr;
        Action::Goal goal;
        goal.request_identity = request_identity;
        goal.target = target.ToMsg();
        goal.duration_s = 30.0F;
        goal.sustain_action = false;
        const auto future = client->async_send_goal(goal);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (std::chrono::steady_clock::now() < deadline &&
               (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready ||
                !handle)) {
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        return handle;
    }

    rclcpp::Node::SharedPtr server_node;
    rclcpp::Node::SharedPtr client_node;
    rclcpp_action::Server<Action>::SharedPtr server;
    rclcpp_action::Client<Action>::SharedPtr client;
    rclcpp::executors::SingleThreadedExecutor executor;
    std::shared_ptr<GoalHandle> handle;
};

TEST(ManeuverReferenceClientTransaction, NearTargetObjectCannotFinishBeforeItsGenerationIsApplied) {
    RclcppContext context;
    TerminalCompletionFixture fixture(
        "maneuver_controller", 10000, "/control/maneuver_controller");
    fixture.awareness->vehicle_status_adapter_history_ = std::make_shared<
        iii_drone::utils::History<iii_drone::adapters::px4::VehicleStatusAdapter>>(1);
    fixture.awareness->vehicle_odometry_adapter_history_ = std::make_shared<
        iii_drone::utils::History<VehicleOdometryAdapter>>(2);
    fixture.awareness->powerline_adapter_history_ = std::make_shared<
        iii_drone::utils::History<iii_drone::adapters::PowerlineAdapter>>(1);
    fixture.awareness->vehicle_status_adapter_history_->Store(
        iii_drone::adapters::px4::VehicleStatusAdapter(px4_msgs::msg::VehicleStatus{}));
    fixture.awareness->vehicle_odometry_adapter_history_->Store(stationaryVehicleState());
    fixture.awareness->powerline_adapter_history_->Store(iii_drone::adapters::PowerlineAdapter());
    iii_drone::adapters::CombinedDroneAwarenessAdapter awareness;
    awareness.armed() = true;
    awareness.offboard() = true;
    awareness.drone_location() = iii_drone::adapters::DRONE_LOCATION_IN_FLIGHT;
    *fixture.awareness->combined_drone_awareness_adapter_ = awareness;
    const auto target = iii_drone::adapters::TargetAdapter(
        iii_drone::adapters::TARGET_TYPE_CABLE, 1, "world",
        iii_drone::types::transform_matrix_t::Identity());
    fixture.awareness->target_adapter_->Store(target);
    fixture.finishWithoutSchedulerTick(
        iii_drone::control::maneuver::MANEUVER_TYPE_CABLE_AWARE_FLY_TO_POSITION,
        "cable_aware_fly_to_position", true);
    ClientFixture consumer("near_target_object_consumer");
    ASSERT_TRUE(consumer.client.BeginManeuverGoalHandoff(kRequestA));
    ASSERT_TRUE(consumer.client.ConfirmManeuverGoalHandoff(kRequestA));
    auto predecessor = stream(consumer.node, "ftp:terminal", 1,
        fixture.hold->lastCommand(), kRequestA);
    predecessor->terminal_hold_active = true;
    consumer.client.receiveReferenceStream(predecessor);
    const auto predecessor_command = consumer.client.GetReference(0.02, [] {});
    ASSERT_EQ(consumer.client.active_request_identity_, kRequestA);
    ASSERT_TRUE(consumer.client.BeginManeuverGoalHandoff(kRequestB));
    ASSERT_TRUE(consumer.client.ConfirmManeuverGoalHandoff(kRequestB));
    auto server = std::make_shared<iii_drone::control::maneuver::FlyToObjectManeuverServer>(
        &fixture.node, fixture.awareness, "fly_to_object_red", 1, 1, fixture.config, nullptr);
    server->target_adapter_ = target;
    server->active_target_reference_ = finiteReference(0.0, 0.0);
    server->active_target_reference_valid_ = true;
    server->maneuver_start_time_ = fixture.node.now();
    server->markTargetObserved();
    AcceptedObjectGoal accepted_goal("near_target_object");
    const auto handle = accepted_goal.accept(kRequestB, target);
    ASSERT_TRUE(handle);
    auto maneuver = iii_drone::control::maneuver::Maneuver::FromGoalHandle<
        AcceptedObjectGoal::Action>(handle);
    maneuver.Start();
    fixture.scheduler.Start();
    fixture.scheduler.maneuver_execution_timer_->cancel();
    auto hover_by_object = std::make_shared<
        iii_drone::control::maneuver::HoverByObjectManeuverServer>(
            &fixture.node, fixture.awareness, "hover_by_object_fallback", 1, 1,
            false, 0.25);
    fixture.scheduler.registered_maneuvers_[
        iii_drone::control::maneuver::MANEUVER_TYPE_HOVER_BY_OBJECT] = hover_by_object;
    fixture.scheduler.RegisterManeuverServer(
        iii_drone::control::maneuver::MANEUVER_TYPE_FLY_TO_OBJECT, server);
    fixture.scheduler.current_maneuver_ = maneuver;
    fixture.scheduler.beginReferenceExecution(
        server->action_name(), kRequestB, initializationHold());
    fixture.scheduler.publishReferenceStream();
    ASSERT_TRUE(fixture.scheduler.reference_stream_state_.valid);
    ASSERT_FALSE(fixture.scheduler.reference_stream_state_.ack_seen);
    EXPECT_FALSE(server->hasSucceeded(maneuver));
    auto wrong_ack = std::make_shared<Ack>();
    wrong_ack->stream_id = "ftp:predecessor";
    wrong_ack->last_applied_sequence = 1;
    wrong_ack->consumer_status = Ack::STATUS_APPLIED;
    fixture.scheduler.acknowledgeReferenceStream(wrong_ack);
    EXPECT_FALSE(fixture.scheduler.reference_stream_state_.ack_seen);
    wrong_ack->stream_id = fixture.scheduler.reference_stream_state_.stream_id;
    wrong_ack->last_applied_sequence = fixture.scheduler.reference_stream_state_.sequence + 1;
    fixture.scheduler.acknowledgeReferenceStream(wrong_ack);
    EXPECT_FALSE(fixture.scheduler.reference_stream_state_.ack_seen);
    wrong_ack->last_applied_sequence = 0;  // no matching published command
    fixture.scheduler.acknowledgeReferenceStream(wrong_ack);
    EXPECT_FALSE(fixture.scheduler.reference_stream_state_.ack_seen);
    EXPECT_FALSE(server->hasSucceeded(maneuver));

    ASSERT_TRUE(consumer.client.pending_goal_handoff_);
    EXPECT_EQ(consumer.client.active_request_identity_, kRequestA);
    EXPECT_LT((consumer.client.reference_.Load().position() -
        predecessor_command.position()).norm(), 1.0e-6);
    consumer.executor.add_node(fixture.node.get_node_base_interface());
    for (int attempt = 0; attempt < 50 &&
         fixture.scheduler.reference_stream_publisher_->get_subscription_count() == 0; ++attempt) {
        consumer.executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    fixture.scheduler.publishReferenceStream();
    for (int attempt = 0; attempt < 50; ++attempt) {
        consumer.executor.spin_some();
        bool received = false;
        {
            std::lock_guard<std::mutex> lock(consumer.client.reference_stream_mutex_);
            received = static_cast<bool>(consumer.client.latest_stream_message_);
        }
        if (received) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    const auto applied = consumer.client.GetReference(0.05, [] {});
    EXPECT_TRUE(applied.position().allFinite());
    EXPECT_FALSE(consumer.client.pending_goal_handoff_);
    EXPECT_EQ(consumer.client.active_request_identity_, kRequestB);
    const auto stream_id = fixture.scheduler.reference_stream_state_.stream_id;
    for (int attempt = 0; attempt < 50 &&
         !fixture.scheduler.reference_stream_state_.ack_seen; ++attempt) {
        consumer.executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    EXPECT_TRUE(fixture.scheduler.reference_stream_state_.ack_seen);
    EXPECT_TRUE(std::any_of(consumer.acknowledgements.begin(),
        consumer.acknowledgements.end(), [&stream_id](const Ack & ack) {
            return ack.stream_id == stream_id &&
                ack.consumer_status == Ack::STATUS_APPLIED &&
                ack.last_applied_sequence > 0;
        }));
    EXPECT_TRUE(server->hasSucceeded(maneuver));
    auto & stream_state = fixture.scheduler.reference_stream_state_;
    const auto fresh_ack = stream_state.last_ack;
    stream_state.last_ack -= std::chrono::seconds(11);
    EXPECT_FALSE(server->hasSucceeded(maneuver));
    stream_state.last_ack = fresh_ack;
    stream_state.last_ack_sequence = 0;
    EXPECT_FALSE(server->hasSucceeded(maneuver));
    stream_state.last_ack_sequence = consumer.client.last_applied_sequence_;
    stream_state.paused = true;
    EXPECT_FALSE(server->hasSucceeded(maneuver));
    stream_state.paused = false;
    stream_state.prepared = true;
    EXPECT_FALSE(server->hasSucceeded(maneuver));
    stream_state.prepared = false;
    stream_state.committed_waiting_for_applied = true;
    EXPECT_FALSE(server->hasSucceeded(maneuver));
    stream_state.committed_waiting_for_applied = false;
    stream_state.abort_waiting_for_consumer_ready = true;
    EXPECT_FALSE(server->hasSucceeded(maneuver));
    stream_state.abort_waiting_for_consumer_ready = false;
    stream_state.claim_ack_pending = true;
    EXPECT_FALSE(server->hasSucceeded(maneuver));
    stream_state.claim_ack_pending = false;
    stream_state.claimed_consumer_identity = "unapplied-successor";
    EXPECT_FALSE(server->hasSucceeded(maneuver));
    stream_state.claimed_consumer_identity.clear();
    EXPECT_FALSE(fixture.scheduler.firstObjectReferenceApplied(kRequestA));
    stream_state.last_consumer_status = Ack::STATUS_PAUSING;
    EXPECT_FALSE(server->hasSucceeded(maneuver));
    stream_state.last_consumer_status = Ack::STATUS_APPLIED;
    EXPECT_TRUE(server->hasSucceeded(maneuver));

    // The real completion callback tries HoverByObject first. The same
    // awareness gap seen in A30 makes it fall back to ordinary Hover, after
    // the successor has already taken control of this generation.
    std::promise<bool> callback_done;
    auto callback_result = callback_done.get_future();
    std::thread completion([&] {
        try {
            if (!server->reference_callback_token_->Acquire(2000)) {
                callback_done.set_value(false);
                return;
            }
            server->registerReferenceCallbackOnSuccess(maneuver);
            maneuver.Terminate(true);
            fixture.scheduler.onManeuverCompleted(maneuver);
            server->reference_callback_token_->Release();
            callback_done.set_value(true);
        } catch (...) {
            callback_done.set_value(false);
        }
    });
    for (int attempt = 0; attempt < 200 &&
         !fixture.scheduler.reference_callback_token_.has_requested_token(
             server->action_name()); ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    const bool requested = fixture.scheduler.reference_callback_token_.has_requested_token(
        server->action_name());
    if (requested) fixture.scheduler.reference_callback_token_.Give(server->action_name());
    const bool completed = callback_result.get();
    completion.join();
    ASSERT_TRUE(requested);
    ASSERT_TRUE(completed);
    EXPECT_FALSE(hover_by_object->has_target_);
    EXPECT_FALSE(fixture.hover->terminalHold());
    EXPECT_TRUE(fixture.scheduler.current_maneuver_.Load().terminated());
    EXPECT_TRUE(fixture.scheduler.current_maneuver_.Load().success());
    EXPECT_TRUE(consumer.client.CompleteManeuverGoalHandoff(
        kRequestB, finiteReference(0.0, 0.0).CopyWithNans()));
    EXPECT_FALSE(consumer.client.pending_goal_handoff_);
    EXPECT_TRUE(consumer.client.BeginManeuverGoalHandoff(kRequestC));
    consumer.executor.remove_node(fixture.node.get_node_base_interface());
    server->Stop();  // Release the registered-maneuver self-reference and token slave.
    fixture.scheduler.registered_maneuvers_.erase(
        iii_drone::control::maneuver::MANEUVER_TYPE_FLY_TO_OBJECT);
}

TEST(ManeuverReferenceClientTransaction, FilteredObjectTargetCannotCompleteBeforeRawHoverEligibility) {
    RclcppContext context;
    TerminalCompletionFixture fixture(
        "filtered_object_eligibility", 10000, "/control/maneuver_controller",
        0.1, 1.0, 50);
    fixture.awareness->vehicle_odometry_adapter_history_ = std::make_shared<
        iii_drone::utils::History<VehicleOdometryAdapter>>(2);
    fixture.awareness->vehicle_odometry_adapter_history_->Store(stationaryVehicleState());
    fixture.awareness->powerline_adapter_history_ = std::make_shared<
        iii_drone::utils::History<iii_drone::adapters::PowerlineAdapter>>(1);
    const auto stamp = fixture.node.now();
    const auto orientation = iii_drone::math::eulToQuat(
        iii_drone::types::euler_angles_t::Zero());
    const point_t line_position(0.6F, 0.0F, 1.5F);
    const iii_drone::adapters::SingleLineAdapter line(
        stamp, "world", 1, line_position, line_position, orientation, true);
    fixture.awareness->powerline_adapter_history_->Store(
        iii_drone::adapters::PowerlineAdapter(stamp, {line},
            iii_drone::types::createPlane(line_position, vector_t::UnitX())));
    geometry_msgs::msg::TransformStamped world_to_drone;
    world_to_drone.header.stamp = stamp;
    world_to_drone.header.frame_id = "world";
    world_to_drone.child_frame_id = "drone";
    world_to_drone.transform.rotation.w = 1.0;
    ASSERT_TRUE(fixture.awareness->tf_buffer()->setTransform(
        world_to_drone, "filtered_object_eligibility", true));
    iii_drone::types::transform_matrix_t target_transform =
        iii_drone::types::transform_matrix_t::Identity();
    target_transform(2, 3) = 1.5F;
    const iii_drone::adapters::TargetAdapter target(
        iii_drone::adapters::TARGET_TYPE_CABLE, 1, "world", target_transform);
    iii_drone::adapters::CombinedDroneAwarenessAdapter awareness;
    awareness.armed() = true;
    awareness.offboard() = true;
    awareness.drone_location() = iii_drone::adapters::DRONE_LOCATION_IN_FLIGHT;
    awareness.state() = fixture.awareness->GetState();
    *fixture.awareness->combined_drone_awareness_adapter_ = awareness;
    fixture.awareness->SetTarget(target);
    ASSERT_TRUE(fixture.awareness->adapter().armed());
    ASSERT_TRUE(fixture.awareness->adapter().offboard());
    ASSERT_TRUE(fixture.awareness->adapter().in_flight());
    ASSERT_TRUE(fixture.awareness->adapter().has_target());

    fixture.scheduler.Start();
    fixture.scheduler.maneuver_execution_timer_->cancel();
    auto hover_by_object = std::make_shared<
        iii_drone::control::maneuver::HoverByObjectManeuverServer>(
            &fixture.node, fixture.awareness, "hover_by_object", 1, 1, false, 0.25);
    fixture.scheduler.RegisterManeuverServer(
        iii_drone::control::maneuver::MANEUVER_TYPE_HOVER_BY_OBJECT,
        hover_by_object);
    auto object = std::make_shared<
        iii_drone::control::maneuver::FlyToObjectManeuverServer>(
            &fixture.node, fixture.awareness, "fly_to_object", 1, 1,
            fixture.config, nullptr);
    fixture.scheduler.RegisterManeuverServer(
        iii_drone::control::maneuver::MANEUVER_TYPE_FLY_TO_OBJECT, object);
    AcceptedObjectGoal accepted("filtered_object_eligibility");
    const auto handle = accepted.accept(kRequestB, target);
    ASSERT_TRUE(handle);
    auto maneuver = iii_drone::control::maneuver::Maneuver::FromGoalHandle<
        AcceptedObjectGoal::Action>(handle);
    maneuver.Start();
    fixture.scheduler.current_maneuver_ = maneuver;
    fixture.scheduler.beginReferenceExecution(
        object->action_name(), kRequestB, fixture.hold->lastCommand());
    const auto execution = fixture.scheduler.current_reference_execution_id_.Load();
    auto interpolator = std::make_shared<iii_drone::control::TrajectoryInterpolator>(
        fixture.config, nullptr);
    auto tracking = std::make_shared<iii_drone::control::maneuver::ObjectTrackingSession>(
        [interpolator](const Reference & start, const Reference & destination, bool reset) {
            return interpolator->ComputeBoundedPositionalTrajectory(
                start, destination, true, reset).references().front();
        }, fixture.hold->lastCommand(), kRequestB, execution, fixture.node.now(), 0.0,
        iii_drone::control::maneuver::ObjectTrackingSession::Limits{});
    object->target_adapter_ = target;
    object->object_tracking_session_ = tracking;
    object->published_object_tracking_session_.Store(tracking);
    object->maneuver_start_time_ = fixture.node.now();
    object->markTargetObserved();
    fixture.scheduler.reference_callback_struct_->set(
        [tracking](const iii_drone::control::State &) {
            return tracking->lastCommand();
        }, object->action_name(), execution, kRequestB);

    // Exercise the same target lookup, filter and coherent observation update
    // as computeReference, without a trajectory-generator service dependency.
    const auto state = fixture.awareness->GetState();
    const auto raw = object->getUpdatedTargetReference(state);
    const auto filtered = object->updateLiveTargetReference(
        state, fixture.scheduler.reference_callback_struct_->snapshot());
    const auto observation = object->nominal_target_observation_.Load();
    ASSERT_TRUE(observation);
    EXPECT_EQ(observation->request_identity, kRequestB);
    EXPECT_EQ(observation->execution_id, execution);
    EXPECT_LT((observation->nominal.position() - raw.position()).norm(), 1.0e-6);
    EXPECT_LT((observation->filtered.position() - filtered.position()).norm(), 1.0e-6);
    const auto raw_target_pose = iii_drone::types::poseFromTransformMatrix(
        fixture.awareness->ComputeTargetTransform(target));
    const double raw_distance = (raw.position() - state.position()).norm();
    const double filtered_distance = (filtered.position() - state.position()).norm();
    const double hover_distance =
        (raw_target_pose.position - fixture.awareness->adapter().state().position()).norm();
    RecordProperty("measured_x_m", state.position()(0));
    RecordProperty("measured_y_m", state.position()(1));
    RecordProperty("measured_z_m", state.position()(2));
    RecordProperty("raw_x_m", raw.position()(0));
    RecordProperty("raw_y_m", raw.position()(1));
    RecordProperty("raw_z_m", raw.position()(2));
    RecordProperty("filtered_x_m", filtered.position()(0));
    RecordProperty("filtered_y_m", filtered.position()(1));
    RecordProperty("filtered_z_m", filtered.position()(2));
    RecordProperty("raw_distance_m", raw_distance);
    RecordProperty("filtered_distance_m", filtered_distance);
    RecordProperty("hover_geometry_distance_m", hover_distance);
    RecordProperty("offboard", fixture.awareness->adapter().offboard());
    RecordProperty("armed", fixture.awareness->adapter().armed());
    RecordProperty("in_flight", fixture.awareness->adapter().in_flight());
    RecordProperty("has_target", fixture.awareness->adapter().has_target());
    EXPECT_LT((raw_target_pose.position - raw.position()).norm(), 1.0e-6);
    EXPECT_GT(raw_distance, 0.25);
    EXPECT_GT(raw_distance, 0.1);
    EXPECT_LT(filtered_distance, 0.1);
    EXPECT_GT(hover_distance, 0.25);
    EXPECT_FALSE(hover_by_object->validateTargetTransform(
        fixture.awareness->ComputeTargetTransform(target),
        fixture.awareness->adapter().state()));
    EXPECT_FALSE(hover_by_object->validateAwareness(
        fixture.awareness->adapter(), target));

    ClientFixture consumer("filtered_object_eligibility_consumer");
    ASSERT_TRUE(consumer.client.BeginManeuverGoalHandoff(kRequestB));
    ASSERT_TRUE(consumer.client.ConfirmManeuverGoalHandoff(kRequestB));
    consumer.executor.add_node(fixture.node.get_node_base_interface());
    for (int attempt = 0; attempt < 50 &&
         fixture.scheduler.reference_stream_publisher_->get_subscription_count() == 0;
         ++attempt) {
        consumer.executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    ASSERT_GT(fixture.scheduler.reference_stream_publisher_->get_subscription_count(), 0U);
    fixture.scheduler.publishReferenceStream();
    for (int attempt = 0; attempt < 50; ++attempt) {
        consumer.executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    (void)consumer.client.GetReference(0.02, [] {});
    for (int attempt = 0; attempt < 50 &&
         !fixture.scheduler.reference_stream_state_.ack_seen; ++attempt) {
        consumer.executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    ASSERT_TRUE(fixture.scheduler.firstObjectReferenceApplied(kRequestB));
    ASSERT_TRUE(consumer.client.currentAppliedObjectTrackingStream(kRequestB));
    EXPECT_LT((object->active_target_reference_.Load().position() -
        filtered.position()).norm(), 1.0e-6)
        << "the control target must remain filtered";
    struct EligibilityResults {
        bool far_succeeded = false;
        bool missing_succeeded = false;
        bool stale_succeeded = false;
        bool future_succeeded = false;
        bool wrong_request_succeeded = false;
        bool wrong_execution_succeeded = false;
        bool grace_lookup_rejected = false;
        bool grace_snapshot_cleared = false;
        bool grace_succeeded = false;
        bool hover_installed_before_arrival = false;
        bool near_succeeded = false;
        double near_nominal_distance = -1.0;
    };
    std::promise<EligibilityResults> result_promise;
    auto result_future = result_promise.get_future();
    std::thread completion([&] {
        bool acquired = false;
        try {
            acquired = object->reference_callback_token_->Acquire(2000);
            if (!acquired) throw std::runtime_error("object callback token unavailable");
            EligibilityResults results;
            results.far_succeeded = object->hasSucceeded(maneuver);

            // The same production perception history now reports an already
            // reached raw target. The current marked ACK remains applicable.
            const point_t near_line_position(0.0F, 0.0F, 1.5F);
            const iii_drone::adapters::SingleLineAdapter near_line(
                stamp, "world", 1, near_line_position, near_line_position,
                orientation, true);
            fixture.awareness->powerline_adapter_history_->Store(
                iii_drone::adapters::PowerlineAdapter(stamp, {near_line},
                    iii_drone::types::createPlane(
                        near_line_position, vector_t::UnitX())));
            const auto binding = fixture.scheduler.reference_callback_struct_->snapshot();
            (void)object->updateLiveTargetReference(state, binding);
            const auto near = object->nominal_target_observation_.Load();
            if (!near) throw std::runtime_error("near nominal observation missing");
            results.near_nominal_distance =
                (near->nominal.position() - state.position()).norm();

            object->nominal_target_observation_.Store(nullptr);
            results.missing_succeeded = object->hasSucceeded(maneuver);
            using Observation = iii_drone::control::maneuver::
                FlyToObjectManeuverServer::NominalTargetObservation;
            auto stale = std::make_shared<Observation>(*near);
            stale->observed_at = fixture.node.now() -
                rclcpp::Duration::from_seconds(11.0);
            stale->received_at -= std::chrono::seconds(11);
            object->nominal_target_observation_.Store(stale);
            results.stale_succeeded = object->hasSucceeded(maneuver);
            auto future = std::make_shared<Observation>(*near);
            future->observed_at = fixture.node.now() +
                rclcpp::Duration::from_seconds(1.0);
            object->nominal_target_observation_.Store(future);
            results.future_succeeded = object->hasSucceeded(maneuver);
            auto wrong_request = std::make_shared<Observation>(*near);
            wrong_request->request_identity = kRequestC;
            object->nominal_target_observation_.Store(wrong_request);
            results.wrong_request_succeeded = object->hasSucceeded(maneuver);
            auto wrong_execution = std::make_shared<Observation>(*near);
            wrong_execution->execution_id = execution + 1;
            object->nominal_target_observation_.Store(wrong_execution);
            results.wrong_execution_succeeded = object->hasSucceeded(maneuver);

            fixture.awareness->powerline_adapter_history_->Store(
                iii_drone::adapters::PowerlineAdapter());
            try {
                (void)object->updateLiveTargetReference(state, binding);
            } catch (const std::runtime_error &) {
                results.grace_lookup_rejected = true;
            }
            results.grace_snapshot_cleared =
                !object->nominal_target_observation_.Load();
            results.grace_succeeded = object->hasSucceeded(maneuver);
            results.hover_installed_before_arrival = hover_by_object->has_target_;

            fixture.awareness->powerline_adapter_history_->Store(
                iii_drone::adapters::PowerlineAdapter(stamp, {near_line},
                    iii_drone::types::createPlane(
                        near_line_position, vector_t::UnitX())));
            (void)object->updateLiveTargetReference(state, binding);
            results.near_succeeded = object->hasSucceeded(maneuver);
            object->reference_callback_token_->Release();
            acquired = false;
            result_promise.set_value(results);
        } catch (...) {
            if (acquired) object->reference_callback_token_->Release();
            result_promise.set_exception(std::current_exception());
        }
    });
    for (int attempt = 0; attempt < 200 &&
         !fixture.scheduler.reference_callback_token_.has_requested_token(
             object->action_name()); ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    const bool requested = fixture.scheduler.reference_callback_token_.has_requested_token(
        object->action_name());
    if (requested) fixture.scheduler.reference_callback_token_.Give(object->action_name());
    completion.join();
    const auto results = result_future.get();
    ASSERT_TRUE(requested);
    RecordProperty("near_nominal_distance_m", results.near_nominal_distance);
    EXPECT_LT(results.near_nominal_distance, 0.1);
    EXPECT_FALSE(results.far_succeeded);
    EXPECT_FALSE(results.missing_succeeded);
    EXPECT_FALSE(results.stale_succeeded);
    EXPECT_FALSE(results.future_succeeded);
    EXPECT_FALSE(results.wrong_request_succeeded);
    EXPECT_FALSE(results.wrong_execution_succeeded);
    EXPECT_TRUE(results.grace_lookup_rejected);
    EXPECT_TRUE(results.grace_snapshot_cleared);
    EXPECT_FALSE(results.grace_succeeded);
    EXPECT_FALSE(results.hover_installed_before_arrival);
    EXPECT_TRUE(results.near_succeeded);
    EXPECT_TRUE(hover_by_object->has_target_);
    EXPECT_FALSE(tracking->failed()) << tracking->failureReason();
    consumer.executor.remove_node(fixture.node.get_node_base_interface());
    object->Stop();
    hover_by_object->Stop();
    fixture.scheduler.registered_maneuvers_.erase(
        iii_drone::control::maneuver::MANEUVER_TYPE_FLY_TO_OBJECT);
    fixture.scheduler.registered_maneuvers_.erase(
        iii_drone::control::maneuver::MANEUVER_TYPE_HOVER_BY_OBJECT);
}

TEST(ManeuverReferenceClientTransaction, TrackedObjectSuccessKeepsFallbackForExplicitHover) {
    RclcppContext context;
    TerminalCompletionFixture fixture(
        "tracked_object_fallback", 1000, "/control/maneuver_controller");
    fixture.awareness->vehicle_odometry_adapter_history_ = std::make_shared<
        iii_drone::utils::History<VehicleOdometryAdapter>>(2);
    fixture.awareness->vehicle_odometry_adapter_history_->Store(stationaryVehicleState());
    fixture.awareness->powerline_adapter_history_ = std::make_shared<
        iii_drone::utils::History<iii_drone::adapters::PowerlineAdapter>>(1);
    const auto line_stamp = fixture.node.now();
    const auto line_orientation = iii_drone::math::eulToQuat(
        iii_drone::types::euler_angles_t::Zero());
    const point_t line_position(0.0F, 0.0F, 1.5F);
    const iii_drone::adapters::SingleLineAdapter line(
        line_stamp, "world", 1, line_position, line_position, line_orientation, true);
    fixture.awareness->powerline_adapter_history_->Store(
        iii_drone::adapters::PowerlineAdapter(line_stamp, {line},
            iii_drone::types::createPlane(line_position, vector_t::UnitX())));
    geometry_msgs::msg::TransformStamped world_to_drone;
    world_to_drone.header.stamp = fixture.node.now();
    world_to_drone.header.frame_id = "world";
    world_to_drone.child_frame_id = "drone";
    world_to_drone.transform.rotation.w = 1.0;
    ASSERT_TRUE(fixture.awareness->tf_buffer()->setTransform(
        world_to_drone, "object_handoff_test", true));
    auto world_to_gripper = world_to_drone;
    world_to_gripper.child_frame_id = "gripper";
    ASSERT_TRUE(fixture.awareness->tf_buffer()->setTransform(
        world_to_gripper, "object_handoff_test", true));
    iii_drone::types::transform_matrix_t target_transform =
        iii_drone::types::transform_matrix_t::Identity();
    target_transform(2, 3) = 1.5F;
    const iii_drone::adapters::TargetAdapter target(
        iii_drone::adapters::TARGET_TYPE_CABLE, 1, "world", target_transform);
    iii_drone::adapters::CombinedDroneAwarenessAdapter awareness;
    awareness.armed() = true;
    awareness.offboard() = true;
    awareness.drone_location() = iii_drone::adapters::DRONE_LOCATION_IN_FLIGHT;
    awareness.state() = fixture.awareness->GetState();
    *fixture.awareness->combined_drone_awareness_adapter_ = awareness;
    fixture.awareness->SetTarget(target);
    fixture.finishWithoutSchedulerTick(
        iii_drone::control::maneuver::MANEUVER_TYPE_CABLE_AWARE_FLY_TO_POSITION,
        "cable_aware_fly_to_position", true);
    fixture.scheduler.Start();
    fixture.scheduler.maneuver_execution_timer_->cancel();
    auto object = std::make_shared<
        iii_drone::control::maneuver::FlyToObjectManeuverServer>(
            &fixture.node, fixture.awareness, "fly_to_object", 1, 1,
            fixture.config, nullptr);
    auto hover_by_object = std::make_shared<
        iii_drone::control::maneuver::HoverByObjectManeuverServer>(
            &fixture.node, fixture.awareness, "hover_by_object", 1, 1, false, 1.0);
    fixture.scheduler.RegisterManeuverServer(
        iii_drone::control::maneuver::MANEUVER_TYPE_HOVER_BY_OBJECT,
        hover_by_object);
    fixture.scheduler.RegisterManeuverServer(
        iii_drone::control::maneuver::MANEUVER_TYPE_FLY_TO_OBJECT, object);
    object->target_adapter_ = target;
    object->maneuver_start_time_ = fixture.node.now();
    object->markTargetObserved();
    AcceptedObjectGoal accepted("tracked_object_fallback");
    const auto handle = accepted.accept(kRequestB, target);
    ASSERT_TRUE(handle);
    auto maneuver = iii_drone::control::maneuver::Maneuver::FromGoalHandle<
        AcceptedObjectGoal::Action>(handle);
    maneuver.Start();
    fixture.scheduler.current_maneuver_ = maneuver;
    const Reference seed = fixture.hold->lastCommand();
    fixture.scheduler.beginReferenceExecution(object->action_name(), kRequestB, seed);
    const auto source_execution = fixture.scheduler.current_reference_execution_id_.Load();
    auto interpolator = std::make_shared<iii_drone::control::TrajectoryInterpolator>(
        fixture.config, nullptr);
    auto tracking = std::make_shared<iii_drone::control::maneuver::ObjectTrackingSession>(
        [interpolator](const Reference & start, const Reference & target, bool reset) {
            return interpolator->ComputeBoundedPositionalTrajectory(
                start, target, true, reset).references().front();
        },
        seed, kRequestB, source_execution, fixture.node.now(), 0.0,
        iii_drone::control::maneuver::ObjectTrackingSession::Limits{});
    object->object_tracking_session_ = tracking;
    object->published_object_tracking_session_.Store(tracking);
    iii_drone::control::MeasuredOdometrySnapshot measured;
    measured.state = iii_drone::control::State(point_t::Zero(),
        vector_t(0.24F, 0.0F, 0.0F), 0.0, vector_t::Zero(), fixture.node.now());
    measured.receipt_stamp = fixture.node.now();
    measured.source_sample_timestamp_us = 1000000;
    fixture.awareness->measured_odometry_.Store(measured);
    Reference moving_command;
    std::string tracking_reason;
    ASSERT_TRUE(tracking->Compute(Reference(point_t::Zero(), 0.0), measured,
        fixture.node.now(), kRequestB, source_execution, 0.0, 0.4,
        moving_command, tracking_reason)) << tracking_reason;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    measured.receipt_stamp = fixture.node.now();
    measured.source_sample_timestamp_us += 100000;
    fixture.awareness->measured_odometry_.Store(measured);
    ASSERT_TRUE(tracking->Compute(Reference(point_t::Zero(), 0.0), measured,
        fixture.node.now(), kRequestB, source_execution, 0.0, 0.4,
        moving_command, tracking_reason)) << tracking_reason;
    const auto correction_before_handoff = tracking->correction();
    EXPECT_GT(correction_before_handoff.norm(), 1.0e-4);
    EXPECT_GT(moving_command.velocity().norm(), 1.0e-5);
    EXPECT_GT(moving_command.acceleration().norm(), 1.0e-5);
    fixture.scheduler.reference_callback_struct_->set(
        [tracking](const iii_drone::control::State &) {
            return tracking->lastCommand();
        }, object->action_name(), source_execution, kRequestB);
    ASSERT_TRUE(tracking->owns(kRequestB, source_execution));
    ASSERT_EQ(fixture.scheduler.reference_callback_struct_->snapshot().request_identity,
        kRequestB);
    ASSERT_EQ(fixture.scheduler.reference_callback_struct_->snapshot().execution_id,
        source_execution);
    (void)object->updateLiveTargetReference(fixture.awareness->GetState(),
        fixture.scheduler.reference_callback_struct_->snapshot());
    ASSERT_TRUE(object->nominal_target_observation_.Load());
    ASSERT_TRUE(hover_by_object->Update(target));

    ClientFixture consumer("tracked_object_fallback_consumer");
    ASSERT_TRUE(consumer.client.BeginManeuverGoalHandoff(kRequestA));
    ASSERT_TRUE(consumer.client.ConfirmManeuverGoalHandoff(kRequestA));
    auto predecessor = stream(consumer.node, "cwa:terminal", 1, seed, kRequestA);
    predecessor->terminal_hold_active = true;
    consumer.client.receiveReferenceStream(predecessor);
    (void)consumer.client.GetReference(0.02, [] {});
    ASSERT_TRUE(consumer.client.BeginManeuverGoalHandoff(kRequestB));
    ASSERT_TRUE(consumer.client.ConfirmManeuverGoalHandoff(kRequestB));
    consumer.executor.add_node(fixture.node.get_node_base_interface());
    for (int attempt = 0; attempt < 50 &&
         fixture.scheduler.reference_stream_publisher_->get_subscription_count() == 0;
         ++attempt) {
        consumer.executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    fixture.scheduler.publishReferenceStream();
    for (int attempt = 0; attempt < 50; ++attempt) {
        consumer.executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    const Reference first = consumer.client.GetReference(0.02, [] {});
    ASSERT_TRUE(first.position().allFinite());
    for (int attempt = 0; attempt < 50 &&
         !fixture.scheduler.reference_stream_state_.ack_seen; ++attempt) {
        consumer.executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    ASSERT_TRUE(fixture.scheduler.firstObjectReferenceApplied(kRequestB));
    std::atomic<bool> applied_in_source{false};
    std::atomic<bool> session_failed_in_source{false};
    std::atomic<bool> source_succeeded{false};
    std::atomic<bool> fallback_retained{false};
    std::promise<bool> callback_done;
    auto callback_result = callback_done.get_future();
    std::thread completion([&] {
        try {
            if (!object->reference_callback_token_->Acquire(2000)) {
                callback_done.set_value(false);
                return;
            }
            applied_in_source = fixture.scheduler.firstObjectReferenceApplied(kRequestB);
            source_succeeded = object->hasSucceeded(maneuver);
            session_failed_in_source = tracking->failed();
            fallback_retained = hover_by_object->RetainsTrackedSource(
                fixture.scheduler.reference_callback_struct_->snapshot());
            if (!source_succeeded || !fallback_retained) {
                object->reference_callback_token_->Release();
                callback_done.set_value(false);
                return;
            }
            object->registerReferenceCallbackOnSuccess(maneuver);
            maneuver.Terminate(true);
            fixture.scheduler.onManeuverCompleted(maneuver);
            object->reference_callback_token_->Release();
            callback_done.set_value(true);
        } catch (...) {
            callback_done.set_value(false);
        }
    });
    for (int attempt = 0; attempt < 200 &&
         !fixture.scheduler.reference_callback_token_.has_requested_token(
             object->action_name()); ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    const bool requested = fixture.scheduler.reference_callback_token_.has_requested_token(
        object->action_name());
    if (requested) fixture.scheduler.reference_callback_token_.Give(object->action_name());
    const bool completed = callback_result.get();
    completion.join();
    ASSERT_TRUE(requested);
    EXPECT_TRUE(applied_in_source);
    EXPECT_FALSE(session_failed_in_source) << tracking->failureReason();
    EXPECT_TRUE(source_succeeded);
    EXPECT_TRUE(fallback_retained);
    ASSERT_TRUE(completed);
    const auto fallback_binding = fixture.scheduler.reference_callback_struct_->snapshot();
    ASSERT_TRUE(hover_by_object->RetainsTrackedSource(fallback_binding));
    EXPECT_EQ(fallback_binding.request_identity, kRequestB);
    EXPECT_EQ(fallback_binding.execution_id, source_execution);
    EXPECT_FALSE(fixture.hover->terminalHold());
    const auto fallback = fallback_binding.callback(fixture.awareness->GetState());
    EXPECT_TRUE(fallback.position().allFinite());
    EXPECT_GT(tracking->correction().norm(), 1.0e-4);
    EXPECT_LT((tracking->correction() - correction_before_handoff).norm(), 1.0e-5);
    EXPECT_GT(fallback.velocity().norm(), 1.0e-5);
    const Reference client_before_completion = consumer.client.GetReference(0.02, [] {});
    ASSERT_TRUE(consumer.client.CompleteManeuverGoalHandoff(
        kRequestB, Reference(point_t::Zero(), 0.0).CopyWithNans()));
    const auto after_completion = consumer.client.GetReference(0.02, [] {});
    EXPECT_LT((after_completion.position() - client_before_completion.position()).norm(), 1.0e-5)
        << "FTO result cleanup dropped the corrected fallback command";
    EXPECT_LT((after_completion.velocity() - client_before_completion.velocity()).norm(), 1.0e-5);
    EXPECT_LT((after_completion.acceleration() - client_before_completion.acceleration()).norm(), 1.0e-5);

    // The real BT takes more than one 50 ms producer tick to send the next
    // action. Exercise the timer's completed -> NONE -> idle sequence before
    // accepting explicit Hover, rather than going straight to a manual Start.
    fixture.scheduler.no_maneuver_idle_cnt_ = -1;
    fixture.scheduler.maneuver_execution_timer_->reset();
    const auto source_stream_id = fixture.scheduler.reference_stream_state_.stream_id;
    const auto source_sequence = fixture.scheduler.reference_stream_state_.sequence;
    fixture.scheduler.maneuverExecutionTimerCallback();
    ASSERT_EQ(fixture.scheduler.current_maneuver_.Load().maneuver_type(),
        iii_drone::control::maneuver::MANEUVER_TYPE_NONE);
    fixture.scheduler.maneuverExecutionTimerCallback();
    const auto idle_binding = fixture.scheduler.reference_callback_struct_->snapshot();
    EXPECT_EQ(idle_binding.reference_provider_name, object->action_name());
    EXPECT_EQ(idle_binding.request_identity, kRequestB);
    EXPECT_EQ(idle_binding.execution_id, source_execution);
    EXPECT_FALSE(fixture.scheduler.maneuver_execution_timer_->is_canceled());
    EXPECT_TRUE(fixture.scheduler.reference_stream_state_.valid);
    EXPECT_EQ(fixture.scheduler.reference_stream_state_.stream_id, source_stream_id);
    EXPECT_GT(fixture.scheduler.reference_stream_state_.sequence, source_sequence);
    consumer.executor.spin_some();
    (void)consumer.client.GetReference(0.02, [] {});

    AcceptedHoverByObjectGoal hover_goal("tracked_object_explicit_hover");
    const auto hover_handle = hover_goal.accept(kRequestC, target);
    ASSERT_TRUE(hover_handle);
    auto hover_maneuver = iii_drone::control::maneuver::Maneuver::FromGoalHandle<
        AcceptedHoverByObjectGoal::Action>(hover_handle);
    ASSERT_TRUE(consumer.client.BeginManeuverGoalHandoff(kRequestC));
    ASSERT_TRUE(consumer.client.ConfirmManeuverGoalHandoff(kRequestC));
    fixture.scheduler.current_maneuver_ = hover_maneuver;
    fixture.scheduler.progressScheduler();
    ASSERT_TRUE(fixture.scheduler.current_maneuver_.Load().started());
    const auto hover_execution = fixture.scheduler.current_reference_execution_id_.Load();
    EXPECT_NE(hover_execution, source_execution);
    EXPECT_EQ(fixture.scheduler.reference_callback_struct_->snapshot().request_identity,
        kRequestC);
    EXPECT_FALSE(hover_by_object->hasSucceeded(hover_maneuver))
        << "explicit object Hover cannot finish before its marked generation is applied";

    std::promise<bool> hover_started_promise;
    auto hover_started_future = hover_started_promise.get_future();
    std::thread hover_start([&] {
        bool acquired = hover_by_object->reference_callback_token_->Acquire(2000);
        if (acquired) {
            try {
                hover_by_object->startExecution(hover_maneuver);
                hover_by_object->reference_callback_token_->resource().set(
                    [hover_by_object](const iii_drone::control::State & state) {
                        return hover_by_object->GetReference(state);
                    }, hover_by_object->action_name(), hover_execution, kRequestC);
            } catch (...) {
                acquired = false;
            }
            hover_by_object->reference_callback_token_->Release();
        }
        hover_started_promise.set_value(acquired);
    });
    for (int attempt = 0; attempt < 200 &&
         !fixture.scheduler.reference_callback_token_.has_requested_token(
             hover_by_object->action_name()); ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    fixture.scheduler.progressScheduler();
    const bool hover_started = hover_started_future.get();
    hover_start.join();
    ASSERT_TRUE(hover_started);
    ASSERT_TRUE(tracking->owns(kRequestC, hover_execution));
    EXPECT_LT((tracking->correction() - correction_before_handoff).norm(), 1.0e-5);

    bool hover_applied = false;
    const auto hover_deadline = std::chrono::steady_clock::now() +
        std::chrono::seconds(2);
    auto next_hover_consumer = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() < hover_deadline && !hover_applied) {
        fixture.scheduler.publishReferenceStream();
        consumer.executor.spin_some();
        if (std::chrono::steady_clock::now() >= next_hover_consumer) {
            measured.receipt_stamp = fixture.node.now();
            measured.source_sample_timestamp_us += 200000;
            fixture.awareness->measured_odometry_.Store(measured);
            (void)consumer.client.GetReference(0.02, [] {});
            next_hover_consumer += std::chrono::milliseconds(200);
        }
        consumer.executor.spin_some();
        hover_applied = fixture.scheduler.firstManeuverReferenceApplied(
            iii_drone::control::maneuver::MANEUVER_TYPE_HOVER_BY_OBJECT, kRequestC);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    ASSERT_TRUE(hover_applied);
    EXPECT_TRUE(hover_by_object->hasSucceeded(hover_maneuver));
    EXPECT_TRUE(consumer.client.CompleteManeuverGoalHandoff(
        kRequestC, Reference(point_t::Zero(), 0.0).CopyWithNans()));
    EXPECT_EQ(consumer.client.active_request_identity_, kRequestC);
    EXPECT_EQ(consumer.client.reference_mode_.Load(), ManeuverReferenceClient::MANEUVER);

    hover_maneuver.Terminate(true);
    fixture.scheduler.onManeuverCompleted(hover_maneuver);
    fixture.scheduler.onReferenceCallbackTokenReacquired();
    ASSERT_TRUE(hover_by_object->RetainsTrackedSource(
        fixture.scheduler.reference_callback_struct_->snapshot()));
    const auto hover_stream_id = fixture.scheduler.reference_stream_state_.stream_id;
    const auto hover_sequence = fixture.scheduler.reference_stream_state_.sequence;
    fixture.scheduler.maneuverExecutionTimerCallback();
    ASSERT_EQ(fixture.scheduler.current_maneuver_.Load().maneuver_type(),
        iii_drone::control::maneuver::MANEUVER_TYPE_NONE);
    fixture.scheduler.maneuverExecutionTimerCallback();
    const auto completed_hover_binding = fixture.scheduler.reference_callback_struct_->snapshot();
    EXPECT_TRUE(hover_by_object->RetainsTrackedSource(completed_hover_binding));
    EXPECT_EQ(completed_hover_binding.reference_provider_name, hover_by_object->action_name());
    EXPECT_FALSE(fixture.scheduler.maneuver_execution_timer_->is_canceled());
    EXPECT_TRUE(fixture.scheduler.reference_stream_state_.valid);
    EXPECT_EQ(fixture.scheduler.reference_stream_state_.stream_id, hover_stream_id);
    EXPECT_GT(fixture.scheduler.reference_stream_state_.sequence, hover_sequence);
    auto generator_service = fixture.node.create_service<
        iii_drone_interfaces::srv::ComputeReferenceTrajectory>(
            "/control/trajectory_generator/compute_reference_trajectory",
            [](const std::shared_ptr<iii_drone_interfaces::srv::ComputeReferenceTrajectory::Request>,
               std::shared_ptr<iii_drone_interfaces::srv::ComputeReferenceTrajectory::Response>) {});
    auto generator_group = fixture.node.create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    auto generator_client = std::make_shared<iii_drone::control::TrajectoryGeneratorClient>(
        &fixture.node, fixture.config, generator_group);
    auto landing = std::make_shared<
        iii_drone::control::maneuver::CableLandingManeuverServer>(
            &fixture.node, fixture.awareness, "cable_landing", 1, 1,
            fixture.config, generator_client);
    fixture.scheduler.RegisterManeuverServer(
        iii_drone::control::maneuver::MANEUVER_TYPE_CABLE_LANDING, landing);
    AcceptedLandingGoal landing_goal("tracked_object_full_landing");
    const auto landing_handle = landing_goal.accept(kRequestD);
    ASSERT_TRUE(landing_handle);
    ASSERT_TRUE(consumer.client.BeginManeuverGoalHandoff(kRequestD));
    ASSERT_TRUE(consumer.client.ConfirmManeuverGoalHandoff(kRequestD));
    fixture.scheduler.current_maneuver_ =
        iii_drone::control::maneuver::Maneuver::FromGoalHandle<
            AcceptedLandingGoal::Action>(landing_handle);
    fixture.scheduler.progressScheduler();
    ASSERT_TRUE(tracking->transitionStopping());
    bool landing_started = false;
    bool handoff_failed = false;
    const auto landing_deadline = std::chrono::steady_clock::now() +
        std::chrono::seconds(3);
    auto next_landing_consumer = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() < landing_deadline && !landing_started) {
        fixture.scheduler.publishReferenceStream();
        consumer.executor.spin_some();
        if (std::chrono::steady_clock::now() >= next_landing_consumer) {
            (void)consumer.client.GetReference(0.02,
                [&handoff_failed] { handoff_failed = true; });
            next_landing_consumer += std::chrono::milliseconds(200);
        }
        fixture.scheduler.progressScheduler();
        landing_started = fixture.scheduler.current_maneuver_.Load().started();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    EXPECT_FALSE(handoff_failed);
    ASSERT_TRUE(landing_started);
    ASSERT_TRUE(tracking->transitionRest());
    const Reference landing_seed = tracking->lastCommand();
    EXPECT_LT(landing_seed.velocity().norm(), 1.0e-5);
    EXPECT_LT(landing_seed.acceleration().norm(), 1.0e-5);
    EXPECT_LT((landing_seed.position() - seed.position()).norm(), 0.4);
    const auto landing_binding = fixture.scheduler.reference_callback_struct_->snapshot();
    ASSERT_EQ(landing_binding.request_identity, kRequestD);
    ASSERT_TRUE(landing_binding.callback);
    EXPECT_LT((landing_binding.callback(fixture.awareness->GetState()).position() -
               landing_seed.position()).norm(), 1.0e-5);

    std::promise<bool> landing_started_promise;
    auto landing_started_future = landing_started_promise.get_future();
    std::optional<Reference> first_line_pid;
    std::thread landing_start([&] {
        bool acquired = landing->reference_callback_token_->Acquire(2000);
        if (acquired) {
            try {
                auto landing_maneuver = fixture.scheduler.current_maneuver_.Load();
                landing->startExecution(landing_maneuver);
                const Reference first = landing->computeReference(
                    fixture.awareness->GetState());
                first_line_pid = first;
                landing->reference_callback_token_->resource().set(
                    [first, &fixture](const iii_drone::control::State &) {
                        return first.CopyWithNewStamp(fixture.node.now());
                    }, landing->action_name(), landing_binding.execution_id, kRequestD);
                acquired = landing->object_transition_start_reference_.has_value() &&
                    landing->line_pid_initialized_ &&
                    std::abs(first.velocity()(2) - 0.20F) < 1.0e-5;
            } catch (...) {
                acquired = false;
            }
            landing->reference_callback_token_->Release();
        }
        landing_started_promise.set_value(acquired);
    });
    for (int attempt = 0; attempt < 200 &&
         !fixture.scheduler.reference_callback_token_.has_requested_token(
             landing->action_name()); ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    fixture.scheduler.progressScheduler();
    const bool line_pid_started = landing_started_future.get();
    landing_start.join();
    ASSERT_TRUE(line_pid_started);
    ASSERT_TRUE(first_line_pid);
    fixture.scheduler.publishReferenceStream();
    for (int attempt = 0; attempt < 30; ++attempt) {
        consumer.executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    bool landing_consumer_failed = false;
    const Reference first_consumed_landing = consumer.client.GetReference(
        0.02, [&landing_consumer_failed] { landing_consumer_failed = true; });
    EXPECT_FALSE(landing_consumer_failed);
    EXPECT_FALSE(consumer.client.reference_safety_guard_->faultLatched());
    EXPECT_NEAR(first_consumed_landing.velocity()(2), 0.20, 1.0e-5);
    EXPECT_TRUE(std::isnan(first_consumed_landing.position()(2)));
    EXPECT_NEAR(first_consumed_landing.position()(0), landing_seed.position()(0), 1.0e-4);
    consumer.executor.remove_node(fixture.node.get_node_base_interface());
    object->Stop();
    hover_by_object->Stop();
    landing->Stop();
    fixture.scheduler.registered_maneuvers_.erase(
        iii_drone::control::maneuver::MANEUVER_TYPE_FLY_TO_OBJECT);
    fixture.scheduler.registered_maneuvers_.erase(
        iii_drone::control::maneuver::MANEUVER_TYPE_HOVER_BY_OBJECT);
    fixture.scheduler.registered_maneuvers_.erase(
        iii_drone::control::maneuver::MANEUVER_TYPE_CABLE_LANDING);
}

TEST(ManeuverReferenceClientTransaction, ObjectIdleRetentionRejectsWrongGenerationOrRetiredSession) {
    RclcppContext context;
    for (const bool wrong_generation : {true, false}) {
        TerminalCompletionFixture fixture(
            wrong_generation ? "object_idle_wrong_generation" : "object_idle_retired",
            1000, "/control/maneuver_controller");
        fixture.hover->ClearTerminalHold();
        fixture.scheduler.Start();
        fixture.scheduler.maneuver_execution_timer_->cancel();
        auto object = std::make_shared<
            iii_drone::control::maneuver::HoverByObjectManeuverServer>(
                &fixture.node, fixture.awareness, "hover_by_object_idle", 1, 1,
                false, 1.0);
        fixture.scheduler.registered_maneuvers_[
            iii_drone::control::maneuver::MANEUVER_TYPE_HOVER_BY_OBJECT] = object;
        iii_drone::control::maneuver::Maneuver completed(
            iii_drone::control::maneuver::MANEUVER_TYPE_HOVER_BY_OBJECT,
            rclcpp_action::GoalUUID{});
        completed.request_identity_ = kRequestB;
        completed.started_ = true;
        fixture.scheduler.current_maneuver_ = completed;
        const Reference command(point_t(0.24F, 0.0F, 1.5F), 0.0,
            vector_t::Zero(), 0.0, vector_t::Zero(), 0.0, fixture.node.now());
        fixture.scheduler.beginReferenceExecution(object->action_name(), kRequestB, command);
        const auto execution = fixture.scheduler.current_reference_execution_id_.Load();
        auto session = std::make_shared<
            iii_drone::control::maneuver::ObjectTrackingSession>(
                [command](const Reference &, const Reference &, bool) { return command; },
                command, kRequestB, execution, fixture.node.now(), 0.0,
                iii_drone::control::maneuver::ObjectTrackingSession::Limits{});
        object->object_tracking_session_ = session;
        object->object_owner_request_identity_ = kRequestB;
        object->object_owner_execution_id_ = execution;
        fixture.scheduler.reference_callback_struct_->set(
            [command](const iii_drone::control::State &) { return command; },
            object->action_name(), execution, kRequestB);
        fixture.scheduler.publishReferenceStream();
        ASSERT_TRUE(fixture.scheduler.reference_stream_state_.valid);
        completed.Terminate(true);
        fixture.scheduler.current_maneuver_ = completed;
        fixture.scheduler.onReferenceCallbackTokenReacquired();
        ASSERT_TRUE(fixture.scheduler.retained_native_hold_epoch_.completed);
        fixture.scheduler.maneuver_execution_timer_->reset();
        fixture.scheduler.maneuverExecutionTimerCallback();
        ASSERT_EQ(fixture.scheduler.current_maneuver_.Load().maneuver_type(),
            iii_drone::control::maneuver::MANEUVER_TYPE_NONE);
        const auto old_binding = fixture.scheduler.reference_callback_struct_->snapshot();
        if (wrong_generation) {
            fixture.scheduler.current_reference_execution_id_.Store(execution + 1);
        } else {
            object->RetireTrackedSource(old_binding);
        }
        fixture.scheduler.maneuverExecutionTimerCallback();
        EXPECT_EQ(fixture.scheduler.reference_callback_struct_->snapshot()
                      .reference_provider_name, "passthrough");
        EXPECT_FALSE(fixture.scheduler.reference_stream_state_.valid);
        EXPECT_TRUE(fixture.scheduler.maneuver_execution_timer_->is_canceled());
    }
}

TEST(ManeuverReferenceClientTransaction, ReleasedFtoManagedCallbackCannotPoisonTrackedHoverAdoption) {
    RclcppContext context;
    TerminalCompletionFixture fixture(
        "released_fto_managed_callback", 1000, "/control/maneuver_controller");
    fixture.awareness->vehicle_odometry_adapter_history_ = std::make_shared<
        iii_drone::utils::History<VehicleOdometryAdapter>>(2);
    fixture.awareness->vehicle_odometry_adapter_history_->Store(stationaryVehicleState());
    fixture.awareness->powerline_adapter_history_ = std::make_shared<
        iii_drone::utils::History<iii_drone::adapters::PowerlineAdapter>>(1);
    const auto stamp = fixture.node.now();
    const point_t line_position(0.0F, 0.0F, 1.5F);
    const auto orientation = iii_drone::math::eulToQuat(
        iii_drone::types::euler_angles_t::Zero());
    const iii_drone::adapters::SingleLineAdapter line(
        stamp, "world", 1, line_position, line_position, orientation, true);
    fixture.awareness->powerline_adapter_history_->Store(
        iii_drone::adapters::PowerlineAdapter(stamp, {line},
            iii_drone::types::createPlane(line_position, vector_t::UnitX())));
    geometry_msgs::msg::TransformStamped world_to_drone;
    world_to_drone.header.stamp = stamp;
    world_to_drone.header.frame_id = "world";
    world_to_drone.child_frame_id = "drone";
    world_to_drone.transform.rotation.w = 1.0;
    ASSERT_TRUE(fixture.awareness->tf_buffer()->setTransform(
        world_to_drone, "released_fto_managed_callback", true));
    iii_drone::types::transform_matrix_t target_transform =
        iii_drone::types::transform_matrix_t::Identity();
    target_transform(2, 3) = 1.5F;
    const iii_drone::adapters::TargetAdapter target(
        iii_drone::adapters::TARGET_TYPE_CABLE, 1, "world", target_transform);
    iii_drone::adapters::CombinedDroneAwarenessAdapter awareness;
    awareness.armed() = true;
    awareness.offboard() = true;
    awareness.drone_location() = iii_drone::adapters::DRONE_LOCATION_IN_FLIGHT;
    awareness.state() = fixture.awareness->GetState();
    *fixture.awareness->combined_drone_awareness_adapter_ = awareness;
    fixture.awareness->SetTarget(target);

    fixture.scheduler.Start();
    fixture.scheduler.maneuver_execution_timer_->cancel();
    auto hover_by_object = std::make_shared<
        iii_drone::control::maneuver::HoverByObjectManeuverServer>(
            &fixture.node, fixture.awareness, "hover_by_object", 1, 1, false, 1.0);
    fixture.scheduler.RegisterManeuverServer(
        iii_drone::control::maneuver::MANEUVER_TYPE_HOVER_BY_OBJECT,
        hover_by_object);
    auto object = std::make_shared<
        iii_drone::control::maneuver::FlyToObjectManeuverServer>(
            &fixture.node, fixture.awareness, "fly_to_object", 1, 1,
            fixture.config, nullptr);
    fixture.scheduler.RegisterManeuverServer(
        iii_drone::control::maneuver::MANEUVER_TYPE_FLY_TO_OBJECT, object);
    const Reference seed = fixture.hold->lastCommand();
    fixture.scheduler.beginReferenceExecution(object->action_name(), kRequestB, seed);
    const auto execution = fixture.scheduler.current_reference_execution_id_.Load();
    auto tracking = std::make_shared<
        iii_drone::control::maneuver::ObjectTrackingSession>(
            [](const Reference & start, const Reference &, bool) { return start; },
            seed, kRequestB, execution, fixture.node.now(), 0.0,
            iii_drone::control::maneuver::ObjectTrackingSession::Limits{});
    const auto measured = fixture.awareness->GetMeasuredOdometry();
    ASSERT_TRUE(measured);
    Reference accepted_command;
    std::string reason;
    ASSERT_TRUE(tracking->Compute(seed, *measured, fixture.node.now(), kRequestB,
        execution, 0.0, 0.4, accepted_command, reason)) << reason;
    ASSERT_FALSE(tracking->failed());
    object->target_adapter_ = target;
    object->object_tracking_session_ = tracking;
    object->published_object_tracking_session_.Store(tracking);
    object->active_target_reference_ = seed;
    object->active_target_reference_valid_ = true;
    object->markTargetObserved();
    ASSERT_TRUE(hover_by_object->UpdateTracked(target, tracking, kRequestB,
        execution, 0.0));
    object->object_hover_ready_ = true;

    iii_drone::control::maneuver::Maneuver completed(
        iii_drone::control::maneuver::MANEUVER_TYPE_FLY_TO_OBJECT,
        rclcpp_action::GoalUUID{});
    completed.request_identity_ = kRequestB;
    // This fixture has no ROS goal handle; mirror the scheduler's started
    // value while still exercising the real callback token and registration.
    completed.started_ = true;
    fixture.scheduler.current_maneuver_ = completed;
    using Binding = iii_drone::control::maneuver::ReferenceCallbackBinding;
    std::promise<Binding> old_binding_promise;
    auto old_binding_future = old_binding_promise.get_future();
    std::promise<void> completed_promise;
    auto completed_future = completed_promise.get_future();
    std::thread completion([&] {
        try {
            if (!object->reference_callback_token_->Acquire(2000)) {
                throw std::runtime_error("FTO worker did not acquire callback token");
            }
            // Install the actual managed FTO callable used by asyncExecute,
            // then retain its copied binding across success and Token::Release.
            object->reference_callback_token_->resource().set(
                std::bind(&iii_drone::control::maneuver::ManeuverServer::computeManagedReference,
                    object.get(), std::placeholders::_1),
                object->action_name(), execution, kRequestB);
            old_binding_promise.set_value(
                fixture.scheduler.reference_callback_struct_->snapshot());
            object->registerReferenceCallbackOnSuccess(completed);
            completed.Terminate(true);
            fixture.scheduler.onManeuverCompleted(completed);
            object->reference_callback_token_->Release();
            completed_promise.set_value();
        } catch (...) {
            completed_promise.set_exception(std::current_exception());
        }
    });
    for (int attempt = 0; attempt < 200 &&
         !fixture.scheduler.reference_callback_token_.has_requested_token(
             object->action_name()); ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    const bool requested = fixture.scheduler.reference_callback_token_.has_requested_token(
        object->action_name());
    if (requested) fixture.scheduler.reference_callback_token_.Give(object->action_name());
    completion.join();
    ASSERT_TRUE(requested);
    completed_future.get();
    const auto old_binding = old_binding_future.get();
    ASSERT_TRUE(old_binding.callback);
    ASSERT_EQ(old_binding.request_identity, kRequestB);
    ASSERT_EQ(old_binding.execution_id, execution);
    ASSERT_TRUE(hover_by_object->RetainsTrackedSource(
        fixture.scheduler.reference_callback_struct_->snapshot()));
    ASSERT_FALSE(tracking->failed());

    bool retired = false;
    try {
        (void)old_binding.callback(fixture.awareness->GetState());
    } catch (const iii_drone::control::maneuver::RetiredReferenceCallback &) {
        retired = true;
    }
    EXPECT_TRUE(retired) << "a copied managed callback must not run after release";
    EXPECT_FALSE(tracking->failed()) << tracking->failureReason();
    EXPECT_TRUE(tracking->Adopt(kRequestB, execution, kRequestC, execution + 1,
        accepted_command, fixture.node.now(), reason)) << reason;
    object->Stop();
    hover_by_object->Stop();
    fixture.scheduler.registered_maneuvers_.erase(
        iii_drone::control::maneuver::MANEUVER_TYPE_FLY_TO_OBJECT);
    fixture.scheduler.registered_maneuvers_.erase(
        iii_drone::control::maneuver::MANEUVER_TYPE_HOVER_BY_OBJECT);
}

void verifyFtoTimingFaultLogsCapturedAndIngressIdentityOnce(bool interval_fault) {
    RclcppContext context;
    TerminalCompletionFixture fixture(interval_fault ? "fto_interval_evidence" :
        "fto_freshness_evidence", 1000,
        "/control/maneuver_controller", 1.0, 0.0);
    auto * clock_handle = fixture.node.get_clock()->get_clock_handle();
    ASSERT_EQ(rcl_enable_ros_time_override(clock_handle), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(clock_handle, 10'000'000'000LL), RCL_RET_OK);
    fixture.awareness->vehicle_odometry_adapter_history_ = std::make_shared<
        iii_drone::utils::History<VehicleOdometryAdapter>>(2);
    fixture.awareness->measured_odometry_.Store(std::nullopt);
    px4_msgs::msg::VehicleOdometry raw;
    raw.pose_frame = iii_drone::adapters::px4::POSE_FRAME_LOCAL_NED;
    raw.velocity_frame = iii_drone::adapters::px4::VELOCITY_FRAME_LOCAL_NED;
    raw.q[0] = 1.0F;
    raw.position[2] = -1.5F;
    raw.reset_counter = 17;
    raw.timestamp_sample = 1'000'000;
    fixture.awareness->ingestVehicleOdometry(raw, fixture.node.now());
    ASSERT_EQ(fixture.awareness->GetMeasuredOdometry()->source_sample_timestamp_us,
        1'000'000U);

    const auto stamp = fixture.node.now();
    const point_t line_position(0.0F, 0.0F, 1.5F);
    const auto orientation = iii_drone::math::eulToQuat(
        iii_drone::types::euler_angles_t::Zero());
    const iii_drone::adapters::SingleLineAdapter line(
        stamp, "world", 1, line_position, line_position, orientation, true);
    fixture.awareness->powerline_adapter_history_ = std::make_shared<
        iii_drone::utils::History<iii_drone::adapters::PowerlineAdapter>>(1);
    fixture.awareness->powerline_adapter_history_->Store(
        iii_drone::adapters::PowerlineAdapter(stamp, {line},
            iii_drone::types::createPlane(line_position, vector_t::UnitX())));
    geometry_msgs::msg::TransformStamped world_to_drone;
    world_to_drone.header.stamp = stamp;
    world_to_drone.header.frame_id = "world";
    world_to_drone.child_frame_id = "drone";
    world_to_drone.transform.rotation.w = 1.0;
    ASSERT_TRUE(fixture.awareness->tf_buffer()->setTransform(
        world_to_drone, "fto_freshness_evidence", true));
    iii_drone::types::transform_matrix_t target_transform =
        iii_drone::types::transform_matrix_t::Identity();
    target_transform(2, 3) = 1.5F;
    const iii_drone::adapters::TargetAdapter target(
        iii_drone::adapters::TARGET_TYPE_CABLE, 1, "world", target_transform);

    fixture.scheduler.Start();
    fixture.scheduler.maneuver_execution_timer_->cancel();
    auto object = std::make_shared<
        iii_drone::control::maneuver::FlyToObjectManeuverServer>(
            &fixture.node, fixture.awareness, "fly_to_object", 1, 1,
            fixture.config, nullptr);
    fixture.scheduler.RegisterManeuverServer(
        iii_drone::control::maneuver::MANEUVER_TYPE_FLY_TO_OBJECT, object);
    const Reference seed(point_t(0.0F, 0.0F, 1.5F), 0.0,
        vector_t::Zero(), 0.0, vector_t::Zero(), 0.0, stamp);
    fixture.scheduler.beginReferenceExecution(object->action_name(), kRequestB, seed);
    const auto execution = fixture.scheduler.current_reference_execution_id_.Load();
    auto tracking = std::make_shared<
        iii_drone::control::maneuver::ObjectTrackingSession>(
            [](const Reference & start, const Reference &, bool) { return start; },
            seed, kRequestB, execution, stamp, 0.0,
            iii_drone::control::maneuver::ObjectTrackingSession::Limits{});
    object->target_adapter_ = target;
    object->object_tracking_session_ = tracking;
    object->published_object_tracking_session_.Store(tracking);
    object->first_iteration_ = false;
    if (!interval_fault) {
        ASSERT_EQ(rcl_set_ros_time_override(clock_handle, 10'300'000'000LL), RCL_RET_OK);
    }

    testing::internal::CaptureStderr();
    std::exception_ptr worker_error;
    std::thread worker([&] {
        try {
            if (!object->reference_callback_token_->Acquire(2000)) {
                throw std::runtime_error("FTO did not acquire callback token");
            }
            if (interval_fault) {
                // Accept the first original source sample, then deliver a
                // fresh receipt whose source interval alone exceeds 250 ms.
                (void)object->computeReference(fixture.awareness->GetState());
                if (rcl_set_ros_time_override(clock_handle, 10'100'000'000LL) != RCL_RET_OK) {
                    throw std::runtime_error("could not advance fixture ROS clock");
                }
                raw.timestamp_sample = 1'300'000;
                fixture.awareness->ingestVehicleOdometry(raw, fixture.node.now());
            }
            (void)object->computeReference(fixture.awareness->GetState());
            (void)object->computeReference(fixture.awareness->GetState());
            object->reference_callback_token_->Release();
        } catch (...) {
            worker_error = std::current_exception();
        }
    });
    for (int attempt = 0; attempt < 200 &&
         !fixture.scheduler.reference_callback_token_.has_requested_token(
             object->action_name()); ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    const bool requested = fixture.scheduler.reference_callback_token_.has_requested_token(
        object->action_name());
    if (requested) fixture.scheduler.reference_callback_token_.Give(object->action_name());
    worker.join();
    const std::string logs = testing::internal::GetCapturedStderr();
    object->Stop();
    ASSERT_TRUE(requested);
    if (worker_error) std::rethrow_exception(worker_error);
    ASSERT_TRUE(tracking->failed());
    const auto diagnostic = std::string("Object approach timing-fault evidence:");
    EXPECT_NE(logs.find(diagnostic), std::string::npos);
    EXPECT_NE(logs.find("captured_source_us=" +
        std::string(interval_fault ? "1300000" : "1000000")), std::string::npos);
    EXPECT_NE(logs.find("captured_receipt_ros_ns=" +
        std::string(interval_fault ? "10100000000" : "10000000000")), std::string::npos);
    EXPECT_NE(logs.find("emission_ros_ns=" +
        std::string(interval_fault ? "10100000000" : "10300000000")), std::string::npos);
    EXPECT_NE(logs.find("latest_source_us=" +
        std::string(interval_fault ? "1300000" : "1000000")), std::string::npos);
    EXPECT_NE(logs.find("latest_receipt_ros_ns=" +
        std::string(interval_fault ? "10100000000" : "10000000000")), std::string::npos);
    EXPECT_NE(logs.find("ingress_count=" +
        std::string(interval_fault ? "2" : "1")), std::string::npos);
    if (interval_fault) {
        EXPECT_NE(tracking->failureReason().find("source_interval_s=0.300000"),
            std::string::npos);
    } else {
        EXPECT_NE(tracking->failureReason().find("odometry_stale=true"),
            std::string::npos);
    }
    EXPECT_EQ(logs.find(diagnostic, logs.find(diagnostic) + 1), std::string::npos);
}

TEST(ManeuverReferenceClientTransaction, FtoFreshnessFaultLogsCapturedAndIngressIdentityOnce) {
    verifyFtoTimingFaultLogsCapturedAndIngressIdentityOnce(false);
}

TEST(ManeuverReferenceClientTransaction, FtoIntervalFaultLogsCapturedAndIngressIdentityOnce) {
    verifyFtoTimingFaultLogsCapturedAndIngressIdentityOnce(true);
}

TEST(ManeuverReferenceClientTransaction, ObjectClockMismatchSignalsOnlyFirstTimingFault) {
    using Session = iii_drone::control::maneuver::ObjectTrackingSession;
    const rclcpp::Time emission(10'000'000'000LL, RCL_SYSTEM_TIME);
    const Reference seed(point_t(0.0F, 0.0F, 1.5F), 0.0,
        vector_t::Zero(), 0.0, vector_t::Zero(), 0.0, emission);
    Session session([](const Reference & start, const Reference &, bool) { return start; },
        seed, kRequestB, 1, emission, 0.0, Session::Limits{});
    iii_drone::control::MeasuredOdometrySnapshot measured;
    measured.state = iii_drone::control::State(seed.position(), vector_t::Zero(),
        0.0, vector_t::Zero(), emission);
    measured.receipt_stamp = rclcpp::Time(10'000'000'000LL, RCL_ROS_TIME);
    measured.source_sample_timestamp_us = 1'000'000;
    Reference output;
    std::string reason;
    bool first_timing_fault = false;
    ASSERT_FALSE(session.Compute(seed, measured, emission, kRequestB, 1, 0.0, 0.4,
        output, reason, &first_timing_fault));
    EXPECT_TRUE(first_timing_fault);
    EXPECT_NE(reason.find("command and odometry clocks differ"), std::string::npos);
    first_timing_fault = true;
    ASSERT_FALSE(session.Compute(seed, measured, emission, kRequestB, 1, 0.0, 0.4,
        output, reason, &first_timing_fault));
    EXPECT_FALSE(first_timing_fault);
    EXPECT_EQ(reason, session.failureReason());
}

TEST(ManeuverReferenceClientTransaction, BlendedFtpActionCompletionKeepsExactManagedLeaseLive) {
    RclcppContext context;
    TerminalCompletionFixture fixture("blended_ftp_lease_completion", 1000,
        "/control/maneuver_controller");
    fixture.hover->Update(Reference(fixture.hold->lastCommand().position(), 0.0));
    fixture.scheduler.Start();
    fixture.scheduler.maneuver_execution_timer_->cancel();
    auto ftp = std::make_shared<BlendedCompletionLeaseServer>(
        &fixture.node, fixture.awareness, "fly_to_position", 1, 1,
        fixture.config, nullptr);
    fixture.scheduler.RegisterManeuverServer(
        iii_drone::control::maneuver::MANEUVER_TYPE_FLY_TO_POSITION, ftp);
    AcceptedPositionGoal goal("blended_ftp_lease_completion_goal");
    const auto handle = goal.accept(kRequestB);
    ASSERT_TRUE(handle);
    fixture.scheduler.current_maneuver_ =
        iii_drone::control::maneuver::Maneuver::FromGoalHandle<
            AcceptedPositionGoal::Action>(handle);
    fixture.scheduler.current_maneuver_->Start();
    const Reference seed(point_t(0.4F, 0.0F, 1.0F), 0.0);
    fixture.scheduler.beginReferenceExecution(ftp->action_name(), kRequestB, seed);
    const auto prepared = fixture.scheduler.reference_callback_struct_->snapshot();
    std::promise<bool> completed;
    auto completed_future = completed.get_future();
    std::thread worker([&] {
        try {
            ftp->asyncExecute<AcceptedPositionGoal::Action>(handle);
            completed.set_value(true);
        } catch (...) {
            completed.set_value(false);
        }
    });
    for (int attempt = 0; attempt < 200 &&
         !fixture.scheduler.reference_callback_token_.has_requested_token(
             ftp->action_name()); ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    const bool requested = fixture.scheduler.reference_callback_token_.has_requested_token(
        ftp->action_name());
    if (requested) fixture.scheduler.reference_callback_token_.Give(ftp->action_name());
    iii_drone::control::maneuver::ReferenceCallbackBinding managed;
    for (int attempt = 0; attempt < 200; ++attempt) {
        managed = fixture.scheduler.reference_callback_struct_->snapshot();
        if (managed.lease && managed.revision > prepared.revision) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    const bool installed = managed.lease && managed.revision > prepared.revision;
    if (installed) (void)managed.callback(fixture.awareness->GetState());
    std::promise<void> entered;
    auto entered_future = entered.get_future();
    std::promise<void> release;
    const auto released = release.get_future().share();
    std::atomic<bool> entered_once{false};
    ftp->compute_hook = [&] {
        if (!entered_once.exchange(true)) {
            entered.set_value();
            released.wait();
        }
    };
    std::thread in_flight;
    if (installed) {
        in_flight = std::thread([&] {
            (void)managed.callback(fixture.awareness->GetState());
        });
    }
    const bool callback_entered = installed &&
        entered_future.wait_for(std::chrono::seconds(2)) == std::future_status::ready;
    ftp->allow_success = true;
    bool completion_waited_for_callback = false;
    if (callback_entered) {
        for (int attempt = 0; attempt < 200 && !managed.lease->quiescing(); ++attempt) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        completion_waited_for_callback = managed.lease->quiescing() &&
            completed_future.wait_for(std::chrono::milliseconds(0)) !=
                std::future_status::ready &&
            fixture.scheduler.reference_callback_token_.token_holder() ==
                ftp->action_name();
    }
    release.set_value();
    if (in_flight.joinable()) in_flight.join();
    ftp->compute_hook = nullptr;
    const bool finished = completed_future.wait_for(std::chrono::seconds(3)) ==
        std::future_status::ready;
    if (!finished) ftp->running_.Store(false);
    worker.join();
    ASSERT_TRUE(requested);
    ASSERT_TRUE(installed);
    ASSERT_TRUE(callback_entered);
    EXPECT_TRUE(completion_waited_for_callback)
        << "the action released its token before the entered callback drained";
    ASSERT_TRUE(finished);
    ASSERT_TRUE(completed_future.get());
    EXPECT_TRUE(fixture.scheduler.current_maneuver_.Load().success());
    const auto retained = fixture.scheduler.reference_callback_struct_->snapshot();
    EXPECT_EQ(retained.request_identity, kRequestB);
    EXPECT_EQ(retained.execution_id, managed.execution_id);
    EXPECT_EQ(retained.revision, managed.revision);
    EXPECT_EQ(retained.lease, managed.lease);
    ASSERT_TRUE(retained.lease);
    EXPECT_FALSE(retained.lease->retired());
    EXPECT_GT(ftp->compute_calls.load(), 0U);
    EXPECT_LT((retained.callback(fixture.awareness->GetState()).position() -
        seed.position()).norm(), 1.0e-5);
    ftp->Stop();
    fixture.scheduler.registered_maneuvers_.erase(
        iii_drone::control::maneuver::MANEUVER_TYPE_FLY_TO_POSITION);
}

TEST(ManeuverReferenceClientTransaction, FailedFallbackDuringDrainKeepsOldStopAndRejectsHover) {
    RclcppContext context;
    TerminalCompletionFixture fixture("failed_fallback_during_drain", 1000,
        "/control/maneuver_controller");
    fixture.hover->Update(Reference(fixture.hold->lastCommand().position(), 0.0));
    fixture.awareness->vehicle_odometry_adapter_history_ = std::make_shared<
        iii_drone::utils::History<VehicleOdometryAdapter>>(2);
    fixture.awareness->vehicle_odometry_adapter_history_->Store(stationaryVehicleState());
    fixture.awareness->powerline_adapter_history_ = std::make_shared<
        iii_drone::utils::History<iii_drone::adapters::PowerlineAdapter>>(1);
    const auto stamp = fixture.node.now();
    const point_t line_position(0.0F, 0.0F, 1.5F);
    const auto orientation = iii_drone::math::eulToQuat(
        iii_drone::types::euler_angles_t::Zero());
    const iii_drone::adapters::SingleLineAdapter line(
        stamp, "world", 1, line_position, line_position, orientation, true);
    fixture.awareness->powerline_adapter_history_->Store(
        iii_drone::adapters::PowerlineAdapter(stamp, {line},
            iii_drone::types::createPlane(line_position, vector_t::UnitX())));
    geometry_msgs::msg::TransformStamped world_to_drone;
    world_to_drone.header.stamp = stamp;
    world_to_drone.header.frame_id = "world";
    world_to_drone.child_frame_id = "drone";
    world_to_drone.transform.rotation.w = 1.0;
    ASSERT_TRUE(fixture.awareness->tf_buffer()->setTransform(
        world_to_drone, "failed_fallback_during_drain", true));
    iii_drone::types::transform_matrix_t target_transform =
        iii_drone::types::transform_matrix_t::Identity();
    target_transform(2, 3) = 1.5F;
    const iii_drone::adapters::TargetAdapter target(
        iii_drone::adapters::TARGET_TYPE_CABLE, 1, "world", target_transform);
    iii_drone::adapters::CombinedDroneAwarenessAdapter awareness;
    awareness.armed() = true;
    awareness.offboard() = true;
    awareness.drone_location() = iii_drone::adapters::DRONE_LOCATION_IN_FLIGHT;
    awareness.state() = fixture.awareness->GetState();
    *fixture.awareness->combined_drone_awareness_adapter_ = awareness;
    fixture.awareness->SetTarget(target);

    fixture.scheduler.Start();
    fixture.scheduler.maneuver_execution_timer_->cancel();
    auto hover_by_object = std::make_shared<
        iii_drone::control::maneuver::HoverByObjectManeuverServer>(
            &fixture.node, fixture.awareness, "hover_by_object", 1, 1, false, 1.0);
    fixture.scheduler.RegisterManeuverServer(
        iii_drone::control::maneuver::MANEUVER_TYPE_HOVER_BY_OBJECT,
        hover_by_object);
    auto object = std::make_shared<
        iii_drone::control::maneuver::FlyToObjectManeuverServer>(
            &fixture.node, fixture.awareness, "fly_to_object", 1, 1,
            fixture.config, nullptr);
    fixture.scheduler.RegisterManeuverServer(
        iii_drone::control::maneuver::MANEUVER_TYPE_FLY_TO_OBJECT, object);
    const Reference seed(fixture.hold->lastCommand().position(), 0.0,
        vector_t(0.12F, 0.0F, 0.0F), 0.0, vector_t::Zero(), 0.0);
    fixture.scheduler.beginReferenceExecution(object->action_name(), kRequestB, seed);
    const auto execution = fixture.scheduler.current_reference_execution_id_.Load();
    auto tracking = std::make_shared<
        iii_drone::control::maneuver::ObjectTrackingSession>(
            [](const Reference & start, const Reference &, bool) { return start; },
            seed, kRequestB, execution, fixture.node.now(), 0.0,
            iii_drone::control::maneuver::ObjectTrackingSession::Limits{});
    ASSERT_TRUE(hover_by_object->UpdateTracked(target, tracking, kRequestB,
        execution, 0.0));
    fixture.scheduler.reference_callback_struct_->set(
        [tracking](const iii_drone::control::State &) {
            return tracking->lastCommand();
        }, object->action_name(), execution, kRequestB);
    fixture.scheduler.maneuver_server_get_reference_callback_still_registered_ = true;
    fixture.scheduler.publishReferenceStream();
    const auto stream_id = fixture.scheduler.reference_stream_state_.stream_id;
    ASSERT_TRUE(fixture.scheduler.reference_stream_state_.valid);
    auto ack = std::make_shared<Ack>();
    ack->stream_id = stream_id;
    ack->last_applied_sequence = fixture.scheduler.reference_stream_state_.sequence;
    ack->consumer_status = Ack::STATUS_APPLIED;
    fixture.scheduler.acknowledgeReferenceStream(ack);

    std::promise<void> entered;
    auto entered_future = entered.get_future();
    std::atomic<bool> entered_once{false};
    std::promise<void> release;
    const auto released = release.get_future().share();
    fixture.scheduler.reference_callback_struct_->set(
        [&entered, &entered_once, released, tracking, hover_by_object](
            const iii_drone::control::State & state) {
            if (!entered_once.exchange(true)) entered.set_value();
            released.wait();
            tracking->Fail("fault in already entered object fallback");
            return hover_by_object->GetReference(state);
        }, object->action_name(), execution, kRequestB);
    const auto fallback = fixture.scheduler.reference_callback_struct_->snapshot();
    std::thread entered_call([&] {
        (void)fallback.callback(fixture.awareness->GetState());
    });
    const bool entered_before_handoff =
        entered_future.wait_for(std::chrono::seconds(2)) == std::future_status::ready;
    if (!entered_before_handoff) {
        release.set_value();
        entered_call.join();
        FAIL() << "the fallback did not enter before successor preparation";
    }
    AcceptedHoverByObjectGoal hover_goal("failed_fallback_next_hover");
    const auto hover_handle = hover_goal.accept(kRequestC, target);
    if (!hover_handle) {
        release.set_value();
        entered_call.join();
        FAIL() << "the successor action was not accepted";
    }
    fixture.scheduler.current_maneuver_ =
        iii_drone::control::maneuver::Maneuver::FromGoalHandle<
            AcceptedHoverByObjectGoal::Action>(hover_handle);
    fixture.scheduler.progressScheduler();
    EXPECT_FALSE(fixture.scheduler.current_maneuver_.Load().started());
    EXPECT_EQ(fixture.scheduler.current_reference_execution_id_.Load(), execution);
    EXPECT_FALSE(fallback.lease->drained());
    release.set_value();
    entered_call.join();
    ASSERT_TRUE(tracking->failed());
    // The normal consumer is still applying the predecessor while its stop
    // begins. Keep this fixture on the fresh-ACK path; expiry has its own test.
    fixture.scheduler.acknowledgeReferenceStream(ack);
    fixture.scheduler.progressScheduler();
    EXPECT_EQ(fixture.scheduler.current_maneuver_.Load().maneuver_type(),
        iii_drone::control::maneuver::MANEUVER_TYPE_HOVER_BY_OBJECT);
    EXPECT_FALSE(fixture.scheduler.current_maneuver_.Load().started());
    EXPECT_EQ(fixture.scheduler.current_reference_execution_id_.Load(), execution);
    EXPECT_TRUE(hover_by_object->RetainsTrackedSource(
        fixture.scheduler.reference_callback_struct_->snapshot()));
    // The failed predecessor still owns an executable finite stop. A rest
    // must be emitted and actually acknowledged before rejecting the new goal.
    fixture.scheduler.publishReferenceStream();
    const iii_drone::control::KinematicStopTrajectory stop(
        seed, iii_drone::control::maneuver::ObjectTrackingSession::Limits{}
            .cancellation_config.limits);
    std::this_thread::sleep_for(std::chrono::duration<double>(
        stop.durationS() + 0.02));
    fixture.scheduler.publishReferenceStream();
    const auto rest = hover_by_object->TrackedFailureRest(
        fixture.scheduler.reference_callback_struct_->snapshot());
    ASSERT_TRUE(rest);
    EXPECT_LT(rest->velocity().norm(), 1.0e-5);
    ack->last_applied_sequence = fixture.scheduler.reference_stream_state_.sequence;
    fixture.scheduler.acknowledgeReferenceStream(ack);
    EXPECT_TRUE(fixture.scheduler.appliedFiniteRestReference(kRequestB, *rest));
    fixture.scheduler.progressScheduler();
    EXPECT_EQ(fixture.scheduler.current_maneuver_.Load().maneuver_type(),
        iii_drone::control::maneuver::MANEUVER_TYPE_NONE);
    EXPECT_EQ(fixture.scheduler.current_reference_execution_id_.Load(), execution);
    EXPECT_EQ(fixture.scheduler.reference_callback_struct_->snapshot().request_identity,
        kRequestB);
    EXPECT_TRUE(fixture.scheduler.reference_stream_state_.valid);
    EXPECT_EQ(fixture.scheduler.reference_stream_state_.stream_id, stream_id);
    EXPECT_FALSE(fixture.scheduler.current_maneuver_.Load().started());
    const auto sequence_after_rejection = fixture.scheduler.reference_stream_state_.sequence;
    const auto execution_after_rejection =
        fixture.scheduler.current_reference_execution_id_.Load();
    const auto retained_binding = fixture.scheduler.reference_callback_struct_->snapshot();
    EXPECT_TRUE(retained_binding.lease);
    EXPECT_FALSE(retained_binding.lease->quiescing());
    EXPECT_TRUE(fixture.scheduler.maneuver_server_get_reference_callback_still_registered_.Load());
    EXPECT_TRUE(fixture.scheduler.currentReferenceValid(retained_binding));
    EXPECT_FALSE(fixture.scheduler.reference_stream_state_.paused);
    fixture.scheduler.publishReferenceStream();
    EXPECT_GT(fixture.scheduler.reference_stream_state_.sequence,
        sequence_after_rejection)
        << "the certified predecessor rest must keep publishing after successor rejection";
    EXPECT_EQ(fixture.scheduler.reference_stream_state_.stream_id, stream_id);
    EXPECT_EQ(fixture.scheduler.current_reference_execution_id_.Load(),
        execution_after_rejection);
    EXPECT_LT((fixture.scheduler.reference_stream_state_.latest_reference.position() -
        rest->position()).norm(), 1.0e-6);
    object->Stop();
    hover_by_object->Stop();
    fixture.scheduler.registered_maneuvers_.erase(
        iii_drone::control::maneuver::MANEUVER_TYPE_FLY_TO_OBJECT);
    fixture.scheduler.registered_maneuvers_.erase(
        iii_drone::control::maneuver::MANEUVER_TYPE_HOVER_BY_OBJECT);
}

TEST(ManeuverReferenceClientTransaction, ExpiredEnteredObjectFallbackRejectsPendingHover) {
    RclcppContext context;
    for (const bool failed_rest : {false, true}) {
        SCOPED_TRACE(failed_rest ? "failed certified rest" : "healthy planner");
        TerminalCompletionFixture fixture(
            failed_rest ? "expired_failed_fallback" : "expired_planner_fallback",
            1000, "/control/maneuver_controller");
        fixture.hover->Update(Reference(fixture.hold->lastCommand().position(), 0.0));
        fixture.awareness->vehicle_odometry_adapter_history_ = std::make_shared<
            iii_drone::utils::History<VehicleOdometryAdapter>>(2);
        fixture.awareness->vehicle_odometry_adapter_history_->Store(stationaryVehicleState());
        fixture.awareness->powerline_adapter_history_ = std::make_shared<
            iii_drone::utils::History<iii_drone::adapters::PowerlineAdapter>>(1);
        const auto stamp = fixture.node.now();
        const point_t line_position(0.0F, 0.0F, 1.5F);
        const auto orientation = iii_drone::math::eulToQuat(
            iii_drone::types::euler_angles_t::Zero());
        const iii_drone::adapters::SingleLineAdapter line(
            stamp, "world", 1, line_position, line_position, orientation, true);
        fixture.awareness->powerline_adapter_history_->Store(
            iii_drone::adapters::PowerlineAdapter(stamp, {line},
                iii_drone::types::createPlane(line_position, vector_t::UnitX())));
        geometry_msgs::msg::TransformStamped world_to_drone;
        world_to_drone.header.stamp = stamp;
        world_to_drone.header.frame_id = "world";
        world_to_drone.child_frame_id = "drone";
        world_to_drone.transform.rotation.w = 1.0;
        ASSERT_TRUE(fixture.awareness->tf_buffer()->setTransform(
            world_to_drone, "expired_entered_fallback", true));
        iii_drone::types::transform_matrix_t target_transform =
            iii_drone::types::transform_matrix_t::Identity();
        target_transform(2, 3) = 1.5F;
        const iii_drone::adapters::TargetAdapter target(
            iii_drone::adapters::TARGET_TYPE_CABLE, 1, "world", target_transform);
        iii_drone::adapters::CombinedDroneAwarenessAdapter awareness;
        awareness.armed() = true;
        awareness.offboard() = true;
        awareness.drone_location() = iii_drone::adapters::DRONE_LOCATION_IN_FLIGHT;
        awareness.state() = fixture.awareness->GetState();
        *fixture.awareness->combined_drone_awareness_adapter_ = awareness;
        fixture.awareness->SetTarget(target);

        fixture.scheduler.Start();
        fixture.scheduler.maneuver_execution_timer_->cancel();
        auto hover_by_object = std::make_shared<
            iii_drone::control::maneuver::HoverByObjectManeuverServer>(
                &fixture.node, fixture.awareness, "hover_by_object", 1, 1, false, 1.0);
        fixture.scheduler.RegisterManeuverServer(
            iii_drone::control::maneuver::MANEUVER_TYPE_HOVER_BY_OBJECT,
            hover_by_object);
        auto object = std::make_shared<
            iii_drone::control::maneuver::FlyToObjectManeuverServer>(
                &fixture.node, fixture.awareness, "fly_to_object", 1, 1,
                fixture.config, nullptr);
        fixture.scheduler.RegisterManeuverServer(
            iii_drone::control::maneuver::MANEUVER_TYPE_FLY_TO_OBJECT, object);
        const Reference seed = fixture.hold->lastCommand();
        fixture.scheduler.beginReferenceExecution(object->action_name(), kRequestB, seed);
        const auto execution = fixture.scheduler.current_reference_execution_id_.Load();
        std::promise<void> entered;
        auto entered_future = entered.get_future();
        std::atomic<bool> entered_once{false};
        std::promise<void> release;
        const auto released = release.get_future().share();
        auto tracking = std::make_shared<
            iii_drone::control::maneuver::ObjectTrackingSession>(
                [&entered, &entered_once, released, failed_rest](
                    const Reference & start, const Reference &, bool) {
                    if (!failed_rest && !entered_once.exchange(true)) {
                        entered.set_value();
                        released.wait();  // Inside the actual session planner mutex.
                    }
                    return start;
                }, seed, kRequestB, execution, fixture.node.now(), 0.0,
                iii_drone::control::maneuver::ObjectTrackingSession::Limits{});
        ASSERT_TRUE(hover_by_object->UpdateTracked(
            target, tracking, kRequestB, execution, 0.0));
        fixture.scheduler.reference_callback_struct_->set(
            [tracking](const iii_drone::control::State &) {
                return tracking->lastCommand();
            }, object->action_name(), execution, kRequestB);
        fixture.scheduler.maneuver_server_get_reference_callback_still_registered_ = true;
        fixture.scheduler.publishReferenceStream();
        const auto stream_id = fixture.scheduler.reference_stream_state_.stream_id;
        auto ack = std::make_shared<Ack>();
        ack->stream_id = stream_id;
        ack->last_applied_sequence = fixture.scheduler.reference_stream_state_.sequence;
        ack->consumer_status = Ack::STATUS_APPLIED;
        fixture.scheduler.acknowledgeReferenceStream(ack);

        std::atomic<int> fallback_entries{0};
        std::promise<void> second_entered;
        auto second_entered_future = second_entered.get_future();
        fixture.scheduler.reference_callback_struct_->set(
            [&, failed_rest](const iii_drone::control::State & state) {
                if (++fallback_entries == 2) second_entered.set_value();
                if (failed_rest && !entered_once.exchange(true)) {
                    const auto begin = fixture.node.now() -
                        rclcpp::Duration::from_seconds(1.0);
                    (void)tracking->FailureReference(begin, "entered fallback failed");
                    (void)tracking->FailureReference(
                        fixture.node.now(), "entered fallback failed");
                    entered.set_value();
                    released.wait();
                }
                return hover_by_object->GetReference(state);
            }, object->action_name(), execution, kRequestB);
        const auto fallback = fixture.scheduler.reference_callback_struct_->snapshot();
        std::thread entered_call([&] {
            try {
                (void)fallback.callback(fixture.awareness->GetState());
            } catch (const std::exception & error) {
                ADD_FAILURE() << "entered fallback threw: " << error.what();
            }
        });
        const bool did_enter =
            entered_future.wait_for(std::chrono::seconds(2)) == std::future_status::ready;
        if (!did_enter) {
            release.set_value();
            entered_call.join();
            hover_by_object->Stop();
            object->Stop();
            FAIL() << "the real fallback did not enter its planner/stop path";
        }
        std::thread second_call;
        if (!failed_rest) {
            second_call = std::thread([&] {
                try {
                    (void)fallback.callback(fixture.awareness->GetState());
                } catch (const std::exception & error) {
                    ADD_FAILURE() << "second entered fallback threw: " << error.what();
                }
            });
            const bool two_entered =
                second_entered_future.wait_for(std::chrono::seconds(2)) ==
                std::future_status::ready;
            if (!two_entered) {
                release.set_value();
                entered_call.join();
                second_call.join();
                object->Stop();
                hover_by_object->Stop();
                FAIL() << "the second copied fallback never entered its lease";
            }
        }
        AcceptedHoverByObjectGoal hover_goal("expired_entered_fallback_hover");
        const auto hover_handle = hover_goal.accept(kRequestC, target);
        if (!hover_handle) {
            release.set_value();
            entered_call.join();
            if (second_call.joinable()) second_call.join();
            hover_by_object->Stop();
            object->Stop();
            FAIL() << "the pending HBO action was not accepted";
        }
        fixture.scheduler.current_maneuver_ =
            iii_drone::control::maneuver::Maneuver::FromGoalHandle<
                AcceptedHoverByObjectGoal::Action>(hover_handle);
        if (!failed_rest) {
            const auto sequence_before = fixture.scheduler.reference_stream_state_.sequence;
            std::promise<void> first_tick_done;
            auto first_tick_future = first_tick_done.get_future();
            std::thread first_tick([&] {
                fixture.scheduler.maneuverExecutionTimerCallback();
                first_tick_done.set_value();
            });
            const bool first_tick_completed_while_entered =
                first_tick_future.wait_for(std::chrono::milliseconds(100)) ==
                std::future_status::ready;
            if (!first_tick_completed_while_entered) {
                // A RED publication may block on the planner mutex. Drain it
                // before joining so this regression fails rather than hangs.
                release.set_value();
                entered_call.join();
                second_call.join();
                first_tick.join();
                ADD_FAILURE() << "the real timer blocked in publication while the "
                    "predecessor planner was entered";
                object->Stop();
                hover_by_object->Stop();
                fixture.scheduler.registered_maneuvers_.erase(
                    iii_drone::control::maneuver::MANEUVER_TYPE_FLY_TO_OBJECT);
                fixture.scheduler.registered_maneuvers_.erase(
                    iii_drone::control::maneuver::MANEUVER_TYPE_HOVER_BY_OBJECT);
                continue;
            }
            first_tick.join();
            EXPECT_EQ(fixture.scheduler.current_maneuver_.Load().maneuver_type(),
                iii_drone::control::maneuver::MANEUVER_TYPE_HOVER_BY_OBJECT);
            EXPECT_FALSE(fixture.scheduler.current_maneuver_.Load().started());
            EXPECT_EQ(fixture.scheduler.reference_stream_state_.sequence, sequence_before)
                << "an entered predecessor cannot publish a superseded sample";
            EXPECT_EQ(fixture.scheduler.reference_stream_state_.stream_id, stream_id);
            EXPECT_EQ(fixture.scheduler.current_reference_execution_id_.Load(), execution);
        }
        {
            std::lock_guard<std::mutex> lock(fixture.scheduler.reference_stream_mutex_);
            fixture.scheduler.reference_stream_state_.last_ack =
                std::chrono::steady_clock::now() - std::chrono::milliseconds(300);
        }
        const auto sequence_before_expiry =
            fixture.scheduler.reference_stream_state_.sequence;
        std::promise<void> tick_done;
        auto tick_future = tick_done.get_future();
        std::thread tick([&] {
            if (failed_rest) {
                fixture.scheduler.progressScheduler();
            } else {
                fixture.scheduler.maneuverExecutionTimerCallback();
            }
            tick_done.set_value();
        });
        const bool tick_completed_while_entered =
            tick_future.wait_for(std::chrono::milliseconds(100)) ==
            std::future_status::ready;
        // Always release the callback before joining, including on RED.
        release.set_value();
        entered_call.join();
        if (second_call.joinable()) second_call.join();
        tick.join();
        EXPECT_TRUE(tick_completed_while_entered)
            << "a blocked planner must not block the scheduler's 250 ms ACK deadline";
        EXPECT_EQ(fixture.scheduler.current_maneuver_.Load().maneuver_type(),
            iii_drone::control::maneuver::MANEUVER_TYPE_NONE)
            << "the expired pending HBO must be rejected, not left waiting";
        EXPECT_EQ(fixture.scheduler.current_reference_execution_id_.Load(), execution);
        EXPECT_EQ(fixture.scheduler.reference_callback_struct_->snapshot().request_identity,
            kRequestB);
        EXPECT_EQ(fixture.scheduler.reference_stream_state_.stream_id, stream_id);
        EXPECT_TRUE(fixture.scheduler.reference_stream_state_.valid);
        if (!failed_rest) {
            EXPECT_EQ(fixture.scheduler.reference_stream_state_.sequence,
                sequence_before_expiry)
                << "the expired successor cannot publish its own generation or "
                    "a stale predecessor sample while the planner is entered";
        }
        const auto previous_sequence = fixture.scheduler.reference_stream_state_.sequence;
        fixture.scheduler.publishReferenceStream();
        EXPECT_GT(fixture.scheduler.reference_stream_state_.sequence, previous_sequence)
            << "the old source must remain executable after the entered call drains";
        EXPECT_TRUE(fixture.scheduler.reference_stream_state_.latest_reference.position().allFinite());
        EXPECT_TRUE(fixture.scheduler.reference_stream_state_.latest_reference.velocity().allFinite());
        EXPECT_TRUE(fixture.scheduler.reference_stream_state_.latest_reference.acceleration().allFinite());
        object->Stop();
        hover_by_object->Stop();
        fixture.scheduler.registered_maneuvers_.erase(
            iii_drone::control::maneuver::MANEUVER_TYPE_FLY_TO_OBJECT);
        fixture.scheduler.registered_maneuvers_.erase(
            iii_drone::control::maneuver::MANEUVER_TYPE_HOVER_BY_OBJECT);
    }
}

TEST(ManeuverReferenceClientTransaction, SameGenerationReplacementDropsNativeAndLegacyCandidates) {
    RclcppContext context;
    TerminalCompletionFixture fixture(
        "same_generation_replacement", 1000, "/control/maneuver_controller");
    fixture.finishWithoutSchedulerTick(
        iii_drone::control::maneuver::MANEUVER_TYPE_CABLE_AWARE_FLY_TO_POSITION,
        "cable_aware_fly_to_position", true);
    fixture.scheduler.Start();
    fixture.scheduler.maneuver_execution_timer_->cancel();
    const auto original = fixture.scheduler.reference_callback_struct_->snapshot();
    ASSERT_TRUE(original.callback);
    const auto sequence_before = fixture.scheduler.reference_stream_state_.sequence;

    std::promise<void> native_entered;
    auto native_entered_future = native_entered.get_future();
    std::promise<void> native_release;
    const auto native_release_future = native_release.get_future().share();
    fixture.scheduler.reference_callback_struct_->set(
        [&](const iii_drone::control::State &) {
            native_entered.set_value();
            native_release_future.wait();
            return fixture.hold->lastCommand();
        }, original.reference_provider_name, original.execution_id,
        original.request_identity);
    const auto native_binding = fixture.scheduler.reference_callback_struct_->snapshot();
    std::thread native([&] { fixture.scheduler.publishReferenceStream(); });
    const bool entered_native =
        native_entered_future.wait_for(std::chrono::seconds(2)) ==
        std::future_status::ready;
    if (!entered_native) {
        native_release.set_value();
        native.join();
        FAIL() << "native publisher did not enter its copied callback";
    }
    fixture.scheduler.reference_callback_struct_->set(
        [hold = fixture.hold](const iii_drone::control::State &) {
            return hold->lastCommand();
        }, native_binding.reference_provider_name, native_binding.execution_id,
        native_binding.request_identity);
    native_release.set_value();
    native.join();
    EXPECT_EQ(fixture.scheduler.reference_stream_state_.sequence, sequence_before)
        << "the replaced same-generation callback allocated a native sequence";

    std::promise<void> legacy_entered;
    auto legacy_entered_future = legacy_entered.get_future();
    std::promise<void> legacy_release;
    const auto legacy_release_future = legacy_release.get_future().share();
    fixture.scheduler.reference_callback_struct_->set(
        [&](const iii_drone::control::State &) {
            legacy_entered.set_value();
            legacy_release_future.wait();
            return fixture.hold->lastCommand();
        }, original.reference_provider_name, original.execution_id,
        original.request_identity);
    auto request = std::make_shared<iii_drone_interfaces::srv::GetReference::Request>();
    auto response = std::make_shared<iii_drone_interfaces::srv::GetReference::Response>();
    std::thread legacy([&] {
        fixture.scheduler.getReferenceServiceCallback(request, response);
    });
    const bool entered_legacy =
        legacy_entered_future.wait_for(std::chrono::seconds(2)) ==
        std::future_status::ready;
    if (!entered_legacy) {
        legacy_release.set_value();
        legacy.join();
        FAIL() << "legacy service did not enter its copied callback";
    }
    fixture.scheduler.reference_callback_struct_->set(
        [hold = fixture.hold](const iii_drone::control::State &) {
            return hold->lastCommand();
        }, original.reference_provider_name, original.execution_id,
        original.request_identity);
    legacy_release.set_value();
    legacy.join();
    EXPECT_FALSE(response->is_valid)
        << "a replaced same-generation callable returned a valid legacy response";
    EXPECT_EQ(fixture.scheduler.reference_stream_state_.sequence, sequence_before);
}

TEST(ManeuverReferenceClientTransaction, TrackedObjectStopSeedsLandingAfterAppliedRest) {
    RclcppContext context;
    TerminalCompletionFixture fixture(
        "object_to_landing", 1000, "/control/maneuver_controller");
    fixture.awareness->vehicle_odometry_adapter_history_ = std::make_shared<
        iii_drone::utils::History<VehicleOdometryAdapter>>(2);
    fixture.awareness->vehicle_odometry_adapter_history_->Store(stationaryVehicleState());
    fixture.awareness->powerline_adapter_history_ = std::make_shared<
        iii_drone::utils::History<iii_drone::adapters::PowerlineAdapter>>(1);
    const auto line_stamp = fixture.node.now();
    const point_t line_position(0.0F, 0.0F, 1.5F);
    const iii_drone::adapters::SingleLineAdapter line(
        line_stamp, "world", 1, line_position, line_position,
        iii_drone::math::eulToQuat(iii_drone::types::euler_angles_t::Zero()), true);
    fixture.awareness->powerline_adapter_history_->Store(
        iii_drone::adapters::PowerlineAdapter(line_stamp, {line},
            iii_drone::types::createPlane(line_position, vector_t::UnitX())));
    for (const auto & child : {"drone", "gripper"}) {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = line_stamp;
        transform.header.frame_id = "world";
        transform.child_frame_id = child;
        transform.transform.rotation.w = 1.0;
        ASSERT_TRUE(fixture.awareness->tf_buffer()->setTransform(
            transform, "object_landing_test", true));
    }
    fixture.finishWithoutSchedulerTick(
        iii_drone::control::maneuver::MANEUVER_TYPE_CABLE_AWARE_FLY_TO_POSITION,
        "cable_aware_fly_to_position", true);
    ClientFixture consumer("object_to_landing_consumer", 3000, 0.5);
    ASSERT_TRUE(consumer.client.BeginManeuverGoalHandoff(kRequestA));
    ASSERT_TRUE(consumer.client.ConfirmManeuverGoalHandoff(kRequestA));
    auto predecessor = stream(consumer.node, "cwa:terminal", 1,
        fixture.hold->lastCommand(), kRequestA);
    predecessor->terminal_hold_active = true;
    consumer.client.receiveReferenceStream(predecessor);
    const auto predecessor_command = consumer.client.GetReference(0.02, [] {});
    ASSERT_TRUE(predecessor_command.position().allFinite());
    fixture.hover->Update(Reference(predecessor_command.position(), 0.0));

    fixture.scheduler.Start();
    fixture.scheduler.maneuver_execution_timer_->cancel();
    auto object = std::make_shared<
        iii_drone::control::maneuver::HoverByObjectManeuverServer>(
            &fixture.node, fixture.awareness, "hover_by_object", 1, 1, false, 1.0);
    auto generator_service = fixture.node.create_service<
        iii_drone_interfaces::srv::ComputeReferenceTrajectory>(
            "/control/trajectory_generator/compute_reference_trajectory",
            [](const std::shared_ptr<iii_drone_interfaces::srv::ComputeReferenceTrajectory::Request>,
               std::shared_ptr<iii_drone_interfaces::srv::ComputeReferenceTrajectory::Response>) {});
    auto generator_group = fixture.node.create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    auto generator_client = std::make_shared<iii_drone::control::TrajectoryGeneratorClient>(
        &fixture.node, fixture.config, generator_group);
    auto landing = std::make_shared<
        iii_drone::control::maneuver::CableLandingManeuverServer>(
            &fixture.node, fixture.awareness, "cable_landing", 1, 1,
            fixture.config, generator_client);
    fixture.scheduler.RegisterManeuverServer(
        iii_drone::control::maneuver::MANEUVER_TYPE_HOVER_BY_OBJECT, object);
    fixture.scheduler.RegisterManeuverServer(
        iii_drone::control::maneuver::MANEUVER_TYPE_CABLE_LANDING, landing);

    const Reference moving(
        point_t(0.24F, 0.0F, 0.0F), 0.0,
        vector_t(0.2F, 0.0F, 0.0F), 0.0,
        vector_t::Zero(), 0.0, fixture.node.now());
    fixture.scheduler.beginReferenceExecution(object->action_name(), kRequestB, moving);
    const auto source_execution = fixture.scheduler.current_reference_execution_id_.Load();
    auto session = std::make_shared<
        iii_drone::control::maneuver::ObjectTrackingSession>(
            [moving](const Reference &, const Reference &, bool) { return moving; },
            moving, kRequestB, source_execution, fixture.node.now(), 0.0,
            iii_drone::control::maneuver::ObjectTrackingSession::Limits{});
    object->object_tracking_session_ = session;
    object->object_owner_request_identity_ = kRequestB;
    object->object_owner_execution_id_ = source_execution;
    fixture.scheduler.maneuver_server_get_reference_callback_still_registered_ = true;

    ASSERT_TRUE(consumer.client.BeginManeuverGoalHandoff(kRequestB));
    ASSERT_TRUE(consumer.client.ConfirmManeuverGoalHandoff(kRequestB));
    consumer.executor.add_node(fixture.node.get_node_base_interface());
    for (int attempt = 0; attempt < 20 &&
         fixture.scheduler.reference_stream_publisher_->get_subscription_count() == 0;
         ++attempt) {
        consumer.executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    fixture.scheduler.publishReferenceStream();
    for (int attempt = 0; attempt < 20; ++attempt) {
        consumer.executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    (void)consumer.client.GetReference(0.02, [] {});
    for (int attempt = 0; attempt < 20 &&
         !fixture.scheduler.reference_stream_state_.ack_seen; ++attempt) {
        consumer.executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    ASSERT_EQ(consumer.client.active_request_identity_, kRequestB);
    ASSERT_TRUE(fixture.scheduler.reference_stream_state_.ack_seen);
    ASSERT_TRUE(consumer.client.currentAppliedObjectTrackingStream(kRequestB));
    ASSERT_TRUE(consumer.client.CompleteManeuverGoalHandoff(kRequestB));
    EXPECT_FALSE(consumer.client.object_stop_);
    EXPECT_EQ(consumer.client.active_request_identity_, kRequestB);
    std::promise<bool> source_callback_result;
    auto source_callback_future = source_callback_result.get_future();
    std::thread source_callback([&] {
        const bool acquired = object->reference_callback_token_->Acquire(2000);
        if (acquired) {
            object->reference_callback_token_->resource().set(
                [object](const iii_drone::control::State & state) {
                    return object->GetReference(state);
                }, object->action_name(), source_execution, kRequestB);
            object->reference_callback_token_->Release();
        }
        source_callback_result.set_value(acquired);
    });
    for (int attempt = 0; attempt < 200 &&
         !fixture.scheduler.reference_callback_token_.has_requested_token(
             object->action_name()); ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    const bool source_requested = fixture.scheduler.reference_callback_token_.has_requested_token(
        object->action_name());
    if (source_requested) fixture.scheduler.reference_callback_token_.Give(object->action_name());
    EXPECT_TRUE(source_callback_future.get());
    source_callback.join();
    ASSERT_TRUE(source_requested);
    AcceptedLandingGoal landing_goal("object_to_landing_goal");
    const auto handle = landing_goal.accept(kRequestC);
    ASSERT_TRUE(handle);
    fixture.scheduler.current_maneuver_ =
        iii_drone::control::maneuver::Maneuver::FromGoalHandle<
            AcceptedLandingGoal::Action>(handle);

    fixture.scheduler.progressScheduler();
    ASSERT_TRUE(session->transitionStopping());
    EXPECT_FALSE(fixture.scheduler.current_maneuver_.Load().started());
    EXPECT_FALSE(object->TrackedTransitionRest(
        fixture.scheduler.reference_callback_struct_->snapshot()).has_value());
    ASSERT_TRUE(consumer.client.BeginManeuverGoalHandoff(kRequestC));
    ASSERT_TRUE(consumer.client.ConfirmManeuverGoalHandoff(kRequestC));

    bool successor_started = false;
    bool failed = false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    auto next_consumer = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() < deadline && !successor_started) {
        fixture.scheduler.publishReferenceStream();
        consumer.executor.spin_some();
        if (std::chrono::steady_clock::now() >= next_consumer) {
            (void)consumer.client.GetReference(0.02, [&failed] { failed = true; });
            next_consumer += std::chrono::milliseconds(200);
        }
        fixture.scheduler.progressScheduler();
        successor_started = fixture.scheduler.current_maneuver_.Load().started();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    EXPECT_FALSE(failed);
    ASSERT_TRUE(successor_started);
    ASSERT_TRUE(session->transitionRest());
    const Reference rest = session->lastCommand();
    EXPECT_LT(rest.velocity().norm(), 1.0e-5);
    EXPECT_LT(rest.acceleration().norm(), 1.0e-5);
    const auto successor_binding = fixture.scheduler.reference_callback_struct_->snapshot();
    ASSERT_TRUE(successor_binding.callback);
    const Reference staged = successor_binding.callback(fixture.awareness->GetState());
    EXPECT_LT((staged.position() - rest.position()).norm(), 1.0e-5);
    EXPECT_LT((staged.velocity() - rest.velocity()).norm(), 1.0e-5);
    EXPECT_FALSE(object->HasTrackedSession());
    EXPECT_EQ(fixture.scheduler.reference_callback_struct_->snapshot().request_identity,
        kRequestC);
    std::promise<bool> token_result;
    auto token_future = token_result.get_future();
    std::string landing_failure;
    std::optional<Reference> first_line_pid;
    std::thread token_thread([&] {
        const bool acquired = landing->reference_callback_token_->Acquire(1000);
        if (!acquired) {
            token_result.set_value(false);
            return;
        }
        try {
            auto landing_maneuver = fixture.scheduler.current_maneuver_.Load();
            landing->startExecution(landing_maneuver);
            const auto first = landing->computeReference(fixture.awareness->GetState());
            first_line_pid = first;
            landing->reference_callback_token_->resource().set(
                [first, &fixture](const iii_drone::control::State &) {
                    return first.CopyWithNewStamp(fixture.node.now());
                }, landing->action_name(), successor_binding.execution_id, kRequestC);
            landing_failure = "seed=" + std::to_string(
                landing->object_transition_start_reference_.has_value()) +
                " initialized=" + std::to_string(landing->line_pid_initialized_) +
                " ascent=" + std::to_string(first.velocity()(2));
            token_result.set_value(
                landing->object_transition_start_reference_.has_value() &&
                landing->line_pid_initialized_ &&
                std::abs(first.velocity()(2) - 0.20F) < 1.0e-5);
        } catch (const std::exception & error) {
            landing_failure = error.what();
            token_result.set_value(false);
        }
        landing->reference_callback_token_->Release();
    });
    for (int attempt = 0; attempt < 200 &&
         !fixture.scheduler.reference_callback_token_.has_requested_token(
             landing->action_name()); ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    fixture.scheduler.progressScheduler();
    EXPECT_TRUE(token_future.get()) << landing_failure;
    token_thread.join();
    ASSERT_TRUE(first_line_pid);
    fixture.scheduler.publishReferenceStream();
    for (int attempt = 0; attempt < 30; ++attempt) {
        consumer.executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    bool landing_consumer_failed = false;
    const Reference consumed_landing = consumer.client.GetReference(
        0.02, [&landing_consumer_failed] { landing_consumer_failed = true; });
    EXPECT_FALSE(landing_consumer_failed);
    EXPECT_FALSE(consumer.client.reference_safety_guard_->faultLatched());
    EXPECT_NEAR(consumed_landing.velocity()(2), 0.20, 1.0e-5);
    EXPECT_NEAR(consumed_landing.position()(0), rest.position()(0), 1.0e-4);
    EXPECT_TRUE(std::isnan(consumed_landing.position()(2)));
    consumer.executor.remove_node(fixture.node.get_node_base_interface());
    object->Stop();
    landing->Stop();
    fixture.scheduler.registered_maneuvers_.erase(
        iii_drone::control::maneuver::MANEUVER_TYPE_HOVER_BY_OBJECT);
    fixture.scheduler.registered_maneuvers_.erase(
        iii_drone::control::maneuver::MANEUVER_TYPE_CABLE_LANDING);
}

TEST(ManeuverReferenceClientTransaction, NegativeObjectGenerationAckCannotProveOwnership) {
    RclcppContext context;
    TerminalCompletionFixture fixture("negative_object_ack");
    auto server = std::make_shared<iii_drone::control::maneuver::FlyToObjectManeuverServer>(
        &fixture.node, fixture.awareness, "fly_to_object_negative_ack", 1, 1,
        fixture.config, nullptr);
    fixture.scheduler.registered_maneuvers_[
        iii_drone::control::maneuver::MANEUVER_TYPE_FLY_TO_OBJECT] = server;
    iii_drone::control::maneuver::Maneuver maneuver(
        iii_drone::control::maneuver::MANEUVER_TYPE_FLY_TO_OBJECT,
        rclcpp_action::GoalUUID{});
    maneuver.request_identity_ = kRequestB;
    maneuver.started_ = true;
    fixture.scheduler.current_maneuver_ = maneuver;
    fixture.scheduler.beginReferenceExecution(
        server->action_name(), kRequestB, initializationHold());
    fixture.scheduler.publishReferenceStream();
    ASSERT_TRUE(fixture.scheduler.reference_stream_state_.valid);
    auto rejected = std::make_shared<Ack>();
    rejected->stream_id = fixture.scheduler.reference_stream_state_.stream_id;
    rejected->last_applied_sequence = fixture.scheduler.reference_stream_state_.sequence;
    rejected->consumer_status = Ack::STATUS_PAUSING;
    fixture.scheduler.acknowledgeReferenceStream(rejected);
    EXPECT_TRUE(fixture.scheduler.reference_stream_state_.ack_seen);
    EXPECT_TRUE(fixture.scheduler.reference_stream_state_.paused);
    EXPECT_FALSE(fixture.scheduler.firstObjectReferenceApplied(kRequestB));
}

TEST(ManeuverReferenceClientTransaction, FreshPublicationCannotRefreshOldObjectApplication) {
    RclcppContext context;
    ClientFixture fixture("stale_object_application");
    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestB));
    ASSERT_TRUE(fixture.client.ConfirmManeuverGoalHandoff(kRequestB));
    auto applied = stream(fixture.node, "object:owned", 1,
        finiteReference(0.24, 0.0), kRequestB);
    applied->object_tracking_active = true;
    fixture.client.receiveReferenceStream(applied);
    (void)fixture.client.GetReference(0.02, [] {});
    ASSERT_EQ(fixture.client.applied_object_tracking_sequence_, 1U);
    ASSERT_TRUE(fixture.client.currentAppliedObjectTrackingStream(kRequestB));

    // Publications can remain fresh after the consumer stops applying them.
    // The old APPLIED is still the same generation, but no longer timely.
    fixture.client.applied_object_tracking_at_ =
        std::chrono::steady_clock::now() - std::chrono::seconds(11);
    auto current = stream(fixture.node, "object:owned", 2,
        finiteReference(0.24, 0.0), kRequestB);
    current->object_tracking_active = true;
    fixture.client.receiveReferenceStream(current);
    EXPECT_FALSE(fixture.client.currentAppliedObjectTrackingStream(kRequestB));
}

TEST(ManeuverReferenceClientTransaction, EmptyObjectResultRequiresExactFreshAppliedOwner) {
    RclcppContext context;
    ClientFixture fixture("empty_object_result", 3000, 1.0);
    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestB));
    ASSERT_TRUE(fixture.client.ConfirmManeuverGoalHandoff(kRequestB));
    auto active = stream(fixture.node, "object:empty_result", 1,
        finiteReference(0.24, 0.20), kRequestB);
    active->object_tracking_active = true;
    fixture.client.receiveReferenceStream(active);
    EXPECT_FALSE(fixture.client.CompleteManeuverGoalHandoff(kRequestB));
    ASSERT_TRUE(fixture.client.pending_goal_handoff_);
    EXPECT_FALSE(fixture.client.object_stop_);

    (void)fixture.client.GetReference(0.02, [] {});
    ASSERT_FALSE(fixture.client.pending_goal_handoff_);
    ASSERT_TRUE(fixture.client.currentAppliedObjectTrackingStream(kRequestB));
    EXPECT_FALSE(fixture.client.CompleteManeuverGoalHandoff(kRequestA));
    EXPECT_TRUE(fixture.client.CompleteManeuverGoalHandoff(kRequestB));
    EXPECT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::MANEUVER);
    EXPECT_FALSE(fixture.client.object_stop_);

    fixture.client.applied_object_tracking_at_ =
        std::chrono::steady_clock::now() - std::chrono::seconds(11);
    auto newer = stream(fixture.node, "object:empty_result", 2,
        finiteReference(0.24, 0.20), kRequestB);
    newer->object_tracking_active = true;
    fixture.client.receiveReferenceStream(newer);
    EXPECT_FALSE(fixture.client.CompleteManeuverGoalHandoff(kRequestB));
    EXPECT_FALSE(fixture.client.object_stop_);
}

TEST(ManeuverReferenceClientTransaction, EmptyObjectResultKeepsOrdinaryAndPendingCleanup) {
    RclcppContext context;
    ClientFixture ordinary("ordinary_empty_object_result");
    ASSERT_TRUE(ordinary.client.BeginManeuverGoalHandoff(kRequestB));
    ASSERT_TRUE(ordinary.client.ConfirmManeuverGoalHandoff(kRequestB));
    ordinary.client.receiveReferenceStream(stream(ordinary.node,
        "ordinary:object", 1, finiteReference(0.0, 0.0), kRequestB));
    (void)ordinary.client.GetReference(0.02, [] {});
    ASSERT_TRUE(ordinary.client.CompleteManeuverGoalHandoff(kRequestB));
    EXPECT_EQ(ordinary.client.reference_mode_.Load(), ManeuverReferenceClient::HOVER);

    ClientFixture pending("pending_empty_object_result");
    ASSERT_TRUE(pending.client.BeginManeuverGoalHandoff(kRequestB));
    ASSERT_TRUE(pending.client.ConfirmManeuverGoalHandoff(kRequestB));
    pending.client.receiveReferenceStream(stream(pending.node,
        "ordinary:pending", 1, finiteReference(0.0, 0.0), kRequestB));
    (void)pending.client.GetReference(0.02, [] {});
    ASSERT_TRUE(pending.client.BeginManeuverGoalHandoff(kRequestC));
    EXPECT_FALSE(pending.client.CompleteManeuverGoalHandoff(kRequestB));
    ASSERT_TRUE(pending.client.pending_goal_handoff_);
    EXPECT_EQ(pending.client.pending_goal_handoff_->request_identity, kRequestC);
}

TEST(ManeuverReferenceClientTransaction, OwnedObjectExpiryWaitsForAppliedCertifiedRest) {
    RclcppContext context;
    ClientFixture fixture("object_owned_expiry", 3000, 0.5);
    ASSERT_TRUE(waitForAckSubscriber(fixture));
    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestB));
    ASSERT_TRUE(fixture.client.ConfirmManeuverGoalHandoff(kRequestB));
    auto active = stream(fixture.node, "object:stop", 1,
        finiteReference(0.24, 0.20), kRequestB);
    active->object_tracking_active = true;
    fixture.client.receiveReferenceStream(active);
    (void)fixture.client.GetReference(0.02, [] {});
    ASSERT_TRUE(observedAppliedAck(fixture, "object:stop", 1));
    ASSERT_TRUE(fixture.client.StopManeuverGoalHandoffAfterTimeout(kRequestB, 10000));
    const auto expiry = *fixture.client.stop_maneuver_timer_callback_;
    ASSERT_TRUE(expiry);
    expiry();
    EXPECT_EQ(fixture.client.reference_mode_.Load(),
        ManeuverReferenceClient::WAIT_FOR_MANEUVER_STOP);
    for (int attempt = 0; attempt < 20; ++attempt) {
        fixture.spinCallbacks();
        if (std::any_of(fixture.acknowledgements.begin(),
                fixture.acknowledgements.end(), [](const Ack & ack) {
                    return ack.consumer_status == Ack::STATUS_OBJECT_STOP_REQUESTED &&
                        ack.stream_id == "object:stop" && ack.last_applied_sequence == 1;
                })) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    EXPECT_TRUE(std::any_of(fixture.acknowledgements.begin(),
        fixture.acknowledgements.end(), [](const Ack & ack) {
            return ack.consumer_status == Ack::STATUS_OBJECT_STOP_REQUESTED &&
                ack.stream_id == "object:stop" && ack.last_applied_sequence == 1;
        }));

    auto stopping = stream(fixture.node, "object:stop", 2,
        finiteReference(0.26, 0.10), kRequestB);
    stopping->object_tracking_active = true;
    stopping->state = Stream::STATE_OBJECT_STOPPING;
    fixture.client.receiveReferenceStream(stopping);
    bool failed = false;
    const Reference mid = fixture.client.GetReference(0.02, [&] { failed = true; });
    EXPECT_FALSE(failed);
    EXPECT_NEAR(mid.position()(0), 0.26, 1.0e-5);
    ASSERT_TRUE(fixture.client.object_stop_);
    EXPECT_TRUE(fixture.client.object_stop_->admitted);
    EXPECT_EQ(fixture.client.reference_mode_.Load(),
        ManeuverReferenceClient::WAIT_FOR_MANEUVER_STOP);

    auto stopped = stream(fixture.node, "object:stop", 3,
        finiteReference(0.30, 0.0), kRequestB);
    stopped->object_tracking_active = true;
    stopped->state = Stream::STATE_OBJECT_STOPPED;
    fixture.client.receiveReferenceStream(stopped);
    const Reference rest = fixture.client.GetReference(0.02, [&] { failed = true; });
    EXPECT_FALSE(failed);
    EXPECT_NEAR(rest.position()(0), 0.30, 1.0e-5);
    EXPECT_NEAR(fixture.client.reference_.Load().position()(0), 0.30, 1.0e-5);
    EXPECT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::HOVER);
    EXPECT_FALSE(fixture.client.object_stop_);
    EXPECT_TRUE(observedAppliedAck(fixture, "object:stop", 3));
    // Local Hold must keep the exact stopped producer stream acknowledged. A
    // later positional goal still needs fresh proof of this same rest anchor.
    EXPECT_EQ(fixture.client.active_stream_id_, "object:stop");
    for (uint64_t sequence = 4; sequence <= 7; ++sequence) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        auto held = stream(fixture.node, "object:stop", sequence,
            finiteReference(0.30, 0.0), kRequestB);
        held->object_tracking_active = true;
        held->state = Stream::STATE_OBJECT_STOPPED;
        fixture.client.receiveReferenceStream(held);
        const Reference still = fixture.client.GetReference(0.20, [&] { failed = true; });
        EXPECT_FALSE(failed);
        EXPECT_NEAR(still.position()(0), 0.30, 1.0e-5);
        EXPECT_TRUE(observedAppliedAck(fixture, "object:stop", sequence));
    }
    auto drifting_rest = stream(fixture.node, "object:stop", 8,
        finiteReference(0.3005, 0.0), kRequestB);
    drifting_rest->object_tracking_active = true;
    drifting_rest->state = Stream::STATE_OBJECT_STOPPED;
    fixture.client.receiveReferenceStream(drifting_rest);
    const Reference unchanged_hold = fixture.client.GetReference(0.20, [&] { failed = true; });
    EXPECT_FALSE(failed);
    EXPECT_NEAR(unchanged_hold.position()(0), 0.30, 1.0e-5);
    EXPECT_EQ(fixture.client.last_applied_sequence_, 7U);
    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestC));
    ASSERT_TRUE(fixture.client.ConfirmManeuverGoalHandoff(kRequestC));
    auto drifting_predecessor = stream(fixture.node, "object:stop", 9,
        finiteReference(0.3005, 0.0), kRequestB);
    drifting_predecessor->object_tracking_active = true;
    drifting_predecessor->state = Stream::STATE_OBJECT_STOPPED;
    fixture.client.receiveReferenceStream(drifting_predecessor);
    const Reference unchanged_predecessor = fixture.client.GetReference(
        0.20, [&] { failed = true; });
    EXPECT_FALSE(failed);
    EXPECT_NEAR(unchanged_predecessor.position()(0), 0.30, 1.0e-5);
    EXPECT_EQ(fixture.client.last_applied_sequence_, 7U);
    auto predecessor_rest = stream(fixture.node, "object:stop", 10,
        finiteReference(0.30, 0.0), kRequestB);
    predecessor_rest->object_tracking_active = true;
    predecessor_rest->state = Stream::STATE_OBJECT_STOPPED;
    fixture.client.receiveReferenceStream(predecessor_rest);
    const Reference while_waiting = fixture.client.GetReference(0.20, [&] { failed = true; });
    EXPECT_FALSE(failed);
    EXPECT_NEAR(while_waiting.position()(0), 0.30, 1.0e-5);
    EXPECT_TRUE(observedAppliedAck(fixture, "object:stop", 10));
    EXPECT_EQ(fixture.client.reference_mode_.Load(),
        ManeuverReferenceClient::WAIT_FOR_MANEUVER_START);
    ASSERT_TRUE(fixture.client.CancelManeuverGoalHandoff(kRequestC));
    EXPECT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::HOVER);
    EXPECT_EQ(fixture.client.active_stream_id_, "object:stop");
    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestD));
    ASSERT_TRUE(fixture.client.ConfirmManeuverGoalHandoff(kRequestD));
    auto after_cancel_rest = stream(fixture.node, "object:stop", 11,
        finiteReference(0.30, 0.0), kRequestB);
    after_cancel_rest->object_tracking_active = true;
    after_cancel_rest->state = Stream::STATE_OBJECT_STOPPED;
    fixture.client.receiveReferenceStream(after_cancel_rest);
    (void)fixture.client.GetReference(0.20, [&] { failed = true; });
    EXPECT_TRUE(observedAppliedAck(fixture, "object:stop", 11));
    fixture.client.receiveReferenceStream(stream(fixture.node, "position:new", 1,
        finiteReference(0.30, 0.0), kRequestD));
    (void)fixture.client.GetReference(0.20, [&] { failed = true; });
    EXPECT_FALSE(failed);
    EXPECT_EQ(fixture.client.reference_mode_.Load(), ManeuverReferenceClient::MANEUVER);
    EXPECT_FALSE(fixture.client.object_stopped_hold_);
}

TEST(ManeuverReferenceClientTransaction, ExplicitControlResetFencesStoppedObjectHold) {
    RclcppContext context;
    ClientFixture fixture("object_stopped_reset");
    const uint64_t owner = fixture.client.AcquireReferenceControl();
    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestB));
    ASSERT_TRUE(fixture.client.ConfirmManeuverGoalHandoff(kRequestB));
    auto active = stream(fixture.node, "object:reset", 1,
        finiteReference(0.24, 0.20), kRequestB);
    active->object_tracking_active = true;
    fixture.client.receiveReferenceStream(active);
    (void)fixture.client.GetReference(0.02, [] {});
    ASSERT_TRUE(fixture.client.CancelManeuverGoalHandoff(kRequestB));
    auto stopping = stream(fixture.node, "object:reset", 2,
        finiteReference(0.26, 0.10), kRequestB);
    stopping->object_tracking_active = true;
    stopping->state = Stream::STATE_OBJECT_STOPPING;
    fixture.client.receiveReferenceStream(stopping);
    (void)fixture.client.GetReference(0.02, [] {});
    auto stopped = stream(fixture.node, "object:reset", 3,
        finiteReference(0.30, 0.0), kRequestB);
    stopped->object_tracking_active = true;
    stopped->state = Stream::STATE_OBJECT_STOPPED;
    fixture.client.receiveReferenceStream(stopped);
    (void)fixture.client.GetReference(0.02, [] {});
    ASSERT_TRUE(fixture.client.ownsObjectStoppedHold());
    ASSERT_TRUE(fixture.client.ReleaseReferenceControl(owner));
    EXPECT_FALSE(fixture.client.object_stopped_hold_);
    EXPECT_TRUE(fixture.client.active_stream_id_.empty());
    auto late = stream(fixture.node, "object:reset", 4,
        finiteReference(0.30, 0.0), kRequestB);
    late->object_tracking_active = true;
    late->state = Stream::STATE_OBJECT_STOPPED;
    fixture.client.receiveReferenceStream(late);
    (void)fixture.client.GetReference(0.02, [] {});
    EXPECT_FALSE(fixture.client.object_stopped_hold_);
    EXPECT_NE(fixture.client.last_applied_sequence_, 4U);
}

TEST(ManeuverReferenceClientTransaction, DroppedObjectStopRequestFailsInStartBudget) {
    RclcppContext context;
    ClientFixture fixture("object_stop_dropped", 40, 0.5);
    ASSERT_TRUE(waitForAckSubscriber(fixture));
    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestB));
    ASSERT_TRUE(fixture.client.ConfirmManeuverGoalHandoff(kRequestB));
    auto active = stream(fixture.node, "object:dropped", 1,
        finiteReference(0.24, 0.20), kRequestB);
    active->object_tracking_active = true;
    fixture.client.receiveReferenceStream(active);
    (void)fixture.client.GetReference(0.02, [] {});
    ASSERT_TRUE(fixture.client.currentAppliedObjectTrackingStream(kRequestB));
    ASSERT_TRUE(fixture.client.CancelManeuverGoalHandoff(kRequestB));
    ASSERT_TRUE(fixture.client.object_stop_);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    int failures = 0;
    const Reference held = fixture.client.GetReference(0.02, [&] { ++failures; });
    EXPECT_EQ(failures, 1);
    EXPECT_NEAR(held.position()(0), 0.24, 1.0e-5);
    EXPECT_EQ(fixture.client.reference_mode_.Load(),
        ManeuverReferenceClient::WAIT_FOR_MANEUVER_STOP);
    EXPECT_TRUE(fixture.client.object_stop_failure_hold_);
    (void)fixture.client.GetReference(0.02, [&] { ++failures; });
    EXPECT_EQ(failures, 1);
}

TEST(ManeuverReferenceClientTransaction, ObjectStopRequestUsesMarkedAppliedGeneration) {
    RclcppContext context;
    TerminalCompletionFixture fixture("object_stop_producer");
    fixture.scheduler.Start();
    fixture.scheduler.maneuver_execution_timer_->cancel();
    auto object = std::make_shared<
        iii_drone::control::maneuver::HoverByObjectManeuverServer>(
            &fixture.node, fixture.awareness, "hover_by_object", 1, 1, false, 1.0);
    fixture.scheduler.RegisterManeuverServer(
        iii_drone::control::maneuver::MANEUVER_TYPE_HOVER_BY_OBJECT, object);
    iii_drone::control::maneuver::Maneuver maneuver(
        iii_drone::control::maneuver::MANEUVER_TYPE_HOVER_BY_OBJECT,
        rclcpp_action::GoalUUID{});
    maneuver.request_identity_ = kRequestB;
    // This fixture does not create a ROS action goal handle. Model the
    // scheduler's already-started copy without dereferencing a null handle.
    maneuver.started_ = true;
    fixture.scheduler.current_maneuver_ = maneuver;
    const Reference moving(point_t(0.24F, 0.0F, 1.5F), 0.0,
        vector_t(0.20F, 0.0F, 0.0F), 0.0, vector_t::Zero(), 0.0,
        fixture.node.now());
    fixture.scheduler.beginReferenceExecution(object->action_name(), kRequestB, moving);
    const uint64_t execution = fixture.scheduler.current_reference_execution_id_.Load();
    auto session = std::make_shared<
        iii_drone::control::maneuver::ObjectTrackingSession>(
            [moving](const Reference &, const Reference &, bool) { return moving; },
            moving, kRequestB, execution, fixture.node.now(), 0.0,
            iii_drone::control::maneuver::ObjectTrackingSession::Limits{});
    object->object_tracking_session_ = session;
    object->object_owner_request_identity_ = kRequestB;
    object->object_owner_execution_id_ = execution;
    auto observer = std::make_shared<rclcpp::Node>("object_stop_stream_observer");
    std::vector<Stream> publications;
    auto subscription = observer->create_subscription<Stream>(
        fixture.scheduler.reference_stream_publisher_->get_topic_name(),
        rclcpp::QoS(10),
        [&publications](const Stream::SharedPtr message) {
            publications.push_back(*message);
        });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(observer);
    for (int attempt = 0; attempt < 30 &&
         fixture.scheduler.reference_stream_publisher_->get_subscription_count() == 0;
         ++attempt) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    fixture.scheduler.publishReferenceStream();
    executor.spin_some();
    const auto stream_id = fixture.scheduler.reference_stream_state_.stream_id;
    const uint64_t marked_sequence = fixture.scheduler.reference_stream_state_.sequence;
    ASSERT_EQ(fixture.scheduler.reference_stream_state_.object_tracking_sequences.back(),
        marked_sequence);
    auto applied = std::make_shared<Ack>();
    applied->stream_id = stream_id;
    applied->last_applied_sequence = marked_sequence;
    applied->consumer_status = Ack::STATUS_APPLIED;
    fixture.scheduler.acknowledgeReferenceStream(applied);
    ASSERT_TRUE(fixture.scheduler.reference_stream_state_.ack_seen);
    const auto applied_at = fixture.scheduler.reference_stream_state_.last_ack;

    auto stop = std::make_shared<Ack>(*applied);
    stop->consumer_status = Ack::STATUS_OBJECT_STOP_REQUESTED;
    stop->last_applied_sequence = marked_sequence + 1;
    fixture.scheduler.acknowledgeReferenceStream(stop);
    EXPECT_FALSE(session->transitionStopping());
    stop->last_applied_sequence = marked_sequence;
    stop->stream_id = "foreign:stream";
    fixture.scheduler.acknowledgeReferenceStream(stop);
    EXPECT_FALSE(session->transitionStopping());
    stop->stream_id = stream_id;
    fixture.scheduler.acknowledgeReferenceStream(stop);
    ASSERT_TRUE(session->transitionStopping());
    EXPECT_EQ(fixture.scheduler.reference_stream_state_.last_ack_sequence,
        marked_sequence);
    EXPECT_EQ(fixture.scheduler.reference_stream_state_.last_consumer_status,
        Ack::STATUS_APPLIED);
    EXPECT_EQ(fixture.scheduler.reference_stream_state_.last_ack, applied_at);
    fixture.scheduler.acknowledgeReferenceStream(stop);  // idempotent duplicate

    fixture.scheduler.reference_callback_struct_->set(
        [object](const iii_drone::control::State & state) {
            return object->GetReference(state);
        }, object->action_name(), execution, kRequestB);
    // A stop request alone does not certify a cached PAUSED/PREPARED sample.
    fixture.scheduler.reference_stream_state_.paused = true;
    fixture.scheduler.publishReferenceStream();
    executor.spin_some();
    ASSERT_FALSE(publications.empty());
    EXPECT_EQ(publications.back().state, Stream::STATE_PAUSED);
    fixture.scheduler.reference_stream_state_.paused = false;
    fixture.scheduler.reference_stream_state_.prepared = true;
    fixture.scheduler.reference_stream_state_.prepared_reference = moving;
    fixture.scheduler.publishReferenceStream();
    executor.spin_some();
    ASSERT_FALSE(publications.empty());
    EXPECT_EQ(publications.back().state, Stream::STATE_PREPARED);
    fixture.scheduler.reference_stream_state_.prepared = false;
    bool saw_stopping = false;
    bool saw_stopped = false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline && !saw_stopped) {
        fixture.scheduler.publishReferenceStream();
        executor.spin_some();
        const auto & published = fixture.scheduler.reference_stream_state_;
        const Reference command = published.latest_reference;
        if (session->transitionRest()) {
            saw_stopped = command.position().allFinite() &&
                command.velocity().norm() <= 1.0e-5 &&
                command.acceleration().norm() <= 1.0e-5 &&
                std::abs(command.yaw_rate()) <= 1.0e-5 &&
                std::abs(command.yaw_acceleration()) <= 1.0e-5;
        } else {
            saw_stopping = true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    EXPECT_TRUE(saw_stopping);
    EXPECT_TRUE(saw_stopped);
    EXPECT_TRUE(std::any_of(publications.begin(), publications.end(),
        [&stream_id](const Stream & message) {
            return message.stream_id == stream_id &&
                message.state == Stream::STATE_OBJECT_STOPPING &&
                message.object_tracking_active;
        }));
    EXPECT_TRUE(std::any_of(publications.begin(), publications.end(),
        [&stream_id](const Stream & message) {
            return message.stream_id == stream_id &&
                message.state == Stream::STATE_OBJECT_STOPPED &&
                message.object_tracking_active;
        }));
    EXPECT_EQ(fixture.scheduler.reference_stream_state_.stream_id, stream_id);
    EXPECT_EQ(fixture.scheduler.reference_stream_state_.last_ack_sequence,
        marked_sequence);
    // The client may hold this certified rest locally for longer than one
    // ACK timeout before another positional action is accepted. Its ongoing
    // STOPPED APPLIED acknowledgements must still seed that action exactly.
    fixture.hover->Update(Reference(moving.position(), 0.0));
    maneuver.terminated_ = true;
    maneuver.success_ = true;
    fixture.scheduler.current_maneuver_ = maneuver;
    fixture.scheduler.onReferenceCallbackTokenReacquired();
    for (int tick = 0; tick < 4; ++tick) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        fixture.scheduler.publishReferenceStream();
        auto held_ack = std::make_shared<Ack>();
        held_ack->stream_id = stream_id;
        held_ack->last_applied_sequence = fixture.scheduler.reference_stream_state_.sequence;
        held_ack->consumer_status = Ack::STATUS_APPLIED;
        fixture.scheduler.acknowledgeReferenceStream(held_ack);
    }
    const Reference stopped_seed = fixture.scheduler.reference_stream_state_.latest_reference;
    ASSERT_NEAR(stopped_seed.velocity().norm(), 0.0, 1.0e-5);
    const auto old_binding = fixture.scheduler.reference_callback_struct_->snapshot();
    const auto old_execution = fixture.scheduler.current_reference_execution_id_.Load();
    iii_drone::types::transform_matrix_t transform =
        iii_drone::types::transform_matrix_t::Identity();
    const iii_drone::adapters::TargetAdapter different_target(
        iii_drone::adapters::TARGET_TYPE_CABLE, 2, "world", transform);
    AcceptedHoverByObjectGoal unsupported("object_stop_different_target");
    const auto unsupported_handle = unsupported.accept(kRequestD, different_target);
    ASSERT_TRUE(unsupported_handle);
    fixture.scheduler.current_maneuver_ =
        iii_drone::control::maneuver::Maneuver::FromGoalHandle<
            AcceptedHoverByObjectGoal::Action>(unsupported_handle);
    std::promise<void> rejection_done;
    auto rejected = rejection_done.get_future();
    std::thread rejected_worker([&] {
        object->asyncExecute<AcceptedHoverByObjectGoal::Action>(unsupported_handle);
        rejection_done.set_value();
    });
    fixture.scheduler.progressScheduler();
    const bool aborted = rejected.wait_for(std::chrono::seconds(3)) ==
        std::future_status::ready;
    if (!aborted) object->running_.Store(false);
    rejected_worker.join();
    ASSERT_TRUE(aborted);
    EXPECT_FALSE(unsupported_handle->is_active());
    EXPECT_EQ(fixture.scheduler.current_reference_execution_id_.Load(), old_execution);
    EXPECT_EQ(fixture.scheduler.reference_callback_struct_->snapshot().request_identity,
        old_binding.request_identity);
    EXPECT_EQ(fixture.scheduler.reference_stream_state_.stream_id, stream_id);
    fixture.scheduler.publishReferenceStream();
    auto after_rejection = std::make_shared<Ack>();
    after_rejection->stream_id = stream_id;
    after_rejection->last_applied_sequence =
        fixture.scheduler.reference_stream_state_.sequence;
    after_rejection->consumer_status = Ack::STATUS_APPLIED;
    fixture.scheduler.acknowledgeReferenceStream(after_rejection);
    EXPECT_TRUE(fixture.scheduler.reference_stream_state_.valid);
    EXPECT_EQ(fixture.scheduler.reference_stream_state_.last_ack_sequence,
        after_rejection->last_applied_sequence);
    EXPECT_NEAR(fixture.scheduler.reference_stream_state_.latest_reference.position()(0),
        stopped_seed.position()(0), 1.0e-5);
    auto position_server = std::make_shared<
        iii_drone::control::maneuver::FlyToPositionManeuverServer>(
            &fixture.node, fixture.awareness, "fly_to_position", 1, 1,
            fixture.config, nullptr);
    fixture.scheduler.registered_maneuvers_[
        iii_drone::control::maneuver::MANEUVER_TYPE_FLY_TO_POSITION] = position_server;
    AcceptedPositionGoal next_goal("object_stop_next_position");
    const auto next_handle = next_goal.accept(kRequestC);
    ASSERT_TRUE(next_handle);
    fixture.scheduler.current_maneuver_ =
        iii_drone::control::maneuver::Maneuver::FromGoalHandle<
            AcceptedPositionGoal::Action>(next_handle);
    fixture.scheduler.progressScheduler();
    ASSERT_TRUE(fixture.scheduler.current_maneuver_.Load().started());
    EXPECT_EQ(position_server->terminal_start_request_identity_, kRequestC);
    const auto next_seed = position_server->terminal_start_reference_;
    ASSERT_TRUE(next_seed);
    EXPECT_LT((next_seed->position() - stopped_seed.position()).norm(), 1.0e-5);
    EXPECT_FALSE(object->HasTrackedSession());
    EXPECT_FALSE(fixture.scheduler.appliedFiniteRestReference(kRequestB, stopped_seed));
    const auto successor_stream_id = fixture.scheduler.reference_stream_state_.stream_id;
    auto stale_owner_ack = std::make_shared<Ack>();
    stale_owner_ack->stream_id = stream_id;
    stale_owner_ack->last_applied_sequence = after_rejection->last_applied_sequence;
    stale_owner_ack->consumer_status = Ack::STATUS_APPLIED;
    fixture.scheduler.acknowledgeReferenceStream(stale_owner_ack);
    EXPECT_EQ(fixture.scheduler.reference_stream_state_.stream_id, successor_stream_id);
    EXPECT_FALSE(fixture.scheduler.reference_stream_state_.ack_seen);
    executor.remove_node(observer);
    object->Stop();
    fixture.scheduler.registered_maneuvers_.erase(
        iii_drone::control::maneuver::MANEUVER_TYPE_HOVER_BY_OBJECT);
}

TEST(ManeuverReferenceClientTransaction, UnsafeObjectStartupAbortsExactAcceptedGoal) {
    RclcppContext context;
    TerminalCompletionFixture fixture("object_unsafe_startup");
    fixture.scheduler.Start();
    fixture.scheduler.maneuver_execution_timer_->cancel();
    auto object = std::make_shared<
        iii_drone::control::maneuver::FlyToObjectManeuverServer>(
            &fixture.node, fixture.awareness, "fly_to_object", 1, 1,
            fixture.config, nullptr);
    fixture.scheduler.RegisterManeuverServer(
        iii_drone::control::maneuver::MANEUVER_TYPE_FLY_TO_OBJECT, object);
    iii_drone::types::transform_matrix_t transform =
        iii_drone::types::transform_matrix_t::Identity();
    const iii_drone::adapters::TargetAdapter target(
        iii_drone::adapters::TARGET_TYPE_CABLE, 1, "world", transform);
    AcceptedObjectGoal goal("object_unsafe_startup_goal");
    const auto handle = goal.accept(kRequestB, target);
    ASSERT_TRUE(handle);
    auto maneuver = iii_drone::control::maneuver::Maneuver::FromGoalHandle<
        AcceptedObjectGoal::Action>(handle);
    fixture.scheduler.current_maneuver_ = maneuver;
    fixture.scheduler.current_maneuver_->Start();
    ASSERT_TRUE(fixture.scheduler.current_maneuver_.Load().started());

    // The finite seed is usable as a command but its downward certified stop
    // crosses the existing commanded-altitude floor. The detached action
    // worker must reject it before installing a new object session.
    const Reference unsafe_seed(point_t(0.24F, 0.0F, 0.10F), 0.0,
        vector_t(0.0F, 0.0F, -1.0F), 0.0, vector_t::Zero(), 0.0,
        fixture.node.now());
    ASSERT_TRUE(object->StageTerminalStartReference(kRequestB, unsafe_seed));
    fixture.scheduler.beginReferenceExecution(object->action_name(), kRequestB, unsafe_seed);
    fixture.scheduler.publishReferenceStream();
    const auto published = fixture.scheduler.reference_stream_state_;
    ASSERT_TRUE(published.valid);
    auto ack = std::make_shared<Ack>();
    ack->stream_id = published.stream_id;
    ack->last_applied_sequence = published.sequence;
    ack->consumer_status = Ack::STATUS_APPLIED;
    fixture.scheduler.acknowledgeReferenceStream(ack);
    ASSERT_TRUE(fixture.scheduler.reference_stream_state_.last_ack_reference_valid);

    std::promise<bool> worker_result;
    auto worker_done = worker_result.get_future();
    std::thread worker([&] {
        try {
            object->asyncExecute<AcceptedObjectGoal::Action>(handle);
            worker_result.set_value(true);
        } catch (...) {
            worker_result.set_value(false);
        }
    });
    for (int attempt = 0; attempt < 200 &&
         !fixture.scheduler.reference_callback_token_.has_requested_token(
             object->action_name()); ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    const bool token_requested =
        fixture.scheduler.reference_callback_token_.has_requested_token(
            object->action_name());
    if (token_requested) fixture.scheduler.reference_callback_token_.Give(object->action_name());
    const bool completed = worker_done.wait_for(std::chrono::seconds(3)) ==
        std::future_status::ready;
    if (completed) {
        EXPECT_TRUE(worker_done.get());
    } else {
        object->running_.Store(false);
    }
    worker.join();
    ASSERT_TRUE(token_requested);
    ASSERT_TRUE(completed);
    EXPECT_TRUE(fixture.scheduler.current_maneuver_.Load().terminated());
    EXPECT_FALSE(fixture.scheduler.current_maneuver_.Load().success());
    EXPECT_FALSE(handle->is_active());
    const auto binding = fixture.scheduler.reference_callback_struct_->snapshot();
    EXPECT_TRUE(object->startupRejected(binding));
    EXPECT_FALSE(object->RetainsTrackedSource(binding));
    fixture.scheduler.publishReferenceStream();
    EXPECT_EQ(fixture.scheduler.reference_stream_state_.stream_id, published.stream_id);
    EXPECT_NEAR(fixture.scheduler.reference_stream_state_.latest_reference.position()(0),
        unsafe_seed.position()(0), 1.0e-5);
    const auto successor = binding;
    auto wrong_generation = successor;
    ++wrong_generation.execution_id;
    EXPECT_FALSE(object->startupRejected(wrong_generation));
    object->Stop();
    fixture.scheduler.registered_maneuvers_.erase(
        iii_drone::control::maneuver::MANEUVER_TYPE_FLY_TO_OBJECT);
}

TEST(ManeuverReferenceClientTransaction, UnsafeObjectStartupWithoutAppliedCommandDoesNotInventHold) {
    RclcppContext context;
    TerminalCompletionFixture fixture("object_unsafe_unapplied_startup");
    fixture.scheduler.Start();
    fixture.scheduler.maneuver_execution_timer_->cancel();
    auto object = std::make_shared<
        iii_drone::control::maneuver::FlyToObjectManeuverServer>(
            &fixture.node, fixture.awareness, "fly_to_object_unapplied", 1, 1,
            fixture.config, nullptr);
    fixture.scheduler.RegisterManeuverServer(
        iii_drone::control::maneuver::MANEUVER_TYPE_FLY_TO_OBJECT, object);
    iii_drone::types::transform_matrix_t transform =
        iii_drone::types::transform_matrix_t::Identity();
    const iii_drone::adapters::TargetAdapter target(
        iii_drone::adapters::TARGET_TYPE_CABLE, 1, "world", transform);
    AcceptedObjectGoal goal("object_unsafe_unapplied_goal");
    const auto handle = goal.accept(kRequestB, target);
    ASSERT_TRUE(handle);
    fixture.scheduler.current_maneuver_ =
        iii_drone::control::maneuver::Maneuver::FromGoalHandle<
            AcceptedObjectGoal::Action>(handle);
    fixture.scheduler.current_maneuver_->Start();
    const Reference predecessor = fixture.hold->lastCommand();
    const Reference unsafe_seed(point_t(0.24F, 0.0F, 0.10F), 0.0,
        vector_t(0.0F, 0.0F, -1.0F), 0.0, vector_t::Zero(), 0.0,
        fixture.node.now());
    ASSERT_TRUE(object->StageTerminalStartReference(kRequestB, unsafe_seed));
    fixture.scheduler.beginReferenceExecution(object->action_name(), kRequestB,
        unsafe_seed);
    fixture.scheduler.publishReferenceStream();
    ASSERT_TRUE(fixture.scheduler.reference_stream_state_.valid);
    ASSERT_FALSE(fixture.scheduler.reference_stream_state_.ack_seen);
    std::promise<bool> worker_result;
    auto worker_done = worker_result.get_future();
    std::thread worker([&] {
        try {
            object->asyncExecute<AcceptedObjectGoal::Action>(handle);
            worker_result.set_value(true);
        } catch (...) {
            worker_result.set_value(false);
        }
    });
    for (int attempt = 0; attempt < 200 &&
         !fixture.scheduler.reference_callback_token_.has_requested_token(
             object->action_name()); ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    const bool token_requested =
        fixture.scheduler.reference_callback_token_.has_requested_token(
            object->action_name());
    if (token_requested) fixture.scheduler.reference_callback_token_.Give(object->action_name());
    const bool completed = worker_done.wait_for(std::chrono::seconds(3)) ==
        std::future_status::ready;
    if (!completed) object->running_.Store(false);
    worker.join();
    ASSERT_TRUE(token_requested);
    ASSERT_TRUE(completed);
    EXPECT_TRUE(worker_done.get());
    EXPECT_TRUE(fixture.scheduler.current_maneuver_.Load().terminated());
    EXPECT_FALSE(fixture.scheduler.current_maneuver_.Load().success());
    EXPECT_FALSE(handle->is_active());
    const auto binding = fixture.scheduler.reference_callback_struct_->snapshot();
    EXPECT_TRUE(object->startupRejected(binding));
    EXPECT_FALSE(binding.callback);
    EXPECT_FALSE(fixture.scheduler.reference_stream_state_.valid);
    EXPECT_FALSE(fixture.scheduler.maneuver_server_get_reference_callback_still_registered_.Load());
    EXPECT_FALSE(object->RetainsTrackedSource(binding));
    EXPECT_EQ(fixture.hover->terminalHoldBinding().request_identity, kRequestA);
    EXPECT_LT((fixture.hold->lastCommand().position() - predecessor.position()).norm(), 1.0e-6);
    object->Stop();
    fixture.scheduler.registered_maneuvers_.erase(
        iii_drone::control::maneuver::MANEUVER_TYPE_FLY_TO_OBJECT);
}

TEST(ManeuverReferenceClientTransaction, SuccessfulTerminalResultRetainsOwnerBeforeSchedulerTick) {
    RclcppContext context;
    TerminalCompletionFixture fixture("terminal_success_completion_gap");
    fixture.finishWithoutSchedulerTick(
        iii_drone::control::maneuver::MANEUVER_TYPE_CABLE_AWARE_FLY_TO_POSITION,
        "cable_aware_fly_to_position", true);
    ASSERT_TRUE(fixture.scheduler.reference_stream_state_.last_ack_reference_valid);
    const auto offer = fixture.query();
    EXPECT_TRUE(offer->accepted) << offer->reason;
    if (offer->accepted) {
        EXPECT_EQ(offer->source_request_identity, kRequestA);
        EXPECT_EQ(offer->source_stream_id, "terminal:completed");
    }
}

TEST(ManeuverReferenceClientTransaction, CanceledWaypointRestRetainsOwnerBeforeSchedulerTick) {
    RclcppContext context;
    TerminalCompletionFixture fixture("terminal_cancel_completion_gap");
    ASSERT_TRUE(fixture.hold->RequestQuiescence());
    fixture.finishWithoutSchedulerTick(
        iii_drone::control::maneuver::MANEUVER_TYPE_FOLLOW_WAYPOINT_PATH,
        "follow_waypoint_path", false);
    const auto binding = fixture.scheduler.reference_callback_struct_->snapshot();
    EXPECT_EQ(binding.reference_provider_name, "hover");
    EXPECT_EQ(binding.request_identity, kRequestA);
    EXPECT_EQ(binding.execution_id, fixture.scheduler.current_reference_execution_id_.Load());
    const auto offer = fixture.query();
    EXPECT_TRUE(offer->accepted) << offer->reason;
    const auto command = binding.callback(iii_drone::control::State());
    EXPECT_TRUE(command.position().allFinite());
    EXPECT_TRUE(command.velocity().allFinite());
    EXPECT_TRUE(command.acceleration().allFinite());
    EXPECT_LT((command.position() - fixture.hold->nominalReference().position()).norm(), 1.0e-5);
}

TEST(ManeuverReferenceClientTransaction, ExactCompletedOwnerHasOnlyBoundedFinalizationTransient) {
    RclcppContext context;
    TerminalCompletionFixture fixture("terminal_exact_finalizing");
    fixture.finishWithoutSchedulerTick(
        iii_drone::control::maneuver::MANEUVER_TYPE_CABLE_AWARE_FLY_TO_POSITION,
        "cable_aware_fly_to_position", true, false);
    const auto before_release = fixture.query();
    EXPECT_FALSE(before_release->accepted);
    EXPECT_EQ(before_release->reason, "terminal callback finalizing");
    fixture.scheduler.onReferenceCallbackTokenReacquired();
    const auto after_release = fixture.query();
    EXPECT_TRUE(after_release->accepted) << after_release->reason;
}

TEST(ManeuverReferenceClientTransaction, TokenCompletionCrossingTransferSnapshotKeepsExactOffer) {
    RclcppContext context;
    TerminalCompletionFixture fixture("terminal_completion_crossing_query");
    fixture.finishWithoutSchedulerTick(
        iii_drone::control::maneuver::MANEUVER_TYPE_CABLE_AWARE_FLY_TO_POSITION,
        "cable_aware_fly_to_position", true, false);
    ASSERT_TRUE(fixture.scheduler.reference_stream_state_.last_ack_reference_valid);
    const auto stream_id = fixture.scheduler.reference_stream_state_.stream_id;
    std::mutex completion_mutex;
    std::condition_variable completion_cv;
    std::thread completion_thread;
    bool completion_ready = false;
    bool completion_blocked_on_snapshot = false;
    fixture.scheduler.terminal_hold_transfer_after_validity_hook_ = [&] {
        completion_thread = std::thread([&] {
            const bool acquired = fixture.scheduler.reference_stream_mutex_.try_lock();
            if (acquired) {
                fixture.scheduler.reference_stream_mutex_.unlock();
                fixture.scheduler.onReferenceCallbackTokenReacquired();
            }
            {
                std::lock_guard<std::mutex> lock(completion_mutex);
                completion_blocked_on_snapshot = !acquired;
                completion_ready = true;
            }
            completion_cv.notify_one();
            if (!acquired) fixture.scheduler.onReferenceCallbackTokenReacquired();
        });
        std::unique_lock<std::mutex> lock(completion_mutex);
        completion_cv.wait(lock, [&] { return completion_ready; });
    };
    const auto crossing = fixture.query();
    completion_thread.join();
    EXPECT_TRUE(completion_blocked_on_snapshot);
    EXPECT_FALSE(crossing->accepted);
    EXPECT_EQ(crossing->reason, "terminal callback finalizing");
    EXPECT_EQ(fixture.scheduler.reference_stream_state_.stream_id, stream_id);
    EXPECT_TRUE(fixture.scheduler.reference_stream_state_.last_ack_reference_valid);
    fixture.scheduler.terminal_hold_transfer_after_validity_hook_ = {};
    const auto completed = fixture.query();
    EXPECT_TRUE(completed->accepted) << completed->reason;
    if (completed->accepted) {
        EXPECT_EQ(completed->source_request_identity, kRequestA);
        EXPECT_EQ(completed->source_stream_id, stream_id);
    }
}

TEST(ManeuverReferenceClientTransaction, SchedulerTickCannotOutrunRetainedTokenCallback) {
    RclcppContext context;
    TerminalCompletionFixture fixture("terminal_tick_before_token_callback");
    ASSERT_TRUE(fixture.hold->RequestQuiescence());
    fixture.finishWithoutSchedulerTick(
        iii_drone::control::maneuver::MANEUVER_TYPE_FOLLOW_WAYPOINT_PATH,
        "follow_waypoint_path", false, false);
    fixture.scheduler.maneuver_queue_ =
        std::make_unique<iii_drone::control::maneuver::ManeuverQueue>(1);
    fixture.scheduler.progressScheduler();
    EXPECT_EQ(fixture.scheduler.current_maneuver_->maneuver_type(),
        iii_drone::control::maneuver::MANEUVER_TYPE_FOLLOW_WAYPOINT_PATH);
    EXPECT_FALSE(fixture.scheduler.maneuver_server_get_reference_callback_still_registered_.Load());
    EXPECT_EQ(fixture.query()->reason, "terminal callback finalizing");
    fixture.scheduler.onReferenceCallbackTokenReacquired();
    EXPECT_TRUE(fixture.query()->accepted);
}

TEST(ManeuverReferenceClientTransaction, TimerBeforeTokenReturnPreservesExactAppliedStream) {
    RclcppContext context;
    TerminalCompletionFixture fixture("terminal_timer_before_token_return");
    fixture.finishWithoutSchedulerTick(
        iii_drone::control::maneuver::MANEUVER_TYPE_CABLE_AWARE_FLY_TO_POSITION,
        "cable_aware_fly_to_position", true, false);
    fixture.scheduler.maneuver_queue_ =
        std::make_unique<iii_drone::control::maneuver::ManeuverQueue>(1);
    ASSERT_TRUE(fixture.scheduler.reference_stream_state_.valid);
    ASSERT_TRUE(fixture.scheduler.reference_stream_state_.last_ack_reference_valid);
    const auto stream_id = fixture.scheduler.reference_stream_state_.stream_id;
    const auto applied_sequence = fixture.scheduler.reference_stream_state_.last_ack_sequence;

    // The production timer publishes immediately after scheduler progression.
    // This tick lands after action termination but before Token::Release calls
    // the master reacquire callback.
    fixture.scheduler.maneuverExecutionTimerCallback();
    EXPECT_TRUE(fixture.scheduler.reference_stream_state_.valid);
    EXPECT_EQ(fixture.scheduler.reference_stream_state_.stream_id, stream_id);
    EXPECT_EQ(fixture.scheduler.reference_stream_state_.last_ack_sequence, applied_sequence);
    EXPECT_EQ(fixture.query()->reason, "terminal callback finalizing");
    fixture.scheduler.onReferenceCallbackTokenReacquired();
    const auto after_release = fixture.query();
    EXPECT_TRUE(after_release->accepted) << after_release->reason;
    if (after_release->accepted) {
        EXPECT_EQ(after_release->source_stream_id, stream_id);
    }
}

TEST(ManeuverReferenceClientTransaction, CanceledHoverRebindAwaitingValidityKeepsSameStream) {
    RclcppContext context;
    TerminalCompletionFixture fixture("terminal_cancel_hover_rebind_gap");
    fixture.finishWithoutSchedulerTick(
        iii_drone::control::maneuver::MANEUVER_TYPE_FOLLOW_WAYPOINT_PATH,
        "follow_waypoint_path", false);
    ASSERT_EQ(fixture.scheduler.reference_callback_struct_->snapshot().reference_provider_name,
        fixture.hover->action_name());
    fixture.scheduler.maneuver_server_get_reference_callback_still_registered_ = false;
    fixture.scheduler.maneuver_queue_ =
        std::make_unique<iii_drone::control::maneuver::ManeuverQueue>(1);
    const auto stream_id = fixture.scheduler.reference_stream_state_.stream_id;
    const auto applied_sequence = fixture.scheduler.reference_stream_state_.last_ack_sequence;

    // The callback has installed Hover for this canceled owner but has not
    // published its validity bit yet. Timer progression and publication may
    // interleave here because Token::Release exposed the master holder first.
    fixture.scheduler.maneuverExecutionTimerCallback();
    EXPECT_EQ(fixture.scheduler.current_maneuver_->maneuver_type(),
        iii_drone::control::maneuver::MANEUVER_TYPE_FOLLOW_WAYPOINT_PATH);
    EXPECT_TRUE(fixture.scheduler.reference_stream_state_.valid);
    EXPECT_EQ(fixture.scheduler.reference_stream_state_.stream_id, stream_id);
    EXPECT_EQ(fixture.scheduler.reference_stream_state_.last_ack_sequence, applied_sequence);
    EXPECT_EQ(fixture.query()->reason, "terminal callback finalizing");
    fixture.scheduler.maneuver_server_get_reference_callback_still_registered_ = true;
    const auto after_finalization = fixture.query();
    EXPECT_TRUE(after_finalization->accepted) << after_finalization->reason;
}

TEST(ManeuverReferenceClientTransaction, CompletionWindowDoesNotPreserveWrongOwnerGenerationOrMissingHold) {
    RclcppContext context;
    const auto check_invalidated = [](TerminalCompletionFixture & fixture) {
        ASSERT_TRUE(fixture.scheduler.reference_stream_state_.valid);
        fixture.scheduler.publishReferenceStream();
        EXPECT_FALSE(fixture.scheduler.reference_stream_state_.valid);
    };
    TerminalCompletionFixture wrong_owner("terminal_window_wrong_owner");
    wrong_owner.finishWithoutSchedulerTick(
        iii_drone::control::maneuver::MANEUVER_TYPE_CABLE_AWARE_FLY_TO_POSITION,
        "cable_aware_fly_to_position", true, false);
    wrong_owner.hover->AdoptTerminalHold(wrong_owner.hold, kRequestB);
    check_invalidated(wrong_owner);

    TerminalCompletionFixture wrong_generation("terminal_window_wrong_generation");
    wrong_generation.finishWithoutSchedulerTick(
        iii_drone::control::maneuver::MANEUVER_TYPE_CABLE_AWARE_FLY_TO_POSITION,
        "cable_aware_fly_to_position", true, false);
    wrong_generation.scheduler.current_reference_execution_id_.Store(
        wrong_generation.scheduler.current_reference_execution_id_.Load() + 1);
    check_invalidated(wrong_generation);

    TerminalCompletionFixture missing_hold("terminal_window_missing_hold");
    missing_hold.finishWithoutSchedulerTick(
        iii_drone::control::maneuver::MANEUVER_TYPE_CABLE_AWARE_FLY_TO_POSITION,
        "cable_aware_fly_to_position", true, false);
    missing_hold.hover->ClearTerminalHold();
    check_invalidated(missing_hold);
}

TEST(ManeuverReferenceClientTransaction, PriorCompletedGenerationCannotValidateNextTerminalCallback) {
    RclcppContext context;
    TerminalCompletionFixture fixture("terminal_two_generations");
    fixture.finishWithoutSchedulerTick(
        iii_drone::control::maneuver::MANEUVER_TYPE_CABLE_AWARE_FLY_TO_POSITION,
        "cable_aware_fly_to_position", true);
    ASSERT_TRUE(fixture.scheduler.maneuver_server_get_reference_callback_still_registered_.Load());
    ASSERT_TRUE(fixture.query()->accepted);
    const auto first_execution = fixture.scheduler.current_reference_execution_id_.Load();

    fixture.hover->AdoptTerminalHold(fixture.hold, kRequestB);
    fixture.finishWithoutSchedulerTick(
        iii_drone::control::maneuver::MANEUVER_TYPE_FOLLOW_WAYPOINT_PATH,
        "follow_waypoint_path", false, false, kRequestB);
    EXPECT_GT(fixture.scheduler.current_reference_execution_id_.Load(), first_execution);
    EXPECT_FALSE(fixture.scheduler.maneuver_server_get_reference_callback_still_registered_.Load());
    const auto before_release = fixture.query();
    EXPECT_FALSE(before_release->accepted);
    EXPECT_EQ(before_release->reason, "terminal callback finalizing");
    fixture.scheduler.onReferenceCallbackTokenReacquired();
    const auto after_release = fixture.query();
    EXPECT_TRUE(after_release->accepted) << after_release->reason;
    if (after_release->accepted) {
        EXPECT_EQ(after_release->source_request_identity, kRequestB);
    }
}

TEST(ManeuverReferenceClientTransaction, NativeHoldRetiresOnlyExactCompletedObjectOwner) {
    RclcppContext context;
    const auto prepare_completed_object = [](TerminalCompletionFixture & fixture) {
        auto object = std::make_shared<
            iii_drone::control::maneuver::HoverByObjectManeuverServer>(
                &fixture.node, fixture.awareness, "hover_by_object_native", 1, 1,
                false, 1.0);
        fixture.scheduler.registered_maneuvers_[
            iii_drone::control::maneuver::MANEUVER_TYPE_HOVER_BY_OBJECT] = object;
        iii_drone::control::maneuver::Maneuver active(
            iii_drone::control::maneuver::MANEUVER_TYPE_HOVER_BY_OBJECT,
            rclcpp_action::GoalUUID{});
        active.request_identity_ = kRequestB;
        active.started_ = true;
        fixture.scheduler.current_maneuver_ = active;
        const Reference command(point_t(0.24F, 0.0F, 1.5F), 0.0,
            vector_t::Zero(), 0.0, vector_t::Zero(), 0.0,
            fixture.node.now());
        fixture.scheduler.beginReferenceExecution(object->action_name(), kRequestB,
            command);
        const uint64_t execution =
            fixture.scheduler.current_reference_execution_id_.Load();
        auto session = std::make_shared<
            iii_drone::control::maneuver::ObjectTrackingSession>(
                [command](const Reference &, const Reference &, bool) {
                    return command;
                }, command, kRequestB, execution, fixture.node.now(), 0.0,
                iii_drone::control::maneuver::ObjectTrackingSession::Limits{});
        object->object_tracking_session_ = session;
        object->object_owner_request_identity_ = kRequestB;
        object->object_owner_execution_id_ = execution;
        fixture.scheduler.publishReferenceStream();
        EXPECT_TRUE(fixture.scheduler.reference_stream_state_.valid);
        active.terminated_ = true;
        active.success_ = true;
        fixture.scheduler.current_maneuver_ = active;
        fixture.scheduler.onReferenceCallbackTokenReacquired();
        EXPECT_TRUE(fixture.scheduler.retained_native_hold_epoch_.completed);
        return object;
    };

    TerminalCompletionFixture exact("native_hold_completed_object");
    exact.observeSourceExternal();
    auto exact_object = prepare_completed_object(exact);
    const auto old_binding = exact.scheduler.reference_callback_struct_->snapshot();
    EXPECT_TRUE(exact_object->RetainsTrackedSource(old_binding));
    exact.observeNativeHold();
    EXPECT_TRUE(exact.scheduler.retireCompletedTerminalHoldAfterNativeHold());
    EXPECT_FALSE(exact_object->HasTrackedSession());
    EXPECT_FALSE(exact.scheduler.reference_stream_state_.valid);
    EXPECT_FALSE(exact.scheduler.reference_callback_struct_->snapshot().callback);

    TerminalCompletionFixture stale("native_hold_stale_object");
    stale.observeNativeHold();  // No positive external owner after this native mode.
    auto stale_object = prepare_completed_object(stale);
    stale.observeNativeHold(3000000, 3000000);
    EXPECT_FALSE(stale.scheduler.retireCompletedTerminalHoldAfterNativeHold());
    EXPECT_TRUE(stale_object->HasTrackedSession());

    TerminalCompletionFixture successor("native_hold_object_successor");
    successor.observeSourceExternal();
    auto old_object = prepare_completed_object(successor);
    successor.scheduler.beginReferenceExecution("fly_to_position", kRequestC,
        Reference(point_t(0.24F, 0.0F, 1.5F), 0.0));
    const auto successor_binding = successor.scheduler.reference_callback_struct_->snapshot();
    successor.observeNativeHold();
    EXPECT_FALSE(successor.scheduler.retireCompletedTerminalHoldAfterNativeHold());
    EXPECT_EQ(successor.scheduler.reference_callback_struct_->snapshot().request_identity,
        kRequestC);
    EXPECT_EQ(successor.scheduler.reference_callback_struct_->snapshot().execution_id,
        successor_binding.execution_id);
    EXPECT_TRUE(old_object->HasTrackedSession());
}

TEST(ManeuverReferenceClientTransaction, NativeHoldRetiresExactCompletedOwnerBeforeAndAfterAckLoss) {
    RclcppContext context;
    using Transfer = iii_drone_interfaces::srv::TerminalHoldTransfer;
    for (const bool after_ack_loss : {false, true}) {
        TerminalCompletionFixture fixture(after_ack_loss
            ? "native_hold_after_ack_loss" : "native_hold_before_ack_loss", 500);
        fixture.observeSourceExternal();
        fixture.finishWithoutSchedulerTick(
            iii_drone::control::maneuver::MANEUVER_TYPE_CABLE_AWARE_FLY_TO_POSITION,
            "cable_aware_fly_to_position", true);
        ASSERT_TRUE(fixture.query()->accepted);
        const auto stream_id = fixture.scheduler.reference_stream_state_.stream_id;
        if (after_ack_loss) {
            std::this_thread::sleep_for(std::chrono::milliseconds(550));
            auto measured = fixture.awareness->measured_odometry_.Load();
            ASSERT_TRUE(measured);
            measured->receipt_stamp = fixture.node.now();
            measured->source_sample_timestamp_us += 550000;
            fixture.awareness->measured_odometry_.Store(measured);
            fixture.scheduler.publishReferenceStream();
            EXPECT_NE(fixture.hold->phase(),
                iii_drone::control::maneuver::TerminalTrackingHold::Phase::Tracking);
            EXPECT_NE(fixture.hold->failureReason().find("acknowledgement"),
                std::string::npos);
        }
        const auto phase = fixture.hold->phase();
        const auto reason = fixture.hold->failureReason();
        fixture.observeNativeHold();
        const auto no_offer = fixture.query();  // same Core QUERY used by Inspection
        EXPECT_FALSE(no_offer->accepted);
        EXPECT_EQ(no_offer->reason, "no retained terminal hold");
        EXPECT_FALSE(fixture.hover->terminalHold());
        EXPECT_FALSE(fixture.scheduler.reference_stream_state_.valid);
        EXPECT_TRUE(fixture.scheduler.reference_stream_state_.offer_consumer_identity.empty());
        EXPECT_EQ(fixture.hold->phase(), phase);
        EXPECT_EQ(fixture.hold->failureReason(), reason);
        auto stale_claim = std::make_shared<Transfer::Request>();
        stale_claim->operation = Transfer::Request::OP_CLAIM;
        stale_claim->consumer_identity = kRequestB;
        stale_claim->source_request_identity = kRequestA;
        stale_claim->source_stream_id = stream_id;
        stale_claim->source_ack_sequence = 1;
        auto rejected_claim = std::make_shared<Transfer::Response>();
        fixture.scheduler.terminalHoldTransfer(stale_claim, rejected_claim);
        EXPECT_FALSE(rejected_claim->accepted);
        EXPECT_EQ(rejected_claim->reason, "no retained terminal hold");
    }
}

TEST(ManeuverReferenceClientTransaction, NativeHoldBeforeTokenFinalizationRetiresOnlyAfterExactCompletion) {
    RclcppContext context;
    TerminalCompletionFixture fixture("native_hold_before_token_return");
    fixture.observeSourceExternal();
    fixture.finishWithoutSchedulerTick(
        iii_drone::control::maneuver::MANEUVER_TYPE_CABLE_AWARE_FLY_TO_POSITION,
        "cable_aware_fly_to_position", true, false);
    fixture.observeNativeHold();
    EXPECT_FALSE(fixture.scheduler.retireCompletedTerminalHoldAfterNativeHold());
    EXPECT_TRUE(fixture.hover->terminalHold());
    fixture.scheduler.onReferenceCallbackTokenReacquired();
    EXPECT_EQ(fixture.query()->reason, "no retained terminal hold");
}

TEST(ManeuverReferenceClientTransaction, NativeHoldRetiresCanceledCompletedOwnerWithoutChangingOutcome) {
    RclcppContext context;
    TerminalCompletionFixture fixture("native_hold_canceled_owner");
    fixture.observeSourceExternal();
    fixture.finishWithoutSchedulerTick(
        iii_drone::control::maneuver::MANEUVER_TYPE_FOLLOW_WAYPOINT_PATH,
        "follow_waypoint_path", false);
    ASSERT_TRUE(fixture.query()->accepted);
    fixture.hold->Fail("test canceled terminal fault");
    const auto prior_phase = fixture.hold->phase();
    const auto prior_reason = fixture.hold->failureReason();
    ASSERT_FALSE(fixture.scheduler.current_maneuver_->success());
    fixture.observeNativeHold();
    EXPECT_EQ(fixture.query()->reason, "no retained terminal hold");
    EXPECT_FALSE(fixture.scheduler.current_maneuver_->success());
    EXPECT_EQ(fixture.hold->phase(), prior_phase);
    EXPECT_EQ(fixture.hold->failureReason(), prior_reason);
}

TEST(ManeuverReferenceClientTransaction, NativeHoldRequiresUniqueFreshOrderedStatusAndExactOwner) {
    RclcppContext context;
    TerminalCompletionFixture stale("native_hold_stale_status");
    stale.observeSourceExternal();
    stale.finishWithoutSchedulerTick(
        iii_drone::control::maneuver::MANEUVER_TYPE_CABLE_AWARE_FLY_TO_POSITION,
        "cable_aware_fly_to_position", true);
    EXPECT_FALSE(stale.scheduler.retireCompletedTerminalHoldAfterNativeHold());
    const auto old_receipt = std::chrono::steady_clock::now() -
        std::chrono::seconds(2);
    stale.observeNavigation(2000000, 2000000,
        px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER, old_receipt);
    stale.observeNativeHold();  // duplicate must not refresh old receipt
    EXPECT_FALSE(stale.scheduler.retireCompletedTerminalHoldAfterNativeHold());
    EXPECT_TRUE(stale.hover->terminalHold());
    stale.observeNavigation(1500000, 1500000,
        px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER);
    EXPECT_FALSE(stale.scheduler.retireCompletedTerminalHoldAfterNativeHold());
    EXPECT_TRUE(stale.hover->terminalHold());

    TerminalCompletionFixture wrong("native_hold_wrong_owner");
    wrong.observeSourceExternal();
    wrong.finishWithoutSchedulerTick(
        iii_drone::control::maneuver::MANEUVER_TYPE_CABLE_AWARE_FLY_TO_POSITION,
        "cable_aware_fly_to_position", true);
    wrong.hover->AdoptTerminalHold(wrong.hold, kRequestB);
    wrong.observeNativeHold();
    EXPECT_FALSE(wrong.scheduler.retireCompletedTerminalHoldAfterNativeHold());
    EXPECT_EQ(wrong.hover->terminalHoldBinding().request_identity, kRequestB);

    TerminalCompletionFixture active("native_hold_active_maneuver");
    active.observeSourceExternal();
    active.finishWithoutSchedulerTick(
        iii_drone::control::maneuver::MANEUVER_TYPE_CABLE_AWARE_FLY_TO_POSITION,
        "cable_aware_fly_to_position", true);
    iii_drone::control::maneuver::Maneuver still_running(
        iii_drone::control::maneuver::MANEUVER_TYPE_CABLE_AWARE_FLY_TO_POSITION,
        rclcpp_action::GoalUUID{});
    still_running.request_identity_ = kRequestA;
    still_running.started_ = true;
    active.scheduler.current_maneuver_ = still_running;
    active.observeNativeHold();
    EXPECT_FALSE(active.scheduler.retireCompletedTerminalHoldAfterNativeHold());
    EXPECT_TRUE(active.hover->terminalHold());

    TerminalCompletionFixture successor("native_hold_new_generation");
    successor.observeSourceExternal();
    successor.finishWithoutSchedulerTick(
        iii_drone::control::maneuver::MANEUVER_TYPE_CABLE_AWARE_FLY_TO_POSITION,
        "cable_aware_fly_to_position", true);
    successor.scheduler.beginReferenceExecution("fly_to_position", kRequestB,
        successor.hold->lastCommand());
    successor.observeNativeHold();
    EXPECT_FALSE(successor.scheduler.retireCompletedTerminalHoldAfterNativeHold());
    EXPECT_TRUE(successor.hover->terminalHold());
}

TEST(ManeuverReferenceClientTransaction, PreexistingNativeHoldCannotSeedNewExternalOwnerEpoch) {
    RclcppContext context;
    TerminalCompletionFixture fixture("native_hold_preexisting_at_begin");
    fixture.observeSourceExternal();
    fixture.observeNativeHold();
    fixture.finishWithoutSchedulerTick(
        iii_drone::control::maneuver::MANEUVER_TYPE_CABLE_AWARE_FLY_TO_POSITION,
        "cable_aware_fly_to_position", true);
    EXPECT_EQ(fixture.scheduler.retained_native_hold_epoch_.external_status_timestamp_us, 0U);
    fixture.observeNativeHold(3000000, 2000000);  // native mode never exited
    EXPECT_FALSE(fixture.scheduler.retireCompletedTerminalHoldAfterNativeHold());
    EXPECT_TRUE(fixture.hover->terminalHold());
}

TEST(ManeuverReferenceClientTransaction, ClaimedSuccessorRequiresNewExternalTransitionBeforeNativeHold) {
    RclcppContext context;
    using Transfer = iii_drone_interfaces::srv::TerminalHoldTransfer;
    TerminalCompletionFixture fixture("native_hold_claim_epoch");
    fixture.observeSourceExternal();
    fixture.finishWithoutSchedulerTick(
        iii_drone::control::maneuver::MANEUVER_TYPE_CABLE_AWARE_FLY_TO_POSITION,
        "cable_aware_fly_to_position", true);
    auto query = std::make_shared<Transfer::Request>();
    query->operation = Transfer::Request::OP_QUERY;
    query->consumer_identity = kRequestB;
    auto offer = std::make_shared<Transfer::Response>();
    fixture.scheduler.terminalHoldTransfer(query, offer);
    ASSERT_TRUE(offer->accepted) << offer->reason;
    auto claim = std::make_shared<Transfer::Request>();
    claim->operation = Transfer::Request::OP_CLAIM;
    claim->consumer_identity = kRequestB;
    claim->source_request_identity = offer->source_request_identity;
    claim->source_stream_id = offer->source_stream_id;
    claim->source_ack_sequence = offer->source_ack_sequence;
    auto claimed = std::make_shared<Transfer::Response>();
    fixture.scheduler.terminalHoldTransfer(claim, claimed);
    ASSERT_TRUE(claimed->accepted) << claimed->reason;
    auto & stream = fixture.scheduler.reference_stream_state_;
    stream.sequence = 2;
    stream.recent_references.emplace_back(2, fixture.hold->lastCommand());
    auto applied = std::make_shared<Ack>();
    applied->stream_id = stream.stream_id;
    applied->consumer_identity = kRequestB;
    applied->last_applied_sequence = 2;
    applied->consumer_status = Ack::STATUS_APPLIED;
    fixture.scheduler.acknowledgeReferenceStream(applied);
    ASSERT_TRUE(fixture.scheduler.retained_native_hold_epoch_.claimed_applied);
    // A second consumer claim must never reset the predecessor transition
    // watermark to zero after the first claim cleared its external sample.
    query->consumer_identity = kRequestC;
    auto second_offer = std::make_shared<Transfer::Response>();
    fixture.scheduler.terminalHoldTransfer(query, second_offer);
    ASSERT_TRUE(second_offer->accepted) << second_offer->reason;
    claim->consumer_identity = kRequestC;
    claim->source_ack_sequence = second_offer->source_ack_sequence;
    auto second_claim = std::make_shared<Transfer::Response>();
    fixture.scheduler.terminalHoldTransfer(claim, second_claim);
    ASSERT_TRUE(second_claim->accepted) << second_claim->reason;
    EXPECT_EQ(fixture.scheduler.retained_native_hold_epoch_.minimum_external_transition_us,
        500000U);
    stream.sequence = 3;
    stream.recent_references.emplace_back(3, fixture.hold->lastCommand());
    applied->consumer_identity = kRequestC;
    applied->last_applied_sequence = 3;
    fixture.scheduler.acknowledgeReferenceStream(applied);
    ASSERT_TRUE(fixture.scheduler.retained_native_hold_epoch_.claimed_applied);
    fixture.observeNativeHold();  // delayed predecessor Hold after new ACK
    EXPECT_FALSE(fixture.scheduler.retireCompletedTerminalHoldAfterNativeHold());
    EXPECT_TRUE(fixture.hover->terminalHold());
    fixture.observeNavigation(3000000, 3000000,
        px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_EXTERNAL1);
    fixture.observeNativeHold(4000000, 4000000);
    EXPECT_TRUE(fixture.scheduler.retireCompletedTerminalHoldAfterNativeHold());
    EXPECT_EQ(fixture.query()->reason, "no retained terminal hold");
}

TEST(ManeuverReferenceClientTransaction, NativeHoldRetirementAndSuccessorBeginKeepLatestBinding) {
    RclcppContext context;
    TerminalCompletionFixture retire_first("native_hold_retire_then_begin");
    retire_first.observeSourceExternal();
    retire_first.finishWithoutSchedulerTick(
        iii_drone::control::maneuver::MANEUVER_TYPE_CABLE_AWARE_FLY_TO_POSITION,
        "cable_aware_fly_to_position", true);
    retire_first.observeNativeHold();
    ASSERT_TRUE(retire_first.scheduler.retireCompletedTerminalHoldAfterNativeHold());
    retire_first.scheduler.beginReferenceExecution("fly_to_position", kRequestB,
        retire_first.hold->lastCommand());
    retire_first.scheduler.onReferenceCallbackTokenReacquired();  // late old callback
    auto binding = retire_first.scheduler.reference_callback_struct_->snapshot();
    EXPECT_TRUE(binding.callback);
    EXPECT_EQ(binding.request_identity, kRequestB);
    EXPECT_EQ(binding.execution_id,
        retire_first.scheduler.current_reference_execution_id_.Load());

    TerminalCompletionFixture begin_first("native_hold_begin_then_retire");
    begin_first.observeSourceExternal();
    begin_first.finishWithoutSchedulerTick(
        iii_drone::control::maneuver::MANEUVER_TYPE_CABLE_AWARE_FLY_TO_POSITION,
        "cable_aware_fly_to_position", true);
    begin_first.scheduler.beginReferenceExecution("fly_to_position", kRequestB,
        begin_first.hold->lastCommand());
    begin_first.observeNativeHold();
    EXPECT_FALSE(begin_first.scheduler.retireCompletedTerminalHoldAfterNativeHold());
    binding = begin_first.scheduler.reference_callback_struct_->snapshot();
    EXPECT_TRUE(binding.callback);
    EXPECT_EQ(binding.request_identity, kRequestB);
}

TEST(ManeuverReferenceClientTransaction, StaleExternalEvidenceStillRaisesRepeatedClaimFence) {
    RclcppContext context;
    using Transfer = iii_drone_interfaces::srv::TerminalHoldTransfer;
    TerminalCompletionFixture fixture("native_hold_stale_claim_floor");
    fixture.observeSourceExternal();
    fixture.finishWithoutSchedulerTick(
        iii_drone::control::maneuver::MANEUVER_TYPE_CABLE_AWARE_FLY_TO_POSITION,
        "cable_aware_fly_to_position", true);
    auto offer_for = [&fixture](const std::string & consumer) {
        auto request = std::make_shared<Transfer::Request>();
        request->operation = Transfer::Request::OP_QUERY;
        request->consumer_identity = consumer;
        auto response = std::make_shared<Transfer::Response>();
        fixture.scheduler.terminalHoldTransfer(request, response);
        return response;
    };
    auto claim_offer = [&fixture](const std::string & consumer,
                                const std::shared_ptr<Transfer::Response> & offer,
                                uint64_t applied_sequence) {
        auto request = std::make_shared<Transfer::Request>();
        request->operation = Transfer::Request::OP_CLAIM;
        request->consumer_identity = consumer;
        request->source_request_identity = offer->source_request_identity;
        request->source_stream_id = offer->source_stream_id;
        request->source_ack_sequence = offer->source_ack_sequence;
        auto response = std::make_shared<Transfer::Response>();
        fixture.scheduler.terminalHoldTransfer(request, response);
        if (!response->accepted) return false;
        auto & stream = fixture.scheduler.reference_stream_state_;
        stream.sequence = applied_sequence;
        stream.recent_references.emplace_back(applied_sequence, fixture.hold->lastCommand());
        auto applied = std::make_shared<Ack>();
        applied->stream_id = stream.stream_id;
        applied->consumer_identity = consumer;
        applied->last_applied_sequence = applied_sequence;
        applied->consumer_status = Ack::STATUS_APPLIED;
        fixture.scheduler.acknowledgeReferenceStream(applied);
        return fixture.scheduler.retained_native_hold_epoch_.claimed_applied;
    };
    const auto first = offer_for(kRequestB);
    ASSERT_TRUE(first->accepted) << first->reason;
    ASSERT_TRUE(claim_offer(kRequestB, first, 2));
    fixture.observeNavigation(3000000, 3000000,
        px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_EXTERNAL1,
        std::chrono::steady_clock::now() - std::chrono::seconds(2));
    const auto second = offer_for(kRequestC);
    ASSERT_TRUE(second->accepted) << second->reason;
    ASSERT_TRUE(claim_offer(kRequestC, second, 3));
    EXPECT_EQ(fixture.scheduler.retained_native_hold_epoch_.minimum_external_transition_us,
        3000000U);
    fixture.observeNativeHold(4000000, 4000000);
    EXPECT_FALSE(fixture.scheduler.retireCompletedTerminalHoldAfterNativeHold());
    EXPECT_TRUE(fixture.hover->terminalHold());
    fixture.observeNavigation(5000000, 5000000,
        px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_EXTERNAL2);
    fixture.observeNativeHold(6000000, 6000000);
    EXPECT_TRUE(fixture.scheduler.retireCompletedTerminalHoldAfterNativeHold());
}

// Captures formatted rcutils log lines for the scope of one test.
class ScopedLogCapture {
public:
    ScopedLogCapture() : previous_(rcutils_logging_get_output_handler()) {
        std::lock_guard<std::mutex> lock(mutex());
        entries().clear();
        rcutils_logging_set_output_handler(&ScopedLogCapture::handler);
    }

    ~ScopedLogCapture() {
        rcutils_logging_set_output_handler(previous_);
    }

    size_t count(int severity, const std::string & needle) const {
        std::lock_guard<std::mutex> lock(mutex());
        return static_cast<size_t>(std::count_if(entries().begin(), entries().end(),
            [severity, &needle](const auto & entry) {
                return entry.first == severity &&
                    entry.second.find(needle) != std::string::npos;
            }));
    }

private:
    static void handler(const rcutils_log_location_t *, int severity, const char *,
                        rcutils_time_point_value_t, const char * format, va_list * args) {
        va_list copy;
        va_copy(copy, *args);
        char buffer[4096];
        std::vsnprintf(buffer, sizeof(buffer), format, copy);
        va_end(copy);
        std::lock_guard<std::mutex> lock(mutex());
        entries().emplace_back(severity, buffer);
    }

    static std::mutex & mutex() {
        static std::mutex value;
        return value;
    }

    static std::vector<std::pair<int, std::string>> & entries() {
        static std::vector<std::pair<int, std::string>> value;
        return value;
    }

    rcutils_logging_output_handler_t previous_;
};

struct HoverOnCableIdleHarness {
    // instant_after_cable_landing reproduces the live SIM order: CableLanding
    // succeeds and installs the HoverOnCable callback under its own request,
    // then a non-sustained HoverOnCable succeeds before any active sample.
    explicit HoverOnCableIdleHarness(TerminalCompletionFixture & fixture_in,
                                     bool instant_after_cable_landing = false)
    : fixture(fixture_in),
      server(std::make_shared<iii_drone::control::maneuver::HoverOnCableManeuverServer>(
          &fixture.node, fixture.awareness, "hover_on_cable_idle", 1, 1, fixture.config)),
      maneuver(iii_drone::control::maneuver::MANEUVER_TYPE_HOVER_ON_CABLE,
          rclcpp_action::GoalUUID{}) {
        fixture.hover->ClearTerminalHold();
        fixture.scheduler.Start();
        fixture.scheduler.maneuver_execution_timer_->cancel();
        server->RegisterOnFailCallback([this] { ++hover_failures; });
        fixture.scheduler.registered_maneuvers_[
            iii_drone::control::maneuver::MANEUVER_TYPE_HOVER_ON_CABLE] = server;
        // The Reach Cable external mode owns the goal (non-sustained, 10 s).
        fixture.observeSourceExternal();
        const auto hover_on_cable = server;
        if (instant_after_cable_landing) {
            iii_drone::control::maneuver::Maneuver landing(
                iii_drone::control::maneuver::MANEUVER_TYPE_CABLE_LANDING,
                rclcpp_action::GoalUUID{});
            landing.request_identity_ = kRequestB;
            landing.started_ = true;
            fixture.scheduler.current_maneuver_ = landing;
            fixture.scheduler.beginReferenceExecution("cable_landing", kRequestB);
            const auto landing_execution =
                fixture.scheduler.current_reference_execution_id_.Load();
            fixture.scheduler.reference_callback_struct_->set(
                [](const iii_drone::control::State & state) { return Reference(state); },
                "cable_landing", landing_execution, kRequestB);
            fixture.scheduler.publishReferenceStream();
            applyLatest();
            fixture.scheduler.reference_callback_struct_->set(
                [hover_on_cable](const iii_drone::control::State & state) {
                    return hover_on_cable->GetReference(state);
                }, server->action_name(), landing_execution, kRequestB);
            landing.Terminate(true);
            fixture.scheduler.current_maneuver_ = landing;
            fixture.scheduler.maneuver_execution_timer_->reset();
            fixture.scheduler.maneuverExecutionTimerCallback();
        }
        maneuver.request_identity_ = kRequestA;
        maneuver.maneuver_params_ = std::make_shared<
            iii_drone::control::maneuver::hover_on_cable_maneuver_params_t>(
                1, 0.0, 0.0, 10.0, false);
        maneuver.started_ = true;
        fixture.scheduler.current_maneuver_ = maneuver;
        fixture.scheduler.beginReferenceExecution(server->action_name(), kRequestA);
        execution = fixture.scheduler.current_reference_execution_id_.Load();
        fixture.scheduler.reference_callback_struct_->set(
            [hover_on_cable](const iii_drone::control::State & state) {
                return hover_on_cable->GetReference(state);
            }, server->action_name(), execution, kRequestA);
        if (!instant_after_cable_landing) {
            fixture.scheduler.publishReferenceStream();
            applyLatest();
        }
    }

    ~HoverOnCableIdleHarness() {
        fixture.scheduler.registered_maneuvers_.erase(
            iii_drone::control::maneuver::MANEUVER_TYPE_HOVER_ON_CABLE);
    }

    void applyLatest() {
        auto ack = std::make_shared<Ack>();
        ack->stream_id = fixture.scheduler.reference_stream_state_.stream_id;
        ack->last_applied_sequence = fixture.scheduler.reference_stream_state_.sequence;
        ack->consumer_status = Ack::STATUS_APPLIED;
        fixture.scheduler.acknowledgeReferenceStream(ack);
    }

    void complete() {
        maneuver.Terminate(true);
        fixture.scheduler.current_maneuver_ = maneuver;
        fixture.scheduler.maneuver_execution_timer_->reset();
        fixture.scheduler.maneuverExecutionTimerCallback();
    }

    TerminalCompletionFixture & fixture;
    std::shared_ptr<iii_drone::control::maneuver::HoverOnCableManeuverServer> server;
    iii_drone::control::maneuver::Maneuver maneuver;
    uint64_t execution = 0;
    int hover_failures = 0;
};

TEST(ManeuverReferenceClientTransaction, NativeLandRetiresCompletedHoverOnCableIdleCallbackQuietly) {
    RclcppContext context;
    ScopedLogCapture logs;
    TerminalCompletionFixture fixture("native_land_hover_on_cable_idle", 500, "", 1.0, 1.0, 50);
    HoverOnCableIdleHarness harness(fixture);
    auto & stream = fixture.scheduler.reference_stream_state_;
    ASSERT_TRUE(stream.valid);
    ASSERT_NE(fixture.scheduler.retained_native_hold_epoch_.external_status_timestamp_us, 0U);

    // Completed non-sustained HoverOnCable keeps its callback for duration_s.
    harness.complete();
    ASSERT_EQ(fixture.scheduler.current_maneuver_.Load().maneuver_type(),
        iii_drone::control::maneuver::MANEUVER_TYPE_NONE);
    ASSERT_TRUE(fixture.scheduler.maneuver_server_get_reference_callback_still_registered_.Load());
    const auto stream_id = stream.stream_id;
    harness.applyLatest();
    fixture.scheduler.maneuverExecutionTimerCallback();
    ASSERT_TRUE(stream.valid);
    EXPECT_EQ(stream.stream_id, stream_id);
    harness.applyLatest();

    // Mission schedules PX4 native Land and stops consuming (StopControls).
    fixture.observeNavigation(2000000, 2000000,
        px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND);
    fixture.scheduler.maneuverExecutionTimerCallback();
    EXPECT_FALSE(stream.valid);
    EXPECT_FALSE(fixture.scheduler.reference_callback_struct_->snapshot().callback);
    EXPECT_FALSE(fixture.scheduler.maneuver_server_get_reference_callback_still_registered_.Load());

    // Consumer silence beyond the unchanged ACK timeout finds no live owner.
    std::this_thread::sleep_for(std::chrono::milliseconds(600));
    fixture.scheduler.maneuverExecutionTimerCallback();
    EXPECT_FALSE(stream.valid);
    EXPECT_FALSE(stream.paused);
    EXPECT_EQ(logs.count(RCUTILS_LOG_SEVERITY_ERROR, ""), 0U);
    EXPECT_EQ(logs.count(RCUTILS_LOG_SEVERITY_WARN, "Drone is not offboard"), 0U);
    EXPECT_EQ(logs.count(RCUTILS_LOG_SEVERITY_INFO,
        "idle hover callback owner retired after fresh PX4 native navigation state 18"), 1U);
    EXPECT_EQ(harness.hover_failures, 0);
}

TEST(ManeuverReferenceClientTransaction, NativeLandRetiresInstantHoverOnCableAfterCableLanding) {
    RclcppContext context;
    ScopedLogCapture logs;
    TerminalCompletionFixture fixture("native_land_instant_hover_on_cable", 500, "", 1.0, 1.0, 50);
    HoverOnCableIdleHarness harness(fixture, true);
    auto & stream = fixture.scheduler.reference_stream_state_;
    // The 68 ms HoverOnCable never published while active.
    ASSERT_FALSE(stream.valid);

    // Completion tick: idle retention begins, then the first sample of this
    // execution is published in the same tick.
    harness.complete();
    ASSERT_EQ(fixture.scheduler.current_maneuver_.Load().maneuver_type(),
        iii_drone::control::maneuver::MANEUVER_TYPE_NONE);
    ASSERT_TRUE(stream.valid);
    ASSERT_EQ(stream.execution_id, harness.execution);
    harness.applyLatest();
    fixture.scheduler.maneuverExecutionTimerCallback();
    harness.applyLatest();

    fixture.observeNavigation(2000000, 2000000,
        px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND);
    fixture.scheduler.maneuverExecutionTimerCallback();
    EXPECT_FALSE(stream.valid);
    EXPECT_FALSE(fixture.scheduler.reference_callback_struct_->snapshot().callback);

    std::this_thread::sleep_for(std::chrono::milliseconds(600));
    fixture.scheduler.maneuverExecutionTimerCallback();
    EXPECT_FALSE(stream.valid);
    EXPECT_FALSE(stream.paused);
    EXPECT_EQ(logs.count(RCUTILS_LOG_SEVERITY_ERROR, ""), 0U);
    EXPECT_EQ(logs.count(RCUTILS_LOG_SEVERITY_INFO,
        "idle hover callback owner retired after fresh PX4 native navigation state 18"), 1U);
}

TEST(ManeuverReferenceClientTransaction, NativeLandCannotRetireActiveHoverOnCableAckLoss) {
    RclcppContext context;
    ScopedLogCapture logs;
    TerminalCompletionFixture fixture("native_land_hover_on_cable_active", 500, "", 1.0, 1.0, 50);
    HoverOnCableIdleHarness harness(fixture);
    auto & stream = fixture.scheduler.reference_stream_state_;
    ASSERT_TRUE(stream.valid);
    fixture.observeNavigation(2000000, 2000000,
        px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND);
    EXPECT_FALSE(fixture.scheduler.retireCompletedTerminalHoldAfterNativeHold());
    fixture.scheduler.publishReferenceStream();
    EXPECT_TRUE(stream.valid);
    EXPECT_FALSE(stream.paused);

    // The executing maneuver's ACK loss still pauses the producer and blocks
    // scheduler progression exactly as before.
    std::this_thread::sleep_for(std::chrono::milliseconds(600));
    fixture.scheduler.maneuverExecutionTimerCallback();
    EXPECT_TRUE(stream.valid);
    EXPECT_TRUE(stream.paused);
    EXPECT_EQ(logs.count(RCUTILS_LOG_SEVERITY_ERROR, "missed consumer acknowledgements"), 1U);
    const auto binding = fixture.scheduler.reference_callback_struct_->snapshot();
    EXPECT_TRUE(binding.callback);
    EXPECT_EQ(binding.request_identity, kRequestA);
    EXPECT_EQ(binding.execution_id, harness.execution);
    EXPECT_FALSE(fixture.scheduler.retireCompletedTerminalHoldAfterNativeHold());
}

TEST(ManeuverReferenceClientTransaction, HoverOnCableValidatesAwarenessOnlyForItsExecutingGoal) {
    RclcppContext context;
    ScopedLogCapture logs;
    TerminalCompletionFixture fixture("hover_on_cable_awareness_owner");
    int failures = 0;
    auto server = std::make_shared<iii_drone::control::maneuver::HoverOnCableManeuverServer>(
        &fixture.node, fixture.awareness, "hover_on_cable_awareness", 1, 1, fixture.config);
    server->RegisterOnFailCallback([&failures] { ++failures; });
    // Retained after completion: no executing goal owns the on-cable check.
    (void)server->GetReference(fixture.awareness->GetState());
    EXPECT_EQ(failures, 0);
    EXPECT_EQ(logs.count(RCUTILS_LOG_SEVERITY_WARN, "Drone is not offboard"), 0U);
    server->current_maneuver_ = iii_drone::control::maneuver::Maneuver(
        iii_drone::control::maneuver::MANEUVER_TYPE_HOVER_ON_CABLE, rclcpp_action::GoalUUID{});
    (void)server->GetReference(fixture.awareness->GetState());
    EXPECT_EQ(failures, 1);
    EXPECT_EQ(logs.count(RCUTILS_LOG_SEVERITY_WARN, "Drone is not offboard"), 1U);
}

TEST(ManeuverReferenceClientTransaction, ClaimantTransitionSeenBeforeClaimArmsNativeRetirement) {
    RclcppContext context;
    using Transfer = iii_drone_interfaces::srv::TerminalHoldTransfer;
    for (const uint8_t native_state : {
             px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER,
             px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND}) {
        ScopedLogCapture logs;
        TerminalCompletionFixture fixture(
            native_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND
                ? "claimed_terminal_native_land" : "claimed_terminal_native_hold", 500);
        // FollowWaypointPath completes under Leave Cable (EXTERNAL5).
        fixture.observeSourceExternal();
        fixture.finishWithoutSchedulerTick(
            iii_drone::control::maneuver::MANEUVER_TYPE_FOLLOW_WAYPOINT_PATH,
            "follow_waypoint_path", true);
        // PX4 activates Inspection Demo, which then claims the retained hold.
        fixture.observeNavigation(1500000, 1500000,
            px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_EXTERNAL1);
        auto query = std::make_shared<Transfer::Request>();
        query->operation = Transfer::Request::OP_QUERY;
        query->consumer_identity = kRequestB;
        auto offer = std::make_shared<Transfer::Response>();
        fixture.scheduler.terminalHoldTransfer(query, offer);
        ASSERT_TRUE(offer->accepted) << offer->reason;
        auto claim = std::make_shared<Transfer::Request>();
        claim->operation = Transfer::Request::OP_CLAIM;
        claim->consumer_identity = kRequestB;
        claim->source_request_identity = offer->source_request_identity;
        claim->source_stream_id = offer->source_stream_id;
        claim->source_ack_sequence = offer->source_ack_sequence;
        auto claimed = std::make_shared<Transfer::Response>();
        fixture.scheduler.terminalHoldTransfer(claim, claimed);
        ASSERT_TRUE(claimed->accepted) << claimed->reason;
        auto & stream = fixture.scheduler.reference_stream_state_;
        stream.sequence = 2;
        stream.recent_references.emplace_back(2, fixture.hold->lastCommand());
        auto applied = std::make_shared<Ack>();
        applied->stream_id = stream.stream_id;
        applied->consumer_identity = kRequestB;
        applied->last_applied_sequence = 2;
        applied->consumer_status = Ack::STATUS_APPLIED;
        fixture.scheduler.acknowledgeReferenceStream(applied);
        ASSERT_TRUE(fixture.scheduler.retained_native_hold_epoch_.claimed_applied);
        EXPECT_EQ(fixture.scheduler.retained_native_hold_epoch_.external_nav_transition_us,
            1500000U);
        EXPECT_FALSE(fixture.scheduler.retireCompletedTerminalHoldAfterNativeHold());

        // px4.hold (or Land) deactivates Inspection Demo; the consumer stops.
        fixture.observeNavigation(2000000, 2000000, native_state);
        EXPECT_TRUE(fixture.scheduler.retireCompletedTerminalHoldAfterNativeHold());
        EXPECT_EQ(fixture.query()->reason, "no retained terminal hold");
        std::this_thread::sleep_for(std::chrono::milliseconds(550));
        auto measured = fixture.awareness->measured_odometry_.Load();
        ASSERT_TRUE(measured);
        measured->receipt_stamp = fixture.node.now();
        measured->source_sample_timestamp_us += 550000;
        fixture.awareness->measured_odometry_.Store(measured);
        fixture.scheduler.publishReferenceStream();
        EXPECT_EQ(fixture.hold->phase(),
            iii_drone::control::maneuver::TerminalTrackingHold::Phase::Tracking);
        EXPECT_EQ(logs.count(RCUTILS_LOG_SEVERITY_ERROR, ""), 0U);
        EXPECT_EQ(logs.count(RCUTILS_LOG_SEVERITY_INFO,
            "terminal hold owner retired after fresh PX4 native navigation state"), 1U);
    }
}

TEST(ManeuverReferenceClientTransaction, NativeLandRetiresUnclaimedCompletedTerminalOwner) {
    RclcppContext context;
    TerminalCompletionFixture fixture("unclaimed_terminal_native_land");
    fixture.observeSourceExternal();
    fixture.finishWithoutSchedulerTick(
        iii_drone::control::maneuver::MANEUVER_TYPE_FOLLOW_WAYPOINT_PATH,
        "follow_waypoint_path", true);
    // Offboard is a Core-command consumer, never native evidence.
    fixture.observeNavigation(1500000, 1500000,
        px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);
    EXPECT_FALSE(fixture.scheduler.retireCompletedTerminalHoldAfterNativeHold());
    EXPECT_TRUE(fixture.hover->terminalHold());
    fixture.observeNavigation(2000000, 2000000,
        px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND);
    EXPECT_TRUE(fixture.scheduler.retireCompletedTerminalHoldAfterNativeHold());
    EXPECT_FALSE(fixture.hover->terminalHold());
}

TEST(ManeuverReferenceClientTransaction, CompletionTransferRejectsWrongOwnerAndGeneration) {
    RclcppContext context;
    TerminalCompletionFixture wrong_owner("terminal_wrong_owner");
    wrong_owner.hover->AdoptTerminalHold(wrong_owner.hold, kRequestB);
    wrong_owner.finishWithoutSchedulerTick(
        iii_drone::control::maneuver::MANEUVER_TYPE_FOLLOW_WAYPOINT_PATH,
        "follow_waypoint_path", false, false);
    const auto wrong_offer = wrong_owner.query();
    EXPECT_FALSE(wrong_offer->accepted);
    EXPECT_NE(wrong_offer->reason, "terminal callback finalizing");
    wrong_owner.scheduler.onReferenceCallbackTokenReacquired();
    EXPECT_EQ(wrong_owner.hover->terminalHoldBinding().request_identity, kRequestB);
    EXPECT_FALSE(wrong_owner.query()->accepted);

    TerminalCompletionFixture stale_generation("terminal_stale_generation");
    stale_generation.finishWithoutSchedulerTick(
        iii_drone::control::maneuver::MANEUVER_TYPE_CABLE_AWARE_FLY_TO_POSITION,
        "cable_aware_fly_to_position", true, false);
    stale_generation.scheduler.current_reference_execution_id_.Store(
        stale_generation.scheduler.current_reference_execution_id_.Load() + 1);
    const auto stale_offer = stale_generation.query();
    EXPECT_FALSE(stale_offer->accepted);
    EXPECT_NE(stale_offer->reason, "terminal callback finalizing");
    stale_generation.scheduler.onReferenceCallbackTokenReacquired();
    EXPECT_FALSE(stale_generation.query()->accepted);

    TerminalCompletionFixture missing_callback("terminal_missing_callback");
    missing_callback.finishWithoutSchedulerTick(
        iii_drone::control::maneuver::MANEUVER_TYPE_CABLE_AWARE_FLY_TO_POSITION,
        "cable_aware_fly_to_position", true, false);
    const auto expected = missing_callback.scheduler.reference_callback_struct_->snapshot();
    missing_callback.scheduler.reference_callback_struct_->set(
        nullptr, expected.reference_provider_name,
        expected.execution_id, expected.request_identity);
    const auto missing_offer = missing_callback.query();
    EXPECT_FALSE(missing_offer->accepted);
    EXPECT_NE(missing_offer->reason, "terminal callback finalizing");
    missing_callback.scheduler.onReferenceCallbackTokenReacquired();
    EXPECT_FALSE(missing_callback.query()->accepted);
}

TEST(ManeuverReferenceClientTransaction, CompletionTransferRejectsDegradedOrStaleAck) {
    RclcppContext context;
    TerminalCompletionFixture degraded("terminal_degraded_completion");
    degraded.finishWithoutSchedulerTick(
        iii_drone::control::maneuver::MANEUVER_TYPE_CABLE_AWARE_FLY_TO_POSITION,
        "cable_aware_fly_to_position", true);
    degraded.hold->Fail("test terminal fault");
    const auto degraded_offer = degraded.query();
    EXPECT_FALSE(degraded_offer->accepted);
    EXPECT_EQ(degraded_offer->reason, "retained terminal hold degraded");

    TerminalCompletionFixture stale_ack("terminal_stale_ack_completion");
    stale_ack.finishWithoutSchedulerTick(
        iii_drone::control::maneuver::MANEUVER_TYPE_CABLE_AWARE_FLY_TO_POSITION,
        "cable_aware_fly_to_position", true);
    stale_ack.scheduler.reference_stream_state_.last_ack -= std::chrono::seconds(20);
    const auto stale_offer = stale_ack.query();
    EXPECT_FALSE(stale_offer->accepted);
    EXPECT_EQ(stale_offer->reason, "retained hold lacks a fresh applied generation");
}

TEST(ManeuverReferenceClientTransaction, RetentionRetriesOnlyExactCallbackFinalization) {
    RclcppContext context;
    ClientFixture fixture("terminal_retention_finalizing_client");
    auto producer = std::make_shared<rclcpp::Node>(
        "terminal_retention_finalizing_producer", "/control/maneuver_controller");
    using Transfer = iii_drone_interfaces::srv::TerminalHoldTransfer;
    std::atomic<int> query_count{0};
    auto service = producer->create_service<Transfer>(
        "terminal_hold_transfer",
        [&query_count](const std::shared_ptr<Transfer::Request> request,
            std::shared_ptr<Transfer::Response> response) {
            if (request->operation != Transfer::Request::OP_QUERY) return;
            if (++query_count == 1) {
                response->reason = "terminal callback finalizing";
                return;
            }
            response->accepted = true;
            response->source_request_identity = kRequestA;
            response->source_stream_id = "terminal:completed";
            response->source_ack_sequence = 1;
            response->reference = ReferenceAdapter(finiteReference(0.24, 0.0)).ToMsg();
        });
    fixture.executor.add_node(producer);
    std::thread spinner([&fixture] { fixture.executor.spin(); });
    ASSERT_TRUE(fixture.client.BeginManeuverGoalHandoff(kRequestA));
    for (int attempt = 0; attempt < 50 &&
         !fixture.client.terminal_hold_transfer_client_->service_is_ready(); ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const auto retention = fixture.client.RetainCompletedTerminalHold(kRequestA, 500);
    EXPECT_EQ(retention, ManeuverReferenceClient::TerminalHoldRetention::Retained);
    EXPECT_EQ(query_count.load(), 2);
    EXPECT_TRUE(fixture.client.terminalHoldContinuityRequired());
    fixture.executor.cancel();
    spinner.join();
    fixture.executor.remove_node(producer);
}

TEST(ManeuverReferenceClientTransaction, AdoptionRetriesOnlyExactCallbackFinalization) {
    RclcppContext context;
    ClientFixture fixture("terminal_adoption_finalizing_client");
    auto producer = std::make_shared<rclcpp::Node>(
        "terminal_adoption_finalizing_producer", "/control/maneuver_controller");
    using Transfer = iii_drone_interfaces::srv::TerminalHoldTransfer;
    std::atomic<int> query_count{0};
    std::atomic<int> claim_count{0};
    auto service = producer->create_service<Transfer>(
        "terminal_hold_transfer",
        [&query_count, &claim_count](const std::shared_ptr<Transfer::Request> request,
            std::shared_ptr<Transfer::Response> response) {
            if (request->operation == Transfer::Request::OP_QUERY && ++query_count == 1) {
                response->reason = "terminal callback finalizing";
                return;
            }
            if (request->operation == Transfer::Request::OP_CLAIM) ++claim_count;
            response->accepted = true;
            response->source_request_identity = kRequestA;
            response->source_stream_id = "terminal:completed";
            response->source_ack_sequence = 1;
            response->reference = ReferenceAdapter(finiteReference(0.24, 0.0)).ToMsg();
        });
    fixture.executor.add_node(producer);
    std::thread spinner([&fixture] { fixture.executor.spin(); });
    fixture.client.AcquireReferenceControl();
    for (int attempt = 0; attempt < 50 &&
         !fixture.client.terminal_hold_transfer_client_->service_is_ready(); ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const auto adoption = fixture.client.TryAdoptTerminalHold(100);
    EXPECT_EQ(adoption, ManeuverReferenceClient::TerminalHoldAdoption::Adopted);
    EXPECT_EQ(query_count.load(), 2);
    EXPECT_EQ(claim_count.load(), 1);
    fixture.executor.cancel();
    spinner.join();
    fixture.executor.remove_node(producer);
}

TEST(ManeuverReferenceClientTransaction, NativeHoldNoOfferStartsIncomingModeFromMeasuredState) {
    RclcppContext context;
    TerminalCompletionFixture core("native_hold_no_offer_core");
    core.observeSourceExternal();
    core.finishWithoutSchedulerTick(
        iii_drone::control::maneuver::MANEUVER_TYPE_CABLE_AWARE_FLY_TO_POSITION,
        "cable_aware_fly_to_position", true);
    core.observeNativeHold();
    ASSERT_EQ(core.query()->reason, "no retained terminal hold");

    ClientFixture incoming("native_hold_no_offer_incoming");
    incoming.client.reference_.Store(finiteReference(0.24, 0.0));
    auto producer = std::make_shared<rclcpp::Node>(
        "native_hold_no_offer_service", "/control/maneuver_controller");
    using Transfer = iii_drone_interfaces::srv::TerminalHoldTransfer;
    auto service = producer->create_service<Transfer>(
        "terminal_hold_transfer",
        [&core](const std::shared_ptr<Transfer::Request> request,
            std::shared_ptr<Transfer::Response> response) {
            core.scheduler.terminalHoldTransfer(request, response);
        });
    incoming.executor.add_node(producer);
    std::thread spinner([&incoming] { incoming.executor.spin(); });
    incoming.client.AcquireReferenceControl();
    for (int attempt = 0; attempt < 50 &&
         !incoming.client.terminal_hold_transfer_client_->service_is_ready(); ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const auto adoption = incoming.client.TryAdoptTerminalHold(250);
    EXPECT_EQ(adoption, ManeuverReferenceClient::TerminalHoldAdoption::NoOffer);
    EXPECT_LT(incoming.client.reference_.Load().position().norm(), 1.0e-6);
    EXPECT_FALSE(incoming.client.terminalHoldContinuityRequired());
    incoming.executor.cancel();
    spinner.join();
    incoming.executor.remove_node(producer);
}

TEST(ManeuverReferenceClientTransaction, LiveCoreTerminalProducerTransfersToDistinctModeConsumer) {
    auto isolated_context = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions init_options;
    init_options.set_domain_id(216);
    isolated_context->init(0, nullptr, init_options);
    rclcpp::NodeOptions node_options;
    node_options.context(isolated_context);
    rclcpp_lifecycle::LifecycleNode producer(
        "maneuver_controller", "/control/maneuver_controller", node_options);
    auto config = makeConfiguration();
    auto awareness = std::make_shared<iii_drone::control::CombinedDroneAwarenessHandler>(
        config, std::make_shared<tf2_ros::Buffer>(producer.get_clock()), &producer);
    iii_drone::control::maneuver::ManeuverScheduler scheduler(
        &producer, awareness, config,
        producer.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
    using Transfer = iii_drone_interfaces::srv::TerminalHoldTransfer;
    auto transfer_service = producer.create_service<Transfer>(
        "terminal_hold_transfer",
        [&scheduler](const std::shared_ptr<Transfer::Request> request,
            std::shared_ptr<Transfer::Response> response) {
            scheduler.terminalHoldTransfer(request, response);
        }, rclcpp::ServicesQoS(), scheduler.get_reference_callback_group_);
    auto hover = std::make_shared<iii_drone::control::maneuver::HoverManeuverServer>(
        &producer, awareness, "hover", 1, 1, false);
    scheduler.registered_maneuvers_[iii_drone::control::maneuver::MANEUVER_TYPE_HOVER] = hover;
    auto hold = std::make_shared<iii_drone::control::maneuver::TerminalTrackingHold>(
        Reference(point_t(0.24F, 0.0F, 0.0F), 0.0), awareness,
        producer.get_clock(),
        iii_drone::control::maneuver::TerminalTrackingHold::Clearance{}, 0.0);
    hover->AdoptTerminalHold(hold, kRequestA);
    const auto execution = scheduler.reference_callback_struct_->beginExecution(
        "hover", kRequestA, [hold](const iii_drone::control::State &) {
            return hold->GetReference();
        });
    scheduler.current_reference_execution_id_.Store(execution);
    scheduler.maneuver_server_get_reference_callback_still_registered_ = true;

    auto consumer_node = std::make_shared<rclcpp::Node>(
        "terminal_successor_mode_test", node_options);
    auto history = std::make_shared<iii_drone::utils::History<VehicleOdometryAdapter>>(4);
    history->Store(stationaryVehicleState());
    ManeuverReferenceClient consumer(
        consumer_node.get(), history, config,
        consumer_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
    rclcpp::ExecutorOptions executor_options;
    executor_options.context = isolated_context;
    rclcpp::executors::MultiThreadedExecutor executor(executor_options, 2);
    executor.add_node(producer.get_node_base_interface());
    executor.add_node(consumer_node);
    std::thread spinner([&executor] { executor.spin(); });
    for (int attempt = 0; attempt < 50 &&
         scheduler.reference_stream_publisher_->get_subscription_count() == 0; ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_GT(scheduler.reference_stream_publisher_->get_subscription_count(), 0U);

    for (int tick = 0; tick < 35; ++tick) {
        iii_drone::control::MeasuredOdometrySnapshot measured;
        measured.receipt_stamp = producer.now();
        measured.state = iii_drone::control::State(
            point_t::Zero(), vector_t::Zero(), 0.0, vector_t::Zero(), measured.receipt_stamp);
        measured.source_sample_timestamp_us = 1000000 + tick * 50000;
        awareness->measured_odometry_.Store(
            std::optional<iii_drone::control::MeasuredOdometrySnapshot>(measured));
        scheduler.publishReferenceStream();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    EXPECT_EQ(hold->phase(), iii_drone::control::maneuver::TerminalTrackingHold::Phase::Tracking);
    EXPECT_GT((hold->lastCommand().position() - hold->nominalReference().position()).norm(), 1.0e-5);
    const auto source_stream_id = scheduler.reference_stream_state_.stream_id;
    auto source_ack = std::make_shared<Ack>();
    source_ack->stream_id = source_stream_id;
    source_ack->last_applied_sequence = scheduler.reference_stream_state_.sequence;
    source_ack->consumer_status = Ack::STATUS_APPLIED;
    scheduler.acknowledgeReferenceStream(source_ack);
    EXPECT_EQ(scheduler.reference_stream_state_.last_ack_sequence,
        scheduler.reference_stream_state_.sequence);
    EXPECT_TRUE(scheduler.reference_stream_state_.last_ack_reference_valid);
    auto diagnostic_query = std::make_shared<iii_drone_interfaces::srv::TerminalHoldTransfer::Request>();
    diagnostic_query->operation = diagnostic_query->OP_QUERY;
    diagnostic_query->consumer_identity = kRequestB;
    auto diagnostic_offer = std::make_shared<iii_drone_interfaces::srv::TerminalHoldTransfer::Response>();
    scheduler.terminalHoldTransfer(diagnostic_query, diagnostic_offer);
    EXPECT_TRUE(diagnostic_offer->accepted) << diagnostic_offer->reason;
    consumer.AcquireReferenceControl();
    EXPECT_TRUE(consumer.terminal_hold_transfer_client_->service_is_ready());
    const auto remote_offer = consumer.requestTerminalHoldTransfer(*diagnostic_query, 500);
    EXPECT_TRUE(remote_offer);
    if (remote_offer) {
        EXPECT_TRUE(remote_offer->accepted) << remote_offer->reason;
    }
    const auto adoption = consumer.TryAdoptTerminalHold(500);
    EXPECT_EQ(adoption, ManeuverReferenceClient::TerminalHoldAdoption::Adopted);
    if (adoption == ManeuverReferenceClient::TerminalHoldAdoption::Adopted) {
        scheduler.publishReferenceStream();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        const auto accepted = consumer.GetReference(0.05, [] {});
        EXPECT_LT((accepted.position() - hold->lastCommand().position()).norm(), 1.0e-3);
        source_ack->last_applied_sequence = scheduler.reference_stream_state_.sequence;
        scheduler.acknowledgeReferenceStream(source_ack);  // old source identity is rejected
        for (int attempt = 0; attempt < 50 &&
             scheduler.reference_stream_state_.claim_ack_pending; ++attempt) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        EXPECT_FALSE(scheduler.reference_stream_state_.claim_ack_pending);
        EXPECT_EQ(scheduler.reference_stream_state_.claimed_consumer_identity,
            consumer.terminal_consumer_identity_);
        const auto seed = hold->lastCommand();
        scheduler.beginReferenceExecution("fly_to_position", kRequestB, seed);
        scheduler.maneuver_server_get_reference_callback_still_registered_ = true;
        scheduler.publishReferenceStream();
        EXPECT_LT((scheduler.reference_stream_state_.latest_reference.position() - seed.position()).norm(), 1.0e-6);
        EXPECT_LT((scheduler.reference_stream_state_.latest_reference.velocity() - seed.velocity()).norm(), 1.0e-6);
        EXPECT_LT((scheduler.reference_stream_state_.latest_reference.acceleration() - seed.acceleration()).norm(), 1.0e-6);
        EXPECT_EQ(hover->terminalHoldBinding().request_identity, kRequestA);
    }
    executor.cancel();
    spinner.join();
    executor.remove_node(consumer_node);
    executor.remove_node(producer.get_node_base_interface());
    isolated_context->shutdown("terminal transfer test complete");
}

// An accepted goal is copied separately into the scheduler and action worker.
// The scheduler alone calls Start(), after popping its queued copy.
struct AcceptedCableGoal {
    using Action = iii_drone_interfaces::action::CableAwareFlyToPosition;
    using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

    explicit AcceptedCableGoal(const std::string & name)
    : server_node(std::make_shared<rclcpp::Node>(name + "_server")),
      client_node(std::make_shared<rclcpp::Node>(name + "_client")) {
        const std::string action_name = "/" + name + "/cable_goal";
        server = rclcpp_action::create_server<Action>(
            server_node, action_name,
            [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>) {
                return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
            },
            [](const std::shared_ptr<GoalHandle>) {
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](const std::shared_ptr<GoalHandle> accepted) { handle = accepted; });
        client = rclcpp_action::create_client<Action>(client_node, action_name);
        executor.add_node(server_node);
        executor.add_node(client_node);
    }

    ~AcceptedCableGoal() {
        executor.remove_node(client_node);
        executor.remove_node(server_node);
    }

    std::shared_ptr<GoalHandle> accept(const std::string & request_identity) {
        if (!client->wait_for_action_server(std::chrono::seconds(2))) return nullptr;
        Action::Goal goal;
        goal.request_identity = request_identity;
        goal.frame_id = "world";
        const auto future = client->async_send_goal(goal);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (std::chrono::steady_clock::now() < deadline &&
               (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready ||
                !handle)) {
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        return handle;
    }

    rclcpp::Node::SharedPtr server_node;
    rclcpp::Node::SharedPtr client_node;
    rclcpp_action::Server<Action>::SharedPtr server;
    rclcpp_action::Client<Action>::SharedPtr client;
    rclcpp::executors::SingleThreadedExecutor executor;
    std::shared_ptr<GoalHandle> handle;
};

struct AcceptedHoverGoal {
    using Action = iii_drone_interfaces::action::Hover;
    using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

    explicit AcceptedHoverGoal(const std::string & name)
    : server_node(std::make_shared<rclcpp::Node>(name + "_hover_server")),
      client_node(std::make_shared<rclcpp::Node>(name + "_hover_client")) {
        const std::string action_name = "/" + name + "/hover_goal";
        server = rclcpp_action::create_server<Action>(
            server_node, action_name,
            [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>) {
                return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
            },
            [](const std::shared_ptr<GoalHandle>) {
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](const std::shared_ptr<GoalHandle> accepted) { handle = accepted; });
        client = rclcpp_action::create_client<Action>(client_node, action_name);
        executor.add_node(server_node);
        executor.add_node(client_node);
    }

    ~AcceptedHoverGoal() {
        executor.remove_node(client_node);
        executor.remove_node(server_node);
    }

    std::shared_ptr<GoalHandle> accept(const std::string & request_identity) {
        if (!client->wait_for_action_server(std::chrono::seconds(2))) return nullptr;
        Action::Goal goal;
        goal.request_identity = request_identity;
        goal.duration_s = 30.0F;
        goal.sustain_duration_s = 0.0F;
        goal.sustain_action = false;
        const auto future = client->async_send_goal(goal);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (std::chrono::steady_clock::now() < deadline &&
               (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready ||
                !handle)) {
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        return handle;
    }

    rclcpp::Node::SharedPtr server_node;
    rclcpp::Node::SharedPtr client_node;
    rclcpp_action::Server<Action>::SharedPtr server;
    rclcpp_action::Client<Action>::SharedPtr client;
    rclcpp::executors::SingleThreadedExecutor executor;
    std::shared_ptr<GoalHandle> handle;
};

TEST(ManeuverReferenceClientTransaction, ObjectStoppedSeedKeepsAllChannelsInNanHover) {
    RclcppContext context;
    TerminalCompletionFixture fixture("object_stopped_hover_seed");
    auto hover = std::make_shared<iii_drone::control::maneuver::HoverManeuverServer>(
        &fixture.node, fixture.awareness, "hover_finite_seed", 1, 1, true);
    AcceptedHoverGoal accepted("object_stopped_hover_goal");
    const auto handle = accepted.accept(kRequestC);
    ASSERT_TRUE(handle);
    auto maneuver = iii_drone::control::maneuver::Maneuver::FromGoalHandle<
        AcceptedHoverGoal::Action>(handle);
    const Reference rest(point_t(0.30F, 0.0F, 1.5F), 0.2,
        vector_t::Zero(), 0.0, vector_t::Zero(), 0.0, fixture.node.now());
    ASSERT_TRUE(hover->StageTerminalStartReference(kRequestC, rest));
    hover->startExecution(maneuver);
    const Reference output = hover->GetReference(fixture.awareness->GetState());
    EXPECT_TRUE(output.position().allFinite());
    EXPECT_TRUE(output.velocity().allFinite());
    EXPECT_TRUE(output.acceleration().allFinite());
    EXPECT_NEAR((output.position() - rest.position()).norm(), 0.0, 1.0e-5);
    EXPECT_NEAR((output.velocity() - rest.velocity()).norm(), 0.0, 1.0e-5);
    EXPECT_NEAR((output.acceleration() - rest.acceleration()).norm(), 0.0, 1.0e-5);
    EXPECT_NEAR(output.yaw(), rest.yaw(), 1.0e-5);
    EXPECT_NEAR(output.yaw_rate(), rest.yaw_rate(), 1.0e-5);
    EXPECT_NEAR(output.yaw_acceleration(), rest.yaw_acceleration(), 1.0e-5);
}

TEST(ManeuverReferenceClientTransaction, PhasedTerminalHoverCannotSucceedBeforeOwnAppliedGeneration) {
    RclcppContext context;
    TerminalCompletionFixture fixture(
        "maneuver_controller", 500, "/control/maneuver_controller");
    fixture.finishWithoutSchedulerTick(
        iii_drone::control::maneuver::MANEUVER_TYPE_CABLE_AWARE_FLY_TO_POSITION,
        "cable_aware_fly_to_position", true);
    ClientFixture consumer("phased_terminal_hover_consumer");
    ASSERT_TRUE(consumer.client.BeginManeuverGoalHandoff(kRequestA));
    ASSERT_TRUE(consumer.client.ConfirmManeuverGoalHandoff(kRequestA));
    auto predecessor = stream(consumer.node, "ftp:terminal", 1,
        fixture.hold->lastCommand(), kRequestA);
    predecessor->terminal_hold_active = true;
    consumer.client.receiveReferenceStream(predecessor);
    const auto predecessor_command = consumer.client.GetReference(0.2, [] {});
    ASSERT_EQ(consumer.client.active_request_identity_, kRequestA);
    ASSERT_TRUE(consumer.client.BeginManeuverGoalHandoff(kRequestB));
    ASSERT_TRUE(consumer.client.ConfirmManeuverGoalHandoff(kRequestB));
    fixture.scheduler.Start();
    fixture.scheduler.maneuver_execution_timer_->cancel();
    fixture.scheduler.registered_maneuvers_.erase(
        iii_drone::control::maneuver::MANEUVER_TYPE_HOVER);
    fixture.scheduler.RegisterManeuverServer(
        iii_drone::control::maneuver::MANEUVER_TYPE_HOVER, fixture.hover);
    AcceptedHoverGoal goal("phased_terminal_hover");
    const auto handle = goal.accept(kRequestB);
    ASSERT_TRUE(handle);
    using Maneuver = iii_drone::control::maneuver::Maneuver;
    Maneuver accepted = Maneuver::FromGoalHandle<AcceptedHoverGoal::Action>(handle);
    Maneuver worker = Maneuver::FromGoalHandle<AcceptedHoverGoal::Action>(handle);
    ASSERT_TRUE(fixture.scheduler.maneuver_queue_->Push(accepted));
    fixture.scheduler.maneuverExecutionTimerCallback();  // retire predecessor
    fixture.scheduler.maneuverExecutionTimerCallback();  // start Hover generation
    ASSERT_TRUE(fixture.scheduler.current_maneuver_.Load().started());
    ASSERT_EQ(fixture.scheduler.reference_stream_state_.request_identity, kRequestB);
    ASSERT_FALSE(fixture.scheduler.reference_stream_state_.ack_seen);

    std::promise<bool> worker_started;
    auto started = worker_started.get_future();
    std::promise<bool> worker_done;
    auto done = worker_done.get_future();
    std::thread worker_thread([&] {
        if (!fixture.hover->reference_callback_token_->Acquire(2000)) {
            worker_started.set_value(false);
            worker_done.set_value(false);
            return;
        }
        fixture.hover->startExecution(worker);
        const auto execution = fixture.scheduler.current_reference_execution_id_.Load();
        fixture.hover->reference_callback_token_->resource().set(
            [&fixture](const iii_drone::control::State & state) {
                return fixture.hover->GetReference(state);
            }, fixture.hover->action_name(), execution, kRequestB);
        worker_started.set_value(true);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        bool success = false;
        while (std::chrono::steady_clock::now() < deadline) {
            if (fixture.hover->hasSucceeded(worker)) {
                worker.Terminate(true);
                fixture.scheduler.onManeuverCompleted(worker);
                success = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        fixture.hover->reference_callback_token_->Release();
        worker_done.set_value(success);
    });
    for (int attempt = 0; attempt < 200 &&
         !fixture.scheduler.reference_callback_token_.has_requested_token(
             fixture.hover->action_name()); ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    const bool token_requested =
        fixture.scheduler.reference_callback_token_.has_requested_token(
            fixture.hover->action_name());
    if (token_requested) fixture.scheduler.maneuverExecutionTimerCallback();
    const bool token_acquired = started.get();
    if (token_acquired) {
        consumer.executor.add_node(fixture.node.get_node_base_interface());
        std::thread spinner([&consumer] { consumer.executor.spin(); });
        for (int attempt = 0; attempt < 50 &&
             fixture.scheduler.reference_stream_publisher_->get_subscription_count() == 0;
             ++attempt) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        fixture.scheduler.publishReferenceStream();
        std::promise<Reference> applied_promise;
        auto applied_result = applied_promise.get_future();
        rclcpp::TimerBase::SharedPtr consumer_tick;
        consumer_tick = consumer.node.create_wall_timer(
            std::chrono::milliseconds(200), [&] {
                consumer_tick->cancel();
                applied_promise.set_value(consumer.client.GetReference(0.2, [] {}));
            });
        const bool completed_before_consumer_tick =
            done.wait_for(std::chrono::milliseconds(170)) == std::future_status::ready;
        EXPECT_FALSE(completed_before_consumer_tick);
        EXPECT_EQ(consumer.client.active_request_identity_, kRequestA);
        EXPECT_LT((consumer.client.reference_.Load().position() -
            predecessor_command.position()).norm(), 1.0e-6);
        const bool consumer_applied =
            applied_result.wait_for(std::chrono::seconds(2)) == std::future_status::ready;
        if (consumer_applied) {
            const auto applied = applied_result.get();
            EXPECT_TRUE(applied.position().allFinite());
        }
        const bool completion_arrived =
            done.wait_for(std::chrono::seconds(2)) == std::future_status::ready;
        EXPECT_TRUE(consumer_applied);
        EXPECT_TRUE(completion_arrived);
        if (completion_arrived) {
            EXPECT_TRUE(done.get());
            EXPECT_TRUE(fixture.scheduler.current_maneuver_.Load().success());
            EXPECT_TRUE(fixture.scheduler.current_maneuver_.Load().terminated());
            EXPECT_EQ(consumer.client.RetainCompletedTerminalHold(kRequestB, 150),
                ManeuverReferenceClient::TerminalHoldRetention::Retained);
            EXPECT_TRUE(consumer.client.BeginManeuverGoalHandoff(kRequestC));
        }
        consumer_tick->cancel();
        consumer.executor.cancel();
        spinner.join();
        consumer.executor.remove_node(fixture.node.get_node_base_interface());
    }
    worker_thread.join();
    EXPECT_TRUE(token_requested);
    EXPECT_TRUE(token_acquired);
    fixture.hover->Stop();
    fixture.scheduler.registered_maneuvers_.erase(
        iii_drone::control::maneuver::MANEUVER_TYPE_HOVER);
}

TEST(ManeuverReferenceClientTransaction, TerminalHoverSuccessRejectsUnownedOrNegativeAcknowledgement) {
    RclcppContext context;
    TerminalCompletionFixture fixture("hover_ack_fences", 500);
    fixture.finishWithoutSchedulerTick(
        iii_drone::control::maneuver::MANEUVER_TYPE_CABLE_AWARE_FLY_TO_POSITION,
        "cable_aware_fly_to_position", true);
    fixture.scheduler.Start();
    fixture.scheduler.maneuver_execution_timer_->cancel();
    fixture.scheduler.registered_maneuvers_.erase(
        iii_drone::control::maneuver::MANEUVER_TYPE_HOVER);
    fixture.scheduler.RegisterManeuverServer(
        iii_drone::control::maneuver::MANEUVER_TYPE_HOVER, fixture.hover);
    AcceptedHoverGoal goal("hover_ack_fences");
    const auto handle = goal.accept(kRequestB);
    ASSERT_TRUE(handle);
    using Maneuver = iii_drone::control::maneuver::Maneuver;
    Maneuver accepted = Maneuver::FromGoalHandle<AcceptedHoverGoal::Action>(handle);
    Maneuver worker = Maneuver::FromGoalHandle<AcceptedHoverGoal::Action>(handle);
    ASSERT_TRUE(fixture.scheduler.maneuver_queue_->Push(accepted));
    fixture.scheduler.maneuverExecutionTimerCallback();
    fixture.scheduler.maneuverExecutionTimerCallback();
    fixture.hover->startExecution(worker);
    fixture.scheduler.publishReferenceStream();
    auto & state = fixture.scheduler.reference_stream_state_;
    ASSERT_TRUE(state.valid);
    ASSERT_GT(state.sequence, 0U);
    EXPECT_FALSE(fixture.hover->hasSucceeded(worker));

    auto ack = std::make_shared<Ack>();
    ack->stream_id = "terminal:completed";
    ack->last_applied_sequence = 1;
    ack->consumer_status = Ack::STATUS_APPLIED;
    fixture.scheduler.acknowledgeReferenceStream(ack);
    EXPECT_FALSE(fixture.hover->hasSucceeded(worker));
    ack->stream_id = state.stream_id;
    ack->last_applied_sequence = state.sequence + 1;
    fixture.scheduler.acknowledgeReferenceStream(ack);
    EXPECT_FALSE(fixture.hover->hasSucceeded(worker));
    ack->last_applied_sequence = 0;  // not a published command
    fixture.scheduler.acknowledgeReferenceStream(ack);
    EXPECT_FALSE(fixture.hover->hasSucceeded(worker));

    ack->last_applied_sequence = state.sequence;
    fixture.scheduler.acknowledgeReferenceStream(ack);
    ASSERT_TRUE(state.ack_seen);
    EXPECT_TRUE(fixture.hover->hasSucceeded(worker));
    const auto fresh_ack = state.last_ack;
    state.last_ack -= std::chrono::milliseconds(501);
    EXPECT_FALSE(fixture.hover->hasSucceeded(worker));
    state.last_ack = fresh_ack;
    const auto execution = state.execution_id;
    ++state.execution_id;
    EXPECT_FALSE(fixture.hover->hasSucceeded(worker));
    state.execution_id = execution;
    fixture.hover->AdoptTerminalHold(fixture.hold, kRequestA);
    EXPECT_FALSE(fixture.hover->hasSucceeded(worker));
    fixture.hover->AdoptTerminalHold(fixture.hold, kRequestB);
    ack->consumer_status = Ack::STATUS_PAUSING;
    fixture.scheduler.acknowledgeReferenceStream(ack);
    EXPECT_FALSE(fixture.hover->hasSucceeded(worker));

    // Ordinary Hover has no terminal owner; its original immediate and
    // duration-based completion behavior remains independent of ACK state.
    fixture.hover->ClearTerminalHold();
    EXPECT_TRUE(fixture.hover->hasSucceeded(worker));
    fixture.hover->sustain_action_ = true;
    fixture.hover->hover_duration_s_ = 30.0F;
    fixture.hover->hover_start_time_ = rclcpp::Clock().now();
    EXPECT_FALSE(fixture.hover->hasSucceeded(worker));
    fixture.hover->hover_start_time_ =
        rclcpp::Clock().now() - rclcpp::Duration::from_seconds(31.0);
    EXPECT_TRUE(fixture.hover->hasSucceeded(worker));
    fixture.hover->Stop();
    fixture.scheduler.registered_maneuvers_.erase(
        iii_drone::control::maneuver::MANEUVER_TYPE_HOVER);
}

TEST(ManeuverReferenceClientTransaction, ImmediateAcceptedHoverHasNoFirstAppliedGeneration) {
    RclcppContext context;
    TerminalCompletionFixture fixture(
        "maneuver_controller", 500, "/control/maneuver_controller");
    fixture.finishWithoutSchedulerTick(
        iii_drone::control::maneuver::MANEUVER_TYPE_CABLE_AWARE_FLY_TO_POSITION,
        "cable_aware_fly_to_position", true);
    fixture.scheduler.Start();
    fixture.scheduler.maneuver_execution_timer_->cancel();
    AcceptedHoverGoal goal("immediate_hover_first_ack");
    const auto handle = goal.accept(kRequestB);
    ASSERT_TRUE(handle);
    using Maneuver = iii_drone::control::maneuver::Maneuver;
    Maneuver accepted = Maneuver::FromGoalHandle<AcceptedHoverGoal::Action>(handle);
    Maneuver worker = Maneuver::FromGoalHandle<AcceptedHoverGoal::Action>(handle);
    ASSERT_TRUE(fixture.scheduler.maneuver_queue_->Push(accepted));
    fixture.scheduler.maneuverExecutionTimerCallback();  // pop completed predecessor
    fixture.scheduler.maneuverExecutionTimerCallback();  // start Hover, publish seed
    ASSERT_TRUE(fixture.scheduler.current_maneuver_.Load().started());
    ASSERT_EQ(fixture.scheduler.reference_stream_state_.request_identity, kRequestB);
    ASSERT_TRUE(fixture.scheduler.reference_stream_state_.valid);
    ASSERT_FALSE(fixture.scheduler.reference_stream_state_.ack_seen);

    auto token = std::make_shared<iii_drone::control::maneuver::ReferenceCallbackToken>(
        fixture.scheduler.reference_callback_token_.CreateSlaveHandle(fixture.hover->action_name()));
    std::promise<bool> acquired;
    auto acquired_result = acquired.get_future();
    std::thread worker_thread([&] {
        if (!token->Acquire(2000)) {
            acquired.set_value(false);
            return;
        }
        fixture.hover->startExecution(worker);
        const auto execution = fixture.scheduler.current_reference_execution_id_.Load();
        token->resource().set(
            [&fixture](const iii_drone::control::State & state) {
                return fixture.hover->GetReference(state);
            }, fixture.hover->action_name(), execution, kRequestB);
        acquired.set_value(true);
        // This R12 test deliberately presents a completed-but-unacknowledged
        // generation to QUERY. The real Hover success gate is exercised above.
        worker.Terminate(true);
        fixture.scheduler.onManeuverCompleted(worker);
        token->Release();
    });
    for (int attempt = 0; attempt < 200 &&
         !fixture.scheduler.reference_callback_token_.has_requested_token(
             fixture.hover->action_name()); ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    const bool requested = fixture.scheduler.reference_callback_token_.has_requested_token(
        fixture.hover->action_name());
    if (requested) fixture.scheduler.maneuverExecutionTimerCallback();  // Give token
    const bool token_acquired = acquired_result.get();
    worker_thread.join();
    ASSERT_TRUE(requested);
    ASSERT_TRUE(token_acquired);
    ASSERT_TRUE(fixture.scheduler.current_maneuver_.Load().terminated());
    ASSERT_TRUE(fixture.scheduler.current_maneuver_.Load().success());
    ASSERT_EQ(fixture.hover->terminalHoldBinding().request_identity, kRequestB);
    ASSERT_TRUE(fixture.scheduler.reference_stream_state_.valid);
    EXPECT_FALSE(fixture.scheduler.reference_stream_state_.ack_seen);
    const auto offer = fixture.query();
    EXPECT_FALSE(offer->accepted);
    EXPECT_EQ(offer->reason, "terminal generation awaiting first applied acknowledgement");

    ClientFixture consumer("immediate_hover_first_ack_consumer");
    EXPECT_TRUE(consumer.client.BeginManeuverGoalHandoff(kRequestB));
    EXPECT_TRUE(consumer.client.ConfirmManeuverGoalHandoff(kRequestB));
    consumer.executor.add_node(fixture.node.get_node_base_interface());
    std::thread spinner([&consumer] { consumer.executor.spin(); });
    for (int attempt = 0; attempt < 50 &&
         fixture.scheduler.reference_stream_publisher_->get_subscription_count() == 0; ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    fixture.scheduler.publishReferenceStream();
    const auto stream_id = fixture.scheduler.reference_stream_state_.stream_id;
    auto stale_ack = std::make_shared<Ack>();
    stale_ack->stream_id = "terminal:completed";  // predecessor generation
    stale_ack->last_applied_sequence = 1;
    stale_ack->consumer_status = Ack::STATUS_APPLIED;
    fixture.scheduler.acknowledgeReferenceStream(stale_ack);
    EXPECT_FALSE(fixture.scheduler.reference_stream_state_.ack_seen);
    stale_ack->stream_id = stream_id;
    stale_ack->last_applied_sequence = 0;  // absent from this generation's ring
    fixture.scheduler.acknowledgeReferenceStream(stale_ack);
    EXPECT_FALSE(fixture.scheduler.reference_stream_state_.ack_seen);
    EXPECT_EQ(fixture.query()->reason,
        "terminal generation awaiting first applied acknowledgement");
    std::atomic<int> queries{0};
    fixture.scheduler.terminal_hold_transfer_after_validity_hook_ = [&] { ++queries; };
    EXPECT_EQ(consumer.client.RetainCompletedTerminalHold(kRequestB, 150),
        ManeuverReferenceClient::TerminalHoldRetention::Failed);
    EXPECT_FALSE(fixture.scheduler.reference_stream_state_.ack_seen);
    queries = 0;
    auto retention = std::async(std::launch::async, [&] {
        return consumer.client.RetainCompletedTerminalHold(kRequestB, 150);
    });
    for (int attempt = 0; attempt < 40 && queries.load() == 0; ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    EXPECT_GT(queries.load(), 0);
    EXPECT_EQ(retention.wait_for(std::chrono::milliseconds(0)), std::future_status::timeout);
    for (int attempt = 0; attempt < 40; ++attempt) {
        {
            std::lock_guard<std::mutex> lock(consumer.client.reference_stream_mutex_);
            if (consumer.client.latest_stream_message_) break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    const auto applied = consumer.client.GetReference(0.05, [] {});
    EXPECT_LT((applied.position() - fixture.hold->lastCommand().position()).norm(), 1.0e-3);
    EXPECT_EQ(retention.get(), ManeuverReferenceClient::TerminalHoldRetention::Retained);
    EXPECT_GE(queries.load(), 2);
    EXPECT_EQ(fixture.query()->accepted, true);
    EXPECT_EQ(fixture.scheduler.reference_stream_state_.stream_id,
        consumer.client.active_stream_id_);
    auto negative_ack = std::make_shared<Ack>();
    negative_ack->stream_id = stream_id;
    negative_ack->last_applied_sequence = fixture.scheduler.reference_stream_state_.sequence;
    negative_ack->consumer_status = Ack::STATUS_PAUSING;
    fixture.scheduler.acknowledgeReferenceStream(negative_ack);
    EXPECT_FALSE(fixture.query()->accepted);
    EXPECT_EQ(consumer.client.RetainCompletedTerminalHold(kRequestB, 150),
        ManeuverReferenceClient::TerminalHoldRetention::Failed);
    fixture.scheduler.terminal_hold_transfer_after_validity_hook_ = {};
    consumer.executor.cancel();
    spinner.join();
    consumer.executor.remove_node(fixture.node.get_node_base_interface());
}

TEST(ManeuverReferenceClientTransaction, FiniteSuccessorSeedUsesAdvancingRosClock) {
    RclcppContext context;
    TerminalCompletionFixture fixture("ros_clock_seed_reference");
    const auto clock = fixture.node.get_clock();
    ASSERT_EQ(clock->get_clock_type(), RCL_ROS_TIME);
    ASSERT_EQ(rcl_enable_ros_time_override(clock->get_clock_handle()), RCL_RET_OK);
    constexpr int64_t first_stamp_ns = 503200000000LL;
    constexpr int64_t second_stamp_ns = 503300000000LL;
    ASSERT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), first_stamp_ns), RCL_RET_OK);
    const Reference seed = finiteReference(0.24, 0.0);
    fixture.scheduler.beginReferenceExecution("hover", kRequestB, seed);
    const auto binding = fixture.scheduler.reference_callback_struct_->snapshot();
    ASSERT_TRUE(binding.callback);
    const auto first = binding.callback(iii_drone::control::State());
    ASSERT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), second_stamp_ns), RCL_RET_OK);
    const auto second = binding.callback(iii_drone::control::State());
    EXPECT_EQ(first.stamp().get_clock_type(), RCL_ROS_TIME);
    EXPECT_EQ(first.stamp().nanoseconds(), first_stamp_ns);
    EXPECT_EQ(second.stamp().nanoseconds(), second_stamp_ns);
    EXPECT_LT((first.position() - seed.position()).norm(), 1.0e-7);
    EXPECT_LT((second.velocity() - seed.velocity()).norm(), 1.0e-7);
    EXPECT_LT((second.acceleration() - seed.acceleration()).norm(), 1.0e-7);
}

TEST(ManeuverReferenceClientTransaction, SchedulerStartSurvivesPreStartWorkerCompletionCopy) {
    RclcppContext context;
    TerminalCompletionFixture fixture("scheduler_worker_copy_completion");
    fixture.hover->Update(Reference(point_t::Zero(), 0.0));
    fixture.scheduler.Start();
    fixture.source_server = std::make_shared<
        iii_drone::control::maneuver::CableAwareFlyToPositionManeuverServer>(
            &fixture.node, fixture.awareness, "cable_aware_fly_to_position", 1, 1,
            fixture.config, nullptr);
    fixture.scheduler.registered_maneuvers_[
        iii_drone::control::maneuver::MANEUVER_TYPE_CABLE_AWARE_FLY_TO_POSITION] =
        fixture.source_server;
    auto worker_token = std::make_shared<iii_drone::control::maneuver::ReferenceCallbackToken>(
        fixture.scheduler.reference_callback_token_.CreateSlaveHandle(
            fixture.source_server->action_name()));

    AcceptedCableGoal goal("scheduler_worker_copy_completion");
    const auto handle = goal.accept(kRequestA);
    ASSERT_TRUE(handle);
    using Maneuver = iii_drone::control::maneuver::Maneuver;
    Maneuver accepted = Maneuver::FromGoalHandle<AcceptedCableGoal::Action>(handle);
    Maneuver worker = Maneuver::FromGoalHandle<AcceptedCableGoal::Action>(handle);
    ASSERT_FALSE(accepted.started());
    ASSERT_FALSE(worker.started());
    ASSERT_TRUE(fixture.scheduler.maneuver_queue_->Push(accepted));
    fixture.scheduler.maneuverExecutionTimerCallback();  // queue pop
    fixture.scheduler.maneuverExecutionTimerCallback();  // production Start
    const Maneuver started = fixture.scheduler.current_maneuver_.Load();
    ASSERT_TRUE(started.started());
    ASSERT_FALSE(started.terminated());
    EXPECT_GT(started.start_time().nanoseconds(), 0);
    ASSERT_FALSE(worker.started());

    fixture.hover->AdoptTerminalHold(fixture.hold, kRequestA);
    const auto execution = fixture.scheduler.current_reference_execution_id_.Load();
    std::promise<bool> token_acquired;
    auto token_acquired_result = token_acquired.get_future();
    std::promise<void> release_token;
    auto release_token_result = release_token.get_future();
    std::thread worker_thread([&] {
        if (!worker_token->Acquire(2000)) {
            token_acquired.set_value(false);
            return;
        }
        worker_token->resource().set(
            [&fixture](const iii_drone::control::State &) {
                return fixture.hold->lastCommand();
            }, fixture.source_server->action_name(), execution, kRequestA);
        token_acquired.set_value(true);
        release_token_result.wait();
        worker_token->Release();
    });
    for (int attempt = 0; attempt < 200 &&
         !fixture.scheduler.reference_callback_token_.has_requested_token(
             fixture.source_server->action_name()); ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (!fixture.scheduler.reference_callback_token_.has_requested_token(
            fixture.source_server->action_name())) {
        worker_thread.join();
        FAIL() << "worker never requested callback token";
    }
    fixture.scheduler.maneuverExecutionTimerCallback();  // production Give
    if (!token_acquired_result.get()) {
        worker_thread.join();
        FAIL() << "scheduler did not give worker the callback token";
    }
    fixture.scheduler.publishReferenceStream();
    EXPECT_TRUE(fixture.scheduler.reference_stream_state_.valid);
    auto ack = std::make_shared<Ack>();
    ack->stream_id = fixture.scheduler.reference_stream_state_.stream_id;
    ack->last_applied_sequence = fixture.scheduler.reference_stream_state_.sequence;
    ack->consumer_status = Ack::STATUS_APPLIED;
    fixture.scheduler.acknowledgeReferenceStream(ack);
    EXPECT_TRUE(fixture.scheduler.reference_stream_state_.last_ack_reference_valid);

    worker.Terminate(true);
    EXPECT_FALSE(worker.started());
    fixture.scheduler.onManeuverCompleted(worker);
    const Maneuver completed = fixture.scheduler.current_maneuver_.Load();
    EXPECT_TRUE(completed.started());
    EXPECT_TRUE(completed.terminated());
    EXPECT_TRUE(completed.success());
    EXPECT_EQ(completed.start_time().nanoseconds(), started.start_time().nanoseconds());
    EXPECT_EQ(completed.creation_time().nanoseconds(), started.creation_time().nanoseconds());
    EXPECT_EQ(completed.requestIdentity(), kRequestA);
    EXPECT_EQ(completed.maneuver_params(), started.maneuver_params());
    EXPECT_EQ(fixture.query()->reason, "terminal callback finalizing");
    fixture.scheduler.maneuverExecutionTimerCallback();
    EXPECT_TRUE(fixture.scheduler.reference_stream_state_.valid);
    release_token.set_value();
    worker_thread.join();  // Token::Release invokes the scheduler callback.
    EXPECT_TRUE(fixture.query()->accepted);
}

TEST(ManeuverReferenceClientTransaction, ManeuverCopiesPreserveLifecycleTimestamps) {
    RclcppContext context;
    using Maneuver = iii_drone::control::maneuver::Maneuver;
    Maneuver source(iii_drone::control::maneuver::MANEUVER_TYPE_HOVER,
        rclcpp_action::GoalUUID{});
    source.request_identity_ = kRequestA;
    source.creation_time_ = rclcpp::Time(int64_t{1000}, RCL_SYSTEM_TIME);
    source.start_time_ = rclcpp::Time(int64_t{2000}, RCL_SYSTEM_TIME);
    source.termination_time_ = rclcpp::Time(int64_t{3000}, RCL_SYSTEM_TIME);
    source.started_ = true;
    source.terminated_ = true;
    source.success_ = false;
    const Maneuver copied(source);
    Maneuver assigned;
    assigned = source;
    for (const auto & value : {copied, assigned}) {
        EXPECT_EQ(value.creation_time().nanoseconds(), 1000);
        EXPECT_EQ(value.start_time().nanoseconds(), 2000);
        EXPECT_EQ(value.termination_time().nanoseconds(), 3000);
        EXPECT_TRUE(value.started());
        EXPECT_TRUE(value.terminated());
        EXPECT_FALSE(value.success());
        EXPECT_EQ(value.requestIdentity(), kRequestA);
    }
}

void startAcceptedCableManeuver(TerminalCompletionFixture & fixture,
    const iii_drone::control::maneuver::Maneuver & accepted) {
    fixture.hover->Update(Reference(point_t::Zero(), 0.0));
    fixture.scheduler.Start();
    fixture.source_server = std::make_shared<
        iii_drone::control::maneuver::CableAwareFlyToPositionManeuverServer>(
            &fixture.node, fixture.awareness, "cable_aware_fly_to_position", 1, 1,
            fixture.config, nullptr);
    fixture.scheduler.registered_maneuvers_[
        iii_drone::control::maneuver::MANEUVER_TYPE_CABLE_AWARE_FLY_TO_POSITION] =
        fixture.source_server;
    ASSERT_TRUE(fixture.scheduler.maneuver_queue_->Push(accepted));
    fixture.scheduler.maneuverExecutionTimerCallback();  // queue pop
    fixture.scheduler.maneuverExecutionTimerCallback();  // scheduler Start
    ASSERT_TRUE(fixture.scheduler.current_maneuver_.Load().started());
}

TEST(ManeuverReferenceClientTransaction, ActiveFailureAndCancelKeepSchedulerStart) {
    RclcppContext context;
    using Maneuver = iii_drone::control::maneuver::Maneuver;
    for (const bool via_cancel : {false, true}) {
        const std::string name = via_cancel ? "worker_cancel_lifecycle" : "worker_failure_lifecycle";
        TerminalCompletionFixture fixture(name);
        AcceptedCableGoal goal(name);
        const auto handle = goal.accept(kRequestA);
        ASSERT_TRUE(handle);
        const Maneuver accepted = Maneuver::FromGoalHandle<AcceptedCableGoal::Action>(handle);
        Maneuver worker = Maneuver::FromGoalHandle<AcceptedCableGoal::Action>(handle);
        startAcceptedCableManeuver(fixture, accepted);
        const Maneuver started = fixture.scheduler.current_maneuver_.Load();
        ASSERT_TRUE(started.started());
        worker.Terminate(false);
        if (via_cancel) {
            ASSERT_TRUE(fixture.scheduler.CancelManeuver(worker));
        } else {
            fixture.scheduler.onManeuverCompleted(worker);
        }
        const Maneuver completed = fixture.scheduler.current_maneuver_.Load();
        EXPECT_TRUE(completed.started());
        EXPECT_TRUE(completed.terminated());
        EXPECT_FALSE(completed.success());
        EXPECT_EQ(completed.creation_time().nanoseconds(), started.creation_time().nanoseconds());
        EXPECT_EQ(completed.start_time().nanoseconds(), started.start_time().nanoseconds());
        EXPECT_GT(completed.termination_time().nanoseconds(), 0);
        EXPECT_EQ(completed.maneuver_params(), started.maneuver_params());
    }
}

TEST(ManeuverReferenceClientTransaction, QueuedAndPreStartCancelNeverFabricateStart) {
    RclcppContext context;
    using Maneuver = iii_drone::control::maneuver::Maneuver;
    for (const bool popped : {false, true}) {
        const std::string name = popped ? "prestart_current_cancel" : "prestart_queue_cancel";
        TerminalCompletionFixture fixture(name);
        AcceptedCableGoal goal(name);
        const auto handle = goal.accept(kRequestA);
        ASSERT_TRUE(handle);
        const Maneuver accepted = Maneuver::FromGoalHandle<AcceptedCableGoal::Action>(handle);
        Maneuver worker = Maneuver::FromGoalHandle<AcceptedCableGoal::Action>(handle);
        fixture.hover->Update(Reference(point_t::Zero(), 0.0));
        fixture.scheduler.Start();
        ASSERT_TRUE(fixture.scheduler.maneuver_queue_->Push(accepted));
        if (popped) fixture.scheduler.maneuverExecutionTimerCallback();
        worker.Terminate(false);
        worker.started_ = true;  // a stale report cannot invent scheduler Start
        if (popped) {
            fixture.scheduler.onManeuverCompleted(worker);
            EXPECT_FALSE(fixture.scheduler.current_maneuver_.Load().started());
            EXPECT_FALSE(fixture.scheduler.current_maneuver_.Load().terminated());
        }
        ASSERT_TRUE(fixture.scheduler.CancelManeuver(worker));
        const Maneuver canceled = popped
            ? fixture.scheduler.current_maneuver_.Load()
            : fixture.scheduler.maneuver_queue_->Find(accepted.uuid());
        EXPECT_FALSE(canceled.started());
        EXPECT_TRUE(canceled.terminated());
        EXPECT_FALSE(canceled.success());
        EXPECT_EQ(canceled.creation_time().nanoseconds(), accepted.creation_time().nanoseconds());
        EXPECT_EQ(canceled.maneuver_params(), accepted.maneuver_params());
        if (!popped) {
            Maneuver wrong_request = worker;
            wrong_request.request_identity_ = kRequestB;
            EXPECT_FALSE(fixture.scheduler.CancelManeuver(wrong_request));
            EXPECT_EQ(fixture.scheduler.maneuver_queue_->Find(accepted.uuid()).requestIdentity(), kRequestA);
        }
    }
}

TEST(ManeuverReferenceClientTransaction, CompletionRejectsWrongRequestAndUnstartedReport) {
    RclcppContext context;
    using Maneuver = iii_drone::control::maneuver::Maneuver;
    TerminalCompletionFixture fixture("wrong_worker_identity");
    AcceptedCableGoal goal("wrong_worker_identity");
    const auto handle = goal.accept(kRequestA);
    ASSERT_TRUE(handle);
    const Maneuver accepted = Maneuver::FromGoalHandle<AcceptedCableGoal::Action>(handle);
    Maneuver worker = Maneuver::FromGoalHandle<AcceptedCableGoal::Action>(handle);
    startAcceptedCableManeuver(fixture, accepted);
    const Maneuver started = fixture.scheduler.current_maneuver_.Load();
    Maneuver unterminated = worker;
    fixture.scheduler.onManeuverCompleted(unterminated);
    EXPECT_FALSE(fixture.scheduler.current_maneuver_.Load().terminated());
    worker.Terminate(true);
    worker.request_identity_ = kRequestB;
    EXPECT_THROW(fixture.scheduler.onManeuverCompleted(worker), std::runtime_error);
    EXPECT_FALSE(fixture.scheduler.CancelManeuver(worker));
    const Maneuver current = fixture.scheduler.current_maneuver_.Load();
    EXPECT_TRUE(current.started());
    EXPECT_FALSE(current.terminated());
    EXPECT_EQ(current.requestIdentity(), kRequestA);
    EXPECT_EQ(current.start_time().nanoseconds(), started.start_time().nanoseconds());
}
