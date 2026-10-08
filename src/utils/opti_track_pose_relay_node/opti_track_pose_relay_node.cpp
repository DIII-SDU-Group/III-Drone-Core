/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "iii_drone_core/utils/opti_track_pose_relay_node/opti_track_pose_relay_node.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

#include <diagnostic_msgs/msg/key_value.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

using namespace iii_drone::utils::opti_track_pose_relay_node;
using namespace iii_drone::utils::opti_track_pose_relay;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

namespace {

/** Shortest interval between two origin commands. */
constexpr int64_t kOriginResendIntervalNs = 5'000'000'000;

/** Largest age of a PX4 sample the origin decision uses (vehicle_status is 2 Hz, estimator_status_flags 1 Hz). */
constexpr int64_t kOriginInputTimeoutNs = 3'000'000'000;

constexpr std::chrono::milliseconds kHealthPeriod{500};

/** Health reports without any pose before the relay warns once. */
constexpr int kNoPoseWarningReports = 10;

int64_t steadyNowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()
    ).count();
}

int64_t systemNowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();
}

template <typename T>
const char * typeDescription();

template <>
const char * typeDescription<int64_t>() { return "an integer"; }

template <>
const char * typeDescription<double>() { return "a double (write 50.0, not 50)"; }

template <>
const char * typeDescription<bool>() { return "a boolean"; }

template <typename T>
T declareReadOnly(
    rclcpp::Node & node,
    const std::string & field,
    const T & default_value,
    const std::string & description,
    std::vector<std::string> & errors
) {
    const std::string name = std::string(kParameterPrefix) + field;
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.read_only = true;
    try {
        return node.declare_parameter<T>(name, default_value, descriptor);
    } catch (const rclcpp::exceptions::InvalidParameterTypeException & error) {
        errors.push_back(name + " must be " + typeDescription<T>() + ": " + error.what());
    } catch (const rclcpp::ParameterTypeException & error) {
        errors.push_back(name + " must be " + typeDescription<T>() + ": " + error.what());
    }
    return default_value;
}

std::string joined(const std::vector<std::string> & errors) {
    std::string message;
    for (const auto & error : errors) {
        message += (message.empty() ? "" : "; ") + error;
    }
    return message;
}

std::string formatNumber(double value) {
    if (std::isnan(value)) {
        return "nan";
    }
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(1) << value;
    return stream.str();
}

diagnostic_msgs::msg::KeyValue keyValue(const std::string & key, const std::string & value) {
    diagnostic_msgs::msg::KeyValue entry;
    entry.key = key;
    entry.value = value;
    return entry;
}

}  // namespace

PoseRelayParameters iii_drone::utils::opti_track_pose_relay_node::DeclarePoseRelayParameters(
    rclcpp::Node & node,
    std::vector<std::string> & errors
) {
    const PoseRelayParameters defaults;
    PoseRelayParameters parameters;
    parameters.rigid_body_id = declareReadOnly<int64_t>(node, "rigid_body_id", defaults.rigid_body_id,
        "Motive rigid-body ID; the lab gateway publishes /body_splitter/body_<id>/pose. -1 is unset.", errors);
    parameters.lab_ros_domain_id = declareReadOnly<int64_t>(node, "lab_ros_domain_id", defaults.lab_ros_domain_id,
        "ROS domain of the lab gateway, [0, 232].", errors);
    parameters.output_rate_hz = declareReadOnly<double>(node, "output_rate_hz", defaults.output_rate_hz,
        "Upper bound of the rate poses are forwarded to PX4 at [Hz], [1, 200].", errors);
    parameters.stale_timeout_s = declareReadOnly<double>(node, "stale_timeout_s", defaults.stale_timeout_s,
        "A pose that arrived longer ago is never forwarded [s], (0, 1].", errors);
    parameters.position_variance_m2 = declareReadOnly<double>(node, "position_variance_m2",
        defaults.position_variance_m2, "Variance reported for each position axis [m^2], (0, 1].", errors);
    parameters.orientation_variance_rad2 = declareReadOnly<double>(node, "orientation_variance_rad2",
        defaults.orientation_variance_rad2, "Variance reported for each orientation axis [rad^2], (0, 1].", errors);
    parameters.send_origin = declareReadOnly<bool>(node, "send_origin", defaults.send_origin,
        "Send PX4 the EKF global origin while it has none (disarmed, vision position fusion intended).", errors);
    parameters.origin_latitude_deg = declareReadOnly<double>(node, "origin_latitude_deg",
        defaults.origin_latitude_deg, "EKF global origin latitude [deg].", errors);
    parameters.origin_longitude_deg = declareReadOnly<double>(node, "origin_longitude_deg",
        defaults.origin_longitude_deg, "EKF global origin longitude [deg].", errors);
    parameters.origin_altitude_m = declareReadOnly<double>(node, "origin_altitude_m",
        defaults.origin_altitude_m, "EKF global origin altitude [m AMSL].", errors);
    return parameters;
}

px4_msgs::msg::VehicleOdometry iii_drone::utils::opti_track_pose_relay_node::MakeVisualOdometry(
    const NedPose & pose,
    uint64_t timestamp_us,
    double position_variance_m2,
    double orientation_variance_rad2
) {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    px4_msgs::msg::VehicleOdometry message;
    message.timestamp = timestamp_us;
    message.timestamp_sample = timestamp_us;
    message.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
    for (std::size_t i = 0; i < 3; ++i) {
        message.position[i] = static_cast<float>(pose.position[i]);
    }
    for (std::size_t i = 0; i < 4; ++i) {
        message.q[i] = static_cast<float>(pose.orientation[i]);
    }
    message.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;
    message.velocity = {nan, nan, nan};
    message.angular_velocity = {nan, nan, nan};
    message.position_variance.fill(static_cast<float>(position_variance_m2));
    message.orientation_variance.fill(static_cast<float>(orientation_variance_rad2));
    message.velocity_variance = {nan, nan, nan};
    message.reset_counter = 0;
    message.quality = 0;
    return message;
}

px4_msgs::msg::VehicleCommand iii_drone::utils::opti_track_pose_relay_node::MakeSetGpsGlobalOriginCommand(
    double latitude_deg,
    double longitude_deg,
    double altitude_m,
    uint64_t timestamp_us,
    uint8_t target_system,
    uint16_t target_component
) {
    px4_msgs::msg::VehicleCommand command;
    command.timestamp = timestamp_us;
    command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN;
    command.param5 = latitude_deg;
    command.param6 = longitude_deg;
    command.param7 = static_cast<float>(altitude_m);
    command.target_system = target_system;
    command.target_component = target_component;
    command.source_system = 1;
    command.source_component = 1;
    command.confirmation = 0;
    command.from_external = true;
    return command;
}

diagnostic_msgs::msg::DiagnosticStatus iii_drone::utils::opti_track_pose_relay_node::MakeHealthStatus(
    const HealthReport & report,
    bool origin_sent,
    int64_t rigid_body_id
) {
    diagnostic_msgs::msg::DiagnosticStatus status;
    switch (report.level) {
        case HealthLevel::OK:
            status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            break;
        case HealthLevel::WARN:
            status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            break;
        case HealthLevel::ERROR:
            status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            break;
    }
    status.name = kHealthName;
    status.message = report.message;
    status.hardware_id = "optitrack_rigid_body_" + std::to_string(rigid_body_id);
    status.values = {
        keyValue("input_rate_hz", formatNumber(report.input_rate_hz)),
        keyValue("output_rate_hz", formatNumber(report.output_rate_hz)),
        keyValue("last_input_age_ms", formatNumber(report.last_input_age_ms)),
        keyValue("max_input_gap_ms", formatNumber(report.max_input_gap_ms)),
        keyValue("lab_stamp_age_ms", formatNumber(report.lab_stamp_age_ms)),
        keyValue("stale", report.stale ? "true" : "false"),
        keyValue("origin_sent", origin_sent ? "true" : "false"),
        keyValue("rigid_body_id", std::to_string(rigid_body_id)),
        keyValue("rejected_samples", std::to_string(report.rejected_samples)),
    };
    return status;
}

OptiTrackPoseRelayNode::OptiTrackPoseRelayNode(
    const std::string & node_name,
    const std::string & node_namespace,
    const rclcpp::NodeOptions & options
) : rclcpp::Node(
        node_name,
        node_namespace,
        options
    ),
    parameters_(DeclarePoseRelayParameters(*this, configuration_errors_)),
    lab_topic_(LabPoseTopic(parameters_.rigid_body_id)) {

    for (const auto & error : ValidatePoseRelayParameters(parameters_)) {
        configuration_errors_.push_back(error);
    }
    configuration_error_ = joined(configuration_errors_);

    health_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
        kHealthTopic,
        rclcpp::QoS(rclcpp::KeepLast(10))
    );

    health_timer_ = this->create_wall_timer(
        kHealthPeriod,
        [this]() { onHealthTimer(); }
    );

    // Exiting would only repeat the error on every supervised restart.
    if (!configuration_error_.empty()) {
        RCLCPP_ERROR(
            get_logger(),
            "OptiTrackPoseRelayNode: not relaying, invalid configuration: %s",
            configuration_error_.c_str()
        );
        return;
    }

    startRelaying();

}

OptiTrackPoseRelayNode::~OptiTrackPoseRelayNode() {

    stopLabSide();

}

const PoseRelayParameters & OptiTrackPoseRelayNode::parameters() const {
    return parameters_;
}

const std::string & OptiTrackPoseRelayNode::configuration_error() const {
    return configuration_error_;
}

void OptiTrackPoseRelayNode::startRelaying() {

    gate_.emplace(parameters_.output_rate_hz, parameters_.stale_timeout_s);
    health_.emplace(parameters_.output_rate_hz, parameters_.stale_timeout_s, steadyNowNs());
    origin_sender_.emplace(parameters_.send_origin, kOriginResendIntervalNs, kOriginInputTimeoutNs);

    // PX4's uXRCE-DDS readers are best effort and volatile; same QoS as the
    // other III /fmu/in publishers.
    const rclcpp::QoS px4_in_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    visual_odometry_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
        kVisualOdometryTopic,
        px4_in_qos
    );

    // Supervision's readiness: a 2 Hz heartbeat instead of the odometry stream.
    fresh_pub_ = this->create_publisher<std_msgs::msg::Header>(
        kFreshTopic,
        rclcpp::QoS(rclcpp::KeepLast(1))
    );

    if (parameters_.send_origin) {

        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command",
            px4_in_qos
        );

        // As Core's other PX4 subscriptions (PX4 writers are transient local).
        // PX4 may come up after the relay: the decision waits for its topics.
        rclcpp::QoS px4_out_qos(rclcpp::KeepLast(1));
        px4_out_qos.best_effort();
        px4_out_qos.transient_local();

        vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status_v1",
            px4_out_qos,
            [this](const px4_msgs::msg::VehicleStatus::ConstSharedPtr message) {
                if (message->system_id != 0) {
                    vehicle_system_id_ = message->system_id;
                }
                if (message->component_id != 0) {
                    vehicle_component_id_ = message->component_id;
                }
                if (message->timestamp != 0) {
                    vehicle_timestamp_us_ = message->timestamp;
                }
                origin_sender_->UpdateDisarmed(
                    message->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED,
                    steadyNowNs()
                );
            }
        );

        vehicle_local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position",
            px4_out_qos,
            [this](const px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr message) {
                origin_sender_->UpdateGlobalOrigin(message->xy_global, steadyNowNs());
            }
        );

        estimator_status_flags_sub_ = this->create_subscription<px4_msgs::msg::EstimatorStatusFlags>(
            "/fmu/out/estimator_status_flags",
            px4_out_qos,
            [this](const px4_msgs::msg::EstimatorStatusFlags::ConstSharedPtr message) {
                origin_sender_->UpdateVisionPositionFusion(message->cs_ev_pos, steadyNowNs());
            }
        );

    }

    startLabSide();

    RCLCPP_INFO(
        get_logger(),
        "OptiTrackPoseRelayNode: relaying rigid body %lld from %s (lab ROS domain %lld) to %s at up to %.1f Hz, stale after %.0f ms; EKF global origin %s",
        static_cast<long long>(parameters_.rigid_body_id),
        lab_topic_.c_str(),
        static_cast<long long>(parameters_.lab_ros_domain_id),
        kVisualOdometryTopic,
        parameters_.output_rate_hz,
        parameters_.stale_timeout_s * 1000.0,
        parameters_.send_origin ? "sent while PX4 has none" : "not sent"
    );

}

void OptiTrackPoseRelayNode::startLabSide() {

    // The lab side joins the lab's domain in its own context; logging is
    // already set up by the process context.
    lab_context_ = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions init_options;
    init_options.auto_initialize_logging(false);
    init_options.set_domain_id(static_cast<size_t>(parameters_.lab_ros_domain_id));
    lab_context_->init(0, nullptr, init_options);

    // Only a subscriber on the lab network: no rosout or parameter services there.
    rclcpp::NodeOptions lab_options;
    lab_options.context(lab_context_);
    lab_options.use_global_arguments(false);
    lab_options.enable_rosout(false);
    lab_options.start_parameter_services(false);
    lab_options.start_parameter_event_publisher(false);
    lab_node_ = std::make_shared<rclcpp::Node>(
        "iii_drone_pose_relay_body_" + std::to_string(parameters_.rigid_body_id),
        lab_options
    );

    lab_pose_sub_ = lab_node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        lab_topic_,
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
        [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
            onLabPose(*message);
        }
    );

    rclcpp::ExecutorOptions executor_options;
    executor_options.context = lab_context_;
    lab_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(executor_options);
    lab_executor_->add_node(lab_node_);
    lab_thread_ = std::thread([this, executor = lab_executor_]() {
        try {
            executor->spin();
        } catch (const std::exception & error) {
            // Nothing is relayed without the lab side: end the process.
            lab_side_failed_ = true;
            RCLCPP_FATAL(
                get_logger(),
                "OptiTrackPoseRelayNode: lab side stopped: %s",
                error.what()
            );
            rclcpp::shutdown(
                get_node_base_interface()->get_context(),
                "opti_track_pose_relay lab side failed"
            );
        }
    });

}

bool OptiTrackPoseRelayNode::lab_side_failed() const {
    return lab_side_failed_;
}

void OptiTrackPoseRelayNode::stopLabSide() {

    // Shutting the context down ends spin() even if it has not started yet.
    if (lab_context_ && lab_context_->is_valid()) {
        lab_context_->shutdown("opti_track_pose_relay stopped");
    }
    if (lab_executor_) {
        lab_executor_->cancel();
    }
    if (lab_thread_.joinable()) {
        lab_thread_.join();
    }
    lab_executor_.reset();
    lab_pose_sub_.reset();
    lab_node_.reset();
    lab_context_.reset();

}

void OptiTrackPoseRelayNode::onLabPose(const geometry_msgs::msg::PoseStamped & message) {

    const int64_t arrival_ns = steadyNowNs();
    const int64_t arrival_system_ns = systemNowNs();

    LabPose pose;
    pose.position = {message.pose.position.x, message.pose.position.y, message.pose.position.z};
    pose.orientation = {
        message.pose.orientation.w,
        message.pose.orientation.x,
        message.pose.orientation.y,
        message.pose.orientation.z
    };
    std::string rejection_reason;
    const auto ned = LabPoseToNed(pose, &rejection_reason);

    const int64_t stamp_ns =
        static_cast<int64_t>(message.header.stamp.sec) * 1'000'000'000 + message.header.stamp.nanosec;
    const double lab_stamp_age_ms = stamp_ns > 0
        ? static_cast<double>(arrival_system_ns - stamp_ns) * 1.0e-6
        : std::numeric_limits<double>::quiet_NaN();

    std::lock_guard<std::mutex> lock(relay_mutex_);

    if (!ned) {
        health_->RecordRejected();
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            steady_clock_,
            5000,
            "OptiTrackPoseRelayNode::onLabPose(): rejected a pose of rigid body %lld: %s",
            static_cast<long long>(parameters_.rigid_body_id),
            rejection_reason.c_str()
        );
        return;
    }

    health_->RecordInput(arrival_ns, lab_stamp_age_ms);
    if (!received_any_.exchange(true)) {
        RCLCPP_INFO(
            get_logger(),
            "OptiTrackPoseRelayNode::onLabPose(): receiving rigid body %lld",
            static_cast<long long>(parameters_.rigid_body_id)
        );
    }

    const int64_t now_ns = steadyNowNs();
    if (!gate_->Offer(arrival_ns, now_ns)) {
        return;
    }

    visual_odometry_pub_->publish(MakeVisualOdometry(
        *ned,
        static_cast<uint64_t>(arrival_system_ns / 1000),
        parameters_.position_variance_m2,
        parameters_.orientation_variance_rad2
    ));
    health_->RecordOutput(now_ns);

}

void OptiTrackPoseRelayNode::onHealthTimer() {

    if (!configuration_error_.empty()) {
        HealthReport idle;
        idle.level = HealthLevel::ERROR;
        idle.message = "not relaying, invalid configuration: " + configuration_error_;
        health_pub_->publish(MakeHealthStatus(idle, false, parameters_.rigid_body_id));
        return;
    }

    const int64_t now_ns = steadyNowNs();

    if (vehicle_command_pub_ && origin_sender_->Due(now_ns)) {
        // PX4 timestamps commands in its boot-time domain; use its latest one.
        const uint64_t timestamp_us = vehicle_timestamp_us_ != 0
            ? vehicle_timestamp_us_
            : static_cast<uint64_t>(systemNowNs() / 1000);
        vehicle_command_pub_->publish(MakeSetGpsGlobalOriginCommand(
            parameters_.origin_latitude_deg,
            parameters_.origin_longitude_deg,
            parameters_.origin_altitude_m,
            timestamp_us,
            vehicle_system_id_,
            vehicle_component_id_
        ));
        origin_sender_->MarkSent(now_ns);
        RCLCPP_INFO(
            get_logger(),
            "OptiTrackPoseRelayNode::onHealthTimer(): PX4 has no EKF global origin: sent %.7f deg, %.7f deg, %.1f m",
            parameters_.origin_latitude_deg,
            parameters_.origin_longitude_deg,
            parameters_.origin_altitude_m
        );
    }

    HealthReport report;
    {
        std::lock_guard<std::mutex> lock(relay_mutex_);
        report = health_->Report(now_ns);
    }

    if (std::isnan(report.last_input_age_ms)) {
        report.message += " on " + lab_topic_ + " in lab ROS domain " +
            std::to_string(parameters_.lab_ros_domain_id);
        if (++reports_without_pose_ == kNoPoseWarningReports) {
            RCLCPP_WARN(
                get_logger(),
                "OptiTrackPoseRelayNode::onHealthTimer(): %s (check the lab gateway, the Wi-Fi and the rigid-body ID)",
                report.message.c_str()
            );
        }
    } else if (report.stale != reported_stale_) {
        if (report.stale) {
            RCLCPP_WARN(get_logger(), "OptiTrackPoseRelayNode::onHealthTimer(): pose %s", report.message.c_str());
        } else {
            RCLCPP_INFO(get_logger(), "OptiTrackPoseRelayNode::onHealthTimer(): pose stream fresh");
        }
        reported_stale_ = report.stale;
    }

    if (report.forwarding) {
        std_msgs::msg::Header heartbeat;
        heartbeat.stamp = this->now();
        heartbeat.frame_id = kHealthName;
        fresh_pub_->publish(heartbeat);
    }

    health_pub_->publish(MakeHealthStatus(report, origin_sender_->sent(), parameters_.rigid_body_id));

}
