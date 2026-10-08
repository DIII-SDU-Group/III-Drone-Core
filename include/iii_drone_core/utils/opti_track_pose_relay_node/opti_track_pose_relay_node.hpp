#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/header.hpp>

/*****************************************************************************/
// PX4:

#include <px4_msgs/msg/estimator_status_flags.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/opti_track_pose_relay.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace utils {
namespace opti_track_pose_relay_node {

    /** Topic PX4's EKF2 takes external vision from. */
    constexpr const char * kVisualOdometryTopic = "/fmu/in/vehicle_visual_odometry";

    /** Relay health topic (diagnostic_msgs/DiagnosticStatus, always 2 Hz). */
    constexpr const char * kHealthTopic = "/opti_track/pose_relay/health";

    /** Readiness heartbeat (std_msgs/Header, 2 Hz only while fresh poses are forwarded). */
    constexpr const char * kFreshTopic = "/opti_track/pose_relay/fresh";

    /** DiagnosticStatus name of the relay, also the heartbeat's frame_id. */
    constexpr const char * kHealthName = "opti_track_pose_relay";

    /**
     * @brief Relays the OptiTrack pose of one rigid body from the SDU lab
     * gateway to PX4's EKF2 as external vision, and sets PX4's EKF global
     * origin while it has none.
     *
     * The lab side is a second rclcpp context in the lab's ROS domain
     * (lab_ros_domain_id) with its own node, executor and thread; it
     * subscribes /body_splitter/body_<rigid_body_id>/pose (best effort,
     * volatile, keep last 1). Every pose is converted to NED/FRD and
     * forwarded on arrival on /fmu/in/vehicle_visual_odometry, subject to the
     * OutputGate. Everything else runs in the process's own ROS domain:
     * health on /opti_track/pose_relay/health (always, 2 Hz), the readiness
     * heartbeat on /opti_track/pose_relay/fresh (2 Hz while a pose was
     * forwarded within stale_timeout_s) and the origin sender
     * (VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN on /fmu/in/vehicle_command, retried
     * for as long as the relay runs, so PX4 may come up later).
     *
     * Parameters are read once at construction (read-only). With an unset
     * rigid-body ID or any invalid parameter the relay stays idle instead of
     * exiting (a supervised restart would only repeat the error): it logs the
     * error once and publishes only ERROR health naming it; it has no lab
     * side and publishes no odometry, heartbeat or origin.
     */
    class OptiTrackPoseRelayNode : public rclcpp::Node {
    public:
        /**
         * @brief Constructor. Reads and validates the parameters, then starts
         * the lab side if they are valid.
         *
         * @param node_name The name of the node.
         * @param node_namespace The namespace of the node.
         * @param options The node options.
         */
        OptiTrackPoseRelayNode(
            const std::string & node_name = "pose_relay",
            const std::string & node_namespace = "/opti_track",
            const rclcpp::NodeOptions & options = rclcpp::NodeOptions()
        );

        /**
         * @brief Destructor. Stops the lab side and shuts its context down.
         */
        ~OptiTrackPoseRelayNode() override;

        /**
         * @brief The parameters as read (defaults where an override had the wrong type).
         */
        const opti_track_pose_relay::PoseRelayParameters & parameters() const;

        /**
         * @brief Why the relay is idle: every invalid parameter, empty when it relays.
         */
        const std::string & configuration_error() const;

        /**
         * @brief True once the lab side stopped on an error. The node's context
         * is then shut down, so the process ends and can be restarted.
         */
        bool lab_side_failed() const;

    private:
        /** Filled while the parameters are declared, so declared before them. */
        std::vector<std::string> configuration_errors_;

        opti_track_pose_relay::PoseRelayParameters parameters_;

        std::string configuration_error_;

        std::string lab_topic_;

        /** Steady clock for throttled logs from either thread. */
        rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

        // Process ROS domain:
        rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr health_pub_;
        rclcpp::TimerBase::SharedPtr health_timer_;
        rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr visual_odometry_pub_;
        rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr fresh_pub_;
        rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
        rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
        rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
        rclcpp::Subscription<px4_msgs::msg::EstimatorStatusFlags>::SharedPtr estimator_status_flags_sub_;

        // Lab ROS domain:
        rclcpp::Context::SharedPtr lab_context_;
        rclcpp::Node::SharedPtr lab_node_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr lab_pose_sub_;
        std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> lab_executor_;
        std::thread lab_thread_;
        std::atomic<bool> lab_side_failed_{false};

        // Set only when relaying.
        /** Guards gate_ and health_, shared by the lab thread and the health timer. */
        std::mutex relay_mutex_;
        std::optional<opti_track_pose_relay::OutputGate> gate_;
        std::optional<opti_track_pose_relay::RelayHealthMonitor> health_;
        std::atomic<bool> received_any_{false};

        // Only used from the process executor (subscriptions and the health timer).
        std::optional<opti_track_pose_relay::OriginSender> origin_sender_;
        uint8_t vehicle_system_id_ = 0;
        uint8_t vehicle_component_id_ = 0;
        uint64_t vehicle_timestamp_us_ = 0;
        bool reported_stale_ = true;
        int reports_without_pose_ = 0;

        /** Creates the publishers, subscriptions and lab side of a valid configuration. */
        void startRelaying();

        /** Converts and forwards one lab pose (lab thread). */
        void onLabPose(const geometry_msgs::msg::PoseStamped & message);

        /** Publishes health and the heartbeat, and sends the origin when due (process executor). */
        void onHealthTimer();

        void startLabSide();

        void stopLabSide();

    };

    /**
     * @brief Reads the relay parameters, declaring them read-only. A parameter
     * whose override has the wrong type keeps its default and adds an error.
     *
     * @param node The node to declare the parameters on.
     * @param errors Receives one error per mistyped parameter.
     */
    opti_track_pose_relay::PoseRelayParameters DeclarePoseRelayParameters(
        rclcpp::Node & node,
        std::vector<std::string> & errors
    );

    /**
     * @brief External vision sample for EKF2: NED pose, unknown velocity
     * frame with NaN velocity, angular velocity and velocity variance,
     * quality 0, reset counter 0.
     *
     * @param pose The pose in PX4's frames.
     * @param timestamp_us timestamp and timestamp_sample [us].
     * @param position_variance_m2 Variance of each position axis.
     * @param orientation_variance_rad2 Variance of each orientation axis.
     */
    px4_msgs::msg::VehicleOdometry MakeVisualOdometry(
        const opti_track_pose_relay::NedPose & pose,
        uint64_t timestamp_us,
        double position_variance_m2,
        double orientation_variance_rad2
    );

    /**
     * @brief VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN with param5 latitude, param6
     * longitude and param7 altitude, addressed like the other III vehicle
     * commands (source system and component 1, from external).
     */
    px4_msgs::msg::VehicleCommand MakeSetGpsGlobalOriginCommand(
        double latitude_deg,
        double longitude_deg,
        double altitude_m,
        uint64_t timestamp_us,
        uint8_t target_system,
        uint16_t target_component
    );

    /**
     * @brief The relay's DiagnosticStatus: level and message from the report,
     * and the values input_rate_hz, output_rate_hz, last_input_age_ms,
     * max_input_gap_ms, lab_stamp_age_ms, stale, origin_sent, rigid_body_id
     * and rejected_samples ("nan" for an unknown number).
     */
    diagnostic_msgs::msg::DiagnosticStatus MakeHealthStatus(
        const opti_track_pose_relay::HealthReport & report,
        bool origin_sent,
        int64_t rigid_body_id
    );

} // namespace opti_track_pose_relay_node
} // namespace utils
} // namespace iii_drone
