#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <array>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

/*****************************************************************************/
// Classes and functions
/*****************************************************************************/

/**
 * ROS-free logic of the OptiTrack pose relay (opti_track_pose_relay): the lab
 * gateway's motion-capture pose of one rigid body is converted to PX4's frames
 * and forwarded to EKF2 as external vision, rate limited and never stale, and
 * PX4 is given an EKF global origin while it has none.
 */
namespace iii_drone {
namespace utils {
namespace opti_track_pose_relay {

    /**
     * @brief Relay configuration, ROS parameters /opti_track/pose_relay/<field>.
     */
    struct PoseRelayParameters {
        /** Motive rigid-body ID (Motive names the body by it); -1 is unset. */
        int64_t rigid_body_id = -1;
        /** ROS domain of the lab gateway. */
        int64_t lab_ros_domain_id = 0;
        /** Upper bound of the rate poses are forwarded at [Hz]. */
        double output_rate_hz = 50.0;
        /** A pose that arrived longer ago than this is never forwarded [s]. */
        double stale_timeout_s = 0.15;
        /** Variance reported for each position axis [m^2]. */
        double position_variance_m2 = 0.0001;
        /** Variance reported for each orientation axis [rad^2]. */
        double orientation_variance_rad2 = 0.0004;
        /** Whether to set PX4's EKF global origin while it has none. */
        bool send_origin = true;
        /** EKF global origin sent to PX4. */
        double origin_latitude_deg = 55.3672;
        double origin_longitude_deg = 10.4310;
        double origin_altitude_m = 20.0;
    };

    /** Prefix of the relay's ROS parameter names. */
    constexpr const char * kParameterPrefix = "/opti_track/pose_relay/";

    /**
     * @brief Every reason the parameters are invalid, naming the parameter;
     * empty when they are valid.
     */
    std::vector<std::string> ValidatePoseRelayParameters(const PoseRelayParameters & parameters);

    /** The lab gateway's pose topic of a rigid body: /body_splitter/body_<id>/pose. */
    std::string LabPoseTopic(int64_t rigid_body_id);

    /**
     * @brief A pose in the lab frame: world Z up, body forward-left-up.
     */
    struct LabPose {
        /** x, y, z [m]. */
        std::array<double, 3> position{0.0, 0.0, 0.0};
        /** w, x, y, z: rotation from the body to the world frame. */
        std::array<double, 4> orientation{1.0, 0.0, 0.0, 0.0};
    };

    /**
     * @brief A pose in PX4's frames: world north-east-down, body forward-right-down.
     */
    struct NedPose {
        /** North, east, down [m]. */
        std::array<double, 3> position{0.0, 0.0, 0.0};
        /** w, x, y, z: unit rotation from the FRD body to the NED world frame. */
        std::array<double, 4> orientation{1.0, 0.0, 0.0, 0.0};
    };

    /** Largest accepted deviation of the lab quaternion's norm from one. */
    constexpr double kQuaternionNormTolerance = 0.1;

    /**
     * @brief Converts a lab pose to PX4's frames. Both frames are rotated by
     * 180 degrees about x (Z up to down, left to right): position (x, -y, -z),
     * quaternion (w, x, -y, -z), normalised.
     *
     * @param pose The lab pose.
     * @param rejection_reason Set when the pose is rejected (may be nullptr).
     *
     * @return std::optional<NedPose> The PX4 pose; empty for a non-finite
     * position or quaternion, or a degenerate quaternion (norm not within
     * kQuaternionNormTolerance of one).
     */
    std::optional<NedPose> LabPoseToNed(
        const LabPose & pose,
        std::string * rejection_reason = nullptr
    );

    /**
     * @brief Selects the received poses that are forwarded, at the moment
     * they arrive (no buffering, so no added latency):
     * - at most output_rate_hz on average: a pose is forwarded once the next
     *   output period is due, and never within half a period of the previous;
     * - never a pose that arrived more than stale_timeout_s ago, and never one
     *   older than the last forwarded pose;
     * - after a gap of over a period (a stale stream, or input slower than
     *   the output rate) the schedule restarts at the first fresh pose, so a
     *   resumed stream is not caught up in a burst.
     * Times are steady-clock nanoseconds. Not thread-safe.
     */
    class OutputGate {
    public:
        /**
         * @brief Constructor.
         *
         * @param output_rate_hz Average forward rate bound [Hz], > 0.
         * @param stale_timeout_s Largest forwarded pose age [s], > 0.
         */
        OutputGate(double output_rate_hz, double stale_timeout_s);

        /**
         * @brief Offers a received pose. A pose for which this returns true
         * must be forwarded now.
         *
         * @param arrival_ns When the pose arrived.
         * @param now_ns The current time.
         *
         * @return bool True to forward the pose.
         */
        bool Offer(int64_t arrival_ns, int64_t now_ns);

    private:
        int64_t period_ns_;
        int64_t min_spacing_ns_;
        int64_t stale_timeout_ns_;
        bool forwarded_any_ = false;
        int64_t last_forward_ns_ = 0;
        int64_t last_forwarded_arrival_ns_ = 0;
        int64_t next_due_ns_ = 0;
    };

    /**
     * @brief Health level, as diagnostic_msgs/DiagnosticStatus levels.
     */
    enum class HealthLevel {
        OK,
        WARN,
        ERROR
    };

    /**
     * @brief Relay health over one report period.
     */
    struct HealthReport {
        HealthLevel level = HealthLevel::ERROR;
        std::string message;
        /** Valid poses received per second over the period. */
        double input_rate_hz = 0.0;
        /** Poses forwarded per second over the period. */
        double output_rate_hz = 0.0;
        /** Age of the last valid pose [ms]; NaN before the first. */
        double last_input_age_ms = std::numeric_limits<double>::quiet_NaN();
        /** Longest interval without a valid pose during the period, including the open one [ms]; NaN before the first. */
        double max_input_gap_ms = std::numeric_limits<double>::quiet_NaN();
        /** Arrival minus header stamp of the last valid pose [ms], informative only (lab and vehicle clocks differ); NaN if unstamped. */
        double lab_stamp_age_ms = std::numeric_limits<double>::quiet_NaN();
        /** No valid pose yet, or none within the stale timeout. */
        bool stale = true;
        /** A pose was forwarded within the stale timeout: the relay feeds PX4 fresh poses. */
        bool forwarding = false;
        /** Rejected (non-finite or degenerate) poses since start. */
        uint64_t rejected_samples = 0;
    };

    /**
     * @brief Collects the relay's input and output statistics and reports them
     * once per health period, including whether it is forwarding fresh poses
     * (one was forwarded within the stale timeout):
     * - ERROR: no valid pose yet, or the last one is older than the stale timeout;
     * - WARN: fresh, but during the period the stream had a gap longer than
     *   the stale timeout, the input rate was below half the output rate, or
     *   poses were rejected;
     * - OK otherwise.
     * Times are steady-clock nanoseconds. Not thread-safe.
     */
    class RelayHealthMonitor {
    public:
        /**
         * @brief Constructor.
         *
         * @param output_rate_hz The configured output rate [Hz].
         * @param stale_timeout_s The stale timeout [s].
         * @param start_ns Start of the first report period.
         */
        RelayHealthMonitor(double output_rate_hz, double stale_timeout_s, int64_t start_ns);

        /**
         * @brief Records a valid pose.
         *
         * @param arrival_ns When the pose arrived.
         * @param lab_stamp_age_ms Arrival minus header stamp [ms], NaN if unstamped.
         */
        void RecordInput(int64_t arrival_ns, double lab_stamp_age_ms);

        /** Records a rejected pose. */
        void RecordRejected();

        /**
         * @brief Records a forwarded pose.
         *
         * @param forwarded_ns When the pose was forwarded.
         */
        void RecordOutput(int64_t forwarded_ns);

        /**
         * @brief Reports the period ending now and starts the next one.
         */
        HealthReport Report(int64_t now_ns);

    private:
        double output_rate_hz_;
        int64_t stale_timeout_ns_;
        int64_t period_start_ns_;
        bool has_input_ = false;
        int64_t last_input_ns_ = 0;
        bool has_output_ = false;
        int64_t last_output_ns_ = 0;
        double lab_stamp_age_ms_ = std::numeric_limits<double>::quiet_NaN();
        uint64_t rejected_total_ = 0;
        uint64_t period_inputs_ = 0;
        uint64_t period_outputs_ = 0;
        uint64_t period_rejected_ = 0;
        int64_t period_max_gap_ns_ = 0;
    };

    /**
     * @brief Decides when to send PX4 the EKF global origin
     * (VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN): only while PX4 is disarmed, has no
     * global origin (local position xy_global false) and EKF2 intends to fuse
     * external vision position (estimator_status_flags.cs_ev_pos), each known
     * from a sample no older than the input timeout, and at most once per
     * resend interval. Sending stops while PX4 reports xy_global. Times are
     * steady-clock nanoseconds. Not thread-safe.
     */
    class OriginSender {
    public:
        /**
         * @brief Constructor.
         *
         * @param enabled Whether the origin is sent at all (send_origin).
         * @param resend_interval_ns Shortest interval between two sends.
         * @param input_timeout_ns Largest age of a PX4 sample the decision uses.
         */
        OriginSender(bool enabled, int64_t resend_interval_ns, int64_t input_timeout_ns);

        /** vehicle_status: arming_state is DISARMED. */
        void UpdateDisarmed(bool disarmed, int64_t now_ns);

        /** vehicle_local_position: xy_global. */
        void UpdateGlobalOrigin(bool xy_global, int64_t now_ns);

        /** estimator_status_flags: cs_ev_pos. */
        void UpdateVisionPositionFusion(bool cs_ev_pos, int64_t now_ns);

        /** True if the origin is to be sent now. */
        bool Due(int64_t now_ns) const;

        /** Records that the origin was sent now. */
        void MarkSent(int64_t now_ns);

        /** Whether the origin was sent at least once. */
        bool sent() const;

    private:
        struct Sample {
            bool value = false;
            int64_t received_ns = 0;
        };

        bool fresh(const std::optional<Sample> & sample, int64_t now_ns) const;

        bool enabled_;
        int64_t resend_interval_ns_;
        int64_t input_timeout_ns_;
        std::optional<Sample> disarmed_;
        std::optional<Sample> xy_global_;
        std::optional<Sample> cs_ev_pos_;
        std::optional<int64_t> last_sent_ns_;
    };

} // namespace opti_track_pose_relay
} // namespace utils
} // namespace iii_drone
