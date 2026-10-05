#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <memory>
#include <thread>
#include <atomic>
#include <limits>
#include <optional>
#include <chrono>
#include <array>
#include <cstdint>
#include <mutex>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

/*****************************************************************************/
// PX4 msgs:

#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/msg/powerline.hpp>
#include <iii_drone_interfaces/msg/gripper_status.hpp>
#include <iii_drone_interfaces/msg/target.hpp>
#include <iii_drone_interfaces/msg/combined_drone_awareness.hpp>
#include <iii_drone_interfaces/msg/state.hpp>

#include <iii_drone_interfaces/srv/register_offboard_mode.hpp>

/*****************************************************************************/
// III-Drone-Configuration:

#include <iii_drone_configuration/configuration.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/types.hpp>
#include <iii_drone_core/utils/math.hpp>

#include <iii_drone_core/utils/atomic.hpp>
#include <iii_drone_core/utils/history.hpp>
#include <iii_drone_core/utils/callback_lifetime.hpp>

#include <iii_drone_core/adapters/px4/vehicle_status_adapter.hpp>
#include <iii_drone_core/adapters/px4/vehicle_odometry_adapter.hpp>
#include <iii_drone_core/adapters/px4/vehicle_global_position_adapter.hpp>
#include <iii_drone_core/adapters/powerline_adapter.hpp>
#include <iii_drone_core/adapters/single_line_adapter.hpp>
#include <iii_drone_core/adapters/gripper_status_adapter.hpp>
#include <iii_drone_core/adapters/target_adapter.hpp>
#include <iii_drone_core/adapters/state_adapter.hpp>
#include <iii_drone_core/adapters/combined_drone_awareness_adapter.hpp>

#include <iii_drone_core/control/state.hpp>
#include <iii_drone_core/control/hover_thrust_meter.hpp>
#include <iii_drone_core/control/reference.hpp>
#include <iii_drone_core/control/position_continuity_identity.hpp>

#include <iii_drone_core/control/maneuver/maneuver_types.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {

namespace control {

    struct MeasuredOdometrySnapshot {
        State state;
        // ROS receipt time is comparable with the command-emission clock.
        // PX4 sample identity, independent of its publication timestamp.
        rclcpp::Time receipt_stamp{0, 0, RCL_ROS_TIME};
        uint64_t source_sample_timestamp_us = 0;
        uint8_t reset_counter = 0;
        PositionContinuityIdentity position_continuity{};
    };

    /** Fixed, process-local receipt evidence. Steady times share one clock epoch. */
    struct OdometryIngressEvent {
        uint64_t source_sample_timestamp_us = 0;
        int64_t callback_receipt_ros_ns = 0;
        int64_t callback_entry_steady_ns = 0;
        int64_t lock_acquired_steady_ns = 0;
        int64_t completed_steady_ns = 0;
        int64_t accepted_steady_ns = 0;
        uint8_t reset_counter = 0;
        bool accepted = false;
        bool pending_before = false;
        bool pending_after = false;
    };

    struct OdometryIngressDiagnostics {
        static constexpr size_t history_capacity = 64;
        bool available = false;
        bool busy = false;
        bool latest_available = false;
        uint64_t latest_source_sample_timestamp_us = 0;
        uint8_t latest_reset_counter = 0;
        int64_t latest_receipt_ros_ns = 0;
        int64_t latest_accepted_steady_ns = 0;
        uint64_t total_callbacks = 0;
        size_t history_count = 0;
        std::array<OdometryIngressEvent, history_capacity> history{};
    };

    struct VehicleNavigationSample {
        uint64_t source_timestamp_us = 0;
        uint64_t nav_state_timestamp_us = 0;
        uint8_t nav_state = 0;
        // PX4 vehicle_status.failsafe of this exact sample.
        bool failsafe = false;
        std::chrono::steady_clock::time_point receipt;
    };

    struct VehicleNavigationEvidence {
        std::optional<VehicleNavigationSample> latest;
        std::optional<VehicleNavigationSample> last_external;
        // A raw PX4 clock regression invalidates all earlier owner epochs.
        uint64_t source_epoch = 0;
    };

    /** Freshness bound of PX4 navigation evidence (vehicle_status cadence plus jitter). */
    constexpr std::chrono::milliseconds kVehicleNavigationFreshness{1500};

    /**
     * Operator native control: the latest fresh PX4 sample is a PX4-native
     * navigation state (not OFFBOARD and not an external mode) and PX4 is not
     * in failsafe. This is how an operator (or test driver) ends a mission by
     * selecting Hold/Position/... . Stale, missing or failsafe evidence is not
     * operator control and keeps fault classification loud.
     */
    bool IsOperatorNativeControl(
        const VehicleNavigationEvidence & navigation,
        std::chrono::steady_clock::time_point now
    );

    /**
     * @brief Class which subscribes to various topics related to the drone awareness and keeps track of the current combined awareness.
     * Does the following:
     * - Subscribes to the PX4 vehicle status and odometry topics, the powerline topic, the gripper status topic, and the target cable id topic.
     * - Keeps track of the history of the received messages.
     * - Updates the combined drone awareness based on the received messages.
     * - Keeps and updates an estimate of the ground altitude and publishes the ground frame to tf2.
     * To be used from a ROS2 node. 
     * Construction of this object will create subscriptions and is not thread-safe.
     */
    class CombinedDroneAwarenessHandler {
        using VehicleStatusAdapterHistory = iii_drone::utils::History<iii_drone::adapters::px4::VehicleStatusAdapter>;
        using VehicleOdometryAdapterHistory = iii_drone::utils::History<iii_drone::adapters::px4::VehicleOdometryAdapter>;
        using VehicleGlobalPositionAdapterHistory = iii_drone::utils::History<iii_drone::adapters::px4::VehicleGlobalPositionAdapter>;
        using PowerlineAdapterHistory = iii_drone::utils::History<iii_drone::adapters::PowerlineAdapter>;
        using GripperStatusAdapterHistory = iii_drone::utils::History<iii_drone::adapters::GripperStatusAdapter>;

        using AtomicIntVector = iii_drone::utils::Atomic<std::vector<int>>;

    public:

        /**
         * @brief Construct a new CombinedDroneAwarenessHandler object, initializing all the subscriptions.
         * 
         * @param params Shared pointer to the combined drone awareness handler configuration view.
         * @param tf_buffer Shared pointer to the tf2 buffer.
         * @param node Simple pointer to the containing node.
         * @param debug Whether to print debug messages.
         */
        CombinedDroneAwarenessHandler(
            iii_drone::configuration::Configuration::SharedPtr params,
            tf2_ros::Buffer::SharedPtr tf_buffer,
            rclcpp_lifecycle::LifecycleNode * node,
            bool debug = false
        );

        /**
         * @brief Destructor
         */
        ~CombinedDroneAwarenessHandler();

        /**
         * @brief Starts the combined drone awareness handler.
         */
        void Start();

        /**
         * @brief Stops the combined drone awareness handler.
         */
        void Stop();

        /**
         * @brief Get the current drone state.
         * 
         * @return The current drone state.
         */
        iii_drone::control::State GetState() const;

        /** Latest measured odometry with its source and receipt metadata. */
        std::optional<MeasuredOdometrySnapshot> GetMeasuredOdometry() const;
        /** Nonblocking diagnostic copy; busy means no ingress lock was acquired. */
        OdometryIngressDiagnostics TryGetOdometryIngressDiagnostics() const;
        static std::optional<MeasuredOdometrySnapshot> AdvanceMeasuredOdometry(
            std::optional<MeasuredOdometrySnapshot> previous,
            const iii_drone::adapters::px4::VehicleOdometryAdapter & adapter,
            uint64_t source_sample_timestamp_us,
            const rclcpp::Time & receipt_stamp
        );
        VehicleNavigationEvidence GetVehicleNavigationEvidence() const;
        /** IsOperatorNativeControl() on the latest navigation evidence. */
        bool OperatorNativeControl() const;
        static VehicleNavigationEvidence AdvanceVehicleNavigation(
            VehicleNavigationEvidence previous,
            const px4_msgs::msg::VehicleStatus & status,
            std::chrono::steady_clock::time_point receipt,
            bool external_mode
        );

        /**
         * @brief Whether status and odometry needed to form a vehicle state are available.
         *
         * Status can arrive before odometry after an XRCE reconnect or controller
         * lifecycle recovery.  Maneuvers must not treat that partial snapshot as
         * a usable state.
         */
        bool state_available() const;

        /**
         * @brief Computes the target state of the drone given a target adapter.
         * 
         * @param target_adapter The target adapter.
         * 
         * @return The target state.
         */
        iii_drone::control::State ComputeTargetState(const iii_drone::adapters::TargetAdapter & target_adapter) const;

        /**
         * @brief Computes the target transform world to drone given a target adapter.
         * 
         * @param target_adapter The target adapter.
         * 
         * @return The target transform.
         */
        iii_drone::types::transform_matrix_t ComputeTargetTransform(const iii_drone::adapters::TargetAdapter & target_adapter) const;

        /**
         * @brief Gets the pose of the target object in the world frame (not considering the target transform).
         * 
         * @param target_adapter The target adapter.
         * 
         * @return The pose of the target.
         */
        iii_drone::types::pose_t GetPoseOfTarget(const iii_drone::adapters::TargetAdapter & target_adapter) const;

        /**
         * @brief Gets the latest powerline adapter.
         *
         * @return The latest powerline adapter.
         */
        iii_drone::adapters::PowerlineAdapter GetPowerlineAdapter() const;

        /**
         * @brief Sets the current target.
         * 
         * @param target_adapter The target adapter.
         * 
         * @return void
         */
        void SetTarget(iii_drone::adapters::TargetAdapter target_adapter);

        /**
         * @brief Clears the current target.
         * 
         * @return void
         */
        void ClearTarget();

        /**
         * @brief Get adapter.
         * 
         * @return The combined drone awareness adapter.
         */
        const iii_drone::adapters::CombinedDroneAwarenessAdapter adapter() const;

        /**
         * @brief Whether the drone is armed.
         * 
         * @return true if the drone is armed.
         */
        bool armed() const;

        /**
         * @brief Whether the drone is in offboard mode.
         * 
         * @return true if the drone is in offboard mode.
         */
        bool offboard() const;

        /**
         * @brief Whether the drone has a target.
         * 
         * @return true if the drone has a target.
         */
        bool has_target() const;

        /**
         * @brief Whether the position of the target is known.
         * 
         * @return true if the position of the target is known.
         */
        bool target_position_known() const;

        /**
         * @brief Returns the current target adapter, will have TARGET_TYPE_NONE if no target.
         * 
         * @return The current target adapter.
         */
        iii_drone::adapters::TargetAdapter target_adapter() const;

        /**
         * @brief Whether the drone is on the ground.
         * 
         * @return true if the drone is on the ground.
         */
        bool on_ground() const;

        /**
         * @brief Whether the drone is on a cable.
         * 
         * @return true if the drone is on a cable.
         */
        bool on_cable() const;

        /**
         * @brief Whether the drone is in flight.
         * 
         * @return true if the drone is in flight.
         */
        bool in_flight() const;

        /**
         * @brief Return the id of the cable that the drone is on if it is on a cable, -1 otherwise.
         * This id can be different from the target cable id.
         * 
         * @return The id of the cable that the drone is on if it is on a cable, -1 otherwise.
         */
        int on_cable_id() const;

        /**
         * @brief Returns the ground altitude estimate.
         * 
         * @return The ground altitude estimate.
         */
        double ground_altitude_estimate() const;

        /**
         * @brief Returns the drone location, checks the drone location and returns the on_cable_id if the drone is on a cable.
         * 
         * @param on_cable_id The id of the cable that the drone is on if it is on a cable, -1 otherwise.
         * 
         * @return The drone location.
         */
        iii_drone::adapters::drone_location_t drone_location(int &on_cable_id) const;

        /**
         * @brief Returns the drone location.
         * 
         * @return The drone location.
         */
        iii_drone::adapters::drone_location_t drone_location() const;

        /**
         * @brief Returns whether the gripper is open.
         * 
         * @return true if the gripper is open.
         */
        bool gripper_open() const;

        /**
         * @brief PX4's land detector stages, as last received.
         */
        struct Px4LandState {
            bool landed = true;
            bool maybe_landed = true;
            bool ground_contact = true;
            std::chrono::steady_clock::time_point received_at{};
        };

        /**
         * @brief The last PX4 land-detector sample, if any was received.
         */
        std::optional<Px4LandState> px4_land_state() const;

        /**
         * @brief Whether PX4 reports the vehicle airborne: a sample no older
         * than kPx4LandStateMaxAge (PX4 republishes at least at 1 Hz) with
         * none of landed, maybe landed or ground contact set. Unknown counts
         * as not airborne.
         *
         * Far above the ground PX4 reports airborne as soon as it arms, while
         * its takeoff state machine still holds thrust at zero; see
         * px4_thrust_up() for the thrust PX4 applies.
         */
        bool px4_airborne(std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now()) const;

        static constexpr std::chrono::milliseconds kPx4LandStateMaxAge{2500};

        /**
         * @brief PX4 position controller output, as last received.
         */
        struct Px4ThrustSetpoint {
            /** Upward collective thrust (normalized, NED z negated). */
            double thrust_up = 0.0;
            /** Upward acceleration setpoint the thrust realizes (m/s^2). */
            double acceleration_up = 0.0;
            std::chrono::steady_clock::time_point received_at{};
        };

        /**
         * @brief The thrust PX4's position controller commands, if a sample
         * no older than kPx4ThrustSetpointMaxAge (PX4 publishes it every
         * control cycle) is available. It stays zero until PX4 has taken off.
         */
        std::optional<Px4ThrustSetpoint> px4_thrust_setpoint(
            std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now()) const;

        static constexpr std::chrono::milliseconds kPx4ThrustSetpointMaxAge{1000};

        /**
         * @brief The thrust the vehicle actually needs to hover, measured from
         * PX4's commanded thrust in the latest steady free flight (not on the
         * cable). Kept while disarmed.
         */
        std::optional<HoverThrustMeter::Estimate> measured_hover_thrust() const;

        /**
         * @brief Returns the tf buffer shared ptr.
         * 
         * @return The tf buffer shared ptr.
         */
        tf2_ros::Buffer::SharedPtr tf_buffer() const;

        /**
         * @brief Shared pointer type for the CombinedDroneAwarenessHandler.
         */
        typedef std::shared_ptr<CombinedDroneAwarenessHandler> SharedPtr;

    private:
        /**
         * @brief Whether to print debug messages.
         */
        bool debug_;

        /**
         * @brief Is started flag
         */
        iii_drone::utils::Atomic<bool> is_started_ = false;

        /**
         * @brief Parameters for the combined drone awareness handler.
         */
        iii_drone::configuration::Configuration::SharedPtr configuration_;

        /**
         * @brief The tf2 buffer.
         */
        tf2_ros::Buffer::SharedPtr tf_buffer_;

        /**
         * @brief The tf2 broadcaster fro publishing ground frame.
         */
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        /**
         * @brief Simple pointer to the containing node.
         */
        rclcpp_lifecycle::LifecycleNode * node_;

        /**
         * @brief Atomic vector of navigation state ids which are considered to be offboard.
         */
        AtomicIntVector offboard_nav_state_ids_;

        /**
         * @brief Register a new navigation state id which should be considered offboard.
         * 
         * @param navigation_state_id The navigation state id to be considered offboard.
         * 
         * @return void
         */
        void registerOffboardMode(int navigation_state_id);

        /**
         * @brief De-register a navigation state id which should no longer be considered offboard.
         * 
         * @param navigation_state_id The navigation state id to no longer be considered offboard.
         * 
         * @return void
         */
        void deregisterOffboardMode(int navigation_state_id);

        /**
         * @brief Register offboard mode service.
         */
        rclcpp::Service<iii_drone_interfaces::srv::RegisterOffboardMode>::SharedPtr register_offboard_mode_srv_;

        /**
         * @brief Atomic combined drone awareness adapter member.
         */
        iii_drone::utils::Atomic<iii_drone::adapters::CombinedDroneAwarenessAdapter>::SharedPtr combined_drone_awareness_adapter_;

        /**
         * @brief Updates the combined drone awareness based on current information.
         * To be called after updating the any internal awareness information.
         * 
         * @return void
         */
        void updateCombinedDroneAwareness();

        /**
         * @brief Combined drone awareness publisher timer.
         */
        rclcpp::TimerBase::SharedPtr combined_drone_awareness_pub_timer_;

        /**
         * @brief Combined drone awareness publisher.
         */
        rclcpp_lifecycle::LifecyclePublisher<iii_drone_interfaces::msg::CombinedDroneAwareness>::SharedPtr combined_drone_awareness_pub_;

        /**
         * @brief Target pose publisher.
         */
        rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;

        /**
         * @brief Target drone pose publisher.
         */
        rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_drone_pose_pub_;

        /**
         * @brief Has found initial location flag.
         */
        iii_drone::utils::Atomic<bool> has_found_initial_location_ = false;

		/**
		 * @brief PX4 vehicle status subscription
		 */
		rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;

		/**
		 * @brief PX4 land detector subscription and its last sample.
		 */
		rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr vehicle_land_detected_sub_;
		iii_drone::utils::Atomic<std::optional<Px4LandState>> px4_land_state_;
		rclcpp::Subscription<px4_msgs::msg::VehicleLocalPositionSetpoint>::SharedPtr vehicle_local_position_setpoint_sub_;
		iii_drone::utils::Atomic<std::optional<Px4ThrustSetpoint>> px4_thrust_setpoint_;
		HoverThrustMeter hover_thrust_meter_;
		mutable std::mutex hover_thrust_meter_mutex_;

        /**
         * @brief Vehicle status adapter history.
        */
        VehicleStatusAdapterHistory::SharedPtr vehicle_status_adapter_history_;
        iii_drone::utils::Atomic<VehicleNavigationEvidence> vehicle_navigation_evidence_;

        /**
         * @brief Updates the combined drone awareness from the vehicle status.
         * Triggers updates of armed, offboard, and location.
         * 
         * @return void
         */
        void updateCombinedDroneAwarenessFromVehicleStatus();

        /**
         * @brief Updates the given combined drone awareness from the vehicle status.
         * Triggers updates of armed, offboard, and location.
         * 
         * @param combined_drone_awareness_adapter The combined drone awareness adapter to update.
         * 
         * @return void
         */
        void updateCombinedDroneAwarenessFromVehicleStatus(iii_drone::adapters::CombinedDroneAwarenessAdapter & combined_drone_awareness_adapter);

		/**
		 * @brief PX4 odometry subscription
		 */
		rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
		// Odometry and local-position ingress (one measured-odometry
		// transaction) run on their own executor thread: object tracking needs
		// every sample promptly, and neither a busy callback group nor an
		// exhausted executor thread pool of the node may delay them. The queue
		// absorbs brief OS scheduling stalls.
		rclcpp::CallbackGroup::SharedPtr odometry_callback_group_;
		std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> odometry_executor_;
		std::thread odometry_thread_;
		void startOdometryIngress();
		void stopOdometryIngress();
		static constexpr size_t odometry_queue_depth_ = 64;  // 0.5 s at 125 Hz
		// Serialises the read-modify-write updates of the awareness snapshot,
		// which now run from more than one callback group.
		std::mutex awareness_update_mutex_;
		iii_drone::utils::CallbackLifetime callback_lifetime_;
		rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;

        /**
         * @brief Vehicle odometry adapter history.
        */
        VehicleOdometryAdapterHistory::SharedPtr vehicle_odometry_adapter_history_;
        iii_drone::utils::Atomic<std::optional<MeasuredOdometrySnapshot>> measured_odometry_;
        // Bumped whenever measured_odometry_ changes: a local-position message
        // that changes nothing (most of them, 100 Hz) does not recompute the
        // awareness.
        std::atomic<uint64_t> measured_odometry_version_{0};
        // AMSL altitude of PX4's local origin (vehicle_local_position.ref_alt),
        // NaN while it has no global reference. PX4 derives the global
        // altitude from the same reference, so the ground estimate's AMSL
        // value needs no 100 Hz global-position subscription.
        std::atomic<double> local_reference_altitude_amsl_{std::numeric_limits<double>::quiet_NaN()};

        struct LocalResetMetadata {
            uint64_t source_sample_us = 0;
            rclcpp::Time receipt{0, 0, RCL_ROS_TIME};
            uint8_t xy = 0, z = 0, vxy = 0, vz = 0, heading = 0;
            bool xy_global = false, z_global = false;
            uint64_t origin_timestamp_us = 0;
            double origin_lat = 0, origin_lon = 0;
            float origin_alt = 0;
            uint8_t aggregate() const {
                return static_cast<uint8_t>(xy + z + vxy + vz + heading);
            }
        };
        struct PendingOdometry {
            px4_msgs::msg::VehicleOdometry message;
            rclcpp::Time receipt{0, 0, RCL_ROS_TIME};
        };
        // The two DDS callbacks and the measured snapshot form one transaction.
        mutable std::mutex odometry_ingest_mutex_;
        std::array<OdometryIngressEvent,
            OdometryIngressDiagnostics::history_capacity> odometry_ingress_history_{};
        size_t odometry_ingress_next_ = 0;
        size_t odometry_ingress_count_ = 0;
        uint64_t odometry_ingress_total_callbacks_ = 0;
        uint64_t accepted_odometry_samples_ = 0;
        bool latest_accepted_odometry_available_ = false;
        uint64_t latest_accepted_source_sample_us_ = 0;
        uint8_t latest_accepted_reset_counter_ = 0;
        int64_t latest_accepted_receipt_ros_ns_ = 0;
        int64_t latest_accepted_steady_ns_ = 0;
        std::optional<LocalResetMetadata> latest_local_reset_;
        std::optional<LocalResetMetadata> verified_local_reset_;
        std::optional<PendingOdometry> pending_odometry_;
        bool local_provenance_invalid_ = false;
        uint64_t odometry_source_epoch_ = 0;
        uint64_t position_epoch_ = 0;
        void ingestVehicleOdometry(const px4_msgs::msg::VehicleOdometry & message,
            const rclcpp::Time & receipt);
        void ingestVehicleLocalPosition(const px4_msgs::msg::VehicleLocalPosition & message,
            const rclcpp::Time & receipt);
        void acceptMeasuredOdometry(const px4_msgs::msg::VehicleOdometry & message,
            const rclcpp::Time & receipt, bool qualified, bool new_position_epoch,
            bool force_source_fault = false);
        bool metadataMatches(const LocalResetMetadata & metadata,
            const px4_msgs::msg::VehicleOdometry & message,
            const rclcpp::Time & receipt) const;
        static bool headingOnly(const LocalResetMetadata & before,
            const LocalResetMetadata & after);
        static bool samePositionBasis(const LocalResetMetadata & before,
            const LocalResetMetadata & after);
        // A PX4 timesync filter reset publishes a few samples with a zero
        // offset (boot-relative stamps between agent-clock stamps). These
        // report whether a regressing sample is such an isolated stamp-only
        // anomaly on an unchanged, source-qualified, fresh position basis.
        // Discarding it never refreshes a receipt; sustained regressions age
        // past the 250 ms bound and fence as before.
        bool isolatedOdometryStampRegression(
            const MeasuredOdometrySnapshot & previous,
            const px4_msgs::msg::VehicleOdometry & message,
            const rclcpp::Time & receipt) const;
        bool isolatedLocalStampRegression(const LocalResetMetadata & metadata) const;
        uint64_t discarded_stamp_regressions_ = 0;
        void logResetClassification(bool heading_only, const char * context,
            uint8_t from_counter, uint8_t to_counter, uint64_t odometry_source_us,
            uint64_t local_source_us, uint64_t prior_local_source_us) const;

        /**
         * @brief Updates the combined drone awareness from the vehicle odometry.
         * Triggers updates of ground altitude estimate and location.
         * 
         * @return void
         */
        void updateCombinedDroneAwarenessFromVehicleOdometry();

        /**
         * @brief Updates the given combined drone awareness from the vehicle odometry.
         * Triggers updates of ground altitude estimate and location.
         * 
         * @param combined_drone_awareness_adapter The combined drone awareness adapter to update.
         * 
         * @return void
         */
        void updateCombinedDroneAwarenessFromVehicleOdometry(iii_drone::adapters::CombinedDroneAwarenessAdapter & combined_drone_awareness_adapter);


		/**
		 * @brief Powerline subscription
		 */
		rclcpp::Subscription<iii_drone_interfaces::msg::Powerline>::SharedPtr powerline_sub_;

        /**
         * @brief Powerline adapter history.
        */
        PowerlineAdapterHistory::SharedPtr powerline_adapter_history_;

        /**
         * @brief Updates the combined drone awareness from the powerline.
         * Triggers updates of target_cable_position_known and location.
         * 
         * @return void
         */
        void updateCombinedDroneAwarenessFromPowerline();

        /**
         * @brief Updates the given combined drone awareness from the powerline.
         * Triggers updates of target_cable_position_known and location.
         * 
         * @param combined_drone_awareness_adapter The combined drone awareness adapter to update.
         * 
         * @return void
         */
        void updateCombinedDroneAwarenessFromPowerline(iii_drone::adapters::CombinedDroneAwarenessAdapter & combined_drone_awareness_adapter);

		/**
		 * @brief Gripper status subscription
		 */
		rclcpp::Subscription<iii_drone_interfaces::msg::GripperStatus>::SharedPtr gripper_status_sub_;

        /**
         * @brief Gripper status adapter history.
        */
        GripperStatusAdapterHistory::SharedPtr gripper_status_adapter_history_;

        // Gripper status arrives at 50-100 Hz; the awareness only depends on
        // whether the gripper is open, so it is recomputed when that changes.
        std::optional<bool> last_gripper_open_;

        /**
         * @brief Updates the combined drone awareness from the gripper status.
         * Triggers updates of gripper_open.
         * 
         * @return void
         */
        void updateCombinedDroneAwarenessFromGripperStatus();

        /**
         * @brief Updates the given combined drone awareness from the gripper status.
         * Triggers updates of gripper_open.
         * 
         * @param combined_drone_awareness_adapter The combined drone awareness to update.
         * 
         * @return void
         */
        void updateCombinedDroneAwarenessFromGripperStatus(iii_drone::adapters::CombinedDroneAwarenessAdapter & combined_drone_awareness_adapter);

        /**
         * @brief Atomic target adapter member.
         */
        iii_drone::utils::Atomic<iii_drone::adapters::TargetAdapter>::SharedPtr target_adapter_;

        /**
         * @brief Target publisher.
         */
        rclcpp_lifecycle::LifecyclePublisher<iii_drone_interfaces::msg::Target>::SharedPtr target_pub_;

        /**
         * @brief Updates the combined drone awareness from the target adapter.
         * Triggers updates of target_adapter, target_position_known, and has_target.
         * 
         * @return void
         */
        void updateCombinedDroneAwarenessFromTarget();

        /**
         * @brief Updates the given combined drone awareness from the target adapter.
         * Triggers updates of target_adapter, target_position_known, and has_target.
         * 
         * @param combined_drone_awareness_adapter The combined drone awareness to update.
         * 
         * @return void
         */
        void updateCombinedDroneAwarenessFromTarget(iii_drone::adapters::CombinedDroneAwarenessAdapter & combined_drone_awareness_adapter);

        /**
         * @brief Atomic ground altitude estimate member.
         */
        iii_drone::utils::Atomic<double>::SharedPtr ground_altitude_estimate_;

        /**
         * @brief Atomic ground altitude estimate AMSL member.
         */
        iii_drone::utils::Atomic<double>::SharedPtr ground_altitude_estimate_amsl_;

        /**
         * @brief Ground altitudes history.
         */
        iii_drone::utils::History<double>::SharedPtr ground_altitudes_history_;

        /**
         * @brief Timer for updating the ground altitude estimate.
         */
        rclcpp::TimerBase::SharedPtr ground_altitude_update_timer_;

        // Last published ground frame (only rviz shows it): published when it
        // moves or once a second, not at the 20 Hz estimate rate, since every
        // /tf listener receives each message.
        double ground_tf_published_altitude_ = std::numeric_limits<double>::quiet_NaN();
        std::chrono::steady_clock::time_point ground_tf_published_at_{};

        /**
         * @brief Updates the ground altitude estimate based on given information.
         * Will either start or stop the timer based on whether the drone is on the ground.
         * If the drone is on the ground, and the timer is not running (has elapsed), will start the timer and update the ground altitude estimate.
         * If the drone is on the ground, and the timer is running, will do nothing.
         * If the drone is not on the ground, and the timer is running, will stop the timer.
         * To be called after updates to vehicle odometry, vehicle status, or target cable.
         * Will set the ground altitude estimate to the current altitude if the drone is on the ground, based on the following criteria:
         * - The drone is unarmed; and
         * - No target cable is registered.
         * The ground altitude estimate is used to determine the drone location (on ground, in flight, on cable).
         * Updates the ground altitude estimate by pushing the current altitude to the history and taking the mean of the history.
         * 
         * @param armed Whether the drone is armed.
         * @param has_target_cable Whether the drone has a target cable.
         * 
         * @return void
         */
        void updateGroundAltitudeEstimate(
            bool armed,
            bool has_target_cable
        );

        /**
         * @brief Updates the drone location of a combined drone awareness object based on current awareness information.
         * Additionally updates the on_cable_id member by evaluating whether the drone is currently on a cable. 
         * on_cable_id will be set to -1 if the drone is not on a cable.
         * on_cable_id can be different from target_cable_id.
         * 
         * @param awareness The combined drone awareness adapter to update.
         * 
         * @return void
         */
        void updateDroneLocation(iii_drone::adapters::CombinedDroneAwarenessAdapter & awareness_adapter);

        /**
         * @brief Timer for publishing members.
         */
        rclcpp::TimerBase::SharedPtr publish_timer_;

        /**
         * @brief Publishes the members.
         * 
         * @return void
         */
        void publishMembers();

    };

} // namespace control
} // namespace iii_drone
