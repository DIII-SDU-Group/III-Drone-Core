#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <memory>
#include <queue>
#include <mutex>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <atomic>
#include <functional>
#include <optional>
#include <string>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/atomic.hpp>
#include <iii_drone_core/utils/history.hpp>

#include <iii_drone_core/control/reference.hpp>
#include <iii_drone_core/diagnostics/hil_trace.hpp>
#include <iii_drone_core/control/kinematic_stop_trajectory.hpp>
#include <iii_drone_core/control/maneuver/maneuver_reference_safety_guard.hpp>
#include <iii_drone_core/control/maneuver/maneuver_reference_startup_policy.hpp>
#include <iii_drone_core/control/maneuver/maneuver_reference_stream_guard.hpp>
#include <iii_drone_core/control/maneuver/maneuver_request_identity.hpp>

#include <iii_drone_core/adapters/px4/vehicle_odometry_adapter.hpp>
#include <iii_drone_core/adapters/reference_adapter.hpp>
#include <iii_drone_core/adapters/state_adapter.hpp>

/*****************************************************************************/
// III-Drone-Configuration:

#include <iii_drone_configuration/configuration.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/msg/reference.hpp>
#include <iii_drone_interfaces/msg/string_stamped.hpp>
#include <iii_drone_interfaces/msg/maneuver_reference_stream.hpp>
#include <iii_drone_interfaces/msg/maneuver_reference_ack.hpp>

#include <iii_drone_interfaces/srv/get_reference.hpp>
#include <iii_drone_interfaces/srv/pause_reference_stream.hpp>
#include <iii_drone_interfaces/srv/rebase_reference_stream.hpp>
#include <iii_drone_interfaces/srv/commit_reference_stream.hpp>
#include <iii_drone_interfaces/srv/terminal_hold_transfer.hpp>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {

namespace control {

namespace maneuver {

    /**
     * @brief The ManeuverReferenceClient class. This class creates a service client for getting references.
     * It exposes API for indicating the start and stop of a maneuver execution, as well as to update the reference to the current state.
     * It is used by a node calling maneuver actions from the maneuver controller node.
     * Upon starting a maneuver, the GetReference() method will fetch the reference using the service client.
     * Upon stopping a maneuver, the reference is reset to the current state.
     * The object can be set to reference mode passthrough or reference mode hover.
     * In passthrough mode, the GetReference() method returns the state.
     * In hover mode, the GetReference() method returns the last updated reference from the state.
     * The class starts in passthrough mode.
     */
    class ManeuverReferenceClient {
    public:
        /**
         * @brief Construct a new ManeuverReferenceClient object
         * 
         * @param node The node pointer.
         * @param vehicle_odometry_adapter_history Shared pointer to the vehicle odometry adapter history.
         * @param parameters Read-only live configuration view
         */
        template <typename NodeT>
        ManeuverReferenceClient(
            NodeT * node,
            iii_drone::utils::History<iii_drone::adapters::px4::VehicleOdometryAdapter>::SharedPtr vehicle_odometry_adapter_history,
            iii_drone::configuration::Configuration::SharedPtr parameters,
            rclcpp::CallbackGroup::SharedPtr get_reference_cb_group
        ) : get_reference_cb_group_(get_reference_cb_group),
            logger_(node->get_logger()),
            clock_(node->get_clock()),
            vehicle_odometry_adapter_history_(vehicle_odometry_adapter_history),
            reference_mode_(reference_mode_t::PASSTHROUGH),
            reference_(iii_drone::control::Reference()),
            maneuver_reference_valid_(false),
            configuration_(parameters) {

            get_reference_client_ = node->template create_client<iii_drone_interfaces::srv::GetReference>(
                "/control/maneuver_controller/get_reference",
                rclcpp::ServicesQoS(),
                get_reference_cb_group_
            );

            // Match the producer's reliable, bounded control-stream QoS.  The
            // consumer runs on the same vehicle host; losing every sample of a
            // newly-created maneuver generation is not an acceptable use of
            // best effort delivery.
            rclcpp::QoS stream_qos(rclcpp::KeepLast(5));
            stream_qos.reliable().durability_volatile();
            stream_qos.deadline(std::chrono::milliseconds(
                configuration_->GetParameter(
                    "/control/maneuver_controller/maneuver_execution_period_ms"
                ).as_int() * 2
            ));
            stream_qos.lifespan(std::chrono::milliseconds(
                configuration_->GetParameter(
                    "/control/maneuver_controller/reference_stream_timeout_ms"
                ).as_int()
            ));
            rclcpp::SubscriptionOptions stream_options;
            stream_options.callback_group = get_reference_cb_group_;
            reference_stream_subscription_ =
                node->template create_subscription<iii_drone_interfaces::msg::ManeuverReferenceStream>(
                    "/control/maneuver_controller/reference_stream",
                    stream_qos,
                    [this](const iii_drone_interfaces::msg::ManeuverReferenceStream::SharedPtr message) {
                        receiveReferenceStream(message);
                    },
                    stream_options
                );

            rclcpp::QoS ack_qos(rclcpp::KeepLast(10));
            ack_qos.reliable().durability_volatile();
            reference_ack_publisher_ =
                node->template create_publisher<iii_drone_interfaces::msg::ManeuverReferenceAck>(
                    "/control/maneuver_controller/reference_ack", ack_qos
                );
            pause_reference_stream_client_ =
                node->template create_client<iii_drone_interfaces::srv::PauseReferenceStream>(
                    "/control/maneuver_controller/pause_reference_stream",
                    rclcpp::ServicesQoS(), get_reference_cb_group_
                );
            rebase_reference_stream_client_ =
                node->template create_client<iii_drone_interfaces::srv::RebaseReferenceStream>(
                    "/control/maneuver_controller/rebase_reference_stream",
                    rclcpp::ServicesQoS(), get_reference_cb_group_
                );
            commit_reference_stream_client_ =
                node->template create_client<iii_drone_interfaces::srv::CommitReferenceStream>(
                    "/control/maneuver_controller/commit_reference_stream",
                    rclcpp::ServicesQoS(), get_reference_cb_group_
                );
            terminal_hold_transfer_client_ =
                node->template create_client<iii_drone_interfaces::srv::TerminalHoldTransfer>(
                    "/control/maneuver_controller/terminal_hold_transfer",
                    rclcpp::ServicesQoS(), get_reference_cb_group_
                );

            reference_mode_publisher_ = node->template create_publisher<iii_drone_interfaces::msg::StringStamped>(
                "maneuver_reference_client/reference_mode",
                10
            );

            create_wall_timer_ = [node](
                std::chrono::milliseconds period,
                std::function<void()> callback
            ) -> rclcpp::TimerBase::SharedPtr {
                std::function<void()> traced_callback =
                    [callback = std::move(callback), node]() {
                        const auto callback_start = std::chrono::steady_clock::now();
                        auto callback_entry = iii_drone::diagnostics::HilTrace::event(
                            "callback_group_callback_entry");
                        callback_entry.text("callback", "maneuver_reference_recovery_timer");
                        callback_entry.text("callback_group", "mission_executor_default_mutually_exclusive");
                        callback_entry.text("callback_group_type", "MutuallyExclusive");
                        callback_entry.text("node", node->get_fully_qualified_name());
                        callback_entry.commit();
                        callback();
                        const auto callback_end = std::chrono::steady_clock::now();
                        auto callback_exit = iii_drone::diagnostics::HilTrace::event(
                            "callback_group_callback_exit");
                        callback_exit.text("callback", "maneuver_reference_recovery_timer");
                        callback_exit.text("callback_group", "mission_executor_default_mutually_exclusive");
                        callback_exit.text("callback_group_type", "MutuallyExclusive");
                        callback_exit.text("node", node->get_fully_qualified_name());
                        callback_exit.number(
                            "duration_ns",
                            static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                callback_end - callback_start).count()));
                        callback_exit.commit();
                    };
                return node->create_wall_timer(period, std::move(traced_callback));
            };

            resetReferenceSafety();
        }

        /**
         * @brief Update the reference to the current state if the reference mode is hover or passthrough, otherwise do nothing.
         * 
         * @param force If true, the reference will be updated regardless of the reference mode.
         */
        void UpdateReference(bool force = false);

        /**
         * @brief Sets the reference if the reference mode is hover or passthrough, otherwise do nothing.
         * 
         * @param reference The reference.
         */
        void SetReference(iii_drone::control::Reference reference);

        /**
         * @brief Sets the reference mode to passthrough, only if the current reference mode is not maneuver.
         */
        void SetReferenceModePassthrough();

        /**
         * @brief Sets the reference mode to hover, only if the current reference mode is not maneuver.
         */
        void SetReferenceModeHover(bool force = false);

        /** Begin a new external mode with fresh hover/reference state. */
        uint64_t AcquireReferenceControl();

        enum class TerminalHoldAdoption { NoOffer, Adopted, Failed };
        /** Claim an exact live Core terminal generation before first setpoint. */
        TerminalHoldAdoption TryAdoptTerminalHold(int timeout_ms = 100);
        /** Retain a completed opt-in maneuver stream instead of its nominal result. */
        enum class TerminalHoldRetention { NoOffer, Retained, Failed };
        TerminalHoldRetention RetainCompletedTerminalHold(
            const std::string & request_identity, int timeout_ms = 1500
        );
        bool terminalHoldContinuityRequired() const;
        void ResetTerminalRetentionFailure();
        bool ReportTerminalRetentionFailure(const std::string & request_identity);
        bool TerminalRetentionFailed();

        /** Retire reference state only if this external mode still owns it. */
        bool ReleaseReferenceControl(uint64_t owner_generation);

        /**
         * @brief Starts a maneuver. References will be consumed from the reference topic.
         * 
         * @return true If the maneuver is started.
         */
        bool StartManeuver();

        /**
         * @brief Keep the current maneuver reference active while explicitly allowing
         * the next blended action to transfer ownership to one successor generation.
         *
         * @return true if a running maneuver stream can accept the handoff.
         */
        bool PrepareManeuverStreamHandoff();

        /**
         * @brief Authorize one successor before its action goal is sent.
         *
         * The authorization is confirmed by the action goal response or revoked
         * if that request fails before a successor is consumed. A blended
         * successor may retain a genuinely running predecessor on that
         * pre-successor cancellation; ordinary successors use safe stop.
         */
        bool BeginManeuverGoalHandoff(
            const std::string & request_identity,
            bool preserve_active_predecessor_on_cancel = false
        );

        /**
         * @brief Adopt the one authorized successor after its action goal is accepted.
         *
         * @return true if a pending goal handoff was confirmed.
         */
        bool ConfirmManeuverGoalHandoff(const std::string & request_identity);

        /**
         * @brief Retire a pending handoff or its confirmed active stream.
         *
         * A stale producer must not revoke a newer operation's successor.
         * @return true only when the matching owner was retired.
         */
        bool CancelManeuverGoalHandoff(const std::string & request_identity);

        /**
         * @brief Complete the matching goal using its final target reference.
         *
         * The identity check and reference transition are atomic with respect
         * to another handoff. A stale goal cannot stop its successor.
         */
        bool CompleteManeuverGoalHandoff(
            const std::string & request_identity,
            iii_drone::control::Reference final_reference
        );

        /** Complete a no-reference result, retaining only an exact fresh applied object stream. */
        bool CompleteManeuverGoalHandoff(const std::string & request_identity);

        /** Schedule terminal cleanup only while this request still owns the handoff. */
        bool StopManeuverGoalHandoffAfterTimeout(
            const std::string & request_identity, int timeout_ms
        );
        bool StopManeuverGoalHandoffAfterTimeout(
            const std::string & request_identity,
            iii_drone::control::Reference final_reference,
            int timeout_ms
        );

        /**
         * @brief Returns true while the client is actively consuming maneuver references.
         */
        bool IsManeuverActive();

        /**
         * @brief Stops a maneuver. The reference will be reset to the current state and the reference mode will be set to hover.
         */
        void StopManeuver();

        /**
         * @brief Stops a maneuver. The reference will be set to the given reference and the reference mode will be set to hover.
         * 
         * @param reference The reference.
         */
        void StopManeuver(iii_drone::control::Reference reference);

        /**
         * @brief Stops a maneuver after a timeout if a maneuver is not started again.
         * The reference will be reset to the current state and the reference mode will be set to hover.
         * 
         * @param timeout_ms The timeout in milliseconds.
         */
        void StopManeuverAfterTimeout(int timeout_ms);

        /**
         * @brief Stops a maneuver after a timeout if a maneuver is not started again.
         * The reference will be set to the given reference and the reference mode will be set to hover.
         * 
         * @param reference The reference.
         * @param timeout_ms The timeout in milliseconds.
         */
        void StopManeuverAfterTimeout(
            iii_drone::control::Reference reference, 
            int timeout_ms
        );

        /**
         * @brief Get the reference.
         * 
         * @param dt The time step since the last update.
         * @param on_fail_during_maneuver The callback to call when failing to acquire valid reference during a maneuver.
         * 
         * @return iii_drone::control::Reference The reference.
         */
        iii_drone::control::Reference GetReference(
            double dt,
            std::function<void()> on_fail_during_maneuver
        );

        /**
         * @brief Shared pointer type.
         */
        typedef std::shared_ptr<ManeuverReferenceClient> SharedPtr;

    private:
        /**
         * @brief Get reference callback group.
         */
        rclcpp::CallbackGroup::SharedPtr get_reference_cb_group_;

        /**
         * @brief Logger captured from the owning node.
         */
        rclcpp::Logger logger_;
        rclcpp::Clock::SharedPtr clock_;

        /**
         * @brief Timer factory captured from the owning node.
         */
        std::function<rclcpp::TimerBase::SharedPtr(std::chrono::milliseconds, std::function<void()>)> create_wall_timer_;

        /**
         * @brief Stop maneuver timer.
         */
        utils::Atomic<rclcpp::TimerBase::SharedPtr> stop_maneuver_timer_;

        /**
         * @brief Stop maneuver timer callback.
         */
        utils::Atomic<std::function<void()>> stop_maneuver_timer_callback_;

        // Distinguishes an already-dispatched timer callback from a newer stop.
        uint64_t stop_maneuver_timer_generation_ = 0;

        bool scheduleOwnedManeuverStop(
            const std::string & request_identity,
            std::optional<iii_drone::control::Reference> final_reference,
            int timeout_ms
        );

        /**
         * @brief Serializes maneuver-mode and delayed-stop transitions.
         *
         * Timer callbacks and the setpoint loop run concurrently with behavior
         * tree action handoffs. Recursive locking is required because a
         * delayed-stop callback completes through StopManeuver().
         */
        std::recursive_mutex transition_mutex_;

        // PX4 may activate a successor before deactivating its predecessor.
        // Keep mode ownership separate from individual maneuver/action epochs.
        uint64_t next_reference_control_generation_ = 0;
        uint64_t reference_control_owner_ = 0;
        uint64_t terminal_retention_failure_owner_ = 0;
        std::string terminal_retention_failure_request_;

        // Advanced when a new maneuver takes ownership. A failure callback may
        // retire its own goal and admit a successor before GetReference resumes.
        uint64_t maneuver_failure_epoch_ = 0;
        bool hoverIfFailureEpochUnchanged(uint64_t observed_epoch);
        std::string currentReferenceModeLabel() const;

        /**
         * @brief Stops the maneuver prematurely.
         */
        void stopManeuverPrematurely();

        /**
         * @brief Manuever start time.
         */
        utils::Atomic<rclcpp::Time> maneuver_start_time_;

        /**
         * @brief Shared pointer to the vehicle odometry adapter history.
         */
        iii_drone::utils::History<iii_drone::adapters::px4::VehicleOdometryAdapter>::SharedPtr vehicle_odometry_adapter_history_;

        /**
         * @brief The reference mode enum.
         */
        enum reference_mode_t {
            PASSTHROUGH,
            HOVER,
            WAIT_FOR_MANEUVER_START,
            MANEUVER,
            WAIT_FOR_MANEUVER_STOP,
            REFERENCE_LOSS_STOP
        };

        /**
         * @brief The reference mode.
         */
        iii_drone::utils::Atomic<reference_mode_t> reference_mode_;

        /**
         * @brief Returns whether the reference mode is either of the maneuver modes.
         * 
         * @return true If the reference mode is either of the maneuver modes.
         */
        bool isManeuverMode();

        /**
         * @brief Returns whether the reference mode is either of the maneuver modes.
         * 
         * @param reference_mode The reference mode.
         * 
         * @return true If the reference mode is either of the maneuver modes.
         */
        bool isManeuverMode(reference_mode_t reference_mode);

        /**
         * @brief Reference mode publisher.
         */
        rclcpp::Publisher<iii_drone_interfaces::msg::StringStamped>::SharedPtr reference_mode_publisher_;

        /**
         * @brief The reference.
         */
        iii_drone::utils::Atomic<iii_drone::control::Reference> reference_;

        /**
         * @brief True once the active maneuver has produced at least one valid reference.
         */
        iii_drone::utils::Atomic<bool> maneuver_reference_valid_;

        /**
         * @brief The reference mutex.
         */
        std::mutex reference_mutex_;

        /**
         * @brief The get reference service client.
         */
        rclcpp::Client<iii_drone_interfaces::srv::GetReference>::SharedPtr get_reference_client_;

        /**
         * @brief The single outstanding get-reference request.
         *
         * Reusing one request across update cycles prevents a temporarily slow
         * service from accumulating stale requests and delaying the response
         * needed to start the next maneuver.
         */
        std::optional<rclcpp::Client<iii_drone_interfaces::srv::GetReference>::FutureAndRequestId>
            pending_get_reference_request_;

        /**
         * @brief Serializes access to the outstanding get-reference request.
         */
        std::mutex get_reference_request_mutex_;

        rclcpp::Subscription<iii_drone_interfaces::msg::ManeuverReferenceStream>::SharedPtr
            reference_stream_subscription_;
        rclcpp::Publisher<iii_drone_interfaces::msg::ManeuverReferenceAck>::SharedPtr
            reference_ack_publisher_;
        rclcpp::Client<iii_drone_interfaces::srv::PauseReferenceStream>::SharedPtr
            pause_reference_stream_client_;
        rclcpp::Client<iii_drone_interfaces::srv::RebaseReferenceStream>::SharedPtr
            rebase_reference_stream_client_;
        rclcpp::Client<iii_drone_interfaces::srv::CommitReferenceStream>::SharedPtr
            commit_reference_stream_client_;
        rclcpp::Client<iii_drone_interfaces::srv::TerminalHoldTransfer>::SharedPtr
            terminal_hold_transfer_client_;
        std::shared_ptr<iii_drone_interfaces::srv::TerminalHoldTransfer::Response>
        requestTerminalHoldTransfer(
            const iii_drone_interfaces::srv::TerminalHoldTransfer::Request & request,
            int timeout_ms
        );

        std::optional<iii_drone_interfaces::msg::ManeuverReferenceStream>
            latest_stream_message_;
        std::chrono::steady_clock::time_point latest_stream_received_at_;
        std::mutex reference_stream_mutex_;
        std::string active_stream_id_;
        std::string active_request_identity_;
        std::string candidate_request_identity_;
        bool candidate_object_tracking_active_ = false;
        uint8_t candidate_stream_state_ = 0;
        std::string terminal_consumer_identity_;
        std::optional<uint8_t> currentTerminalStreamState();
        bool currentAppliedObjectTrackingStream(const std::string & request_identity);
        std::string applied_object_tracking_stream_id_;
        std::string applied_object_tracking_request_identity_;
        uint64_t applied_object_tracking_sequence_ = 0;
        std::chrono::steady_clock::time_point applied_object_tracking_at_{};
        uint8_t applied_object_tracking_state_ = 0;
        std::string applied_terminal_stream_id_;
        std::string applied_terminal_request_identity_;
        std::optional<uint8_t> applied_terminal_stream_state_;
        std::atomic_bool terminal_hold_continuity_required_{false};
        bool terminal_degraded_hold_ = false;
        bool object_stop_failure_hold_ = false;
        uint64_t last_applied_sequence_ = 0;
        uint64_t candidate_sequence_ = 0;
        ManeuverReferenceStreamGuard reference_stream_guard_;
        enum class StreamReadResult {
            Unavailable,
            FreshHeld,
            PredecessorActive,
            NewActive,
            Prepared,
            Paused,
        };

        ManeuverReferenceStartupPolicy startup_reference_policy_;

        struct ReferenceStreamIdentity {
            std::string stream_id;
            std::string request_identity;
            uint64_t last_applied_sequence = 0;

            bool valid() const {
                return !stream_id.empty() &&
                    isValidManeuverRequestIdentity(request_identity);
            }
        };

        struct ObjectStop {
            ReferenceStreamIdentity identity;
            std::chrono::steady_clock::time_point requested_at;
            std::optional<std::chrono::steady_clock::time_point> completion_deadline;
            bool admitted = false;
            bool failure_reported = false;
            std::string failure_reason;
        };
        std::optional<ObjectStop> object_stop_;
        struct ObjectStoppedHold {
            std::string stream_id;
            std::string request_identity;
            uint64_t control_owner_generation = 0;
            iii_drone::control::Reference anchor;
            std::chrono::steady_clock::time_point last_applied_at{};
            bool failure_reported = false;
        };
        std::optional<ObjectStoppedHold> object_stopped_hold_;
        bool ownsObjectStoppedHold() const;
        bool beginObjectStopLocked(const std::string & request_identity);
        bool appliedObjectStopRestLocked(const iii_drone::control::Reference & reference) const;
        void finishObjectStopLocked(const iii_drone::control::Reference & rest);

        struct ReferenceLossStopStart {
            iii_drone::control::Reference reference;
            std::optional<ReferenceStreamIdentity> pause_identity;
        };

        struct ReferenceConsumption {
            StreamReadResult stream_result = StreamReadResult::Unavailable;
            bool accepted = false;
            bool began_reference_loss_stop = false;
            bool terminal_degraded = false;
            bool object_unrecoverable = false;
            std::optional<ReferenceStreamIdentity> applied_ack_identity;
            std::optional<ReferenceStreamIdentity> pause_identity;
        };

        struct PendingManeuverGoalHandoff {
            std::string request_identity;
            std::string predecessor_stream_id;
            bool predecessor_was_running = false;
            bool preserve_active_predecessor_on_cancel = false;
            bool goal_accepted = false;
            bool successor_consumed = false;
            std::string successor_stream_id;
        };

        // An active-stream BT goal owns exactly one successor allowance from
        // dispatch until G2 commits or the goal is cancelled. While pending,
        // G1 remains a consumable predecessor but cannot satisfy the new
        // goal's WAIT_FOR_MANEUVER_START transition.
        std::optional<PendingManeuverGoalHandoff> pending_goal_handoff_;

        // Retained from the rejecting generation before recovery pauses or
        // rebases it.  Never reconstruct this identity from later samples.
        std::optional<ReferenceStreamIdentity> fault_stream_identity_;

        enum class RecoveryPhase {
            None,
            Stopping,
            RebaseRequested,
            WaitPrepared,
            CommitRequested,
            WaitActive,
        };
        RecoveryPhase recovery_phase_ = RecoveryPhase::None;
        std::string prepared_stream_id_;
        std::optional<iii_drone::control::Reference> prepared_reference_anchor_;
        std::chrono::steady_clock::time_point recovery_phase_started_;
        std::optional<rclcpp::Client<iii_drone_interfaces::srv::RebaseReferenceStream>::FutureAndRequestId>
            pending_rebase_request_;
        std::optional<rclcpp::Client<iii_drone_interfaces::srv::CommitReferenceStream>::FutureAndRequestId>
            pending_commit_request_;

        void receiveReferenceStream(
            const iii_drone_interfaces::msg::ManeuverReferenceStream::SharedPtr message
        );
        StreamReadResult readReferenceStream(Reference & reference);
        // Caller holds transition_mutex_. This ownership check is shared by
        // subscription ingress and cache consumption so a retired request
        // cannot gain authority merely by arriving before cancellation.
        bool ownsReferenceStreamLocked(
            const iii_drone_interfaces::msg::ManeuverReferenceStream & message
        );
        void retireInadmissibleCachedStreamLocked();
        ReferenceConsumption consumeReferenceCandidate(
            Reference & reference,
            reference_mode_t expected_mode,
            bool mark_maneuver_reference_valid,
            bool transition_to_maneuver
        );
        void publishReferenceAck(uint8_t status, const std::string & detail);
        void publishReferenceAckForStream(
            const std::string & stream_id,
            uint64_t last_applied_sequence,
            uint8_t status,
            const std::string & detail
        );
        void requestProducerPause(
            const ReferenceStreamIdentity & identity,
            const std::string & reason
        );
        bool advanceReferenceRecovery(
            Reference & reference,
            std::function<void()> on_fail_during_maneuver,
            std::string & reference_mode
        );
        void resetReferenceStreamState();

        /**
         * @brief Consecutive failed reference acquisitions for this client.
         */
        int failed_attempts_ = 0;

        /**
         * @brief Guards maneuver reference delivery and continuity.
         */
        std::optional<ManeuverReferenceSafetyGuard> reference_safety_guard_;

        /**
         * @brief Client-local stop used after reference delivery becomes unsafe.
         */
        std::optional<iii_drone::control::KinematicStopTrajectory>
            reference_loss_stop_trajectory_;
        std::chrono::steady_clock::time_point reference_loss_stop_start_time_;
        std::optional<std::chrono::steady_clock::time_point>
            reference_loss_below_threshold_since_;
        bool reference_loss_failure_reported_ = false;
        std::string reference_loss_reason_;
        std::mutex reference_safety_mutex_;

        ManeuverReferenceSafetyConfig referenceSafetyConfig() const;
        iii_drone::control::ControlledCancellationConfig referenceLossStopConfig() const;
        void resetReferenceSafety();
        ReferenceLossStopStart beginReferenceLossStop(
            const ManeuverReferenceSafetyEvaluation & evaluation
        );
        ReferenceLossStopStart beginReferenceLossStopLocked(
            const ManeuverReferenceSafetyEvaluation & evaluation
        );
        iii_drone::control::Reference sampleReferenceLossStop(
            std::function<void()> on_fail_during_maneuver,
            std::string & reference_mode
        );

        /**
         * @brief Removes any outstanding request when maneuver ownership changes.
         */
        void clearPendingReferenceRequest();

        /**
         * @brief Sends a request to the get reference service.
         * 
         * @param reference The reference output.
         * @param timeout_ms Maximum time to wait for the service response.
         * 
         * @return true If the request was successful.
         */
        bool getReferenceFromServer(Reference & reference, int timeout_ms);

        /**
         * @brief Returns a get-reference timeout bounded by the PX4 mode update budget.
         *
         * The configured timeout is an upper bound. In the mode update path a long
         * blocking wait can make PX4 consider the mode unresponsive, so the effective
         * timeout is capped to a fraction of the current update period.
         *
         * @param dt_s Time since the last mode update in seconds.
         *
         * @return Effective timeout in milliseconds.
         */
        int boundedGetReferenceTimeoutMs(double dt_s);

        /**
         * @brief Read-only live configuration view
         */
        iii_drone::configuration::Configuration::SharedPtr configuration_;

    };

} // namespace maneuver
} // namespace control
} // namespace iii_drone
