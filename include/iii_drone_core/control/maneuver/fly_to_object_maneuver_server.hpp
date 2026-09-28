#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>

/*****************************************************************************/
// III-Drone-Configuration:

#include <iii_drone_configuration/configuration.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/types.hpp>
#include <iii_drone_core/utils/math.hpp>

#include <iii_drone_core/control/combined_drone_awareness_handler.hpp>

#include <iii_drone_core/control/maneuver/maneuver_server.hpp>
#include <iii_drone_core/control/maneuver/maneuver.hpp>
#include <iii_drone_core/control/maneuver/maneuver_types.hpp>

#include <iii_drone_core/control/maneuver/hover_by_object_maneuver_server.hpp>
#include <iii_drone_core/control/maneuver/hover_maneuver_server.hpp>
#include <iii_drone_core/control/maneuver/object_tracking_session.hpp>
#include <iii_drone_core/control/maneuver/terminal_tracking_hold.hpp>

#include <iii_drone_core/control/trajectory_generator_client.hpp>

#include <iii_drone_core/adapters/state_adapter.hpp>
#include <iii_drone_core/adapters/reference_trajectory_adapter.hpp>
#include <iii_drone_core/adapters/combined_drone_awareness_adapter.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/action/fly_to_object.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace control {
namespace maneuver {

    /**
     * @brief Class for serving flying to an object.     
     */
    class FlyToObjectManeuverServer : public ManeuverServer {
        using FlyToObject = iii_drone_interfaces::action::FlyToObject;
        using GoalHandleFlyToObject = rclcpp_action::ServerGoalHandle<FlyToObject>;
    public:
        /**
         * @brief Constructor.
         * 
         * @param node Node pointer
         * @param combined_drone_awareness_handler Combined drone awareness handler pointer
         * @param action_name Action name
         * @param wait_for_execute_poll_ms Wait for execute poll milliseconds
         * @param evaluate_done_poll_ms Evaluate done poll milliseconds
         * @param fly_to_object_maneuver_server_parameters Fly to object maneuver server parameters
         * @param trajectory_generator_client Trajectory generator client shared pointer
         */
        FlyToObjectManeuverServer(
            rclcpp_lifecycle::LifecycleNode * node,
            CombinedDroneAwarenessHandler::SharedPtr combined_drone_awareness_handler,
            const std::string & action_name,
            unsigned int wait_for_execute_poll_ms,
            unsigned int evaluate_done_poll_ms,
            iii_drone::configuration::Configuration::SharedPtr fly_to_object_maneuver_server_parameters,
            iii_drone::control::TrajectoryGeneratorClient::SharedPtr trajectory_generator_client
        );

        // The first command in this goal's own stream establishes consumer
        // ownership; proximity alone does not prove that handoff occurred.
        void RegisterFirstReferenceAppliedCallback(
            std::function<bool(const std::string &)> callback);
        void RegisterAppliedRestReferenceCallback(
            std::function<bool(const std::string &, const Reference &)> callback);
        bool RetainsTrackedSource(const ReferenceCallbackBinding & source) const;
        bool TrackedSourceUnrecoverable(const ReferenceCallbackBinding & source) const;
        bool RequestTrackedTransitionStop(const ReferenceCallbackBinding & source);
        bool TrackedTransitionStopping(const ReferenceCallbackBinding & source) const;
        std::optional<Reference> TrackedTransitionRest(
            const ReferenceCallbackBinding & source) const;

        /**
         * @brief Whether the maneuver can be executed.
         * 
         * @param maneuver The maneuver.
         * @param drone_awareness The drone awareness.
         * 
         * @return bool Whether the maneuver can be executed.
         */
        bool CanExecuteManeuver(
            const Maneuver & maneuver,
            const iii_drone::adapters::CombinedDroneAwarenessAdapter & drone_awareness
        ) const override;

        /**
         * @brief Expected awareness after execution.
         * 
         * @param maneuver The maneuver.
         * 
         * @return The expected awareness after execution.
         */
        iii_drone::adapters::CombinedDroneAwarenessAdapter ExpectedAwarenessAfterExecution(const Maneuver & maneuver) override;

    private:
        /**
         * @brief The maneuver type (MANEUVER_TYPE_FLY_TO_OBJECT)
         * 
         * @return The maneuver type.
         */
        maneuver_type_t maneuver_type() const override;

        /**
         * @brief Starts the execution of the maneuver.
         * 
         * @param maneuver The maneuver.
         * 
         * @return void
         */
        void startExecution(Maneuver & maneuver) override;
        bool supportsObjectTracking() const;

        /**
         * @brief Whether the maneuver can be canceled, always returns true.
         * 
         * @return bool Whether the maneuver can be canceled.
         */
        bool canCancel() override;
        std::optional<ControlledCancellationConfig> controlledCancellationConfig() const override;
        bool controlledCancellationComplete(const ControlledCancellationConfig & config) override;
        bool controlledCancellationFailure() const override;
        bool validateControlledCancellationStop(
            const Reference & initial, const KinematicStopTrajectory & candidate,
            std::string & reason) override;

        /**
         * @brief Compute the reference.
         * 
         * @param state The current state.
         * 
         * @return The reference.
         */
        iii_drone::control::Reference computeReference(const iii_drone::control::State & state) override;
        iii_drone::control::Reference initializationReference(const iii_drone::control::State & state) const override;
        bool rebaseExecution(const State & stopped_state, std::string & reason) override;

        /**
         * @brief Whether the maneuver has succeeded, returns true if the drone is within the position tolerance.
         * 
         * @param maneuver The maneuver.
         * 
         * @return bool Whether the maneuver has succeeded.
         */
        bool hasSucceeded(Maneuver & maneuver) override;

        /**
         * @brief Whether the maneuver has failed.
         * 
         * @param maneuver The maneuver.
         * 
         * @return bool Whether the maneuver has failed.
         */
        bool hasFailed(Maneuver & maneuver) override;

        /**
         * @brief Get the feedback.
         * 
         * @param maneuver The maneuver.
         * 
         * @return The feedback.
         */
        std::shared_ptr<void> getFeedback(Maneuver & maneuver) override;

        /**
         * @brief Publishes the result and finalizes the maneuver according to the maneuver result type.
         * 
         * @param maneuver The maneuver.
         * @param maneuver_result_type The maneuver result type.
         * 
         * @return void
         */
        void publishResultAndFinalize(
            Maneuver & maneuver,
            maneuver_result_type_t maneuver_result_type
        ) override;

        /**
         * @brief Registers the hover by object reference callback on success.
         * 
         * @param maneuver The maneuver.
         * 
         * @return void
         */
        void registerReferenceCallbackOnSuccess(const Maneuver & maneuver) override;

        /**
         * @brief The fly to object maneuver server parameters shared pointer.
         */
        iii_drone::configuration::Configuration::SharedPtr configuration_;

        /**
         * @brief The trajectory generator client shared pointer.
         */
        iii_drone::control::TrajectoryGeneratorClient::SharedPtr trajectory_generator_client_;

        std::function<bool(const std::string &)> first_reference_applied_;
        std::function<bool(const std::string &, const Reference &)> applied_rest_reference_;

        /**
         * @brief The hover reference target adapter.
         */
        iii_drone::utils::Atomic<iii_drone::adapters::TargetAdapter> target_adapter_;

        /**
         * @brief Flag for first iteration.
         */
        iii_drone::utils::Atomic<bool> first_iteration_ = true;
        std::optional<iii_drone::control::Reference> terminal_start_reference_;
        std::shared_ptr<ObjectTrackingSession> object_tracking_session_;
        // Scheduler callbacks run independently of the action worker. They
        // read this synchronized snapshot instead of the worker-owned pointer.
        iii_drone::utils::Atomic<std::shared_ptr<ObjectTrackingSession>>
            published_object_tracking_session_;
        std::shared_ptr<TerminalTrackingHold> object_failure_hold_;
        bool object_hover_ready_ = false;

        /**
         * @brief Has failed flag.
         */
        iii_drone::utils::Atomic<bool> has_failed_ = false;

        /**
         * @brief Last target reference actually handed to the trajectory
         * generator after target-frame lookup, yaw normalization and optional
         * target-position filtering.
         */
        iii_drone::utils::Atomic<iii_drone::control::Reference> active_target_reference_;

        /**
         * @brief Whether active_target_reference_ has been initialized for the
         * current maneuver execution.
         */
        iii_drone::utils::Atomic<bool> active_target_reference_valid_ = false;

        // One coherent live observation for tracked arrival. The nominal and
        // filtered references belong to the same lookup and owner generation.
        // The reference callback writes it; the action worker reads it.
        struct NominalTargetObservation {
            iii_drone::control::Reference nominal;
            iii_drone::control::Reference filtered;
            std::string request_identity;
            uint64_t execution_id = 0;
            rclcpp::Time observed_at{0, 0, RCL_ROS_TIME};
            std::chrono::steady_clock::time_point received_at;
        };
        iii_drone::utils::Atomic<std::shared_ptr<const NominalTargetObservation>>
            nominal_target_observation_;

        /**
         * @brief Whether the target-position low-pass filter has been initialized.
         */
        bool target_position_filter_initialized_ = false;

        /**
         * @brief Filtered fly-to-object target position.
         */
        iii_drone::types::point_t filtered_target_position_ = iii_drone::types::point_t::Zero();

        /**
         * @brief Last time the target-position low-pass filter was updated.
         */
        rclcpp::Time last_target_position_filter_update_time_;

        /**
         * @brief Whether an MPC maneuver has entered its final interpolation
         * settle phase.
         */
        iii_drone::utils::Atomic<bool> mpc_settle_active_ = false;

        /**
         * @brief Whether the final interpolation settle trajectory needs a
         * reset on the next reference computation.
         */
        iii_drone::utils::Atomic<bool> mpc_settle_first_iteration_ = false;

        /**
         * @brief Frozen target streamed during the final post-MPC interpolation
         * settle phase.
         */
        iii_drone::utils::Atomic<iii_drone::control::Reference> mpc_settle_target_reference_;

        rclcpp::Time maneuver_start_time_;

        bool threshold_reached_logged_ = false;

        bool settle_threshold_reached_logged_ = false;

        bool final_reference_streamed_logged_ = false;

        bool success_timing_logged_ = false;

        /** Monotonic ROS-clock timestamp of the most recent valid target observation. */
        std::atomic<int64_t> last_target_observation_ns_{0};

        /**
         * @brief Get updated target reference. 
         * Sets the has_failed flag if the target is not visible.
         * 
         * @return Updated target reference.
         */
        iii_drone::control::Reference getUpdatedTargetReference(const iii_drone::control::State & state);

        /** Update the control filter and record its fresh nominal source together. */
        iii_drone::control::Reference updateLiveTargetReference(
            const iii_drone::control::State & state,
            const ReferenceCallbackBinding & binding);

        /** Record a successful live target observation for transient-loss handling. */
        void markTargetObserved();

        /** Whether the last valid target may still be used during a short sensor dropout. */
        bool targetLossWithinGrace() const;

        /**
         * @brief Apply a low-pass filter to the target position while preserving the raw target yaw.
         *
         * @param raw_reference The raw reference computed from perception.
         * @param state The current drone state.
         *
         * @return The reference with filtered position and raw yaw.
         */
        iii_drone::control::Reference filterTargetPositionReference(
            const iii_drone::control::Reference & raw_reference,
            const iii_drone::control::State & state
        );

        /**
         * @brief Clamp a computed object target to the configured minimum
         * target altitude above the current ground estimate.
         *
         * Fly-to-object targets are perception-derived and can move slightly
         * below the generic fly target altitude bound as cable estimates jitter.
         * The maneuver should keep the safety bound by clamping the target
         * reference instead of rejecting the goal after a valid cable target was
         * selected.
         */
        iii_drone::control::Reference enforceMinimumTargetAltitude(
            const iii_drone::control::Reference & reference
        ) const;

        /**
         * @brief True when the latest streamed interpolation reference is the
         * final target reference used by the active trajectory. MPC maneuvers
         * intentionally do not use this gate because their current reference is
         * not guaranteed to terminate exactly at the target.
         */
        bool interpolationFinalReferenceStreamed(
            const iii_drone::control::Reference & target_reference
        ) const;

        /**
         * @brief Validates the drone awareness and maneuver parameters.
         * 
         * @param drone_awareness The drone awareness.
         * @param maneuever_params The maneuver parameters.
         * 
         * @return bool Whether the drone awareness and maneuver parameters are valid.
         */
        bool validateAwarenessAndParameters(
            const iii_drone::adapters::CombinedDroneAwarenessAdapter & drone_awareness,
            const fly_to_object_maneuver_params_t & maneuver_params
        ) const;

    };

} // namespace maneuver
} // namespace control
} // namespace iii_drone
