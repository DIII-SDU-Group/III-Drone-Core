#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <mutex>
#include <optional>

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

#include <iii_drone_core/control/maneuver/hover_maneuver_server.hpp>

#include <iii_drone_core/control/trajectory_generator_client.hpp>

#include <iii_drone_core/adapters/state_adapter.hpp>
#include <iii_drone_core/adapters/reference_trajectory_adapter.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/action/fly_to_position.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace control {
namespace maneuver {

    /**
     * @brief Class for serving flying to a position.     
     */
    class FlyToPositionManeuverServer : public ManeuverServer {
        using FlyToPosition = iii_drone_interfaces::action::FlyToPosition;
        using GoalHandleFlyToPosition = rclcpp_action::ServerGoalHandle<FlyToPosition>;
    public:
        /**
         * @brief Constructor.
         * 
         * @param node Node pointer
         * @param combined_drone_awareness_handler Combined drone awareness handler pointer
         * @param action_name Action name
         * @param wait_for_execute_poll_ms Wait for execute poll milliseconds
         * @param evaluate_done_poll_ms Evaluate done poll milliseconds
         * @param fly_to_position_maneuver_server_parameters Fly to position maneuver server parameters
         * @param trajectory_generator_client Trajectory generator client shared pointer
         */
        FlyToPositionManeuverServer(
            rclcpp_lifecycle::LifecycleNode * node,
            CombinedDroneAwarenessHandler::SharedPtr combined_drone_awareness_handler,
            const std::string & action_name,
            unsigned int wait_for_execute_poll_ms,
            unsigned int evaluate_done_poll_ms,
            iii_drone::configuration::Configuration::SharedPtr fly_to_position_maneuver_server_parameters,
            iii_drone::control::TrajectoryGeneratorClient::SharedPtr trajectory_generator_client
        );

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
         * @brief The maneuver type (MANEUVER_TYPE_FLY_TO_POSITION)
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

        /**
         * @brief Whether the maneuver can be canceled, always returns true.
         * 
         * @return bool Whether the maneuver can be canceled.
         */
        bool canCancel() override;

        std::optional<ControlledCancellationConfig> controlledCancellationConfig() const override;
        bool rebaseExecution(const State & stopped_state, std::string & reason) override;

        /**
         * @brief Compute the reference.
         * 
         * @param state The current state.
         * 
         * @return The reference.
         */
        iii_drone::control::Reference computeReference(const iii_drone::control::State & state) override;

        /**
         * @brief Whether the maneuver has succeeded, returns true if the drone is within the position and yaw tolerances.
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
         * @brief Registers the hover reference callback on success.
         * 
         * @param maneuver The maneuver.
         * 
         * @return void
         */
        void registerReferenceCallbackOnSuccess(const Maneuver & maneuver) override;

        /**
         * @brief The fly to position maneuver server parameters shared pointer.
         */
        iii_drone::configuration::Configuration::SharedPtr configuration_;

        /**
         * @brief The trajectory generator client shared pointer.
         */
        iii_drone::control::TrajectoryGeneratorClient::SharedPtr trajectory_generator_client_;

        /**
         * @brief Serializes maneuver initialization with reference generation.
         *
         * A retained blended callback may still be executing when the next
         * goal starts. Without this guard, that stale call can consume the new
         * goal's first-iteration reset and leave the old trajectory active.
         */
        std::mutex reference_generation_mutex_;

        /**
         * @brief The target reference.
         */
        iii_drone::utils::Atomic<iii_drone::control::Reference> target_reference_;

        /**
         * @brief Flag for first iteration.
         */
        iii_drone::utils::Atomic<bool> first_iteration_ = true;

        /**
         * @brief has failed flag
         */
        iii_drone::utils::Atomic<bool> has_failed_ = false;

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

        /**
         * @brief Latest reference streamed by this FTP server.
         */
        mutable std::mutex blend_mutex_;

        std::optional<iii_drone::control::Reference> latest_streamed_reference_;

        std::optional<iii_drone::control::Reference> pending_blend_start_reference_;

        rclcpp::Time pending_blend_start_time_;

        std::optional<iii_drone::control::Reference> initial_blend_start_reference_;

        std::optional<iii_drone::control::Reference> blend_completion_reference_;

        bool active_blend_to_next_ = false;

        double active_completion_position_tolerance_m_ = 0.0;

        /**
         * @brief True when the latest streamed interpolation reference is the
         * final target reference.
         */
        bool interpolationFinalReferenceStreamed(
            const iii_drone::control::Reference & target_reference
        ) const;

        bool consumePendingBlendStartReference(iii_drone::control::Reference & start_reference);

        iii_drone::control::Reference latestStreamedReferenceOrState(const iii_drone::control::State & state) const;

        void storeLatestStreamedReference(const iii_drone::control::Reference & reference);

        void prepareBlendCompletionReference(const iii_drone::control::State & state);

        iii_drone::control::Reference successReferenceForResult() const;

        iii_drone::control::State stateFromReference(const iii_drone::control::Reference & reference) const;

        /**
         * @brief Validates the maneuver parameters.
         * 
         * @param maneuver_params The maneuver parameters.
         * 
         * @return bool Whether the maneuver parameters are valid.
         */
        bool validateManeuverParameters(const fly_to_position_maneuver_params_t & maneuver_params) const;

    };

} // namespace maneuver
} // namespace control
} // namespace iii_drone
