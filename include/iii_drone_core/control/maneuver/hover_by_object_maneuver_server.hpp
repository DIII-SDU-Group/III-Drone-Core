#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/types.hpp>
#include <iii_drone_core/utils/atomic.hpp>

#include <iii_drone_core/control/combined_drone_awareness_handler.hpp>

#include <iii_drone_core/control/maneuver/maneuver_server.hpp>
#include <iii_drone_core/control/maneuver/maneuver.hpp>
#include <iii_drone_core/control/maneuver/maneuver_types.hpp>

#include <iii_drone_core/adapters/target_adapter.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/action/hover_by_object.hpp>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace control {
namespace maneuver {

    /**
     * @brief HoverByObject maneuver server class. When the hover by object action is called,
     * and the target is valid (object is visible and the drone is within the maximum euclidean distance to the object),
     * the drone will start hovering by the object. Hovering will continue for the given duration.
     * Exposes external methods for getting the reference
     * and updating the hover reference parameters. Other maneuver servers will
     * use this server to generate references before a new maneuver is scheduled.
     * Will fail if the target is not visible or the drone has drifted too far away from the target relative pose.
     */
    class HoverByObjectManeuverServer : public ManeuverServer {
        using HoverByObject = iii_drone_interfaces::action::HoverByObject;
    public:
        /**
         * @brief Constructor.
         * 
         * @param node Node pointer
         * @param combined_drone_awareness_handler Combined drone awareness handler pointer
         * @param action_name Action name
         * @param wait_for_execute_poll_ms Wait for execute poll milliseconds
         * @param evaluate_done_poll_ms Evaluate done poll milliseconds
         * @param use_nans Whether to use NaNs for velocity and acceleration, otherwise uses zeros
         * @param max_euc_dist The maximum euclidean distance to the object before the fail callback is called.
         */
        HoverByObjectManeuverServer(
            rclcpp_lifecycle::LifecycleNode * node,
            CombinedDroneAwarenessHandler::SharedPtr combined_drone_awareness_handler,
            const std::string & action_name,
            unsigned int wait_for_execute_poll_ms,
            unsigned int evaluate_done_poll_ms,
            bool use_nans,
            double max_euc_dist
        );

        /**
         * @brief Whether the maneuver can be executed, will return false if the target is not visible or the drone has drifted too far away from the target relative pose.
         * 
         * @param maneuver The maneuver.
         * @param drone_awareness The drone awareness.
         * 
         * @return bool Whether the maneuver can be executed.
         */
        bool CanExecuteManeuver(
            const Maneuver & maneuver,
            const combined_drone_awareness_t & drone_awareness
        ) const override;

        /**
         * @brief The expected awareness after execution. The drone will be armed and offboard.
         * 
         * @param maneuver The maneuver.
         * 
         * @return combined_drone_awareness_t The expected awareness after execution.
         */
        combined_drone_awareness_t ExpectedAwarenessAfterExecution(const Maneuver & maneuver) override;

        /**
         * @brief Register a callback for when the target is not visible or the drone has drifted too far away from the target relative pose.
         * 
         * @param on_fail_callback The on fail callback.
         */
        void RegisterOnFailCallback(std::function<void()> on_fail_callback);

        /**
         * @brief Updates the hover reference.
         * 
         * @param target_adapter The hover object target adapter.
         * 
         * @return bool Whether the update was successful.
         */
        bool Update(const iii_drone::adapters::TargetAdapter &target_adapter);

        /**
         * @brief Gets the hover reference. The current state is not used.
         * Will call the on fail callback if the target is not visible or the drone has drifted too far away from the target relative pose.
         * 
         * @param state The current state, is not used.
         * 
         * @return The hover reference.
         */
        iii_drone::control::Reference GetReference(const iii_drone::control::State &state);

    private:
        /**
         * @brief Returns the maneuver type (MANEUVER_TYPE_HOVER_BY_OBJECT).
         * 
         * @return maneuver_type_t The maneuver type.
         */
        maneuver_type_t maneuver_type() const override;

        /**
         * @brief Starts the execution of the maneuver. Derives the TargetAdapter from the maneuver object parameters
         * and sets the has_target flag to true.
         * 
         * @param maneuver The maneuver.
         * 
         * @return void
         */
        void startExecution(Maneuver & maneuver) override;

        /**
         * @brief Returns true as the maneuver can be canceled.
         * 
         * @return bool True.
         */
        bool canCancel() override;

        /**
         * @brief Computes the hover by object reference for the maneuver server.
         * 
         * @param state The current state.
         * 
         * @return iii_drone::control::Reference The hover reference.
         */
        iii_drone::control::Reference computeReference(const iii_drone::control::State & state) override;

        /**
         * @brief Returns true if the maneuver has succeeded. Has succeeded if the target is visible and the drone is within the maximum euclidean distance to the object.
         * 
         * @param maneuver The maneuver.
         * 
         * @return bool True if the maneuver has succeeded.
         */
        bool hasSucceeded(Maneuver & maneuver) override;

        /**
         * @brief Returns true if the maneuver has failed. Has failed if the target is not visible or the drone has drifted too far away from the target relative pose.
         * 
         * @param maneuver The maneuver.
         * 
         * @return bool True if the maneuver has failed.
         */
        bool hasFailed(Maneuver & maneuver) override;

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
         * @brief The hover reference target adapter.
         */
        iii_drone::utils::Atomic<iii_drone::adapters::TargetAdapter> target_adapter_;

        /**
         * @brief Has target flag.
         */
        iii_drone::utils::Atomic<bool> has_target_;

        /**
         * @brief The callback for when the target is not visible or the drone has drifted too far away from the target relative pose.
         */
        std::function<void()> on_fail_callback_;

        /**
         * @brief Has on fail callback flag.
         */
        iii_drone::utils::Atomic<bool> has_on_fail_callback_;

        /**
         * @brief Whether to use NaNs for velocity and acceleration, otherwise uses zeros.
         */
        bool use_nans_;

        /**
         * @brief The maximum euclidean distance to the object before the fail callback is called.
         */
        double max_euc_dist_;

       /**
         * @brief Validates that the target transform and the drone state are within the maximum euclidean distance.
         * 
         * @param target_transform The target transform.
         * @param state The drone state.
         * 
         * @return bool Whether the target transform and the drone state are within the maximum euclidean distance.
         */
        bool validateTargetTransform(
            const iii_drone::types::transform_matrix_t &target_transform, 
            const iii_drone::control::State &state
        ) const;

        /**
         * @brief Validates the drone awareness.
         * 
         * @param drone_awareness The drone awareness.
         * 
         * @return bool Whether the awareness is valid.
         */
        bool validateAwareness(combined_drone_awareness_t drone_awareness) const;

        /**
         * @brief The hover duration.
         */
        iii_drone::utils::Atomic<float> hover_duration_s_;

        /**
         * @brief Whether to sustain the action.
         */
        iii_drone::utils::Atomic<bool> sustain_action_;

        /**
         * @brief The start time of the hover maneuver.
         */
        iii_drone::utils::Atomic<rclcpp::Time> hover_start_time_;

    };

} // namespace maneuver
} // namespace control
} // namespace iii_drone