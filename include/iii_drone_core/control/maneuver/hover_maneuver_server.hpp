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

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/action/hover.hpp>

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
     * @brief Class for serving hovering. When the hover action is called,
     * the drone will start hovering, and the action
     * will succeed after the given duration. Exposes external methods for getting the reference
     * and updating the hover reference. Other maneuver servers will
     * use this server to generate references before a new maneuver is scheduled.
     */
    class HoverManeuverServer : public ManeuverServer {
        using Hover = iii_drone_interfaces::action::Hover;
    public:
        /**
         * @brief Constructor.
         * 
         * @param node Node pointer
         * @param combined_drone_awareness_handler Combined drone awareness handler pointer
         * @param action_name Action name
         * @param wait_for_execute_poll_ms Wait for execute poll milliseconds
         * @param evaluate_done_poll_ms Evaluate done poll milliseconds
         * @param use_nans Whether to use NaNs for velocity and acceleration, otherwise uses zeros, default true.
         */
        HoverManeuverServer(
            rclcpp_lifecycle::LifecycleNode * node,
            CombinedDroneAwarenessHandler::SharedPtr combined_drone_awareness_handler,
            const std::string & action_name,
            unsigned int wait_for_execute_poll_ms,
            unsigned int evaluate_done_poll_ms,
            bool use_nans=true
        );

        /**
         * @brief Whether the maneuver can be executed, will return true if the drone is armed and offboard.
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
         * @brief Expected awareness after execution.
         * 
         * @param maneuver The maneuver.
         * 
         * @return The expected awareness after execution.
         */
        combined_drone_awareness_t ExpectedAwarenessAfterExecution(const Maneuver & maneuver) override;

       /**
         * @brief Updates the hover reference.
         * 
         * @param state The current state.
         * 
         * @return void
         */
        void Update(const iii_drone::control::State &state);

       /**
         * @brief Updates the hover reference.
         * 
         * @param reference The hover reference.
         * 
         * @return void
         */
        void Update(const iii_drone::control::Reference &reference);

        /**
         * @brief Gets the hover reference.
         * 
         * @param state The current state, is not used.
         * 
         * @return The hover reference.
         */
        iii_drone::control::Reference GetReference(const iii_drone::control::State &state);

    private:
        /**
         * @brief The maneuver type (MANEUVER_TYPE_HOVER)
         * 
         * @return The maneuver type.
         */
        maneuver_type_t maneuver_type() const override;

        /**
         * @brief Starts the execution of the maneuver. Registers the current drone state as the hover reference.
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

        /**
         * @brief Compute the hover reference.
         * 
         * @param state The current state.
         * 
         * @return The hover reference.
         */
        iii_drone::control::Reference computeReference(const iii_drone::control::State & state) override;

        /**
         * @brief Whether the maneuver has succeeded, always returns true.
         * 
         * @param maneuver The maneuver.
         * 
         * @return bool Whether the maneuver has succeeded.
         */
        bool hasSucceeded(Maneuver & maneuver) override;

        /**
         * @brief Whether the maneuver has failed, always returns false.
         * 
         * @param maneuver The maneuver.
         * 
         * @return bool Whether the maneuver has failed.
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
         * @brief Registers the hover reference callback on success.
         * 
         * @param maneuver The maneuver.
         * 
         * @return void
         */
        void registerReferenceCallbackOnSuccess(const Maneuver & maneuver) override;

        /**
         * @brief The hover reference.
         */
        iii_drone::utils::Atomic<Reference> hover_reference_;

        /**
         * @brief Whether to use NaNs for velocity and acceleration, otherwise uses zeros.
         */
        bool use_nans_;

        /**
         * @brief The hover duration.
         */
        iii_drone::utils::Atomic<float> hover_duration_s_;

        /**
         * @brief Sustain action flag.
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