#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

/*****************************************************************************/
// Std:

#include <memory>

/*****************************************************************************/
// III-Drone-Configuration:

#include <iii_drone_configuration/parameter_bundle.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/control/state.hpp>
#include <iii_drone_core/control/reference.hpp>

#include <iii_drone_core/control/combined_drone_awareness_handler.hpp>

#include <iii_drone_core/utils/types.hpp>
#include <iii_drone_core/utils/math.hpp>
#include <iii_drone_core/utils/atomic.hpp>

#include <iii_drone_core/adapters/target_adapter.hpp>
#include <iii_drone_core/adapters/combined_drone_awareness_adapter.hpp>

#include <iii_drone_core/control/maneuver/maneuver_server.hpp>
#include <iii_drone_core/control/maneuver/maneuver.hpp>
#include <iii_drone_core/control/maneuver/maneuver_types.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/action/hover_on_cable.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {

namespace control {

namespace maneuver {

    /**
     * @brief Class for serving hovering on a cable. When the hover on cable action is called,
     * the drone will start hovering on the cable. Hovering will continue for the given duration.
     * Exposes external methods for getting the reference
     * and updating the hover reference parameters. Other maneuver server will
     * use this server to generate references before a new maneuver is scheduled.
     * Will fail if the drone is not on a cable.
     */
    class HoverOnCableManeuverServer : public ManeuverServer {
        using HoverOnCable = iii_drone_interfaces::action::HoverOnCable;
    public:
        /**
         * @brief Constructor.
         * 
         * @param node Node pointer
         * @param combined_drone_awareness_handler Combined drone awareness handler pointer
         * @param action_name Action name
         * @param wait_for_execute_poll_ms Wait for execute poll milliseconds
         * @param evaluate_done_poll_ms Evaluate done poll milliseconds
         * @param parameters The hover on cable parameter bundle
         */
        HoverOnCableManeuverServer(
            rclcpp_lifecycle::LifecycleNode * node,
            CombinedDroneAwarenessHandler::SharedPtr combined_drone_awareness_handler,
            const std::string & action_name,
            unsigned int wait_for_execute_poll_ms,
            unsigned int evaluate_done_poll_ms,
            iii_drone::configuration::ParameterBundle::SharedPtr parameters
        );

        /**
         * @brief Whether the maneuver can be executed, will return false if the drone is not on the cable.
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
         * @brief The expected combined drone awareness after successful execution of the maneuver.
         * 
         * @param maneuver Maneuver
         * 
         * @return iii_drone::adapters::CombinedDroneAwarenessAdapter The expected combined drone awareness
         */
        iii_drone::adapters::CombinedDroneAwarenessAdapter ExpectedAwarenessAfterExecution(const Maneuver & maneuver);

        /**
         * @brief Register a callback for when the target is not visible or the drone has drifted too far away from the target relative pose.
         * 
         * @param on_fail_callback The on fail callback, called when the drone is not on the cable.
         */
        void RegisterOnFailCallback(std::function<void()> on_fail_callback);

        /**
         * @brief Updates the hover on cable reference.
         * 
         * @param target_cable_id The target cable id.
         * @param target_z_velocity The target z velocity.
         * @param target_yaw_rate The target yaw rate.
         * 
         * @return bool Whether the update was successful.
         */
        bool Update(
            int target_cable_id,
            double target_z_velocity,
            double target_yaw_rate
        );

        /**
         * @brief Gets the hover on cable reference. Will call the on fail callback if the drone is not on the cable.
         * 
         * @param state The current state, is not used.
         * 
         * @return The hover on cable reference.
         */
        iii_drone::control::Reference GetReference(const iii_drone::control::State &state);

    private:
        /**
         * @brief The hover on cable parameter bundle.
         */
        iii_drone::configuration::ParameterBundle::SharedPtr parameters_;

        /**
         * @brief Get the maneuver type (MANEUVER_TYPE_HOVER_ON_CABLE).
         * 
         * @return maneuver_type_t The maneuver type.
        */
        maneuver_type_t maneuver_type() const override;

        /**
         * @brief Start the execution of the maneuver.
         * 
         * @param maneuver The maneuver.
         * 
         * @return void
         */
        void startExecution(Maneuver & maneuver) override;

        /**
         * @brief Whether the maneuver can be canceled, will always return false as the action succeeds immediately.
         * 
         * @return bool Whether the maneuver can be canceled.
         */
        bool canCancel() override;

        /**
         * @brief Compute the hover on cable reference. Will call the on fail callback if the drone is not on the cable.
         * 
         * @param state The current state.
         * 
         * @return iii_drone::control::Reference The hover on cable reference.
         */
        iii_drone::control::Reference computeReference(const iii_drone::control::State & state) override;

        /**
         * @brief Whether the maneuver has succeeded, will always return true as the action succeeds immediately.
         * 
         * @param maneuver The maneuver.
         * 
         * @return bool Whether the maneuver has succeeded.
         */
        bool hasSucceeded(Maneuver & maneuver) override;

        /**
         * @brief Whether the maneuver has failed, will always return false as the action succeeds immediately.
         * 
         * @param maneuver The maneuver.
         * 
         * @return bool Whether the maneuver has failed.
         */
        bool hasFailed(Maneuver & maneuver) override;

        /**
         * @brief Publish the result and finalize the maneuver based on the maneuver result type.
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
         * @brief Register the hover on cable reference callback on success.
         * 
         * @param maneuver The maneuver.
         * 
         * @return void
         */
        void registerReferenceCallbackOnSuccess(const Maneuver & maneuver) override;

        /**
         * @brief The on fail callback, called when the drone is not on the cable.
         */
        std::function<void()> on_fail_callback_;

        /**
         * @brief Has on fail callback flag.
         */
        iii_drone::utils::Atomic<bool> has_on_fail_callback_;

        /**
         * @brief The target cable id.
         */
        int target_cable_id_;

        /**
         * @brief The target z velocity.
         */
        double target_z_velocity_;

        /**
         * @brief The target yaw rate.
         */
        double target_yaw_rate_;

        /**
         * @brief Validates that the drone is on the cable.
         * 
         * @param drone_awareness The drone awareness.
         * 
         * @return bool Whether the state is valid.
         */
        bool validateAwareness(iii_drone::adapters::CombinedDroneAwarenessAdapter drone_awareness) const;

        /**
         * @brief The hover duration.
         */
        iii_drone::utils::Atomic<float> hover_duration_s_;

        /**
         * @brief The sustain action flag.
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