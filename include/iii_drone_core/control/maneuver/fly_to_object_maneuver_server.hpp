#pragma once

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
// III-Drone-Core:

#include <iii_drone_core/utils/types.hpp>
#include <iii_drone_core/utils/math.hpp>

#include <iii_drone_core/configuration/parameter_bundle.hpp>

#include <iii_drone_core/control/combined_drone_awareness_handler.hpp>

#include <iii_drone_core/control/maneuver/maneuver_server.hpp>
#include <iii_drone_core/control/maneuver/maneuver.hpp>
#include <iii_drone_core/control/maneuver/maneuver_types.hpp>

#include <iii_drone_core/control/maneuver/hover_by_object_maneuver_server.hpp>

#include <iii_drone_core/control/trajectory_generator_client.hpp>

#include <iii_drone_core/adapters/state_adapter.hpp>
#include <iii_drone_core/adapters/reference_trajectory_adapter.hpp>

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
            rclcpp::Node * node,
            CombinedDroneAwarenessHandler::SharedPtr combined_drone_awareness_handler,
            const std::string & action_name,
            unsigned int wait_for_execute_poll_ms,
            unsigned int evaluate_done_poll_ms,
            iii_drone::configuration::ParameterBundle::SharedPtr fly_to_object_maneuver_server_parameters,
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

        /**
         * @brief Whether the maneuver can be canceled, always returns true.
         * 
         * @return bool Whether the maneuver can be canceled.
         */
        bool canCancel() override;

        /**
         * @brief Compute the reference.
         * 
         * @param state The current state.
         * 
         * @return The reference.
         */
        iii_drone::control::Reference computeReference(const iii_drone::control::State & state) override;

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
        iii_drone::configuration::ParameterBundle::SharedPtr parameters_;

        /**
         * @brief The trajectory generator client shared pointer.
         */
        iii_drone::control::TrajectoryGeneratorClient::SharedPtr trajectory_generator_client_;

        /**
         * @brief The hover reference target adapter.
         */
        iii_drone::utils::Atomic<iii_drone::adapters::TargetAdapter> target_adapter_;

        /**
         * @brief Flag for first iteration.
         */
        iii_drone::utils::Atomic<bool> first_iteration_ = true;

        /**
         * @brief Has failed flag.
         */
        iii_drone::utils::Atomic<bool> has_failed_ = false;

        /**
         * @brief Get updated target reference. 
         * Sets the has_failed flag if the target is not visible.
         * 
         * @return Updated target reference.
         */
        iii_drone::control::Reference getUpdatedTargetReference(const iii_drone::control::State & state);

        /**
         * @brief Validates the drone awareness and maneuver parameters.
         * 
         * @param drone_awareness The drone awareness.
         * @param maneuever_params The maneuver parameters.
         * 
         * @return bool Whether the drone awareness and maneuver parameters are valid.
         */
        bool validateAwarenessAndParameters(
            const combined_drone_awareness_t & drone_awareness,
            const fly_to_object_maneuver_params_t & maneuver_params
        ) const;

    };

} // namespace maneuver
} // namespace control
} // namespace iii_drone