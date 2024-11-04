#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <memory>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/action/fly_to_position.hpp>
#include <iii_drone_interfaces/action/fly_to_object.hpp>
#include <iii_drone_interfaces/action/cable_landing.hpp>
#include <iii_drone_interfaces/action/cable_takeoff.hpp>
#include <iii_drone_interfaces/action/hover.hpp>
#include <iii_drone_interfaces/action/hover_by_object.hpp>
#include <iii_drone_interfaces/action/hover_on_cable.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/token.hpp>
#include <iii_drone_core/utils/atomic.hpp>

#include <iii_drone_core/control/combined_drone_awareness_handler.hpp>
#include <iii_drone_core/control/reference.hpp>
#include <iii_drone_core/control/state.hpp>

#include <iii_drone_core/control/maneuver/maneuver.hpp>
#include <iii_drone_core/control/maneuver/maneuver_types.hpp>
#include <iii_drone_core/control/maneuver/reference_callback_token.hpp>

#include <iii_drone_core/adapters/combined_drone_awareness_adapter.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace control {
namespace maneuver {

    /**
     * @brief Flight maneuver server abstract base class. 
     * Follows a defined interface for handling by the maneuver scheduler.
     * Underlyingly initializes an action server and handles the execution of the actions.
     * Any deriving class must implement the following methods:
     * - CanExecuteManeuver: Checks if a maneuver can be executed given an awareness.
     * - maneuver_type: Returns the maneuver type of the specific maneuver server.
     * - startExecution: Called when starting execution of a maneuver.
     * - canCancel: Returns whether a registered maneuver can be canceled.
     * - computeReference: Computes the next reference given the current state of the drone.
     * - hasSucceeded: Returns whether the maneuver has successfully completed.
     * - hasFailed: Returns whether the maneuver has failed.
     * - publishFeedback: Publishes feedback for the maneuver.
     * - publishResultAndFinalize: Publishes the result and finalizes the maneuver.
     * - registerReferenceCallbackOnSuccess: Registers a callback function for the reference.
     * Additionally, createServer() must be called in the constructor of the derived class.
     * 
     * After creation, the maneuver server must be started using the Start method from the maneuver scheduler, 
     * supplying function pointers for registering and updating maneuvers, and a done callback function.
     * Upon receiving a goal, the maneuver server will register the goal with the maneuver scheduler,
     * which through the function pointer will tell if the maneuver can execute immediately or be queued. 
     * The action will be accepted, defered or rejected accordingly.
     * When the action is executing, the maneuver server will register the reference callback function with the maneuver scheduler,
     * and will execute the maneuver. The maneuver server will then periodically check if the maneuver has succeeded or failed,
     * and publish feedback accordingly. When the maneuver is done, the maneuver server will call the done callback function,
     * and the maneuver scheduler will take back the reference callback token.
     */
    class ManeuverServer {
    public:
        /**
         * @brief Constructor
         * 
         * @param node Node pointer
         * @param awareness_handler Shared pointer to the combined drone awareness handler
         * @param action_name Action name
         * @param wait_for_execute_poll_ms Wait for execute poll milliseconds, default 1000
         * @param evaluate_done_poll_ms Evaluate done poll milliseconds, default 100
         */
        ManeuverServer(
            rclcpp_lifecycle::LifecycleNode * node,
            CombinedDroneAwarenessHandler::SharedPtr awareness_handler,
            const std::string & action_name,
            unsigned int wait_for_execute_poll_ms,
            unsigned int evaluate_done_poll_ms
        );

        /**
         * @brief Method to start the maneuver server. The maneuver server will reject any new goals until started.
         * 
         * @param register_maneuver_function Function pointer for registering a maneuver. Must be supplied by the maneuver scheduler.
         * The function should take the maneuver to be registered, and a boolean reference which should be set to true if the maneuver can be executed instantly
         * and false if the maneuver can only be executed later. The function should return false if the maneuver can not be queued or executed, and true otherwise.
         * @param update_manuever_function Function pointer for updating a maneuver. Must be supplied by the maneuver scheduler.
         * The function takes a maneuver initiated with the goal handle, and should internally update the maneuver entry with the new goal handle.
         * This function will be executed at some time after the goal is accepted, and before the goal is executed.
         * Should return false if the maneuver can not be updated, in which case the goal will be aborted, and true otherwise.
         * @param cancel_maneuver_function Function pointer for canceling a maneuver. Must be supplied by the maneuver scheduler.
         * The function should take the maneuver to be canceled, and return true if the maneuver was canceled successfully, and false otherwise.
         * The maneuver object must be terminated using the Terminate() method before canceling with success=false.
         * @param verify_maneuver_in_queue_function Function pointer for verifying that the maneuver is in the queue, pending execution. Must be supplied by the maneuver scheduler.
         * The function should take the maneuver to be verified, and return true if the maneuver is in the queue or is the currently active maneuver, and false otherwise.
         * @param verify_maneuver_active_function Function pointer for verifying that the maneuver is currently active for execution. Must be supplied by the maneuver scheduler.
         * The function should take the maneuver to be verified, and return true if the maneuver is currently active, and false otherwise.
         * If the maneuver is not active, the action will be aborted. The queue integrity must be kept, as the maneuver will not return to the queue if it stops being active.
         * @param done_callback Function pointer for done callback. Must be supplied by the maneuver scheduler.
         * The function should take the maneuver. The maneuver object must be terminated using the Terminate() method.
         * Any cleanup or post-maneuver actions should be executed in this function.
         * The function must reacquire the reference callback token from the maneuver server 
         * and register an intermediate reference callback function with the token.
         * @param reference_callback_token Slave token shared pointer for the reference callback shared function pointer handle. 
         * This token will be acquired by the maneuver server upon beginning execution of a maneuver. The maneuver server will register
         * its own reference callback function with the token. The master is responsible for taking the token back when the maneuver is done or have failed.
         * @param registered_maneuvers The registered maneuver server map. Is used to register the reference callback of another maneuver server
         * upon the completion of the current maneuver, which is specific to the inheriting maneuver type.
         * 
         * @return void
         */
        void Start(
            std::function<bool(Maneuver, bool &)> register_maneuver_function,
            std::function<bool(Maneuver)> update_manuever_function,
            std::function<bool(Maneuver)> cancel_maneuver_function,
            std::function<bool(Maneuver)> verify_maneuver_in_queue_function,
            std::function<bool(Maneuver)> verify_maneuver_active_function,
            std::function<void(Maneuver)> done_callback,
            ReferenceCallbackToken::SharedPtr reference_callback_token,
            std::map<iii_drone::control::maneuver::maneuver_type_t, std::shared_ptr<ManeuverServer>> registered_maneuvers
        );

        /**
         * @brief Method to stop the maneuver server. The maneuver server will reject any new goals until started.
         * 
         * @return void
         */
        void Stop();
 
        /**
         * @brief Virtual function which checks if a maneuver can be executed given an awareness.
         * Must be implemented specific to the maneuver type.
         * Considers only the awareness, and not whether a maneuver is currently being executed.
         * 
         * @param maneuver Maneuver
         * @param awareness Combined drone awareness
         * 
         * @return bool True if the maneuver can be executed, false otherwise
         */
        virtual bool CanExecuteManeuver(
            const Maneuver & maneuver,
            const iii_drone::adapters::CombinedDroneAwarenessAdapter & awareness
        ) const = 0;

        /**
         * @brief Returns the expected combined drone awareness after successful execution of the maneuver.
         * Must be implemented specific to the maneuver type.
         * 
         * @param maneuver Maneuver
         * 
         * @return iii_drone::adapters::CombinedDroneAwarenessAdapter The expected combined drone awareness
         */
        virtual iii_drone::adapters::CombinedDroneAwarenessAdapter ExpectedAwarenessAfterExecution(const Maneuver & maneuver) = 0;

        /**
         * @brief Action name getter.
         * 
         * @return std::string The action name
         */
        std::string action_name() const;

        /**
         * @brief Returns whether the maneuver server is running.
         * 
         * @return bool True if the maneuver server is running, false otherwise
         */
        bool running() const;

        /**
         * @brief Shared pointer type.
         */
        typedef std::shared_ptr<ManeuverServer> SharedPtr;

    protected:
        /**
         * @brief Combined drone awareness handler, accessible to derived classes through the protected method.
         */
        const CombinedDroneAwarenessHandler::SharedPtr & awareness_handler() const;

        /**
         * @brief The currently executing maneuver, accessible to derived classes through the protected method.
         */
        const iii_drone::utils::Atomic<iii_drone::control::maneuver::Maneuver> & current_maneuver() const;

        /**
         * @brief Node pointer getter.
         */
        rclcpp_lifecycle::LifecycleNode * node() const;

        /**
         * @brief Registered maneuvers map getter.
         * 
         * @return The registered maneuvers map.
         */
        std::map<iii_drone::control::maneuver::maneuver_type_t, std::shared_ptr<iii_drone::control::maneuver::ManeuverServer>> registered_maneuvers() const;

        /**
         * @brief Maneuver result type enum.
         */
        enum maneuver_result_type_t {
            MANEUVER_RESULT_TYPE_SUCCEED,
            MANEUVER_RESULT_TYPE_CANCEL,
            MANEUVER_RESULT_TYPE_ABORT
        };

        /**
         * @brief Function to get the maneuver type of the specific maneuver server.
         * Must be implemented specific to the maneuver type.
         * 
         * @return maneuver_type_t The maneuver type
         */
        virtual maneuver_type_t maneuver_type() const = 0;

        /**
         * @brief Function which is called when starting execution of a maneuver.
         * 
         * @param maneuver The maneuver to start executing
         * 
         * @return void
         */
        virtual void startExecution(Maneuver & maneuver) = 0;

        /**
         * @brief Can cancel virtual condition function. Must be implemented specific to the maneuver type.
         * Should return true if the maneuver can currently be canceled, and false otherwise.
         * 
         * @return bool True if the maneuver can be canceled, false otherwise
         */
        virtual bool canCancel() = 0;

        /**
         * @brief Compute reference callback. Must be implemented specific to the maneuver type.
         * This function takes the current state of the drone and computes the next reference.
         */
        virtual Reference computeReference(const State &) = 0;

        /**
         * @brief Has succeeded callback, must return whether the maneuver has successfully completed.
         */
        virtual bool hasSucceeded(Maneuver &) = 0;

        /**
         * @brief Has failed callback, returns whether the maneuver has failed.
         */
        virtual bool hasFailed(Maneuver &) = 0;

        /**
         * @brief Returns the feedback. This function can be overwritten if feedback is desired.
         */
        virtual std::shared_ptr<void> getFeedback(Maneuver & maneuver);

        /**
         * @brief This function should publish the result and finalize the maneuver according to the result type.
         * This must be implemented specific to the maneuver type. Finalization needs to be done on the maneuver's goal handle.
         * This function is responsible for clearing or setting the target adapter with the combined drone awareness handler.
         */
        virtual void publishResultAndFinalize(
            Maneuver & maneuver,
            maneuver_result_type_t maneuver_result_type
        ) = 0;

        /**
         * @brief Must register the reference callback function of another maneuver server upon the completion of the current maneuver.
         * Must be implemented specific to the maneuver type. Is called after successful completion of the maneuver.
         * Should use the registerCallback() function.
         * 
         * @param maneuver The maneuver which has completed
         * 
         * @return void
         */
        virtual void registerReferenceCallbackOnSuccess(const Maneuver & maneuver) = 0;

        /**
         * @brief Registers a callback function for the reference. The callback function should be registered with the reference callback token.
         * 
         * @param callback The callback function
         * 
         * @return void
         */
        void registerCallback(const ReferenceCallback & callback);

        /**
         * @brief Creates the underlying action server.
         * 
         * @tparam ActionT Action type
         * 
         * @return void
         */
        template <typename ActionT>
        void createServer();

        /**
         * @brief Node getter.
         */
        const rclcpp_lifecycle::LifecycleNode & node_handle() const;

    private:
        /**
         * @brief Node pointer
         */
        rclcpp_lifecycle::LifecycleNode * node_;

        /**
         * @brief Combined drone awareness handler, accessible to derived classes through the protected method.
         */
        CombinedDroneAwarenessHandler::SharedPtr awareness_handler_;

        /**
         * @brief The currently executing maneuver, accessible to derived classes through the protected method.
         */
        iii_drone::utils::Atomic<iii_drone::control::maneuver::Maneuver> current_maneuver_;

        /**
         * @brief Mutex for restricting execution to one goal at a time.
         */
        std::mutex mutex_;

        /**
         * @brief Action name
         */
        std::string action_name_;

        /**
         * @brief Function pointer for registering a maneuver.
         * 
         * @param maneuver The maneuver to register
         * @param executing_instantly Output, whether the maneuver can be executed instantly
         * 
         * @return bool True if the maneuver was queued successfully, false otherwise
         */
        std::function<bool(Maneuver, bool &)> register_maneuver_;

        /**
         * @brief Function pointer for updating an already registered maneuver (with goal handle).
         * 
         * @param maneuver The new maneuver.
         * 
         * @return bool True if the maneuver was updated successfully, false otherwise
         */
        std::function<bool(Maneuver)> update_maneuver_;

        /**
         * @brief Function pointer for canceling a maneuver which has already been registered.
         * 
         * @param maneuver The maneuver to cancel. Terminate() must have been called on the maneuver before canceling.
         * 
         * @return bool True if the maneuver was canceled successfully, false otherwise
         */
        std::function<bool(Maneuver)> cancel_maneuver_;

        /**
         * @brief Function pointer for verifying that the maneuver is in the queue, pending execution.
         * 
         * @param maneuver The maneuver to verify.
         * 
         * @return bool True if the maneuver is in the queue, false otherwise
         */
        std::function<bool(Maneuver)> verify_maneuver_in_queue_;

        /**
         * @brief Function pointer for verifying that the maneuver is currently active for executiong.
         * 
         * @param maneuver The maneuver to verify.
         * 
         * @return bool True if the maneuver is currently active, false otherwise
         */
        std::function<bool(Maneuver)> verify_maneuver_active_;

        /**
         * @brief Done callback.
         */
        std::function<void(Maneuver)> done_callback_;

        /**
         * @brief Reference callback token.
         */
        ReferenceCallbackToken::SharedPtr reference_callback_token_;

        /**
         * @brief Wait for execute poll milliseconds
         */
        const unsigned int wait_for_execute_poll_ms_;

        /**
         * @brief Evaluate done poll milliseconds
         */
        const unsigned int evaluate_done_poll_ms_;

        /**
         * @brief Whether the server is running, atomic.
         */
        iii_drone::utils::Atomic<bool> running_;

        /**
         * @brief Action server shared void pointer
         */
        std::shared_ptr<void> server_;

        /**
         * @brief Handle goal callback.
         * 
         * @param uuid Goal UUID
         * @param goal Goal
         * @return rclcpp_action::GoalResponse Goal response
         */
        template <typename ActionT>
        rclcpp_action::GoalResponse handleGoal(
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const typename ActionT::Goal> goal
        );

        /**
         * @brief Handle cancel callback.
         * 
         * @param goal_handle Goal handle
         * 
         * @return rclcpp_action::CancelResponse Cancel response
         */
        template <typename ActionT>
        rclcpp_action::CancelResponse handleCancel(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle
        );

        /**
         * @brief Handle accepted callback.
         * 
         * @param goal_handle Goal handle
         */
        template <typename ActionT>
        void handleAccepted(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle
        );

        /**
         * @brief Execute.
         */
        template <typename ActionT>
        void asyncExecute(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle
        );

        /**
         * @brief The registered maneuver server map. Is used to register the reference callback of another maneuver server
         * upon the completion of the current maneuver, which is specific to the inheriting maneuver type.
        */
        std::map<iii_drone::control::maneuver::maneuver_type_t, iii_drone::control::maneuver::ManeuverServer::SharedPtr> registered_maneuvers_;

    };

} // namespace maneuver
} // namespace control
} // namespace iii_drone