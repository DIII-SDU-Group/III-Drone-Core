#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <memory>
#include <mutex>
#include <shared_mutex>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <tf2_ros/buffer.h>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/msg/reference.hpp>
#include <iii_drone_interfaces/msg/maneuver.hpp>
#include <iii_drone_interfaces/msg/maneuver_queue.hpp>
#include <iii_drone_interfaces/msg/string_stamped.hpp>

#include <iii_drone_interfaces/srv/get_reference.hpp>

/*****************************************************************************/
// III-Drone-Configuration:

#include <iii_drone_configuration/parameter_bundle.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/types.hpp>
#include <iii_drone_core/utils/atomic.hpp>
#include <iii_drone_core/utils/history.hpp>
#include <iii_drone_core/utils/token.hpp>

#include <iii_drone_core/adapters/px4/vehicle_status_adapter.hpp>
#include <iii_drone_core/adapters/px4/vehicle_odometry_adapter.hpp>
#include <iii_drone_core/adapters/powerline_adapter.hpp>
#include <iii_drone_core/adapters/single_line_adapter.hpp>
#include <iii_drone_core/adapters/gripper_status_adapter.hpp>
#include <iii_drone_core/adapters/reference_adapter.hpp>
#include <iii_drone_core/adapters/maneuver_adapter.hpp>

#include <iii_drone_core/control/state.hpp>
#include <iii_drone_core/control/reference.hpp>
#include <iii_drone_core/control/combined_drone_awareness_handler.hpp>

#include <iii_drone_core/control/maneuver/maneuver_types.hpp>
#include <iii_drone_core/control/maneuver/maneuver.hpp>
#include <iii_drone_core/control/maneuver/maneuver_queue.hpp>
#include <iii_drone_core/control/maneuver/reference_callback_token.hpp>

#include <iii_drone_core/control/maneuver/maneuver_server.hpp>
#include <iii_drone_core/control/maneuver/hover_maneuver_server.hpp>
#include <iii_drone_core/control/maneuver/hover_by_object_maneuver_server.hpp>
#include <iii_drone_core/control/maneuver/hover_on_cable_maneuver_server.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {

namespace control {

namespace maneuver {

    /**
     * @brief Class that schedules execution of drone maneuvers.
     * Maneuver server are registered using the dedicated functions. 
     * Each registered maneuver is associated with a type and a maneuver server.
     * The server exposes an action that can be used to execute the maneuver.
     * This class is responsible for scheduling the execution of the registered maneuvers,
     * including checking whether a maneuver can be executed immediately or later.
     * Maneuvers are queued for later execution. When a maneuver is finished,
     * the next maneuver is started using the maneuver server API.
     * The class exposes a service with which a client can fetch the next reference,
     * which underlyingly is fetched from the executing maneuver server.
     */
    class ManeuverScheduler {
    public:
        /**
         * @brief Constructor
         * 
         * @param node Simple pointer to the containing node.
         * @param combined_drone_awareness_handler Shared pointer to the combined drone awareness handler for retrieving drone awareness.
         * @param parameters Shared pointer to the parameters for the maneuver scheduler.
         * @param maneuver_execution_callback_group Shared pointer to the callback group for parallel maneuver execution.
         */
        ManeuverScheduler(
            rclcpp_lifecycle::LifecycleNode *node,
            const iii_drone::control::CombinedDroneAwarenessHandler::SharedPtr combined_drone_awareness_handler,
            const iii_drone::configuration::ParameterBundle::SharedPtr parameters,
            rclcpp::CallbackGroup::SharedPtr maneuver_execution_callback_group
        );

        /**
         * @brief Destructor
         */
        ~ManeuverScheduler();

        /**
         * @brief Starts the maneuver scheduler.
         */
        void Start();

        /**
         * @brief Stops the maneuver scheduler.
         */
        void Stop();

        /**
         * @brief Registers a maneuver type with a maneuver server.
         * 
         * @param maneuver_type The maneuver type.
         * @param server The maneuver server shared pointer.
         */
        void RegisterManeuverServer(
            iii_drone::control::maneuver::maneuver_type_t maneuver_type,
            ManeuverServer::SharedPtr maneuver_server
        );

        /**
         * @brief Unregisters a maneuver type. Removes the maneuver server associated with the maneuver type.
         * 
         * @param maneuver_type The maneuver type.
         */
        void UnregisterManeuverServer(iii_drone::control::maneuver::maneuver_type_t maneuver_type);

        /**
         * @brief Returns the expected combined drone awareness upon completion or failure of given maneuver type, considering the full maneuver queue.
         * 
         * @param maneuver The maneuver.
         * @param awareness_after The combined drone awareness after the maneuver.
         * 
         * @return true if all maneuvers in the queue could execute successfully, and the maneuver could execute succesfully after them.
         */
        bool ProjectExpectedAwarenessFull(
            const iii_drone::control::maneuver::Maneuver & maneuver,
            combined_drone_awareness_t & awareness_after
        ) const;

        /**
         * @brief Returns the expected combined drone awareness upon completion or failure of given maneuver type, given a current combined drone awareness.
         * 
         * @param maneuver The maneuver to be executed at the end of the current queue.
         * @param awareness_before The inital combined drone awareness.
         * @param awareness_after The combined drone awareness after the maneuver.
         * 
         * @return Whether the maneuver could execute succesfully.
         */
        bool ProjectExpectedAwarenessSingle(
            const iii_drone::control::maneuver::Maneuver & maneuver, 
            const combined_drone_awareness_t & awareness_before,
            combined_drone_awareness_t & awareness_after
        ) const;

        /**
         * @brief Returns whether a given maneuver can immediately execute
         * 
         * @param maneuver The maneuver.
         * 
         * @return true if the maneuver can immediately execute.
         */
        bool CanExecute(const iii_drone::control::maneuver::Maneuver & maneuver) const;

        /**
         * @brief Returns whether a given maneuver can execute later.
         * 
         * @param maneuver The maneuver.
         * 
         * @return true if the maneuver can execute later.
         */
        bool CanSchedule(const iii_drone::control::maneuver::Maneuver & maneuver) const;

        /**
         * @brief Registers a maneuver with the maneuver scheduler. To be called from a maneuver server.
         * If the maneuver execution timer is not running, it is started.
         * After call to this function, the maneuver must be updated with the correct goal handle
         * using the UpdateManeuver() method before it is selected for execution.
         * This must happen before the timeout, otherwise the maneuver is aborted.
         * 
         * @param maneuver The maneuver.
         * @param will_execute_immediately Whether the maneuver will execute immediately, output.
         * 
         * @return true if the maneuver was successfully registered, false if the maneuver is rejected by the scheduler.
         */
        bool RegisterManeuver(
            iii_drone::control::maneuver::Maneuver maneuver,
            bool & will_execute_immediately
        );

        /**
         * @brief Updates a maneuver entry which has been registered with the correct goal handle after the maneuver has been selected for execution.
         * 
         * @param maneuver The maneuver.
         * 
         * @return true if the maneuver was successfully updated, false if the maneuver is not in the queue or has timed out.
         */
        bool UpdateManeuver(iii_drone::control::maneuver::Maneuver maneuver);

        /**
         * @brief Cancels a maneuver entry which has been registered before execution starts.
         * 
         * @param maneuver The maneuver. Must have been terminated using the Terminate() method.
         * 
         * @return true if the maneuver was successfully canceled, false if the maneuver is not in the queue.
         */
        bool CancelManeuver(iii_drone::control::maneuver::Maneuver maneuver);

        /**
         * @brief Get copy of the currently executing maneuver.
         * The currently executing maneuver is of type MANEUVER_TYPE_NONE if no maneuver is being executed.
         * 
         * @return The current maneuver.
         */
        iii_drone::control::maneuver::Maneuver current_maneuver() const;

        /**
         * @brief Get whether a maneuver is being executed or pending.
         * 
         * @return true if a maneuver is being executed or pending.
         */
        bool maneuverIsExecutingOrPending() const;

        /**
         * @brief Shared pointer type.
         */
        typedef std::shared_ptr<ManeuverScheduler> SharedPtr;

        /**
         * @brief Unique pointer type.
         */
        typedef std::unique_ptr<ManeuverScheduler> UniquePtr;

    private:
        /**
         * @brief Is started flag.
         */
        iii_drone::utils::Atomic<bool> is_started_ = false;

        /**
         * @brief Evaluates whether a maneuver can be executed given an awareness.
         * 
         * @param maneuver The maneuver.
         * @param awareness The combined drone awareness.
         * 
         * @return true if the maneuver can be executed.
         */
        bool maneuverCanExecute(
            const iii_drone::control::maneuver::Maneuver & maneuver, 
            const iii_drone::control::combined_drone_awareness_t & awareness
        ) const;

        /**
         * @brief OnManeuverCompleted callback. Called every time a maneuver is completed.
         * 
         * @param maneuver The maneuver. This maneuver must have been terminated using the Termiante() method.
         */
        void onManeuverCompleted(iii_drone::control::maneuver::Maneuver maneuver);

        /**
         * @brief Callback function for when the token is yielded from a maneuver server.
         * Registers the hovering reference callback function if the maneuver failed.
         * 
         * @return void
         */
        void onReferenceCallbackTokenReacquired();

        /**
         * @brief Get passthrough reference.
         * 
         * @param state The state.
         * 
         * @return The reference.
         */
        Reference getPassthroughReference(const State & state) const;

        /**
         * @brief Reference to the node for creating of ROS2 objects.
         */
        rclcpp_lifecycle::LifecycleNode *node_;

        /**
         * @brief Shared pointer to the combined drone awareness handler for retrieving drone awareness.
         */
        iii_drone::control::CombinedDroneAwarenessHandler::SharedPtr combined_drone_awareness_handler_;

        /**
         * @brief The maneuver scheduler parameters.
         */
        iii_drone::configuration::ParameterBundle::SharedPtr parameters_;

        /**
         * @brief The maneuver execution callback group.
         */
        rclcpp::CallbackGroup::SharedPtr maneuver_execution_callback_group_;

        /**
         * @brief The maneuver queue.
         */
        iii_drone::control::maneuver::ManeuverQueue::UniquePtr maneuver_queue_;

        /**
         * @brief The current maneuver.
         */
        iii_drone::utils::Atomic<iii_drone::control::maneuver::Maneuver> current_maneuver_;

        /**
         * @brief Maneuver mutex.
         */
        mutable std::shared_mutex maneuver_mutex_;

        /**
         * @brief Main timer for maneuver execution.
         */
        rclcpp::TimerBase::SharedPtr maneuver_execution_timer_;

        /**
         * @brief The reference callback struct.
         */
        ReferenceCallbackStruct::SharedPtr reference_callback_struct_;

        /**
         * @brief The master token for the reference callback function access.
         */
        ReferenceCallbackToken reference_callback_token_;

        /**
         * @brief Reference callback provider publisher.
         */
        rclcpp_lifecycle::LifecyclePublisher<iii_drone_interfaces::msg::StringStamped>::SharedPtr reference_callback_provider_publisher_;

        /**
         * @brief Reference callback provider publish timer.
         */
        rclcpp::TimerBase::SharedPtr reference_callback_provider_publish_timer_;

        /**
         * @brief The counter for the number of iterations without a maneuver before the scheduler goes from hovering to idle.
         */
        iii_drone::utils::Atomic<int> no_maneuver_idle_cnt_ = -1;

        /**
         * @brief Flag indicating that a maneuver server get reference callback is still registered
         * even though the maneuver has terminated, as can be the case for hover maneuvers that are not sustained.
         */
        iii_drone::utils::Atomic<bool> maneuver_server_get_reference_callback_still_registered_ = false;

        /**
         * @brief Callback for when hovering fails (hover by object or hover on cable).
         * 
         * @return void
         */
        void onHoveringFail();

        /**
         * @brief ROS2 reference publisher.
         */
        rclcpp_lifecycle::LifecyclePublisher<iii_drone_interfaces::msg::Reference>::SharedPtr reference_publisher_;

        /**
         * @brief Get reference service.
         */
        rclcpp::Service<iii_drone_interfaces::srv::GetReference>::SharedPtr get_reference_service_;

        /**
         * @brief Get reference service callback.
         * 
         * @param request The request.
         * @param response The response.
         * 
         * @return void
         */
        void getReferenceServiceCallback(
            const std::shared_ptr<iii_drone_interfaces::srv::GetReference::Request> request,
            std::shared_ptr<iii_drone_interfaces::srv::GetReference::Response> response
        );

        /**
         * @brief The maneuver execution timer callback.
         */
        void maneuverExecutionTimerCallback();

        /**
         * @brief Progresses the scheduler. If a maneuver is executing, evaluates whether it has terminated.
         * If a maneuver maneuver has terminated successfully, the next maneuver is started.
         * If a maneuver has terminated unsuccessfully, all maneuvers are canceled by clearing the maneuver queue.
         * When a maneuver stops executing, one of the hover maneuver server callbacks are registered through
         * the onFinishedManeuver callback. Whenever no maneuver is executing, a counter counts down.
         * When the counter reaches 0 without a maneuver scheduled for execution,
         * the timer for running the maneuver execution loop is stopped.
         * 
         * @return void
         */
        void progressScheduler();

        /**
         * @brief Fetches the next reference and publishes it. Will fetch the reference using
         * the registered callback, either associated with a maneuver server during active execution
         * or one of the hover server callbacks. When the maneuver execution timer is stopped,
         * no more references will be published until a new maneuver is scheduled and
         * the timer is restarted.
         * 
         * @return reference msg
         */
        iii_drone_interfaces::msg::Reference fetchNextReferenceAndPublish();

        /**
         * @brief Cancels all pending maneuvers.
         * 
         * @return void
         */
        void cancelAllPendingManeuvers();

        /**
         * @brief The registered maneuvers. Map of maneuver type to maneuver server base class.
         */
        std::map<iii_drone::control::maneuver::maneuver_type_t, iii_drone::control::maneuver::ManeuverServer::SharedPtr> registered_maneuvers_;

        /**
         * @brief Current maneuver publisher.
         */
        rclcpp_lifecycle::LifecyclePublisher<iii_drone_interfaces::msg::Maneuver>::SharedPtr current_maneuver_publisher_;

        /**
         * @brief Maneuver queue publisher.
         */
        rclcpp_lifecycle::LifecyclePublisher<iii_drone_interfaces::msg::ManeuverQueue>::SharedPtr maneuver_queue_publisher_;

        /**
         * @brief Maneuver publish timer.
         */
        rclcpp::TimerBase::SharedPtr maneuver_publish_timer_;


    };

} // namespace maneuver
} // namespace control
} // namespace iii_drone