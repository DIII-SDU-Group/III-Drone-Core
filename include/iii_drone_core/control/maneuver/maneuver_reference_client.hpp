#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <memory>
#include <queue>
#include <mutex>
#include <chrono>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/atomic.hpp>
#include <iii_drone_core/utils/history.hpp>

#include <iii_drone_core/control/reference.hpp>

#include <iii_drone_core/adapters/px4/vehicle_odometry_adapter.hpp>
#include <iii_drone_core/adapters/reference_adapter.hpp>

/*****************************************************************************/
// III-Drone-Configuration:

#include <iii_drone_configuration/parameter_bundle.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/msg/reference.hpp>
#include <iii_drone_interfaces/msg/string_stamped.hpp>

#include <iii_drone_interfaces/srv/get_reference.hpp>

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
         * @param parameters Parameter bundle
         */
        ManeuverReferenceClient(
            rclcpp_lifecycle::LifecycleNode * node,
            iii_drone::utils::History<iii_drone::adapters::px4::VehicleOdometryAdapter>::SharedPtr vehicle_odometry_adapter_history,
            iii_drone::configuration::ParameterBundle::SharedPtr parameters
        );

        /**
         * @brief Update the reference to the current state if the reference mode is hover or passthrough, otherwise do nothing.
         */
        void UpdateReference();

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

        /**
         * @brief Starts a maneuver. References will be consumed from the reference topic.
         * 
         * @return true If the maneuver is started.
         */
        bool StartManeuver();

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
         * @brief The node pointer.
         */
        rclcpp_lifecycle::LifecycleNode * node_;

        /**
         * @brief Get reference callback group.
         */
        rclcpp::CallbackGroup::SharedPtr get_reference_cb_group_;

        /**
         * @brief Stop maneuver timer.
         */
        utils::Atomic<rclcpp::TimerBase::SharedPtr> stop_maneuver_timer_;

        /**
         * @brief Stop maneuver timer callback.
         */
        utils::Atomic<std::function<void()>> stop_maneuver_timer_callback_;

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
            WAIT_FOR_MANEUVER_STOP
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
         * @brief The reference mutex.
         */
        std::mutex reference_mutex_;

        /**
         * @brief The get reference service client.
         */
        rclcpp::Client<iii_drone_interfaces::srv::GetReference>::SharedPtr get_reference_client_;

        /**
         * @brief Sends a request to the get reference service.
         * 
         * @param reference The reference output.
         * 
         * @return true If the request was successful.
         */
        bool getReferenceFromServer(Reference & reference);

        /**
         * @brief Parameter bundle
         */
        iii_drone::configuration::ParameterBundle::SharedPtr parameters_;

    };

} // namespace maneuver
} // namespace control
} // namespace iii_drone