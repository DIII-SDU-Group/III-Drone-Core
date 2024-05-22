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

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/atomic.hpp>
#include <iii_drone_core/utils/history.hpp>

#include <iii_drone_core/control/reference.hpp>

#include <iii_drone_core/adapters/px4/vehicle_odometry_adapter.hpp>
#include <iii_drone_core/adapters/reference_adapter.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/msg/reference.hpp>
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
         * @param use_nans_when_hovering Whether to use nans for velocities and accelerations when hovering.
         * @param max_failed_attempts_during_maneuver The maximum number of failed attempts to acquire reference during a maneuver.
         * @param get_reference_timeout_ms The timeout in milliseconds for obtaining a reference from the service.
         * @param on_fail_during_maneuver The callback to call when failing to acquire valid reference during a maneuver.
         */
        ManeuverReferenceClient(
            rclcpp::Node * node,
            iii_drone::utils::History<iii_drone::adapters::px4::VehicleOdometryAdapter>::SharedPtr vehicle_odometry_adapter_history,
            bool use_nans_when_hovering,
            int max_failed_attempts_during_maneuver,
            int  get_reference_timeout_ms,
            std::function<void()> on_fail_during_maneuver
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
        void SetReferenceModeHover();

        /**
         * @brief Starts a maneuver. References will be consumed from the reference topic.
         */
        void StartManeuver();

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
         * 
         * @return iii_drone::control::Reference The reference.
         */
        iii_drone::control::Reference GetReference(double dt);

        /**
         * @brief Shared pointer type.
         */
        typedef std::shared_ptr<ManeuverReferenceClient> SharedPtr;

    private:
        /**
         * @brief The node pointer.
         */
        rclcpp::Node * node_;

        /**
         * @brief Get reference callback group.
         */
        rclcpp::CallbackGroup::SharedPtr get_reference_cb_group_;

        /**
         * @brief Stop maneuver timer.
         */
        rclcpp::TimerBase::SharedPtr stop_maneuver_timer_;

        /**
         * @brief Stop maneuver timer mutex.
         */
        std::mutex stop_maneuver_timer_mutex_;

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
            MANEUVER
        };

        /**
         * @brief The reference mode.
         */
        iii_drone::utils::Atomic<reference_mode_t> reference_mode_;

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
         * @brief Whether to use nans for velocities and accelerations when hovering.
         */
        const bool use_nans_when_hovering_;

        /**
         * @brief The maximum number of failed attempts to acquire reference during a maneuver.
         */
        const int max_failed_attempts_during_maneuver_;

        /**
         * @brief The timeout in milliseconds for obtaining a reference from the service.
         */
        const int get_reference_timeout_ms_;

        /**
         * @brief The callback to call when failing to acquire valid reference during a maneuver.
         */
        std::function<void()> on_fail_during_maneuver_;

    };

} // namespace maneuver
} // namespace control
} // namespace iii_drone