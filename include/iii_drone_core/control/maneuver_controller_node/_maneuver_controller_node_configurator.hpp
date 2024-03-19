#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <memory>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/configuration/configurator.hpp>

#include <iii_drone_core/control/combined_drone_awareness_handler_parameters.hpp>
#include <iii_drone_core/control/maneuver/maneuver_scheduler_parameters.hpp>
#include <iii_drone_core/control/maneuver/fly_to_position_maneuver_server_parameters.hpp>
#include <iii_drone_core/control/maneuver/fly_to_object_maneuver_server_parameters.hpp>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {

namespace control {

namespace maneuver_controller_node {

    /**
     * @brief Class for handling parameters for ManeuverController.
     * Exposes a CombinedDroneAwarenessHandlerParameters object, a ManeuverSchedulerParameters object,
     * and parameters for the maneuver servers.
     */
    class ManeuverControllerNodeConfigurator : public configuration::Configurator {

    public:
        /**
         * @brief Constructor
         *
         * @param node Reference to the handling node
         * @param after_parameter_change_callback Callback function called after successful parameter change, default is nullptr
         */
        ManeuverControllerNodeConfigurator(
            rclcpp::Node *node,
            std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback = nullptr
        );

        /**
         * @brief Constructor
         * 
         * @param node Reference to the handling node
         * @param qos QoS profile for the parameter event subscription
         * @param after_parameter_change_callback Callback function called after successful parameter change, default is nullptr
         */
        ManeuverControllerNodeConfigurator(
            rclcpp::Node *node, 
            const rclcpp::QoS & qos,
            std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback = nullptr
        );

        /**
         * @brief Get the combined drone awareness handler parameters object
         * 
         * @return Combined drone awareness handler parameters object reference
         */
        const std::shared_ptr<control::CombinedDroneAwarenessHandlerParameters> combined_drone_awareness_handler_parameters() const;

        /**
         * @brief Get the maneuver scheduler parameters object
         * 
         * @return Maneuver scheduler parameters object reference
         */
        const std::shared_ptr<control::maneuver::ManeuverSchedulerParameters> maneuver_scheduler_parameters() const;

        /**
         * @brief Get the fly to position maneuver server parameters object
         * 
         * @return Fly to position maneuver server parameters object reference
         */
        const std::shared_ptr<control::maneuver::FlyToPositionManeuverServerParameters> fly_to_position_maneuver_server_parameters() const;

        /**
         * @brief Get the fly to object maneuver server parameters object
         * 
         * @return Fly to object maneuver server parameters object reference
         */
        const std::shared_ptr<control::maneuver::FlyToObjectManeuverServerParameters> fly_to_object_maneuver_server_parameters() const;

        /**
         * @brief Get the window size for the ground estimate history.
         * 
         * @return The window size for the ground estimate history.
         */
        int ground_estimate_window_size() const;

        /**
         * @brief Get the period for updating the ground estimate.
         * 
         * @return The period for updating the ground estimate.
         */
        int ground_estimate_update_period_ms() const;

        /**
         * @brief Get the altitude threshold for the drone to be considered landed.
         * 
         * @return The altitude threshold for the drone to be considered landed.
         */
        double landed_altitude_threshold() const;

        /**
         * @brief Get the on_cable_max_euc_distance value.
         * 
         * @return The on_cable_max_euc_distance value.
         */
        double on_cable_max_euc_distance() const;

        /**
         * @brief Get the fail_on_unable_to_locate value.
         * 
         * @return The fail_on_unable_to_locate value.
         */
        bool fail_on_unable_to_locate() const;

        /**
         * @brief Get the cable gripper frame id.
         * 
         * @return The cable gripper frame id.
         */
        std::string cable_gripper_frame_id() const;

        /**
         * @brief Get the drone frame id.
         * 
         * @return The drone frame id.
         */
        std::string drone_frame_id() const;

        /**
         * @brief Get the world frame id.
         * 
         * @return The world frame id.
         */
        std::string world_frame_id() const;

        /**
         * @brief Get the ground frame id.
         * 
         * @return The ground frame id.
         */
        std::string ground_frame_id() const;

        /**
         * @brief Get the timeout for updating a maneuver after registering it.
         * 
         * @return The timeout for updating a maneuver after registering it.
         */
        double maneuver_register_update_timeout_s() const;

        /**
         * @brief Get the timeout for the server to start executing a maneuver after Start() has been called.
         * 
         * @return The timeout for the server to start executing a maneuver after Start() has been called.
         */
        double maneuver_start_timeout_s() const;

        /**
         * @brief Get the size of the maneuver queue.
         * 
         * @return The size of the maneuver queue.
         */
        unsigned int maneuver_queue_size() const;

        /**
         * @brief Get the period for executing maneuvers.
         * 
         * @return The period for executing maneuvers.
         */
        unsigned int maneuver_execution_period_ms() const;

        /**
         * @brief Get whether to use NaNs for velocity and acceleration when hovering, otherwise uses zeros.
         * 
         * @return Whether to use NaNs for velocity and acceleration when hovering, otherwise uses zeros.
         */
        bool use_nans_when_hovering() const;

        /**
         * @brief Get the number of seconds with no maneuver before the scheduler goes from hovering to idle.
         * 
         * @return The number of seconds with no maneuver before the scheduler goes from hovering to idle.
         */
        double no_maneuver_idle_cnt_s() const;

        /**
         * @brief Get the maximum euclidean distance to the object before the fail callback is called.
         * 
         * @return The maximum euclidean distance to the object before the fail callback is called.
         */
        double hover_by_object_max_euc_dist() const;

        /**
         * @brief Get the default z velocity for hovering on a cable.
         * 
         * @return The default z velocity for hovering on a cable.
         */
        double hover_on_cable_default_z_velocity() const;

        /**
         * @brief Get the default yaw rate for hovering on a cable.
         * 
         * @return The default yaw rate for hovering on a cable.
         */
        double hover_on_cable_default_yaw_rate() const;

        /**
         * @brief Get the PX4 mode ids considered as offboard.
         * 
         * @return The PX4 mode ids considered as offboard.
         */
        std::vector<int64_t> px4_offboard_mode_ids() const;

        /**
         * @brief Get the poll period in ms for waiting for execution of maneuver.
         * 
         * @return The poll period in ms for waiting for execution of maneuver.
         */
        unsigned int maneuver_wait_for_execute_poll_ms() const;

        /**
         * @brief Get the poll period in ms for evaluating if a maneuver is done.
         * 
         * @return The poll period in ms for evaluating if a maneuver is done.
         */
        unsigned int maneuver_evaluate_done_poll_ms() const;

        /**
         * @brief Get the reached position threshold for evaluating if the target position has been reached.
         * 
         * @return The reached position threshold for evaluating if the target position has been reached.
         */
        double reached_position_euclidean_distance_threshold() const;

        /**
         * @brief Get the minimum target altitude for the target position.
         * 
         * @return The minimum target altitude for the target position.
         */
        double minimum_target_altitude() const;

        /**
         * @brief Get the flag for whether to generate trajectories asynchronously
         * with one iteration delay.
         * 
         * @return The flag for whether to generate trajectories asynchronously
         * with one iteration delay.
         */
        bool generate_trajectories_asynchronously_with_delay() const;

        /**
         * @brief Get the poll period in ms for waiting for the generation of trajectories.
         * 
         * @return The poll period in ms for waiting for the generation of trajectories.
         */
        unsigned int generate_trajectories_poll_period_ms() const;

    private:
        /**
         * @brief Combined drone awareness handler parameters object
         */
        std::shared_ptr<control::CombinedDroneAwarenessHandlerParameters> combined_drone_awareness_handler_parameters_;

        /**
         * @brief Maneuver scheduler parameters object
         */
        std::shared_ptr<control::maneuver::ManeuverSchedulerParameters> maneuver_scheduler_parameters_;

        /**
         * @brief Fly to position maneuver server parameters object
         */
        std::shared_ptr<control::maneuver::FlyToPositionManeuverServerParameters> fly_to_position_maneuver_server_parameters_;

        /**
         * @brief Fly to object maneuver server parameters object
         */
        std::shared_ptr<control::maneuver::FlyToObjectManeuverServerParameters> fly_to_object_maneuver_server_parameters_;

        /**
         * @brief Parameters objects initialized flag
         */
        bool parameters_objects_initialized_ = false;

        /**
         * @brief Declares the node specific parameters
         * 
         * @return void
         */
        void declareNodeParameters();

        /**
         * @brief Initializes the parameters objects
         */
        void initParametersObjects();

        /**
         * @brief Callback function called after successful parameter change to update parameters object.
         */
        void updateParametersObjectsCallback(const rclcpp::Parameter & parameter);

        /**
         * @brief Callback function reference called after successful parameter change.
         */
        std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback_;

    };

} // namespace maneuver_controller_node
} // namespace control
} // namespace iii_drone
