#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/configuration/configurator.hpp>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {

namespace perception {

namespace pl_dir_computer_node {

    /**
     * @brief Class for handling parameters for PowerlineDirectionComputer.
    */
    class PowerlineDirectionComputerConfigurator : public configuration::Configurator {

    public:
        /**
         * @brief Constructor
         *
         * @param node Reference to the handling node
         * @param after_parameter_change_callback Callback function called after successful parameter change, default is nullptr
         */
        PowerlineDirectionComputerConfigurator(
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
        PowerlineDirectionComputerConfigurator(
            rclcpp::Node *node, 
            const rclcpp::QoS & qos,
            std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback = nullptr
        );

        /**
         * @brief Get the Kalman filter r coefficient
         *
         * @return Kalman filter r coefficient
         */
        const float kf_r() const;

        /**
         * @brief Get the Kalman filter q coefficient
         *
         * @return Kalman filter q coefficient
         */
        const float kf_q() const;

        /**
         * @brief Get the initial ms sleep time
         *
         * @return Initial ms sleep time
         */
        const int init_sleep_time_ms() const;

        /**
         * @brief Get the odometry callback period ms
         * 
         * @return Odometry callback period ms
         */
        const int odometry_callback_period_ms() const;

        /**
         * @brief Get the pl_mapper min_point_dist parameter
         * 
         * @return pl_mapper min_point_dist parameter
         */
        const float min_point_dist() const;

        /**
         * @brief Get the pl_mapper max_point_dist parameter
         * 
         * @return pl_mapper max_point_dist parameter
         */
        const float max_point_dist() const;

        /**
         * @brief Get the pl_mapper view_cone_slope parameter
         * 
         * @return pl_mapper view_cone_slope parameter
         */
        const float view_cone_slope() const;

        /**
         * @brief Get the drone frame ID
         *
         * @return Drone frame ID
         */
        const std::string drone_frame_id() const;

        /**
         * @brief Get the world frame ID
         *
         * @return World frame ID
         */
        const std::string world_frame_id() const;

        /**
         * @brief Get the cable gripper frame ID
         * 
         * @return Cable gripper frame ID
         */
        const std::string cable_gripper_frame_id() const;

        /**
         * @brief Get the mmWave frame ID
         * 
         * @return mmWave frame ID
         */
        const std::string mmwave_frame_id() const;

    private:
        /**
         * @brief Declares the node specific parameters
         * 
         * @return void
         */
        void declareNodeParameters();

    };

} // namespace hough_transformer_node
} // namespace perception
} // namespace iii_drone
