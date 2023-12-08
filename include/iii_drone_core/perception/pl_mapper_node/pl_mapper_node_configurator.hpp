#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/configuration/configurator.hpp>

#include <iii_drone_core/perception/powerline_parameters.hpp>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

/*****************************************************************************/
// Std:

#include <memory>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {

namespace perception {

namespace pl_mapper_node {

    /**
     * @brief Class for handling parameters for PowerlineMapper.
    */
    class PowerlineMapperConfigurator : public configuration::Configurator {

    public:
        /**
         * @brief Constructor
         *
         * @param node Reference to the handling node
         * @param after_parameter_change_callback Callback function called after successful parameter change, default is nullptr
         */
        PowerlineMapperConfigurator(
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
        PowerlineMapperConfigurator(
            rclcpp::Node *node, 
            const rclcpp::QoS & qos,
            std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback = nullptr
        );

        /**
         * @brief Get the Powerline Kalman filter r coefficient
         * 
         * @return Kalman filter r coefficient
         */
        float kf_r() const;

        /**
         * @brief Get the Powerline Kalman filter q coefficient
         * 
         * @return Kalman filter q coefficient
         */
        float kf_q() const;

        /**
         * @brief Get the alive counter low threshold 
         * 
         * @return Alive counter low threshold
         */
        int alive_cnt_low_thresh() const;

        /**
         * @brief Get the alive counter high threshold
         * 
         * @return Alive counter high threshold
         */
        int alive_cnt_high_thresh() const;

        /**
         * @brief Get the alive counter ceiling
         * 
         * @return Alive counter ceiling
         */
        int alive_cnt_ceiling() const;

        /**
         * @brief Get the minimum distance between two points in the point cloud
         * 
         * @return Minimum distance between two points in the point cloud
         */
        float min_point_dist() const;

        /**
         * @brief Get the maximum distance between two points in the point cloud
         * 
         * @return Maximum distance between two points in the point cloud
         */
        float max_point_dist() const;

        /**
         * @brief Get the slope of the view cone
         * 
         * @return Slope of the view cone
         */
        float view_cone_slope() const;

        /**
         * @brief Get the minimum distance between two points in the point cloud (strict)
         * 
         * @return Minimum distance between two points in the point cloud (strict)
         */
        float strict_min_point_dist() const;

        /**
         * @brief Get the maximum distance between two points in the point cloud (strict)
         * 
         * @return Maximum distance between two points in the point cloud (strict)
         */
        float strict_max_point_dist() const;

        /**
         * @brief Get the slope of the view cone (strict)
         * 
         * @return Slope of the view cone (strict)
         */
        float strict_view_cone_slope() const;

        /**
         * @brief Get the maximum distance between two points in the point cloud (for line matching)
         * 
         * @return Maximum distance between two points in the point cloud (for line matching)
         */
        float matching_line_max_dist() const;

        /**
         * @brief Get the initial sleep time in milliseconds
         * 
         * @return Initial sleep time in milliseconds
         */
        int init_sleep_time_ms() const;

        /**
         * @brief Get the odometry callback period in milliseconds
         * 
         * @return Odometry callback period in milliseconds
         */
        int odometry_callback_period_ms() const;

        /**
         * @brief Get the maximum number of lines
         * 
         * @return Maximum number of lines
         */
        int max_lines() const;

        /**
         * @brief Get the inter line position window size
         * 
         * @return Inter line position window size
         */
        int inter_pos_window_size() const;

        /**
         * @brief Get simulation mode flag
         * 
         * @return Simulation mode flag
         */
        bool simulation() const;

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

        /**
         * @brief PowerlineParameters getter
        */
        std::shared_ptr<PowerlineParameters> powerline_parameters() const;

    private:
        /**
         * @brief Declares the node specific parameters
         * 
         * @return void
         */
        void declareNodeParameters();

        /**
         * @brief Initialize the parameters
        */
        void initializeParameters();

        /**
         * @brief On changed parameter callback.
        */
        void plMapperOnParameterChange(const rclcpp::Parameter &parameter);

        /**
         * @brief On parameter change callback handle.
        */
        std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback_;

        /**
         * @brief The Powerline parameters.
        */
        std::shared_ptr<PowerlineParameters> parameters_;

    };

} // namespace hough_transformer_node
} // namespace perception
} // namespace iii_drone
