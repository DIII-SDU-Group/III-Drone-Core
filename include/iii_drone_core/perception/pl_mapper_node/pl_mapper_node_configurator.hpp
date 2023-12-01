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
        const float kf_r() const;

        /**
         * @brief Get the Powerline Kalman filter q coefficient
         * 
         * @return Kalman filter q coefficient
         */
        const float kf_q() const;

        /**
         * @brief Get the alive counter low threshold 
         * 
         * @return Alive counter low threshold
         */
        const int alive_cnt_low_thresh() const;

        /**
         * @brief Get the alive counter high threshold
         * 
         * @return Alive counter high threshold
         */
        const int alive_cnt_high_thresh() const;

        /**
         * @brief Get the alive counter ceiling
         * 
         * @return Alive counter ceiling
         */
        const int alive_cnt_ceiling() const;

        /**
         * @brief Get the minimum distance between two points in the point cloud
         * 
         * @return Minimum distance between two points in the point cloud
         */
        const float min_point_dist() const;

        /**
         * @brief Get the maximum distance between two points in the point cloud
         * 
         * @return Maximum distance between two points in the point cloud
         */
        const float max_point_dist() const;

        /**
         * @brief Get the slope of the view cone
         * 
         * @return Slope of the view cone
         */
        const float view_cone_slope() const;

        /**
         * @brief Get the minimum distance between two points in the point cloud (strict)
         * 
         * @return Minimum distance between two points in the point cloud (strict)
         */
        const float min_point_dist_strict() const;

        /**
         * @brief Get the maximum distance between two points in the point cloud (strict)
         * 
         * @return Maximum distance between two points in the point cloud (strict)
         */
        const float max_point_dist_strict() const;

        /**
         * @brief Get the slope of the view cone (strict)
         * 
         * @return Slope of the view cone (strict)
         */
        const float view_cone_slope_strict() const;

        /**
         * @brief Get the maximum distance between two points in the point cloud (for line matching)
         * 
         * @return Maximum distance between two points in the point cloud (for line matching)
         */
        const float matching_line_max_dist() const;

        /**
         * @brief Get the initial sleep time in milliseconds
         * 
         * @return Initial sleep time in milliseconds
         */
        const int init_sleep_time_ms() const;

        /**
         * @brief Get the odometry callback period in milliseconds
         * 
         * @return Odometry callback period in milliseconds
         */
        const int odometry_callback_period_ms() const;

        /**
         * @brief Get the maximum number of lines
         * 
         * @return Maximum number of lines
         */
        const int max_lines() const;

        /**
         * @brief Get the skip predict when on cable flag
         * 
         * @return Skip predict when on cable flag
         */
        const bool skip_predict_when_on_cable() const;

        /**
         * @brief Get the inter line position window size
         * 
         * @return Inter line position window size
         */
        const int inter_pos_window_size() const;

        /**
         * @brief Get simulation mode flag
         * 
         * @return Simulation mode flag
         */
        const bool simulation() const;

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
