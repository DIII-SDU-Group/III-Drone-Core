#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/configuration/configurator.hpp>
#include <iii_drone_core/perception/hough_transformer_parameters.hpp>

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

namespace hough_transformer_node {

    /**
     * @brief Class for handling parameters for HoughTransformer.
    */
    class HoughTransformerConfigurator : public configuration::Configurator {

    public:
        /**
         * @brief Constructor
         *
         * @param node Reference to the handling node
         * @param after_parameter_change_callback Callback function called after successful parameter change, default is nullptr
         */
        HoughTransformerConfigurator(
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
        HoughTransformerConfigurator(
            rclcpp::Node *node, 
            const rclcpp::QoS & qos,
            std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback = nullptr
        );

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
         * @brief Get the Canny low threshold
         *
         * @return Canny low threshold
         */
        int canny_low_threshold() const;

        /**
         * @brief Get the Canny ratio
         *
         * @return Canny ratio
         */
        int canny_ratio() const;

        /**
         * @brief Get the Canny kernel size
         *
         * @return Canny kernel size
         */
        int canny_kernel_size() const;

        /**
         * @brief Get the number of lines to include
         *
         * @return Number of lines to include
         */
        int n_lines_include() const;

        /**
         * @brief HoughTransformerParameters getter
         * 
         * @return HoughTransformerParameters
         */
        std::shared_ptr<HoughTransformerParameters> hough_transformer_parameters() const;

    private:
        /**
         * @brief Declares the node specific parameters
         * 
         * @return void
         */
        void declareNodeParameters();

        /**
         * @brief Initialises the HoughTransformer parameters
         * 
         * @return void
         */
        void initHoughTransformerParameters();

        /**
         * @brief The HoughTransformer parameters
         */
        std::shared_ptr<HoughTransformerParameters> hough_transformer_parameters_;

        /**
         * @brief The callback function called after successful parameter change
         */
        std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback_ = nullptr;

        /**
         * @brief the HoughTransformerConfiguration callback function called after successful parameter change
         * 
         * @param parameter The parameter that was changed
         * 
         * @return void
        */
        void houghTransformerConfiguratorParameterChangeCallback(const rclcpp::Parameter & parameter);

    };

} // namespace hough_transformer_node
} // namespace perception
} // namespace iii_drone
