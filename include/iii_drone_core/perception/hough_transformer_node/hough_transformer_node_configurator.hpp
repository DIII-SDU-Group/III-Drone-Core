#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/configuration/tf/tf_configurator.hpp>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {

namespace perception {

namespace hough_transformer_node {

    /**
     * @brief Class for handling parameters for HoughTransformer.
    */
    class HoughTransformerConfigurator : public configuration::TFConfigurator {

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
         * @brief Get the Canny low threshold
         *
         * @return Canny low threshold
         */
        const int canny_low_threshold() const;

        /**
         * @brief Get the Canny ratio
         *
         * @return Canny ratio
         */
        const int canny_ratio() const;

        /**
         * @brief Get the Canny kernel size
         *
         * @return Canny kernel size
         */
        const int canny_kernel_size() const;

        /**
         * @brief Get the number of lines to include
         *
         * @return Number of lines to include
         */
        const int n_lines_include() const;

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
