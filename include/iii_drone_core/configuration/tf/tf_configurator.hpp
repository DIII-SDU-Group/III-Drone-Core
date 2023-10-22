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

namespace configuration {

    /**
     * @brief Class for handling TF-related parameters.
    */
    class TFConfigurator : public virtual Configurator {

    public:
        /**
         * @brief Constructor
         *
         * @param node Reference to the handling node
         * @param after_parameter_change_callback Callback function called after successful parameter change, default is nullptr
         */
        TFConfigurator(
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
        TFConfigurator(
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

    private:
        /**
         * @brief Declares the TF related parameters
         * 
         * @return void
         */
        void declareTFParameters();

    };

} // namespace configuration
} // namespace iii_drone