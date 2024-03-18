#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/configuration/configurator.hpp>

#include <iii_drone_core/control/mpc_parameters.hpp>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

/*****************************************************************************/
// Std:

#include <string>
#include <memory>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {

namespace configuration {

    /**
     * @brief Class for handling parameters for a positional MPC.
    */
    class MPCConfigurator : virtual public Configurator {

    public:
        /**
         * @brief Constructor
         *
         * @param node Reference to the handling node
         * @param parameter_prefix Prefix for the MPC parameters
         * @param after_parameter_change_callback Callback function called after successful parameter change, default is nullptr
         */
        MPCConfigurator(
            rclcpp::Node *node,
            std::string parameter_prefix,
            std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback = nullptr
        );

        /**
         * @brief Constructor
         * 
         * @param node Reference to the handling node
         * @param qos QoS profile for the parameter event subscription
         * @param parameter_prefix Prefix for the MPC parameters
         * @param after_parameter_change_callback Callback function called after successful parameter change, default is nullptr
         */
        MPCConfigurator(
            rclcpp::Node *node, 
            const rclcpp::QoS & qos,
            std::string parameter_prefix,
            std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback = nullptr
        );

        /**
         * @brief Get the MPC parameters object
         * 
         * @return MPC parameters object reference
         */
        const std::shared_ptr<control::MPCParameters> mpc_parameters() const;

    private:
        /**
         * @brief MPC parameters object
         */
        std::shared_ptr<control::MPCParameters> mpc_parameters_;

        /**
         * @brief MPCParameters object initialized flag
         */
        bool mpc_parameters_initialized_ = false;

        /**
         * @brief Prefix for the MPC parameters
         */
        std::string parameter_prefix_;

        /**
         * @brief Declares the MPC specific parameters
         * 
         * @return void
         */
        void declareMPCParameters();

        /**
         * @brief Initializes the MPC parameters object
         * 
         * @return void
         */ 
        void initMPCParametersObject();

        /**
         * @brief Callback function called after successful parameter change to update MPCParameters object.
        */
        void updateMPCParametersObjectCallback(const rclcpp::Parameter & parameter);

        /**
         * @brief Callback function reference called after successful parameter change.
         */
        std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback_;

    };

} // namespace configuration
} // namespace iii_drone