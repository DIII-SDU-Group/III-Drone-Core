#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/configuration/configurator.hpp>

#include <iii_drone_core/control/mpc_parameters.hpp>

#include <iii_drone_core/utils/types.hpp>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {

namespace control {

namespace trajectory_generator_node {

    /**
     * @brief Class for handling parameters for TrajectoryGenerator.
    */
    class TrajectoryGeneratorNodeConfigurator : public configuration::Configurator {

    public:
        /**
         * @brief Constructor
         *
         * @param node Reference to the handling node
         * @param after_parameter_change_callback Callback function called after successful parameter change, default is nullptr
         */
        TrajectoryGeneratorNodeConfigurator(
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
        TrajectoryGeneratorNodeConfigurator(
            rclcpp::Node *node, 
            const rclcpp::QoS & qos,
            std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback = nullptr
        );

        /**
         * @brief MPC Use state feedback getter.
         */
        bool MPC_use_state_feedback() const;

        /**
         * @brief MPC horizon N getter.
         */
        int MPC_N() const;

        /**
         * @brief MPC dt getter.
         */
        double dt() const;

        /**
         * @brief World frame id getter.
         */
        std::string world_frame_id() const;

        /**
         * @brief Drone frame id getter.
         */
        std::string drone_frame_id() const;

        /**
         * @brief Positional mpc parameters getter.
         */
        const std::shared_ptr<iii_drone::control::MPCParameters> positional_mpc_parameters() const;

        /**
         * @brief Cable landing mpc parameters getter.
         */
        const std::shared_ptr<iii_drone::control::MPCParameters> cable_landing_mpc_parameters() const;

        /**
         * @brief Cable takeoff mpc parameters getter.
         */
        const std::shared_ptr<iii_drone::control::MPCParameters> cable_takeoff_mpc_parameters() const;

    private:
        /**
         * @brief Declares the MPC specific parameters
         * 
         * @return void
         */
        void declareMPCParameters();

        /**
         * @brief Initializes the MPC parameters objects.
         * 
         * @return void
         */
        void initMPCParametersObjects();

        /**
         * @brief Callback for updating the MPC parameters objects on parameters change.
         * 
         * @param parameter The parameter that has changed.
         * 
         * @return void
         */
        void updateMPCParametersObjectsCallback(const rclcpp::Parameter &parameter);

        /**
         * @brief Positional MPC parameters.
         */
        std::shared_ptr<iii_drone::control::MPCParameters> positional_mpc_parameters_;

        /**
         * @brief Cable landing MPC parameters.
         */
        std::shared_ptr<iii_drone::control::MPCParameters> cable_landing_mpc_parameters_;

        /**
         * @brief Cable takeoff MPC parameters.
         */
        std::shared_ptr<iii_drone::control::MPCParameters> cable_takeoff_mpc_parameters_;

        /**
         * @brief Positional mpc param name prefix.
         */
        const std::string positional_mpc_parameter_name_prefix = "/control/MPC/position_MPC_";

        /**
         * @brief Cable landing mpc param name prefix.
         */
        const std::string cable_landing_mpc_parameter_name_prefix = "/control/MPC/cable_landing_MPC_";

        /**
         * @brief Cable takeoff mpc param name prefix.
         */
        const std::string cable_takeoff_mpc_parameter_name_prefix = "/control/MPC/cable_takeoff_MPC_";

        /**
         * @brief MPC parameters intiialized flag.
         */
        bool mpc_parameters_initialized_;

        /**
         * @brief After parameter name change callback
         */
        std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback_;

    };


} // namespace trajectory_generator_node
} // namespace control
} // namespace iii_drone