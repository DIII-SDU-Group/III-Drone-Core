#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <memory>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/msg/state.hpp>
#include <iii_drone_interfaces/msg/reference.hpp>
#include <iii_drone_interfaces/msg/reference_trajectory.hpp>
              
#include <iii_drone_interfaces/srv/compute_reference_trajectory.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/types.hpp>
#include <iii_drone_core/utils/atomic.hpp>
#include <iii_drone_core/utils/history.hpp>

#include <iii_drone_core/adapters/state_adapter.hpp>
#include <iii_drone_core/adapters/reference_adapter.hpp>
#include <iii_drone_core/adapters/reference_trajectory_adapter.hpp>

#include <iii_drone_core/control/mpc_mode.hpp>
#include <iii_drone_core/control/state.hpp>
#include <iii_drone_core/control/reference.hpp>
#include <iii_drone_core/control/reference_trajectory.hpp>

#include <iii_drone_core/configuration/parameter_bundle.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace control {

    /**
     * @brief Trajectory generator client class. 
     * Exposes API to generate reference trajectories by invoking the trajectory generator service.
    */
    class TrajectoryGeneratorClient {
    public:
        /**
         * @brief Constructor.
         * 
         * @param node Node pointer
         * @param parameters Trajectory generator client parameter bundle
         */
        TrajectoryGeneratorClient(
            rclcpp::Node * node,
            iii_drone::configuration::ParameterBundle::SharedPtr parameters
        );

        /**
         * @brief Resets the client with the current state.
         * 
         * @param state Current state
         */
        void Reset(const iii_drone::control::State & state = iii_drone::control::State());

        /**
         * @brief Cancels the trajectory generation if it is active.
         */
        void Cancel();

        /**
         * @brief Computes a reference according to the settings.
         * 
         * @param state Current state
         * @param reference Reference
         * @param set_reference Updates the internal MPC reference if true
         * @param reset Resets the internal MPC if true
         * @param mpc_mode The MPC mode
         * 
         * @return reference
         * 
         * @throws std::runtime_error if the trajectory generator is busy
         */
        iii_drone::control::Reference ComputeReference(
            const iii_drone::control::State & state,
            const iii_drone::control::Reference & reference,
            bool set_reference,
            bool reset,
            MPC_mode_t mpc_mode
        );

        /**
         * @brief Starts generating a trajectory.
         * 
         * @param state Current state
         * @param reference Reference
         * @param set_reference Updates the internal MPC reference if true
         * @param reset Resets the internal MPC if true
         * @param mpc_mode The MPC mode
         * 
         * @throws std::runtime_error if the trajectory generator is busy
         */
        void ComputeReferenceTrajectoryAsync(
            const iii_drone::control::State & state,
            const iii_drone::control::Reference & reference,
            bool set_reference,
            bool reset,
            MPC_mode_t mpc_mode
        );

        /**
         * @brief Generates a trajectory and blocks until it is done.
         * 
         * @param state Current state
         * @param reference Reference
         * @param set_reference Updates the internal MPC reference if true
         * @param reset Resets the internal MPC if true
         * @param mpc_mode The MPC mode
         * @param poll_period_ms Poll period in milliseconds
         * 
         * @throws std::runtime_error if the trajectory generator is busy
         */
        void ComputeReferenceTrajectoryBlocking(
            const iii_drone::control::State & state,
            const iii_drone::control::Reference & reference,
            bool set_reference,
            bool reset,
            MPC_mode_t mpc_mode,
            unsigned int poll_period_ms
        );

        /**
         * @brief Returns the reference trajectory.
         * 
         * @return reference trajectory
         * 
         * @throws std::runtime_error if no reference trajectory has been generated
         */
        iii_drone::control::ReferenceTrajectory GetReferenceTrajectory() const;

        /**
         * @brief Returns true if the trajectory generator is busy.
         * 
         * @return true if the trajectory generator is busy
         */
        bool busy() const;

        /**
         * @brief Returns true if the trajectory generator is done.
         * 
         * @return true if the trajectory generator is done
         */
        bool done() const;

        /**
         * @brief Shared pointer type.
         */
        typedef std::shared_ptr<TrajectoryGeneratorClient> SharedPtr;

    private:
        /**
         * @brief Node pointer
         */
        rclcpp::Node * node_;

        /**
         * @brief Trajectory generator client parameters
         */
        iii_drone::configuration::ParameterBundle::SharedPtr parameters_;

        /**
         * @brief Service client
         */
        rclcpp::Client<iii_drone_interfaces::srv::ComputeReferenceTrajectory>::SharedPtr client_;

        /**
         * @brief Client callback group
         */
        rclcpp::CallbackGroup::SharedPtr callback_group_;

        /**
         * @brief Reference trajectory adapter history
         */
        iii_drone::utils::History<iii_drone::adapters::ReferenceTrajectoryAdapter>::SharedPtr reference_trajectory_adapter_history_;

        /**
         * @brief Busy flag
         */
        iii_drone::utils::Atomic<bool> busy_;

        /**
         * @brief Done flag
         */
        iii_drone::utils::Atomic<bool> done_;

        /**
         * @brief Service future
         */
        rclcpp::Client<iii_drone_interfaces::srv::ComputeReferenceTrajectory>::SharedFuture future_;

        /**
         * @brief Service result callback.
         * 
         * @param future Future
         */
        void serviceResultCallback(
            const rclcpp::Client<iii_drone_interfaces::srv::ComputeReferenceTrajectory>::SharedFuture future
        );

    };

} // namespace control
} // namespace iii_drone