#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// III-Drone-Configuration:

#include <iii_drone_configuration/parameter_bundle.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/types.hpp>
#include <iii_drone_core/utils/math.hpp>

#include <iii_drone_core/control/state.hpp>
#include <iii_drone_core/control/reference.hpp>
#include <iii_drone_core/control/reference_trajectory.hpp>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

/*****************************************************************************/
// MATLAB MPC generated code:

#include <matlab_MPC_5Hz_hp10/mpcmoveCodeGeneration.h>
#include <matlab_MPC_5Hz_hp10/mpcmoveCodeGeneration_terminate.h>
#include <matlab_MPC_5Hz_hp10/mpcmoveCodeGeneration_types.h>
#include <matlab_MPC_5Hz_hp10/rt_nonfinite.h>

/*****************************************************************************/
// Std:

#include <vector>
#include <memory>

/*****************************************************************************/
// Defines
/*****************************************************************************/

namespace iii_drone {
namespace control {

	/**
	 * @brief The supported MPC modes
	 */
	enum MPC_mode_t {
		positional = 0,
		cable_landing = 1,
		cable_takeoff = 2
	};

}
}

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace control {

    class TrajectoryGenerator {
    public:
        /**
         * @brief Constructor.
         * 
         * @param positional_mpc_params The positional MPC parameters
         * @param cable_landing_mpc_params The cable landing MPC parameters
         * @param cable_takeoff_mpc_params The cable takeoff MPC parameters
         */
        TrajectoryGenerator(
            iii_drone::configuration::ParameterBundle::SharedPtr positional_mpc_params, 
            iii_drone::configuration::ParameterBundle::SharedPtr cable_landing_mpc_params, 
            iii_drone::configuration::ParameterBundle::SharedPtr cable_takeoff_mpc_params
        );

		/**
		 * @brief Computes the reference trajectory for the MPC.
		 * 
		 * @param state The current vehicle state
		 * @param reference The reference state
		 * @param set_target Whether to update the target state
		 * @param reset Whether to reset the MPC
		 * @param mpc_mode The MPC mode
		 * 
		 * @return The MPC reference trajectory
		*/
		ReferenceTrajectory ComputeReferenceTrajectory(
			State state, 
			Reference reference, 
			bool set_target, 
			bool reset, 
			MPC_mode_t mpc_mode
		);

    private:
        /**
         * @brief The MPC parameters
         */
        iii_drone::configuration::ParameterBundle::SharedPtr positional_mpc_params_, cable_landing_mpc_params_, cable_takeoff_mpc_params_;

        /**
         * @brief Sets up the MPC.
         * 
         * @param mpc_params The MPC parameters
         * @param set_target Whether to update the target state
         * @param first Whether this is the first MPC call
         * @param reset Whether to reset the MPC
         * @param state The current vehicle state
         * @param prev_state The previous vehicle state
         * @param reference The reference state
         * @param reference_internal The internal reference state
         * @param reset_target Whether to reset the MPC target state
         * @param reset_trajectory Whether to reset the MPC trajectory
         * @param reset_bounds Whether to reset the MPC bounds
         * @param reset_weights Whether to reset the MPC weights
         * @param use_state_feedback Whether to use state feedback
         * @param planned_traj The MPC planned trajectory
         * @param planned_u_traj The MPC planned control input trajectory
         * @param x The MPC state
         * @param target The MPC target state
         * 
         * @return void
        */
        void setupMPC(
            iii_drone::configuration::ParameterBundle::SharedPtr mpc_params,
            bool set_target,
            bool first,
            bool reset,
            iii_drone::control::State &state,
            iii_drone::control::State &prev_state,
            iii_drone::control::Reference &reference,
            iii_drone::control::Reference &reference_internal,
            int &reset_target,
            int &reset_trajectory,
            int &reset_bounds,
            int &reset_weights,
            bool &use_state_feedback,
            double planned_traj[120],
            double planned_u_traj[30],
            double x[6],
            double target[6]
        );

		/**
		 * @brief Runs the actual MPC one step.
		 * 
		 * @param x The MPC state
		 * @param u The MPC control input
		 * @param planned_traj The MPC planned trajectory
         * @param planned_u_traj The MPC planned control input trajectory
		 * @param target The MPC target state
		 * @param reset_target Whether to reset the MPC target state
		 * @param reset_trajectory Whether to reset the MPC trajectory
		 * @param reset_bounds Whether to reset the MPC bounds
		 * @param reset_weights Whether to reset the MPC weights
		 * @param mpc_parameters The MPC parameters
		 * 
		 * @return void
		*/
		void stepMPC(
			double *x, 
			double *u, 
			double *planned_traj,
            double *planned_u_traj,
			double *target, 
			int reset_target, 
			int reset_trajectory, 
			int reset_bounds, 
			int reset_weights, 
            iii_drone::configuration::ParameterBundle::SharedPtr mpc_parameters
		);

        /**
         * @brief Post processes the MPC results.
         * 
         * @param t0 The MPC start time
         * @param N The MPC horizon
         * @param dt The MPC timestep
         * @param u The MPC control input
         * @param planned_traj The MPC planned trajectory
         * @param planned_u_traj The MPC planned control input trajectory
         * @param target_yaw The MPC target yaw
         * 
         * @return The MPC reference trajectory
         */
        ReferenceTrajectory postProcessMPC(
            rclcpp::Time t0,
            int N,
            double dt,
            double *u, 
            double *planned_traj,
            double *planned_u_traj,
            double target_yaw
        );

    };

} // namespace control
} // namespace iii_drone