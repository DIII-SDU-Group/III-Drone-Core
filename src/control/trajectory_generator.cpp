/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/trajectory_generator.hpp>
#include <iostream>

using namespace iii_drone::control;
using namespace iii_drone::types;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

TrajectoryGenerator::TrajectoryGenerator(
    iii_drone::configuration::ParameterBundle::SharedPtr positional_mpc_params,
    iii_drone::configuration::ParameterBundle::SharedPtr cable_landing_mpc_params,
    iii_drone::configuration::ParameterBundle::SharedPtr cable_takeoff_mpc_params
) : positional_mpc_params_(positional_mpc_params),
    cable_landing_mpc_params_(cable_landing_mpc_params),
    cable_takeoff_mpc_params_(cable_takeoff_mpc_params) { }

ReferenceTrajectory TrajectoryGenerator::ComputeReferenceTrajectory(
    State state, 
    Reference reference, 
    bool set_target, 
    bool reset, 
    MPC_mode_t mpc_mode
) {

	// Variables:

    iii_drone::configuration::ParameterBundle::SharedPtr mpc_params;

    switch(mpc_mode) {
        case MPC_mode_t::positional:
            mpc_params = positional_mpc_params_;
            break;
        case MPC_mode_t::cable_landing:
            mpc_params = cable_landing_mpc_params_;
            break;
        case MPC_mode_t::cable_takeoff:
            mpc_params = cable_takeoff_mpc_params_;
            break;
    }

	static bool first = true;

	static int MPC_N = mpc_params->GetParameter("N").as_int();
    static double dt = mpc_params->GetParameter("dt").as_double();

	static bool use_state_feedback = mpc_params->GetParameter("use_state_feedback").as_bool();

	static double planned_traj[120];
	static double planned_u_traj[30];
	static double x[6];
	static double u[3];

	static double target[6];

	static int reset_target;
	static int reset_trajectory;
	static int reset_bounds;
	static int reset_weights;

	static State prev_state = state;

	static Reference ref;

    rclcpp::Time t0 = state.stamp();

    setupMPC(
        mpc_params,
        set_target,
        first,
        reset,
        state,
        prev_state,
        reference,
        ref,
        reset_target,
        reset_trajectory,
        reset_bounds,
        reset_weights,
        use_state_feedback,
        planned_traj,
        planned_u_traj,
        x,
        target
    );


	prev_state = state;

    // Run MPC:
    stepMPC(
        x, 
        u, 
        planned_traj, 
        planned_u_traj,
        target, 
        reset_target, 
        reset_trajectory, 
        reset_bounds, 
        reset_weights, 
        mpc_params
    );

	// Output:
    return postProcessMPC(
        t0,
        MPC_N,
        dt,
        u, 
        planned_traj,
        planned_u_traj,
        ref.yaw()
    );

}

void TrajectoryGenerator::setupMPC(
    iii_drone::configuration::ParameterBundle::SharedPtr mpc_params,
    bool set_target,
    bool first,
    bool reset,
    State &state,
    State &prev_state,
    Reference &reference,
    Reference &reference_internal,
    int &reset_target,
    int &reset_trajectory,
    int &reset_bounds,
    int &reset_weights,
    bool &use_state_feedback,
    double planned_traj[120],
    double planned_u_traj[30],
    double x[6],
    double target[6]
) {

	if (set_target || first || reset) {

		reference_internal = reference;

	}

	if (first || reset) {

		prev_state = state;

	}

	// Initialization:

	if (reset) {

		reset_trajectory = 1;
		reset_bounds = 1;
		reset_weights = 1;

	} else {

		reset_trajectory = 0;
		reset_bounds = 0;
		reset_weights = 0;

	}

	reset_target = set_target;

	if (reset_trajectory) {

		use_state_feedback = mpc_params->GetParameter("use_state_feedback").as_bool();

	}

	if (set_target) {

        point_t reference_position = reference_internal.position();
        vector_t reference_velocity = reference_internal.velocity();

		for (int i = 0; i < 3; i++) {

			target[i] = reference_position[i];
			target[i+3] = reference_velocity[i];

		}
	}

	if (first || reset) {

        point_t vehicle_position = state.position();

		for (int i = 0; i < mpc_params->GetParameter("N").as_int(); i++) {
			for (int j = 0; j < 3; j++) {

				planned_traj[i*6+j] = vehicle_position[j];
                planned_u_traj[i*3+j] = 0;

			}

			for (int j = 3; j < 6; j++) {

				planned_traj[i*6+j] = 0;

			}
		}

		for (int i = 0; i < 6; i++) x[i] = planned_traj[i];

	}

	// No state progression:
	if (use_state_feedback || reset) {

        point_t vehicle_position = state.position();
        vector_t vehicle_velocity = state.velocity();

		for (int i = 0; i < 3; i++) {

            x[i] = vehicle_position[i];
            x[i+3] = vehicle_velocity[i];

        }
	}

}

void TrajectoryGenerator::stepMPC(
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
) {

	const int N = 10;

	static pos_MPC::struct10_T Info;
	static pos_MPC::struct4_T mpcmovestate;
	static pos_MPC::struct5_T mpconlinedata;

	// if (reset_target && reset_bounds && reset_weights && reset_trajectory) {

	// 	RCLCPP_INFO(this->get_logger(), "Resetting MPC fully");

	// }

	if (reset_trajectory) {

		for(int i = 0; i < 81; i++) mpcmovestate.Covariance[i] = 0;
		for(int i=0; i < 3; i++) mpcmovestate.Disturbance[i] = 0;
		for(int i = 0; i < 6*2*N; i++) mpcmovestate.iA[i] = 0;
		for(int i=0; i < 3; i++) mpcmovestate.LastMove[i] = 0;
		for(int i=0; i < 6; i++) mpcmovestate.Plant[i] = x[i];

	}

	if (reset_bounds) {

		vector_t a_max(
            mpc_parameters->GetParameter("ax_max").as_double(),
            mpc_parameters->GetParameter("ay_max").as_double(),
            mpc_parameters->GetParameter("az_max").as_double()
        );
		vector_t v_max(
            mpc_parameters->GetParameter("vx_max").as_double(),
            mpc_parameters->GetParameter("vy_max").as_double(),
            mpc_parameters->GetParameter("vz_max").as_double()
        );

		mpconlinedata.limits.umax[0] = a_max[0];
		mpconlinedata.limits.umax[1] = a_max[1];
		mpconlinedata.limits.umax[2] = a_max[2];

		mpconlinedata.limits.umin[0] = -a_max[0];
		mpconlinedata.limits.umin[1] = -a_max[1];
		mpconlinedata.limits.umin[2] = -a_max[2];

		mpconlinedata.limits.ymax[0] = 100000;
		mpconlinedata.limits.ymax[1] = 100000;
		mpconlinedata.limits.ymax[2] = 100000;
		mpconlinedata.limits.ymax[3] = v_max[0];
		mpconlinedata.limits.ymax[4] = v_max[1];
		mpconlinedata.limits.ymax[5] = v_max[2];

		mpconlinedata.limits.ymin[0] = -100000;
		mpconlinedata.limits.ymin[1] = -100000;
		mpconlinedata.limits.ymin[2] = -100000;
		mpconlinedata.limits.ymin[3] = -v_max[0];
		mpconlinedata.limits.ymin[4] = -v_max[1];
		mpconlinedata.limits.ymin[5] = -v_max[2];

	}

	if (reset_target) {

		// RCLCPP_DEBUG(this->get_logger(), "MPC resetting Target");
		// RCLCPP_DEBUG(this->get_logger(), "MPC Thread State: %f, %f, %f", x[0], x[1], x[2]);
		// RCLCPP_DEBUG(this->get_logger(), "MPC Thread Target: %f, %f, %f", target[0], target[1], target[2]);

		for (int i = 0; i < 6; i++) {

			mpconlinedata.signals.ref[i] = target[i];

		}

	}
	if (reset_weights) {

		vector_t wp(
            mpc_parameters->GetParameter("wx").as_double(),
            mpc_parameters->GetParameter("wy").as_double(),
            mpc_parameters->GetParameter("wz").as_double()
        );
		vector_t wv(
            mpc_parameters->GetParameter("wvx").as_double(),
            mpc_parameters->GetParameter("wvy").as_double(),
            mpc_parameters->GetParameter("wvz").as_double()
        );
		vector_t wa(
            mpc_parameters->GetParameter("wax").as_double(),
            mpc_parameters->GetParameter("way").as_double(),
            mpc_parameters->GetParameter("waz").as_double()
        );
		vector_t wj(
            mpc_parameters->GetParameter("wjx").as_double(),
            mpc_parameters->GetParameter("wjy").as_double(),
            mpc_parameters->GetParameter("wjz").as_double()
        );

		mpconlinedata.weights.du[0] = wj[0];
		mpconlinedata.weights.du[1] = wj[1];
		mpconlinedata.weights.du[2] = wj[2];

		mpconlinedata.weights.u[0] = wa[0];
		mpconlinedata.weights.u[1] = wa[1];
		mpconlinedata.weights.u[2] = wa[2];

		mpconlinedata.weights.y[0] = wp[0];
		mpconlinedata.weights.y[1] = wp[1];
		mpconlinedata.weights.y[2] = wp[2];
		mpconlinedata.weights.y[3] = wv[0];
		mpconlinedata.weights.y[4] = wv[1];
		mpconlinedata.weights.y[5] = wv[2];

	}

	for(int i = 0; i < 6; i++) mpconlinedata.signals.ym[i] = x[i];

	// if (mpc_mode == cable_takeoff) {
	// 	RCLCPP_DEBUG(this->get_logger(), "MPC Thread Target stored: %f, %f, %f", mpconlinedata.signals.ref[0], mpconlinedata.signals.ref[1], mpconlinedata.signals.ref[2]);
	// 	RCLCPP_DEBUG(this->get_logger(), "MPC Thread State stored: %f, %f, %f", mpconlinedata.signals.ym[0], mpconlinedata.signals.ym[1], mpconlinedata.signals.ym[2]);
	// 	RCLCPP_DEBUG(this->get_logger(), "MPC Thread wxyz stored: %f, %f, %f, %f", mpconlinedata.weights.y[0], mpconlinedata.weights.y[1], mpconlinedata.weights.y[2]);
	// 	RCLCPP_DEBUG(this->get_logger(), "MPC Thread wvxyz stored: %f, %f, %f, %f", mpconlinedata.weights.y[3], mpconlinedata.weights.y[4], mpconlinedata.weights.y[5]);
	// }

	pos_MPC::coder::mpcmoveCodeGeneration(&mpcmovestate, &mpconlinedata, u, &Info);

	for (int j = 0; j < 6; j++) x[j] = Info.Yopt[j*(N+1)];
	// for (int j = 0; j < 6; j++) x[j] = Info.Yopt[j*(N+1)+1];
	for (int i = 0; i < N; i++) {

		for (int j = 0; j < 6; j++) planned_traj[i*6+j] = Info.Yopt[j*(N+1)+(i)];
		// for (int j = 0; j < 6; j++) planned_traj[i*6+j] = Info.Yopt[j*(N+1)+(i+1)];

        for (int j = 0; j < 3; j++) planned_u_traj[i*3+j] = Info.Uopt[j*N+i];

	} 

}

ReferenceTrajectory TrajectoryGenerator::postProcessMPC(
    rclcpp::Time t0,
    int N,
    double dt,
    double *u, 
    double *planned_traj,
    double *planned_u_traj,
    double target_yaw
) {

    std::vector<Reference> trajectory_vector;

    trajectory_vector.resize(N);

    rclcpp::Time stamp = t0;

    for (int i = 0; i < N; i++) {

        point_t reference_position(
            planned_traj[i*6+0],
            planned_traj[i*6+1],
            planned_traj[i*6+2]
        );

        vector_t reference_velocity(
            planned_traj[i*6+3],
            planned_traj[i*6+4],
            planned_traj[i*6+5]
        );

        Reference ref;

        // if (i == 0) {

            // vector_t reference_acceleration(
            //     u[0],
            //     u[1],
            //     u[2]
            // );

            vector_t reference_acceleration(
                planned_u_traj[i*3+0],
                planned_u_traj[i*3+1],
                planned_u_traj[i*3+2]
            );

            ref = Reference(
                reference_position,
                target_yaw,
                reference_velocity,
                NAN,
                reference_acceleration,
                NAN,
                stamp
            );

        // } else {

        //     ref = Reference(
        //         reference_position,
        //         target_yaw,
        //         reference_velocity,
        //         NAN,
        //         vector_t::Constant(NAN),
        //         NAN,
        //         stamp
        //     );
        
        // }

        trajectory_vector[i] = ref;

        stamp = rclcpp::Time(stamp.nanoseconds() + dt*1e9);


    }

    return ReferenceTrajectory(trajectory_vector);

}
