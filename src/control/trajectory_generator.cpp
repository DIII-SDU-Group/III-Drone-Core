/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/trajectory_generator.hpp>
#include <iostream>

using namespace iii_drone::control;
using namespace iii_drone::types;

namespace {

std::string ResolveMpcPrefix(const iii_drone::configuration::Configuration::SharedPtr & mpc_params)
{
    const std::vector<std::string> prefixes = {
        "position_MPC",
        "cable_landing_MPC",
        "cable_takeoff_MPC",
    };

    for (const auto & prefix : prefixes) {
        const std::string candidate = "/control/trajectory_generator/" + prefix + "_vx_max";
        if (mpc_params->HasParameter(candidate)) {
            return prefix;
        }
    }

    throw std::runtime_error("TrajectoryGenerator: could not resolve MPC parameter prefix.");
}

std::string ResolveMpcParameterName(
    const iii_drone::configuration::Configuration::SharedPtr & mpc_params,
    const std::string & suffix
)
{
    return "/control/trajectory_generator/" + ResolveMpcPrefix(mpc_params) + "_" + suffix;
}

}  // namespace

/*****************************************************************************/
// Implementation
/*****************************************************************************/

TrajectoryGenerator::TrajectoryGenerator(
    iii_drone::configuration::Configuration::SharedPtr positional_mpc_params,
    iii_drone::configuration::Configuration::SharedPtr cable_landing_mpc_params,
    iii_drone::configuration::Configuration::SharedPtr cable_takeoff_mpc_params,
    rclcpp_lifecycle::LifecycleNode * node
) : positional_mpc_params_(positional_mpc_params),
    cable_landing_mpc_params_(cable_landing_mpc_params),
    cable_takeoff_mpc_params_(cable_takeoff_mpc_params),
    node_(node) { }

ReferenceTrajectory TrajectoryGenerator::ComputeReferenceTrajectory(
    State state, 
    Reference reference, 
    bool set_target, 
    bool reset, 
    trajectory_mode_t trajectory_mode
) {

	// Variables:

    iii_drone::configuration::Configuration::SharedPtr mpc_params;

    switch(trajectory_mode) {
        case trajectory_mode_t::positional:
            mpc_params = positional_mpc_params_;
            break;
        case trajectory_mode_t::cable_landing:
            mpc_params = cable_landing_mpc_params_;
            break;
        case trajectory_mode_t::cable_takeoff:
            mpc_params = cable_takeoff_mpc_params_;
            break;
    }

	static bool first = true;

	static int MPC_N = mpc_params->GetParameter("/control/trajectory_generator/MPC_N").as_int();
    static double dt = mpc_params->GetParameter("/control/dt").as_double();

	static bool use_state_feedback = mpc_params->GetParameter("/control/trajectory_generator/MPC_use_state_feedback").as_bool();

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

    if (trajectory_mode == trajectory_mode_t::cable_landing) {

        // RCLCPP_DEBUG(
        //     node_->get_logger(),
        //     "TrajectoryGenerator::ComputeReferenceTrajectory(): Reference yaw: %f",
        //     reference.yaw()
        // );

    }

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

    // RCLCPP_DEBUG(
    //     node_->get_logger(),
    //     "Reference yaw: %f",
    //     reference.yaw()
    // );

    // RCLCPP_DEBUG(
    //     node_->get_logger(),
    //     "Ref yaw: %f",
    //     ref.yaw()
    // );

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
    ReferenceTrajectory ref_traj = postProcessMPC(
        t0,
        MPC_N,
        dt,
        planned_traj,
        planned_u_traj,
        ref.yaw()
    );

    Reference ref_out = ref_traj.references()[0];

    // RCLCPP_DEBUG(
    //     node_->get_logger(), 
    //     "TrajectoryGenerator::ComputeReferenceTrajectory(): Reference computed:\n%f, %f, %f\n%f\n%f, %f, %f\n%f\n%f, %f, %f",
    //     ref_out.position()[0],
    //     ref_out.position()[1],
    //     ref_out.position()[2],
    //     ref_out.yaw(),
    //     ref_out.velocity()[0],
    //     ref_out.velocity()[1],
    //     ref_out.velocity()[2],
    //     ref_out.yaw_rate(),
    //     ref_out.acceleration()[0],
    //     ref_out.acceleration()[1],
    //     ref_out.acceleration()[2]
    // );

    first = false;

    return ref_traj;

}

void TrajectoryGenerator::setupMPC(
    iii_drone::configuration::Configuration::SharedPtr mpc_params,
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

		use_state_feedback = mpc_params->GetParameter("/control/trajectory_generator/MPC_use_state_feedback").as_bool();

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

        RCLCPP_DEBUG(
            node_->get_logger(), 
            "TrajectoryGenerator::setupMPC(): First or reset, with first: %d, reset: %d.",
            first,
            reset
        );

        point_t vehicle_position = state.position();

		for (int i = 0; i < mpc_params->GetParameter("/control/trajectory_generator/MPC_N").as_int(); i++) {
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

        RCLCPP_DEBUG(
            node_->get_logger(), 
            "TrajectoryGenerator::setupMPC(): Using state feedback."
        );

        point_t vehicle_position = state.position();
        vector_t vehicle_velocity = state.velocity();

		for (int i = 0; i < 3; i++) {

            x[i] = vehicle_position[i];
            x[i+3] = vehicle_velocity[i];

        }
	}

    // // Print all info:
    // RCLCPP_DEBUG(
    //     node_->get_logger(), 
    //     "Set target: %d",
    //     set_target
    // );

    // RCLCPP_DEBUG(
    //     node_->get_logger(), 
    //     "First: %d",
    //     first
    // );

    // RCLCPP_DEBUG(
    //     node_->get_logger(), 
    //     "Reset: %d",
    //     reset
    // );

    // RCLCPP_DEBUG(
    //     node_->get_logger(), 
    //     "Reset target: %d",
    //     reset_target
    // );

    // RCLCPP_DEBUG(
    //     node_->get_logger(), 
    //     "Reset trajectory: %d",
    //     reset_trajectory
    // );

    // RCLCPP_DEBUG(
    //     node_->get_logger(), 
    //     "Reset bounds: %d",
    //     reset_bounds
    // );

    // RCLCPP_DEBUG(
    //     node_->get_logger(), 
    //     "Reset weights: %d",
    //     reset_weights
    // );

    // RCLCPP_DEBUG(
    //     node_->get_logger(), 
    //     "Use state feedback: %d",
    //     use_state_feedback
    // );

    // RCLCPP_DEBUG(
    //     node_->get_logger(), 
    //     "State: %f, %f, %f, %f, %f, %f, %f, %f",
    //     state.position()[0],
    //     state.position()[1],
    //     state.position()[2],
    //     state.yaw(),
    //     state.velocity()[0],
    //     state.velocity()[1],
    //     state.velocity()[2],
    //     state.angular_velocity()[2]
    // );

    // RCLCPP_DEBUG(
    //     node_->get_logger(), 
    //     "Previous state: %f, %f, %f, %f, %f, %f, %f, %f",
    //     prev_state.position()[0],
    //     prev_state.position()[1],
    //     prev_state.position()[2],
    //     prev_state.yaw(),
    //     prev_state.velocity()[0],
    //     prev_state.velocity()[1],
    //     prev_state.velocity()[2],
    //     prev_state.angular_velocity()[2]
    // );

    // RCLCPP_DEBUG(
    //     node_->get_logger(), 
    //     "Reference: %f, %f, %f, %f, %f, %f, %f, %f",
    //     reference.position()[0],
    //     reference.position()[1],
    //     reference.position()[2],
    //     reference.yaw(),
    //     reference.velocity()[0],
    //     reference.velocity()[1],
    //     reference.velocity()[2],
    //     reference.yaw_rate()
    // );

    // RCLCPP_DEBUG(
    //     node_->get_logger(), 
    //     "Reference internal: %f, %f, %f, %f, %f, %f, %f, %f",
    //     reference_internal.position()[0],
    //     reference_internal.position()[1],
    //     reference_internal.position()[2],
    //     reference_internal.yaw(),
    //     reference_internal.velocity()[0],
    //     reference_internal.velocity()[1],
    //     reference_internal.velocity()[2],
    //     reference_internal.yaw_rate()
    // );

    // RCLCPP_DEBUG(
    //     node_->get_logger(), 
    //     "Planned trajectory: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f",
    //     planned_traj[0],
    //     planned_traj[1],
    //     planned_traj[2],
    //     planned_traj[3],
    //     planned_traj[4],
    //     planned_traj[5],
    //     planned_traj[6],
    //     planned_traj[7],
    //     planned_traj[8],
    //     planned_traj[9]
    // );

    // RCLCPP_DEBUG(
    //     node_->get_logger(), 
    //     "Planned u trajectory: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f",
    //     planned_u_traj[0],
    //     planned_u_traj[1],
    //     planned_u_traj[2],
    //     planned_u_traj[3],
    //     planned_u_traj[4],
    //     planned_u_traj[5],
    //     planned_u_traj[6],
    //     planned_u_traj[7],
    //     planned_u_traj[8],
    //     planned_u_traj[9]
    // );

    // RCLCPP_DEBUG(
    //     node_->get_logger(), 
    //     "X: %f, %f, %f, %f, %f, %f",
    //     x[0],
    //     x[1],
    //     x[2],
    //     x[3],
    //     x[4],
    //     x[5]
    // );

    // RCLCPP_DEBUG(
    //     node_->get_logger(), 
    //     "Target: %f, %f, %f, %f, %f, %f",
    //     target[0],
    //     target[1],
    //     target[2],
    //     target[3],
    //     target[4],
    //     target[5]
    // );


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
    iii_drone::configuration::Configuration::SharedPtr mpc_parameters
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
            mpc_parameters->GetParameter(ResolveMpcParameterName(mpc_parameters, "ax_max")).as_double(),
            mpc_parameters->GetParameter(ResolveMpcParameterName(mpc_parameters, "ay_max")).as_double(),
            mpc_parameters->GetParameter(ResolveMpcParameterName(mpc_parameters, "az_max")).as_double()
        );
		vector_t v_max(
            mpc_parameters->GetParameter(ResolveMpcParameterName(mpc_parameters, "vx_max")).as_double(),
            mpc_parameters->GetParameter(ResolveMpcParameterName(mpc_parameters, "vy_max")).as_double(),
            mpc_parameters->GetParameter(ResolveMpcParameterName(mpc_parameters, "vz_max")).as_double()
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
            mpc_parameters->GetParameter(ResolveMpcParameterName(mpc_parameters, "wx")).as_double(),
            mpc_parameters->GetParameter(ResolveMpcParameterName(mpc_parameters, "wy")).as_double(),
            mpc_parameters->GetParameter(ResolveMpcParameterName(mpc_parameters, "wz")).as_double()
        );
		vector_t wv(
            mpc_parameters->GetParameter(ResolveMpcParameterName(mpc_parameters, "wvx")).as_double(),
            mpc_parameters->GetParameter(ResolveMpcParameterName(mpc_parameters, "wvy")).as_double(),
            mpc_parameters->GetParameter(ResolveMpcParameterName(mpc_parameters, "wvz")).as_double()
        );
		vector_t wa(
            mpc_parameters->GetParameter(ResolveMpcParameterName(mpc_parameters, "wax")).as_double(),
            mpc_parameters->GetParameter(ResolveMpcParameterName(mpc_parameters, "way")).as_double(),
            mpc_parameters->GetParameter(ResolveMpcParameterName(mpc_parameters, "waz")).as_double()
        );
		vector_t wj(
            mpc_parameters->GetParameter(ResolveMpcParameterName(mpc_parameters, "wjx")).as_double(),
            mpc_parameters->GetParameter(ResolveMpcParameterName(mpc_parameters, "wjy")).as_double(),
            mpc_parameters->GetParameter(ResolveMpcParameterName(mpc_parameters, "wjz")).as_double()
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
