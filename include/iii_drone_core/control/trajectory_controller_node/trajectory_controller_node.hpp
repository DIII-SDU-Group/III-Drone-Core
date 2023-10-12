#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <stdint.h>
#include <functional>
#include <memory>
#include <thread>
#include <climits>
#include <math.h>
#include <chrono>
#include <iostream>
#include <queue>
#include <vector>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/position_setpoint.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/home_position.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <std_msgs/msg/int16.hpp>

#include "iii_drone_interfaces/msg/control_state.hpp"
#include "iii_drone_interfaces/msg/powerline.hpp"
#include "iii_drone_interfaces/msg/gripper_status.hpp"
              
#include "iii_drone_interfaces/srv/set_general_target_yaw.hpp"
              
#include "iii_drone_interfaces/action/takeoff.hpp"
#include "iii_drone_interfaces/action/landing.hpp"
#include "iii_drone_interfaces/action/fly_to_position.hpp"
#include "iii_drone_interfaces/action/cable_landing.hpp"
#include "iii_drone_interfaces/action/cable_takeoff.hpp"
#include "iii_drone_interfaces/action/fly_under_cable.hpp"
#include "iii_drone_interfaces/action/disarm_on_cable.hpp"
#include "iii_drone_interfaces/action/arm_on_cable.hpp"

#include "iii_drone_core/utils/blocking_queue.hpp"
#include "iii_drone_core/utils/math.hpp"
#include "iii_drone_core/utils/types.hpp"

#include "mpcmoveCodeGeneration.h"
#include "mpcmoveCodeGeneration_terminate.h"
#include "mpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"

/*****************************************************************************/
// Defines
/*****************************************************************************/

namespace iii_drone {
namespace control {
namespace trajectory_controller_node {

	typedef Eigen::Matrix<float, 3, 1> pos3_t;
	typedef Eigen::Matrix<float, 4, 1> pos4_t;
	typedef Eigen::Matrix<float, 6, 1> state3_t;
	typedef Eigen::Matrix<float, 12, 1> state4_t;

	enum state_t {
		init = 0,
		on_ground_non_offboard,
		in_flight_non_offboard,
		arming,
		setting_offboard,
		taking_off,
		hovering,
		landing,
		in_positional_flight,
		during_cable_landing,
		on_cable_armed,
		during_cable_takeoff,
		hovering_under_cable,
		disarming_on_cable,
		on_cable_disarmed,
		arming_on_cable,
		setting_offboard_on_cable
	};

	enum request_type_t {
		cancel_request,
		takeoff_request,
		landing_request,
		fly_to_position_request,
		cable_landing_request,
		cable_takeoff_request,
		fly_under_cable_request,
		disarm_on_cable_request,
		arm_on_cable_request
	};

	struct takeoff_request_params_t {
		float takeoff_altitude;
	};

	struct fly_to_position_request_params_t {
		pos4_t target_position;
	};

	struct cable_landing_request_params_t {
		int cable_id;
	};

	struct cable_takeoff_request_params_t {
		float target_cable_distance;
	};

	struct fly_under_cable_request_params_t {
		int cable_id;
		float target_cable_distance;
	};

	struct request_t {
		rclcpp_action::GoalUUID action_id;
		request_type_t request_type;
		void *request_params;


		bool operator==(const request_t & rhs) const {

			return action_id == rhs.action_id && request_type == rhs.request_type && 
				request_params == rhs.request_params;

		}

		void operator=(const request_t & rhs) {

			action_id = rhs.action_id;
			request_type = rhs.request_type;
			request_params = rhs.request_params;

		}
	};

	enum request_reply_type_t {
		accept,
		reject,
		success,
		fail,
		cancel
	};

	struct request_reply_t {
		rclcpp_action::GoalUUID action_id;
		request_reply_type_t reply_type;

		bool operator==(const request_reply_t & rhs) const {

			return action_id == rhs.action_id && reply_type == rhs.reply_type;

		}

		void operator=(const request_reply_t & rhs) {

			action_id = rhs.action_id;
			reply_type = rhs.reply_type;

		}
	};

	enum request_queue_action_t {
		yes,
		no,
		if_match
	};

	enum MPC_mode_t {
		positional,
		cable_landing,
		cable_takeoff
	};

	struct MPC_parameters_t {
		double dt;
		
		double vx_max;
		double vy_max;
		double vz_max;

		double ax_max;
		double ay_max;
		double az_max;

		double wx;
		double wy;
		double wz;

		double wvx;
		double wvy;
		double wvz;

		double wax;
		double way;
		double waz;

		double wjx;
		double wjy;
		double wjz;
	};

} // namespace trajectory_controller_node
} // namespace control
} // namespace iii_drone

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace control {
namespace trajectory_controller_node {

	class TrajectoryController : public rclcpp::Node {
	public:
		using Takeoff = iii_drone_interfaces::action::Takeoff;
		using GoalHandleTakeoff = rclcpp_action::ServerGoalHandle<Takeoff>;

		using Landing = iii_drone_interfaces::action::Landing;
		using GoalHandleLanding = rclcpp_action::ServerGoalHandle<Landing>;

		using FlyToPosition = iii_drone_interfaces::action::FlyToPosition;
		using GoalHandleFlyToPosition = rclcpp_action::ServerGoalHandle<FlyToPosition>;

		using FlyUnderCable = iii_drone_interfaces::action::FlyUnderCable;
		using GoalHandleFlyUnderCable = rclcpp_action::ServerGoalHandle<FlyUnderCable>;

		using CableLanding = iii_drone_interfaces::action::CableLanding;
		using GoalHandleCableLanding = rclcpp_action::ServerGoalHandle<CableLanding>;

		using CableTakeoff = iii_drone_interfaces::action::CableTakeoff;
		using GoalHandleCableTakeoff = rclcpp_action::ServerGoalHandle<CableTakeoff>;

		using DisarmOnCable = iii_drone_interfaces::action::DisarmOnCable;
		using GoalHandleDisarmOnCable = rclcpp_action::ServerGoalHandle<DisarmOnCable>;

		using ArmOnCable = iii_drone_interfaces::action::ArmOnCable;
		using GoalHandleArmOnCable = rclcpp_action::ServerGoalHandle<ArmOnCable>;

		TrajectoryController(const std::string & node_name="trajectory_controller", 
				const std::string & node_namespace="/trajectory_controller", 
				const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
		~TrajectoryController();

	private:
		// Takeoff action:
		rclcpp_action::Server<Takeoff>::SharedPtr takeoff_server_;

		rclcpp_action::GoalResponse handleGoalTakeoff(
			const rclcpp_action::GoalUUID & uuid, 
			std::shared_ptr<const Takeoff::Goal> goal
		);
		rclcpp_action::CancelResponse handleCancelTakeoff(const std::shared_ptr<GoalHandleTakeoff> goal_handle);
		void handleAcceptedTakeoff(const std::shared_ptr<GoalHandleTakeoff> goal_handle);
		void followTakeoffCompletion(const std::shared_ptr<GoalHandleTakeoff> goal_handle);

		// Landing action:
		rclcpp_action::Server<Landing>::SharedPtr landing_server_;

		rclcpp_action::GoalResponse handleGoalLanding(
			const rclcpp_action::GoalUUID & uuid, 
			std::shared_ptr<const Landing::Goal> goal
		);
		rclcpp_action::CancelResponse handleCancelLanding(const std::shared_ptr<GoalHandleLanding> goal_handle);
		void handleAcceptedLanding(const std::shared_ptr<GoalHandleLanding> goal_handle);
		void followLandingCompletion(const std::shared_ptr<GoalHandleLanding> goal_handle);

		// Fly to position action:
		rclcpp_action::Server<FlyToPosition>::SharedPtr fly_to_position_server_;

		rclcpp_action::GoalResponse handleGoalFlyToPosition(
			const rclcpp_action::GoalUUID & uuid, 
			std::shared_ptr<const FlyToPosition::Goal> goal
		);
		rclcpp_action::CancelResponse handleCancelFlyToPosition(const std::shared_ptr<GoalHandleFlyToPosition> goal_handle);
		void handleAcceptedFlyToPosition(const std::shared_ptr<GoalHandleFlyToPosition> goal_handle);
		void followFlyToPositionCompletion(const std::shared_ptr<GoalHandleFlyToPosition> goal_handle);

		// Fly under cable action:
		rclcpp_action::Server<FlyUnderCable>::SharedPtr fly_under_cable_server_;

		rclcpp_action::GoalResponse handleGoalFlyUnderCable(
			const rclcpp_action::GoalUUID & uuid, 
			std::shared_ptr<const FlyUnderCable::Goal> goal
		);
		rclcpp_action::CancelResponse handleCancelFlyUnderCable(const std::shared_ptr<GoalHandleFlyUnderCable> goal_handle);
		void handleAcceptedFlyUnderCable(const std::shared_ptr<GoalHandleFlyUnderCable> goal_handle);
		void followFlyUnderCableCompletion(const std::shared_ptr<GoalHandleFlyUnderCable> goal_handle);

		// Cable landing action:
		rclcpp_action::Server<CableLanding>::SharedPtr cable_landing_server_;

		rclcpp_action::GoalResponse handleGoalCableLanding(
			const rclcpp_action::GoalUUID & uuid, 
			std::shared_ptr<const CableLanding::Goal> goal
		);
		rclcpp_action::CancelResponse handleCancelCableLanding(const std::shared_ptr<GoalHandleCableLanding> goal_handle);
		void handleAcceptedCableLanding(const std::shared_ptr<GoalHandleCableLanding> goal_handle);
		void followCableLandingCompletion(const std::shared_ptr<GoalHandleCableLanding> goal_handle);

		// Cable takeoff action:
		rclcpp_action::Server<CableTakeoff>::SharedPtr cable_takeoff_server_;

		rclcpp_action::GoalResponse handleGoalCableTakeoff(
			const rclcpp_action::GoalUUID & uuid, 
			std::shared_ptr<const CableTakeoff::Goal> goal
		);
		rclcpp_action::CancelResponse handleCancelCableTakeoff(const std::shared_ptr<GoalHandleCableTakeoff> goal_handle);
		void handleAcceptedCableTakeoff(const std::shared_ptr<GoalHandleCableTakeoff> goal_handle);
		void followCableTakeoffCompletion(const std::shared_ptr<GoalHandleCableTakeoff> goal_handle);

		// Disarm on cable action:
		rclcpp_action::Server<DisarmOnCable>::SharedPtr disarm_on_cable_server_;

		rclcpp_action::GoalResponse handleGoalDisarmOnCable(
			const rclcpp_action::GoalUUID & uuid,
			std::shared_ptr<const DisarmOnCable::Goal> goal
		);
		rclcpp_action::CancelResponse handleCancelDisarmOnCable(const std::shared_ptr<GoalHandleDisarmOnCable> goal_handle);
		void handleAcceptedDisarmOnCable(const std::shared_ptr<GoalHandleDisarmOnCable> goal_handle);
		void followDisarmOnCableCompletion(const std::shared_ptr<GoalHandleDisarmOnCable> goal_handle);

		// Arm on cable action:
		rclcpp_action::Server<ArmOnCable>::SharedPtr arm_on_cable_server_;

		rclcpp_action::GoalResponse handleGoalArmOnCable(
			const rclcpp_action::GoalUUID & uuid,
			std::shared_ptr<const ArmOnCable::Goal> goal
		);
		rclcpp_action::CancelResponse handleCancelArmOnCable(const std::shared_ptr<GoalHandleArmOnCable> goal_handle);
		void handleAcceptedArmOnCable(const std::shared_ptr<GoalHandleArmOnCable> goal_handle);
		void followArmOnCableCompletion(const std::shared_ptr<GoalHandleArmOnCable> goal_handle);

		// Set yaw service:
		rclcpp::Service<iii_drone_interfaces::srv::SetGeneralTargetYaw>::SharedPtr set_yaw_service_;
		void setYawServiceCallback(const std::shared_ptr<iii_drone_interfaces::srv::SetGeneralTargetYaw::Request> request,
										std::shared_ptr<iii_drone_interfaces::srv::SetGeneralTargetYaw::Response> response);

		// General member variables:
		state_t state_ = init;

		uint8_t arming_state_; // armed = 4
		uint8_t nav_state_; // offboard = 14
		std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

		iii_drone::types::quat_t odom_q_;
		iii_drone::types::vector_t odom_ang_vel_;
		iii_drone::types::vector_t odom_pos_;
		iii_drone::types::vector_t odom_vel_;
		iii_drone::types::vector_t odom_last_vel_;
		iii_drone::types::vector_t odom_acc_;

		float target_yaw_ = 0;

		std::vector<state4_t> planned_trajectory_;
		std::vector<state4_t> planned_macro_trajectory_;
		state4_t trajectory_target_;

		iii_drone_interfaces::msg::Powerline powerline_;
		int target_cable_id_ = -1;
		geometry_msgs::msg::PoseStamped target_cable_pose_;
		float target_cable_distance_ = -1;
		iii_drone::types::plane_t target_cable_plane_;

		std::mutex odometry_mutex_;
		std::mutex planned_trajectory_mutex_;
		std::mutex powerline_mutex_;
		std::mutex target_yaw_mutex_;

		std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
		std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
		std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

		BlockingQueue<request_t> request_queue_;
		BlockingQueue<request_reply_t> request_reply_queue_;

		rclcpp::Rate request_reply_poll_rate_;
		rclcpp::Rate request_completion_poll_rate_;

		rclcpp::TimerBase::SharedPtr main_state_machine_timer_;

		rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
		rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_sub_;
		rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;

		rclcpp::Subscription<iii_drone_interfaces::msg::Powerline>::SharedPtr powerline_sub_;

		rclcpp::Subscription<px4_msgs::msg::HomePosition>::SharedPtr home_position_sub_;

		rclcpp::Subscription<iii_drone_interfaces::msg::GripperStatus>::SharedPtr gripper_status_sub_;

		rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
		rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
		rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr thrust_setpoint_pub_;
		rclcpp::Publisher<px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr torque_setpoint_pub_;

		rclcpp::Publisher<iii_drone_interfaces::msg::ControlState>::SharedPtr control_state_pub_;

		rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;

		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_traj_pub_;
		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_macro_traj_pub_;
		rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr planned_target_pub_;

		rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pose_pub_;

		rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr target_cable_id_pub_;

		std::thread MPC_thread_;
		double MPC_u_[3];
		double MPC_x_[6];
		double MPC_planned_traj_[120];

		int MPC_N_;
		bool MPC_use_state_feedback_;

		px4_msgs::msg::HomePosition home_position_;
		std::mutex home_position_mutex_;
		bool home_position_set_ = false;

		// gripper status member and mutex:
		iii_drone_interfaces::msg::GripperStatus gripper_status_;
		std::mutex gripper_status_mutex_;

		// General member methods:
		void stateMachineCallback();
		void odometryCallback(px4_msgs::msg::VehicleOdometry::SharedPtr msg);
		void powerlineCallback(iii_drone_interfaces::msg::Powerline::SharedPtr msg);
		void homePositionCallback(px4_msgs::msg::HomePosition::SharedPtr msg);

		void gripperStatusCallback(iii_drone_interfaces::msg::GripperStatus::SharedPtr msg);

		void setHomePosition(px4_msgs::msg::HomePosition new_home);
		void setHomePositionIfChanged(px4_msgs::msg::HomePosition old_home, px4_msgs::msg::HomePosition new_home);

		bool isOffboard();
		bool isArmed();

		void setModeOffboard();

		void land();

		void arm();
		void disarm(bool force=false);

		void disarmOnCable();

		double disarm_on_cable_thrust_;
		rclcpp::Time disarm_on_cable_thrust_start_time_;
		bool is_disarming_on_cable_by_thrust_ = false;

		bool is_on_cable_armed_using_thrust_control_ = false;

		void publishVehicleCommand(uint16_t command, float param1 = 0.0,
						float param2 = 0.0,
						float param3 = 0.0,
						float param4 = 0.0,
						float param5 = 0.0,
						float param6 = 0.0,
						float param7 = 0.0);
		void publishOffboardControlMode();
		void publishControlState();
		void publishActuatorSetpoints();
		void publishTargetCableId();
		void publishTrajectorySetpoint(state4_t set_point) ;
		void publishSetpointPose(state4_t set_point);

		void publishPlannedTrajectory();

		void publishGroundAltitudeOffsetTf(state3_t ground_altitude_offset);

		state4_t loadVehicleState();
		geometry_msgs::msg::PoseStamped loadVehiclePose();
		nav_msgs::msg::Path loadPlannedPath();
		nav_msgs::msg::Path loadPlannedMacroPath();
		geometry_msgs::msg::PoseStamped loadPlannedTarget();
		state4_t loadTargetCableState();
		state4_t loadTargetUnderCableState();

		iii_drone::types::vector_t stepCartesianVelocityPID(state4_t vehicle_state, state4_t target_state, bool reset);

		state4_t stepMPC(state4_t vehicle_state, state4_t target_state, bool set_target, bool reset, MPC_mode_t mpc_mode);
		void threadFunctionMPC(double *x, double *u, double *planned_traj, double *target, 
			int reset_target, int reset_trajectory, int reset_bounds, int reset_weights, MPC_mode_t mpc_mode);

		void loadPeriodMPC(MPC_parameters_t &mpc_params, MPC_mode_t mpc_mode);
		void loadBoundsMPC(MPC_parameters_t &mpc_params, MPC_mode_t mpc_mode);
		void loadWeightsMPC(MPC_parameters_t &mpc_params, MPC_mode_t mpc_mode);

		void clearPlannedTrajectory();
		void setTrajectoryTarget(state4_t target);

		bool updateTargetCablePose(state4_t vehicle_state, int new_id = -1);
		void clearTargetCable();

	};

} // namespace trajectory_controller_node
} // namespace control
} // namespace iii_drone

/*****************************************************************************/
// Main
/*****************************************************************************/

int main(int argc, char* argv[]);