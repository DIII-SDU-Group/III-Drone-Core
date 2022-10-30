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
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "iii_interfaces/msg/control_state.hpp"
#include "iii_interfaces/msg/powerline.hpp"

#include "iii_interfaces/srv/set_general_target_yaw.hpp"

#include "iii_interfaces/action/takeoff.hpp"
#include "iii_interfaces/action/landing.hpp"
#include "iii_interfaces/action/fly_to_position.hpp"
#include "iii_interfaces/action/cable_landing.hpp"
#include "iii_interfaces/action/cable_takeoff.hpp"

#include "geometry.h"
#include "blocking_queue.h"

//#include "MPCStepFunction.h"
//#include "PositionMPCStepFunction.h"
//#include "rt_nonfinite.h"
#include "mpcmoveCodeGeneration.h"
#include "mpcmoveCodeGeneration_terminate.h"
#include "mpcmoveCodeGeneration_types.h"
#include "rt_nonfinite.h"

/*****************************************************************************/
// Defines
/*****************************************************************************/

#define ROS_DEFAULT_API

using namespace std::chrono_literals;

#define LOG_INFO(str) RCLCPP_INFO(this->get_logger(),str)

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
	during_cable_takeoff
};

enum request_type_t {
	cancel_request,
	takeoff_request,
	landing_request,
	fly_to_position_request,
	cable_landing_request,
	cable_takeoff_request
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

/*****************************************************************************/
// Class
/*****************************************************************************/

class TrajectoryController : public rclcpp::Node {
public:
	using Takeoff = iii_interfaces::action::Takeoff;
	using GoalHandleTakeoff = rclcpp_action::ServerGoalHandle<Takeoff>;

	using Landing = iii_interfaces::action::Landing;
	using GoalHandleLanding = rclcpp_action::ServerGoalHandle<Landing>;

	using FlyToPosition = iii_interfaces::action::FlyToPosition;
	using GoalHandleFlyToPosition = rclcpp_action::ServerGoalHandle<FlyToPosition>;

	using CableLanding = iii_interfaces::action::CableLanding;
	using GoalHandleCableLanding = rclcpp_action::ServerGoalHandle<CableLanding>;

	using CableTakeoff = iii_interfaces::action::CableTakeoff;
	using GoalHandleCableTakeoff = rclcpp_action::ServerGoalHandle<CableTakeoff>;

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

    // Set yaw service:
    rclcpp::Service<iii_interfaces::srv::SetGeneralTargetYaw>::SharedPtr set_yaw_service_;
    void setYawServiceCallback(const std::shared_ptr<iii_interfaces::srv::SetGeneralTargetYaw::Request> request,
                                    std::shared_ptr<iii_interfaces::srv::SetGeneralTargetYaw::Response> response);

	// General member variables:
	state_t state_ = init;

	uint8_t arming_state_; // armed = 4
	uint8_t nav_state_; // offboard = 14
	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	quat_t odom_q_;
	vector_t odom_ang_vel_;
	vector_t odom_pos_;
	vector_t odom_vel_;
	vector_t odom_last_vel_;
	vector_t odom_acc_;

	float target_yaw_ = 0;

	std::vector<state4_t> planned_trajectory_;
	std::vector<state4_t> planned_macro_trajectory_;
	state4_t trajectory_target_;

	iii_interfaces::msg::Powerline powerline_;
	int target_cable_id_ = -1;
	geometry_msgs::msg::PoseStamped target_cable_pose_;

	std::mutex odometry_mutex_;
	std::mutex planned_trajectory_mutex_;
	std::mutex powerline_mutex_;
	std::mutex target_yaw_mutex_;

    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

	BlockingQueue<request_t> request_queue_;
	BlockingQueue<request_reply_t> request_reply_queue_;

	rclcpp::Rate request_reply_poll_rate_;
	rclcpp::Rate request_completion_poll_rate_;

	rclcpp::TimerBase::SharedPtr main_state_machine_timer_;

	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;

	rclcpp::Subscription<iii_interfaces::msg::Powerline>::SharedPtr powerline_sub_;

	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
	rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;

	rclcpp::Publisher<iii_interfaces::msg::ControlState>::SharedPtr control_state_pub_;

	rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;

	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_traj_pub_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_macro_traj_pub_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr planned_target_pub_;

	std::thread MPC_thread_;
	double MPC_u_[3];
	double MPC_x_[6];
	double MPC_planned_traj_[120];

	// General member methods:
	void stateMachineCallback();
	void odometryCallback(px4_msgs::msg::VehicleOdometry::SharedPtr msg);
	void powerlineCallback(iii_interfaces::msg::Powerline::SharedPtr msg);

	bool isOffboard();
	bool isArmed();

	void setModeOffboard();

	void land();

	void arm();
	void disarm();

	void publishVehicleCommand(uint16_t command, float param1 = 0.0,
					 float param2 = 0.0,
					 float param3 = 0.0,
					 float param4 = 0.0,
					 float param5 = 0.0,
					 float param6 = 0.0,
					 float param7 = 0.0) const;
	void publishOffboardControlMode() const;
	void publishControlState();
	void publishTrajectorySetpoint(state4_t set_point) const;

	void publishPlannedTrajectory();

	state4_t loadVehicleState();
	geometry_msgs::msg::PoseStamped loadVehiclePose();
	nav_msgs::msg::Path loadPlannedPath();
	nav_msgs::msg::Path loadPlannedMacroPath();
	geometry_msgs::msg::PoseStamped loadPlannedTarget();
	state4_t loadTargetCableState();

	state4_t stepMPC(state4_t vehicle_state, state4_t target_state, bool reset, MPC_mode_t mpc_mode);
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



/*****************************************************************************/
// Main
/*****************************************************************************/

int main(int argc, char* argv[]);