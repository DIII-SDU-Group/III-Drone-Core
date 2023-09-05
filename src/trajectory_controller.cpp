/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "trajectory_controller.h"

/*****************************************************************************/
// Implementation
/*****************************************************************************/

TrajectoryController::TrajectoryController(const std::string & node_name, 
						const std::string & node_namespace, const rclcpp::NodeOptions & options) : 
	Node(node_name, node_namespace, options),
	request_queue_(1), 
	request_reply_queue_(2),
	request_reply_poll_rate_(50ms),
	request_completion_poll_rate_(100ms) {

	this->declare_parameter<int>("controller_period_ms", 100);
	this->declare_parameter<float>("landed_altitude_threshold", 0.15);
	this->declare_parameter<bool>("use_ground_altitude_offset", true);
	this->declare_parameter<float>("reached_position_euclidean_distance_threshold", 0.1);
	this->declare_parameter<float>("minimum_target_altitude", 1);
	this->declare_parameter<float>("target_cable_fixed_position_distance_threshold", 0.25);
	this->declare_parameter<float>("target_cable_safety_margin_distance_threshold", 0.25);
	this->declare_parameter<float>("target_cable_safety_margin_max_euc_distance", 0.1);
	this->declare_parameter<float>("target_cable_safety_margin_max_euc_velocity", 1.);
	this->declare_parameter<float>("target_cable_safety_margin_max_euc_acceleration", 1.);
	this->declare_parameter<float>("target_cable_safety_margin_max_yaw_distance", M_PI/10);
	this->declare_parameter<float>("target_cable_safety_margin_max_yaw_velocity", M_PI_2);
	this->declare_parameter<float>("target_cable_set_point_truncate_distance_threshold", 0.1);
	this->declare_parameter<bool>("always_hover_in_offboard",false);

	this->declare_parameter<double>("disarming_on_cable_max_descend_distance", 0.25);
	this->declare_parameter<std::string>("disarm_on_cable_mode", "land");
	this->declare_parameter<double>("disarm_on_cable_thrust_decrease_time_s", 5.);
	this->declare_parameter<double>("disarm_on_cable_thrust_wait_for_disarm_time_s", 5.0);
	this->declare_parameter<double>("disarm_on_cable_thrust_initial", 0.75);

	this->declare_parameter<float>("arming_on_cable_upwards_velocity", 0.1);

	this->declare_parameter<bool>("update_home_position", true);

	this->declare_parameter<int>("arm_cnt_timout", 10);
	this->declare_parameter<int>("offboard_cnt_timeout", 10);
	this->declare_parameter<int>("land_cnt_timeout", 10);
	this->declare_parameter<int>("target_cable_cnt_timeout", 10);

	this->declare_parameter<float>("max_acceleration", 1.);

	this->declare_parameter<double>("dt", 0.2);

	this->declare_parameter<bool>("use_cartesian_PID", true);

	this->declare_parameter<double>("cartesian_PID_Kp", 0.1);
	this->declare_parameter<double>("cartesian_PID_Ki", 0.1);
	this->declare_parameter<double>("cartesian_PID_Kd", 0.1);
	this->declare_parameter<double>("cartesian_PID_integral_reset_error_threshold", 0.05);

	this->declare_parameter<int>("MPC_N", 10);
	this->declare_parameter<bool>("MPC_use_state_feedback", true);

	this->declare_parameter<float>("direct_target_setpoint_dist_threshold", 0.25);

	this->declare_parameter<bool>("hover_under_cable_on_aborted_cable_landing", true);

	this->declare_parameter<float>("cable_landing_target_upwards_velocity", 0.1);

	this->declare_parameter<double>("position_MPC_vx_max", 10.);
	this->declare_parameter<double>("position_MPC_vy_max", 10.);
	this->declare_parameter<double>("position_MPC_vz_max", 10.);

	this->declare_parameter<double>("position_MPC_ax_max", 10.);
	this->declare_parameter<double>("position_MPC_ay_max", 10.);
	this->declare_parameter<double>("position_MPC_az_max", 10.);

	this->declare_parameter<double>("position_MPC_wx", 0.);
	this->declare_parameter<double>("position_MPC_wy", 0.);
	this->declare_parameter<double>("position_MPC_wz", 0.);

	this->declare_parameter<double>("position_MPC_wvx", 10.00);
	this->declare_parameter<double>("position_MPC_wvy", 10.00);
	this->declare_parameter<double>("position_MPC_wvz", 10.00);

	this->declare_parameter<double>("position_MPC_wax", 10.00);
	this->declare_parameter<double>("position_MPC_way", 10.00);
	this->declare_parameter<double>("position_MPC_waz", 10.00);

	this->declare_parameter<double>("position_MPC_wjx", 10.00);
	this->declare_parameter<double>("position_MPC_wjy", 10.00);
	this->declare_parameter<double>("position_MPC_wjz", 10.00);

	this->declare_parameter<double>("fly_along_cable_MPC_vz_max", 10.);

	this->declare_parameter<double>("fly_along_cable_MPC_ax_max", 10.);
	this->declare_parameter<double>("fly_along_cable_MPC_ay_max", 10.);
	this->declare_parameter<double>("fly_along_cable_MPC_az_max", 10.);

	this->declare_parameter<double>("fly_along_cable_MPC_wx", 0.);
	this->declare_parameter<double>("fly_along_cable_MPC_wy", 0.);
	this->declare_parameter<double>("fly_along_cable_MPC_wz", 0.);

	this->declare_parameter<double>("fly_along_cable_MPC_wvx", 10.00);
	this->declare_parameter<double>("fly_along_cable_MPC_wvy", 10.00);
	this->declare_parameter<double>("fly_along_cable_MPC_wvz", 10.00);

	this->declare_parameter<double>("fly_along_cable_MPC_wax", 10.00);
	this->declare_parameter<double>("fly_along_cable_MPC_way", 10.00);
	this->declare_parameter<double>("fly_along_cable_MPC_waz", 10.00);

	this->declare_parameter<double>("fly_along_cable_MPC_wjx", 10.00);
	this->declare_parameter<double>("fly_along_cable_MPC_wjy", 10.00);
	this->declare_parameter<double>("fly_along_cable_MPC_wjz", 10.00);

	this->declare_parameter<double>("cable_landing_MPC_vx_max", 10.);
	this->declare_parameter<double>("cable_landing_MPC_vy_max", 10.);
	this->declare_parameter<double>("cable_landing_MPC_vz_max", 10.);

	this->declare_parameter<double>("cable_landing_MPC_ax_max", 10.);
	this->declare_parameter<double>("cable_landing_MPC_ay_max", 10.);
	this->declare_parameter<double>("cable_landing_MPC_az_max", 10.);

	this->declare_parameter<double>("cable_landing_MPC_wx", 0.);
	this->declare_parameter<double>("cable_landing_MPC_wy", 0.);
	this->declare_parameter<double>("cable_landing_MPC_wz", 0.);

	this->declare_parameter<double>("cable_landing_MPC_wvx", 0.);
	this->declare_parameter<double>("cable_landing_MPC_wvy", 0.);
	this->declare_parameter<double>("cable_landing_MPC_wvz", 0.);

	this->declare_parameter<double>("cable_landing_MPC_wax", 0.);
	this->declare_parameter<double>("cable_landing_MPC_way", 0.);
	this->declare_parameter<double>("cable_landing_MPC_waz", 0.);

	this->declare_parameter<double>("cable_landing_MPC_wjx", 0.);
	this->declare_parameter<double>("cable_landing_MPC_wjy", 0.);
	this->declare_parameter<double>("cable_landing_MPC_wjz", 0.);

	this->declare_parameter<double>("cable_takeoff_MPC_vx_max", 10.);
	this->declare_parameter<double>("cable_takeoff_MPC_vy_max", 10.);
	this->declare_parameter<double>("cable_takeoff_MPC_vz_max", 10.);

	this->declare_parameter<double>("cable_takeoff_MPC_ax_max", 10.);
	this->declare_parameter<double>("cable_takeoff_MPC_ay_max", 10.);
	this->declare_parameter<double>("cable_takeoff_MPC_az_max", 10.);

	this->declare_parameter<double>("cable_takeoff_MPC_wx", 0.);
	this->declare_parameter<double>("cable_takeoff_MPC_wy", 0.);
	this->declare_parameter<double>("cable_takeoff_MPC_wz", 0.);

	this->declare_parameter<double>("cable_takeoff_MPC_wvx", 0.);
	this->declare_parameter<double>("cable_takeoff_MPC_wvy", 0.);
	this->declare_parameter<double>("cable_takeoff_MPC_wvz", 0.);

	this->declare_parameter<double>("cable_takeoff_MPC_wax", 0.);
	this->declare_parameter<double>("cable_takeoff_MPC_way", 0.);
	this->declare_parameter<double>("cable_takeoff_MPC_waz", 0.);

	this->declare_parameter<double>("cable_takeoff_MPC_wjx", 0.);
	this->declare_parameter<double>("cable_takeoff_MPC_wjy", 0.);
	this->declare_parameter<double>("cable_takeoff_MPC_wjz", 0.);

	bool use_cartesian_PID;
	this->get_parameter("use_cartesian_PID",use_cartesian_PID);

	bool always_hover_in_offboard;
	this->get_parameter("always_hover_in_offboard",always_hover_in_offboard);

	if (use_cartesian_PID && always_hover_in_offboard) {
	    RCLCPP_FATAL(this->get_logger(), "use_cartesian_PID and always_hover_in_offboard are not compatible");
	    exit(1);
	}

	this->get_parameter("MPC_N", MPC_N_);

	quat_t temp_q(1,0,0,0);
	vector_t temp_vec(0,0,0);

	odom_q_ = temp_q;
	odom_pos_ = temp_vec;
	odom_vel_ = temp_vec;
	odom_last_vel_ = temp_vec;
	odom_acc_ = temp_vec;
	odom_ang_vel_ = temp_vec;

	// tf
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	// Takeoff action:
	this->takeoff_server_ = rclcpp_action::create_server<Takeoff>(
		this,
		"takeoff",
		std::bind(&TrajectoryController::handleGoalTakeoff, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&TrajectoryController::handleCancelTakeoff, this, std::placeholders::_1),
		std::bind(&TrajectoryController::handleAcceptedTakeoff, this, std::placeholders::_1)
	);

	// Landing action:
	this->landing_server_ = rclcpp_action::create_server<Landing>(
		this,
		"landing",
		std::bind(&TrajectoryController::handleGoalLanding, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&TrajectoryController::handleCancelLanding, this, std::placeholders::_1),
		std::bind(&TrajectoryController::handleAcceptedLanding, this, std::placeholders::_1)
	);

	// Fly to position action:
	this->fly_to_position_server_ = rclcpp_action::create_server<FlyToPosition>(
		this,
		"fly_to_position",
		std::bind(&TrajectoryController::handleGoalFlyToPosition, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&TrajectoryController::handleCancelFlyToPosition, this, std::placeholders::_1),
		std::bind(&TrajectoryController::handleAcceptedFlyToPosition, this, std::placeholders::_1)
	);

	// Fly under cable action:
	this->fly_under_cable_server_ = rclcpp_action::create_server<FlyUnderCable>(
		this,
		"fly_under_cable",
		std::bind(&TrajectoryController::handleGoalFlyUnderCable, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&TrajectoryController::handleCancelFlyUnderCable, this, std::placeholders::_1),
		std::bind(&TrajectoryController::handleAcceptedFlyUnderCable, this, std::placeholders::_1)
	);

	// Fly along cable action:
	this->fly_along_cable_server_ = rclcpp_action::create_server<FlyAlongCable>(
		this,
		"fly_along_cable",
		std::bind(&TrajectoryController::handleGoalFlyAlongCable, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&TrajectoryController::handleCancelFlyAlongCable, this, std::placeholders::_1),
		std::bind(&TrajectoryController::handleAcceptedFlyAlongCable, this, std::placeholders::_1)
	);

	// Cable landing action:
	this->cable_landing_server_ = rclcpp_action::create_server<CableLanding>(
		this,
		"cable_landing",
		std::bind(&TrajectoryController::handleGoalCableLanding, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&TrajectoryController::handleCancelCableLanding, this, std::placeholders::_1),
		std::bind(&TrajectoryController::handleAcceptedCableLanding, this, std::placeholders::_1)
	);

	// Cable takeoff action:
	this->cable_takeoff_server_ = rclcpp_action::create_server<CableTakeoff>(
		this,
		"cable_takeoff",
		std::bind(&TrajectoryController::handleGoalCableTakeoff, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&TrajectoryController::handleCancelCableTakeoff, this, std::placeholders::_1),
		std::bind(&TrajectoryController::handleAcceptedCableTakeoff, this, std::placeholders::_1)
	);

	// Arm on cable action:
	this->arm_on_cable_server_ = rclcpp_action::create_server<ArmOnCable>(
		this,
		"arm_on_cable",
		std::bind(&TrajectoryController::handleGoalArmOnCable, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&TrajectoryController::handleCancelArmOnCable, this, std::placeholders::_1),
		std::bind(&TrajectoryController::handleAcceptedArmOnCable, this, std::placeholders::_1)
	);

	// Disarm on cable action:
	this->disarm_on_cable_server_ = rclcpp_action::create_server<DisarmOnCable>(
		this,
		"disarm_on_cable",
		std::bind(&TrajectoryController::handleGoalDisarmOnCable, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&TrajectoryController::handleCancelDisarmOnCable, this, std::placeholders::_1),
		std::bind(&TrajectoryController::handleAcceptedDisarmOnCable, this, std::placeholders::_1)
	);

	// Set yaw service:
    set_yaw_service_ = this->create_service<iii_interfaces::srv::SetGeneralTargetYaw>("set_general_target_yaw", 
        std::bind(&TrajectoryController::setYawServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

	// Publishers and subscriptions:
	planned_traj_pub_ = this->create_publisher<nav_msgs::msg::Path>("planned_trajectory", 10);
	planned_macro_traj_pub_ = this->create_publisher<nav_msgs::msg::Path>("planned_macro_trajectory", 10);
	planned_target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("planned_target", 10);

	setpoint_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("setpoint_pose", 10);

	offboard_control_mode_pub_ =
		this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
	trajectory_setpoint_pub_ =
		this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
	vehicle_command_pub_ =
		this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

	thrust_setpoint_pub_ =
		this->create_publisher<px4_msgs::msg::VehicleThrustSetpoint>("/fmu/in/vehicle_thrust_setpoint", 10);

	torque_setpoint_pub_ =
		this->create_publisher<px4_msgs::msg::VehicleTorqueSetpoint>("/fmu/in/vehicle_torque_setpoint", 10);

	control_state_pub_ = 
		this->create_publisher<iii_interfaces::msg::ControlState>("control_state", 10);

	target_cable_id_pub_ = 
		this->create_publisher<std_msgs::msg::Int16>("target_cable_id", 10);

	rclcpp::QoS sub_qos(rclcpp::KeepLast(1));
	sub_qos.transient_local();
	sub_qos.best_effort();


	// check nav_state if in offboard (14)
	// VehicleStatus: https://github.com/PX4/px4_msgs/blob/master/msg/VehicleStatus.msg
	vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
		"/fmu/out/vehicle_status",
		sub_qos,
		[this](px4_msgs::msg::VehicleStatus::ConstSharedPtr msg) {
			arming_state_ = msg->arming_state;
			nav_state_ = msg->nav_state;
		}
	);

	// get common timestamp
	timesync_sub_ = this->create_subscription<px4_msgs::msg::TimesyncStatus>(
		"/fmu/out/timesync_status",
		sub_qos,
		[this](const px4_msgs::msg::TimesyncStatus::UniquePtr msg) {
			timestamp_.store(msg->timestamp);
		}
	);

	odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(  ////
		"/fmu/out/vehicle_odometry", 
		sub_qos,
		std::bind(&TrajectoryController::odometryCallback, this, std::placeholders::_1));

	home_position_sub_ = this->create_subscription<px4_msgs::msg::HomePosition>(
		"/fmu/out/home_position", 
		sub_qos,
		std::bind(&TrajectoryController::homePositionCallback, this, std::placeholders::_1));

	powerline_sub_ = this->create_subscription<iii_interfaces::msg::Powerline>(
		"/pl_mapper/powerline", 
		sub_qos,
		std::bind(&TrajectoryController::powerlineCallback, this, std::placeholders::_1));

	int controller_period_ms;
	this->get_parameter("controller_period_ms", controller_period_ms);

	main_state_machine_timer_ = this->create_wall_timer(
		std::chrono::milliseconds(controller_period_ms), std::bind(&TrajectoryController::stateMachineCallback, this));

	// RCLCPP debug successfully initilized trajectory controller
	RCLCPP_INFO(this->get_logger(), "Successfully initialized trajectory controller");

}

TrajectoryController::~TrajectoryController() {

	RCLCPP_INFO(this->get_logger(),  "Shutting down offboard control..");
	land();
	std::this_thread::sleep_for(std::chrono::milliseconds(50));
	
}

rclcpp_action::GoalResponse TrajectoryController::handleGoalTakeoff(
	const rclcpp_action::GoalUUID & uuid, 
	std::shared_ptr<const Takeoff::Goal> goal
) {

	RCLCPP_DEBUG(this->get_logger(), "Received takeoff goal request with target altitude %d", goal->target_altitude);
	
	(void)uuid;

	float min_target_altitude;
	this->get_parameter("min_target_altitude", min_target_altitude);

	if (goal->target_altitude < min_target_altitude){
		// debug target altitude too low
		RCLCPP_DEBUG(this->get_logger(), "Target altitude %f too low", goal->target_altitude);
		return rclcpp_action::GoalResponse::REJECT;
	}


	if (state_ != on_ground_non_offboard) {
		// debug not in on ground non offboard state
		RCLCPP_DEBUG(this->get_logger(), "Not in on ground non offboard state");
		return rclcpp_action::GoalResponse::REJECT;
	}

	takeoff_request_params_t *params = new takeoff_request_params_t;
	params->takeoff_altitude = goal->target_altitude;

	rclcpp_action::GoalUUID action_id = (rclcpp_action::GoalUUID)uuid;

	request_t request = {
		.action_id = action_id,
		.request_type = takeoff_request,
		.request_params = (void *)params
	};

	if (!request_queue_.Push(request, false)) {
		// debug request queue full
		RCLCPP_DEBUG(this->get_logger(), "Request queue full");
		return rclcpp_action::GoalResponse::REJECT;
	}

	rclcpp_action::GoalResponse return_response;

	return_response = rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

	// debug takeoff goal accepted
	RCLCPP_DEBUG(this->get_logger(), "Takeoff goal accepted");

	return return_response;

}

rclcpp_action::CancelResponse TrajectoryController::handleCancelTakeoff(const std::shared_ptr<GoalHandleTakeoff> goal_handle) {

	RCLCPP_DEBUG(this->get_logger(), "Received request to cancel takeoff, rejecting..");

	return rclcpp_action::CancelResponse::REJECT;

}

void TrajectoryController::handleAcceptedTakeoff(const std::shared_ptr<GoalHandleTakeoff> goal_handle) {

	// debug takeoff goal accepted, starting thread
	RCLCPP_DEBUG(this->get_logger(), "Takeoff goal accepted, starting thread");

	using namespace std::placeholders;

	std::thread{ std::bind(&TrajectoryController::followTakeoffCompletion, this, _1), goal_handle}.detach();

}

void TrajectoryController::followTakeoffCompletion(const std::shared_ptr<GoalHandleTakeoff> goal_handle) {

	// debug takeoff goal thread started
	RCLCPP_DEBUG(this->get_logger(), "Takeoff goal thread started");

	rclcpp_action::GoalUUID action_id = (rclcpp_action::GoalUUID)goal_handle->get_goal_id();

	auto feedback = std::make_shared<Takeoff::Feedback>();

	auto result = std::make_shared<Takeoff::Result>();

	request_reply_t reply = {
		.action_id = action_id
	};
	
	while(true) {

		while (!request_reply_queue_.Pop(reply, false) || reply.action_id != action_id) {
		// while (!request_reply_queue_.Peak(reply, false) || reply.action_id != action_id) {

			state4_t vechicle_state = loadVehicleState();
			feedback->altitude = vechicle_state(2);

			goal_handle->publish_feedback(feedback);

			// debug waiting for takeoff completion, published feedback with vehicle state
			RCLCPP_DEBUG(
				this->get_logger(), 
				"Waiting for takeoff completion, published feedback with vehicle state: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f", 
				vechicle_state(0), 
				vechicle_state(1), 
				vechicle_state(2), 
				vechicle_state(3), 
				vechicle_state(4), 
				vechicle_state(5), 
				vechicle_state(6), 
				vechicle_state(7), 
				vechicle_state(8), 
				vechicle_state(9), 
				vechicle_state(10), 
				vechicle_state(11)
			);

			request_completion_poll_rate_.sleep();

		}

		// if(!request_reply_queue_.Pop(reply, false)) throw std::exception(); // The reply should still be in the queue
			

		switch (reply.reply_type) {

		default:
		case cancel:
			// debug takeoff goal cancelled
			RCLCPP_DEBUG(this->get_logger(), "Takeoff goal cancelled");
		case reject:
			// debug takeoff goal rejected
			RCLCPP_DEBUG(this->get_logger(), "Takeoff goal rejected");
		case fail:
			// debug takeoff goal failed
			RCLCPP_DEBUG(this->get_logger(), "Takeoff goal failed");

			result->success = false;
			goal_handle->abort(result);

			// debug takeoff goal aborted
			RCLCPP_DEBUG(this->get_logger(), "Takeoff goal aborted");

			return;
			break;
		
		case accept:

			break;

		case success:
			// debug takeoff goal succeeded, notifying success
			RCLCPP_DEBUG(this->get_logger(), "Takeoff goal succeeded, notifying success");

			result->success = true;
			goal_handle->succeed(result);

			return;
			break;

		}
	}
}

rclcpp_action::GoalResponse TrajectoryController::handleGoalLanding(
	const rclcpp_action::GoalUUID & uuid, 
	std::shared_ptr<const Landing::Goal> goal
) {

	RCLCPP_DEBUG(this->get_logger(), "Received landing goal request");
	
	(void)uuid;

	if (state_ != hovering && state_ != hovering_under_cable){
		// debug landing goal rejected, not hovering
		RCLCPP_DEBUG(this->get_logger(), "Landing goal rejected, not hovering");
		return rclcpp_action::GoalResponse::REJECT;
	}

	rclcpp_action::GoalUUID action_id = (rclcpp_action::GoalUUID)uuid;

	request_t request = {
		.action_id = action_id,
		.request_type = landing_request,
		.request_params = NULL
	};

	if (!request_queue_.Push(request, false))  {
		// debug landing goal rejected, request queue full
		RCLCPP_DEBUG(this->get_logger(), "Landing goal rejected, request queue full");
		return rclcpp_action::GoalResponse::REJECT;
	}

	rclcpp_action::GoalResponse return_response;

	return_response = rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

	// debug landing goal accepted
	RCLCPP_DEBUG(this->get_logger(), "Landing goal accepted");

	return return_response;

}

rclcpp_action::CancelResponse TrajectoryController::handleCancelLanding(const std::shared_ptr<GoalHandleLanding> goal_handle) {

	RCLCPP_DEBUG(this->get_logger(), "Received landing goal cancel request, rejecting");

	return rclcpp_action::CancelResponse::REJECT;

}

void TrajectoryController::handleAcceptedLanding(const std::shared_ptr<GoalHandleLanding> goal_handle) {

	// debug landing goal accepted, starting thread
	RCLCPP_DEBUG(this->get_logger(), "Landing goal accepted, starting thread");

	using namespace std::placeholders;

	std::thread{ std::bind(&TrajectoryController::followLandingCompletion, this, _1), goal_handle}.detach();

}

void TrajectoryController::followLandingCompletion(const std::shared_ptr<GoalHandleLanding> goal_handle) {

	// debug landing goal thread started
	RCLCPP_DEBUG(this->get_logger(), "Landing goal thread started");

	rclcpp_action::GoalUUID action_id = (rclcpp_action::GoalUUID)goal_handle->get_goal_id();

	auto feedback = std::make_shared<Landing::Feedback>();

	auto result = std::make_shared<Landing::Result>();

	request_reply_t reply = {
		.action_id = action_id
	};
	
	while(true) {

		while (!request_reply_queue_.Pop(reply, false) || reply.action_id != action_id) {
		// while (!request_reply_queue_.Peak(reply, false) || reply.action_id != action_id) {

			state4_t vechicle_state = loadVehicleState();
			feedback->altitude = vechicle_state(2);

			goal_handle->publish_feedback(feedback);
		
			// debug landing goal thread waiting for reply,	publishing feedback with vehicle state
			RCLCPP_DEBUG(this->get_logger(), "Landing goal thread waiting for reply, publishing feedback with vehicle state: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f", 
				vechicle_state(0), 
				vechicle_state(1), 
				vechicle_state(2), 
				vechicle_state(3), 
				vechicle_state(4), 
				vechicle_state(5), 
				vechicle_state(6), 
				vechicle_state(7), 
				vechicle_state(8), 
				vechicle_state(9), 
				vechicle_state(10), 
				vechicle_state(11)
			);

			request_completion_poll_rate_.sleep();

		}

		// if(!request_reply_queue_.Pop(reply, false)) throw std::exception(); // The reply should still be in the queue
			

		switch (reply.reply_type) {

		default:
		case cancel:
			// debug landing goal cancelled
			RCLCPP_DEBUG(this->get_logger(), "Landing goal cancelled");
		case reject:
			// debug landing goal rejected
			RCLCPP_DEBUG(this->get_logger(), "Landing goal rejected");
		case fail:
			// debug landing goal failed
			RCLCPP_DEBUG(this->get_logger(), "Landing goal failed");

			result->success = false;
			goal_handle->abort(result);

			return;
			break;
		
		case accept:

			break;

		case success:
			// debug landing goal succeeded
			RCLCPP_DEBUG(this->get_logger(), "Landing goal succeeded, noitifying success");

			result->success = true;
			goal_handle->succeed(result);

			return;
			break;

		}
	}
}

rclcpp_action::GoalResponse TrajectoryController::handleGoalFlyToPosition(
	const rclcpp_action::GoalUUID & uuid, 
	std::shared_ptr<const FlyToPosition::Goal> goal
) {

	// debug received fly to position goal request
	RCLCPP_DEBUG(this->get_logger(), "Received fly to position goal request");
	
	(void)uuid;

	if (state_ != hovering && state_ != hovering_under_cable) {
		// debug fly to position goal rejected, not hovering
		RCLCPP_DEBUG(this->get_logger(), "Fly to position goal rejected, not hovering");
		return rclcpp_action::GoalResponse::REJECT;
	}

	rclcpp_action::GoalUUID action_id = (rclcpp_action::GoalUUID)uuid;

	fly_to_position_request_params_t *params = new fly_to_position_request_params_t;

	geometry_msgs::msg::PoseStamped target_pose;
	target_pose.header.frame_id = goal->target_pose.header.frame_id;
	target_pose.pose = goal->target_pose.pose;
	target_pose = tf_buffer_->transform(target_pose, "world");

	quat_t quat(
		target_pose.pose.orientation.w,
		target_pose.pose.orientation.x,
		target_pose.pose.orientation.y,
		target_pose.pose.orientation.z
	);

	orientation_t eul = quatToEul(quat);

	pos4_t target_position;
	target_position(0) = target_pose.pose.position.x;
	target_position(1) = target_pose.pose.position.y;
	target_position(2) = target_pose.pose.position.z;
	target_position(3) = eul(2);

	params->target_position = target_position;

	request_t request = {
		.action_id = action_id,
		.request_type = fly_to_position_request,
		.request_params = (void *)params
	};

	if (!request_queue_.Push(request, false)) {
		// debug fly to position goal rejected, request queue full
		RCLCPP_DEBUG(this->get_logger(), "Fly to position goal rejected, request queue full");
		return rclcpp_action::GoalResponse::REJECT;
	}

	// debug fly to position goal accepted
	RCLCPP_DEBUG(this->get_logger(), "Fly to position goal accepted");

	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

}

rclcpp_action::CancelResponse TrajectoryController::handleCancelFlyToPosition(const std::shared_ptr<GoalHandleFlyToPosition> goal_handle) {

	RCLCPP_DEBUG(this->get_logger(), "Received fly to position cancel request");
	
	rclcpp_action::GoalUUID action_id = (rclcpp_action::GoalUUID)goal_handle->get_goal_id();

	request_t request = {
		.action_id = action_id,
		.request_type = cancel_request,
		.request_params = NULL
	};

	if (!request_queue_.Push(request, false)) {
		// debug fly to position cancel rejected, request queue full
		return rclcpp_action::CancelResponse::REJECT;
	}

	// debug fly to position cancel accepted
	RCLCPP_DEBUG(this->get_logger(), "Fly to position cancel accepted");

	return rclcpp_action::CancelResponse::ACCEPT;

}

void TrajectoryController::handleAcceptedFlyToPosition(const std::shared_ptr<GoalHandleFlyToPosition> goal_handle) {

	// debug fly to position goal accepted
	RCLCPP_DEBUG(this->get_logger(), "Fly to position goal accepted, starting thread");

	using namespace std::placeholders;

	std::thread{ std::bind(&TrajectoryController::followFlyToPositionCompletion, this, _1), goal_handle}.detach();

}

void TrajectoryController::followFlyToPositionCompletion(const std::shared_ptr<GoalHandleFlyToPosition> goal_handle) {

	// debug fly to position goal thread started
	RCLCPP_DEBUG(this->get_logger(), "Fly to position goal thread started");

	rclcpp_action::GoalUUID action_id = (rclcpp_action::GoalUUID)goal_handle->get_goal_id();

	auto feedback = std::make_shared<FlyToPosition::Feedback>();

	auto result = std::make_shared<FlyToPosition::Result>();

	request_reply_t reply = {
		.action_id = action_id
	};
	
	while(true) {

		while (!request_reply_queue_.Pop(reply, false) || reply.action_id != action_id) {
		// while (!request_reply_queue_.Peak(reply, false) || reply.action_id != action_id) {

			geometry_msgs::msg::PoseStamped vehicle_pose = loadVehiclePose();
			nav_msgs::msg::Path planned_path = loadPlannedPath();

			feedback->vehicle_pose = vehicle_pose;
			feedback->planned_path = planned_path;

			goal_handle->publish_feedback(feedback);

			// debug fly to position goal thread waiting for reply, publishing feedback with vehicle pose and planned path
			RCLCPP_DEBUG(this->get_logger(), "Fly to position goal thread waiting for reply, publishing feedback with vehicle pose: (%f, %f, %f) and planned path with %d poses", vehicle_pose.pose.position.x, vehicle_pose.pose.position.y, vehicle_pose.pose.position.z, planned_path.poses.size());

			request_completion_poll_rate_.sleep();

		}

		// if(!request_reply_queue_.Pop(reply, false)) throw std::exception(); // The reply should still be in the queue

		switch (reply.reply_type) {

		default:
		case cancel:
			// debug fly to position goal thread received cancel reply
			RCLCPP_DEBUG(this->get_logger(), "Fly to position goal thread received cancel reply");

			if (goal_handle->is_canceling()) {
				// debug fly to position goal thread canceling
				RCLCPP_DEBUG(this->get_logger(), "Fly to position goal thread canceling");

				result->success = false;
				goal_handle->canceled(result);

				return;
				break;

			}

		case reject:
			// debug fly to position goal thread received reject reply
			RCLCPP_DEBUG(this->get_logger(), "Fly to position goal thread received reject reply");
		case fail:
			// debug fly to position goal thread received fail reply
			RCLCPP_DEBUG(this->get_logger(), "Fly to position goal thread received fail reply, aborting");

			result->success = false;
			goal_handle->abort(result);

			return;
			break;
		
		case accept:
			break;

		case success:
			// debug fly to position goal thread received success reply
			RCLCPP_DEBUG(this->get_logger(), "Fly to position goal thread received success reply, succeeding");

			result->success = true;
			goal_handle->succeed(result);

			return;
			break;

		}
	}
}

rclcpp_action::GoalResponse TrajectoryController::handleGoalFlyUnderCable(
	const rclcpp_action::GoalUUID & uuid, 
	std::shared_ptr<const FlyUnderCable::Goal> goal
) {

	RCLCPP_DEBUG(this->get_logger(), "Received fly under cable goal request");
	
	(void)uuid;

	if (state_ != hovering && state_ != hovering_under_cable) {
		// debug fly under cable goal rejected, not hovering
		RCLCPP_DEBUG(this->get_logger(), "Fly under cable goal rejected, not hovering");
		return rclcpp_action::GoalResponse::REJECT;
	}

	rclcpp_action::GoalUUID action_id = (rclcpp_action::GoalUUID)uuid;

	fly_under_cable_request_params_t *params = new fly_under_cable_request_params_t;

	int cable_id = goal->target_cable_id;
	float cable_distance = goal->target_cable_distance;

	if (cable_distance <= 0) {
		// debug fly under cable goal rejected, invalid cable distance
		RCLCPP_DEBUG(this->get_logger(), "Fly under cable goal rejected, invalid cable distance");
		return rclcpp_action::GoalResponse::REJECT;
	}

	state4_t veh_state = loadVehicleState();

	if (!updateTargetCablePose(veh_state, cable_id)) {

		// debug fly under cable goal rejected, cable with cable id %d not visible
		RCLCPP_DEBUG(this->get_logger(), "Fly under cable goal rejected, cable with cable id %d not visible", cable_id);

		clearTargetCable();

		return rclcpp_action::GoalResponse::REJECT;

	}

	target_cable_distance_ = cable_distance;

	params->cable_id = cable_id;
	params->target_cable_distance = cable_distance;

	request_t request = {
		.action_id = action_id,
		.request_type = fly_under_cable_request,
		.request_params = (void *)params
	};

	if (!request_queue_.Push(request, false))  {
		// debug fly under cable goal rejected, request queue full
		return rclcpp_action::GoalResponse::REJECT;
	}

	// debug fly under cable goal accepted
	RCLCPP_DEBUG(this->get_logger(), "Fly under cable goal accepted");

	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

}

rclcpp_action::CancelResponse TrajectoryController::handleCancelFlyUnderCable(const std::shared_ptr<GoalHandleFlyUnderCable> goal_handle) {

	RCLCPP_INFO(this->get_logger(), "Received fly under cable cancel request");
	
	rclcpp_action::GoalUUID action_id = (rclcpp_action::GoalUUID)goal_handle->get_goal_id();

	request_t request = {
		.action_id = action_id,
		.request_type = cancel_request,
		.request_params = NULL
	};

	if (!request_queue_.Push(request, false)) {
		// debug fly under cable cancel request rejected, request queue full
		RCLCPP_DEBUG(this->get_logger(), "Fly under cable cancel request rejected, request queue full");
		return rclcpp_action::CancelResponse::REJECT;
	}

	// debug fly under cable cancel request accepted
	RCLCPP_DEBUG(this->get_logger(), "Fly under cable cancel request accepted");

	return rclcpp_action::CancelResponse::ACCEPT;

}

void TrajectoryController::handleAcceptedFlyUnderCable(const std::shared_ptr<GoalHandleFlyUnderCable> goal_handle) {

	// debug fly under cable goal accepted, starting thread
	RCLCPP_DEBUG(this->get_logger(), "Fly under cable goal accepted, starting thread");

	using namespace std::placeholders;

	std::thread{ std::bind(&TrajectoryController::followFlyUnderCableCompletion, this, _1), goal_handle}.detach();

}

void TrajectoryController::followFlyUnderCableCompletion(const std::shared_ptr<GoalHandleFlyUnderCable> goal_handle) {

	// debug fly under cable goal thread started
	RCLCPP_DEBUG(this->get_logger(), "Fly under cable goal thread started");

	rclcpp_action::GoalUUID action_id = (rclcpp_action::GoalUUID)goal_handle->get_goal_id();

	auto feedback = std::make_shared<FlyUnderCable::Feedback>();

	auto result = std::make_shared<FlyUnderCable::Result>();

	request_reply_t reply = {
		.action_id = action_id
	};
	
	while(true) {

		while (!request_reply_queue_.Pop(reply, false) || reply.action_id != action_id) {
		// while (!request_reply_queue_.Peak(reply, false) || reply.action_id != action_id) {

			geometry_msgs::msg::PoseStamped vehicle_pose = loadVehiclePose();
			nav_msgs::msg::Path planned_path = loadPlannedPath();
			geometry_msgs::msg::PoseStamped target = loadPlannedTarget();

			feedback->vehicle_pose = vehicle_pose;
			feedback->planned_path = planned_path;

			quat_t veh_quat(
				vehicle_pose.pose.orientation.w,
				vehicle_pose.pose.orientation.x,
				vehicle_pose.pose.orientation.y,
				vehicle_pose.pose.orientation.z
			);
			orientation_t veh_eul = quatToEul(veh_quat);
			pos4_t veh_state(
				vehicle_pose.pose.position.x,
				vehicle_pose.pose.position.y,
				vehicle_pose.pose.position.z,
				veh_eul(2)
			);
			quat_t target_quat(
				target.pose.orientation.w,
				target.pose.orientation.x,
				target.pose.orientation.y,
				target.pose.orientation.z
			);
			orientation_t target_eul = quatToEul(target_quat);
			pos4_t target_state(
				target.pose.position.x,
				target.pose.position.y,
				target.pose.position.z,
				target_eul(2)
			);

			feedback->distance_vehicle_to_target = (target_state - veh_state).norm();

			goal_handle->publish_feedback(feedback);

			// debug fly under cable goal thread waiting for reply, publish feedback with vehicle pose: %f, %f, %f, %f, target pose: %f, %f, %f, %f, distance: %f, planned path size: %d
			RCLCPP_DEBUG(this->get_logger(), "Fly under cable goal thread waiting for reply, publish feedback with vehicle pose: %f, %f, %f, %f, target pose: %f, %f, %f, %f, distance: %f, planned path size: %d",
				vehicle_pose.pose.position.x,
				vehicle_pose.pose.position.y,
				vehicle_pose.pose.position.z,
				veh_eul(2),
				target.pose.position.x,
				target.pose.position.y,
				target.pose.position.z,
				target_eul(2),
				feedback->distance_vehicle_to_target,
				planned_path.poses.size()
			);

			request_completion_poll_rate_.sleep();

		}

		// if(!request_reply_queue_.Pop(reply, false)) throw std::exception(); // The reply should still be in the queue
			
		switch (reply.reply_type) {

		default:
		case cancel:
			// debug fly under cable goal thread received cancel reply
			RCLCPP_DEBUG(this->get_logger(), "Fly under cable goal thread received cancel reply");

			if (goal_handle->is_canceling()) {
				// debug fly under cable goal thread cancel request accepted
				RCLCPP_DEBUG(this->get_logger(), "Fly under cable goal thread cancel request accepted");

				result->success = false;
				goal_handle->canceled(result);

				return;
				break;

			}

		case reject:
			// debug fly under cable goal thread received reject reply
			RCLCPP_DEBUG(this->get_logger(), "Fly under cable goal thread received reject reply");
		case fail:
			// debug fly under cable goal thread received fail reply
			RCLCPP_DEBUG(this->get_logger(), "Fly under cable goal thread received fail reply, aborting");

			result->success = false;
			goal_handle->abort(result);

			return;
			break;
		
		case accept:
			break;

		case success:
			//	debug fly under cable goal thread received success reply
			RCLCPP_DEBUG(this->get_logger(), "Fly under cable goal thread received success reply, notifying success");

			result->success = true;
			goal_handle->succeed(result);

			return;
			break;

		}
	}
}

rclcpp_action::GoalResponse TrajectoryController::handleGoalFlyAlongCable(
	const rclcpp_action::GoalUUID & uuid,
	std::shared_ptr<const FlyAlongCable::Goal> goal
) {

	RCLCPP_DEBUG(this->get_logger(), "Received fly along cable goal request");

	(void)uuid;

	rclcpp_action::GoalUUID action_id = (rclcpp_action::GoalUUID)uuid;

	if (state_ != hovering_under_cable) {
		RCLCPP_ERROR(this->get_logger(), "Cannot fly along cable when not hovering under cable");
		return rclcpp_action::GoalResponse::REJECT;
	}

	float distance = goal->distance;
	float velocity = goal->velocity;

	if (distance <= 0) {
		RCLCPP_ERROR(this->get_logger(), "Distance must be positive");
		return rclcpp_action::GoalResponse::REJECT;
	}

	if (velocity <= 0) {
		RCLCPP_ERROR(this->get_logger(), "Velocity must be positive");
		return rclcpp_action::GoalResponse::REJECT;
	}

	request_t request = {
		.action_id = action_id,
		.request_type = fly_along_cable_request,
		.request_params = new fly_along_cable_request_params_t{
			.distance = distance,
			.velocity = velocity,
			.inverse_direction = goal->inverse_direction
		}
	};

	if (!request_queue_.Push(request, false)) {
		RCLCPP_ERROR(this->get_logger(), "Failed to push request to queue");
		return rclcpp_action::GoalResponse::REJECT;
	}

	// debug fly along cable goal request accepted
	RCLCPP_DEBUG(this->get_logger(), "Fly along cable goal request accepted");

	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

}

rclcpp_action::CancelResponse TrajectoryController::handleCancelFlyAlongCable(
	const std::shared_ptr<rclcpp_action::ServerGoalHandle<FlyAlongCable>> goal_handle
) {

	RCLCPP_DEBUG(this->get_logger(), "Received cancel fly along cable request");

	request_t request = {
		.action_id = goal_handle->get_goal_id(),
		.request_type = cancel_request,
		.request_params = NULL
	};

	if (request_queue_.Push(request, false)) {
		return rclcpp_action::CancelResponse::ACCEPT;
	} else {
		RCLCPP_ERROR(this->get_logger(), "Failed to push cancel request to queue");
		return rclcpp_action::CancelResponse::REJECT;
	}

}

void TrajectoryController::handleAcceptedFlyAlongCable(
	const std::shared_ptr<rclcpp_action::ServerGoalHandle<FlyAlongCable>> goal_handle
) {

	using namespace std::placeholders;

	RCLCPP_DEBUG(this->get_logger(), "Accepted fly along cable goal");

	std::thread{std::bind(&TrajectoryController::followFlyAlongCableCompletion, this, _1), goal_handle}.detach();

}

void TrajectoryController::followFlyAlongCableCompletion(const std::shared_ptr<GoalHandleFlyAlongCable> goal_handle) {
	
	RCLCPP_DEBUG(this->get_logger(), "Starting fly along cable completion thread");

	rclcpp_action::GoalUUID action_id = goal_handle->get_goal_id();

	auto feedback = std::make_shared<FlyAlongCable::Feedback>();
	auto result = std::make_shared<FlyAlongCable::Result>();

	request_reply_t reply = {
		.action_id = action_id
	};


	while(true) {

		while(!request_reply_queue_.Pop(reply, false) || reply.action_id != action_id) {
		// while(!request_reply_queue_.Peak(reply, false) || reply.action_id != action_id) {
			
			geometry_msgs::msg::PoseStamped vehicle_pose = loadVehiclePose();
			feedback->vehicle_pose = vehicle_pose;

			goal_handle->publish_feedback(feedback);

			// debug fly along cable goal thread waiting for reply, publishing feedback woth vehicle pose: %s
			RCLCPP_DEBUG(this->get_logger(), "Fly along cable goal thread waiting for reply, publishing feedback woth vehicle pose: %s", vehicle_pose.pose.position);

			request_completion_poll_rate_.sleep();

		}

		// if (!request_reply_queue_.Pop(reply, false)) throw std::exception(); // The reply should still be in the queue

		switch (reply.reply_type) {

		default:
		case cancel:
			// debug fly along cable goal thread received cancel reply
			RCLCPP_DEBUG(this->get_logger(), "Fly along cable goal thread received cancel reply");

			if (goal_handle->is_canceling()) {
				// debug fly along cable goal thread received cancel reply and goal is canceling
				RCLCPP_DEBUG(this->get_logger(), "Fly along cable goal thread received cancel reply and goal is canceling");

				result->success = false;
				goal_handle->canceled(result);

				return;
				break;

			}

		case reject:
		case fail:
			// debug fly along cable goal thread received reject or fail reply
			RCLCPP_DEBUG(this->get_logger(), "Fly along cable goal thread received reject or fail reply");

			result->success = false;
			goal_handle->abort(result);

			return;
			break;

		case accept:

			break;

		case success:

			// debug fly along cable goal thread received success reply
			RCLCPP_DEBUG(this->get_logger(), "Fly along cable goal thread received success reply, notifying success");

			result->success = true;
			goal_handle->succeed(result);

			return;
			break;

		}
	}
}

rclcpp_action::GoalResponse TrajectoryController::handleGoalCableLanding(
	const rclcpp_action::GoalUUID & uuid, 
	std::shared_ptr<const CableLanding::Goal> goal
) {

	RCLCPP_DEBUG(this->get_logger(), "Received cable landing goal request");
	
	(void)uuid;

	if (state_ != hovering_under_cable) {
		RCLCPP_ERROR(this->get_logger(), "Received cable landing goal request while not hovering under cable");
		return rclcpp_action::GoalResponse::REJECT;
	}

	int cable_id = goal->target_cable_id;

	if (cable_id != target_cable_id_) {
		RCLCPP_ERROR(this->get_logger(), "Received cable landing goal request for cable %d while target cable is %d", cable_id, target_cable_id_);
		return rclcpp_action::GoalResponse::REJECT;
	}

	state4_t veh_state = loadVehicleState();

	if (!updateTargetCablePose(veh_state, cable_id)) {

		// debug failed to update target cable pose, cable with id %d not visible, rejecting cable landing goal
		RCLCPP_DEBUG(this->get_logger(), "Failed to update target cable pose, cable with id %d not visible, rejecting cable landing goal", cable_id);

		clearTargetCable();

		return rclcpp_action::GoalResponse::REJECT;

	}

	rclcpp_action::GoalUUID action_id = (rclcpp_action::GoalUUID)uuid;

	cable_landing_request_params_t *params = new cable_landing_request_params_t;

	params->cable_id = cable_id;

	request_t request = {
		.action_id = action_id,
		.request_type = cable_landing_request,
		.request_params = (void *)params
	};

	if (!request_queue_.Push(request, false))  {

		// debug failed to push cable landing request to request queue, rejecting cable landing goal
		RCLCPP_DEBUG(this->get_logger(), "Failed to push cable landing request to request queue, rejecting cable landing goal");

		clearTargetCable();

		return rclcpp_action::GoalResponse::REJECT;

	}

	// debug cable landing goal accepted
	RCLCPP_DEBUG(this->get_logger(), "Cable landing goal accepted");

	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

}

rclcpp_action::CancelResponse TrajectoryController::handleCancelCableLanding(const std::shared_ptr<GoalHandleCableLanding> goal_handle) {

	RCLCPP_DEBUG(this->get_logger(), "Received cable landing cancel request");
	
	rclcpp_action::GoalUUID action_id = (rclcpp_action::GoalUUID)goal_handle->get_goal_id();

	request_t request = {
		.action_id = action_id,
		.request_type = cancel_request,
		.request_params = NULL
	};

	if (!request_queue_.Push(request, false)) {
		// debug failed to push cable landing cancel request to request queue, rejecting cancel request
		RCLCPP_DEBUG(this->get_logger(), "Failed to push cable landing cancel request to request queue, rejecting cancel request");
		return rclcpp_action::CancelResponse::REJECT;
	}

	return rclcpp_action::CancelResponse::ACCEPT;

}

void TrajectoryController::handleAcceptedCableLanding(const std::shared_ptr<GoalHandleCableLanding> goal_handle) {

	// debug cable landing goal accepted
	RCLCPP_DEBUG(this->get_logger(), "Cable landing goal accepted, starting thread to follow completion");

	using namespace std::placeholders;

	std::thread{ std::bind(&TrajectoryController::followCableLandingCompletion, this, _1), goal_handle}.detach();

}

void TrajectoryController::followCableLandingCompletion(const std::shared_ptr<GoalHandleCableLanding> goal_handle) {

	// debug following cable landing completion
	RCLCPP_DEBUG(this->get_logger(), "Following cable landing completion");

	rclcpp_action::GoalUUID action_id = (rclcpp_action::GoalUUID)goal_handle->get_goal_id();

	auto feedback = std::make_shared<CableLanding::Feedback>();

	auto result = std::make_shared<CableLanding::Result>();

	request_reply_t reply = {
		.action_id = action_id
	};
	
	while(true) {

		while (!request_reply_queue_.Pop(reply, false) || reply.action_id != action_id) {
		// while (!request_reply_queue_.Peak(reply, false) || reply.action_id != action_id) {

			geometry_msgs::msg::PoseStamped vehicle_pose = loadVehiclePose();
			nav_msgs::msg::Path planned_path = loadPlannedPath();
			nav_msgs::msg::Path planned_macro_path = loadPlannedMacroPath();
			geometry_msgs::msg::PoseStamped planned_target = loadPlannedTarget();

			float distance = 0;
			distance += pow(vehicle_pose.pose.position.x - planned_target.pose.position.x, 2);
			distance += pow(vehicle_pose.pose.position.y - planned_target.pose.position.y, 2);
			distance += pow(vehicle_pose.pose.position.z - planned_target.pose.position.z, 2);
			distance = sqrt(distance);


			feedback->vehicle_pose = vehicle_pose;
			feedback->planned_path = planned_path;
			feedback->planned_macro_path = planned_macro_path;
			feedback->distance_vehicle_to_cable = distance;

			goal_handle->publish_feedback(feedback);

			// debug waiting for cable landing completion, publishing feedback with vehicle pose (%f, %f, %f), planned path with %d poses, planned macro path with %d poses, distance to cable %f

			request_completion_poll_rate_.sleep();

		}

		// if(!request_reply_queue_.Pop(reply, false)) throw std::exception(); // The reply should still be in the queue
	
		switch (reply.reply_type) {

		default:
		case cancel:
			// debug cable landing goal canceled
			RCLCPP_DEBUG(this->get_logger(), "Cable landing goal canceled");

			if (goal_handle->is_canceling()) {

				// debug cable landing goal canceled, sending cancel reply
				RCLCPP_DEBUG(this->get_logger(), "Cable landing goal canceled, sending cancel reply");

				result->success = false;
				goal_handle->canceled(result);

				return;
				break;

			}

		case reject:
		case fail:
			// debug cable landing goal failed
			RCLCPP_DEBUG(this->get_logger(), "Cable landing goal failed");

			result->success = false;
			goal_handle->abort(result);

			return;
			break;
	
		case accept:

			break;

		case success:
			// debug cable landing goal succeeded
			RCLCPP_DEBUG(this->get_logger(), "Cable landing goal succeeded");

			result->success = true;
			goal_handle->succeed(result);

			return;
			break;

		}
	}
}

rclcpp_action::GoalResponse TrajectoryController::handleGoalCableTakeoff(
	const rclcpp_action::GoalUUID & uuid, 
	std::shared_ptr<const CableTakeoff::Goal> goal
) {

	RCLCPP_INFO(this->get_logger(), "Received cable takeoff goal request");
	
	(void)uuid;

	if (state_ != on_cable_armed) {
		// debug cable takeoff goal rejected, not on cable armed
		RCLCPP_INFO(this->get_logger(), "Cable takeoff goal rejected, not on cable armed");
		return rclcpp_action::GoalResponse::REJECT;
	}

	float target_cable_distance = goal->target_cable_distance;

	if (target_cable_distance < 1.) {
		// debug cable takeoff goal rejected, target cable distance too small
		RCLCPP_INFO(this->get_logger(), "Cable takeoff goal rejected, target cable distance too small");
		return rclcpp_action::GoalResponse::REJECT;
	}

	target_cable_distance_ = target_cable_distance;

	rclcpp_action::GoalUUID action_id = (rclcpp_action::GoalUUID)uuid;

	cable_takeoff_request_params_t *params = new cable_takeoff_request_params_t;

	params->target_cable_distance = target_cable_distance;

	request_t request = {
		.action_id = action_id,
		.request_type = cable_takeoff_request,
		.request_params = (void *)params
	};

	if (!request_queue_.Push(request, false)) {
		// debug cable takeoff goal rejected, request queue full
		RCLCPP_INFO(this->get_logger(), "Cable takeoff goal rejected, request queue full");

		return rclcpp_action::GoalResponse::REJECT;
	}

	// debug cable takeoff goal accepted
	RCLCPP_INFO(this->get_logger(), "Cable takeoff goal accepted");

	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

}

rclcpp_action::CancelResponse TrajectoryController::handleCancelCableTakeoff(const std::shared_ptr<GoalHandleCableTakeoff> goal_handle) {

	RCLCPP_DEBUG(this->get_logger(), "Received cable takeoff cancel request, rejecting");

	return rclcpp_action::CancelResponse::REJECT;

}

void TrajectoryController::handleAcceptedCableTakeoff(const std::shared_ptr<GoalHandleCableTakeoff> goal_handle) {

	// debug cable takeoff goal accepted
	RCLCPP_DEBUG(this->get_logger(), "Cable takeoff goal accepted, starting thread");

	using namespace std::placeholders;

	std::thread{ std::bind(&TrajectoryController::followCableTakeoffCompletion, this, _1), goal_handle}.detach();

}

void TrajectoryController::followCableTakeoffCompletion(const std::shared_ptr<GoalHandleCableTakeoff> goal_handle) {

	// debug cable takeoff goal thread started
	RCLCPP_DEBUG(this->get_logger(), "Cable takeoff goal thread started");

	rclcpp_action::GoalUUID action_id = (rclcpp_action::GoalUUID)goal_handle->get_goal_id();

	auto feedback = std::make_shared<CableTakeoff::Feedback>();

	auto result = std::make_shared<CableTakeoff::Result>();

	request_reply_t reply = {
		.action_id = action_id
	};
	
	while(true) {

		while (!request_reply_queue_.Pop(reply, false) || reply.action_id != action_id) {
		// while (!request_reply_queue_.Peak(reply, false) || reply.action_id != action_id) {

			//geometry_msgs::msg::PoseStamped vehicle_pose = loadVehiclePose();
			//nav_msgs::msg::Path planned_path = loadPlannedPath();
			//nav_msgs::msg::Path planned_macro_path = loadPlannedMacroPath();
			//geometry_msgs::msg::PoseStamped planned_target = loadPlannedTarget();

			//float distance = 0;
			//distance += pow(vehicle_pose.pose.position.x - planned_target.pose.position.x, 2);
			//distance += pow(vehicle_pose.pose.position.y - planned_target.pose.position.y, 2);
			//distance += pow(vehicle_pose.pose.position.z - planned_target.pose.position.z, 2);
			//distance = sqrt(distance);


			feedback->distance_vehicle_to_cable = 1;

			goal_handle->publish_feedback(feedback);

			// debug cable takeoff goal thread waiting for completion, publish feedback
			RCLCPP_DEBUG(this->get_logger(), "Cable takeoff goal thread waiting for completion, publish feedback");

			request_completion_poll_rate_.sleep();

		}

		// if(!request_reply_queue_.Pop(reply, false)) throw std::exception(); // The reply should still be in the queue
	
		switch (reply.reply_type) {

		default:
		case cancel:
			// debug cable takeoff goal thread waiting for completion, cancel
			RCLCPP_DEBUG(this->get_logger(), "Cable takeoff goal thread waiting for completion, cancel");

			if (goal_handle->is_canceling()) {
				// debug cable takeoff goal thread waiting for completion, cancel, goal handle canceling
				RCLCPP_DEBUG(this->get_logger(), "Cable takeoff goal thread waiting for completion, cancel, goal handle canceling");

				result->success = false;
				goal_handle->canceled(result);

				return;
				break;

			}

		case reject:
		case fail:
			// debug cable takeoff goal thread waiting for completion, fail
			RCLCPP_DEBUG(this->get_logger(), "Cable takeoff goal thread waiting for completion, fail");

			result->success = false;
			goal_handle->abort(result);

			return;
			break;
	
		case accept:

			break;

		case success:
			// debug cable takeoff goal thread waiting for completion, success
			RCLCPP_DEBUG(this->get_logger(), "Cable takeoff goal thread waiting for completion, success");

			result->success = true;
			goal_handle->succeed(result);

			return;
			break;

		}
	}
}






















rclcpp_action::GoalResponse TrajectoryController::handleGoalDisarmOnCable(
	const rclcpp_action::GoalUUID & uuid, 
	std::shared_ptr<const DisarmOnCable::Goal> goal
) {

	RCLCPP_INFO(this->get_logger(), "Received disarm on cable goal request");
	
	(void)uuid;

	if (state_ != on_cable_armed) {
		// debug disarm on cable goal rejected, not on cable armed
		RCLCPP_INFO(this->get_logger(), "Disarm on cable goal rejected, not on cable armed");
		return rclcpp_action::GoalResponse::REJECT;
	}

	rclcpp_action::GoalUUID action_id = (rclcpp_action::GoalUUID)uuid;

	request_t request = {
		.action_id = action_id,
		.request_type = disarm_on_cable_request,
		.request_params = NULL
	};

	if (!request_queue_.Push(request, false)) {
		// debug cable takeoff goal rejected, request queue full
		RCLCPP_INFO(this->get_logger(), "Disarm on cable goal rejected, request queue full");

		return rclcpp_action::GoalResponse::REJECT;
	}

	// debug cable takeoff goal accepted
	RCLCPP_INFO(this->get_logger(), "Disarm on cable goal accepted");

	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

}

rclcpp_action::CancelResponse TrajectoryController::handleCancelDisarmOnCable(const std::shared_ptr<GoalHandleDisarmOnCable> goal_handle) {

	RCLCPP_DEBUG(this->get_logger(), "Received disarm on cable cancel request, rejecting");

	return rclcpp_action::CancelResponse::REJECT;

}

void TrajectoryController::handleAcceptedDisarmOnCable(const std::shared_ptr<GoalHandleDisarmOnCable> goal_handle) {

	// debug cable takeoff goal accepted
	RCLCPP_DEBUG(this->get_logger(), "Disarm on cable goal accepted, starting thread");

	using namespace std::placeholders;

	std::thread{ std::bind(&TrajectoryController::followDisarmOnCableCompletion, this, _1), goal_handle}.detach();

}

void TrajectoryController::followDisarmOnCableCompletion(const std::shared_ptr<GoalHandleDisarmOnCable> goal_handle) {

	RCLCPP_DEBUG(this->get_logger(), "Disarm on cable goal thread started");

	rclcpp_action::GoalUUID action_id = (rclcpp_action::GoalUUID)goal_handle->get_goal_id();

	auto feedback = std::make_shared<DisarmOnCable::Feedback>();

	auto result = std::make_shared<DisarmOnCable::Result>();

	request_reply_t reply = {
		.action_id = action_id
	};
	
	while(true) {

		while (!request_reply_queue_.Pop(reply, false) || reply.action_id != action_id) {

			geometry_msgs::msg::PoseStamped vehicle_pose = loadVehiclePose();

			feedback->distance_vehicle_to_cable = 1;
			feedback->vehicle_pose = vehicle_pose;

			goal_handle->publish_feedback(feedback);

			RCLCPP_DEBUG(this->get_logger(), "Disarm on cable goal thread waiting for completion, publish feedback");

			request_completion_poll_rate_.sleep();

		}

		switch (reply.reply_type) {

		default:
		case cancel:
			RCLCPP_DEBUG(this->get_logger(), "Disarm on cable goal thread waiting for completion, cancel");

			if (goal_handle->is_canceling()) {
				RCLCPP_DEBUG(this->get_logger(), "Disarm on cable goal thread waiting for completion, cancel, goal handle canceling");

				result->success = false;
				goal_handle->canceled(result);

				return;
				break;

			}

		case reject:
		case fail:
			RCLCPP_DEBUG(this->get_logger(), "Disarm on cable goal thread waiting for completion, fail");

			result->success = false;
			goal_handle->abort(result);

			return;
			break;
	
		case accept:

			break;

		case success:
			RCLCPP_DEBUG(this->get_logger(), "Disarm on cable goal thread waiting for completion, success");

			result->success = true;
			goal_handle->succeed(result);

			return;
			break;

		}
	}
}



rclcpp_action::GoalResponse TrajectoryController::handleGoalArmOnCable(
	const rclcpp_action::GoalUUID & uuid, 
	std::shared_ptr<const ArmOnCable::Goal> goal
) {

	RCLCPP_INFO(this->get_logger(), "Received disarm on cable goal request");
	
	(void)uuid;

	if (state_ != on_cable_armed) {
		// debug disarm on cable goal rejected, not on cable armed
		RCLCPP_INFO(this->get_logger(), "Arm on cable goal rejected, not on cable armed");
		return rclcpp_action::GoalResponse::REJECT;
	}

	rclcpp_action::GoalUUID action_id = (rclcpp_action::GoalUUID)uuid;

	request_t request = {
		.action_id = action_id,
		.request_type = disarm_on_cable_request,
		.request_params = NULL
	};

	if (!request_queue_.Push(request, false)) {
		// debug cable takeoff goal rejected, request queue full
		RCLCPP_INFO(this->get_logger(), "Arm on cable goal rejected, request queue full");

		return rclcpp_action::GoalResponse::REJECT;
	}

	// debug cable takeoff goal accepted
	RCLCPP_INFO(this->get_logger(), "Arm on cable goal accepted");

	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

}

rclcpp_action::CancelResponse TrajectoryController::handleCancelArmOnCable(const std::shared_ptr<GoalHandleArmOnCable> goal_handle) {

	RCLCPP_DEBUG(this->get_logger(), "Received disarm on cable cancel request, rejecting");

	return rclcpp_action::CancelResponse::REJECT;

}

void TrajectoryController::handleAcceptedArmOnCable(const std::shared_ptr<GoalHandleArmOnCable> goal_handle) {

	// debug cable takeoff goal accepted
	RCLCPP_DEBUG(this->get_logger(), "Arm on cable goal accepted, starting thread");

	using namespace std::placeholders;

	std::thread{ std::bind(&TrajectoryController::followArmOnCableCompletion, this, _1), goal_handle}.detach();

}

void TrajectoryController::followArmOnCableCompletion(const std::shared_ptr<GoalHandleArmOnCable> goal_handle) {

	RCLCPP_DEBUG(this->get_logger(), "Arm on cable goal thread started");

	rclcpp_action::GoalUUID action_id = (rclcpp_action::GoalUUID)goal_handle->get_goal_id();

	auto feedback = std::make_shared<ArmOnCable::Feedback>();

	auto result = std::make_shared<ArmOnCable::Result>();

	request_reply_t reply = {
		.action_id = action_id
	};
	
	while(true) {

		while (!request_reply_queue_.Pop(reply, false) || reply.action_id != action_id) {

			geometry_msgs::msg::PoseStamped vehicle_pose = loadVehiclePose();

			feedback->distance_vehicle_to_cable = 1;
			feedback->vehicle_pose = vehicle_pose;

			goal_handle->publish_feedback(feedback);

			RCLCPP_DEBUG(this->get_logger(), "Arm on cable goal thread waiting for completion, publish feedback");

			request_completion_poll_rate_.sleep();

		}

		switch (reply.reply_type) {

		default:
		case cancel:
			RCLCPP_DEBUG(this->get_logger(), "Arm on cable goal thread waiting for completion, cancel");

			if (goal_handle->is_canceling()) {
				RCLCPP_DEBUG(this->get_logger(), "Arm on cable goal thread waiting for completion, cancel, goal handle canceling");

				result->success = false;
				goal_handle->canceled(result);

				return;
				break;

			}

		case reject:
		case fail:
			RCLCPP_DEBUG(this->get_logger(), "Arm on cable goal thread waiting for completion, fail");

			result->success = false;
			goal_handle->abort(result);

			return;
			break;
	
		case accept:

			break;

		case success:
			RCLCPP_DEBUG(this->get_logger(), "Arm on cable goal thread waiting for completion, success");

			result->success = true;
			goal_handle->succeed(result);

			return;
			break;

		}
	}
}

void TrajectoryController::setYawServiceCallback(const std::shared_ptr<iii_interfaces::srv::SetGeneralTargetYaw::Request> request,
                                std::shared_ptr<iii_interfaces::srv::SetGeneralTargetYaw::Response> response) {

    target_yaw_mutex_.lock(); {

		target_yaw_ = request->target_yaw;

	} target_yaw_mutex_.unlock();

    response->success = true;

}

void TrajectoryController::stateMachineCallback() {

	float landed_altitude_threshold;
	this->get_parameter("landed_altitude_threshold", landed_altitude_threshold);

	float reached_pos_euc_dist_thresh;
	this->get_parameter("reached_position_euclidean_distance_threshold", reached_pos_euc_dist_thresh);

	float direct_target_setpoint_dist_threshold;
	this->get_parameter("direct_target_setpoint_dist_threshold", direct_target_setpoint_dist_threshold);

	bool use_cartesian_PID;
	this->get_parameter("use_cartesian_PID", use_cartesian_PID);

	bool always_hover_in_offboard;
	this->get_parameter("always_hover_in_offboard", always_hover_in_offboard);

	static bool printed = false;
	if (!printed && always_hover_in_offboard) {
		RCLCPP_INFO(this->get_logger(),"Always hover in offboard");
		printed = true;
	}

	static state4_t prev_veh_state;
	static state4_t veh_state;
	prev_veh_state = veh_state;
	veh_state = loadVehicleState();

	static state4_t set_point;
	static state4_t prev_set_point;
	prev_set_point = set_point;
	set_point = veh_state;

	static state4_t fixed_reference = veh_state;
	static state4_t always_hover_state = fixed_reference;
	static state4_t prev_fixed_reference;

	static bool direct_target_setpoint = false;

	rclcpp_action::GoalUUID temp_uuid;
	
	static request_t request = {
		.action_id = (rclcpp_action::GoalUUID &)temp_uuid
	};

	static int arm_cnt = 0;
	static int offboard_cnt = 0;
	static int land_cnt = 0;
	static int target_cable_cnt = 0;

	static int cable_id = -1;
	static float target_cable_distance = -1;
	static float fly_along_cable_distance = -1;
	static float fly_along_cable_velocity = -1;
	static bool fly_along_cable_inverse_direction = false;

	static state4_t prev_fly_along_cable_state;
	static state4_t fly_along_cable_state;

	static vector_t disarm_on_cable_initial_position;
	static int disarm_on_cable_cnt;

	bool offboard = isOffboard();
	bool armed = isArmed();

	static bool has_determined_ground_altitude_offset = false;
	static float ground_altitude_offset = 0.;

	bool use_ground_altitude_offset;
	this->get_parameter("use_ground_altitude_offset", use_ground_altitude_offset);

	if (use_ground_altitude_offset) {
		if (state_ != disarming_on_cable && state_ != on_cable_disarmed && state_ != arming_on_cable && state_ != setting_offboard_on_cable) {
			if (!offboard && !armed) {
				ground_altitude_offset = veh_state(2);
				has_determined_ground_altitude_offset = true;
			}
		}
	}

	auto notifyCurrentRequest = [&](request_reply_type_t reply_type) -> bool {

		RCLCPP_DEBUG(this->get_logger(), "Notify current request");

		request_reply_t reply = {
			.action_id = request.action_id,
			.reply_type = reply_type
		};

		RCLCPP_DEBUG(this->get_logger(), "Push reply to queue");

		request_reply_t dummy_reply;

		while (request_reply_queue_.Pop(dummy_reply, false));

		request_reply_queue_.Push(reply, true);

		RCLCPP_DEBUG(this->get_logger(), "Reply pushed to queue");

		return true;

	};

	auto rejectPendingRequest = [&]() -> bool {

		if (request_queue_.Pop(request, false)) {

			notifyCurrentRequest(reject);

			return true;

		} else {

			return false;

		}
	};

	auto tryPendingRequest = [&](request_type_t request_type, 
			request_queue_action_t do_notify, request_queue_action_t do_pop) -> bool {

		bool result;

		if (do_pop == yes)
			result = request_queue_.Pop(request, false);
		else
			result = request_queue_.Peak(request, false);

		if (result) {

			if (request.request_type == request_type) {

				if (do_pop == if_match) {

					result = request_queue_.Pop(request, false);

					if (do_notify == yes || do_notify == if_match) notifyCurrentRequest(accept);

				}

				return true;

			} else {
				
				if (do_notify == yes && do_pop == yes) notifyCurrentRequest(reject);
				return false;

			}

		} else return false;

	};

	auto currentRequestIsCancelled = [&](request_queue_action_t do_notify, request_queue_action_t do_pop) -> bool {

		// LOG_INFO("b1");

		request_t tmp_request;

		bool result;

		// RCLCPP_DEBUG(this->get_logger(), "Dummy write");

		if (do_pop == yes)
			result = request_queue_.Pop(tmp_request, false);
		else
			result = request_queue_.Peak(tmp_request, false);

		// LOG_INFO("b3");

		if (result && tmp_request.action_id == request.action_id) {

		// LOG_INFO("b4");

			if (do_pop == if_match) {

		// LOG_INFO("b5");

				result = request_queue_.Pop(request, false);

		// LOG_INFO("b6");

				if (do_notify == yes || do_notify == if_match) notifyCurrentRequest(cancel);

		// LOG_INFO("b7");

			}

		// LOG_INFO("b8");

			return true;

		} else if (result) {

		// LOG_INFO("b9");

			if (do_notify == yes && do_pop == yes) notifyCurrentRequest(reject);

		// LOG_INFO("b10");
			return false;

		}

		return false;

	};

	auto setZeroVelocity = [](state4_t state) -> state4_t {

		for (int i = 4; i < 12; i++) state(i) = 0;

		return state;

	};

	auto setNanVelocity = [](state4_t state) -> state4_t {

		for (int i = 4; i < 12; i++) state(i) = NAN;

		return state;

	};

	auto appendZeroVelocity = [](pos4_t pos) -> state4_t {

		state4_t state;

		for (int i = 0; i < 4; i++) {

			state(i) = pos(i);
			state(i+4) = 0;
			state(i+8) = 0;

		}

		return state;

	};

	auto setVelocityControl = [](state4_t fixed_reference, vector_t vel_control) -> state4_t {

		state4_t set_pt;

		for (int i = 0; i < 3; i++) set_pt(i) = NAN;
		set_pt(3) = fixed_reference(3);
		for (int i = 0; i < 3; i++) set_pt(i+4) = vel_control(i);
		for (int i = 7; i < 12; i++) set_pt(i) = NAN;

		return set_pt;

	};

	auto reachedPosition = [this, reached_pos_euc_dist_thresh](state4_t state, state4_t target) -> bool {

		for(int i=4;i<12;i++) target(i) = 0;
		for(int i=8;i<12;i++) state(i) = 0;

		RCLCPP_DEBUG(this->get_logger(), "Target: %f, %f, %f, %f", target(0), target(1), target(2), target(3));
		RCLCPP_DEBUG(this->get_logger(), "State: %f, %f, %f, %f", state(0), state(1), state(2), state(3));
		RCLCPP_DEBUG(this->get_logger(), "Reached position: %s < %s ?", std::to_string((state-target).norm()), std::to_string(reached_pos_euc_dist_thresh));
		

		float norm = (state-target).norm();

		//LOG_INFO(std::to_string(norm));

		return norm <= reached_pos_euc_dist_thresh;

	};

	auto withinDirectTargetDistance = [this, direct_target_setpoint_dist_threshold](state4_t state, state4_t target) -> bool {

		for(int i=3;i<12;i++) target(i) = 0;
		for(int i=3;i<12;i++) state(i) = 0;

		RCLCPP_DEBUG(this->get_logger(), "Target: %f, %f, %f, %f", target(0), target(1), target(2), target(3));
		RCLCPP_DEBUG(this->get_logger(), "State: %f, %f, %f, %f", state(0), state(1), state(2), state(3));
		RCLCPP_DEBUG(this->get_logger(), "Within direct target distance: %s < %s ?", std::to_string((state-target).norm()), std::to_string(direct_target_setpoint_dist_threshold));
		

		float norm = (state-target).norm();

		//LOG_INFO(std::to_string(norm));

		return norm <= direct_target_setpoint_dist_threshold;

	};

	auto withinSafetyMargins = [this](state4_t veh_state, state4_t target) -> bool {

		float target_cable_safety_margin_distance_threshold;
		this->get_parameter("target_cable_safety_margin_distance_threshold", target_cable_safety_margin_distance_threshold);

		float target_cable_safety_margin_max_euc_distance;
		this->get_parameter("target_cable_safety_margin_max_euc_distance", target_cable_safety_margin_max_euc_distance);

		float target_cable_safety_margin_max_euc_velocity;
		this->get_parameter("target_cable_safety_margin_max_euc_velocity", target_cable_safety_margin_max_euc_velocity);

		float target_cable_safety_margin_max_euc_acceleration;
		this->get_parameter("target_cable_safety_margin_max_euc_acceleration", target_cable_safety_margin_max_euc_acceleration);

		float target_cable_safety_margin_max_yaw_distance;
		this->get_parameter("target_cable_safety_margin_max_yaw_distance", target_cable_safety_margin_max_yaw_distance);

		float target_cable_safety_margin_max_yaw_velocity;
		this->get_parameter("target_cable_safety_margin_max_yaw_velocity", target_cable_safety_margin_max_yaw_velocity);

		vector_t target_vec(
			target(0),
			target(1),
			target(2)
		);

		vector_t veh(
			veh_state(0),
			veh_state(1),
			veh_state(2)
		);

		vector_t target_xy(
			target(0),
			target(1),
			0
		);

		vector_t veh_xy(
			veh_state(0),
			veh_state(1),
			0
		);

		vector_t veh_vel_xy(
			veh_state(4),
			veh_state(5),
			0
		);

		vector_t veh_acc_xy(
			veh_state(8),
			veh_state(9),
			0
		);

		if ((target_vec-veh).norm() < target_cable_safety_margin_distance_threshold) {

			if ((target_xy - veh_xy).norm() > target_cable_safety_margin_max_euc_distance) {

				RCLCPP_WARN(this->get_logger(), "Target cable safety margin max euclidean distance exceeded: %f > %f", (target_xy - veh_xy).norm(), target_cable_safety_margin_max_euc_distance);

				return false;

			} 

			if (veh_vel_xy.norm() > target_cable_safety_margin_max_euc_velocity) {

				RCLCPP_WARN(this->get_logger(), "Target cable safety margin max euclidean velocity exceeded: %f > %f", veh_vel_xy.norm(), target_cable_safety_margin_max_euc_velocity);

				return false;

			}

			if (veh_acc_xy.norm() > target_cable_safety_margin_max_euc_acceleration) {

				RCLCPP_WARN(this->get_logger(), "Target cable safety margin max euclidean acceleration exceeded: %f > %f", veh_acc_xy.norm(), target_cable_safety_margin_max_euc_acceleration);

				return false;

			}

			if (abs(veh_state(3) - target(3)) > target_cable_safety_margin_max_yaw_distance) {

				RCLCPP_WARN(this->get_logger(), "Target cable safety margin max yaw distance exceeded: %f > %f", abs(veh_state(3) - target(3)), target_cable_safety_margin_max_yaw_distance);

				return false;

			}

			if (veh_state(7) > target_cable_safety_margin_max_yaw_velocity) {

				RCLCPP_WARN(this->get_logger(), "Target cable safety margin max yaw velocity exceeded: %f > %f", veh_state(7), target_cable_safety_margin_max_yaw_velocity);

				return false;
			}

		}

		return true;

	};

	auto setPointSafetyMarginTruncate = [this](state4_t set_point, state4_t veh_state, state4_t target_cable) -> state4_t {

		float target_cable_set_point_truncate_distance_threshold;
		this->get_parameter("target_cable_set_point_truncate_distance_threshold", target_cable_set_point_truncate_distance_threshold);

		vector_t target(
			target_cable(0),
			target_cable(1),
			target_cable(2)
		);

		vector_t veh(
			veh_state(0),
			veh_state(1),
			veh_state(2)
		);

		state4_t ret_set_point;
		for (int i = 3; i < 12; i++)
			ret_set_point(i) = set_point(i);

		if ((target - veh).norm() <= target_cable_set_point_truncate_distance_threshold) {

			ret_set_point(0) = NAN;
			ret_set_point(1) = NAN;
			ret_set_point(2) = NAN;

		} else {

			ret_set_point(0) = set_point(0);
			ret_set_point(1) = set_point(1);
			ret_set_point(2) = set_point(2);

		}

		return ret_set_point;

	};

	switch (state_) {

	case init:
	default:

		// debug in state init
		RCLCPP_DEBUG(this->get_logger(), "state: init");

		if (!offboard && veh_state(2) < landed_altitude_threshold + ground_altitude_offset) {

			// debug not offboard and under landed altitude threshold
			RCLCPP_DEBUG(this->get_logger(), "not offboard and under landed altitude threshold");

			state_ = on_ground_non_offboard;

		} else if(!offboard && veh_state(2) >= landed_altitude_threshold) {

			// debug not offboard and over landed altitude threshold
			RCLCPP_DEBUG(this->get_logger(), "not offboard and over landed altitude threshold");

			state_ = in_flight_non_offboard;

		} else if (offboard && veh_state(2) < landed_altitude_threshold) {

			// debug offboard and under landed altitude threshold, landing
			RCLCPP_DEBUG(this->get_logger(), "offboard and under landed altitude threshold, landing");

			set_point = setNanVelocity(veh_state);

			land();

			state_ = init;

		} else if(offboard && veh_state(2) >= landed_altitude_threshold) {

			// debug offboard and over landed altitude threshold
			RCLCPP_DEBUG(this->get_logger(), "offboard and over landed altitude threshold");

			fixed_reference = setZeroVelocity(veh_state);

			setTrajectoryTarget(fixed_reference);

			//if (use_cartesian_PID) {

			//	vector_t vel_control = stepCartesianVelocityPID(veh_state, fixed_reference, true);
			//	set_point = setVelocityControl(fixed_reference, vel_control);

			//} else {

				set_point = setNanVelocity(fixed_reference);

			//}

			state_ = hovering;

		}

		break;

	case on_ground_non_offboard:

		// debug in state on_ground_non_offboard
		RCLCPP_DEBUG(this->get_logger(), "state: on_ground_non_offboard");

		if (!offboard && veh_state(2) >= landed_altitude_threshold && armed) {

			// debug not offboard and over landed altitude threshold and armed
			RCLCPP_DEBUG(this->get_logger(), "not offboard and over landed altitude threshold and armed");

			state_ = in_flight_non_offboard;

		} else if (offboard) {
			// debug offboard
			RCLCPP_DEBUG(this->get_logger(), "offboard");

			state_ = init;

		} else if(tryPendingRequest(takeoff_request, yes, yes)) {

			// debug takeoff request, arming
			RCLCPP_DEBUG(this->get_logger(), "takeoff request, arming");

			arm();

			takeoff_request_params_t *takeoff_params = (takeoff_request_params_t *)request.request_params;
			float takeoff_altitude = takeoff_params->takeoff_altitude;

			delete request.request_params;

			fixed_reference = setZeroVelocity(veh_state);
			fixed_reference(2) = takeoff_altitude;

			setTrajectoryTarget(fixed_reference);

			set_point = setNanVelocity(veh_state);

			this->get_parameter("arm_cnt_timeout", arm_cnt);

			state_ = arming;

			// debug set trajectory target
			RCLCPP_DEBUG(this->get_logger(), "set trajectory target: %f, %f, %f", fixed_reference(0), fixed_reference(1), fixed_reference(2));

		} else {

			set_point = setNanVelocity(veh_state);

			rejectPendingRequest();

		} 

		break;

	case in_flight_non_offboard:

		// debug in state in_flight_non_offboard
		RCLCPP_DEBUG(this->get_logger(), "state: in_flight_non_offboard");
		
		if (!offboard && veh_state(2) < landed_altitude_threshold || !armed) {

			// debug not offboard and under landed altitude threshold or not armed
			RCLCPP_DEBUG(this->get_logger(), "not offboard and under landed altitude threshold or not armed");

			state_ = on_ground_non_offboard;

		} else if (offboard) {

			// debug offboard
			RCLCPP_DEBUG(this->get_logger(), "offboard");

			fixed_reference = setZeroVelocity(veh_state);

			setTrajectoryTarget(fixed_reference);

			//if (use_cartesian_PID) {

			//	vector_t vel_control = stepCartesianVelocityPID(veh_state, fixed_reference, true);
			//	set_point = setVelocityControl(fixed_reference, vel_control);

			//} else {

				set_point = setNanVelocity(fixed_reference);

			//}

			state_ = hovering;

		} else { 

			set_point = setNanVelocity(veh_state);

			rejectPendingRequest();

		}

		break;
	
	case arming:

		// debug in state arming
		RCLCPP_DEBUG(this->get_logger(), "state: arming");
		
		if (arm_cnt == 0 && !armed) {

			// debug arm count is zero and not armed, disarming
			RCLCPP_DEBUG(this->get_logger(), "arm count is zero and not armed, disarming");

			set_point = setNanVelocity(veh_state);

			clearPlannedTrajectory();

			disarm();

			notifyCurrentRequest(fail);
			
			state_ = on_ground_non_offboard;

		} else if(armed) {

			// debug is armed
			RCLCPP_DEBUG(this->get_logger(), "is armed");

			set_point = setNanVelocity(veh_state);
			
			setModeOffboard();

			this->get_parameter("offboard_cnt_timeout", offboard_cnt);

			state_ = setting_offboard;

		} else {

			arm_cnt--;

			rejectPendingRequest();		
		
		}

		break;

	case setting_offboard:

		// debug in state setting_offboard
		RCLCPP_DEBUG(this->get_logger(), "state: setting_offboard");

		if (offboard_cnt == 0 && !offboard) {

			// debug offboard count is zero and not offboard, disarming
			RCLCPP_DEBUG(this->get_logger(), "offboard count is zero and not offboard, disarming");

			set_point = setNanVelocity(veh_state);

			clearPlannedTrajectory();

			disarm();

			notifyCurrentRequest(fail);
			
			state_ = on_ground_non_offboard;

		} else if (offboard) {

			// debug is offboard, taking off
			RCLCPP_DEBUG(this->get_logger(), "is offboard, taking off");

			fixed_reference = setZeroVelocity(fixed_reference);

			setTrajectoryTarget(fixed_reference);

			//if (use_cartesian_PID) {

			//	vector_t vel_control = stepCartesianVelocityPID(veh_state, fixed_reference, true);
			//	set_point = setVelocityControl(fixed_reference, vel_control);

			//} else {

				set_point = setNanVelocity(fixed_reference);

			//}

			state_ = taking_off;

		} else {

			rejectPendingRequest();

			set_point = setNanVelocity(veh_state);

			offboard_cnt--;
			
		}

		break;

	case taking_off:

		// debug in state taking_off
		RCLCPP_DEBUG(this->get_logger(), "state: taking_off");
		
		if (!offboard || !armed) {

			// debug not offboard or not armed, going to init
			RCLCPP_DEBUG(this->get_logger(), "not offboard or not armed, going to init");

			notifyCurrentRequest(fail);
			rejectPendingRequest();

			clearPlannedTrajectory();

			state_ = init;

		} else if (reachedPosition(veh_state, fixed_reference)) {

			// debug reached position, takeoff successful
			RCLCPP_DEBUG(this->get_logger(), "reached position, takeoff successful");

			RCLCPP_INFO(this->get_logger(), "Takeoff successful");

			notifyCurrentRequest(success);

			clearPlannedTrajectory();

			fixed_reference = fixed_reference; // For explicability

			setTrajectoryTarget(fixed_reference);

			//if (use_cartesian_PID) {

			//	vector_t vel_control = stepCartesianVelocityPID(veh_state, fixed_reference, true);
			//	set_point = setVelocityControl(fixed_reference, vel_control);

			//} else {

				set_point = setNanVelocity(fixed_reference);

			//}

			state_ = hovering;

		} else {

			rejectPendingRequest();

			set_point = fixed_reference;

			setTrajectoryTarget(fixed_reference);

		}

		break;

	case hovering:

		// debug in state hovering
		RCLCPP_DEBUG(this->get_logger(), "state: hovering");

		if (!offboard || !armed) {

			// debug not offboard or not armed, going to init
			RCLCPP_DEBUG(this->get_logger(), "not offboard or not armed, going to init");

			rejectPendingRequest();

			state_ = init;

		} else if (tryPendingRequest(landing_request, if_match, if_match)) {

			// debug landing request, landing
			RCLCPP_DEBUG(this->get_logger(), "landing request, landing");

			land();

			set_point = setNanVelocity(veh_state);

			this->get_parameter("land_cnt_timeout", land_cnt);

			state_ = landing;

		} else if (tryPendingRequest(fly_to_position_request, if_match, if_match)) {

			RCLCPP_INFO(this->get_logger(), "Using MPC");

			direct_target_setpoint = false;

			// debug fly to position request, flying to position
			RCLCPP_DEBUG(this->get_logger(), "fly to position request, flying to position");

			fly_to_position_request_params_t *request_params = (fly_to_position_request_params_t *)request.request_params;
			pos4_t target_position = request_params->target_position;

			delete request.request_params;

			fixed_reference = appendZeroVelocity(target_position);

			setTrajectoryTarget(fixed_reference);

			if (withinDirectTargetDistance(veh_state, fixed_reference)) {

				direct_target_setpoint = true;
				
				//if (use_cartesian_PID) {

				//	vector_t vel_control = stepCartesianVelocityPID(veh_state, fixed_reference, true);
				//	set_point = setVelocityControl(fixed_reference, vel_control);

				//} else {

					set_point = setNanVelocity(fixed_reference);

				//}

				// debug within direct target setpoint dist threshold, setting direct target setpoint
				RCLCPP_DEBUG(this->get_logger(), "within direct target setpoint dist threshold, setting direct target setpoint: %f, %f, %f, %f", set_point(0), set_point(1), set_point(2), set_point(3));

			} else {
				set_point = stepMPC(veh_state, fixed_reference, true, true, positional);

				// debug set point is
				RCLCPP_DEBUG(this->get_logger(), "set point is: %f, %f, %f, %f", set_point(0), set_point(1), set_point(2), set_point(3));
			}
			//set_point = stepMPC(prev_veh_state, fixed_reference, true, true, positional);
			//set_point = setZeroVelocity(fixed_reference);

			// debug set point is
			RCLCPP_DEBUG(this->get_logger(), "set point is: %f, %f, %f, %f", set_point(0), set_point(1), set_point(2), set_point(3));

			this->get_parameter("offboard_cnt_timeout", offboard_cnt);

			state_ = in_positional_flight;

		} else if (tryPendingRequest(fly_under_cable_request, if_match, if_match)) {

			RCLCPP_INFO(this->get_logger(), "Using MPC");

			direct_target_setpoint = false;

			// debug fly under cable request, flying under cable
			RCLCPP_DEBUG(this->get_logger(), "fly under cable request, flying under cable");

			fly_under_cable_request_params_t *request_params = (fly_under_cable_request_params_t *)request.request_params;
			cable_id = request_params->cable_id;
			target_cable_distance = request_params->target_cable_distance;

			delete request.request_params;

			this->get_parameter("target_cable_cnt_timeout", target_cable_cnt);

			target_cable_cnt = updateTargetCablePose(veh_state, cable_id) ? target_cable_cnt : target_cable_cnt-1;

			fixed_reference = loadTargetUnderCableState();

			setTrajectoryTarget(fixed_reference);

			if (withinDirectTargetDistance(veh_state, fixed_reference)) {

				direct_target_setpoint = true;
				
				//if (use_cartesian_PID) {

				//	vector_t vel_control = stepCartesianVelocityPID(veh_state, fixed_reference, true);
				//	set_point = setVelocityControl(fixed_reference, vel_control);

				//} else {

					set_point = setNanVelocity(fixed_reference);

				//}

				// debug within direct target setpoint dist threshold, setting direct target setpoint
				RCLCPP_DEBUG(this->get_logger(), "within direct target setpoint dist threshold, setting direct target setpoint: %f, %f, %f, %f", set_point(0), set_point(1), set_point(2), set_point(3));

			} else {
				set_point = stepMPC(prev_veh_state, fixed_reference, true, true, positional);

				// debug set point is
				RCLCPP_DEBUG(this->get_logger(), "set point is: %f, %f, %f, %f", set_point(0), set_point(1), set_point(2), set_point(3));
			}

			//set_point = stepMPC(prev_veh_state, fixed_reference, true, true, positional);

			// debug set point is
			RCLCPP_DEBUG(this->get_logger(), "set point is: %f, %f, %f, %f", set_point(0), set_point(1), set_point(2), set_point(3));

			this->get_parameter("offboard_cnt_timeout", offboard_cnt);

			state_ = in_positional_flight;

		} else {

			rejectPendingRequest();

			//if (use_cartesian_PID) {

			//	vector_t vel_control = stepCartesianVelocityPID(veh_state, fixed_reference, false);
			//	set_point = setVelocityControl(fixed_reference, vel_control);

			//} else {

				set_point = setNanVelocity(fixed_reference);
				always_hover_state = set_point;

			//}


		}

		break;

	case landing:

		// debug in state landing
		RCLCPP_DEBUG(this->get_logger(), "state: landing");

		if (!offboard || !armed) {

			// debug not offboard or not armed, landing successful
			RCLCPP_DEBUG(this->get_logger(), "not offboard or not armed, landing successful");

			notifyCurrentRequest(success);

			set_point = setNanVelocity(veh_state);

			state_ = init;

		} else if (land_cnt == 0) {

			// debug land_cnt == 0, landing failed
			RCLCPP_DEBUG(this->get_logger(), "land_cnt == 0, landing failed");

			notifyCurrentRequest(fail);

			set_point = setNanVelocity(veh_state);

			state_ = init;

		} else {

			rejectPendingRequest();

			set_point = setNanVelocity(veh_state);

			land_cnt--;

		}

		break;

	case hovering_under_cable:

		// debug in state hovering_under_cable
		RCLCPP_DEBUG(this->get_logger(), "state: hovering_under_cable");

		if (!offboard || !armed) {

			// debug not offboard or not armed, going to init

			rejectPendingRequest();

			clearPlannedTrajectory();
			clearTargetCable();

			state_ = init;

		} else if (target_cable_cnt == 0) {

			// debug target_cable_cnt == 0, going to hovering

			clearPlannedTrajectory();
			clearTargetCable();

			fixed_reference = setZeroVelocity(veh_state);

			setTrajectoryTarget(fixed_reference);

			//if (use_cartesian_PID) {

			//	vector_t vel_control = stepCartesianVelocityPID(veh_state, fixed_reference, true);
			//	set_point = setVelocityControl(fixed_reference, vel_control);

			//} else {

				set_point = setNanVelocity(fixed_reference);

			//}

			state_ = hovering;

		} else if (tryPendingRequest(landing_request, if_match, if_match)) {

			// debug landing request, going to landing

			clearPlannedTrajectory();
			clearTargetCable();

			land();

			set_point = setNanVelocity(veh_state);

			this->get_parameter("land_cnt_timeout", land_cnt);

			state_ = landing;

		} else if (tryPendingRequest(fly_to_position_request, if_match, if_match)) {

			RCLCPP_INFO(this->get_logger(), "Using MPC");

			direct_target_setpoint = false;

			// debug fly to position request, flying to position
			RCLCPP_DEBUG(this->get_logger(), "fly to position request, flying to position");

			fly_to_position_request_params_t *request_params = (fly_to_position_request_params_t *)request.request_params;
			pos4_t target_position = request_params->target_position;

			delete request.request_params;

			clearTargetCable();

			fixed_reference = appendZeroVelocity(target_position);

			setTrajectoryTarget(fixed_reference);

			if (direct_target_setpoint || withinDirectTargetDistance(veh_state, fixed_reference)) {

				direct_target_setpoint = true;
				
				//if (use_cartesian_PID) {

				//	vector_t vel_control = stepCartesianVelocityPID(veh_state, fixed_reference, true);
				//	set_point = setVelocityControl(fixed_reference, vel_control);

				//} else {

					set_point = setNanVelocity(fixed_reference);

				//}

				// debug within direct target setpoint dist threshold, setting direct target setpoint
				RCLCPP_DEBUG(this->get_logger(), "within direct target setpoint dist threshold, setting direct target setpoint: %f, %f, %f, %f", set_point(0), set_point(1), set_point(2), set_point(3));
			} else {
				set_point = stepMPC(prev_veh_state, fixed_reference, true, true, positional);

				// debug set point is
				RCLCPP_DEBUG(this->get_logger(), "set point is: %f, %f, %f, %f", set_point(0), set_point(1), set_point(2), set_point(3));
			}

			this->get_parameter("offboard_cnt_timeout", offboard_cnt);

			state_ = in_positional_flight;

		} else if (tryPendingRequest(fly_under_cable_request, if_match, if_match)) {

			RCLCPP_INFO(this->get_logger(), "Using MPC");

			direct_target_setpoint = false;

			// debug fly under cable request, flying under cable
			RCLCPP_DEBUG(this->get_logger(), "fly under cable request, flying under cable");

			fly_under_cable_request_params_t *request_params = (fly_under_cable_request_params_t *)request.request_params;
			cable_id = request_params->cable_id;
			target_cable_distance = request_params->target_cable_distance;

			delete request.request_params;

			this->get_parameter("target_cable_cnt_timeout", target_cable_cnt);

			target_cable_cnt = updateTargetCablePose(veh_state, cable_id) ? target_cable_cnt : target_cable_cnt-1;

			fixed_reference = loadTargetUnderCableState();

			setTrajectoryTarget(fixed_reference);

			if (direct_target_setpoint || withinDirectTargetDistance(veh_state, fixed_reference)) {

				direct_target_setpoint = true;

				// debug within direct target setpoint dist threshold, setting direct target setpoint: %f, %f, %f, %f
				RCLCPP_DEBUG(this->get_logger(), "within direct target setpoint dist threshold, setting direct target setpoint: %f, %f, %f, %f", set_point(0), set_point(1), set_point(2), set_point(3));
				
				if (use_cartesian_PID) {

					vector_t vel_control = stepCartesianVelocityPID(veh_state, fixed_reference, true);
					set_point = setVelocityControl(fixed_reference, vel_control);

				} else {

					set_point = setNanVelocity(fixed_reference);

				}

			} else {
				set_point = stepMPC(prev_veh_state, fixed_reference, true, true, positional);
				// debug set point is: %f, %f, %f, %f
				RCLCPP_DEBUG(this->get_logger(), "set point is: %f, %f, %f, %f", set_point(0), set_point(1), set_point(2), set_point(3));
			}

			this->get_parameter("offboard_cnt_timeout", offboard_cnt);

			state_ = in_positional_flight;

		} else if (tryPendingRequest(cable_landing_request, if_match, if_match)) { 

			direct_target_setpoint = false;

			// debug cable landing request, landing on cable
			RCLCPP_DEBUG(this->get_logger(), "cable landing request, landing on cable");

			cable_landing_request_params_t *request_params = (cable_landing_request_params_t *)request.request_params;
			cable_id = request_params->cable_id;
			
			delete request.request_params;

			this->get_parameter("target_cable_cnt_timeout", target_cable_cnt);

			target_cable_cnt = updateTargetCablePose(veh_state, cable_id) ? target_cable_cnt : target_cable_cnt-1;

			fixed_reference = loadTargetCableState();

			float cable_landing_target_upwards_velocity;
			this->get_parameter("cable_landing_target_upwards_velocity", cable_landing_target_upwards_velocity);

			fixed_reference(6) = cable_landing_target_upwards_velocity;

			setTrajectoryTarget(fixed_reference);

			if (direct_target_setpoint || withinDirectTargetDistance(veh_state, fixed_reference)) {

				direct_target_setpoint = true;

				// debug within direct target setpoint dist threshold, setting direct target setpoint: %f, %f, %f, %f
				RCLCPP_DEBUG(this->get_logger(), "within direct target setpoint dist threshold, setting direct target setpoint: %f, %f, %f, %f", set_point(0), set_point(1), set_point(2), set_point(3));

				set_point = fixed_reference;
			} else {
				set_point = stepMPC(veh_state, fixed_reference, true, true, cable_landing);
				// debug set point is: %f, %f, %f, %f
				RCLCPP_DEBUG(this->get_logger(), "set point is: %f, %f, %f, %f", set_point(0), set_point(1), set_point(2), set_point(3));
			}
			set_point = setPointSafetyMarginTruncate(set_point, veh_state, loadTargetCableState());

			// Truncated set point is: %f, %f, %f, %f
			RCLCPP_DEBUG(this->get_logger(), "Truncated set point is: %f, %f, %f, %f", set_point(0), set_point(1), set_point(2), set_point(3));

			state_ = during_cable_landing;

		} else if (tryPendingRequest(fly_along_cable_request, if_match, if_match)) {

			RCLCPP_INFO(this->get_logger(), "Using MPC");

			direct_target_setpoint = false;

			// debug fly along cable request, flying along cable
			RCLCPP_DEBUG(this->get_logger(), "fly along cable request, flying along cable");

			fly_along_cable_request_params_t *request_params = (fly_along_cable_request_params_t *)request.request_params;

			fly_along_cable_distance = request_params->distance;
			fly_along_cable_velocity = request_params->velocity;
			fly_along_cable_inverse_direction = request_params->inverse_direction;

			setFlyAlongCableParams(fly_along_cable_distance, fly_along_cable_velocity, fly_along_cable_inverse_direction);

			target_cable_cnt = updateTargetCablePose(veh_state, cable_id) ? target_cable_cnt : target_cable_cnt-1;

			// prev_fly_along_cable_state = veh_state;
			// fly_along_cable_state = loadTargetFlyAlongCableState(
			// 	fly_along_cable_velocity,
			// 	fly_along_cable_distance,
			// 	fly_along_cable_inverse_direction,
			// 	prev_fly_along_cable_state,
			// 	true
			// );

			fixed_reference = setZeroVelocity(loadTargetFlyAlongCableState(true));

			setTrajectoryTarget(fixed_reference);

			if (direct_target_setpoint || withinDirectTargetDistance(veh_state, fixed_reference)) {

				direct_target_setpoint = true;

				// debug within direct target setpoint dist threshold, setting direct target setpoint: %f, %f, %f, %f
				RCLCPP_DEBUG(this->get_logger(), "within direct target setpoint dist threshold, setting direct target setpoint: %f, %f, %f, %f", set_point(0), set_point(1), set_point(2), set_point(3));
				
				if (use_cartesian_PID) {

					vector_t vel_control = stepCartesianVelocityPID(veh_state, fixed_reference, true);
					set_point = setVelocityControl(fixed_reference, vel_control);

				} else {

					set_point = setNanVelocity(fixed_reference);

				}

			} else {
				set_point = stepMPC(prev_veh_state, fixed_reference, true, true, fly_along_cable);
			}

			this->get_parameter("offboard_cnt_timeout", offboard_cnt);

			// debug set point is: %f, %f, %f, %f
			RCLCPP_DEBUG(this->get_logger(), "set point is: %f, %f, %f, %f", set_point(0), set_point(1), set_point(2), set_point(3));

			state_ = flying_along_cable;

		} else {

			rejectPendingRequest();

			target_cable_cnt = updateTargetCablePose(veh_state) ? target_cable_cnt : target_cable_cnt-1;

			fixed_reference = setZeroVelocity(loadTargetUnderCableState());

			setTrajectoryTarget(fixed_reference);

			if (use_cartesian_PID) {

				vector_t vel_control = stepCartesianVelocityPID(veh_state, fixed_reference, false);
				set_point = setVelocityControl(fixed_reference, vel_control);

			} else {

				set_point = setNanVelocity(fixed_reference);

			}

		}

		break;


	case in_positional_flight:

		// debug in positional flight
		RCLCPP_DEBUG(this->get_logger(), "in positional flight");

		if ((!offboard || !armed) && --offboard_cnt == 0) {

			// debug offboard or armed is false, aborting request, going to state hovering
			RCLCPP_DEBUG(this->get_logger(), "offboard or armed is false, aborting request, going to state hovering");

			notifyCurrentRequest(fail);
			rejectPendingRequest();

			clearPlannedTrajectory();
			clearTargetCable();

			state_ = init;

		} else if (request.request_type == fly_under_cable_request && target_cable_cnt == 0) {

			RCLCPP_DEBUG(this->get_logger(), "Target cable counter is zero, aborting request, going to state hovering");

			notifyCurrentRequest(fail);

			clearPlannedTrajectory();
			clearTargetCable();

			fixed_reference = setZeroVelocity(veh_state);

			setTrajectoryTarget(fixed_reference);

			//if (use_cartesian_PID) {

			//	vector_t vel_control = stepCartesianVelocityPID(veh_state, fixed_reference, true);
			//	set_point = setVelocityControl(fixed_reference, vel_control);

			//} else {

				set_point = setNanVelocity(fixed_reference);

			//}


			state_ = hovering;

		 } else if (currentRequestIsCancelled(if_match, if_match)) {

			// debug current request is cancelled, aborting request, going to state hovering
			RCLCPP_DEBUG(this->get_logger(), "current request is cancelled, aborting request, going to state hovering");

		 	fixed_reference = setZeroVelocity(veh_state);

		 	clearPlannedTrajectory();
			clearTargetCable();

			setTrajectoryTarget(fixed_reference);

			if (use_cartesian_PID) {

				vector_t vel_control = stepCartesianVelocityPID(veh_state, fixed_reference, true);
				set_point = setVelocityControl(fixed_reference, vel_control);

			} else {

				set_point = setNanVelocity(fixed_reference);

			}

		 	state_ = hovering;

		} else if (reachedPosition(veh_state, fixed_reference)) {

			// debug reached position, going to state hovering
			RCLCPP_DEBUG(this->get_logger(), "reached position, going to state hovering");

			notifyCurrentRequest(success);

			if (request.request_type == fly_under_cable_request) {

				clearPlannedTrajectory();

				target_cable_cnt = updateTargetCablePose(veh_state) ? target_cable_cnt : target_cable_cnt-1;

				fixed_reference = setZeroVelocity(loadTargetUnderCableState());

				setTrajectoryTarget(fixed_reference);

				if (use_cartesian_PID) {

					vector_t vel_control = stepCartesianVelocityPID(veh_state, fixed_reference, true);
					set_point = setVelocityControl(fixed_reference, vel_control);

				} else {

					set_point = setNanVelocity(fixed_reference);

				}
				
				state_ = hovering_under_cable;

			} else {

				clearPlannedTrajectory();

				fixed_reference = setZeroVelocity(fixed_reference);

				setTrajectoryTarget(fixed_reference);

				if (use_cartesian_PID) {

					vector_t vel_control = stepCartesianVelocityPID(veh_state, fixed_reference, true);
					set_point = setVelocityControl(fixed_reference, vel_control);

				} else {

					set_point = setNanVelocity(fixed_reference);

				}

				state_ = hovering;

			}

		} else {

			rejectPendingRequest();

			bool set_target = false;

			if (request.request_type == fly_under_cable_request) {

				set_target = true;

				target_cable_cnt = updateTargetCablePose(veh_state) ? target_cable_cnt : target_cable_cnt-1;

				fixed_reference = loadTargetUnderCableState();

				setTrajectoryTarget(fixed_reference);

			}

			if (direct_target_setpoint || withinDirectTargetDistance(veh_state, fixed_reference)) {

				if (!direct_target_setpoint){

					// RCLCPP_INFO(this->get_logger(), "Using direct target setpoint");
					// print fixed reference and vehicle state
					// RCLCPP_INFO(this->get_logger(), "fixed reference: %f, %f, %f, %f", fixed_reference(0), fixed_reference(1), fixed_reference(2), fixed_reference(3));
					// RCLCPP_INFO(this->get_logger(), "vehicle state: %f, %f, %f, %f", veh_state(0), veh_state(1), veh_state(2), veh_state(3));

				};

				direct_target_setpoint = true;

				if (use_cartesian_PID) {

					vector_t vel_control = stepCartesianVelocityPID(veh_state, fixed_reference, true);
					set_point = setVelocityControl(fixed_reference, vel_control);

				} else {

					set_point = setNanVelocity(fixed_reference);

				}

				// debug direct target setpoint: %f, %f, %f, %f
				RCLCPP_DEBUG(this->get_logger(), "direct target setpoint: %f, %f, %f, %f", set_point(0), set_point(1), set_point(2), set_point(3));
			} else {
				set_point = stepMPC(veh_state, fixed_reference, set_target, false, positional);
				// debug mpc target setpoint: %f, %f, %f, %f
				RCLCPP_DEBUG(this->get_logger(), "mpc target setpoint: %f, %f, %f, %f", set_point(0), set_point(1), set_point(2), set_point(3));
			}

			this->get_parameter("offboard_cnt_timeout", offboard_cnt);

		}

		break;

	case flying_along_cable:

		// debug in flying along cable
		RCLCPP_DEBUG(this->get_logger(), "in flying along cable");

		if ((!offboard || !armed) && --offboard_cnt == 0) {
			// debug offboard or armed is false, aborting request, going to state hovering
			RCLCPP_DEBUG(this->get_logger(), "offboard or armed is false, aborting request, going to state init");

			notifyCurrentRequest(fail);
			rejectPendingRequest();

			clearPlannedTrajectory();
			clearTargetCable();

			state_ = init;

		} else if (target_cable_cnt == 0) {
			// debug Target cable counter is zero, aborting request, going to state hovering
			RCLCPP_DEBUG(this->get_logger(), "Target cable counter is zero, aborting request, going to state hovering");

			clearPlannedTrajectory();
			clearTargetCable();

			fixed_reference = setZeroVelocity(veh_state);

			setTrajectoryTarget(fixed_reference);

			if (use_cartesian_PID) {

				vector_t vel_control = stepCartesianVelocityPID(veh_state, fixed_reference, true);
				set_point = setVelocityControl(fixed_reference, vel_control);

			} else {

				set_point = setNanVelocity(fixed_reference);

			}

			state_ = hovering;

		//} else if (fly_along_cable_distance <= 0) {

		//	// debug fly along cable distance is zero, aborting request, going to state hovering under cable
		//	RCLCPP_DEBUG(this->get_logger(), "fly along cable distance is zero, aborting request, going to state hovering under cable");

		//	notifyCurrentRequest(success);

		//	clearPlannedTrajectory();

		//	target_cable_cnt = updateTargetCablePose(veh_state) ? target_cable_cnt : target_cable_cnt-1;

		//	fixed_reference = setZeroVelocity(loadTargetUnderCableState());

		//	setTrajectoryTarget(fixed_reference);

		//	if (use_cartesian_PID) {

		//		vector_t vel_control = stepCartesianVelocityPID(veh_state, fixed_reference, true);
		//		set_point = setVelocityControl(fixed_reference, vel_control);

		//	} else {

		//		set_point = setNanVelocity(fixed_reference);

		//	}
		//	
		//	state_ = hovering_under_cable;

		} else if (currentRequestIsCancelled(if_match, if_match)) {

			// debug current request is cancelled, aborting request, going to state hovering under cable
			RCLCPP_DEBUG(this->get_logger(), "current request is cancelled, aborting request, going to state hovering under cable");

			clearPlannedTrajectory();

			target_cable_cnt = updateTargetCablePose(veh_state) ? target_cable_cnt : target_cable_cnt-1;

			fixed_reference = setZeroVelocity(loadTargetUnderCableState());

			setTrajectoryTarget(fixed_reference);

			if (use_cartesian_PID) {

				vector_t vel_control = stepCartesianVelocityPID(veh_state, fixed_reference, true);
				set_point = setVelocityControl(fixed_reference, vel_control);

			} else {

				set_point = setNanVelocity(fixed_reference);

			}
			
			state_ = hovering_under_cable;

		} else if (reachedPosition(veh_state, fixed_reference)) {

			// debug reached position, going to state hovering
			RCLCPP_DEBUG(this->get_logger(), "reached position, going to state hovering_under_cable");

			notifyCurrentRequest(success);

			clearPlannedTrajectory();

			target_cable_cnt = updateTargetCablePose(veh_state) ? target_cable_cnt : target_cable_cnt-1;

			fixed_reference = setZeroVelocity(loadTargetUnderCableState());

			setTrajectoryTarget(fixed_reference);

			if (use_cartesian_PID) {

				vector_t vel_control = stepCartesianVelocityPID(veh_state, fixed_reference, true);
				set_point = setVelocityControl(fixed_reference, vel_control);

			} else {

				set_point = setNanVelocity(fixed_reference);

			}
			
			state_ = hovering_under_cable;

		// } else {

		// 	// debug current request is not cancelled, continuing request
		// 	RCLCPP_DEBUG(this->get_logger(), "current request is not cancelled, continuing request");

		// 	rejectPendingRequest();

		// 	target_cable_cnt = updateTargetCablePose(veh_state, cable_id) ? target_cable_cnt : target_cable_cnt-1;

		// 	prev_fly_along_cable_state = fly_along_cable_state;
		// 	fly_along_cable_state = loadTargetFlyAlongCableState(
		// 		fly_along_cable_velocity,
		// 		fly_along_cable_distance,
		// 		fly_along_cable_inverse_direction,
		// 		prev_fly_along_cable_state,
		// 		false
		// 	);

		// 	point_t p0(
		// 		prev_fly_along_cable_state(0),
		// 		prev_fly_along_cable_state(1),
		// 		prev_fly_along_cable_state(2)
		// 	);

		// 	point_t p1(
		// 		fly_along_cable_state(0),
		// 		fly_along_cable_state(1),
		// 		fly_along_cable_state(2)
		// 	);

		// 	fly_along_cable_distance -= (p1-p0).norm();

		// 	set_point = fly_along_cable_state;

		// 	// debug fly along cable distance: %f, setpoint: %f, %f, %f, %f
		// 	RCLCPP_DEBUG(this->get_logger(), "fly along cable distance: %f, setpoint: %f, %f, %f, %f", fly_along_cable_distance, set_point(0), set_point(1), set_point(2), set_point(3));

		// }

		} else {

			target_cable_cnt = updateTargetCablePose(veh_state, cable_id) ? target_cable_cnt : target_cable_cnt-1;

			fixed_reference = setZeroVelocity(loadTargetFlyAlongCableState(false));

			setTrajectoryTarget(fixed_reference);

			if (direct_target_setpoint || withinDirectTargetDistance(veh_state, fixed_reference)) {

				if (!direct_target_setpoint){

					// RCLCPP_INFO(this->get_logger(), "Using direct target setpoint");
					// print fixed reference and vehicle state
					// RCLCPP_INFO(this->get_logger(), "fixed reference: %f, %f, %f, %f", fixed_reference(0), fixed_reference(1), fixed_reference(2), fixed_reference(3));
					// RCLCPP_INFO(this->get_logger(), "vehicle state: %f, %f, %f, %f", veh_state(0), veh_state(1), veh_state(2), veh_state(3));

				};

				direct_target_setpoint = true;

				// debug within direct target setpoint dist threshold, setting direct target setpoint: %f, %f, %f, %f
				RCLCPP_DEBUG(this->get_logger(), "within direct target setpoint dist threshold, setting direct target setpoint: %f, %f, %f, %f", set_point(0), set_point(1), set_point(2), set_point(3));
				
				if (use_cartesian_PID) {

					vector_t vel_control = stepCartesianVelocityPID(veh_state, fixed_reference, true);
					set_point = setVelocityControl(fixed_reference, vel_control);

				} else {

					set_point = setNanVelocity(fixed_reference);

				}

			} else {
				set_point = stepMPC(prev_veh_state, fixed_reference, true, false, fly_along_cable);
			}

			set_point = setZeroVelocity(veh_state);

			this->get_parameter("offboard_cnt_timeout", offboard_cnt);

			// debug set point is: %f, %f, %f, %f
			RCLCPP_DEBUG(this->get_logger(), "set point is: %f, %f, %f, %f", set_point(0), set_point(1), set_point(2), set_point(3));

			state_ = flying_along_cable;

		}

		break;

	case during_cable_landing:

		bool hover_under_cable_on_aborted_cable_landing;
		this->get_parameter("hover_under_cable_on_aborted_cable_landing", hover_under_cable_on_aborted_cable_landing);

		// debug in during cable landing
		RCLCPP_DEBUG(this->get_logger(), "in during cable landing");

		if (!offboard || !armed) {

			RCLCPP_DEBUG(this->get_logger(), "Going to init state because vehicle not in offboard");

			notifyCurrentRequest(fail);
			rejectPendingRequest();

			state_ = init;

		} else if (currentRequestIsCancelled(if_match, if_match)) {

			if (!hover_under_cable_on_aborted_cable_landing) {

				RCLCPP_DEBUG(this->get_logger(), "Going to state hovering, goal was cancelled");

				fixed_reference = setZeroVelocity(veh_state); // For explicability

				clearPlannedTrajectory(); // Also clear macro trajectory
				clearTargetCable();

				fixed_reference = setZeroVelocity(veh_state);

				setTrajectoryTarget(fixed_reference);

				vector_t vel_control = stepCartesianVelocityPID(veh_state, fixed_reference, true);
				set_point = setVelocityControl(fixed_reference, vel_control);

				state_ = hovering;

			} else {

				RCLCPP_INFO(this->get_logger(), "Going to state during cable takeoff");


				target_cable_cnt = updateTargetCablePose(veh_state) ? target_cable_cnt : target_cable_cnt-1;

				fixed_reference = loadTargetUnderCableState();

				setTrajectoryTarget(fixed_reference);

				state4_t target_cable = loadTargetCableState();

				// fixed_reference = target_cable;
				// fixed_reference(2) -= target_cable_distance;

				// fixed_reference = setZeroVelocity(fixed_reference);

				// setTrajectoryTarget(fixed_reference);

				direct_target_setpoint = false;

				if (direct_target_setpoint || withinDirectTargetDistance(veh_state, fixed_reference)) {
					direct_target_setpoint = true;
					if (use_cartesian_PID) {

						vector_t vel_control = stepCartesianVelocityPID(veh_state, fixed_reference, true);
						set_point = setVelocityControl(fixed_reference, vel_control);

					} else {

						set_point = setNanVelocity(fixed_reference);

					}
					// debug direct target setpoint: %f, %f, %f, %f
					RCLCPP_DEBUG(this->get_logger(), "direct target setpoint: %f, %f, %f, %f", set_point(0), set_point(1), set_point(2), set_point(3));
				} else {
					set_point = stepMPC(veh_state, fixed_reference, true, true, positional);
					// debug target setpoint: %f, %f, %f, %f
					RCLCPP_DEBUG(this->get_logger(), "target setpoint: %f, %f, %f, %f", set_point(0), set_point(1), set_point(2), set_point(3));
				}
				// set_point = setPointSafetyMarginTruncate(set_point, veh_state, target_cable);

				state_ = during_cable_takeoff;

			}

		} else if (reachedPosition(veh_state, fixed_reference)) {

			RCLCPP_DEBUG(this->get_logger(), "Reached cable, going to state on_cable_armed");

			notifyCurrentRequest(success);

			clearPlannedTrajectory();

			fixed_reference = setZeroVelocity(fixed_reference);

			for (int i = 0; i < 4; i++) {

				set_point(i) = NAN;
				set_point(i+4) = 0;
				set_point(i+8) = NAN;

			}

			state_ = on_cable_armed;

		} else if (target_cable_cnt == 0) {

			RCLCPP_DEBUG(this->get_logger(), "Target cable counter is zero, aborting request, going to state hovering");

			notifyCurrentRequest(fail);

			clearPlannedTrajectory();
			clearTargetCable();

			fixed_reference = setZeroVelocity(veh_state);

			setTrajectoryTarget(fixed_reference);

			vector_t vel_control = stepCartesianVelocityPID(veh_state, fixed_reference, true);
			set_point = setVelocityControl(fixed_reference, vel_control);

			state_ = hovering;

		} else {

			RCLCPP_DEBUG(this->get_logger(), "Evaluating if within safety margins");

			rejectPendingRequest();

			target_cable_cnt = updateTargetCablePose(veh_state) ? target_cable_cnt : target_cable_cnt-1;

			state4_t target_cable = loadTargetCableState();

			if (withinSafetyMargins(veh_state, target_cable)) {

				RCLCPP_DEBUG(this->get_logger(), "Within safety margins, stepping MPC");

				fixed_reference = setZeroVelocity(target_cable);

				setTrajectoryTarget(fixed_reference);

				if (direct_target_setpoint || withinDirectTargetDistance(veh_state, fixed_reference)) {

					// if (!direct_target_setpoint) RCLCPP_INFO(this->get_logger(), "Using direct target setpoint");

					direct_target_setpoint = true;

					set_point = fixed_reference;
					// debug direct target setpoint: %f, %f, %f, %f
					RCLCPP_DEBUG(this->get_logger(), "direct target setpoint: %f, %f, %f, %f", set_point(0), set_point(1), set_point(2), set_point(3));

				} else {
					set_point = stepMPC(veh_state, fixed_reference, true, false, cable_landing);
					// debug target setpoint: %f, %f, %f, %f
					RCLCPP_DEBUG(this->get_logger(), "target setpoint: %f, %f, %f, %f", set_point(0), set_point(1), set_point(2), set_point(3));
				}
				set_point = setPointSafetyMarginTruncate(set_point, veh_state, loadTargetCableState());

			} else {

				RCLCPP_DEBUG(this->get_logger(), "Not within safety margins, cancelling request");

				notifyCurrentRequest(cancel);

				if (!hover_under_cable_on_aborted_cable_landing) {

					RCLCPP_DEBUG(this->get_logger(), "Going to state hovering");

					clearPlannedTrajectory();
					clearTargetCable();

					fixed_reference = setZeroVelocity(veh_state);

					setTrajectoryTarget(fixed_reference);

					vector_t vel_control = stepCartesianVelocityPID(veh_state, fixed_reference, true);
					set_point = setVelocityControl(fixed_reference, vel_control);

					state_ = hovering;

				} else {

					RCLCPP_DEBUG(this->get_logger(), "Going to state during cable takeoff");


					target_cable_cnt = updateTargetCablePose(veh_state) ? target_cable_cnt : target_cable_cnt-1;

					fixed_reference = loadTargetUnderCableState();

					setTrajectoryTarget(fixed_reference);


					// state4_t target_cable = loadTargetCableState();

					// fixed_reference = target_cable;
					// fixed_reference(2) -= target_cable_distance;

					// fixed_reference = setZeroVelocity(fixed_reference);

					// setTrajectoryTarget(fixed_reference);

					direct_target_setpoint = false;

					if (direct_target_setpoint || withinDirectTargetDistance(veh_state, fixed_reference)) {
						direct_target_setpoint = true;
						if (use_cartesian_PID) {

							vector_t vel_control = stepCartesianVelocityPID(veh_state, fixed_reference, true);
							set_point = setVelocityControl(fixed_reference, vel_control);

						} else {

							set_point = setNanVelocity(fixed_reference);

						}
						// debug direct target setpoint: %f, %f, %f, %f
						RCLCPP_DEBUG(this->get_logger(), "direct target setpoint: %f, %f, %f, %f", set_point(0), set_point(1), set_point(2), set_point(3));
					} else {
						set_point = stepMPC(setZeroVelocity(prev_veh_state), fixed_reference, true, true, positional);
						// debug target setpoint: %f, %f, %f, %f
						RCLCPP_DEBUG(this->get_logger(), "target setpoint: %f, %f, %f, %f", set_point(0), set_point(1), set_point(2), set_point(3));
					}
					// set_point = setPointSafetyMarginTruncate(set_point, veh_state, target_cable);

					state_ = during_cable_takeoff;

				}
			}
		}

		break;

	case on_cable_armed:

		// debug in on cable armed
		RCLCPP_DEBUG(this->get_logger(), "in on cable armed");

		if (!offboard || !armed) {

			RCLCPP_DEBUG(this->get_logger(), "Going to init state because vehicle not in offboard");

			rejectPendingRequest();

			state_ = init;

		} else if (tryPendingRequest(cable_takeoff_request, if_match, if_match)) {

			RCLCPP_DEBUG(this->get_logger(), "Going to state during cable takeoff");

			cable_takeoff_request_params_t *request_params = (cable_takeoff_request_params_t *)request.request_params;
			target_cable_distance = request_params->target_cable_distance;
			
			delete request.request_params;

			fixed_reference = veh_state;
			fixed_reference(2) -= target_cable_distance;

			fixed_reference = setZeroVelocity(fixed_reference);

			setTrajectoryTarget(fixed_reference);

			state4_t target_cable = loadTargetCableState();

			if (direct_target_setpoint || withinDirectTargetDistance(veh_state, fixed_reference)) {
				direct_target_setpoint = true;
				set_point = fixed_reference;
				// debug direct target setpoint: %f, %f, %f, %f
				RCLCPP_DEBUG(this->get_logger(), "direct target setpoint: %f, %f, %f, %f", set_point(0), set_point(1), set_point(2), set_point(3));
			} else {
				set_point = stepMPC(veh_state, fixed_reference, true, true, positional);
				// debug target setpoint: %f, %f, %f, %f
				RCLCPP_DEBUG(this->get_logger(), "target setpoint: %f, %f, %f, %f", set_point(0), set_point(1), set_point(2), set_point(3));
			}
			// set_point = setPointSafetyMarginTruncate(set_point, veh_state, target_cable);

			state_ = during_cable_takeoff;

		} else if (tryPendingRequest(disarm_on_cable_request, if_match, if_match)) {

			RCLCPP_DEBUG(this->get_logger(), "Going to state disarming on cable");

			for (int i = 0; i < 3; i++) {

				disarm_on_cable_initial_position(i) = veh_state(i);

			}

			for (int i = 0; i < 4; i++) {

				set_point(i) = fixed_reference(i);
				set_point(i+4) = 0;
				set_point(i+8) = NAN;

			}

			this->get_parameter("land_cnt_timeout", land_cnt);

			disarmOnCable();

			state_ = disarming_on_cable;

		} else {

			rejectPendingRequest();

			for (int i = 0; i < 4; i++) {

				set_point(i) = fixed_reference(i);
				set_point(i+4) = 0;
				set_point(i+8) = NAN;

			}

			// set_point(6) = 0.01;

			// debug stay on cable, setpoint: %f, %f, %f, %f
			RCLCPP_DEBUG(this->get_logger(), "stay on cable, setpoint: %f, %f, %f, %f", set_point(0), set_point(1), set_point(2), set_point(3));
		}

		break;

	case disarming_on_cable:

		// debug in disarming on cable
		RCLCPP_DEBUG(this->get_logger(), "in disarming on cable state");

		float disarming_on_cable_max_descend_distance;
		this->get_parameter("disarming_on_cable_max_descend_distance", disarming_on_cable_max_descend_distance);

		if (disarm_on_cable_initial_position(2) - veh_state(2) >= abs(disarming_on_cable_max_descend_distance)) {

			RCLCPP_DEBUG(this->get_logger(), "Going to state during cable takeoff");

			notifyCurrentRequest(fail);

			target_cable_cnt = updateTargetCablePose(veh_state) ? target_cable_cnt : target_cable_cnt-1;

			fixed_reference = loadTargetUnderCableState();

			setTrajectoryTarget(fixed_reference);


			// state4_t target_cable = loadTargetCableState();

			// fixed_reference = target_cable;
			// fixed_reference(2) -= target_cable_distance;

			// fixed_reference = setZeroVelocity(fixed_reference);

			// setTrajectoryTarget(fixed_reference);

			direct_target_setpoint = false;

			if (direct_target_setpoint || withinDirectTargetDistance(veh_state, fixed_reference)) {
				direct_target_setpoint = true;
				if (use_cartesian_PID) {

					vector_t vel_control = stepCartesianVelocityPID(veh_state, fixed_reference, true);
					set_point = setVelocityControl(fixed_reference, vel_control);

				} else {

					set_point = setNanVelocity(fixed_reference);

				}
				// debug direct target setpoint: %f, %f, %f, %f
				RCLCPP_DEBUG(this->get_logger(), "direct target setpoint: %f, %f, %f, %f", set_point(0), set_point(1), set_point(2), set_point(3));
			} else {
				set_point = stepMPC(setZeroVelocity(prev_veh_state), fixed_reference, true, true, positional);
				// debug target setpoint: %f, %f, %f, %f
				RCLCPP_DEBUG(this->get_logger(), "target setpoint: %f, %f, %f, %f", set_point(0), set_point(1), set_point(2), set_point(3));
			}
			// set_point = setPointSafetyMarginTruncate(set_point, veh_state, target_cable);

			state_ = during_cable_takeoff;

		} else if (!armed) {

			// Debug going to state on cable disarmed
			RCLCPP_DEBUG(this->get_logger(), "Going to state on cable disarmed");

			notifyCurrentRequest(success);

			state_ = on_cable_disarmed;

		} else {

			bool disarm_on_cable_timeout = false;

			if (is_disarming_on_cable_by_thrust_) {

				double seconds_elapsed = (rclcpp::Clock().now() - disarm_on_cable_thrust_start_time_).seconds();

				double disarm_on_cable_thrust_decrease_time_s, disarm_on_cable_thrust_wait_for_disarm_time_s;

				this->get_parameter("disarm_on_cable_thrust_decrease_time_s", disarm_on_cable_thrust_decrease_time_s);
				this->get_parameter("disarm_on_cable_thrust_wait_for_disarm_time_s", disarm_on_cable_thrust_wait_for_disarm_time_s);

				disarm_on_cable_timeout = seconds_elapsed > disarm_on_cable_thrust_decrease_time_s + disarm_on_cable_thrust_wait_for_disarm_time_s;

			} else {

				disarm_on_cable_timeout = disarm_on_cable_cnt <= 0;

			}


			// if (disarm_on_cable_cnt <= 0) {
			if (disarm_on_cable_timeout) {

				RCLCPP_DEBUG(this->get_logger(), "Could not disarm, going to state on_cable_armed");

				notifyCurrentRequest(fail);

				clearPlannedTrajectory();

				fixed_reference = setZeroVelocity(fixed_reference);

				for (int i = 0; i < 4; i++) {

					set_point(i) = NAN;
					set_point(i+4) = 0;
					set_point(i+8) = NAN;

				}

				state_ = on_cable_armed;

			} else {
				
				rejectPendingRequest();

				for (int i = 0; i < 4; i++) {

					set_point(i) = fixed_reference(i);
					set_point(i+4) = 0;
					set_point(i+8) = NAN;

				}

				land_cnt--;

				// set_point(6) = 0.01;

				// debug disarming on cable, setpoint: %f, %f, %f, %f
				RCLCPP_DEBUG(this->get_logger(), "disarming on cable, setpoint: %f, %f, %f, %f", set_point(0), set_point(1), set_point(2), set_point(3));

			}
		}

		break;

	case on_cable_disarmed:

		// debug in on cable disarmed
		RCLCPP_DEBUG(this->get_logger(), "in on cable disarmed state");

		if (tryPendingRequest(arm_on_cable_request, if_match, if_match)) {

			// Debug arm on cable request, going to state arming on cable
			RCLCPP_DEBUG(this->get_logger(), "arm on cable request, going to state arming on cable");

			arm();

			setTrajectoryTarget(fixed_reference);

			set_point = setNanVelocity(fixed_reference);

			this->get_parameter("arm_cnt_timeout", arm_cnt);

			state_ = arming_on_cable;

			// debug set trajectory target: %f %f %f %f
			RCLCPP_DEBUG(this->get_logger(), "set trajectory target: %f %f %f %f", fixed_reference(0), fixed_reference(1), fixed_reference(2), fixed_reference(3));

		} else  {
			
			rejectPendingRequest();

		}

		break;

	case arming_on_cable:

		// debug in arming on cable
		RCLCPP_DEBUG(this->get_logger(), "in arming on cable");

		if (arm_cnt <= 0 && !armed) {

			// debug arm cnt zero and not armed, going to state on cable disarmed
			RCLCPP_DEBUG(this->get_logger(), "arm cnt zero and not armed, going to state on cable disarmed");

			notifyCurrentRequest(fail);

			state_ = on_cable_disarmed;

		} else if (armed) {

			// debug armed, going to state setting offboard on cable
			RCLCPP_DEBUG(this->get_logger(), "armed, going to state setting offboard on cable");

			set_point = setNanVelocity(fixed_reference);
			this->get_parameter("arming_on_cable_upwards_velocity", set_point(6));

			this->get_parameter("offboard_cnt_timeout", offboard_cnt);

			state_ = setting_offboard_on_cable;

		} else {

			arm_cnt--;

			rejectPendingRequest();

		}

		break;

	case setting_offboard_on_cable:

		// debug in setting offboard on cable
		RCLCPP_DEBUG(this->get_logger(), "in setting offboard on cable");

		if (offboard_cnt <= 0 && !offboard) {

			// debug offboard cnt zero and not offboard, going to state on cable disarmed
			RCLCPP_DEBUG(this->get_logger(), "offboard cnt zero and not offboard, going to state on cable disarmed");

			notifyCurrentRequest(fail);

			disarm();

			state_ = on_cable_disarmed;

		} else if (offboard) {

			// debug offboard, going to state on cable armed
			RCLCPP_DEBUG(this->get_logger(), "offboard, going to state on cable armed");

			notifyCurrentRequest(success);

			for (int i = 0; i < 4; i++) {

				set_point(i) = fixed_reference(i);
				set_point(i+4) = 0;
				set_point(i+8) = NAN;

			}

			this->get_parameter("arming_on_cable_upwards_velocity", set_point(6));

			state_ = on_cable_armed;

		} else {

			set_point = setNanVelocity(fixed_reference);
			this->get_parameter("arming_on_cable_upwards_velocity", set_point(6));

			offboard_cnt--;

			rejectPendingRequest();

		}

		break;

	case during_cable_takeoff:

		// debug in during cable takeoff
		RCLCPP_DEBUG(this->get_logger(), "in during cable takeoff");

		if (!offboard || !armed) {

			// debug Going to init state because vehicle not in offboard
			RCLCPP_DEBUG(this->get_logger(), "Going to init state because vehicle not in offboard");

			notifyCurrentRequest(fail);
			rejectPendingRequest();

			state_ = init;

		} else if (currentRequestIsCancelled(if_match, if_match)) {

			// debug Going to on cable armed state because request was cancelled
			RCLCPP_DEBUG(this->get_logger(), "Going to on hovering state because request was cancelled");

			fixed_reference = setNanVelocity(veh_state); // For explicability

			clearPlannedTrajectory(); // Also clear macro trajectory
			clearTargetCable();

			fixed_reference = setZeroVelocity(veh_state);

			setTrajectoryTarget(fixed_reference);

			// vector_t vel_control = stepCartesianVelocityPID(veh_state, fixed_reference, true);
			// set_point = setVelocityControl(fixed_reference, vel_control);

			set_point = setNanVelocity(fixed_reference);

			state_ = hovering;

		} else if (reachedPosition(veh_state, fixed_reference)) {

			// debug Going to on hovering state because request was completed
			RCLCPP_DEBUG(this->get_logger(), "Going to on hovering state because request was completed");

			notifyCurrentRequest(success);

			clearPlannedTrajectory();

			target_cable_cnt = updateTargetCablePose(veh_state) ? target_cable_cnt : target_cable_cnt-1;

			fixed_reference = setZeroVelocity(loadTargetUnderCableState());

			setTrajectoryTarget(fixed_reference);

			if (use_cartesian_PID) {
				vector_t vel_control = stepCartesianVelocityPID(veh_state, fixed_reference, true);
				set_point = setVelocityControl(fixed_reference, vel_control);
			} else {
				set_point = setNanVelocity(fixed_reference);
			}
			
			state_ = hovering_under_cable;

		} else {

			// debug during cable takeoff
			RCLCPP_DEBUG(this->get_logger(), "during cable takeoff");

			rejectPendingRequest();

			state4_t target_cable = loadTargetCableState();

			target_cable_cnt = updateTargetCablePose(veh_state) ? target_cable_cnt : target_cable_cnt-1;

			fixed_reference = loadTargetUnderCableState();

			setTrajectoryTarget(fixed_reference);

			RCLCPP_DEBUG(this->get_logger(), "State: %f, %f, %f", veh_state(0), veh_state(1), veh_state(2));
			RCLCPP_DEBUG(this->get_logger(), "Fixed reference: %f, %f, %f", fixed_reference(0), fixed_reference(1), fixed_reference(2));

			if (direct_target_setpoint || withinDirectTargetDistance(veh_state, fixed_reference)) {

				direct_target_setpoint = true;
			
				vector_t vel_control = stepCartesianVelocityPID(veh_state, fixed_reference, true);
				set_point = setVelocityControl(fixed_reference, vel_control);
				// debug direct target setpoint: %f, %f, %f, %f
				RCLCPP_DEBUG(this->get_logger(), "direct target setpoint: %f, %f, %f, %f", set_point(0), set_point(1), set_point(2), set_point(3));
			} else {
				set_point = stepMPC(veh_state, fixed_reference, true, false, positional);
				// set_point = setNanVelocity(fixed_reference);
				// debug target setpoint: %f, %f, %f, %f
				RCLCPP_DEBUG(this->get_logger(), "MPC target setpoint: %f, %f, %f, %f", set_point(0), set_point(1), set_point(2), set_point(3));
			}
			// set_point = setPointSafetyMarginTruncate(set_point, veh_state, target_cable);

			// debug truncated setpoint: %f, %f, %f, %f
			RCLCPP_DEBUG(this->get_logger(), "truncated setpoint: %f, %f, %f, %f", set_point(0), set_point(1), set_point(2), set_point(3));

		}

		break;
	
	}

	publishControlState();
	publishPlannedTrajectory();
	publishTargetCableId();

	publishOffboardControlMode();
	if (offboard && always_hover_in_offboard && state_ != hovering)
		set_point = always_hover_state;

	if (is_disarming_on_cable_by_thrust_) {

		publishActuatorSetpoints();

	} else {

		publishTrajectorySetpoint(set_point);

	}

	publishSetpointPose(set_point);

}

void TrajectoryController::odometryCallback(px4_msgs::msg::VehicleOdometry::SharedPtr msg) {

	quat_t q(
		msg->q[0],
		msg->q[1],
		msg->q[2],
		msg->q[3]
	);

	vector_t ang_vel(
		msg->angular_velocity[0],
		msg->angular_velocity[1],
		msg->angular_velocity[2]
	);

	vector_t pos(
		msg->position[0],
		msg->position[1],
		msg->position[2]
	);

	vector_t vel(
		msg->velocity[0],
		msg->velocity[1],
		msg->velocity[2]
	);

	int controller_period_ms;
	this->get_parameter("controller_period_ms", controller_period_ms);

	odometry_mutex_.lock(); {

		odom_q_ = q;
		odom_ang_vel_ = ang_vel;
		odom_pos_ = pos;
		vector_t last_vel = odom_vel_;
		odom_vel_ = vel;
		odom_acc_ = (vel - last_vel) * 1000 / controller_period_ms;

	} odometry_mutex_.unlock();

}

void TrajectoryController::powerlineCallback(iii_interfaces::msg::Powerline::SharedPtr msg) {

	powerline_mutex_.lock(); {

		powerline_ = *msg;

	} powerline_mutex_.unlock();

}

void TrajectoryController::homePositionCallback(px4_msgs::msg::HomePosition::SharedPtr msg) {

	bool update_home;
	this->get_parameter("update_home_position", update_home);

	home_position_mutex_.lock(); {

		if (!home_position_set_) {

			RCLCPP_INFO(this->get_logger(), "Initiated home position");

			home_position_ = *msg;

			home_position_set_ = true;

			if (update_home)
				setHomePosition(home_position_);

		} else {

			if (update_home)
				setHomePositionIfChanged(home_position_, *msg);

		}

	} home_position_mutex_.unlock();

}

void TrajectoryController::setHomePosition(px4_msgs::msg::HomePosition new_home) {

	RCLCPP_DEBUG(this->get_logger(), "new home: %f, %f, %f, %f, %f, %f, %f", new_home.lat, new_home.lon, new_home.alt, new_home.x, new_home.y, new_home.z, new_home.yaw);

	publishVehicleCommand(
		px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_HOME,
		0,
		0,
		0,
		0,
		new_home.lat,
		new_home.lon,
		new_home.alt
	);

}

void TrajectoryController::setHomePositionIfChanged(px4_msgs::msg::HomePosition old_home, px4_msgs::msg::HomePosition new_home) {

	bool home_different = false;

	home_different |= old_home.lat != new_home.lat;
	home_different |= old_home.lon != new_home.lon;
	home_different |= old_home.alt != new_home.alt;

	if (home_different) {

		RCLCPP_INFO(this->get_logger(), "Home position changed, updating home position to old value");
		// RCLCPP_INFO old home:
		RCLCPP_DEBUG(this->get_logger(), "old home: %f, %f, %f, %f, %f, %f, %f", old_home.lat, old_home.lon, old_home.alt, old_home.x, old_home.y, old_home.z, old_home.yaw);
		// RCLCPP_INFO new home:

		setHomePosition(old_home);

	}
}

bool TrajectoryController::isOffboard() {

	return nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;

}

bool TrajectoryController::isArmed() {

	return arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;

}

void TrajectoryController::setModeOffboard() {

	publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

}

void TrajectoryController::land() {

	publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);

}

/**
 * @brief Send a command to Arm the vehicle
 */
void TrajectoryController::arm() {

	//armed = true;
	publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_DEBUG(this->get_logger(), "Arm command send");

}


/**
 * @brief Send a command to Disarm the vehicle
 */
void TrajectoryController::disarm() {

	//armed = false;
	publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
	RCLCPP_DEBUG(this->get_logger(), "Disarm command send");

}

void TrajectoryController::disarmOnCable() {

	std::string disarm_on_cable_mode;
	this->get_parameter("disarm_on_cable_mode", disarm_on_cable_mode);

	if (disarm_on_cable_mode == "thrust") {

		disarm_on_cable_thrust_start_time_ = this->get_clock()->now();
		is_disarming_on_cable_by_thrust_ = true;

		return;

	} else if (disarm_on_cable_mode != "land") {

		RCLCPP_ERROR(this->get_logger(), "Invalid disarm on cable mode: %s, falling back to disarming by issuing land command", disarm_on_cable_mode.c_str());

	}

	land();

}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void TrajectoryController::publishVehicleCommand(uint16_t command, float param1,
					      float param2, float param3, float param4,
					      float param5, float param6,
					      float param7) const {

	px4_msgs::msg::VehicleCommand msg{};
	msg.timestamp = timestamp_.load();
	msg.param1 = param1;
	msg.param2 = param2;
	msg.param3 = param3;
	msg.param4 = param4;
	msg.param5 = param5;
	msg.param6 = param6;
	msg.param7 = param7;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	vehicle_command_pub_->publish(msg);

}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void TrajectoryController::publishOffboardControlMode() {
	px4_msgs::msg::OffboardControlMode msg{};
	msg.timestamp = timestamp_.load();

	if (is_disarming_on_cable_by_thrust_ && state_ == disarming_on_cable) {

		msg.position = false;
		msg.velocity = false;
		msg.acceleration = false;
		msg.attitude = false;
		msg.body_rate = false;	
		msg.actuator = true;

	} else {

		is_disarming_on_cable_by_thrust_ = false;

		if (state_ == on_cable_armed) {
			msg.position = false;
			// msg.velocity = false;
			msg.acceleration = false;
		} else {
			msg.position = true;
			// msg.velocity = true;
			msg.acceleration = true;
		}
		msg.position = true;
		msg.velocity = true;
		msg.attitude = false;
		msg.body_rate = false;	
		msg.actuator = false;

	}

	offboard_control_mode_pub_->publish(msg);

}

void TrajectoryController::publishControlState() {

	iii_interfaces::msg::ControlState msg;

	switch (state_) {

	default:
	case init:
		
		msg.state = msg.CONTROL_STATE_INIT;
		break;

	case on_ground_non_offboard:

		msg.state = msg.CONTROL_STATE_ON_GROUND_NON_OFFBOARD;
		break;

	case in_flight_non_offboard:

		msg.state = msg.CONTROL_STATE_IN_FLIGHT_NON_OFFBOARD;
		break;

	case arming:

		msg.state = msg.CONTROL_STATE_ARMING;
		break;

	case setting_offboard:

		msg.state = msg.CONTROL_STATE_SETTING_OFFBOARD;
		break;

	case taking_off:

		msg.state = msg.CONTROL_STATE_TAKING_OFF;
		break;

	case hovering:

		msg.state = msg.CONTROL_STATE_HOVERING;
		break;

	case in_positional_flight:

		msg.state = msg.CONTROL_STATE_IN_POSITIONAL_FLIGHT;
		break;

	case during_cable_landing:

		msg.state = msg.CONTROL_STATE_DURING_CABLE_LANDING;
		break;

	case on_cable_armed:

		msg.state = msg.CONTROL_STATE_ON_CABLE_ARMED;
		break;

	case during_cable_takeoff:

		msg.state = msg.CONTROL_STATE_DURING_CABLE_TAKEOFF;
		break;

	case hovering_under_cable:
		msg.state = msg.CONTROL_STATE_HOVERING_UNDER_CABLE;
		break;

	case flying_along_cable:

		msg.state = msg.CONTROL_STATE_FLYING_ALONG_CABLE;
		break;
	
	}

	control_state_pub_->publish(msg);

}

/**
 * @brief Publish a trajectory setpoint
 */
void TrajectoryController::publishTrajectorySetpoint(state4_t set_point) const {

	static rotation_matrix_t R_NED_to_body_frame = eulToR(orientation_t(M_PI, 0, 0));

	vector_t pos(
		set_point(0),
		set_point(1),
		set_point(2)
	);

	pos = R_NED_to_body_frame.transpose() * pos;

	vector_t vel(
		set_point(4),
		set_point(5),
		set_point(6)
	);

	vel = R_NED_to_body_frame.transpose() * vel;

	vector_t acc(
		set_point(8),
		set_point(9),
		set_point(10)
	);

	acc = R_NED_to_body_frame.transpose() * acc;

	float yaw = -set_point(3);

	float yaw_rate = -set_point(7);

	//float yaw_acc = -set_point(11);

	px4_msgs::msg::TrajectorySetpoint msg{};

	msg.timestamp = timestamp_.load();
	for (int i = 0; i < 3; i++) {

		//msg.position[i] = pos(i);
		//msg.velocity[i] = vel(i);
		msg.acceleration[i] = acc(i);
		// msg.acceleration[i] = NAN;
		msg.jerk[i] = NAN; //jerk(i);

	}

	msg.position[0] = pos(0);
	msg.position[1] = pos(1);
	msg.position[2] = pos(2);
	// msg.x = NAN;
	// msg.y = NAN;
	// msg.z = NAN;

	msg.velocity[0] = vel(0);
	msg.velocity[1] = vel(1);
	msg.velocity[2] = vel(2);
	// msg.vx = NAN;
	// msg.vy = NAN;
	// msg.vz = NAN;

	msg.yaw = yaw;
	msg.yawspeed = yaw_rate;

	// RCLCPP_INFO(this->get_logger(),  "Publishing trajectory setpoint: \n x: %f, y: %f, z: %f, yaw: %f, vx: %f, vy: %f, vz: %f, yawspeed: %f",
	// 	pos(0), pos(1), pos(2), yaw, vel(0), vel(1), vel(2), yaw_rate);
	//msg.acceleration	// in meters/sec^2
	//msg.jerk			// in meters/sec^3
	//msg.thrust		// normalized thrust vector in NED

	trajectory_setpoint_pub_->publish(msg);

}

void TrajectoryController::publishActuatorSetpoints() {

	double disarm_on_cable_thrust_initial, disarm_on_cable_thrust_decrease_time_s;
	this->get_parameter("disarm_on_cable_thrust_initial", disarm_on_cable_thrust_initial);
	this->get_parameter("disarm_on_cable_thrust_decrease_time_s", disarm_on_cable_thrust_decrease_time_s);

	double a = -disarm_on_cable_thrust_initial / disarm_on_cable_thrust_decrease_time_s;

	double seconds_elapsed = (this->get_clock()->now() - disarm_on_cable_thrust_start_time_).seconds();

	double thrust = a * seconds_elapsed + disarm_on_cable_thrust_initial;

	if (thrust > 1) {

		thrust = 1;

	} else if (thrust < 0) {

		thrust = 0;

	}

	px4_msgs::msg::VehicleThrustSetpoint thrust_msg{};
	thrust_msg.timestamp_sample = timestamp_.load();
	thrust_msg.timestamp = timestamp_.load();
	thrust_msg.xyz[0] = 0;
	thrust_msg.xyz[1] = 0;
	thrust_msg.xyz[2] = -thrust;

	px4_msgs::msg::VehicleTorqueSetpoint torque_msg{};
	torque_msg.timestamp_sample = timestamp_.load();
	torque_msg.timestamp = timestamp_.load();
	torque_msg.xyz[0] = 0;
	torque_msg.xyz[1] = 0;
	torque_msg.xyz[2] = 0;

	thrust_setpoint_pub_->publish(thrust_msg);
	torque_setpoint_pub_->publish(torque_msg);


}

void TrajectoryController::publishSetpointPose(state4_t set_point) {

	geometry_msgs::msg::PoseStamped msg;

	msg.header.stamp = this->get_clock()->now();
	msg.header.frame_id = "world";

	msg.pose.position.x = set_point(0);
	msg.pose.position.y = set_point(1);
	msg.pose.position.z = set_point(2);
 
	quat_t q = eulToQuat(orientation_t(0, 0, set_point(3)));

	msg.pose.orientation.w = q(0);
	msg.pose.orientation.x = q(1);
	msg.pose.orientation.y = q(2);
	msg.pose.orientation.z = q(3);

	setpoint_pose_pub_->publish(msg);

}

void TrajectoryController::publishPlannedTrajectory() {

	nav_msgs::msg::Path path = loadPlannedPath();
	nav_msgs::msg::Path macro_path = loadPlannedMacroPath();

	geometry_msgs::msg::PoseStamped target;

	if (path.poses.size() == 0) {

		target = loadVehiclePose();
		path.poses.push_back(target);

		if (state_ == arming || state_ == setting_offboard || state_ == taking_off || state_ == hovering_under_cable || state_ == flying_along_cable \
			|| state_ == during_cable_takeoff || state_ == during_cable_landing || state_ == on_cable_armed || state_ == hovering) {

			target = loadPlannedTarget();

		}

	} else {

		target = loadPlannedTarget();

	}

	macro_path.poses.push_back(target);

	planned_traj_pub_->publish(path);
	planned_macro_traj_pub_->publish(macro_path);
	planned_target_pub_->publish(target);

}

void TrajectoryController::publishTargetCableId() {

	auto msg = std::make_unique<std_msgs::msg::Int16>();

	// switch (state_) {

	// 	case in_positional_flight:
	// 		if (target_cable_id_ < 0)
	// 			break;
	// 	case hovering_under_cable:
	// 	case during_cable_landing:
	// 	case on_cable_armed:
	// 	case disarming_on_cable:
	// 	case on_cable_disarmed:
	// 	case arming_on_cable:
	// 	case setting_offboard_on_cable:
	// 	case during_cable_takeoff:
	// 		msg->data = target_cable_id_;
	// 		target_cable_id_pub_->publish(std::move(msg));
	// 		break;

	// 	default:
	// 		break;

	// }

	msg->data = target_cable_id_;
	target_cable_id_pub_->publish(std::move(msg));

}


state4_t TrajectoryController::loadVehicleState() {

	static rotation_matrix_t R_NED_to_body_frame = eulToR(orientation_t(M_PI, 0, 0));

	quat_t q;
	vector_t ang_vel;
	vector_t pos;
	vector_t vel;
	vector_t acc;

	odometry_mutex_.lock(); {

		q = odom_q_;
		ang_vel = odom_ang_vel_;
		pos = odom_pos_;
		vel = odom_vel_;
		acc = odom_acc_;

	} odometry_mutex_.unlock();

	pos = R_NED_to_body_frame * pos;
	vel = R_NED_to_body_frame * vel;

	orientation_t eul = quatToEul(q);

	state4_t vehicle_state;
	vehicle_state(0) = pos(0);
	vehicle_state(1) = pos(1);
	vehicle_state(2) = pos(2);
	vehicle_state(3) = -eul(2);
	vehicle_state(4) = vel(0);
	vehicle_state(5) = vel(1);
	vehicle_state(6) = vel(2);
	vehicle_state(7) = -ang_vel(2);
	vehicle_state(8) = acc(0);
	vehicle_state(9) = acc(1);
	vehicle_state(10) = acc(2);
	vehicle_state(11) = NAN;

	return vehicle_state;

}

geometry_msgs::msg::PoseStamped TrajectoryController::loadVehiclePose() {

	state4_t veh_state = loadVehicleState();

	geometry_msgs::msg::PoseStamped pose;

	pose.header.frame_id = "world";
	pose.header.stamp = this->get_clock()->now();

	pose.pose.position.x = veh_state(0);
	pose.pose.position.y = veh_state(1);
	pose.pose.position.z = veh_state(2);

	orientation_t eul(0,0,veh_state(3));
	quat_t quat = eulToQuat(eul);

	pose.pose.orientation.w = quat(0);
	pose.pose.orientation.x = quat(1);
	pose.pose.orientation.y = quat(2);
	pose.pose.orientation.z = quat(3);

	return pose;

}

nav_msgs::msg::Path TrajectoryController::loadPlannedPath() {

	std::vector<state4_t> trajectory_vec;

	planned_trajectory_mutex_.lock(); {

		trajectory_vec = planned_trajectory_;

	} planned_trajectory_mutex_.unlock();

	nav_msgs::msg::Path path;

	auto stamp = this->get_clock()->now();

	path.header.frame_id = "world";
	path.header.stamp = stamp;

	std::vector<geometry_msgs::msg::PoseStamped> poses_vec((std::size_t)trajectory_vec.size());

	for (int i = 0; i < trajectory_vec.size(); i++) {

		geometry_msgs::msg::PoseStamped pose;
		pose.header.frame_id = "world";
		pose.header.stamp = stamp;

		state4_t state = trajectory_vec[i];

		pose.pose.position.x = state(0);
		pose.pose.position.y = state(1);
		pose.pose.position.z = state(2);

		orientation_t eul(0,0,state(3));
		quat_t quat = eulToQuat(eul);

		pose.pose.orientation.w = quat(0);
		pose.pose.orientation.x = quat(1);
		pose.pose.orientation.y = quat(2);
		pose.pose.orientation.z = quat(3);

		poses_vec[i] = pose;

	}

	path.poses = poses_vec;

	return path;

}

nav_msgs::msg::Path TrajectoryController::loadPlannedMacroPath() {

	std::vector<state4_t> trajectory_vec;

	planned_trajectory_mutex_.lock(); {

		trajectory_vec = planned_macro_trajectory_;

	} planned_trajectory_mutex_.unlock();

	nav_msgs::msg::Path path;

	auto stamp = this->get_clock()->now();

	path.header.frame_id = "world";
	path.header.stamp = stamp;

	std::vector<geometry_msgs::msg::PoseStamped> poses_vec((std::size_t)trajectory_vec.size());

	for (int i = 0; i < trajectory_vec.size(); i++) {

		geometry_msgs::msg::PoseStamped pose;
		pose.header.frame_id = "world";
		pose.header.stamp = stamp;

		state4_t state = trajectory_vec[i];

		pose.pose.position.x = state(0);
		pose.pose.position.y = state(1);
		pose.pose.position.z = state(2);

		orientation_t eul(0,0,state(3));
		quat_t quat = eulToQuat(eul);

		pose.pose.orientation.w = quat(0);
		pose.pose.orientation.x = quat(1);
		pose.pose.orientation.y = quat(2);
		pose.pose.orientation.z = quat(3);

		poses_vec[i] = pose;

	}

	path.poses = poses_vec;

	return path;

}

geometry_msgs::msg::PoseStamped TrajectoryController::loadPlannedTarget() {

	state4_t target_cp;

	planned_trajectory_mutex_.lock(); {

		target_cp = trajectory_target_;

	} planned_trajectory_mutex_.unlock();

	geometry_msgs::msg::PoseStamped target_pose;

	target_pose.header.stamp = this->get_clock()->now();
	target_pose.header.frame_id = "world";

	target_pose.pose.position.x = target_cp(0);
	target_pose.pose.position.y = target_cp(1);
	target_pose.pose.position.z = target_cp(2);

	orientation_t eul(0,0,target_cp(3));
	quat_t quat = eulToQuat(eul);

	target_pose.pose.orientation.w = quat(0);
	target_pose.pose.orientation.x = quat(1);
	target_pose.pose.orientation.y = quat(2);
	target_pose.pose.orientation.z = quat(3);

	return target_pose;

}

state4_t TrajectoryController::loadTargetCableState() {

	state4_t cable_state;

	geometry_msgs::msg::Pose cable_pose;

	powerline_mutex_.lock(); {

		cable_pose = target_cable_pose_.pose;

	} powerline_mutex_.unlock();

	quat_t cable_quat(
		cable_pose.orientation.w,
		cable_pose.orientation.x,
		cable_pose.orientation.y,
		cable_pose.orientation.z
	);

	orientation_t cable_eul = quatToEul(cable_quat);

	geometry_msgs::msg::TransformStamped T_drone_to_cable_gripper = tf_buffer_->lookupTransform("cable_gripper", "drone", tf2::TimePointZero);

	quat_t q_drone_to_cable_gripper(
		T_drone_to_cable_gripper.transform.rotation.w,
		T_drone_to_cable_gripper.transform.rotation.x,
		T_drone_to_cable_gripper.transform.rotation.y,
		T_drone_to_cable_gripper.transform.rotation.z
	);

	orientation_t eul_drone_to_cable_gripper = quatToEul(q_drone_to_cable_gripper);

	float target_yaw;

	target_yaw_mutex_.lock(); {

		target_yaw = target_yaw_;

	} target_yaw_mutex_.unlock();

	if (target_yaw > M_PI) 
		target_yaw -= 2*M_PI;
	else if (target_yaw < -M_PI)
		target_yaw += 2*M_PI;

	float cable_gripper_target_yaw = cable_eul(2) + eul_drone_to_cable_gripper(2);

	if (cable_gripper_target_yaw > M_PI)
		cable_gripper_target_yaw -= 2*M_PI;
	else if (cable_gripper_target_yaw < -M_PI)
		cable_gripper_target_yaw += 2*M_PI;

	float best_yaw = cable_gripper_target_yaw;

	if (abs(best_yaw - target_yaw) > abs(cable_gripper_target_yaw-M_PI - target_yaw))
		best_yaw = cable_gripper_target_yaw - M_PI;

	if (abs(best_yaw - target_yaw) > abs(cable_gripper_target_yaw+M_PI-target_yaw))
		best_yaw = cable_state(3) + M_PI;

	cable_state(0) = cable_pose.position.x + T_drone_to_cable_gripper.transform.translation.x;
	cable_state(1) = cable_pose.position.y + T_drone_to_cable_gripper.transform.translation.y;
	cable_state(2) = cable_pose.position.z + T_drone_to_cable_gripper.transform.translation.z;
	cable_state(3) = best_yaw; //cable_eul(2) + eul_drone_to_cable_gripper(2);
	cable_state(4) = 0;
	cable_state(5) = 0;
	cable_state(6) = 0;
	cable_state(7) = 0;


	//cable_state(3) = best_yaw;

	return cable_state;

}

state4_t TrajectoryController::loadTargetUnderCableState() {

	state4_t under_cable_state;

	geometry_msgs::msg::Pose cable_pose;

	powerline_mutex_.lock(); {

		cable_pose = target_cable_pose_.pose;

	} powerline_mutex_.unlock();

	quat_t cable_quat(
		cable_pose.orientation.w,
		cable_pose.orientation.x,
		cable_pose.orientation.y,
		cable_pose.orientation.z
	);

	orientation_t cable_eul = quatToEul(cable_quat);

	geometry_msgs::msg::TransformStamped T_drone_to_cable_gripper = tf_buffer_->lookupTransform("cable_gripper", "drone", tf2::TimePointZero);

	quat_t q_drone_to_cable_gripper(
		T_drone_to_cable_gripper.transform.rotation.w,
		T_drone_to_cable_gripper.transform.rotation.x,
		T_drone_to_cable_gripper.transform.rotation.y,
		T_drone_to_cable_gripper.transform.rotation.z
	);

	orientation_t eul_drone_to_cable_gripper = quatToEul(q_drone_to_cable_gripper);

	float target_yaw;

	target_yaw_mutex_.lock(); {

		target_yaw = target_yaw_;

	} target_yaw_mutex_.unlock();

	if (target_yaw > M_PI) 
		target_yaw -= 2*M_PI;
	else if (target_yaw < -M_PI)
		target_yaw += 2*M_PI;

	float cable_gripper_target_yaw = cable_eul(2) + eul_drone_to_cable_gripper(2);

	if (cable_gripper_target_yaw > M_PI)
		cable_gripper_target_yaw -= 2*M_PI;
	else if (cable_gripper_target_yaw < -M_PI)
		cable_gripper_target_yaw += 2*M_PI;

	float best_yaw = cable_gripper_target_yaw;

	if (abs(best_yaw - target_yaw) > abs(cable_gripper_target_yaw-M_PI - target_yaw))
		best_yaw = cable_gripper_target_yaw - M_PI;

	if (abs(best_yaw - target_yaw) > abs(cable_gripper_target_yaw+M_PI-target_yaw))
		best_yaw = cable_gripper_target_yaw + M_PI;

	under_cable_state(0) = cable_pose.position.x;
	under_cable_state(1) = cable_pose.position.y;
	under_cable_state(2) = cable_pose.position.z - target_cable_distance_;
	under_cable_state(3) = best_yaw; 
	under_cable_state(4) = 0;
	under_cable_state(5) = 0;
	under_cable_state(6) = 0;
	under_cable_state(7) = 0;


	//cable_state(3) = best_yaw;

	return under_cable_state;

}

//state4_t TrajectoryController::loadTargetFlyAlongCableState(float velocity, float distance_left, 
//					bool inverse_direction, state4_t prev_fly_along_cable_state, bool first) {
//
//	float max_acceleration;
//	this->get_parameter("max_acceleration", max_acceleration);
//
//	int controller_period_ms;
//	this->get_parameter("controller_period_ms", controller_period_ms);
//
//	float dt = controller_period_ms/1000.0;
//
//	state4_t fly_along_cable_state;
//
//	geometry_msgs::msg::PoseStamped cable_pose;
//
//	powerline_mutex_.lock(); {
//
//		cable_pose = target_cable_pose_;
//
//	} powerline_mutex_.unlock();
//
//	quat_t cable_quat(
//		cable_pose.pose.orientation.w,
//		cable_pose.pose.orientation.x,
//		cable_pose.pose.orientation.y,
//		cable_pose.pose.orientation.z
//	);
//
//	orientation_t cable_eul = quatToEul(cable_quat);
//
//	geometry_msgs::msg::TransformStamped T_drone_to_cable_gripper = tf_buffer_->lookupTransform("cable_gripper", "drone", tf2::TimePointZero);
//
//	quat_t q_drone_to_cable_gripper(
//		T_drone_to_cable_gripper.transform.rotation.w,
//		T_drone_to_cable_gripper.transform.rotation.x,
//		T_drone_to_cable_gripper.transform.rotation.y,
//		T_drone_to_cable_gripper.transform.rotation.z
//	);
//
//	orientation_t eul_drone_to_cable_gripper = quatToEul(q_drone_to_cable_gripper);
//
//	float target_yaw;
//
//	target_yaw_mutex_.lock(); {
//
//		target_yaw = target_yaw_;
//
//	} target_yaw_mutex_.unlock();
//
//	if (target_yaw > M_PI) 
//		target_yaw -= 2*M_PI;
//	else if (target_yaw < -M_PI)
//		target_yaw += 2*M_PI;
//
//	float cable_gripper_target_yaw = cable_eul(2) + eul_drone_to_cable_gripper(2);
//
//	if (cable_gripper_target_yaw > M_PI)
//		cable_gripper_target_yaw -= 2*M_PI;
//	else if (cable_gripper_target_yaw < -M_PI)
//		cable_gripper_target_yaw += 2*M_PI;
//
//	float best_yaw = cable_gripper_target_yaw;
//
//	if (abs(best_yaw - target_yaw) > abs(cable_gripper_target_yaw-M_PI - target_yaw))
//		best_yaw = cable_gripper_target_yaw - M_PI;
//
//	if (abs(best_yaw - target_yaw) > abs(cable_gripper_target_yaw+M_PI-target_yaw))
//		best_yaw = cable_gripper_target_yaw + M_PI;
//
//	point_t pm1, p0, p1, p2;
//	vector_t vm1, v0, v1, v2, am1, a0, a1, a2;
//
//	vector_t unit_x(1,0,0);
//	vector_t unit_y(0,1,0);
//	
//	geometry_msgs::msg::PoseStamped cable_pose_W;
//	cable_pose_W.header.frame_id = cable_pose.header.frame_id;
//	cable_pose_W.pose = cable_pose.pose;
//
//	cable_pose_W = tf_buffer_->transform(cable_pose_W, "world");
//
//	quat_t cable_quat_W(
//		cable_pose_W.pose.orientation.w,
//		cable_pose_W.pose.orientation.x,
//		cable_pose_W.pose.orientation.y,
//		cable_pose_W.pose.orientation.z
//	);
//
//	rotation_matrix_t R_W_to_cable = quatToMat(cable_quat_W);
//
//	vector_t cable_x_W = R_W_to_cable*unit_x;
//	vector_t cable_y_W = R_W_to_cable*unit_y;
//
//	if (inverse_direction) {
//		cable_x_W = -cable_x_W;
//		cable_y_W = -cable_y_W;
//	}
//
//	RCLCPP_INFO(this->get_logger(), "cable_x_W: %f, %f, %f", cable_x_W(0), cable_x_W(1), cable_x_W(2));
//	RCLCPP_INFO(this->get_logger(), "cable_y_W: %f, %f, %f", cable_y_W(0), cable_y_W(1), cable_y_W(2));
//
//	plane_t cable_plane_W = {
//		.p = point_t(
//			cable_pose_W.pose.position.x, 
//			cable_pose_W.pose.position.y, 
//			cable_pose_W.pose.position.z
//		),
//		.normal = cable_y_W
//	};
//
//	vector_t v_max = cable_x_W*velocity;
//	vector_t a_max = cable_x_W*max_acceleration;
//
//	RCLCPP_INFO(this->get_logger(), "v_max: %f, %f, %f", v_max(0), v_max(1), v_max(2));
//	RCLCPP_INFO(this->get_logger(), "a_max: %f, %f, %f", a_max(0), a_max(1), a_max(2));
//
//	pm1(0) = prev_fly_along_cable_state(0);
//	pm1(1) = prev_fly_along_cable_state(1);
//	pm1(2) = prev_fly_along_cable_state(2);
//
//	vm1(0) = prev_fly_along_cable_state(4);
//	vm1(1) = prev_fly_along_cable_state(5);
//	vm1(2) = prev_fly_along_cable_state(6);
//
//	am1(0) = prev_fly_along_cable_state(8);
//	am1(1) = prev_fly_along_cable_state(9);
//	am1(2) = prev_fly_along_cable_state(10);
//
//	RCLCPP_INFO(this->get_logger(), "pm1: %f, %f, %f", pm1(0), pm1(1), pm1(2));
//	RCLCPP_INFO(this->get_logger(), "vm1: %f, %f, %f", vm1(0), vm1(1), vm1(2));
//	RCLCPP_INFO(this->get_logger(), "am1: %f, %f, %f", am1(0), am1(1), am1(2));
//
//	p0 = pm1 + vm1*dt + 0.5*am1*dt*dt;
//	v0 = vm1 + am1*dt;
//
//	v1 = v_max;
//	a0 = (v1-v0)/dt;
//
//	if (a0.norm() > a_max.norm()) {
//		a0 = a_max;
//		v1 = v0 + a0*dt;
//	}
//
//	p1 = p0 + v0*dt + 0.5*a0*dt*dt;
//
//	if ((p1-p0).norm() >= distance_left) {
//
//		a0 = - cable_x_W * (distance_left)/(0.5*dt*dt);
//		v0 = -a0*dt;
//		
//	} else {
//
//		p1 = p0 + v0*dt + 0.5*a0*dt*dt;
//		v1 = v0 + a0*dt;
//
//		a1 = (v_max-v1)/dt;
//
//		if (a1.norm() > a_max.norm()) {
//
//			a1 = a_max;
//
//		}
//
//		p2 = p1 + v1*dt + 0.5*a1*dt*dt;
//		v2 = v1 + a1*dt;
//
//		if ((p2-p0).norm() > distance_left) {
//
//			a0 = - (cable_x_W * distance_left - 1.5*v0*dt)/(dt*dt);
//
//		}
//	}
//
//	// p1 = p0 + v0*dt + 0.5*a0*dt*dt;
//	// RCLCPP_INFO(this->get_logger(), "p1 before proj: %f, %f, %f", p1(0), p1(1), p1(2));
//	// p1 = projectPointOnPlane(p1, cable_plane_W);
//
//	// RCLCPP_INFO(this->get_logger(), "cable_plane_W.p: %f, %f, %f", cable_plane_W.p(0), cable_plane_W.p(1), cable_plane_W.p(2));
//	// RCLCPP_INFO(this->get_logger(), "cable_plane_W.n: %f, %f, %f", cable_plane_W.normal(0), cable_plane_W.normal(1), cable_plane_W.normal(2));
//	// RCLCPP_INFO(this->get_logger(), "p1 after proj: %f, %f, %f", p1(0), p1(1), p1(2));
//
//	// v0 = (p1 - p0 - 0.5*v1) / (dt - 0.5);
//	// a0 = (v1-v0) / dt;
//
//	RCLCPP_INFO(this->get_logger(), "p0: %f, %f, %f", p0(0), p0(1), p0(2));
//	RCLCPP_INFO(this->get_logger(), "v0: %f, %f, %f", v0(0), v0(1), v0(2));
//	RCLCPP_INFO(this->get_logger(), "a0: %f, %f, %f", a0(0), a0(1), a0(2));
//
//	p0 = projectPointOnPlane(p0, cable_plane_W);
//
//	fly_along_cable_state(0) = p0(0);
//	fly_along_cable_state(1) = p0(1);
//	fly_along_cable_state(2) = p0(2);
//	fly_along_cable_state(3) = best_yaw;
//	fly_along_cable_state(4) = v0(0);
//	fly_along_cable_state(5) = v0(1);
//	fly_along_cable_state(6) = v0(2);
//	fly_along_cable_state(7) = 0;
//	fly_along_cable_state(8) = a0(0);
//	fly_along_cable_state(9) = a0(1);
//	fly_along_cable_state(10) = a0(2);
//	fly_along_cable_state(11) = 0;
//
//	return fly_along_cable_state;
//
//}

state4_t TrajectoryController::loadTargetFlyAlongCableState(bool first) {

	static vector_t first_under_cable_pos(0,0,0);

	float target_distance;
	bool inverse_direction;

	fly_along_cable_mutex_.lock(); {

		target_distance = fly_along_cable_distance_;
		inverse_direction = fly_along_cable_inverse_direction_;

	} fly_along_cable_mutex_.unlock();

	state4_t fly_along_cable_state;

	geometry_msgs::msg::Pose cable_pose;

	powerline_mutex_.lock(); {

		cable_pose = target_cable_pose_.pose;

	} powerline_mutex_.unlock();

	quat_t cable_quat(
		cable_pose.orientation.w,
		cable_pose.orientation.x,
		cable_pose.orientation.y,
		cable_pose.orientation.z
	);

	orientation_t cable_eul = quatToEul(cable_quat);

	geometry_msgs::msg::TransformStamped T_drone_to_cable_gripper = tf_buffer_->lookupTransform("cable_gripper", "drone", tf2::TimePointZero);

	quat_t q_drone_to_cable_gripper(
		T_drone_to_cable_gripper.transform.rotation.w,
		T_drone_to_cable_gripper.transform.rotation.x,
		T_drone_to_cable_gripper.transform.rotation.y,
		T_drone_to_cable_gripper.transform.rotation.z
	);

	orientation_t eul_drone_to_cable_gripper = quatToEul(q_drone_to_cable_gripper);

	float target_yaw;

	target_yaw_mutex_.lock(); {

		target_yaw = target_yaw_;

	} target_yaw_mutex_.unlock();

	if (target_yaw > M_PI) 
		target_yaw -= 2*M_PI;
	else if (target_yaw < -M_PI)
		target_yaw += 2*M_PI;

	float cable_gripper_target_yaw = cable_eul(2) + eul_drone_to_cable_gripper(2);

	if (cable_gripper_target_yaw > M_PI)
		cable_gripper_target_yaw -= 2*M_PI;
	else if (cable_gripper_target_yaw < -M_PI)
		cable_gripper_target_yaw += 2*M_PI;

	float best_yaw = cable_gripper_target_yaw;

	if (abs(best_yaw - target_yaw) > abs(cable_gripper_target_yaw-M_PI - target_yaw))
		best_yaw = cable_gripper_target_yaw - M_PI;

	if (abs(best_yaw - target_yaw) > abs(cable_gripper_target_yaw+M_PI-target_yaw))
		best_yaw = cable_gripper_target_yaw + M_PI;

	vector_t under_cable_pos(
		cable_pose.position.x,
		cable_pose.position.y,
		cable_pose.position.z - target_cable_distance_
	);

	if (first) {
		first_under_cable_pos = under_cable_pos;
	}

	float dist = sqrt(pow(under_cable_pos(0)-first_under_cable_pos(0),2) + pow(under_cable_pos(1)-first_under_cable_pos(1),2));

	float dist_left = target_distance - dist;

	if (inverse_direction) {
		dist_left = -dist_left;
	}

	fly_along_cable_state(0) = under_cable_pos(0) + dist_left*cos(cable_eul(2));
	fly_along_cable_state(1) = under_cable_pos(1) + dist_left*sin(cable_eul(2));
	fly_along_cable_state(2) = under_cable_pos(2);
	fly_along_cable_state(3) = best_yaw;
	fly_along_cable_state(4) = 0;
	fly_along_cable_state(5) = 0;
	fly_along_cable_state(6) = 0;
	fly_along_cable_state(7) = 0;
	fly_along_cable_state(8) = 0;
	fly_along_cable_state(9) = 0;
	fly_along_cable_state(10) = 0;
	fly_along_cable_state(11) = 0;

	return fly_along_cable_state;

}

void TrajectoryController::setFlyAlongCableParams(float distance, float velocity, float inverse_direction) {

	fly_along_cable_mutex_.lock(); {

		fly_along_cable_distance_ = distance;
		fly_along_cable_velocity_ = velocity;
		fly_along_cable_inverse_direction_ = inverse_direction;

	} fly_along_cable_mutex_.unlock();

}

vector_t TrajectoryController::stepCartesianVelocityPID(state4_t vehicle_state, state4_t target_state, bool reset) {

	// Variables:
	static bool first = true;

	static vector_t prev_error;

	static vector_t integral_error;

	static vector_t derivative_error;

	static const vector_t zero_vec(0,0,0);

	double dt;
	this->get_parameter("dt", dt);

	double Kp, Ki, Kd, i_reset_error_threshold;
	this->get_parameter("cartesian_PID_Kp", Kp);
	this->get_parameter("cartesian_PID_Ki", Ki);
	this->get_parameter("cartesian_PID_Kd", Kd);
	this->get_parameter("cartesian_PID_integral_reset_error_threshold", i_reset_error_threshold);

	// Init:
	vector_t target = target_state.head(3);
	vector_t pos = vehicle_state.head(3);

	// Compute error:
	vector_t error = target - pos;

	if (first || reset) {
		prev_error = error;
		integral_error = zero_vec;
		derivative_error = zero_vec;
	}

	// for (int i = 0; i < 3; i++) {
	// 	if (abs(error(i)) <= i_reset_error_threshold || error(i)*prev_error(i) < 0)
	// 		integral_error(i) = 0.;
	// }

	integral_error += error*dt;
	derivative_error = (error - prev_error)/dt;

	// Compute control:
	vector_t control = Kp*error + Ki*integral_error + Kd*derivative_error;

	// Update:
	prev_error = error;

	first = false;

	// Return:
	return control;

}

state4_t TrajectoryController::stepMPC(state4_t vehicle_state, state4_t target_state, bool set_target, bool reset, MPC_mode_t mpc_mode) {

	// Variables:

	state4_t veh_state = vehicle_state;

	static bool first = true;

	static double dt;

	static bool use_state_feedback;

	static double planned_traj[120];
	static double x[6];
	static double u[3];

	static double target[6];

	static int reset_target;
	static int reset_trajectory;
	static int reset_bounds;
	static int reset_weights;

	static state4_t prev_vehicle_state = vehicle_state;

	static state4_t ref_target_state;

	if (set_target) {

		RCLCPP_DEBUG(this->get_logger(), "Setting target state to: %f, %f, %f, %f", target_state(0), target_state(1), target_state(2), target_state(3));

		ref_target_state = target_state;

	}

	// cos_yaw = cos(ref_target_state(3));
	// sin_yaw = sin(ref_target_state(3));

	// vector_t translated_veh_state(
	// 	vehicle_state(0) - ref_target_state(0),
	// 	vehicle_state(1) - ref_target_state(1),
	// 	vehicle_state(2) - ref_target_state(2)
	// );

	// vector_t transformed_veh_state(
	// 	cos_yaw*translated_veh_state(0) - sin_yaw*translated_veh_state(1),
	// 	cos_yaw*translated_veh_state(1) + sin_yaw*translated_veh_state(0),
	// 	translated_veh_state(2)
	// );

	// vehicle_state(0) = transformed_veh_state(0);
	// vehicle_state(1) = transformed_veh_state(1);
	// vehicle_state(2) = transformed_veh_state(2);

	if (first || reset) {

		RCLCPP_INFO(this->get_logger(), "Initializing MPC with vehicle state");

		prev_vehicle_state = vehicle_state;

	}

	// Initialization:

	if (reset) {

		// reset_target = 1;
		reset_trajectory = 1;
		reset_bounds = 1;
		reset_weights = 1;

	} else {

		// if (mpc_mode == cable_landing)
		// 	reset_target = 1;
		// else
		// 	reset_target = 0;

		reset_trajectory = 0;
		reset_bounds = 0;
		reset_weights = 0;

	}

	reset_target = set_target;

	if (reset_trajectory) {

		this->get_parameter("dt", dt);
		this->get_parameter("MPC_use_state_feedback", use_state_feedback);

	}

	if (set_target) {

		// vector_t translated_target(
		// 	target_state(0) - ref_target_state(0),
		// 	target_state(1) - ref_target_state(1),
		// 	target_state(2) - ref_target_state(2)
		// );

		// vector_t transformed_target(
		// 	cos_yaw*translated_target(0) - sin_yaw*translated_target(1),
		// 	cos_yaw*translated_target(1) + sin_yaw*translated_target(0),
		// 	translated_target(2)
		// );

		// target[0] = transformed_target(0);
		// target[1] = transformed_target(1);
		// target[2] = transformed_target(2);

		// target[0] = ref_target_state(0);
		// target[1] = ref_target_state(1);
		// target[2] = ref_target_state(2);

		for (int i = 0; i < 3; i++) {

			target[i] = target_state(i);
			target[i+3] = target_state(i+4);

		}
	}

	// Step:

	auto resetTraj = [&]() -> void {

		for (int i = 0; i < MPC_N_; i++) {
			for (int j = 0; j < 3; j++) {

				planned_traj[i*6+j] = vehicle_state(j);

			}

			for (int j = 3; j < 6; j++) {

				planned_traj[i*6+j] = 0;

			}
		}

		for (int i = 0; i < 6; i++) x[i] = planned_traj[i];

	};

	if (first) {

		resetTraj();

	} else {

		MPC_thread_.join();

		if (reset) {

			resetTraj();

		} else {

			for (int i = 0; i < MPC_N_*6; i++) planned_traj[i] = MPC_planned_traj_[i];
			for (int i = 0; i < 6; i++) x[i] = MPC_x_[i];
			for (int i = 0; i < 3; i++) u[i] = MPC_u_[i];

		}
	}

	// No state progression:
	if (use_state_feedback || reset) {

		for (int i = 0; i < 3; i++) MPC_x_[i] = vehicle_state(i);
		for (int i = 3; i < 6; i++) MPC_x_[i] = vehicle_state(i+1);

	}

	// With state progression:
	//for (int i = 0; i < 3; i++) MPC_x_[i] = vehicle_state(i) + vehicle_state(i+4)*dt + 0.5*u[i]*dt*dt;
	//for (int i = 0; i < 3; i++) MPC_x_[i+3] = vehicle_state(i+4) + u[i]*dt;

	prev_vehicle_state = vehicle_state;

	MPC_thread_ = std::thread( &TrajectoryController::threadFunctionMPC, this,
					MPC_x_, MPC_u_, MPC_planned_traj_, target, reset_target, reset_trajectory, reset_bounds, reset_weights, mpc_mode);

	// Output:

	state4_t set_point;

	if (first) {

		set_point = vehicle_state;

		first = false;

	} else {

		planned_trajectory_mutex_.lock(); {

			planned_trajectory_.clear();
			planned_trajectory_.resize(MPC_N_);

			planned_trajectory_.resize(0);

			for (int i = 0; i < MPC_N_; i++) {

				state4_t tmp_state;

				if (true) {
				// if (mpc_mode == fly_along_cable) {

					// vector_t rotated_pos(
					// 	cos_yaw*planned_traj[i*6+0] + sin_yaw*planned_traj[i*6+1],
					// 	cos_yaw*planned_traj[i*6+1] - sin_yaw*planned_traj[i*6+0],
					// 	planned_traj[i*6+2]
					// );

					// vector_t transformed_pos(
					// 	rotated_pos(0) + ref_target_state(0),
					// 	rotated_pos(1) + ref_target_state(1),
					// 	rotated_pos(2) + ref_target_state(2)
					// );

					// tmp_state[0] = transformed_pos(0);
					// tmp_state[1] = transformed_pos(1);
					// tmp_state[2] = transformed_pos(2);

					tmp_state[0] = planned_traj[i*6+0];
					tmp_state[1] = planned_traj[i*6+1];
					tmp_state[2] = planned_traj[i*6+2];

					// vector_t transformed_vel(
					// 	cos_yaw*planned_traj[i*6+3] + sin_yaw*planned_traj[i*6+4],
					// 	cos_yaw*planned_traj[i*6+4] - sin_yaw*planned_traj[i*6+3],
					// 	planned_traj[i*6+5]
					// );

					//vector_t transformed_vel(
					//	rotated_vel(0) + prev_target_cable_state(3),
					//	rotated_vel(1) + prev_target_cable_state(4),
					//	rotated_vel(2) + prev_target_cable_state(5)
					//);

					// tmp_state[3] = transformed_vel(0);
					// tmp_state[4] = transformed_vel(1);
					// tmp_state[5] = transformed_vel(2);

					tmp_state[3] = planned_traj[i*6+0];
					tmp_state[4] = planned_traj[i*6+1];
					tmp_state[5] = planned_traj[i*6+2];

				} else {

					for (int j = 0; j < 3; j++) tmp_state(j) = planned_traj[i*6+j];
					for (int j = 3; j < 6; j++) tmp_state(j+1) = planned_traj[i*6+j];

				}

				tmp_state(3) = target_state(3);
				tmp_state(7) = 0;
				tmp_state(11) = 0;

				planned_trajectory_.push_back(tmp_state);

			}

		} planned_trajectory_mutex_.unlock();

		if (true) {
		// if (mpc_mode == fly_along_cable) {

			// vector_t rotated_pos(
			// 	cos_yaw*x[0] + sin_yaw*x[1],
			// 	cos_yaw*x[1] - sin_yaw*x[0],
			// 	x[2]
			// );

			// vector_t transformed_pos(
			// 	rotated_pos(0) + ref_target_state(0),
			// 	rotated_pos(1) + ref_target_state(1),
			// 	rotated_pos(2) + ref_target_state(2)
			// );

			// set_point(0) = transformed_pos(0);
			// set_point(1) = transformed_pos(1);
			// set_point(2) = transformed_pos(2);

			set_point(0) = x[0];
			set_point(1) = x[1];
			set_point(2) = x[2];

			// vector_t transformed_vel(
			// 	cos_yaw*x[3] + sin_yaw*x[4],
			// 	cos_yaw*x[4] - sin_yaw*x[3],
			// 	x[5]
			// );

			//vector_t transformed_vel(
			//	rotated_vel(0) + prev_target_cable_state(3),
			//	rotated_vel(1) + prev_target_cable_state(4),
			//	rotated_vel(2) + prev_target_cable_state(5)
			//);

			// set_point(4) = transformed_vel(0);
			// set_point(5) = transformed_vel(1);
			// set_point(6) = transformed_vel(2);

			set_point(4) = x[3];
			set_point(5) = x[4];
			set_point(6) = x[5];

			// vector_t transformed_acc(
			// 	cos_yaw*u[0] + sin_yaw*u[1],
			// 	cos_yaw*u[1] - sin_yaw*u[0],
			// 	u[2]
			// );

			// vector_t transformed_acc(
			// 	rotated_acc(0) + prev_target_cable_state(6),
			// 	rotated_acc(1) + prev_target_cable_state(7),
			// 	rotated_acc(2) + prev_target_cable_state(8)
			// );

			// set_point(8) = transformed_acc(0);
			// set_point(9) = transformed_acc(1);
			// set_point(10) = transformed_acc(2);

			set_point(8) = u[0];
			set_point(9) = u[1];
			set_point(10) = u[2];

		} else {

			for (int i = 0; i < 3; i++) set_point(i) = x[i];
			for (int i = 0; i < 3; i++) set_point(i+4) = x[i+3];
			for (int i = 0; i < 3; i++) set_point(i+8) = u[i];

		}

		set_point(3) = target_state(3);
		set_point(7) = 0;
		set_point(11) = NAN;

	}

	return set_point;

}

void TrajectoryController::threadFunctionMPC(double *x, double *u, double *planned_traj, double *target, 
		int reset_target, int reset_trajectory, int reset_bounds, int reset_weights, MPC_mode_t mpc_mode) {

	static MPC_parameters_t mpc_params;

	const int N = 10;

	static pos_MPC::struct10_T Info;
	static pos_MPC::struct4_T mpcmovestate;
	static pos_MPC::struct5_T mpconlinedata;

	if (reset_target && reset_bounds && reset_weights && reset_trajectory) {

		RCLCPP_INFO(this->get_logger(), "Resetting MPC fully");

	}

	if (reset_trajectory) {

		loadPeriodMPC(mpc_params, mpc_mode);

	}

	if (reset_bounds) {

		loadBoundsMPC(mpc_params, mpc_mode);

	}

	if (reset_weights) {

		loadWeightsMPC(mpc_params, mpc_mode);

	}

	if (reset_trajectory) {

		for(int i = 0; i < 81; i++) mpcmovestate.Covariance[i] = 0;
		for(int i=0; i < 3; i++) mpcmovestate.Disturbance[i] = 0;
		for(int i = 0; i < 6*2*N; i++) mpcmovestate.iA[i] = 0;
		for(int i=0; i < 3; i++) mpcmovestate.LastMove[i] = 0;
		for(int i=0; i < 6; i++) mpcmovestate.Plant[i] = x[i];

	}

	if (reset_bounds) {

		mpconlinedata.limits.umax[0] = mpc_params.ax_max;
		mpconlinedata.limits.umax[1] = mpc_params.ay_max;
		mpconlinedata.limits.umax[2] = mpc_params.az_max;

		mpconlinedata.limits.umin[0] = -mpc_params.ax_max;
		mpconlinedata.limits.umin[1] = -mpc_params.ay_max;
		mpconlinedata.limits.umin[2] = -mpc_params.az_max;

		mpconlinedata.limits.ymax[0] = 100000;
		mpconlinedata.limits.ymax[1] = 100000;
		mpconlinedata.limits.ymax[2] = 100000;
		mpconlinedata.limits.ymax[3] = mpc_params.vx_max;
		mpconlinedata.limits.ymax[4] = mpc_params.vy_max;
		mpconlinedata.limits.ymax[5] = mpc_params.vz_max;

		mpconlinedata.limits.ymin[0] = -100000;
		mpconlinedata.limits.ymin[1] = -100000;
		mpconlinedata.limits.ymin[2] = -100000;
		mpconlinedata.limits.ymin[3] = -mpc_params.vx_max;
		mpconlinedata.limits.ymin[4] = -mpc_params.vy_max;
		mpconlinedata.limits.ymin[5] = -mpc_params.vz_max;

	}

	if (reset_target) {

		RCLCPP_DEBUG(this->get_logger(), "MPC resetting Target");
		RCLCPP_DEBUG(this->get_logger(), "MPC Thread State: %f, %f, %f", x[0], x[1], x[2]);
		RCLCPP_DEBUG(this->get_logger(), "MPC Thread Target: %f, %f, %f", target[0], target[1], target[2]);

		for (int i = 0; i < 6; i++) {
		// for (int i = 0; i < 3; i++) {

			mpconlinedata.signals.ref[i] = target[i];
			// mpconlinedata.signals.ref[i+3] = 0;

		}

	}
	if (reset_weights) {

		mpconlinedata.weights.du[0] = mpc_params.wjx;
		mpconlinedata.weights.du[1] = mpc_params.wjy;
		mpconlinedata.weights.du[2] = mpc_params.wjz;

		mpconlinedata.weights.u[0] = mpc_params.wax;
		mpconlinedata.weights.u[1] = mpc_params.way;
		mpconlinedata.weights.u[2] = mpc_params.waz;

		mpconlinedata.weights.y[0] = mpc_params.wx;
		mpconlinedata.weights.y[1] = mpc_params.wy;
		mpconlinedata.weights.y[2] = mpc_params.wz;
		mpconlinedata.weights.y[3] = mpc_params.wvx;
		mpconlinedata.weights.y[4] = mpc_params.wvy;
		mpconlinedata.weights.y[5] = mpc_params.wvz;

	}

	for(int i = 0; i < 6; i++) mpconlinedata.signals.ym[i] = x[i];

	if (mpc_mode == cable_takeoff) {
		RCLCPP_DEBUG(this->get_logger(), "MPC Thread Target stored: %f, %f, %f", mpconlinedata.signals.ref[0], mpconlinedata.signals.ref[1], mpconlinedata.signals.ref[2]);
		RCLCPP_DEBUG(this->get_logger(), "MPC Thread State stored: %f, %f, %f", mpconlinedata.signals.ym[0], mpconlinedata.signals.ym[1], mpconlinedata.signals.ym[2]);
		RCLCPP_DEBUG(this->get_logger(), "MPC Thread wxyz stored: %f, %f, %f, %f", mpconlinedata.weights.y[0], mpconlinedata.weights.y[1], mpconlinedata.weights.y[2]);
		RCLCPP_DEBUG(this->get_logger(), "MPC Thread wvxyz stored: %f, %f, %f, %f", mpconlinedata.weights.y[3], mpconlinedata.weights.y[4], mpconlinedata.weights.y[5]);
	}

	pos_MPC::coder::mpcmoveCodeGeneration(&mpcmovestate, &mpconlinedata, u, &Info);

	for (int j = 0; j < 6; j++) x[j] = Info.Yopt[j*(N+1)+1];
	for (int i = 0; i < N; i++) {

		for (int j = 0; j < 6; j++) planned_traj[i*6+j] = Info.Yopt[j*(N+1)+(i+1)];

	} 

	// for (int i = 0; i < 60; i++) {
	// 	if (i % 6 == 0) RCLCPP_INFO(this->get_logger(), " ");
	// 	RCLCPP_INFO(this->get_logger(), "planned_traj[%d] = %f", i, planned_traj[i]);
	// }

	// RCLCPP_INFO(this->get_logger(), "\n\n");

}

void TrajectoryController::loadPeriodMPC(MPC_parameters_t &mpc_params, MPC_mode_t mpc_mode) {

	this->get_parameter("dt", mpc_params.dt);

}

void TrajectoryController::loadBoundsMPC(MPC_parameters_t &mpc_params, MPC_mode_t mpc_mode) {

	switch(mpc_mode) {
	default:
	case positional:
		this->get_parameter("position_MPC_vx_max", mpc_params.vx_max);
		this->get_parameter("position_MPC_vy_max", mpc_params.vy_max);
		this->get_parameter("position_MPC_vz_max", mpc_params.vz_max);

		this->get_parameter("position_MPC_ax_max", mpc_params.ax_max);
		this->get_parameter("position_MPC_ay_max", mpc_params.ay_max);
		this->get_parameter("position_MPC_az_max", mpc_params.az_max);

		break;

	case cable_landing:
		this->get_parameter("cable_landing_MPC_vx_max", mpc_params.vx_max);
		this->get_parameter("cable_landing_MPC_vy_max", mpc_params.vy_max);
		this->get_parameter("cable_landing_MPC_vz_max", mpc_params.vz_max);

		this->get_parameter("cable_landing_MPC_ax_max", mpc_params.ax_max);
		this->get_parameter("cable_landing_MPC_ay_max", mpc_params.ay_max);
		this->get_parameter("cable_landing_MPC_az_max", mpc_params.az_max);

		break;

	case cable_takeoff:
		this->get_parameter("cable_takeoff_MPC_vx_max", mpc_params.vx_max);
		this->get_parameter("cable_takeoff_MPC_vy_max", mpc_params.vy_max);
		this->get_parameter("cable_takeoff_MPC_vz_max", mpc_params.vz_max);

		this->get_parameter("cable_takeoff_MPC_ax_max", mpc_params.ax_max);
		this->get_parameter("cable_takeoff_MPC_ay_max", mpc_params.ay_max);
		this->get_parameter("cable_takeoff_MPC_az_max", mpc_params.az_max);

	case fly_along_cable:
		state4_t target_cable_state = loadTargetCableState();

		float max_vel;
		
		fly_along_cable_mutex_.lock(); {

			max_vel = fly_along_cable_velocity_;

		} fly_along_cable_mutex_.unlock();

		mpc_params.vx_max = max_vel*cos(target_cable_state(3));
		mpc_params.vy_max = max_vel*sin(target_cable_state(3));

		this->get_parameter("fly_along_cable_MPC_vz_max", mpc_params.vz_max);

		this->get_parameter("fly_along_cable_MPC_ax_max", mpc_params.ax_max);
		this->get_parameter("fly_along_cable_MPC_ay_max", mpc_params.ay_max);
		this->get_parameter("fly_along_cable_MPC_az_max", mpc_params.az_max);

		break;

	} 
}

void TrajectoryController::loadWeightsMPC(MPC_parameters_t &mpc_params, MPC_mode_t mpc_mode) {

	switch(mpc_mode) {
	default:
	case positional:
		this->get_parameter("position_MPC_wx", mpc_params.wx);
		this->get_parameter("position_MPC_wy", mpc_params.wy);
		this->get_parameter("position_MPC_wz", mpc_params.wz);

		this->get_parameter("position_MPC_wvx", mpc_params.wvx);
		this->get_parameter("position_MPC_wvy", mpc_params.wvy);
		this->get_parameter("position_MPC_wvz", mpc_params.wvz);

		this->get_parameter("position_MPC_wax", mpc_params.wax);
		this->get_parameter("position_MPC_way", mpc_params.way);
		this->get_parameter("position_MPC_waz", mpc_params.waz);

		this->get_parameter("position_MPC_wjx", mpc_params.wjx);
		this->get_parameter("position_MPC_wjy", mpc_params.wjy);
		this->get_parameter("position_MPC_wjz", mpc_params.wjz);

		break;

	case cable_landing:
		this->get_parameter("cable_landing_MPC_wx", mpc_params.wx);
		this->get_parameter("cable_landing_MPC_wy", mpc_params.wy);
		this->get_parameter("cable_landing_MPC_wz", mpc_params.wz);
                                   
		this->get_parameter("cable_landing_MPC_wvx", mpc_params.wvx);
		this->get_parameter("cable_landing_MPC_wvy", mpc_params.wvy);
		this->get_parameter("cable_landing_MPC_wvz", mpc_params.wvz);
                                   
		this->get_parameter("cable_landing_MPC_wax", mpc_params.wax);
		this->get_parameter("cable_landing_MPC_way", mpc_params.way);
		this->get_parameter("cable_landing_MPC_waz", mpc_params.waz);
                                   
		this->get_parameter("cable_landing_MPC_wjx", mpc_params.wjx);
		this->get_parameter("cable_landing_MPC_wjy", mpc_params.wjy);
		this->get_parameter("cable_landing_MPC_wjz", mpc_params.wjz);

		break;

	case cable_takeoff:
		this->get_parameter("cable_takeoff_MPC_wx", mpc_params.wx);
		this->get_parameter("cable_takeoff_MPC_wy", mpc_params.wy);
		this->get_parameter("cable_takeoff_MPC_wz", mpc_params.wz);

		// RCLCPP_INFO(this->get_logger(), "cable_takeoff_MPC_wxyz: %f, %f, %f", mpc_params.wx, mpc_params.wy, mpc_params.wz);

		this->get_parameter("cable_takeoff_MPC_wvx", mpc_params.wvx);
		this->get_parameter("cable_takeoff_MPC_wvy", mpc_params.wvy);
		this->get_parameter("cable_takeoff_MPC_wvz", mpc_params.wvz);

		// RCLCPP_INFO(this->get_logger(), "cable_takeoff_MPC_wvxyz: %f, %f, %f", mpc_params.wvx, mpc_params.wvy, mpc_params.wvz);

		this->get_parameter("cable_takeoff_MPC_wax", mpc_params.wax);
		this->get_parameter("cable_takeoff_MPC_way", mpc_params.way);
		this->get_parameter("cable_takeoff_MPC_waz", mpc_params.waz);

		// RCLCPP_INFO(this->get_logger(), "cable_takeoff_MPC_waxyz: %f, %f, %f", mpc_params.wax, mpc_params.way, mpc_params.waz);

		this->get_parameter("cable_takeoff_MPC_wjx", mpc_params.wjx);
		this->get_parameter("cable_takeoff_MPC_wjy", mpc_params.wjy);
		this->get_parameter("cable_takeoff_MPC_wjz", mpc_params.wjz);

		// RCLCPP_INFO(this->get_logger(), "cable_takeoff_MPC_wjxyz: %f, %f, %f", mpc_params.wjx, mpc_params.wjy, mpc_params.wjz);

	case fly_along_cable:
		this->get_parameter("fly_along_cable_MPC_wx", mpc_params.wx);
		this->get_parameter("fly_along_cable_MPC_wy", mpc_params.wy);
		this->get_parameter("fly_along_cable_MPC_wz", mpc_params.wz);

		this->get_parameter("fly_along_cable_MPC_wvx", mpc_params.wvx);
		this->get_parameter("fly_along_cable_MPC_wvy", mpc_params.wvy);
		this->get_parameter("fly_along_cable_MPC_wvz", mpc_params.wvz);

		this->get_parameter("fly_along_cable_MPC_wax", mpc_params.wax);
		this->get_parameter("fly_along_cable_MPC_way", mpc_params.way);
		this->get_parameter("fly_along_cable_MPC_waz", mpc_params.waz);

		this->get_parameter("fly_along_cable_MPC_wjx", mpc_params.wjx);
		this->get_parameter("fly_along_cable_MPC_wjy", mpc_params.wjy);
		this->get_parameter("fly_along_cable_MPC_wjz", mpc_params.wjz);

		break;

	}
}

void TrajectoryController::clearPlannedTrajectory() {

	planned_trajectory_mutex_.lock(); {

		planned_trajectory_.clear();
		planned_trajectory_.resize(0);

		planned_macro_trajectory_.clear();
		planned_macro_trajectory_.resize(0);

		for (int i = 0; i < 8; i++) trajectory_target_(i) = NAN;

	} planned_trajectory_mutex_.unlock();

}

void TrajectoryController::setTrajectoryTarget(state4_t target) {

	planned_trajectory_mutex_.lock(); {

		trajectory_target_ = target;

	} planned_trajectory_mutex_.unlock();

}

bool TrajectoryController::updateTargetCablePose(state4_t vehicle_state, int new_id) {

	bool success = false;

	powerline_mutex_.lock(); {

		if (new_id > -1) {

			target_cable_id_ = new_id;

		}

		for (int i = 0; i < powerline_.count; i++) {

			if (powerline_.ids[i] == target_cable_id_) {

				geometry_msgs::msg::PoseStamped tmp_pose;
				tmp_pose.header.frame_id = powerline_.poses[i].header.frame_id;
				tmp_pose.pose = powerline_.poses[i].pose;

				target_cable_pose_ = tf_buffer_->transform(tmp_pose, "world");

				vector_t unit_x(1,0,0);
				quat_t cable_quat(
					target_cable_pose_.pose.orientation.w,
					target_cable_pose_.pose.orientation.x,
					target_cable_pose_.pose.orientation.y,
					target_cable_pose_.pose.orientation.z
				);
				vector_t cable_dir = quatToMat(cable_quat) * unit_x;

				point_t cable_point(
					target_cable_pose_.pose.position.x,
					target_cable_pose_.pose.position.y,
					target_cable_pose_.pose.position.z
				);

				target_cable_plane_.normal = cable_dir;

				if (new_id > -1) {

					target_cable_plane_.p(0) = vehicle_state(0);
					target_cable_plane_.p(1) = vehicle_state(1);
					target_cable_plane_.p(2) = vehicle_state(2);

				} 

				cable_point = projectPointOnPlane(cable_point, target_cable_plane_);

				target_cable_pose_.pose.position.x = cable_point(0);
				target_cable_pose_.pose.position.y = cable_point(1);
				target_cable_pose_.pose.position.z = cable_point(2);

				success = true;

				break;
				
			}
		}

	} powerline_mutex_.unlock();

	return success;

}

void TrajectoryController::clearTargetCable() {

	powerline_mutex_.lock(); {

		target_cable_id_ = -1;

	} powerline_mutex_.unlock();

}

int main(int argc, char* argv[]) {
	std::cout << "Starting cable landing controller node..." << std::endl;

	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TrajectoryController>());

	rclcpp::shutdown();
	return 0;
}
