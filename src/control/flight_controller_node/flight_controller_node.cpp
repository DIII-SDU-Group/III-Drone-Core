/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "iii_drone_core/control/flight_controller_node/flight_controller_node.hpp"

using namespace iii_drone::control::flight_controller_node;
using namespace iii_drone::types;
using namespace iii_drone::math;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

FlightController::FlightController(
	const std::string & node_name, 
	const std::string & node_namespace, 
	const rclcpp::NodeOptions & options
) : Node(
	node_name, 
	node_namespace, 
	options
), configurator_(this),
request_queue_(1), 
request_reply_queue_(2),
request_reply_poll_rate_(50ms),
request_completion_poll_rate_(100ms) {

	if (configurator_.use_cartesian_PID() && configurator_.always_hover_in_offboard()) {

	    RCLCPP_FATAL(
			this->get_logger(), 
			"use_cartesian_PID and always_hover_in_offboard are not compatible"
		);

	    exit(1);

	}

	quaternion_t temp_q(1,0,0,0);
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
	tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

	// Takeoff action:
	this->takeoff_server_ = rclcpp_action::create_server<Takeoff>(
		this,
		"takeoff",
		std::bind(&FlightController::handleGoalTakeoff, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&FlightController::handleCancelTakeoff, this, std::placeholders::_1),
		std::bind(&FlightController::handleAcceptedTakeoff, this, std::placeholders::_1)
	);

	// Landing action:
	this->landing_server_ = rclcpp_action::create_server<Landing>(
		this,
		"landing",
		std::bind(&FlightController::handleGoalLanding, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&FlightController::handleCancelLanding, this, std::placeholders::_1),
		std::bind(&FlightController::handleAcceptedLanding, this, std::placeholders::_1)
	);

	// Fly to position action:
	this->fly_to_position_server_ = rclcpp_action::create_server<FlyToPosition>(
		this,
		"fly_to_position",
		std::bind(&FlightController::handleGoalFlyToPosition, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&FlightController::handleCancelFlyToPosition, this, std::placeholders::_1),
		std::bind(&FlightController::handleAcceptedFlyToPosition, this, std::placeholders::_1)
	);

	// Fly under cable action:
	this->fly_under_cable_server_ = rclcpp_action::create_server<FlyUnderCable>(
		this,
		"fly_under_cable",
		std::bind(&FlightController::handleGoalFlyUnderCable, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&FlightController::handleCancelFlyUnderCable, this, std::placeholders::_1),
		std::bind(&FlightController::handleAcceptedFlyUnderCable, this, std::placeholders::_1)
	);

	// Cable landing action:
	this->cable_landing_server_ = rclcpp_action::create_server<CableLanding>(
		this,
		"cable_landing",
		std::bind(&FlightController::handleGoalCableLanding, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&FlightController::handleCancelCableLanding, this, std::placeholders::_1),
		std::bind(&FlightController::handleAcceptedCableLanding, this, std::placeholders::_1)
	);

	// Cable takeoff action:
	this->cable_takeoff_server_ = rclcpp_action::create_server<CableTakeoff>(
		this,
		"cable_takeoff",
		std::bind(&FlightController::handleGoalCableTakeoff, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&FlightController::handleCancelCableTakeoff, this, std::placeholders::_1),
		std::bind(&FlightController::handleAcceptedCableTakeoff, this, std::placeholders::_1)
	);

	// Arm on cable action:
	this->arm_on_cable_server_ = rclcpp_action::create_server<ArmOnCable>(
		this,
		"arm_on_cable",
		std::bind(&FlightController::handleGoalArmOnCable, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&FlightController::handleCancelArmOnCable, this, std::placeholders::_1),
		std::bind(&FlightController::handleAcceptedArmOnCable, this, std::placeholders::_1)
	);

	// Disarm on cable action:
	this->disarm_on_cable_server_ = rclcpp_action::create_server<DisarmOnCable>(
		this,
		"disarm_on_cable",
		std::bind(&FlightController::handleGoalDisarmOnCable, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&FlightController::handleCancelDisarmOnCable, this, std::placeholders::_1),
		std::bind(&FlightController::handleAcceptedDisarmOnCable, this, std::placeholders::_1)
	);

	// Set yaw service:
    set_yaw_service_ = this->create_service<iii_drone_interfaces::srv::SetGeneralTargetYaw>("set_general_target_yaw", 
        std::bind(&FlightController::setYawServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

	// Publishers and subscriptions:
	planned_traj_pub_ = this->create_publisher<nav_msgs::msg::Path>("planned_trajectory", 10);
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
		this->create_publisher<iii_drone_interfaces::msg::ControlState>("control_state", 10);

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
		std::bind(&FlightController::odometryCallback, this, std::placeholders::_1));

	home_position_sub_ = this->create_subscription<px4_msgs::msg::HomePosition>(
		"/fmu/out/home_position", 
		sub_qos,
		std::bind(&FlightController::homePositionCallback, this, std::placeholders::_1));

	powerline_sub_ = this->create_subscription<iii_drone_interfaces::msg::Powerline>(
		"/perception/pl_mapper/powerline", 
		10,
		std::bind(&FlightController::powerlineCallback, this, std::placeholders::_1));

	gripper_status_sub_ = this->create_subscription<iii_drone_interfaces::msg::GripperStatus>(
		"/payload/charger_gripper/gripper_status", 
		10,
		std::bind(&FlightController::gripperStatusCallback, this, std::placeholders::_1));

	main_state_machine_timer_ = this->create_wall_timer(
		std::chrono::milliseconds((int)(configurator_.dt()*1000)), 
		std::bind(
			&FlightController::stateMachineCallback, 
			this
		)
	);

	// RCLCPP debug successfully initilized trajectory controller
	RCLCPP_INFO(this->get_logger(), "Successfully initialized trajectory controller");

}

FlightController::~FlightController() {

	RCLCPP_INFO(this->get_logger(),  "Shutting down offboard control..");
	land();
	std::this_thread::sleep_for(std::chrono::milliseconds(50));
	
}

rclcpp_action::GoalResponse FlightController::handleGoalTakeoff(
	const rclcpp_action::GoalUUID & uuid, 
	std::shared_ptr<const Takeoff::Goal> goal
) {

	RCLCPP_DEBUG(this->get_logger(), "Received takeoff goal request with target altitude %d", goal->target_altitude);
	
	(void)uuid;

	if (goal->target_altitude < configurator_.minimum_target_altitude()){
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

rclcpp_action::CancelResponse FlightController::handleCancelTakeoff(const std::shared_ptr<GoalHandleTakeoff> goal_handle) {

	RCLCPP_DEBUG(this->get_logger(), "Received request to cancel takeoff, rejecting..");

	return rclcpp_action::CancelResponse::REJECT;

}

void FlightController::handleAcceptedTakeoff(const std::shared_ptr<GoalHandleTakeoff> goal_handle) {

	// debug takeoff goal accepted, starting thread
	RCLCPP_DEBUG(this->get_logger(), "Takeoff goal accepted, starting thread");

	using namespace std::placeholders;

	std::thread{ std::bind(&FlightController::followTakeoffCompletion, this, _1), goal_handle}.detach();

}

void FlightController::followTakeoffCompletion(const std::shared_ptr<GoalHandleTakeoff> goal_handle) {

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

rclcpp_action::GoalResponse FlightController::handleGoalLanding(
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

rclcpp_action::CancelResponse FlightController::handleCancelLanding(const std::shared_ptr<GoalHandleLanding> goal_handle) {

	RCLCPP_DEBUG(this->get_logger(), "Received landing goal cancel request, rejecting");

	return rclcpp_action::CancelResponse::REJECT;

}

void FlightController::handleAcceptedLanding(const std::shared_ptr<GoalHandleLanding> goal_handle) {

	// debug landing goal accepted, starting thread
	RCLCPP_DEBUG(this->get_logger(), "Landing goal accepted, starting thread");

	using namespace std::placeholders;

	std::thread{ std::bind(&FlightController::followLandingCompletion, this, _1), goal_handle}.detach();

}

void FlightController::followLandingCompletion(const std::shared_ptr<GoalHandleLanding> goal_handle) {

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

rclcpp_action::GoalResponse FlightController::handleGoalFlyToPosition(
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

	quaternion_t quat(
		target_pose.pose.orientation.w,
		target_pose.pose.orientation.x,
		target_pose.pose.orientation.y,
		target_pose.pose.orientation.z
	);

	euler_angles_t eul = quatToEul(quat);

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

rclcpp_action::CancelResponse FlightController::handleCancelFlyToPosition(const std::shared_ptr<GoalHandleFlyToPosition> goal_handle) {

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

void FlightController::handleAcceptedFlyToPosition(const std::shared_ptr<GoalHandleFlyToPosition> goal_handle) {

	// debug fly to position goal accepted
	RCLCPP_DEBUG(this->get_logger(), "Fly to position goal accepted, starting thread");

	using namespace std::placeholders;

	std::thread{ std::bind(&FlightController::followFlyToPositionCompletion, this, _1), goal_handle}.detach();

}

void FlightController::followFlyToPositionCompletion(const std::shared_ptr<GoalHandleFlyToPosition> goal_handle) {

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

rclcpp_action::GoalResponse FlightController::handleGoalFlyUnderCable(
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

rclcpp_action::CancelResponse FlightController::handleCancelFlyUnderCable(const std::shared_ptr<GoalHandleFlyUnderCable> goal_handle) {

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

void FlightController::handleAcceptedFlyUnderCable(const std::shared_ptr<GoalHandleFlyUnderCable> goal_handle) {

	// debug fly under cable goal accepted, starting thread
	RCLCPP_DEBUG(this->get_logger(), "Fly under cable goal accepted, starting thread");

	using namespace std::placeholders;

	std::thread{ std::bind(&FlightController::followFlyUnderCableCompletion, this, _1), goal_handle}.detach();

}

void FlightController::followFlyUnderCableCompletion(const std::shared_ptr<GoalHandleFlyUnderCable> goal_handle) {

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

			quaternion_t veh_quat(
				vehicle_pose.pose.orientation.w,
				vehicle_pose.pose.orientation.x,
				vehicle_pose.pose.orientation.y,
				vehicle_pose.pose.orientation.z
			);
			euler_angles_t veh_eul = quatToEul(veh_quat);
			pos4_t veh_state(
				vehicle_pose.pose.position.x,
				vehicle_pose.pose.position.y,
				vehicle_pose.pose.position.z,
				veh_eul(2)
			);
			quaternion_t target_quat(
				target.pose.orientation.w,
				target.pose.orientation.x,
				target.pose.orientation.y,
				target.pose.orientation.z
			);
			euler_angles_t target_eul = quatToEul(target_quat);
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

rclcpp_action::GoalResponse FlightController::handleGoalCableLanding(
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

rclcpp_action::CancelResponse FlightController::handleCancelCableLanding(const std::shared_ptr<GoalHandleCableLanding> goal_handle) {

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

void FlightController::handleAcceptedCableLanding(const std::shared_ptr<GoalHandleCableLanding> goal_handle) {

	// debug cable landing goal accepted
	RCLCPP_DEBUG(this->get_logger(), "Cable landing goal accepted, starting thread to follow completion");

	using namespace std::placeholders;

	std::thread{ std::bind(&FlightController::followCableLandingCompletion, this, _1), goal_handle}.detach();

}

void FlightController::followCableLandingCompletion(const std::shared_ptr<GoalHandleCableLanding> goal_handle) {

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
			geometry_msgs::msg::PoseStamped planned_target = loadPlannedTarget();

			float distance = 0;
			distance += pow(vehicle_pose.pose.position.x - planned_target.pose.position.x, 2);
			distance += pow(vehicle_pose.pose.position.y - planned_target.pose.position.y, 2);
			distance += pow(vehicle_pose.pose.position.z - planned_target.pose.position.z, 2);
			distance = sqrt(distance);


			feedback->vehicle_pose = vehicle_pose;
			feedback->planned_path = planned_path;
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

rclcpp_action::GoalResponse FlightController::handleGoalCableTakeoff(
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

	// Check ROS2 parameter use_gripper_status_condition, and if true, check if gripper_status_ is open:
	if (configurator_.use_gripper_status_condition()) {
		iii_drone_interfaces::msg::GripperStatus gripper_status; {

			std::lock_guard<std::mutex> lock(gripper_status_mutex_);
			gripper_status = gripper_status_;
			
		}

		if (gripper_status.gripper_status != iii_drone_interfaces::msg::GripperStatus::GRIPPER_STATUS_OPEN) {
			// debug cable takeoff goal rejected, gripper not open
			RCLCPP_INFO(this->get_logger(), "Cable takeoff goal rejected, gripper not open");
			return rclcpp_action::GoalResponse::REJECT;
		}
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

rclcpp_action::CancelResponse FlightController::handleCancelCableTakeoff(const std::shared_ptr<GoalHandleCableTakeoff> goal_handle) {

	RCLCPP_DEBUG(this->get_logger(), "Received cable takeoff cancel request, rejecting");

	return rclcpp_action::CancelResponse::REJECT;

}

void FlightController::handleAcceptedCableTakeoff(const std::shared_ptr<GoalHandleCableTakeoff> goal_handle) {

	// debug cable takeoff goal accepted
	RCLCPP_DEBUG(this->get_logger(), "Cable takeoff goal accepted, starting thread");

	using namespace std::placeholders;

	std::thread{ std::bind(&FlightController::followCableTakeoffCompletion, this, _1), goal_handle}.detach();

}

void FlightController::followCableTakeoffCompletion(const std::shared_ptr<GoalHandleCableTakeoff> goal_handle) {

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






















rclcpp_action::GoalResponse FlightController::handleGoalDisarmOnCable(
	const rclcpp_action::GoalUUID & uuid, 
	std::shared_ptr<const DisarmOnCable::Goal> goal
) {

	RCLCPP_INFO(this->get_logger(), "Received disarm on cable goal request");
	
	(void)uuid;

	if (state_ != on_cable_armed) {
		RCLCPP_INFO(this->get_logger(), "Disarm on cable goal rejected, not on cable armed");
		return rclcpp_action::GoalResponse::REJECT;
	}

	if (configurator_.use_gripper_status_condition()) {

		// Load gripper_status_:
		iii_drone_interfaces::msg::GripperStatus gripper_status;
		{
			std::lock_guard<std::mutex> lock(gripper_status_mutex_);
			gripper_status = gripper_status_;
		}

		if (gripper_status.gripper_status != iii_drone_interfaces::msg::GripperStatus::GRIPPER_STATUS_CLOSED) {
			RCLCPP_INFO(this->get_logger(), "Disarm on cable goal rejected, gripper not closed");
			return rclcpp_action::GoalResponse::REJECT;
		}

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

rclcpp_action::CancelResponse FlightController::handleCancelDisarmOnCable(const std::shared_ptr<GoalHandleDisarmOnCable> goal_handle) {

	RCLCPP_DEBUG(this->get_logger(), "Received disarm on cable cancel request, rejecting");

	return rclcpp_action::CancelResponse::REJECT;

}

void FlightController::handleAcceptedDisarmOnCable(const std::shared_ptr<GoalHandleDisarmOnCable> goal_handle) {

	// debug cable takeoff goal accepted
	RCLCPP_DEBUG(this->get_logger(), "Disarm on cable goal accepted, starting thread");

	using namespace std::placeholders;

	std::thread{ std::bind(&FlightController::followDisarmOnCableCompletion, this, _1), goal_handle}.detach();

}

void FlightController::followDisarmOnCableCompletion(const std::shared_ptr<GoalHandleDisarmOnCable> goal_handle) {

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



rclcpp_action::GoalResponse FlightController::handleGoalArmOnCable(
	const rclcpp_action::GoalUUID & uuid, 
	std::shared_ptr<const ArmOnCable::Goal> goal
) {

	RCLCPP_INFO(this->get_logger(), "Received disarm on cable goal request");
	
	(void)uuid;

	if (state_ != on_cable_disarmed) {
		// debug disarm on cable goal rejected, not on cable armed
		RCLCPP_INFO(this->get_logger(), "Arm on cable goal rejected, not on cable armed");
		return rclcpp_action::GoalResponse::REJECT;
	}

	rclcpp_action::GoalUUID action_id = (rclcpp_action::GoalUUID)uuid;

	request_t request = {
		.action_id = action_id,
		.request_type = arm_on_cable_request,
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

rclcpp_action::CancelResponse FlightController::handleCancelArmOnCable(const std::shared_ptr<GoalHandleArmOnCable> goal_handle) {

	RCLCPP_DEBUG(this->get_logger(), "Received disarm on cable cancel request, rejecting");

	return rclcpp_action::CancelResponse::REJECT;

}

void FlightController::handleAcceptedArmOnCable(const std::shared_ptr<GoalHandleArmOnCable> goal_handle) {

	// debug cable takeoff goal accepted
	RCLCPP_DEBUG(this->get_logger(), "Arm on cable goal accepted, starting thread");

	using namespace std::placeholders;

	std::thread{ std::bind(&FlightController::followArmOnCableCompletion, this, _1), goal_handle}.detach();

}

void FlightController::followArmOnCableCompletion(const std::shared_ptr<GoalHandleArmOnCable> goal_handle) {

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

void FlightController::setYawServiceCallback(const std::shared_ptr<iii_drone_interfaces::srv::SetGeneralTargetYaw::Request> request,
                                std::shared_ptr<iii_drone_interfaces::srv::SetGeneralTargetYaw::Response> response) {

    target_yaw_mutex_.lock(); {

		target_yaw_ = request->target_yaw;

	} target_yaw_mutex_.unlock();

    response->success = true;

}

void FlightController::stateMachineCallback() {

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

	static vector_t disarm_on_cable_initial_position;
	static int disarm_on_cable_cnt;
	static rclcpp::Time disarm_on_cable_start_time;
	static bool has_started_disarm_on_cable_countdown = false;
	static bool disarm_on_cable_flight_has_been_terminated = false;

	bool offboard = isOffboard();
	bool armed = isArmed();

	armed |= configurator_.always_armed_for_debug();

	static bool has_determined_ground_altitude_offset = false;
	static state3_t ground_altitude_offset(0,0,0,0,0,0);

	if (configurator_.use_ground_altitude_offset()) {
		if (state_ != disarming_on_cable && state_ != on_cable_disarmed && state_ != arming_on_cable && state_ != setting_offboard_on_cable) {
			if (!offboard && !armed) {
				for (int i = 0; i < 3; i++) {

					ground_altitude_offset(i) = veh_state(i);
					ground_altitude_offset(i+3) = veh_state(i+4);

				}

				has_determined_ground_altitude_offset = true;
			}
		}
	}

	float landed_altitude_threshold = configurator_.landed_altitude_threshold();

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

	auto reachedPosition = [this](state4_t state, state4_t target) -> bool {

		for(int i=3;i<12;i++) target(i) = 0;
		for(int i=8;i<12;i++) state(i) = 0;
		state(3) = 0;

		RCLCPP_DEBUG(this->get_logger(), "Target: %f, %f, %f, %f", target(0), target(1), target(2), target(3));
		RCLCPP_DEBUG(this->get_logger(), "State: %f, %f, %f, %f", state(0), state(1), state(2), state(3));
		RCLCPP_DEBUG(this->get_logger(), "Reached position: %s < %s ?", std::to_string((state-target).norm()), std::to_string(configurator_.reached_position_euclidean_distance_threshold()));
		

		float norm = (state-target).norm();

		//LOG_INFO(std::to_string(norm));

		return norm <= configurator_.reached_position_euclidean_distance_threshold();

	};

	auto withinDirectTargetDistance = [this](state4_t state, state4_t target) -> bool {

		for(int i=3;i<12;i++) target(i) = 0;
		for(int i=3;i<12;i++) state(i) = 0;

		RCLCPP_DEBUG(this->get_logger(), "Target: %f, %f, %f, %f", target(0), target(1), target(2), target(3));
		RCLCPP_DEBUG(this->get_logger(), "State: %f, %f, %f, %f", state(0), state(1), state(2), state(3));
		RCLCPP_DEBUG(this->get_logger(), "Within direct target distance: %s < %s ?", std::to_string((state-target).norm()), std::to_string(configurator_.direct_target_setpoint_dist_threshold()));
		

		float norm = (state-target).norm();

		//LOG_INFO(std::to_string(norm));

		return norm <= configurator_.direct_target_setpoint_dist_threshold();

	};

	auto withinSafetyMargins = [this](state4_t veh_state, state4_t target) -> bool {

		float target_cable_safety_margin_distance_threshold = configurator_.target_cable_safety_margin_distance_threshold();
		float target_cable_safety_margin_max_euc_distance = configurator_.target_cable_safety_margin_max_euc_distance();
		float target_cable_safety_margin_max_euc_velocity = configurator_.target_cable_safety_margin_max_euc_velocity();
		float target_cable_safety_margin_max_euc_acceleration = configurator_.target_cable_safety_margin_max_euc_acceleration();
		float target_cable_safety_margin_max_yaw_distance = configurator_.target_cable_safety_margin_max_yaw_distance();
		float target_cable_safety_margin_max_yaw_velocity = configurator_.target_cable_safety_margin_max_yaw_velocity();

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

		float target_cable_set_point_truncate_distance_threshold = configurator_.target_cable_set_point_truncate_distance_threshold();

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
		default: {

			// debug in state init
			RCLCPP_DEBUG(this->get_logger(), "state: init");

			if (!offboard && veh_state(2) < landed_altitude_threshold + ground_altitude_offset(2)) {

				// debug not offboard and under landed altitude threshold
				RCLCPP_DEBUG(this->get_logger(), "not offboard and under landed altitude threshold");

				state_ = on_ground_non_offboard;

			} else if(!offboard && veh_state(2) >= landed_altitude_threshold + ground_altitude_offset(2)) {

				// debug not offboard and over landed altitude threshold
				RCLCPP_DEBUG(this->get_logger(), "not offboard and over landed altitude threshold");

				state_ = in_flight_non_offboard;

			} else if (offboard && veh_state(2) < landed_altitude_threshold + ground_altitude_offset(2)) {

				// debug offboard and under landed altitude threshold, landing
				RCLCPP_DEBUG(this->get_logger(), "offboard and under landed altitude threshold, landing");

				set_point = setNanVelocity(veh_state);

				land();

				state_ = init;

			} else if(offboard && veh_state(2) >= landed_altitude_threshold + ground_altitude_offset(2)) {

				// debug offboard and over landed altitude threshold
				RCLCPP_DEBUG(this->get_logger(), "offboard and over landed altitude threshold");

				fixed_reference = setZeroVelocity(veh_state);

				setTrajectoryTarget(fixed_reference);

				//if (configurator_.use_cartesian_PID()) {

				//	vector_t vel_control = stepCartesianVelocityPID(veh_state, fixed_reference, true);
				//	set_point = setVelocityControl(fixed_reference, vel_control);

				//} else {

					set_point = setNanVelocity(fixed_reference);

				//}

				state_ = hovering;

			}

			break;

		}

		case on_ground_non_offboard: {

			// debug in state on_ground_non_offboard
			RCLCPP_DEBUG(this->get_logger(), "state: on_ground_non_offboard");

			if (!offboard && veh_state(2) >= landed_altitude_threshold + ground_altitude_offset(2) && armed) {

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

				arm_cnt = configurator_.arm_cnt_timeout();

				state_ = arming;

				// debug set trajectory target
				RCLCPP_DEBUG(this->get_logger(), "set trajectory target: %f, %f, %f", fixed_reference(0), fixed_reference(1), fixed_reference(2));

			} else {

				set_point = setNanVelocity(veh_state);

				rejectPendingRequest();

			} 

			break;

		}

		case in_flight_non_offboard: {

			// debug in state in_flight_non_offboard
			RCLCPP_DEBUG(this->get_logger(), "state: in_flight_non_offboard");
			
			if ((!offboard || !armed) && veh_state(2) < landed_altitude_threshold + ground_altitude_offset(2)) {

				// debug not offboard and under landed altitude threshold or not armed
				RCLCPP_DEBUG(this->get_logger(), "not offboard or not armed and under landed altitude threshold");

				state_ = on_ground_non_offboard;

			} else if (!armed && veh_state(2) > landed_altitude_threshold + ground_altitude_offset(2)) {

				iii_drone_interfaces::msg::Powerline powerline;

				powerline_mutex_.lock(); {

					powerline = powerline_;

				} powerline_mutex_.unlock();

				unsigned int powerline_cnt = powerline.lines.size();

				if (powerline_cnt < 1) {

					state_ = init;

				}

				double landed_on_powerline_non_offboard_max_euc_distance = configurator_.landed_on_powerline_non_offboard_max_euc_distance();

				for (int i = 0; i < powerline.count; i++) {

					geometry_msgs::msg::PoseStamped pl_pose = powerline.poses[i];

					// Transform pose to world coordinates:
					pl_pose = tf_buffer_->transform(pl_pose, "world", tf2::durationFromSec(0.1));

					vector_t pl_pos(
						pl_pose.pose.position.x,
						pl_pose.pose.position.y,
						pl_pose.pose.position.z
					);

					vector_t veh_pos(
						veh_state(0),
						veh_state(1),
						veh_state(2)
					);

					// Load transform from frame drone to frame cable_gripper
					geometry_msgs::msg::TransformStamped tf;
					tf = tf_buffer_->lookupTransform("cable_gripper", "drone", tf2::TimePointZero);

					vector_t gripper_pos(
						tf.transform.translation.x,
						tf.transform.translation.y,
						tf.transform.translation.z
					);

					gripper_pos += veh_pos;


					if ((pl_pos - gripper_pos).norm() < landed_on_powerline_non_offboard_max_euc_distance) {

						cable_id = powerline.ids[i];

						target_cable_cnt = configurator_.target_cable_cnt_timeout();
						target_cable_cnt = updateTargetCablePose(veh_state, cable_id) ? target_cable_cnt : target_cable_cnt - 1;

						fixed_reference = loadTargetCableState();

						setTrajectoryTarget(fixed_reference);

						state_ = on_cable_disarmed;

						break;

					}
				}

				if (state_ == in_flight_non_offboard) {

					state_ = init;

				}

			} else if (offboard) {

				// debug offboard
				RCLCPP_DEBUG(this->get_logger(), "offboard");

				fixed_reference = setZeroVelocity(veh_state);

				setTrajectoryTarget(fixed_reference);

				//if (configurator_.use_cartesian_PID()) {

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

		}
		
		case arming: {

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

				offboard_cnt = configurator_.offboard_cnt_timeout();

				state_ = setting_offboard;

			} else {

				arm_cnt--;

				rejectPendingRequest();		
			
			}

			break;

		}

		case setting_offboard: {

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

				//if (configurator_.use_cartesian_PID()) {

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

		}

		case taking_off: {

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

				//if (configurator_.use_cartesian_PID()) {

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

		}

		case hovering: {

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

				land_cnt = configurator_.land_cnt_timeout();

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
					
					//if (configurator_.use_cartesian_PID()) {

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

				offboard_cnt = configurator_.offboard_cnt_timeout();

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

				target_cable_cnt = configurator_.target_cable_cnt_timeout();
				target_cable_cnt = updateTargetCablePose(veh_state, cable_id) ? target_cable_cnt : target_cable_cnt-1;

				fixed_reference = loadTargetUnderCableState();

				setTrajectoryTarget(fixed_reference);

				if (withinDirectTargetDistance(veh_state, fixed_reference)) {

					direct_target_setpoint = true;
					
					//if (configurator_.use_cartesian_PID()) {

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

				offboard_cnt = configurator_.offboard_cnt_timeout();

				state_ = in_positional_flight;

			} else {

				rejectPendingRequest();

				//if (configurator_.use_cartesian_PID()) {

				//	vector_t vel_control = stepCartesianVelocityPID(veh_state, fixed_reference, false);
				//	set_point = setVelocityControl(fixed_reference, vel_control);

				//} else {

					set_point = setNanVelocity(fixed_reference);
					always_hover_state = set_point;

				//}


			}

			break;

		}

		case landing: {

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

		}

		case hovering_under_cable: {

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

				//if (configurator_.use_cartesian_PID()) {

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

				land_cnt = configurator_.land_cnt_timeout();

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
					
					//if (configurator_.use_cartesian_PID()) {

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

				offboard_cnt = configurator_.offboard_cnt_timeout();

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

				target_cable_cnt = configurator_.target_cable_cnt_timeout();

				target_cable_cnt = updateTargetCablePose(veh_state, cable_id) ? target_cable_cnt : target_cable_cnt-1;

				fixed_reference = loadTargetUnderCableState();

				setTrajectoryTarget(fixed_reference);

				if (direct_target_setpoint || withinDirectTargetDistance(veh_state, fixed_reference)) {

					direct_target_setpoint = true;

					// debug within direct target setpoint dist threshold, setting direct target setpoint: %f, %f, %f, %f
					RCLCPP_DEBUG(this->get_logger(), "within direct target setpoint dist threshold, setting direct target setpoint: %f, %f, %f, %f", set_point(0), set_point(1), set_point(2), set_point(3));
					
					if (configurator_.use_cartesian_PID()) {

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

				offboard_cnt = configurator_.offboard_cnt_timeout();

				state_ = in_positional_flight;

			} else if (tryPendingRequest(cable_landing_request, if_match, if_match)) { 

				direct_target_setpoint = false;

				// debug cable landing request, landing on cable
				RCLCPP_DEBUG(this->get_logger(), "cable landing request, landing on cable");

				cable_landing_request_params_t *request_params = (cable_landing_request_params_t *)request.request_params;
				cable_id = request_params->cable_id;
				
				delete request.request_params;

				target_cable_cnt = configurator_.target_cable_cnt_timeout();

				target_cable_cnt = updateTargetCablePose(veh_state, cable_id) ? target_cable_cnt : target_cable_cnt-1;

				fixed_reference = loadTargetCableState();

				float on_cable_upwards_velocity = configurator_.on_cable_upwards_velocity();

				fixed_reference(6) = on_cable_upwards_velocity;

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

			} else {

				rejectPendingRequest();

				target_cable_cnt = updateTargetCablePose(veh_state) ? target_cable_cnt : target_cable_cnt-1;

				fixed_reference = setZeroVelocity(loadTargetUnderCableState());

				setTrajectoryTarget(fixed_reference);

				if (configurator_.use_cartesian_PID()) {

					vector_t vel_control = stepCartesianVelocityPID(veh_state, fixed_reference, false);
					set_point = setVelocityControl(fixed_reference, vel_control);

				} else {

					set_point = setNanVelocity(fixed_reference);

				}

			}

			break;

		}

		case in_positional_flight: {

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

				//if (configurator_.use_cartesian_PID()) {

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

				if (configurator_.use_cartesian_PID()) {

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

					if (configurator_.use_cartesian_PID()) {

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

					if (configurator_.use_cartesian_PID()) {

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

					if (configurator_.use_cartesian_PID()) {

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

				offboard_cnt = configurator_.offboard_cnt_timeout();

			}

			break;

		}

		case during_cable_landing: {

			bool hover_under_cable_on_aborted_cable_landing = configurator_.hover_under_cable_on_aborted_cable_landing();

			bool use_gripper_status_condition = configurator_.use_gripper_status_condition();

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
						if (configurator_.use_cartesian_PID()) {

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

			} else if (reachedPosition(veh_state, fixed_reference) || (use_gripper_status_condition && gripper_status_.gripper_status == gripper_status_.GRIPPER_STATUS_CLOSED)) {

				RCLCPP_DEBUG(this->get_logger(), "Reached cable or gripper is closed, going to state on_cable_armed");

				notifyCurrentRequest(success);

				clearPlannedTrajectory();

				fixed_reference = setZeroVelocity(fixed_reference);

				for (int i = 0; i < 4; i++) {

					set_point(i) = NAN;
					set_point(i+4) = 0;
					set_point(i+8) = NAN;

				}

				set_point(6) = configurator_.on_cable_upwards_velocity();

				std::string on_cable_control_mode = configurator_.on_cable_control_mode();

				if (on_cable_control_mode == "thrust") {

					is_on_cable_armed_using_thrust_control_ = true;

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
							if (configurator_.use_cartesian_PID()) {

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

		}

		case on_cable_armed: {

			// debug in on cable armed
			RCLCPP_DEBUG(this->get_logger(), "in on cable armed");

			if (!offboard) {

				RCLCPP_DEBUG(this->get_logger(), "Going to init state because vehicle not in offboard");

				rejectPendingRequest();

				state_ = init;

			} else if (!armed) {

				RCLCPP_DEBUG(this->get_logger(), "Disarmed, going to state on cable disarmed");

				state_ = on_cable_disarmed;

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

				set_point(6) = configurator_.on_cable_upwards_velocity();

				disarm_on_cable_cnt = configurator_.disarm_on_cable_cnt_timeout();

				disarmOnCable();

				has_started_disarm_on_cable_countdown = false;
				disarm_on_cable_flight_has_been_terminated = false;

				state_ = disarming_on_cable;

			} else {

				rejectPendingRequest();

				for (int i = 0; i < 4; i++) {

					set_point(i) = fixed_reference(i);
					set_point(i+4) = 0;
					set_point(i+8) = NAN;

				}

				set_point(6) = configurator_.on_cable_upwards_velocity();

				// debug stay on cable, setpoint: %f, %f, %f, %f
				RCLCPP_DEBUG(this->get_logger(), "stay on cable, setpoint: %f, %f, %f, %f", set_point(0), set_point(1), set_point(2), set_point(3));
			}

			break; 

		}

		case disarming_on_cable: {

			// debug in disarming on cable
			RCLCPP_DEBUG(this->get_logger(), "in disarming on cable state");

			float disarming_on_cable_max_descend_distance = configurator_.disarming_on_cable_max_descend_distance();

			if (disarm_on_cable_initial_position(2) - veh_state(2) >= abs(disarming_on_cable_max_descend_distance) && armed) {

				RCLCPP_DEBUG(this->get_logger(), "Descended too long distance, Going to state during cable takeoff");

				notifyCurrentRequest(fail);

				setModeOffboard();

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
					if (configurator_.use_cartesian_PID()) {

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
				RCLCPP_DEBUG(this->get_logger(), "Not armed, Going to state on cable disarmed");

				notifyCurrentRequest(success);

				state_ = on_cable_disarmed;

			} else {

				bool disarm_on_cable_timeout = false;

				if (is_disarming_on_cable_by_thrust_) {

					double seconds_elapsed = (rclcpp::Clock().now() - disarm_on_cable_thrust_start_time_).seconds();

					double disarm_on_cable_thrust_decrease_time_s = configurator_.disarm_on_cable_thrust_decrease_time_s();
					double disarm_on_cable_thrust_wait_for_disarm_time_s = configurator_.disarm_on_cable_thrust_wait_for_disarm_time_s();

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

					set_point(6) = configurator_.on_cable_upwards_velocity();

					std::string on_cable_control_mode = configurator_.on_cable_control_mode();

					if (on_cable_control_mode == "thrust") {

						is_on_cable_armed_using_thrust_control_ = true;

					}

					arm();

					state_ = on_cable_armed;

				} else {
					
					rejectPendingRequest();

					for (int i = 0; i < 4; i++) {

						set_point(i) = fixed_reference(i);
						set_point(i+4) = 0;
						set_point(i+8) = NAN;

					}

					set_point(6) = configurator_.on_cable_upwards_velocity();

					disarm_on_cable_cnt--;

					// debug disarming on cable, setpoint: %f, %f, %f, %f
					RCLCPP_DEBUG(this->get_logger(), "disarming on cable, setpoint: %f, %f, %f, %f", set_point(0), set_point(1), set_point(2), set_point(3));

				}

				int disarm_on_cable_flight_termination_timeout_s = configurator_.disarm_on_cable_flight_termination_timeout_s();

				if (disarm_on_cable_flight_termination_timeout_s > 0) {

					bool condition = !offboard;

					bool use_gripper_status_condition = configurator_.use_gripper_status_condition();

					if (use_gripper_status_condition) {

						iii_drone_interfaces::msg::GripperStatus gripper_status; {

							std::lock_guard<std::mutex> lock(gripper_status_mutex_);

							gripper_status = gripper_status_;

						}

						condition = condition && gripper_status.gripper_status == iii_drone_interfaces::msg::GripperStatus::GRIPPER_STATUS_CLOSED;

					}

					if (condition) {
			
						if (!has_started_disarm_on_cable_countdown) {

							RCLCPP_DEBUG(this->get_logger(), "Not offboard, during disarming on cable, starting start timer");

							has_started_disarm_on_cable_countdown = true;
							disarm_on_cable_start_time = rclcpp::Clock().now();

						} else {

							double seconds_elapsed = (rclcpp::Clock().now() - disarm_on_cable_start_time).seconds();

							if (seconds_elapsed > disarm_on_cable_flight_termination_timeout_s && !disarm_on_cable_flight_has_been_terminated) {

								RCLCPP_DEBUG(this->get_logger(), "Not offboard, during disarming on cable, flight termination timer elapsed, terminating flight");

								disarm_on_cable_flight_has_been_terminated = true;
								disarm(true);

							}
						}


					}
				}
			}

			break;

		}

		case on_cable_disarmed: {

			// debug in on cable disarmed
			RCLCPP_DEBUG(this->get_logger(), "in on cable disarmed state");

			if (tryPendingRequest(arm_on_cable_request, if_match, if_match)) {

				// Debug arm on cable request, going to state arming on cable
				RCLCPP_DEBUG(this->get_logger(), "arm on cable request, going to state arming on cable");

				arm();

				setTrajectoryTarget(fixed_reference);

				set_point = setNanVelocity(fixed_reference);

				set_point(6) = configurator_.on_cable_upwards_velocity();

				arm_cnt = configurator_.arm_cnt_timeout();

				std::string on_cable_control_mode = configurator_.on_cable_control_mode();

				if (on_cable_control_mode == "thrust") {

					is_on_cable_armed_using_thrust_control_ = true;

				}

				state_ = arming_on_cable;

				// debug set trajectory target: %f %f %f %f
				RCLCPP_DEBUG(this->get_logger(), "set trajectory target: %f %f %f %f", fixed_reference(0), fixed_reference(1), fixed_reference(2), fixed_reference(3));

			} else  {
				
				rejectPendingRequest();

			}

			break;

		}
	
		case arming_on_cable: {

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

				for (int i = 0; i < 4; i++) {

					set_point(i) = fixed_reference(i);
					set_point(i+4) = 0;
					set_point(i+8) = NAN;

				}

				set_point(6) = configurator_.on_cable_upwards_velocity();

				offboard_cnt = configurator_.offboard_cnt_timeout();

				state_ = setting_offboard_on_cable;

			} else {

				arm_cnt--;

				setTrajectoryTarget(fixed_reference);

				for (int i = 0; i < 4; i++) {

					set_point(i) = fixed_reference(i);
					set_point(i+4) = 0;
					set_point(i+8) = NAN;

				}

				set_point(6) = configurator_.on_cable_upwards_velocity();

				rejectPendingRequest();

			}

			break;

		}

		case setting_offboard_on_cable: {

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
				RCLCPP_DEBUG(this->get_logger(), "offboard, starting spool up timer");

				for (int i = 0; i < 4; i++) {

					set_point(i) = fixed_reference(i);
					set_point(i+4) = 0;
					set_point(i+8) = NAN;

				}

				set_point(6) = configurator_.on_cable_upwards_velocity();

				static bool timer_started = false;
				static float timer_value;
				static int controller_period_ms;
				static float dt = configurator_.dt();
				controller_period_ms = (int)(dt*1000);

				if (!timer_started) {

					timer_value = configurator_.arming_on_cable_spool_up_time_s();
					timer_started = true;

				} else if (timer_value <= 0) {

					timer_started = false;

					std::string on_cable_control_mode = configurator_.on_cable_control_mode();

					if (on_cable_control_mode == "thrust") {

						is_on_cable_armed_using_thrust_control_ = true;

					}

					notifyCurrentRequest(success);

					state_ = on_cable_armed;

				} else {

					timer_value -= ((float)controller_period_ms)/1000.;

					state_ = setting_offboard_on_cable;

				}

			} else {

				for (int i = 0; i < 4; i++) {

					set_point(i) = fixed_reference(i);
					set_point(i+4) = 0;
					set_point(i+8) = NAN;

				}
				set_point(6) = configurator_.on_cable_upwards_velocity();

				offboard_cnt--;

				rejectPendingRequest();

			}

			break; 

		}

		case during_cable_takeoff: {

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

				if (configurator_.use_cartesian_PID()) {
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
	}

	publishControlState();
	publishPlannedTrajectory();
	publishTargetCableId();

	publishOffboardControlMode();
	if (offboard && configurator_.always_hover_in_offboard() && state_ != hovering)
		set_point = always_hover_state;

	if (is_disarming_on_cable_by_thrust_ || is_on_cable_armed_using_thrust_control_) {

		publishActuatorSetpoints();

	} else {

		publishTrajectorySetpoint(set_point);

	}

	publishSetpointPose(set_point);

	if (configurator_.use_ground_altitude_offset()) {
		publishGroundAltitudeOffsetTf(ground_altitude_offset);
	}

}

void FlightController::odometryCallback(px4_msgs::msg::VehicleOdometry::SharedPtr msg) {

	quaternion_t q(
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

	static float dt = configurator_.dt();
	int controller_period_ms = (int)(dt*1000);

	odometry_mutex_.lock(); {

		odom_q_ = q;
		odom_ang_vel_ = ang_vel;
		odom_pos_ = pos;
		vector_t last_vel = odom_vel_;
		odom_vel_ = vel;
		odom_acc_ = (vel - last_vel) * 1000 / controller_period_ms;

	} odometry_mutex_.unlock();

}

void FlightController::powerlineCallback(iii_drone_interfaces::msg::Powerline::SharedPtr msg) {

	powerline_mutex_.lock(); {

		powerline_ = *msg;

	} powerline_mutex_.unlock();

}

void FlightController::gripperStatusCallback(iii_drone_interfaces::msg::GripperStatus::SharedPtr msg) {

	gripper_status_mutex_.lock(); {

		gripper_status_ = *msg;

	} gripper_status_mutex_.unlock();

}

void FlightController::homePositionCallback(px4_msgs::msg::HomePosition::SharedPtr msg) {

	bool update_home = configurator_.update_home_position();

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

void FlightController::setHomePosition(px4_msgs::msg::HomePosition new_home) {

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

void FlightController::setHomePositionIfChanged(px4_msgs::msg::HomePosition old_home, px4_msgs::msg::HomePosition new_home) {

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

bool FlightController::isOffboard() {

	return nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;

}

bool FlightController::isArmed() {

	return arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;

}

void FlightController::setModeOffboard() {

	publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

}

void FlightController::land() {

	publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);

}

/**
 * @brief Send a command to Arm the vehicle
 */
void FlightController::arm() {

	//armed = true;
	publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_DEBUG(this->get_logger(), "Arm command send");

}


/**
 * @brief Send a command to Disarm the vehicle
 */
void FlightController::disarm(bool force) {

	//armed = false;
	float force_param = force ? 21196 : 0.0;
	publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, force_param);
	if (!force) {
		RCLCPP_DEBUG(this->get_logger(), "Disarm command send");
	} else {
		RCLCPP_DEBUG(this->get_logger(), "Force disarm command send");
	}

}

void FlightController::disarmOnCable() {

	std::string disarm_on_cable_mode = configurator_.disarm_on_cable_mode();

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
void FlightController::publishVehicleCommand(uint16_t command, float param1,
					      float param2, float param3, float param4,
					      float param5, float param6,
					      float param7) {

	px4_msgs::msg::VehicleCommand msg{};
	// msg.timestamp = timestamp_.load();
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
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
void FlightController::publishOffboardControlMode() {
	px4_msgs::msg::OffboardControlMode msg{};
	// msg.timestamp = timestamp_.load();
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

	if (is_on_cable_armed_using_thrust_control_ && (state_ == on_cable_armed || state_ == arming_on_cable || state_ == setting_offboard_on_cable)) {

		is_disarming_on_cable_by_thrust_ = false;

		msg.position = false;
		msg.velocity = false;
		msg.acceleration = false;
		msg.attitude = false;
		msg.body_rate = false;	
		msg.actuator = true;

	} else if (is_disarming_on_cable_by_thrust_ && state_ == disarming_on_cable) {

		is_on_cable_armed_using_thrust_control_ = false;

		msg.position = false;
		msg.velocity = false;
		msg.acceleration = false;
		msg.attitude = false;
		msg.body_rate = false;	
		msg.actuator = true;

	} else {

		is_on_cable_armed_using_thrust_control_ = false;
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

void FlightController::publishControlState() {

	iii_drone_interfaces::msg::ControlState msg;

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

	case disarming_on_cable:

		msg.state = msg.CONTROL_STATE_DISARMING_ON_CABLE;
		break;

	case on_cable_disarmed:

		msg.state = msg.CONTROL_STATE_ON_CABLE_DISARMED;
		break;

	case arming_on_cable:

		msg.state = msg.CONTROL_STATE_ARMING_ON_CABLE;
		break;

	case setting_offboard_on_cable:

		msg.state = msg.CONTROL_STATE_SETTING_OFFBOARD_ON_CABLE;
		// Dummy comment
		break;	   
	}

	control_state_pub_->publish(msg);

}

/**
 * @brief Publish a trajectory setpoint
 */
void FlightController::publishTrajectorySetpoint(state4_t set_point) {

	static rotation_matrix_t R_NED_to_body_frame = eulToMat(euler_angles_t(M_PI, 0, 0));

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

	// msg.timestamp = timestamp_.load();
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
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

void FlightController::publishActuatorSetpoints() {

	double on_cable_upwards_thrust = configurator_.on_cable_upwards_thrust();
	double disarm_on_cable_thrust_decrease_time_s = configurator_.disarm_on_cable_thrust_decrease_time_s();

	double a, seconds_elapsed, thrust;

	if (is_disarming_on_cable_by_thrust_) {

		a = -on_cable_upwards_thrust / disarm_on_cable_thrust_decrease_time_s;

		seconds_elapsed = (this->get_clock()->now() - disarm_on_cable_thrust_start_time_).seconds();

		thrust = a * seconds_elapsed + on_cable_upwards_thrust;

	} else if (is_on_cable_armed_using_thrust_control_) {

		thrust = on_cable_upwards_thrust;

	} else {

		RCLCPP_FATAL(this->get_logger(), "publishActuatorSetpoints called when not supposed to control by thrust");
		exit(1);

	}

	if (thrust > 1) {

		thrust = 1;

	} else if (thrust < 0) {

		thrust = 0;

	}

	px4_msgs::msg::VehicleThrustSetpoint thrust_msg{};
	// thrust_msg.timestamp_sample = timestamp_.load();
	thrust_msg.timestamp_sample = this->get_clock()->now().nanoseconds() / 1000;
	// thrust_msg.timestamp = timestamp_.load();
	thrust_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	thrust_msg.xyz[0] = 0;
	thrust_msg.xyz[1] = 0;
	thrust_msg.xyz[2] = -thrust;

	px4_msgs::msg::VehicleTorqueSetpoint torque_msg{};
	// torque_msg.timestamp_sample = timestamp_.load();
	torque_msg.timestamp_sample = this->get_clock()->now().nanoseconds() / 1000;
	// torque_msg.timestamp = timestamp_.load();
	torque_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	torque_msg.xyz[0] = 0;
	torque_msg.xyz[1] = 0;
	torque_msg.xyz[2] = 0;

	thrust_setpoint_pub_->publish(thrust_msg);
	torque_setpoint_pub_->publish(torque_msg);

}

void FlightController::publishSetpointPose(state4_t set_point) {

	geometry_msgs::msg::PoseStamped msg;

	msg.header.stamp = this->get_clock()->now();
	msg.header.frame_id = "world";

	msg.pose.position.x = set_point(0);
	msg.pose.position.y = set_point(1);
	msg.pose.position.z = set_point(2);
 
	quaternion_t q = eulToQuat(euler_angles_t(0, 0, set_point(3)));

	msg.pose.orientation.w = q(0);
	msg.pose.orientation.x = q(1);
	msg.pose.orientation.y = q(2);
	msg.pose.orientation.z = q(3);

	setpoint_pose_pub_->publish(msg);

}

void FlightController::publishPlannedTrajectory() {

	nav_msgs::msg::Path path = loadPlannedPath();

	geometry_msgs::msg::PoseStamped target;

	if (path.poses.size() == 0) {

		target = loadVehiclePose();
		path.poses.push_back(target);

		if (state_ == arming || state_ == setting_offboard || state_ == taking_off || state_ == hovering_under_cable \
			|| state_ == during_cable_takeoff || state_ == during_cable_landing || state_ == on_cable_armed || state_ == hovering) {

			target = loadPlannedTarget();

		}

	} else {

		target = loadPlannedTarget();

	}

	planned_traj_pub_->publish(path);
	planned_target_pub_->publish(target);

}

void FlightController::publishTargetCableId() {

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


void FlightController::publishGroundAltitudeOffsetTf(state3_t ground_altitude_offset) {

	rclcpp::Time now = this->get_clock()->now();
	geometry_msgs::msg::TransformStamped t;

	t.header.stamp = now;
	t.header.frame_id = "world";
	t.child_frame_id = "ground";

	t.transform.translation.x = ground_altitude_offset(0);
	t.transform.translation.y = ground_altitude_offset(1);
	t.transform.translation.z = ground_altitude_offset(2);

	t.transform.rotation.w = 1;
	t.transform.rotation.x = 0;
	t.transform.rotation.y = 0;
	t.transform.rotation.z = 0;

	tf_broadcaster_->sendTransform(t);

}

state4_t FlightController::loadVehicleState() {

	static rotation_matrix_t R_NED_to_body_frame = eulToMat(euler_angles_t(M_PI, 0, 0));

	quaternion_t q;
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

	euler_angles_t eul = quatToEul(q);

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

geometry_msgs::msg::PoseStamped FlightController::loadVehiclePose() {

	state4_t veh_state = loadVehicleState();

	geometry_msgs::msg::PoseStamped pose;

	pose.header.frame_id = "world";
	pose.header.stamp = this->get_clock()->now();

	pose.pose.position.x = veh_state(0);
	pose.pose.position.y = veh_state(1);
	pose.pose.position.z = veh_state(2);

	euler_angles_t eul(0,0,veh_state(3));
	quaternion_t quat = eulToQuat(eul);

	pose.pose.orientation.w = quat(0);
	pose.pose.orientation.x = quat(1);
	pose.pose.orientation.y = quat(2);
	pose.pose.orientation.z = quat(3);

	return pose;

}

nav_msgs::msg::Path FlightController::loadPlannedPath() {

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

		euler_angles_t eul(0,0,state(3));
		quaternion_t quat = eulToQuat(eul);

		pose.pose.orientation.w = quat(0);
		pose.pose.orientation.x = quat(1);
		pose.pose.orientation.y = quat(2);
		pose.pose.orientation.z = quat(3);

		poses_vec[i] = pose;

	}

	path.poses = poses_vec;

	return path;

}

geometry_msgs::msg::PoseStamped FlightController::loadPlannedTarget() {

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

	euler_angles_t eul(0,0,target_cp(3));
	quaternion_t quat = eulToQuat(eul);

	target_pose.pose.orientation.w = quat(0);
	target_pose.pose.orientation.x = quat(1);
	target_pose.pose.orientation.y = quat(2);
	target_pose.pose.orientation.z = quat(3);

	return target_pose;

}

state4_t FlightController::loadTargetCableState() {

	state4_t cable_state;

	geometry_msgs::msg::Pose cable_pose;

	powerline_mutex_.lock(); {

		cable_pose = target_cable_pose_.pose;

	} powerline_mutex_.unlock();

	quaternion_t cable_quat(
		cable_pose.orientation.w,
		cable_pose.orientation.x,
		cable_pose.orientation.y,
		cable_pose.orientation.z
	);

	euler_angles_t cable_eul = quatToEul(cable_quat);

	geometry_msgs::msg::TransformStamped T_drone_to_cable_gripper = tf_buffer_->lookupTransform("cable_gripper", "drone", tf2::TimePointZero);

	quaternion_t q_drone_to_cable_gripper(
		T_drone_to_cable_gripper.transform.rotation.w,
		T_drone_to_cable_gripper.transform.rotation.x,
		T_drone_to_cable_gripper.transform.rotation.y,
		T_drone_to_cable_gripper.transform.rotation.z
	);

	euler_angles_t eul_drone_to_cable_gripper = quatToEul(q_drone_to_cable_gripper);

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

state4_t FlightController::loadTargetUnderCableState() {

	state4_t under_cable_state;

	geometry_msgs::msg::Pose cable_pose;

	powerline_mutex_.lock(); {

		cable_pose = target_cable_pose_.pose;

	} powerline_mutex_.unlock();

	quaternion_t cable_quat(
		cable_pose.orientation.w,
		cable_pose.orientation.x,
		cable_pose.orientation.y,
		cable_pose.orientation.z
	);

	euler_angles_t cable_eul = quatToEul(cable_quat);

	geometry_msgs::msg::TransformStamped T_drone_to_cable_gripper = tf_buffer_->lookupTransform("cable_gripper", "drone", tf2::TimePointZero);

	quaternion_t q_drone_to_cable_gripper(
		T_drone_to_cable_gripper.transform.rotation.w,
		T_drone_to_cable_gripper.transform.rotation.x,
		T_drone_to_cable_gripper.transform.rotation.y,
		T_drone_to_cable_gripper.transform.rotation.z
	);

	euler_angles_t eul_drone_to_cable_gripper = quatToEul(q_drone_to_cable_gripper);

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

vector_t FlightController::stepCartesianVelocityPID(state4_t vehicle_state, state4_t target_state, bool reset) {

	// Variables:
	static bool first = true;

	static vector_t prev_error;

	static vector_t integral_error;

	static vector_t derivative_error;

	static const vector_t zero_vec(0,0,0);

	static double dt = configurator_.dt();

	double Kp = configurator_.cartesian_PID_Kp();
	double Ki = configurator_.cartesian_PID_Ki();
	double Kd = configurator_.cartesian_PID_Kd();
	double i_reset_error_threshold = configurator_.cartesian_PID_integral_reset_error_threshold();

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

void FlightController::clearPlannedTrajectory() {

	planned_trajectory_mutex_.lock(); {

		planned_trajectory_.clear();
		planned_trajectory_.resize(0);

		for (int i = 0; i < 8; i++) trajectory_target_(i) = NAN;

	} planned_trajectory_mutex_.unlock();

}

void FlightController::setTrajectoryTarget(state4_t target) {

	planned_trajectory_mutex_.lock(); {

		trajectory_target_ = target;

	} planned_trajectory_mutex_.unlock();

}

bool FlightController::updateTargetCablePose(state4_t vehicle_state, int new_id) {

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
				quaternion_t cable_quat(
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

void FlightController::clearTargetCable() {

	powerline_mutex_.lock(); {

		target_cable_id_ = -1;

	} powerline_mutex_.unlock();

}

int main(int argc, char* argv[]) {
	std::cout << "Starting cable landing controller node..." << std::endl;

	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<FlightController>());

	rclcpp::shutdown();
	return 0;
}
