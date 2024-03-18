#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

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

/*****************************************************************************/
// Eigen:

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

/*****************************************************************************/
// PX4 msgs:

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/position_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

/*****************************************************************************/
// ROS2:

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

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/msg/control_state.hpp>
#include <iii_drone_interfaces/msg/powerline.hpp>
#include <iii_drone_interfaces/msg/gripper_status.hpp>
              
#include <iii_drone_interfaces/action/takeoff.hpp>
#include <iii_drone_interfaces/action/landing.hpp>
#include <iii_drone_interfaces/action/fly_to_position.hpp>
#include <iii_drone_interfaces/action/cable_landing.hpp>
#include <iii_drone_interfaces/action/cable_takeoff.hpp>
#include <iii_drone_interfaces/action/fly_under_cable.hpp>
#include <iii_drone_interfaces/action/disarm_on_cable.hpp>
#include <iii_drone_interfaces/action/arm_on_cable.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/math.hpp>
#include <iii_drone_core/utils/types.hpp>
#include <iii_drone_core/utils/blocking_queue.hpp>
#include <iii_drone_core/utils/atomic.hpp>
#include <iii_drone_core/utils/history.hpp>

#include <iii_drone_core/control/flight_controller_node/flight_controller_node_configurator.hpp>
#include <iii_drone_core/control/mpc_parameters.hpp>
#include <iii_drone_core/control/flight_control_requests.hpp>
#include <iii_drone_core/control/setpoint_controller.hpp>

#include <iii_drone_core/control/combined_drone_awareness_handler.hpp>
#include <iii_drone_core/control/maneuver/maneuver_scheduler.hpp>

#include <iii_drone_core/adapters/px4/vehicle_status_adapter.hpp>
#include <iii_drone_core/adapters/px4/vehicle_odometry_adapter.hpp>
#include <iii_drone_core/adapters/px4/trajectory_setpoint_adapter.hpp>

#include <iii_drone_core/adapters/powerline_adapter.hpp>
#include <iii_drone_core/adapters/gripper_status_adapter.hpp>


/*****************************************************************************/
// Defines
/*****************************************************************************/

namespace iii_drone {
namespace control {
namespace flight_controller_node {

	/**
	 * @brief Trajectory controller FSM state
	 */
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

} // namespace flight_controller_node
} // namespace control
} // namespace iii_drone

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace control {
namespace flight_controller_node {

	/**
	 * @brief The trajectory controller node
	*/
	class FlightController : public rclcpp::Node {
	public:
		// using Takeoff = iii_drone_interfaces::action::Takeoff;
		// using GoalHandleTakeoff = rclcpp_action::ServerGoalHandle<Takeoff>;

		// using Landing = iii_drone_interfaces::action::Landing;
		// using GoalHandleLanding = rclcpp_action::ServerGoalHandle<Landing>;

		// using FlyToPosition = iii_drone_interfaces::action::FlyToPosition;
		// using GoalHandleFlyToPosition = rclcpp_action::ServerGoalHandle<FlyToPosition>;

		// using FlyUnderCable = iii_drone_interfaces::action::FlyUnderCable;
		// using GoalHandleFlyUnderCable = rclcpp_action::ServerGoalHandle<FlyUnderCable>;

		// using CableLanding = iii_drone_interfaces::action::CableLanding;
		// using GoalHandleCableLanding = rclcpp_action::ServerGoalHandle<CableLanding>;

		// using CableTakeoff = iii_drone_interfaces::action::CableTakeoff;
		// using GoalHandleCableTakeoff = rclcpp_action::ServerGoalHandle<CableTakeoff>;

		// using DisarmOnCable = iii_drone_interfaces::action::DisarmOnCable;
		// using GoalHandleDisarmOnCable = rclcpp_action::ServerGoalHandle<DisarmOnCable>;

		// using ArmOnCable = iii_drone_interfaces::action::ArmOnCable;
		// using GoalHandleArmOnCable = rclcpp_action::ServerGoalHandle<ArmOnCable>;

		/**
		 * @brief Construct a new Trajectory Controller object
		 * 
		 * @param node_name The name of the node
		 * @param node_namespace The namespace of the node
		 * @param options The node options
		 */
		FlightController(
			const std::string & node_name="flight_controller", 
			const std::string & node_namespace="/control/flight_controller", 
			const rclcpp::NodeOptions & options = rclcpp::NodeOptions()
		);

		/**
		 * @brief Destroy the Trajectory Controller object
		 * 
		 */
		~FlightController();

	private:
		// BehaviorState::SharedPtr behavior_state_;

		CombinedDroneAwarenessHandler::SharedPtr combined_drone_awareness_handler_;

		maneuver::ManeuverScheduler::SharedPtr maneuver_scheduler_;

		// // Takeoff action:
		// /**
		//  * @brief The takeoff action server
		// */
		// rclcpp_action::Server<Takeoff>::SharedPtr takeoff_server_;

		// /**
		//  * @brief The takeoff action goal callback
		//  * 
		//  * @param uuid The goal UUID
		//  * @param goal The goal
		//  * 
		//  * @return rclcpp_action::GoalResponse The goal response
		//  */
		// rclcpp_action::GoalResponse handleGoalTakeoff(
		// 	const rclcpp_action::GoalUUID & uuid, 
		// 	std::shared_ptr<const Takeoff::Goal> goal
		// );

		// /**
		//  * @brief The takeoff action cancel callback
		//  * 
		//  * @param goal_handle The goal handle
		//  * 
		//  * @return rclcpp_action::CancelResponse The cancel response
		//  */
		// rclcpp_action::CancelResponse handleCancelTakeoff(const std::shared_ptr<GoalHandleTakeoff> goal_handle);

		// /**
		//  * @brief The takeoff action accepted callback
		//  * 
		//  * @param goal_handle The goal handle
		//  * 
		//  * @return void
		//  */
		// void handleAcceptedTakeoff(const std::shared_ptr<GoalHandleTakeoff> goal_handle);

		// /**
		//  * @brief The takeoff action thread which monitors the status of the takeoff action completion
		//  * 
		//  * @param goal_handle The goal handle
		//  * 
		//  * @return void
		//  */
		// void followTakeoffCompletion(const std::shared_ptr<GoalHandleTakeoff> goal_handle);

		// // Landing action:

		// /**
		//  * @brief The landing action server
		// */
		// rclcpp_action::Server<Landing>::SharedPtr landing_server_;

		// /**
		//  * @brief The landing action goal callback
		//  * 
		//  * @param uuid The goal UUID
		//  * @param goal The goal
		//  * 
		//  * @return rclcpp_action::GoalResponse The goal response
		//  */
		// rclcpp_action::GoalResponse handleGoalLanding(
		// 	const rclcpp_action::GoalUUID & uuid, 
		// 	std::shared_ptr<const Landing::Goal> goal
		// );
		
		// /**
		//  * @brief The landing action cancel callback
		//  * 
		//  * @param goal_handle The goal handle
		//  * 
		//  * @return rclcpp_action::CancelResponse The cancel response
		//  */
		// rclcpp_action::CancelResponse handleCancelLanding(const std::shared_ptr<GoalHandleLanding> goal_handle);

		// /**
		//  * @brief The landing action accepted callback
		//  * 
		//  * @param goal_handle The goal handle
		//  * 
		//  * @return void
		//  */
		// void handleAcceptedLanding(const std::shared_ptr<GoalHandleLanding> goal_handle);

		// /**
		//  * @brief The landing action thread which monitors the status of the landing action completion
		//  * 
		//  * @param goal_handle The goal handle
		//  * 
		//  * @return void
		//  */
		// void followLandingCompletion(const std::shared_ptr<GoalHandleLanding> goal_handle);

		// // Fly to position action:
		// /**
		//  * @brief The fly to position action server
		// */
		// rclcpp_action::Server<FlyToPosition>::SharedPtr fly_to_position_server_;

		// /**
		//  * @brief The fly to position action goal callback
		//  * 
		//  * @param uuid The goal UUID
		//  * @param goal The goal
		//  * 
		//  * @return rclcpp_action::GoalResponse The goal response
		//  */
		// rclcpp_action::GoalResponse handleGoalFlyToPosition(
		// 	const rclcpp_action::GoalUUID & uuid, 
		// 	std::shared_ptr<const FlyToPosition::Goal> goal
		// );

		// /**
		//  * @brief The fly to position action cancel callback
		//  * 
		//  * @param goal_handle The goal handle
		//  * 
		//  * @return rclcpp_action::CancelResponse The cancel response
		//  */
		// rclcpp_action::CancelResponse handleCancelFlyToPosition(const std::shared_ptr<GoalHandleFlyToPosition> goal_handle);

		// /**
		//  * @brief The fly to position action accepted callback
		//  * 
		//  * @param goal_handle The goal handle
		//  * 
		//  * @return void
		//  */
		// void handleAcceptedFlyToPosition(const std::shared_ptr<GoalHandleFlyToPosition> goal_handle);

		// /**
		//  * @brief The fly to position action thread which monitors the status of the fly to position action completion
		//  * 
		//  * @param goal_handle The goal handle
		//  * 
		//  * @return void
		//  */
		// void followFlyToPositionCompletion(const std::shared_ptr<GoalHandleFlyToPosition> goal_handle);

		// // Fly under cable action:
		// /**
		//  * @brief The fly under cable action server
		// */
		// rclcpp_action::Server<FlyUnderCable>::SharedPtr fly_under_cable_server_;

		// /**
		//  * @brief The fly under cable action goal callback
		//  * 
		//  * @param uuid The goal UUID
		//  * @param goal The goal
		//  * 
		//  * @return rclcpp_action::GoalResponse The goal response
		//  */
		// rclcpp_action::GoalResponse handleGoalFlyUnderCable(
		// 	const rclcpp_action::GoalUUID & uuid, 
		// 	std::shared_ptr<const FlyUnderCable::Goal> goal
		// );

		// /**
		//  * @brief The fly under cable action cancel callback
		//  * 
		//  * @param goal_handle The goal handle
		//  * 
		//  * @return rclcpp_action::CancelResponse The cancel response
		//  */
		// rclcpp_action::CancelResponse handleCancelFlyUnderCable(const std::shared_ptr<GoalHandleFlyUnderCable> goal_handle);

		// /**
		//  * @brief The fly under cable action accepted callback
		//  * 
		//  * @param goal_handle The goal handle
		//  * 
		//  * @return void
		//  */
		// void handleAcceptedFlyUnderCable(const std::shared_ptr<GoalHandleFlyUnderCable> goal_handle);

		// /**
		//  * @brief The fly under cable action thread which monitors the status of the fly under cable action completion
		//  * 
		//  * @param goal_handle The goal handle
		//  * 
		//  * @return void
		//  */
		// void followFlyUnderCableCompletion(const std::shared_ptr<GoalHandleFlyUnderCable> goal_handle);

		// // Cable landing action:
		// /**
		//  * @brief The cable landing action server
		// */
		// rclcpp_action::Server<CableLanding>::SharedPtr cable_landing_server_;

		// /**
		//  * @brief The cable landing action goal callback
		//  * 
		//  * @param uuid The goal UUID
		//  * @param goal The goal
		//  * 
		//  * @return rclcpp_action::GoalResponse The goal response
		//  */
		// rclcpp_action::GoalResponse handleGoalCableLanding(
		// 	const rclcpp_action::GoalUUID & uuid, 
		// 	std::shared_ptr<const CableLanding::Goal> goal
		// );

		// /**
		//  * @brief The cable landing action cancel callback
		//  * 
		//  * @param goal_handle The goal handle
		//  * 
		//  * @return rclcpp_action::CancelResponse The cancel response
		//  */
		// rclcpp_action::CancelResponse handleCancelCableLanding(const std::shared_ptr<GoalHandleCableLanding> goal_handle);

		// /**
		//  * @brief The cable landing action accepted callback
		//  * 
		//  * @param goal_handle The goal handle
		//  * 
		//  * @return void
		//  */
		// void handleAcceptedCableLanding(const std::shared_ptr<GoalHandleCableLanding> goal_handle);

		// /**
		//  * @brief The cable landing action thread which monitors the status of the cable landing action completion
		//  * 
		//  * @param goal_handle The goal handle
		//  * 
		//  * @return void
		//  */
		// void followCableLandingCompletion(const std::shared_ptr<GoalHandleCableLanding> goal_handle);

		// // Cable takeoff action:

		// /**
		//  * @brief The cable takeoff action server
		// */
		// rclcpp_action::Server<CableTakeoff>::SharedPtr cable_takeoff_server_;

		// /**
		//  * @brief The cable takeoff action goal callback
		//  * 
		//  * @param uuid The goal UUID
		//  * @param goal The goal
		//  * 
		//  * @return rclcpp_action::GoalResponse The goal response
		//  */
		// rclcpp_action::GoalResponse handleGoalCableTakeoff(
		// 	const rclcpp_action::GoalUUID & uuid, 
		// 	std::shared_ptr<const CableTakeoff::Goal> goal
		// );

		// /**
		//  * @brief The cable takeoff action cancel callback
		//  * 
		//  * @param goal_handle The goal handle
		//  * 
		//  * @return rclcpp_action::CancelResponse The cancel response
		//  */
		// rclcpp_action::CancelResponse handleCancelCableTakeoff(const std::shared_ptr<GoalHandleCableTakeoff> goal_handle);

		// /**
		//  * @brief The cable takeoff action accepted callback
		//  * 
		//  * @param goal_handle The goal handle
		//  * 
		//  * @return void
		//  */
		// void handleAcceptedCableTakeoff(const std::shared_ptr<GoalHandleCableTakeoff> goal_handle);

		// /**
		//  * @brief The cable takeoff action thread which monitors the status of the cable takeoff action completion
		//  * 
		//  * @param goal_handle The goal handle
		//  * 
		//  * @return void
		//  */
		// void followCableTakeoffCompletion(const std::shared_ptr<GoalHandleCableTakeoff> goal_handle);

		// // Disarm on cable action:

		// /**
		//  * @brief The disarm on cable action server
		// */
		// rclcpp_action::Server<DisarmOnCable>::SharedPtr disarm_on_cable_server_;

		// /**
		//  * @brief The disarm on cable action goal callback
		//  * 
		//  * @param uuid The goal UUID
		//  * @param goal The goal
		//  * 
		//  * @return rclcpp_action::GoalResponse The goal response
		//  */
		// rclcpp_action::GoalResponse handleGoalDisarmOnCable(
		// 	const rclcpp_action::GoalUUID & uuid,
		// 	std::shared_ptr<const DisarmOnCable::Goal> goal
		// );

		// /**
		//  * @brief The disarm on cable action cancel callback
		//  * 
		//  * @param goal_handle The goal handle
		//  * 
		//  * @return rclcpp_action::CancelResponse The cancel response
		//  */
		// rclcpp_action::CancelResponse handleCancelDisarmOnCable(const std::shared_ptr<GoalHandleDisarmOnCable> goal_handle);

		// /**
		//  * @brief The disarm on cable action accepted callback
		//  * 
		//  * @param goal_handle The goal handle
		//  * 
		//  * @return void
		//  */
		// void handleAcceptedDisarmOnCable(const std::shared_ptr<GoalHandleDisarmOnCable> goal_handle);

		// /**
		//  * @brief The disarm on cable action thread which monitors the status of the disarm on cable action completion
		//  * 
		//  * @param goal_handle The goal handle
		//  * 
		//  * @return void
		//  */
		// void followDisarmOnCableCompletion(const std::shared_ptr<GoalHandleDisarmOnCable> goal_handle);

		// // Arm on cable action:

		// /**
		//  * @brief The arm on cable action server
		// */
		// rclcpp_action::Server<ArmOnCable>::SharedPtr arm_on_cable_server_;

		// /**
		//  * @brief The arm on cable action goal callback
		//  * 
		//  * @param uuid The goal UUID
		//  * @param goal The goal
		//  * 
		//  * @return rclcpp_action::GoalResponse The goal response
		//  */
		// rclcpp_action::GoalResponse handleGoalArmOnCable(
		// 	const rclcpp_action::GoalUUID & uuid,
		// 	std::shared_ptr<const ArmOnCable::Goal> goal
		// );

		// /**
		//  * @brief The arm on cable action cancel callback
		//  * 
		//  * @param goal_handle The goal handle
		//  * 
		//  * @return rclcpp_action::CancelResponse The cancel response
		//  */
		// rclcpp_action::CancelResponse handleCancelArmOnCable(const std::shared_ptr<GoalHandleArmOnCable> goal_handle);

		// /**
		//  * @brief The arm on cable action accepted callback
		//  * 
		//  * @param goal_handle The goal handle
		//  * 
		//  * @return void
		//  */
		// void handleAcceptedArmOnCable(const std::shared_ptr<GoalHandleArmOnCable> goal_handle);

		// /**
		//  * @brief The arm on cable action thread which monitors the status of the arm on cable action completion
		//  * 
		//  * @param goal_handle The goal handle
		//  * 
		//  * @return void
		//  */
		// void followArmOnCableCompletion(const std::shared_ptr<GoalHandleArmOnCable> goal_handle);

		// General member variables:
		/**
		 * @brief The trajectory controller node configurator
		*/
		FlightControllerConfigurator configurator_;

		// /**
		//  * @brief The FSM state of the node
		// */
		// iii_drone::utils::Atomic<state_t> state_ = init;




		// /**
		//  * @brief PX4 vehicle status subscription
		//  */
		// rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;

		// /**
		//  * @brief VehicleStatusAdapter object history for the PX4 vehicle status
		//  */
		// iii_drone::utils::History<iii_drone::adapters::px4::VehicleStatusAdapter> vehicle_status_adapter_history_;


		// /**
		//  * @brief PX4 odometry subscription
		//  */
		// rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;

		// /**
		//  * @brief Vehicle odometry adapter history.
		//  */
		// iii_drone::utils::History<iii_drone::adapters::px4::VehicleOdometryAdapter> vehicle_odometry_adapter_history_;


		// /**
		//  * @brief Powerline subscription
		//  */
		// rclcpp::Subscription<iii_drone_interfaces::msg::Powerline>::SharedPtr powerline_sub_;

		// /**
		//  * @brief Adapter history for the most recent powerline message received from pl_mapper
		// */
		// iii_drone::utils::History<iii_drone::adapters::PowerlineAdapter> powerline_adapter_history_;


		// /**
		//  * @brief Gripper status subscription
		//  */
		// rclcpp::Subscription<iii_drone_interfaces::msg::GripperStatus>::SharedPtr gripper_status_sub_;

		// /**
		//  * @brief Gripper status message adapter history
		//  */
		// iii_drone::utils::History<iii_drone::adapters::GripperStatusAdapter>::SharedPtr gripper_status_adapter_history_;


		/**
		 * @brief PX4 trajectory setpoint publisher
		 */
		rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;

		/**
		 * @brief Trajectory setpoint adapter.
		 */
		iii_drone::utils::Atomic<iii_drone::adapters::px4::TrajectorySetpointAdapter> trajectory_setpoint_adapter_;


		// /**
		//  * @brief The id of the current target cable
		// */
		// int target_cable_id_ = -1;

		// /**
		//  * @brief The pose of the current target cable
		// */
		// geometry_msgs::msg::PoseStamped target_cable_pose_;

		// /**
		//  * @brief The target distance to the cable
		// */
		// float target_cable_distance_ = -1;

		// /**
		//  * @brief The plane in which the target cable was sat
		// */
		// iii_drone::types::plane_t target_cable_plane_;

		/**
		 * @brief ROS2 transform listener
		*/
		std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};

		/**
		 * @brief ROS2 transform buffer
		*/
		tf2_ros::Buffer::SharedPtr tf_buffer_;

		/**
		 * @brief ROS2 transform broadcaster
		*/
		std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

		// /**
		//  * @brief Queue for requests from the action callbacks to the FSM
		// */
		// BlockingQueue<request_t> request_queue_;

		// /**
		//  * @brief Queue for replies from the FSM to the action callbacks
		// */
		// BlockingQueue<request_reply_t> request_reply_queue_;

		// /**
		//  * @brief The poll rate for the request reply queue for monitoring from the action callbacks
		// */
		// rclcpp::Rate request_reply_poll_rate_;

		// /**
		//  * @brief The poll rate for monitoring the completion of the actions from the action callbacks
		// */
		// rclcpp::Rate request_completion_poll_rate_;

		// /**
		//  * @brief The main state machine timer
		// */
		// rclcpp::TimerBase::SharedPtr main_state_machine_timer_;

		// /**
		//  * @brief PX4 vehicle command publisher
		// */
		// rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;

		// /**
		//  * @brief PX4 offboard control mode publisher
		// */
		// rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;

		// /**
		//  * @brief Publisher for FSM state
		// */
		// rclcpp::Publisher<iii_drone_interfaces::msg::ControlState>::SharedPtr control_state_pub_;

		// /**
		//  * @brief Publisher for current setpoint pose
		// */
		// rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pose_pub_;

		// /**
		//  * @brief Publisher for current target cable id
		// */
		// rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr target_cable_id_pub_;

		// // General member methods:
		// /**
		//  * @brief The main state machine callback
		//  * 
		//  * @return void
		// */
		// void stateMachineCallback();

		// /**
		//  * @brief The gripper status callback
		//  * 
		//  * @param msg The gripper status message
		//  * 
		//  * @return void
		// */
		// void gripperStatusCallback(iii_drone_interfaces::msg::GripperStatus::SharedPtr msg);

		// /**
		//  * @brief Whether PX4 is in offboard mode
		//  * 
		//  * @return bool Whether PX4 is in offboard mode
		// */
		// bool isOffboard();

		// /**
		//  * @brief Whether the drone is armed
		//  * 
		//  * @return bool Whether the drone is armed
		// */
		// bool isArmed();

		// /**
		//  * @brief Publishes a PX4 command to set the mode to offboard
		//  * 
		//  * @return void
		// */
		// void setModeOffboard();

		// /**
		//  * @brief Publishes a PX4 command to land the drone
		//  * 
		//  * @return void
		// */
		// void land();

		// /**
		//  * @brief Publishes a PX4 command to arm the drone
		//  * 
		//  * @return void
		// */
		// void arm();

		// /**
		//  * @brief Publishes a PX4 command to disarm the drone
		//  * 
		//  * @param force Whether to force disarm, even if the drone is not landed, default is false
		//  * 
		//  * @return void
		// */
		// void disarm(bool force=false);

		// /**
		//  * @brief Disarms the drone on the cable by issuing a land command or starting the cable disarm
		//  * using thrust, based on the cable disarm mode parameter.
		//  * 
		//  * @return void
		// */
		// void disarmOnCable();

		// /**
		//  * @brief Current thrust while disarming on the cable.
		// */
		// double disarm_on_cable_thrust_;

		// /**
		//  * @brief The time at which the drone started disarming on the cable.
		// */
		// rclcpp::Time disarm_on_cable_thrust_start_time_;

		// /**
		//  * @brief Whether the drone is currently disarming on the cable.
		// */
		// bool is_disarming_on_cable_by_thrust_ = false;

		// /**
		//  * @brief Whether the drone is currently on the cable using thrust control.
		// */
		// bool is_on_cable_armed_using_thrust_control_ = false;

		// /**
		//  * @brief Publishes a PX4 vehicle command 
		//  * 
		//  * @param command The command to publish
		//  * @param param1 The first parameter of the command, default is 0.0
		//  * @param param2 The second parameter of the command, default is 0.0
		//  * @param param3 The third parameter of the command, default is 0.0
		//  * @param param4 The fourth parameter of the command, default is 0.0
		//  * @param param5 The fifth parameter of the command, default is 0.0
		//  * @param param6 The sixth parameter of the command, default is 0.0
		//  * @param param7 The seventh parameter of the command, default is 0.0
		//  * 
		//  * @return void
		// */
		// void publishVehicleCommand(uint16_t command, float param1 = 0.0,
		// 				float param2 = 0.0,
		// 				float param3 = 0.0,
		// 				float param4 = 0.0,
		// 				float param5 = 0.0,
		// 				float param6 = 0.0,
		// 				float param7 = 0.0);

		// /**
		//  * @brief Publishes a PX4 OffboardControlMode message
		//  * 
		//  * @return void
		//  * 
		//  */
		// void publishOffboardControlMode();

		// /**
		//  * @brief Publishes the FSM state
		//  * 
		//  * @return void
		//  * 
		//  */
		// void publishControlState();

		// /**
		//  * @brief Publishes the id of the current target cable
		//  * 
		//  * @return void
		//  * 
		//  */
		// void publishTargetCableId();

		// /**
		//  * @brief Publishes the trajectory setpoint
		//  * 
		//  * @return void
		//  * 
		//  */
		// void publishTrajectorySetpoint(state4_t set_point) ;

		// /**
		//  * @brief Publishes the trajectory setpoint as a pose message
		//  * 
		//  * @return void
		//  * 
		//  */
		// void publishSetpointPose(state4_t set_point);

		// /**
		//  * @brief Publishes the transform to the ground altitude offset
		//  * 
		//  * @return void
		//  * 
		//  */
		// void publishGroundAltitudeOffsetTf(state3_t ground_altitude_offset);

		// /**
		//  * @brief Loads the vehicle state
		//  * 
		//  * @return The vehicle state
		//  * 
		//  */
		// state4_t loadVehicleState();

		// /**
		//  * @brief Loads the vehicle pose
		//  * 
		//  * @return The vehicle pose
		//  * 
		//  */
		// geometry_msgs::msg::PoseStamped loadVehiclePose();

		// /**
		//  * @brief Loads the currently planned path
		//  * 
		//  * @return The currently planned path
		//  * 
		//  */
		// nav_msgs::msg::Path loadPlannedPath();
		
		// /**
		//  * @brief Loads the currently planned target
		//  * 
		//  * @return The currently planned target
		//  * 
		//  */
		// geometry_msgs::msg::PoseStamped loadPlannedTarget();

		// /**
		//  * @brief Loads the desired drone state that would yield the gripper to be on the cable.
		//  * 
		//  * @return The desired target cable state
		//  * 
		//  */
		// state4_t loadTargetCableState();

		// /**
		//  * @brief Loads the desired drone state under the cable
		//  * 
		//  * @return The desired target under cable state
		//  * 
		//  */
		// state4_t loadTargetUnderCableState();

		// /**
		//  * @brief Forwards the cartesian PID controller one step
		//  * 
		//  * @param vehicle_state The current vehicle state
		//  * @param target_state The target state
		//  * @param reset Whether to reset the integral term of the PID controller
		//  * 
		//  * @return The cartesian velocity PID output
		// */
		// iii_drone::types::vector_t stepCartesianVelocityPID(
		// 	state4_t vehicle_state, 
		// 	state4_t target_state, 
		// 	bool reset
		// );


		// /**
		//  * @brief Updates the pose of the target cable either with current or new id.
		//  * 
		//  * @param vehicle_state The current vehicle state
		//  * @param new_id The new id of the target cable, default is -1 (use current id)
		//  * 
		//  * @return Whether the cable with the target id was currently visible
		// */
		// bool updateTargetCablePose(
		// 	state4_t vehicle_state, 
		// 	int new_id = -1
		// );

		// /**
		//  * @brief Clears the target cable id.
		//  * 
		//  * @return void
		// */
		// void clearTargetCable();

	};

} // namespace flight_controller_node
} // namespace control
} // namespace iii_drone

/*****************************************************************************/
// Main
/*****************************************************************************/

int main(int argc, char* argv[]);