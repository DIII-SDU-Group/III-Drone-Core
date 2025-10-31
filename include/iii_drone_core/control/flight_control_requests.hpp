#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/types.hpp>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

/*****************************************************************************/
// Defines
/*****************************************************************************/

namespace iii_drone {
namespace control {

	/**
	 * @brief Takeoff request parameters
	 */
	struct takeoff_request_params_t {
		float takeoff_altitude;
	};

	/**
	 * @brief FlyToPosition request parameters
	 */
	struct fly_to_position_request_params_t {
		iii_drone::types::point4_t target_position;
	};

	/**
	 * @brief CableLanding request parameters
	 */
	struct cable_landing_request_params_t {
		int cable_id;
	};

	/**
	 * @brief CableTakeoff request parameters
	 */
	struct cable_takeoff_request_params_t {
		float target_cable_distance;
	};

	/**
	 * @brief FlyUnderCable request parameters
	 */
	struct fly_under_cable_request_params_t {
		int cable_id;
		float target_cable_distance;
	};

	/**
	 * @brief Types of action requests
	 */
	enum flight_control_request_type_t {
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

	/**
	 * @brief Request structure, used to communicate action requests to the main state machine
	 */
	struct flight_control_request_t {
		rclcpp_action::GoalUUID action_id;
		flight_control_request_type_t request_type;
		void *request_params;

		bool operator==(const flight_control_request_t & rhs) const;
		void operator=(const flight_control_request_t & rhs);
	};

	/**
	 * @brief Request reply types, used to communicate action request replies to the action callbacks
	 */
	enum flight_control_request_reply_type_t {
		accept,
		reject,
		success,
		fail,
		cancel
	};

	/**
	 * @brief Request reply structure, used to communicate action request replies to the action callbacks
	 */
	struct flight_control_request_reply_t {
		rclcpp_action::GoalUUID action_id;
		flight_control_request_reply_type_t reply_type;

		bool operator==(const flight_control_request_reply_t & rhs) const;
		void operator=(const flight_control_request_reply_t & rhs);
	};

	/**
	 * @brief Request queue action types
	 */
	enum flight_control_request_queue_action_t {
		yes,
		no,
		if_match
	};

} // namespace control
} // namespace iii_drone