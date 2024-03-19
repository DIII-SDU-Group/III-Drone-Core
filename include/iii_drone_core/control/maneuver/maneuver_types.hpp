#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/types.hpp>

#include <iii_drone_core/adapters/target_adapter.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/action/takeoff.hpp>
#include <iii_drone_interfaces/action/landing.hpp>
#include <iii_drone_interfaces/action/fly_to_position.hpp>
#include <iii_drone_interfaces/action/fly_to_object.hpp>
#include <iii_drone_interfaces/action/cable_landing.hpp>
#include <iii_drone_interfaces/action/cable_takeoff.hpp>
#include <iii_drone_interfaces/action/disarm_on_cable.hpp>
#include <iii_drone_interfaces/action/arm_on_cable.hpp>

#include <iii_drone_interfaces/msg/target.hpp>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

/*****************************************************************************/
// Defines
/*****************************************************************************/

namespace iii_drone {
namespace control {
namespace maneuver {

    /**
	 * @brief Maneuver type enum.
	 */
    typedef enum {
		MANEUVER_TYPE_NONE = -1,
        MANEUVER_TYPE_FLY_TO_POSITION = 0,
        MANEUVER_TYPE_FLY_TO_OBJECT = 1,
        MANEUVER_TYPE_CABLE_LANDING = 2,
        MANEUVER_TYPE_CABLE_TAKEOFF = 3,
		MANEUVER_TYPE_HOVER = 4,
		MANEUVER_TYPE_HOVER_BY_OBJECT = 5,
		MANEUVER_TYPE_HOVER_ON_CABLE = 6
    } maneuver_type_t;

	/**
	 * @brief FlyToPosition maneuver parameters
	 */
	struct fly_to_position_maneuver_params_t {
		std::string frame_id;
		iii_drone::types::point_t target_position;
		float target_yaw;

		fly_to_position_maneuver_params_t();
		fly_to_position_maneuver_params_t(
			const std::string frame_id,
			const iii_drone::types::point_t target_position,
			const float target_yaw
		);
		fly_to_position_maneuver_params_t(std::shared_ptr<void> params);

		iii_drone::types::point_t transform_target_position(
			const std::string target_frame_id,
			tf2_ros::Buffer::SharedPtr tf_buffer
		) const;

	};

	/**
	 * @brief CableTakeoff maneuver parameters
	 */
	struct cable_takeoff_maneuver_params_t {
		float target_cable_distance;

		cable_takeoff_maneuver_params_t();
		cable_takeoff_maneuver_params_t(float target_cable_distance);
		cable_takeoff_maneuver_params_t(std::shared_ptr<void> params);
	};

	/**
	 * @brief FlyToObject maneuver parameters
	 */
	struct fly_to_object_maneuver_params_t {
		iii_drone::adapters::TargetAdapter target_adapter;

		fly_to_object_maneuver_params_t();
		fly_to_object_maneuver_params_t(iii_drone::adapters::TargetAdapter target_adapter);
		fly_to_object_maneuver_params_t(std::shared_ptr<void> params);
	};

	/**
	 * @brief Hover maneuver parameters
	 */
	struct hover_maneuver_params_t {
		double duration_s;

		hover_maneuver_params_t();
		hover_maneuver_params_t(double duration_s);
		hover_maneuver_params_t(std::shared_ptr<void> params);
	};

	/**
	 * @brief HoverByObject maneuver parameters
	 */
	struct hover_by_object_maneuver_params_t {
		iii_drone::adapters::TargetAdapter target_adapter;
		double duration_s;

		hover_by_object_maneuver_params_t();
		hover_by_object_maneuver_params_t(
			iii_drone::adapters::TargetAdapter target_adapter,
			double duration_s
		);
		hover_by_object_maneuver_params_t(std::shared_ptr<void> params);
	};

	/**
	 * @brief HoverOnCable maneuver parameters
	 */
	struct hover_on_cable_maneuver_params_t {
		double target_z_velocity;
		double target_yaw_rate;
		double duration_s;

		hover_on_cable_maneuver_params_t();
		hover_on_cable_maneuver_params_t(
			double target_z_velocity, 
			double target_yaw_rate,
			double duration_s
		);
		hover_on_cable_maneuver_params_t(std::shared_ptr<void> params);
	};


} // namespace maneuver
} // namespace control
} // namespace iii_drone
