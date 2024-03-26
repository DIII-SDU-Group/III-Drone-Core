#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <memory>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/types.hpp>

#include <iii_drone_core/control/maneuver/maneuver_types.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/action/fly_to_position.hpp>
#include <iii_drone_interfaces/action/fly_to_object.hpp>
#include <iii_drone_interfaces/action/cable_landing.hpp>
#include <iii_drone_interfaces/action/cable_takeoff.hpp>
#include <iii_drone_interfaces/action/hover.hpp>
#include <iii_drone_interfaces/action/hover_by_object.hpp>
#include <iii_drone_interfaces/action/hover_on_cable.hpp>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace control {
namespace maneuver {

    /**
	 * @brief Abstract representation of a maneuver. Used to pass maneuvers around in the system.
	 * Contains a maneuver type, parameters, and goal handle. Created from a maneuver server.
	 */
    class Maneuver {
    public:
		/**
		 * @brief Default.
		 */
		Maneuver();

		/**
		 * @brief Constructor from type and id.
		 * 
		 * @param maneuver_type Maneuver type
		 * @param uuid ROS2 UUID.
		 */
		Maneuver(
			maneuver_type_t maneuver_type,
			const rclcpp_action::GoalUUID & uuid
		);

		/**
		 * @brief Copy constructor.
		 * 
		 * @param other Other maneuver
		 */
        Maneuver(const Maneuver & rhs);

		/**
		 * @brief Static constructor from goal handle.
		 * 
		 * @tparam ActionT Action type
		 * @param goal_handle Goal handle
		 */
		template <typename ActionT>
		static Maneuver FromGoalHandle(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle);

		/**
		 * @brief Sets type and parameters from goal.
		 * 
		 * @tparam ActionT Action type
		 * @param goal Goal
		 */
		template <typename ActionT>
		void SetFromGoal(const std::shared_ptr<const typename ActionT::Goal> goal);

		/**
		 * @brief Triggers the maneuver to start by calling the underlying execute function on the goal handle.
		 * 
		 * @return void
		 */
		void Start();

		/**
		 * @brief Indicate that the maneuver has terminated. Will delete the goal handle pointer.
		 * 
		 * @param success Whether the maneuver was successful.
		 * 
		 * @return void
		 */
		void Terminate(bool success);

		/**
		 * @brief Published feedback to the goal handle.
		 * 
		 * @param feedback Feedback
		 * 
		 * @return void
		 */
		template <typename ActionT>
		void PublishFeedback(const std::shared_ptr<void> feedback);

        /**
		 * @brief Equality operator.
		 * 
		 * @param rhs Right-hand side
		 */
        bool operator==(const Maneuver & rhs) const;
		
		/**
		 * @brief Inequality operator.
		 * 
		 * @param rhs Right-hand side
		 */
		bool operator!=(const Maneuver & rhs) const;
			
		/**
		 * @brief Assignment operator.
		 * 
		 * @param rhs Right-hand side
		 */
		Maneuver & operator=(const Maneuver & rhs);

        /**
         * @brief Goal handle getter.
         * 
         * @return The goal handle.
         */
        const std::shared_ptr<void> goal_handle() const;

        /**
         * @brief UUID getter.
         * 
         * @return The UUID.
         */
        const rclcpp_action::GoalUUID uuid() const;

        /**
         * @brief Maneuver type getter.
         * 
         * @return The maneuver type.
         */
        maneuver_type_t maneuver_type() const;

        /**
         * @brief Maneuver parameters getter.
         * 
         * @return The maneuver parameters.
         */
        const std::shared_ptr<void> maneuver_params() const;

		/**
		 * @brief Creation time getter.
		 * 
		 * @return The creation time.
		 */
		const rclcpp::Time creation_time() const;

		/**
		 * @brief Start time getter.
		 * 
		 * @return The start time.
		 */
		const rclcpp::Time start_time() const;

		/**
		 * @brief Getter for the termianted flag.
		 *
		 * @return Whether the maneuver has terminated.
		 */
		bool terminated() const;

		/**
		 * @brief Getter for the success flag.
		 *
		 * @return Whether the maneuver was successful.
		 */
		bool success() const;

    private:
		/**
		 * @brief The ROS2 time at which the maneuver was created.
		 */
		rclcpp::Time creation_time_;

		/**
		 * @brief The ROS2 time at which the maneuver is started.
		 */
		rclcpp::Time start_time_;

		/**
		 * @brief The ROS2 action server goal handle void pointer.
		 */
        std::shared_ptr<void> goal_handle_;

		/**
		 * @brief The ROS2 goal UUID.
		 */
		rclcpp_action::GoalUUID uuid_;

		/**
		 * @brief The maneuver type.
		 */
        maneuver_type_t maneuver_type_;

		/**
		 * @brief The maneuver parameters void pointer.
		 */
		std::shared_ptr<void> maneuver_params_;

		/**
		 * @brief Whether the maneuver has terminated.
		 */
		bool terminated_ = false;

		/**
		 * @brief Whether the maneuver was successful.
		 */
		bool success_ = true;

    };

} // namespace maneuver
} // namespace control
} // namespace iii_drone
