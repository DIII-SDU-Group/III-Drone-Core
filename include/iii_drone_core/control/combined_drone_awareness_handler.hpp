#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <memory>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

/*****************************************************************************/
// PX4 msgs:

#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/msg/powerline.hpp>
#include <iii_drone_interfaces/msg/gripper_status.hpp>
#include <iii_drone_interfaces/msg/target.hpp>
#include <iii_drone_interfaces/msg/combined_drone_awareness.hpp>
#include <iii_drone_interfaces/msg/state.hpp>

#include <iii_drone_interfaces/srv/register_offboard_mode.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/types.hpp>
#include <iii_drone_core/utils/math.hpp>

#include <iii_drone_core/utils/atomic.hpp>
#include <iii_drone_core/utils/history.hpp>

#include <iii_drone_core/configuration/parameter_bundle.hpp>

#include <iii_drone_core/adapters/px4/vehicle_status_adapter.hpp>
#include <iii_drone_core/adapters/px4/vehicle_odometry_adapter.hpp>
#include <iii_drone_core/adapters/powerline_adapter.hpp>
#include <iii_drone_core/adapters/single_line_adapter.hpp>
#include <iii_drone_core/adapters/gripper_status_adapter.hpp>
#include <iii_drone_core/adapters/target_adapter.hpp>
#include <iii_drone_core/adapters/state_adapter.hpp>

#include <iii_drone_core/control/state.hpp>
#include <iii_drone_core/control/reference.hpp>

#include <iii_drone_core/control/maneuver/maneuver_types.hpp>

/*****************************************************************************/
// Defines
/*****************************************************************************/

namespace iii_drone {
namespace control {

    /**
     * @brief Where the drone is located in a high-level sense.
     */
    typedef enum {
        DRONE_LOCATION_UNKNOWN = 0,
        DRONE_LOCATION_ON_GROUND = 1,
        DRONE_LOCATION_IN_FLIGHT = 2,
        DRONE_LOCATION_ON_CABLE = 3
    } drone_location_t;

	/**
	 * @brief Represents a combination of drone awareness information, on which behavior execution depends.
	 */
    typedef struct {
        iii_drone::control::State state;
        bool armed;
        bool offboard;
        iii_drone::adapters::TargetAdapter target_adapter;
        bool target_position_known;
        drone_location_t drone_location;
        int on_cable_id;
        double ground_altitude_estimate;
        bool gripper_open;

        bool has_target() const {
            return target_adapter.target_type() != iii_drone::adapters::TARGET_TYPE_NONE;
        }

        bool on_ground() const {
            return drone_location == DRONE_LOCATION_ON_GROUND;
        }

        bool on_cable() const {
            return drone_location == DRONE_LOCATION_ON_CABLE;
        }

        bool in_flight() const {
            return drone_location == DRONE_LOCATION_IN_FLIGHT;
        }

    } combined_drone_awareness_t;

} // namespace control
} // namespace iii_drone

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {

namespace control {

    /**
     * @brief Class which subscribes to various topics related to the drone awareness and keeps track of the current combined awareness.
     * Does the following:
     * - Subscribes to the PX4 vehicle status and odometry topics, the powerline topic, the gripper status topic, and the target cable id topic.
     * - Keeps track of the history of the received messages.
     * - Updates the combined drone awareness based on the received messages.
     * - Keeps and updates an estimate of the ground altitude and publishes the ground frame to tf2.
     * To be used from a ROS2 node. 
     * Construction of this object will create subscriptions and is not thread-safe.
     */
    class CombinedDroneAwarenessHandler {
        using VehicleStatusAdapterHistory = iii_drone::utils::History<iii_drone::adapters::px4::VehicleStatusAdapter>;
        using VehicleOdometryAdapterHistory = iii_drone::utils::History<iii_drone::adapters::px4::VehicleOdometryAdapter>;
        using PowerlineAdapterHistory = iii_drone::utils::History<iii_drone::adapters::PowerlineAdapter>;
        using GripperStatusAdapterHistory = iii_drone::utils::History<iii_drone::adapters::GripperStatusAdapter>;

        using AtomicIntVector = iii_drone::utils::Atomic<std::vector<int>>;

    public:

        /**
         * @brief Construct a new CombinedDroneAwarenessHandler object, initializing all the subscriptions.
         * 
         * @param params Shared pointer to the parameters for the combined drone awareness handler object.
         * @param tf_buffer Shared pointer to the tf2 buffer.
         * @param node Simple pointer to the containing node.
         * @param debug Whether to print debug messages.
         */
        CombinedDroneAwarenessHandler(
            iii_drone::configuration::ParameterBundle::SharedPtr params,
            tf2_ros::Buffer::SharedPtr tf_buffer,
            rclcpp::Node * node,
            bool debug = false
        );

        /**
         * @brief Get the current drone state.
         * 
         * @return The current drone state.
         */
        iii_drone::control::State GetState() const;

        /**
         * @brief Computes the target state of the drone given a target adapter.
         * 
         * @param target_adapter The target adapter.
         * 
         * @return The target state.
         */
        iii_drone::control::State ComputeTargetState(const iii_drone::adapters::TargetAdapter & target_adapter) const;

        /**
         * @brief Computes the target transform world to drone given a target adapter.
         * 
         * @param target_adapter The target adapter.
         * 
         * @return The target transform.
         */
        iii_drone::types::transform_matrix_t ComputeTargetTransform(const iii_drone::adapters::TargetAdapter & target_adapter) const;

        /**
         * @brief Gets the pose of the target.
         * 
         * @param target_adapter The target adapter.
         * 
         * @return The pose of the target.
         */
        iii_drone::types::pose_t GetPoseOfTarget(const iii_drone::adapters::TargetAdapter & target_adapter) const;

        /**
         * @brief Sets the current target.
         * 
         * @param target_adapter The target adapter.
         * 
         * @return void
         */
        void SetTarget(iii_drone::adapters::TargetAdapter target_adapter);

        /**
         * @brief Clears the current target.
         * 
         * @return void
         */
        void ClearTarget();

        /**
         * @brief Getter for the combined drone awareness simple type by copy.
         * 
         * @return The combined drone awareness member.
         */
        const combined_drone_awareness_t combined_drone_awareness() const;

        /**
         * @brief Whether the drone is armed.
         * 
         * @return true if the drone is armed.
         */
        bool armed() const;

        /**
         * @brief Whether the drone is in offboard mode.
         * 
         * @return true if the drone is in offboard mode.
         */
        bool offboard() const;

        /**
         * @brief Whether the drone has a target.
         * 
         * @return true if the drone has a target.
         */
        bool has_target() const;

        /**
         * @brief Whether the position of the target is known.
         * 
         * @return true if the position of the target is known.
         */
        bool target_position_known() const;

        /**
         * @brief Returns the current target adapter, will have TARGET_TYPE_NONE if no target.
         * 
         * @return The current target adapter.
         */
        iii_drone::adapters::TargetAdapter target_adapter() const;

        /**
         * @brief Whether the drone is on the ground.
         * 
         * @return true if the drone is on the ground.
         */
        bool on_ground() const;

        /**
         * @brief Whether the drone is on a cable.
         * 
         * @return true if the drone is on a cable.
         */
        bool on_cable() const;

        /**
         * @brief Whether the drone is in flight.
         * 
         * @return true if the drone is in flight.
         */
        bool in_flight() const;

        /**
         * @brief Return the id of the cable that the drone is on if it is on a cable, -1 otherwise.
         * This id can be different from the target cable id.
         * 
         * @return The id of the cable that the drone is on if it is on a cable, -1 otherwise.
         */
        int on_cable_id() const;

        /**
         * @brief Returns the ground altitude estimate.
         * 
         * @return The ground altitude estimate.
         */
        double ground_altitude_estimate() const;

        /**
         * @brief Returns the drone location, checks the drone location and returns the on_cable_id if the drone is on a cable.
         * 
         * @param on_cable_id The id of the cable that the drone is on if it is on a cable, -1 otherwise.
         * 
         * @return The drone location.
         */
        drone_location_t drone_location(int &on_cable_id) const;

        /**
         * @brief Returns the drone location.
         * 
         * @return The drone location.
         */
        drone_location_t drone_location() const;

        /**
         * @brief Returns the tf buffer shared ptr.
         * 
         * @return The tf buffer shared ptr.
         */
        tf2_ros::Buffer::SharedPtr tf_buffer() const;

        /**
         * @brief Shared pointer type for the CombinedDroneAwarenessHandler.
         */
        typedef std::shared_ptr<CombinedDroneAwarenessHandler> SharedPtr;

    private:
        /**
         * @brief Whether to print debug messages.
         */
        bool debug_;

        /**
         * @brief Parameters for the combined drone awareness handler.
         */
        iii_drone::configuration::ParameterBundle::SharedPtr params_;

        /**
         * @brief The tf2 buffer.
         */
        tf2_ros::Buffer::SharedPtr tf_buffer_;

        /**
         * @brief The tf2 broadcaster fro publishing ground frame.
         */
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        /**
         * @brief Simple pointer to the containing node.
         */
        rclcpp::Node * node_;

        /**
         * @brief Atomic vector of navigation state ids which are considered to be offboard.
         */
        AtomicIntVector offboard_nav_state_ids_;

        /**
         * @brief Register a new navigation state id which should be considered offboard.
         * 
         * @param navigation_state_id The navigation state id to be considered offboard.
         * 
         * @return void
         */
        void registerOffboardMode(int navigation_state_id);

        /**
         * @brief De-register a navigation state id which should no longer be considered offboard.
         * 
         * @param navigation_state_id The navigation state id to no longer be considered offboard.
         * 
         * @return void
         */
        void deregisterOffboardMode(int navigation_state_id);

        /**
         * @brief Register offboard mode service.
         */
        rclcpp::Service<iii_drone_interfaces::srv::RegisterOffboardMode>::SharedPtr register_offboard_mode_srv_;

        /**
         * @brief Atomic combined drone awareness simple type member.
         */
        iii_drone::utils::Atomic<combined_drone_awareness_t>::SharedPtr combined_drone_awareness_;

        /**
         * @brief Updates the combined drone awareness based on current information.
         * To be called after updating the any internal awareness information.
         * 
         * @return void
         */
        void updateCombinedDroneAwareness();

        /**
         * @brief Combined drone awareness publisher timer.
         */
        rclcpp::TimerBase::SharedPtr combined_drone_awareness_pub_timer_;

        /**
         * @brief Combined drone awareness publisher.
         */
        rclcpp::Publisher<iii_drone_interfaces::msg::CombinedDroneAwareness>::SharedPtr combined_drone_awareness_pub_;

        /**
         * @brief Has found initial location flag.
         */
        iii_drone::utils::Atomic<bool> has_found_initial_location_ = false;

		/**
		 * @brief PX4 vehicle status subscription
		 */
		rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;

        /**
         * @brief Vehicle status adapter history.
        */
        VehicleStatusAdapterHistory::SharedPtr vehicle_status_adapter_history_;

        /**
         * @brief Updates the combined drone awareness from the vehicle status.
         * Triggers updates of armed, offboard, and location.
         * 
         * @return void
         */
        void updateCombinedDroneAwarenessFromVehicleStatus();

        /**
         * @brief Updates the given combined drone awareness from the vehicle status.
         * Triggers updates of armed, offboard, and location.
         * 
         * @param combined_drone_awareness The combined drone awareness to update.
         * 
         * @return void
         */
        void updateCombinedDroneAwarenessFromVehicleStatus(combined_drone_awareness_t & combined_drone_awareness);

		/**
		 * @brief PX4 odometry subscription
		 */
		rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;

        /**
         * @brief Vehicle odometry adapter history.
        */
        VehicleOdometryAdapterHistory::SharedPtr vehicle_odometry_adapter_history_;

        /**
         * @brief Updates the combined drone awareness from the vehicle odometry.
         * Triggers updates of ground altitude estimate and location.
         * 
         * @return void
         */
        void updateCombinedDroneAwarenessFromVehicleOdometry();

        /**
         * @brief Updates the given combined drone awareness from the vehicle odometry.
         * Triggers updates of ground altitude estimate and location.
         * 
         * @param combined_drone_awareness The combined drone awareness to update.
         * 
         * @return void
         */
        void updateCombinedDroneAwarenessFromVehicleOdometry(combined_drone_awareness_t & combined_drone_awareness);

		/**
		 * @brief Powerline subscription
		 */
		rclcpp::Subscription<iii_drone_interfaces::msg::Powerline>::SharedPtr powerline_sub_;

        /**
         * @brief Powerline adapter history.
        */
        PowerlineAdapterHistory::SharedPtr powerline_adapter_history_;

        /**
         * @brief Updates the combined drone awareness from the powerline.
         * Triggers updates of target_cable_position_known and location.
         * 
         * @return void
         */
        void updateCombinedDroneAwarenessFromPowerline();

        /**
         * @brief Updates the given combined drone awareness from the powerline.
         * Triggers updates of target_cable_position_known and location.
         * 
         * @param combined_drone_awareness The combined drone awareness to update.
         * 
         * @return void
         */
        void updateCombinedDroneAwarenessFromPowerline(combined_drone_awareness_t & combined_drone_awareness);

		/**
		 * @brief Gripper status subscription
		 */
		rclcpp::Subscription<iii_drone_interfaces::msg::GripperStatus>::SharedPtr gripper_status_sub_;

        /**
         * @brief Gripper status adapter history.
        */
        GripperStatusAdapterHistory::SharedPtr gripper_status_adapter_history_;

        /**
         * @brief Updates the combined drone awareness from the gripper status.
         * Triggers updates of gripper_open.
         * 
         * @return void
         */
        void updateCombinedDroneAwarenessFromGripperStatus();

        /**
         * @brief Updates the given combined drone awareness from the gripper status.
         * Triggers updates of gripper_open.
         * 
         * @param combined_drone_awareness The combined drone awareness to update.
         * 
         * @return void
         */
        void updateCombinedDroneAwarenessFromGripperStatus(combined_drone_awareness_t & combined_drone_awareness);

        /**
         * @brief Atomic target adapter member.
         */
        iii_drone::utils::Atomic<iii_drone::adapters::TargetAdapter>::SharedPtr target_adapter_;

        /**
         * @brief Target publisher.
         */
        rclcpp::Publisher<iii_drone_interfaces::msg::Target>::SharedPtr target_pub_;

        /**
         * @brief Updates the combined drone awareness from the target adapter.
         * Triggers updates of target_adapter, target_position_known, and has_target.
         * 
         * @return void
         */
        void updateCombinedDroneAwarenessFromTarget();

        /**
         * @brief Updates the given combined drone awareness from the target adapter.
         * Triggers updates of target_adapter, target_position_known, and has_target.
         * 
         * @param combined_drone_awareness The combined drone awareness to update.
         * 
         * @return void
         */
        void updateCombinedDroneAwarenessFromTarget(combined_drone_awareness_t & combined_drone_awareness);

        /**
         * @brief Atomic ground altitude estimate member.
         */
        iii_drone::utils::Atomic<double>::SharedPtr ground_altitude_estimate_;

        /**
         * @brief Ground altitudes history.
         */
        iii_drone::utils::History<double>::SharedPtr ground_altitudes_history_;

        /**
         * @brief Timer for updating the ground altitude estimate.
         */
        rclcpp::TimerBase::SharedPtr ground_altitude_update_timer_;

        /**
         * @brief Updates the ground altitude estimate based on given information.
         * Will either start or stop the timer based on whether the drone is on the ground.
         * If the drone is on the ground, and the timer is not running (has elapsed), will start the timer and update the ground altitude estimate.
         * If the drone is on the ground, and the timer is running, will do nothing.
         * If the drone is not on the ground, and the timer is running, will stop the timer.
         * To be called after updates to vehicle odometry, vehicle status, or target cable.
         * Will set the ground altitude estimate to the current altitude if the drone is on the ground, based on the following criteria:
         * - The drone is unarmed; and
         * - No target cable is registered.
         * The ground altitude estimate is used to determine the drone location (on ground, in flight, on cable).
         * Updates the ground altitude estimate by pushing the current altitude to the history and taking the mean of the history.
         * 
         * @param armed Whether the drone is armed.
         * @param offboard Whether the drone is in offboard mode.
         * @param has_target_cable Whether the drone has a target cable.
         * 
         * @return void
         */
        void updateGroundAltitudeEstimate(
            bool armed,
            bool offboard,
            bool has_target_cable
        );

        /**
         * @brief Updates the drone location of a combined drone awareness object based on current awareness information.
         * Additionally updates the on_cable_id member by evaluating whether the drone is currently on a cable. 
         * on_cable_id will be set to -1 if the drone is not on a cable.
         * on_cable_id can be different from target_cable_id.
         * 
         * @param awareness The combined drone awareness object to update.
         * 
         * @return void
         */
        void updateDroneLocation(combined_drone_awareness_t & awareness);

        /**
         * @brief Timer for publishing members.
         */
        rclcpp::TimerBase::SharedPtr publish_timer_;

        /**
         * @brief Publishes the members.
         * 
         * @return void
         */
        void publishMembers();

    };

} // namespace control
} // namespace iii_drone

