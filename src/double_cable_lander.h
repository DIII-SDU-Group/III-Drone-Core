#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <stdint.h>
#include <mutex>
#include <functional>
#include <memory>
#include <thread>
#include <climits>
#include <math.h>
#include <chrono>
#include <iostream>
#include <queue>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_msgs/msg/string.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "iii_interfaces/srv/drum_set_gain.hpp"
#include "iii_interfaces/srv/drum_set_mode.hpp"
#include "iii_interfaces/srv/drum_set_reference.hpp"
#include "iii_interfaces/srv/set_general_target_yaw.hpp"

#include "iii_interfaces/msg/cable_drum_info.hpp"
#include "iii_interfaces/msg/control_state.hpp"
#include "iii_interfaces/msg/powerline.hpp"

#include "iii_interfaces/action/double_cable_landing.hpp"

#include "iii_interfaces/action/drum_manual_roll.hpp"

#include "iii_interfaces/action/fly_to_position.hpp"
#include "iii_interfaces/action/cable_landing.hpp"
#include "iii_interfaces/action/cable_takeoff.hpp"

#include "geometry.h"

/*****************************************************************************/
// Defines
/*****************************************************************************/

#define ROS_DEFAULT_API

using namespace std::chrono_literals;

using namespace std::placeholders;

/*****************************************************************************/
// Class
/*****************************************************************************/

class DoubleCableLander : public rclcpp::Node {
public:
    using DoubleCableLanding = iii_interfaces::action::DoubleCableLanding;
    using GoalHandleDoubleCableLanding = rclcpp_action::ServerGoalHandle<DoubleCableLanding>;

	using FlyToPosition = iii_interfaces::action::FlyToPosition;
	using GoalHandleFlyToPosition = rclcpp_action::ClientGoalHandle<FlyToPosition>;

	using CableLanding = iii_interfaces::action::CableLanding;
	using GoalHandleCableLanding = rclcpp_action::ClientGoalHandle<CableLanding>;

	using CableTakeoff = iii_interfaces::action::CableTakeoff;
	using GoalHandleCableTakeoff = rclcpp_action::ClientGoalHandle<CableTakeoff>;

	using DrumManualRoll = iii_interfaces::action::DrumManualRoll;
	using GoalHandleDrumManualRoll = rclcpp_action::ClientGoalHandle<DrumManualRoll>;

	DoubleCableLander(const std::string & node_name="double_cable_lander", 
			const std::string & node_namespace="/double_cable_lander", 
			const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
	~DoubleCableLander();

private:
    // Double cable landing action server:
	rclcpp_action::Server<DoubleCableLanding>::SharedPtr double_cable_landing_server_;

	rclcpp_action::GoalResponse handleGoalDoubleCableLanding(
		const rclcpp_action::GoalUUID & uuid, 
		std::shared_ptr<const DoubleCableLanding::Goal> goal
	);
	rclcpp_action::CancelResponse handleCancelDoubleCableLanding(const std::shared_ptr<GoalHandleDoubleCableLanding> goal_handle);
	void handleAcceptedDoubleCableLanding(const std::shared_ptr<GoalHandleDoubleCableLanding> goal_handle);
	void followDoubleCableLandingCompletion(const std::shared_ptr<GoalHandleDoubleCableLanding> goal_handle);

    // Fly to position action client:
    rclcpp_action::Client<FlyToPosition>::SharedPtr fly_to_position_client_;

    // Cable landing action client:
    rclcpp_action::Client<CableLanding>::SharedPtr cable_landing_client_;

    // Cable takeoff action client:
    rclcpp_action::Client<CableTakeoff>::SharedPtr cable_takeoff_client_;

    // Drum manual roll client:
    rclcpp_action::Client<DrumManualRoll>::SharedPtr drum_manual_roll_client_;

    // Service clients:
    rclcpp::Client<iii_interfaces::srv::DrumSetGain>::SharedPtr drum_set_gain_client_;
    rclcpp::Client<iii_interfaces::srv::DrumSetMode>::SharedPtr drum_set_mode_client_;
    rclcpp::Client<iii_interfaces::srv::DrumSetReference>::SharedPtr drum_set_reference_client_;
    rclcpp::Client<iii_interfaces::srv::SetGeneralTargetYaw>::SharedPtr target_yaw_client_;

    // Subsciptions:
	rclcpp::Subscription<iii_interfaces::msg::Powerline>::SharedPtr powerline_sub_;

    // Publishers:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;

    // tf:
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    // Powerline:
    iii_interfaces::msg::Powerline powerline_;
    iii_interfaces::msg::Powerline getPowerline();
    void setPowerline(iii_interfaces::msg::Powerline powerline);

    std::mutex powerline_mutex_;

    quat_t target_quat_;

    void powerlineCallback(iii_interfaces::msg::Powerline::SharedPtr msg);

    // State:
    enum state_t {
        idle,
        fly_under_first_cable,
        land_on_first_cable,
        leave_first_cable,
        fly_under_second_cable,
        land_on_second_cable,
        on_second_cable
    };

    state_t state_ = idle;

    // Aux
    int first_cable_id_;
    int second_cable_id_;

    int trajectory_goal_response_ = 0;
    int cable_drum_goal_response_ = 0;

    int trajectory_goal_result_ = 0;
    int cable_drum_goal_result_ = 0;

    void flyToPositionGoalResponseCallback(std::shared_future<GoalHandleFlyToPosition::SharedPtr> future);
    void flyToPositionFeedbackCallback(GoalHandleFlyToPosition::SharedPtr, const std::shared_ptr<const FlyToPosition::Feedback> feedback);
    void flyToPositionResultCallback(const GoalHandleFlyToPosition::WrappedResult &result);

    void cableLandingGoalResponseCallback(std::shared_future<GoalHandleCableLanding::SharedPtr> future);
    void cableLandingFeedbackCallback(GoalHandleCableLanding::SharedPtr, const std::shared_ptr<const CableLanding::Feedback> feedback);
    void cableLandingResultCallback(const GoalHandleCableLanding::WrappedResult &result);

    void cableTakeoffGoalResponseCallback(std::shared_future<GoalHandleCableTakeoff::SharedPtr> future);
    void cableTakeoffFeedbackCallback(GoalHandleCableTakeoff::SharedPtr, const std::shared_ptr<const CableTakeoff::Feedback> feedback);
    void cableTakeoffResultCallback(const GoalHandleCableTakeoff::WrappedResult &result);

    void drumManualRollGoalResponseCallback(std::shared_future<GoalHandleDrumManualRoll::SharedPtr> future);
    void drumManualRollFeedbackCallback(GoalHandleDrumManualRoll::SharedPtr, const std::shared_ptr<const DrumManualRoll::Feedback> feedback);
    void drumManualRollResultCallback(const GoalHandleDrumManualRoll::WrappedResult &result);

    // Timed
	rclcpp::TimerBase::SharedPtr publish_timer_;
    void publishCallback();

};

/*****************************************************************************/
// Main
/*****************************************************************************/

int main(int argc, char* argv[]);