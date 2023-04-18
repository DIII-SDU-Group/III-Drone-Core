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

#include <std_msgs/msg/u_int8.hpp>

#include "iii_interfaces/action/drum_manual_roll.hpp"
#include "iii_interfaces/srv/drum_set_gain.hpp"
#include "iii_interfaces/srv/drum_set_mode.hpp"
#include "iii_interfaces/srv/drum_set_reference.hpp"
#include "iii_interfaces/msg/cable_drum_info.hpp"

#include "xcabledrumbridge.h"

using namespace std::chrono_literals;

/*****************************************************************************/
// Defines
/*****************************************************************************/

#define PARAM_DIR_IN 0b1
#define PARAM_DIR_OUT 0b0

#define PARAM_MODE_OFF 0b00
#define PARAM_MODE_REF_TRACK 0b01
#define PARAM_MODE_MAN 0b11

/*****************************************************************************/
// Class
/*****************************************************************************/

class CableDrumController : public rclcpp::Node {
public:
	using DrumManualRoll = iii_interfaces::action::DrumManualRoll;
	using GoalHandleDrumManualRoll = rclcpp_action::ServerGoalHandle<DrumManualRoll>;

	CableDrumController(const std::string & node_name="cable_drum_controller", 
			const std::string & node_namespace="/cable_drum_controller", 
			const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
	~CableDrumController();

private:
    // DrumManualRoll action:
	rclcpp_action::Server<DrumManualRoll>::SharedPtr drum_manual_roll_server_;

	rclcpp_action::GoalResponse handleGoalDrumManualRoll(
		const rclcpp_action::GoalUUID & uuid, 
		std::shared_ptr<const DrumManualRoll::Goal> goal
	);
	rclcpp_action::CancelResponse handleCancelDrumManualRoll(const std::shared_ptr<GoalHandleDrumManualRoll> goal_handle);
	void handleAcceptedDrumManualRoll(const std::shared_ptr<GoalHandleDrumManualRoll> goal_handle);
	void followDrumManualRollCompletion(const std::shared_ptr<GoalHandleDrumManualRoll> goal_handle);

    std::mutex drum_manual_roll_mutex_;

    // Services:
    rclcpp::Service<iii_interfaces::srv::DrumSetGain>::SharedPtr set_gain_service_;
    void drumSetGainServiceCallback(const std::shared_ptr<iii_interfaces::srv::DrumSetGain::Request> request,
                                    std::shared_ptr<iii_interfaces::srv::DrumSetGain::Response> response);

    rclcpp::Service<iii_interfaces::srv::DrumSetMode>::SharedPtr set_mode_service_;
    void drumSetModeServiceCallback(const std::shared_ptr<iii_interfaces::srv::DrumSetMode::Request> request,
                                    std::shared_ptr<iii_interfaces::srv::DrumSetMode::Response> response);

    rclcpp::Service<iii_interfaces::srv::DrumSetReference>::SharedPtr set_reference_service_;
    void drumSetReferenceServiceCallback(const std::shared_ptr<iii_interfaces::srv::DrumSetReference::Request> request,
                                    std::shared_ptr<iii_interfaces::srv::DrumSetReference::Response> response);

    // Publisher:
	rclcpp::Publisher<iii_interfaces::msg::CableDrumInfo>::SharedPtr cable_drum_info_pub_;

    // Sensor flex:
    uint8_t sensor_flex_;
    std::mutex sensor_flex_mutex_;

    // Cable drum info:
    std::mutex cable_drum_info_mutex_;

    uint8_t mode_;
    uint8_t direction_;
    uint8_t reference_;
    uint8_t duty_cycle_;
    uint8_t gain_;

    uint8_t target_direction_;
    uint8_t target_duty_cycle_;
    uint8_t target_seconds_;

    // FPGA:
    XCabledrumbridge cdb_;
    std::mutex fpga_mutex_;

    // Info publish timer:
	rclcpp::TimerBase::SharedPtr info_publish_timer_;
    void infoPublishTimerCallback();

};

/*****************************************************************************/
// Main
/*****************************************************************************/

int main(int argc, char* argv[]);