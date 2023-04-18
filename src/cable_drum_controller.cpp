/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "cable_drum_controller.h"

/*****************************************************************************/
// Implementation
/*****************************************************************************/

CableDrumController::CableDrumController(const std::string & node_name, 
            const std::string & node_namespace, const rclcpp::NodeOptions & options) :
        Node(node_name, node_namespace, options) {

	this->declare_parameter<uint8_t>("cable_drum_initial_gain", 15);
	this->declare_parameter<uint8_t>("cable_drum_initial_reference", 0b1000);

    // FPGA init:
    int status;
    status = XCabledrumbridge_Initialize(&cdb_, "CableDrumBridge");

    if (status == XST_DEVICE_NOT_FOUND) {

        RCLCPP_FATAL(this->get_logger(), "CableDrumBridge device not found!");

    } else if (status == XST_OPEN_DEVICE_FAILED) {

        RCLCPP_FATAL(this->get_logger(), "CableDrumBridge open device failed!");

    } else if (status == XST_SUCCESS) {

        RCLCPP_INFO(this->get_logger(), "CableDrumBridge device opened successfully");

    } else {

        RCLCPP_FATAL(this->get_logger(), "CableDrumBridge unknown error while initializing");

    }

    mode_ = PARAM_MODE_OFF;
    direction_ = PARAM_DIR_OUT;
    duty_cycle_ = 0;
    this->get_parameter("cable_drum_initial_reference", reference_);
    this->get_parameter("cable_drum_initial_gain", gain_);

    XCabledrumbridge_Set_mode_CPU(&cdb_, mode_);
    XCabledrumbridge_Set_duty_cycle_CPU(&cdb_, duty_cycle_);
    XCabledrumbridge_Set_man_dir_CPU(&cdb_, direction_);
    XCabledrumbridge_Set_ref_flex_CPU(&cdb_, reference_);
    XCabledrumbridge_Set_gain_CPU(&cdb_, gain_);

	// DrumManualRoll action:
	this->drum_manual_roll_server_ = rclcpp_action::create_server<DrumManualRoll>(
		this,
		"drum_manual_roll",
		std::bind(&CableDrumController::handleGoalDrumManualRoll, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&CableDrumController::handleCancelDrumManualRoll, this, std::placeholders::_1),
		std::bind(&CableDrumController::handleAcceptedDrumManualRoll, this, std::placeholders::_1)
	);

    // Services:
    set_gain_service_ = this->create_service<iii_interfaces::srv::DrumSetGain>("drum_set_gain", 
        std::bind(&CableDrumController::drumSetGainServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
    set_mode_service_ = this->create_service<iii_interfaces::srv::DrumSetMode>("drum_set_mode", 
        std::bind(&CableDrumController::drumSetModeServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
    set_reference_service_ = this->create_service<iii_interfaces::srv::DrumSetReference>("drum_set_reference", 
        std::bind(&CableDrumController::drumSetReferenceServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Publisher:
    cable_drum_info_pub_ = this->create_publisher<iii_interfaces::msg::CableDrumInfo>("cable_drum_info", 10);

    // Timer:
	info_publish_timer_ = this->create_wall_timer(
		100ms, std::bind(&CableDrumController::infoPublishTimerCallback, this));

}

CableDrumController::~CableDrumController() {

    XCabledrumbridge_Set_mode_CPU(&cdb_, 0);

    XCabledrumbridge_Release(&cdb_);

}

rclcpp_action::GoalResponse CableDrumController::handleGoalDrumManualRoll(
    const rclcpp_action::GoalUUID & uuid, 
    std::shared_ptr<const DrumManualRoll::Goal> goal
) {

	(void)uuid;

    uint8_t mode;

    cable_drum_info_mutex_.lock(); {
        
        mode = mode_;

    } cable_drum_info_mutex_.unlock();

    if (mode != PARAM_MODE_MAN)
        return rclcpp_action::GoalResponse::REJECT;

    target_duty_cycle_ = goal->duty_cycle;
    target_direction_ = goal->direction == goal->DIRECTION_IN ? PARAM_DIR_IN : PARAM_DIR_OUT;
    target_seconds_ = goal->seconds;

	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

}

rclcpp_action::CancelResponse CableDrumController::handleCancelDrumManualRoll(const std::shared_ptr<GoalHandleDrumManualRoll> goal_handle) {

	return rclcpp_action::CancelResponse::ACCEPT;

}

void CableDrumController::handleAcceptedDrumManualRoll(const std::shared_ptr<GoalHandleDrumManualRoll> goal_handle) {

	using namespace std::placeholders;

	std::thread{ std::bind(&CableDrumController::followDrumManualRollCompletion, this, _1), goal_handle}.detach();

}

void CableDrumController::followDrumManualRollCompletion(const std::shared_ptr<GoalHandleDrumManualRoll> goal_handle) {

	auto feedback = std::make_shared<DrumManualRoll::Feedback>();

	auto result = std::make_shared<DrumManualRoll::Result>();

    if(drum_manual_roll_mutex_.try_lock()) {

        rclcpp::Rate rate(100ms);

        cable_drum_info_mutex_.lock(); {

            direction_ = target_direction_;
            duty_cycle_ = target_duty_cycle_;

            fpga_mutex_.lock(); {

                XCabledrumbridge_Set_man_dir_CPU(&cdb_, direction_);
                XCabledrumbridge_Set_duty_cycle_CPU(&cdb_, duty_cycle_);


            } fpga_mutex_.unlock();

        } cable_drum_info_mutex_.unlock();

        for (float t = 0.; t < target_seconds_; t+=0.1) {

            if (goal_handle->is_canceling()) {

                fpga_mutex_.lock(); {

                    XCabledrumbridge_Set_duty_cycle_CPU(&cdb_, 0);

                } fpga_mutex_.unlock();

                cable_drum_info_mutex_.lock(); {

                    duty_cycle_ = 0;

                } cable_drum_info_mutex_.unlock();

                result->success = false;
                goal_handle->canceled(result);

                drum_manual_roll_mutex_.unlock();

                return;

            }

            uint8_t sensor_flex;
            uint8_t mode;

            cable_drum_info_mutex_.lock(); {

                sensor_flex = sensor_flex_;
                mode = mode_;

            } cable_drum_info_mutex_.unlock();

            if (mode != PARAM_MODE_MAN) {

                fpga_mutex_.lock(); {

                    XCabledrumbridge_Set_duty_cycle_CPU(&cdb_, 0);

                } fpga_mutex_.unlock();

                cable_drum_info_mutex_.lock(); {

                    duty_cycle_ = 0;

                } cable_drum_info_mutex_.unlock();

                result->success = false;
                goal_handle->abort(result);

                drum_manual_roll_mutex_.unlock();

                return;

            }

            feedback->sensor_flex = sensor_flex;
            feedback->seconds_elapsed = t;
            
            goal_handle->publish_feedback(feedback);

            rate.sleep();

        }

        fpga_mutex_.lock(); {

            XCabledrumbridge_Set_duty_cycle_CPU(&cdb_, 0);

        } fpga_mutex_.unlock();

        cable_drum_info_mutex_.lock(); {

            duty_cycle_ = 0;

        } cable_drum_info_mutex_.unlock();

        result->success = true;
        goal_handle->succeed(result);

        drum_manual_roll_mutex_.unlock();

    } else {

        result->success = false;
        goal_handle->abort(result);

    }
}

void CableDrumController::drumSetGainServiceCallback(const std::shared_ptr<iii_interfaces::srv::DrumSetGain::Request> request,
                                std::shared_ptr<iii_interfaces::srv::DrumSetGain::Response> response) {

    fpga_mutex_.lock(); {

        XCabledrumbridge_Set_gain_CPU(&cdb_, request->gain);

    } fpga_mutex_.unlock();

    cable_drum_info_mutex_.lock(); {

        gain_ = request->gain;

    } cable_drum_info_mutex_.unlock();

    response->success = true;

}

void CableDrumController::drumSetModeServiceCallback(const std::shared_ptr<iii_interfaces::srv::DrumSetMode::Request> request,
                                std::shared_ptr<iii_interfaces::srv::DrumSetMode::Response> response) {

    uint8_t fpga_mode;
    switch(request->mode) {
    default:
    case iii_interfaces::srv::DrumSetMode::Request::MODE_OFF:
        fpga_mode = PARAM_MODE_OFF;
        break;
    case iii_interfaces::srv::DrumSetMode::Request::MODE_REF_TRACK:
        fpga_mode = PARAM_MODE_REF_TRACK;
        break;
    case iii_interfaces::srv::DrumSetMode::Request::MODE_MANUAL:
        fpga_mode = PARAM_MODE_MAN;
        break;
    }

    fpga_mutex_.lock(); {

        XCabledrumbridge_Set_mode_CPU(&cdb_, fpga_mode);

    } fpga_mutex_.unlock();

    cable_drum_info_mutex_.lock(); {

        mode_ = fpga_mode;

    } cable_drum_info_mutex_.unlock();

    response->success = true;

}

void CableDrumController::drumSetReferenceServiceCallback(const std::shared_ptr<iii_interfaces::srv::DrumSetReference::Request> request,
                                std::shared_ptr<iii_interfaces::srv::DrumSetReference::Response> response) {

    fpga_mutex_.lock(); {

        XCabledrumbridge_Set_ref_flex_CPU(&cdb_, request->reference);

    } fpga_mutex_.unlock();

    cable_drum_info_mutex_.lock(); {

        reference_ = request->reference;

    } cable_drum_info_mutex_.unlock();

    response->success = true;

}

void CableDrumController::infoPublishTimerCallback() {

    bool direction;
    uint8_t duty_cycle;
    uint8_t gain;
    uint8_t mode;
    uint8_t reference;
    uint8_t sensor_flex;

    fpga_mutex_.lock(); {

        sensor_flex = XCabledrumbridge_Get_sensor_flex_CPU(&cdb_);

    } fpga_mutex_.unlock();

    cable_drum_info_mutex_.lock(); {

        direction = direction_ == PARAM_DIR_IN ? iii_interfaces::msg::CableDrumInfo::DIRECTION_IN : 
                                        iii_interfaces::msg::CableDrumInfo::DIRECTION_OUT;
        duty_cycle = duty_cycle_;
        gain = gain_;
        mode = mode_;
        reference = reference_;

        sensor_flex_ = sensor_flex;

    } cable_drum_info_mutex_.unlock();

    iii_interfaces::msg::CableDrumInfo msg;

    msg.direction = direction;
    msg.duty_cycle = duty_cycle;
    msg.gain = gain;
    switch(mode) {
    default:
    case PARAM_MODE_OFF:
        msg.mode = msg.MODE_OFF;
        break;
    case PARAM_MODE_MAN:
        msg.mode = msg.MODE_MANUAL;
        break;
    case PARAM_MODE_REF_TRACK:
        msg.mode = msg.MODE_REF_TRACK;
        break;
    }
    msg.reference = reference;
    msg.sensor_flex = sensor_flex;

    cable_drum_info_pub_->publish(msg);

}

int main(int argc, char* argv[]) {
	 std::cout << "Starting cable drum controller node..." << std::endl;

	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<CableDrumController>());

	rclcpp::shutdown();
	return 0;
}