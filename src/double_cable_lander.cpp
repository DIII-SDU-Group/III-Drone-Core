/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "double_cable_lander.h"

/*****************************************************************************/
// Implementation
/*****************************************************************************/

DoubleCableLander::DoubleCableLander(const std::string & node_name, 
                const std::string & node_namespace, const rclcpp::NodeOptions & options) :
        Node(node_name, node_namespace, options) {

    // Parameters:
    this->declare_parameter<float>("under_cable_altitude_subtract", 1.);
    this->declare_parameter<float>("cable_takeoff_target_cable_distance", 1.);
    this->declare_parameter<uint8_t>("cable_drum_manual_duty_cycle", 50);
    this->declare_parameter<uint8_t>("cable_drum_manual_seconds", 5);
    this->declare_parameter<uint8_t>("cable_drum_tracking_gain", 15);
    this->declare_parameter<uint8_t>("cable_drum_tracking_reference", 7);

	// tf
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Publisher:
    state_pub_ = this->create_publisher<std_msgs::msg::String>("state", 10);

    // Subscription
	powerline_sub_ = this->create_subscription<iii_interfaces::msg::Powerline>(
		"/pl_mapper/powerline", 10,
		std::bind(&DoubleCableLander::powerlineCallback, this, std::placeholders::_1));

    // Action clients:
    this->fly_to_position_client_ = rclcpp_action::create_client<FlyToPosition>(
       this, "/trajectory_controller/fly_to_position"
    );
    this->cable_landing_client_ = rclcpp_action::create_client<CableLanding>(
       this, "/trajectory_controller/cable_landing"
    );
    this->cable_takeoff_client_ = rclcpp_action::create_client<CableTakeoff>(
       this, "/trajectory_controller/cable_takeoff"
    );
    this->drum_manual_roll_client_ = rclcpp_action::create_client<DrumManualRoll>(
       this, "/cable_drum_controller/drum_manual_roll"
    );

	// DoubleCableLanding action server:
	this->double_cable_landing_server_ = rclcpp_action::create_server<DoubleCableLanding>(
		this,
		"double_cable_landing",
		std::bind(&DoubleCableLander::handleGoalDoubleCableLanding, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&DoubleCableLander::handleCancelDoubleCableLanding, this, std::placeholders::_1),
		std::bind(&DoubleCableLander::handleAcceptedDoubleCableLanding, this, std::placeholders::_1)
	);


    // Service clients:
    this->drum_set_gain_client_ = this->create_client<iii_interfaces::srv::DrumSetGain>("/cable_drum_controller/drum_set_gain");
    this->drum_set_mode_client_ = this->create_client<iii_interfaces::srv::DrumSetMode>("/cable_drum_controller/drum_set_mode");
    this->drum_set_reference_client_ = this->create_client<iii_interfaces::srv::DrumSetReference>("/cable_drum_controller/drum_set_reference");

    // auto hej = this->create_client<iii_interfaces::srv::SetGeneralTargetYaw>("/trajectory_controller/set_general_target_yaw");

    // this->target_yaw_client_ = hej;

    std::cout << "hej\n";

	publish_timer_ = this->create_wall_timer(
		500ms, std::bind(&DoubleCableLander::publishCallback, this));

}

DoubleCableLander::~DoubleCableLander() {

}

rclcpp_action::GoalResponse DoubleCableLander::handleGoalDoubleCableLanding(
        const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const DoubleCableLanding::Goal> goal) {

    std::cout << "hej2\n";

    if (state_ != idle) {

        std::cout << "Reject, not idle\n";

        return rclcpp_action::GoalResponse::REJECT;

    }

    first_cable_id_ = goal->first_cable_id;
    second_cable_id_ = goal->second_cable_id;

    iii_interfaces::msg::Powerline powerline = getPowerline();

    bool first_id_found = false;
    bool second_id_found = false;

    for (int i = 0; i < powerline.count; i++) {

        if (powerline.ids[i] == first_cable_id_) {

            first_id_found = true;

        } else if (powerline.ids[i] == second_cable_id_) {

            second_id_found = true;

        }
    }

    if (!first_id_found || !second_id_found) {

        std::cout << "Reject, ids not found\n";

        return rclcpp_action::GoalResponse::REJECT;

    }

	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

}

rclcpp_action::CancelResponse DoubleCableLander::handleCancelDoubleCableLanding(const std::shared_ptr<GoalHandleDoubleCableLanding> goal_handle) {

    return rclcpp_action::CancelResponse::ACCEPT;

}

void DoubleCableLander::handleAcceptedDoubleCableLanding(const std::shared_ptr<GoalHandleDoubleCableLanding> goal_handle) {

    using namespace std::placeholders;

    std::thread{ std::bind(&DoubleCableLander::followDoubleCableLandingCompletion, this, _1), goal_handle}.detach();

}

void DoubleCableLander::followDoubleCableLandingCompletion(const std::shared_ptr<GoalHandleDoubleCableLanding> goal_handle) {

    std::cout << "starting\n";

    // this->target_yaw_client_ = this->create_client<iii_interfaces::srv::SetGeneralTargetYaw>("/trajectory_controller/set_general_target_yaw");

	auto feedback = std::make_shared<DoubleCableLanding::Feedback>();
	auto result = std::make_shared<DoubleCableLanding::Result>();

    iii_interfaces::msg::Powerline powerline;
    geometry_msgs::msg::PoseStamped under_cable_pose;

    bool exit = false;

    auto cancel_all_action_requests = [this]() -> void {

        this->fly_to_position_client_->async_cancel_all_goals();
        this->cable_landing_client_->async_cancel_all_goals();
        this->cable_takeoff_client_->async_cancel_all_goals();
        this->drum_manual_roll_client_->async_cancel_all_goals();

    };

    auto abort_goal = [&]() -> void {

        cancel_all_action_requests();

        result->success = false;

        goal_handle->abort(result);

        state_ = idle;

        exit = true;

    };

    auto compute_pose_under_cable = [this](iii_interfaces::msg::Powerline powerline, int id, geometry_msgs::msg::PoseStamped &pose) -> bool {

        geometry_msgs::msg::PoseStamped cable_pose;
        bool pose_found = false;

        for (int i = 0; i < powerline.count; i++) {

            if (powerline.ids[i] == id) {

                cable_pose = powerline.poses[i];

                pose_found = true;

                break;

            }
        }

        if (!pose_found) 
            return false;

        float under_cable_altitude_subtract;
        this->get_parameter("under_cable_altitude_subtract", under_cable_altitude_subtract);

        geometry_msgs::msg::PoseStamped tmp_pose;

        tmp_pose.header.frame_id = cable_pose.header.frame_id;
        tmp_pose.pose = cable_pose.pose;
        tmp_pose.pose.position.z -= under_cable_altitude_subtract;
        pose = tf_buffer_->transform(tmp_pose, "world");

        pose.pose.orientation.w = target_quat_(0);
        pose.pose.orientation.x = target_quat_(1);
        pose.pose.orientation.y = target_quat_(2);
        pose.pose.orientation.z = target_quat_(3);

        return true;

    };

    auto trajectory_goal_wait = [this]() -> bool {

        rclcpp::Rate rate(100ms);

        while(true) {

            if (trajectory_goal_response_ < 0)
                return false;

            if (trajectory_goal_response_ > 0 && trajectory_goal_result_ < 0)
                return false;

            if (trajectory_goal_response_ > 0 && trajectory_goal_result_ > 0)
                return true;

            rate.sleep();

        }
    };

    auto trajectory_and_drum_goal_wait = [this]() -> bool {

        rclcpp::Rate rate(100ms);

        while(true) {

            if (trajectory_goal_response_ < 0 || cable_drum_goal_response_ < 0)
                return false;

            if (trajectory_goal_response_ > 0 && trajectory_goal_result_ < 0 || cable_drum_goal_response_ > 0 && cable_drum_goal_result_ < 0)
                return false;

            if (trajectory_goal_response_ > 0 && trajectory_goal_result_ > 0 && cable_drum_goal_response_ > 0 && cable_drum_goal_result_ > 0)
                return true;

            rate.sleep();

        }
    };

    auto fly_to_under_cable_blocking = [this, trajectory_goal_wait](geometry_msgs::msg::PoseStamped pose) -> bool {

        std::cout << "c.1\n";

        if (!this->fly_to_position_client_->wait_for_action_server()) {

        std::cout << "c.2\n";

            return false;

        }

        std::cout << "c.3\n";

        trajectory_goal_response_ = 0;
        trajectory_goal_result_ = 0;

        auto goal_msg = FlyToPosition::Goal();

        goal_msg.target_pose = pose;

        auto send_goal_options = rclcpp_action::Client<FlyToPosition>::SendGoalOptions();

        send_goal_options.goal_response_callback = std::bind(&DoubleCableLander::flyToPositionGoalResponseCallback, this, _1);
        send_goal_options.feedback_callback = std::bind(&DoubleCableLander::flyToPositionFeedbackCallback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&DoubleCableLander::flyToPositionResultCallback, this, _1);

        this->fly_to_position_client_->async_send_goal(goal_msg, send_goal_options);

        std::cout << "c.4\n";

        return trajectory_goal_wait();

    };

    auto land_on_cable_blocking = [this, trajectory_goal_wait](int cable_id) -> bool {

        std::cout << "d.1\n";

        if (!this->cable_landing_client_->wait_for_action_server()) {

        std::cout << "d.2\n";

            return false;

        }

        std::cout << "d.3\n";

        trajectory_goal_response_ = 0;
        trajectory_goal_result_ = 0;

        auto goal_msg = CableLanding::Goal();

        std::cout << "d.4\n";

        goal_msg.target_cable_id = cable_id;

        auto send_goal_options = rclcpp_action::Client<CableLanding>::SendGoalOptions();

        std::cout << "d.5\n";

        send_goal_options.goal_response_callback = std::bind(&DoubleCableLander::cableLandingGoalResponseCallback, this, _1);
        send_goal_options.feedback_callback = std::bind(&DoubleCableLander::cableLandingFeedbackCallback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&DoubleCableLander::cableLandingResultCallback, this, _1);

        std::cout << "d.6\n";

        this->cable_landing_client_->async_send_goal(goal_msg, send_goal_options);

        std::cout << "d.7\n";

        return trajectory_goal_wait();

    };

    auto leave_cable_blocking = [this, trajectory_and_drum_goal_wait]() -> bool {

        if(!this->cable_takeoff_client_->wait_for_action_server()) {

            return false;

        }

        trajectory_goal_response_ = 0;
        trajectory_goal_result_ = 0;
        cable_drum_goal_response_ = 0;
        cable_drum_goal_result_ = 0;


        auto traj_goal_msg = CableTakeoff::Goal();

        float cable_takeoff_target_cable_distance;
        this->get_parameter("cable_takeoff_target_cable_distance", cable_takeoff_target_cable_distance);

        traj_goal_msg.target_cable_distance = cable_takeoff_target_cable_distance;

        auto traj_send_goal_options = rclcpp_action::Client<CableTakeoff>::SendGoalOptions();

        traj_send_goal_options.goal_response_callback = std::bind(&DoubleCableLander::cableTakeoffGoalResponseCallback, this, _1);
        traj_send_goal_options.feedback_callback = std::bind(&DoubleCableLander::cableTakeoffFeedbackCallback, this, _1, _2);
        traj_send_goal_options.result_callback = std::bind(&DoubleCableLander::cableTakeoffResultCallback, this, _1);



        auto cable_drum_goal_msg = DrumManualRoll::Goal();

        uint8_t cable_drum_manual_duty_cycle;
        this->get_parameter("cable_drum_manual_duty_cycle", cable_drum_manual_duty_cycle);

        uint8_t cable_drum_manual_seconds;
        this->get_parameter("cable_drum_manual_seconds", cable_drum_manual_seconds);

        cable_drum_goal_msg.direction = DrumManualRoll::Goal::DIRECTION_OUT;
        cable_drum_goal_msg.duty_cycle = cable_drum_manual_duty_cycle;
        cable_drum_goal_msg.seconds = cable_drum_manual_seconds;

        auto drum_send_goal_options = rclcpp_action::Client<DrumManualRoll>::SendGoalOptions();

        drum_send_goal_options.goal_response_callback = std::bind(&DoubleCableLander::drumManualRollGoalResponseCallback, this, _1);
        drum_send_goal_options.feedback_callback = std::bind(&DoubleCableLander::drumManualRollFeedbackCallback, this, _1, _2);
        drum_send_goal_options.result_callback = std::bind(&DoubleCableLander::drumManualRollResultCallback, this, _1);



        this->cable_takeoff_client_->async_send_goal(traj_goal_msg, traj_send_goal_options);
        this->drum_manual_roll_client_->async_send_goal(cable_drum_goal_msg, drum_send_goal_options);

        return trajectory_and_drum_goal_wait();

    };


    auto set_cable_drum_tracking = [this]() -> bool {

        uint8_t cable_drum_tracking_gain;
        this->get_parameter("cable_drum_tracking_gain", cable_drum_tracking_gain);

        auto gain_request = std::make_shared<iii_interfaces::srv::DrumSetGain::Request>();
        gain_request->gain = cable_drum_tracking_gain;

        while(!this->drum_set_gain_client_->wait_for_service(100ms)) {

            if (!rclcpp::ok()) {

                RCLCPP_FATAL(this->get_logger(), "Interrupted while waiting for service, exiting...");

            }
        }

        auto gain_result = this->drum_set_gain_client_->async_send_request(gain_request);

        gain_result.wait();

        bool success = gain_result.get().get()->success;

        //bool success = rclcpp::spin_until_future_complete(this->get_node_base_interface(), gain_result) == rclcpp::executor::FutureReturnCode::SUCCESS;

        if (!success)
            return false;




        uint8_t cable_drum_tracking_reference;
        this->get_parameter("cable_drum_tracking_reference", cable_drum_tracking_reference);

        auto reference_request = std::make_shared<iii_interfaces::srv::DrumSetReference::Request>();
        reference_request->reference = cable_drum_tracking_reference;

        while(!this->drum_set_reference_client_->wait_for_service(100ms)) {

            if (!rclcpp::ok()) {

                RCLCPP_FATAL(this->get_logger(), "Interrupted while waiting for service, exiting...");

            }
        }

        auto reference_result = this->drum_set_reference_client_->async_send_request(reference_request);

        reference_result.wait();

        success = reference_result.get().get()->success;

        //success = rclcpp::spin_until_future_complete(this->get_node_base_interface(), reference_result) == rclcpp::executor::FutureReturnCode::SUCCESS;

        if (!success)
            return false;

        


        auto mode_request = std::make_shared<iii_interfaces::srv::DrumSetMode::Request>();
        mode_request->mode = iii_interfaces::srv::DrumSetMode::Request::MODE_REF_TRACK;

        while(!this->drum_set_mode_client_->wait_for_service(100ms)) {

            if (!rclcpp::ok()) {

                RCLCPP_FATAL(this->get_logger(), "Interrupted while waiting for service, exiting...");

            }
        }

        auto mode_result = this->drum_set_mode_client_->async_send_request(mode_request);

        mode_result.wait();

        success = mode_result.get().get()->success;

        //success = rclcpp::spin_until_future_complete(this->get_node_base_interface(), mode_result) == rclcpp::executor::FutureReturnCode::SUCCESS;

        return success;

    };

    auto set_cable_drum_manual = [this]() -> bool {

        auto mode_request = std::make_shared<iii_interfaces::srv::DrumSetMode::Request>();
        mode_request->mode = iii_interfaces::srv::DrumSetMode::Request::MODE_MANUAL;

        while(!this->drum_set_mode_client_->wait_for_service(100ms)) {

            if (!rclcpp::ok()) {

                RCLCPP_FATAL(this->get_logger(), "Interrupted while waiting for service, exiting...");

            }
        }

        auto mode_result = this->drum_set_mode_client_->async_send_request(mode_request);

        mode_result.wait();

        bool success = mode_result.get().get()->success;

        //bool success = rclcpp::spin_until_future_complete(this->get_node_base_interface(), mode_result) == rclcpp::executor::FutureReturnCode::SUCCESS;

        return success;

    };

    auto set_cable_drum_off = [this]() -> bool {

        auto mode_request = std::make_shared<iii_interfaces::srv::DrumSetMode::Request>();
        mode_request->mode = iii_interfaces::srv::DrumSetMode::Request::MODE_OFF;

        while(!this->drum_set_mode_client_->wait_for_service(100ms)) {

            if (!rclcpp::ok()) {

                RCLCPP_FATAL(this->get_logger(), "Interrupted while waiting for service, exiting...");

            }
        }

        auto mode_result = this->drum_set_mode_client_->async_send_request(mode_request);

        mode_result.wait();
        
        bool success = mode_result.get().get()->success;

        //bool success = rclcpp::spin_until_future_complete(this->get_node_base_interface(), mode_result) == rclcpp::executor::FutureReturnCode::SUCCESS;

        return success;

    };

    auto set_general_target_yaw = [this](iii_interfaces::msg::Powerline powerline, int first_cable_id, int second_cable_id) -> bool {

        std::cout << "a.1\n";

        vector_t first_cable_pos;
        vector_t second_cable_pos;

        bool first_cable_found = false;
        bool second_cable_found = false;

        std::cout << "a.2\n";

        for (int i = 0; i < powerline.count; i++) {

        std::cout << "a.3\n";

            if (powerline.ids[i] == first_cable_id) {

        std::cout << "a.4\n";

                geometry_msgs::msg::PoseStamped tmp_pose;
                tmp_pose.header.frame_id = powerline.poses[i].header.frame_id;
                tmp_pose.pose = powerline.poses[i].pose;

                geometry_msgs::msg::PoseStamped transformed_pose = tf_buffer_->transform(tmp_pose, "world");

                first_cable_pos(0) = transformed_pose.pose.position.x;
                first_cable_pos(1) = transformed_pose.pose.position.y;
                first_cable_pos(2) = transformed_pose.pose.position.z;

                first_cable_found = true;

        std::cout << "a.5\n";

            } else if (powerline.ids[i] == second_cable_id) {

        std::cout << "a.6\n";

                geometry_msgs::msg::PoseStamped tmp_pose;
                tmp_pose.header.frame_id = powerline.poses[i].header.frame_id;
                tmp_pose.pose = powerline.poses[i].pose;

                geometry_msgs::msg::PoseStamped transformed_pose = tf_buffer_->transform(tmp_pose, "world");

                second_cable_pos(0) = transformed_pose.pose.position.x;
                second_cable_pos(1) = transformed_pose.pose.position.y;
                second_cable_pos(2) = transformed_pose.pose.position.z;

                second_cable_found = true;

        std::cout << "a.7\n";

            }

            if (first_cable_found && second_cable_found)
                break;
        }

        std::cout << "a.8\n";

        if (!first_cable_found || !second_cable_found)
            return false;

        std::cout << "a.9\n";

        vector_t second_to_first_cable_vec = first_cable_pos - second_cable_pos;
        second_to_first_cable_vec(2) = 0;

        vector_t unit_x(1,0,0);

        vector_t tmp_vec(
            unit_x(0)*second_to_first_cable_vec(1)-second_to_first_cable_vec(0)*unit_x(1),
            unit_x(0)*unit_x(1)+second_to_first_cable_vec(0)*second_to_first_cable_vec(1),
            0
        );

        std::cout << "a.10\n";

        float target_yaw_cable_drum = atan2(tmp_vec(0), tmp_vec(1)) + M_PI_2;

        geometry_msgs::msg::TransformStamped T_drone_to_cable_drum = tf_buffer_->lookupTransform("cable_drum", "drone", tf2::TimePointZero);

        std::cout << "a.11\n";

        quat_t q(
            T_drone_to_cable_drum.transform.rotation.w,
            T_drone_to_cable_drum.transform.rotation.x,
            T_drone_to_cable_drum.transform.rotation.y,
            T_drone_to_cable_drum.transform.rotation.z
        );

        orientation_t eul = quatToEul(q);

        float target_yaw = target_yaw_cable_drum - eul(2);

        orientation_t target_eul(0,0,target_yaw);
        target_quat_ = eulToQuat(target_eul);

        auto request = std::make_shared<iii_interfaces::srv::SetGeneralTargetYaw::Request>();
        request->target_yaw = target_yaw;

        std::cout << "a.12\n";

        auto target_yaw_client = this->create_client<iii_interfaces::srv::SetGeneralTargetYaw>("/trajectory_controller/set_general_target_yaw");

        while(!target_yaw_client->wait_for_service(100ms)) {

        std::cout << "a.13\n";

            if (!rclcpp::ok()) {

                RCLCPP_FATAL(this->get_logger(), "Interrupted while waiting for service, exiting...");

        std::cout << "a.14\n";

                return false;

            }

        std::cout << "a.15\n";
        }

        std::cout << "a.16\n";

        auto result = target_yaw_client->async_send_request(request);

        std::cout << "a.17\n";

        result.wait();
        
        bool success = result.get().get()->success;

        //bool success = result.get() == rclcpp::executor::FutureReturnCode::SUCCESS;
        // bool success = rclcpp::spin_until_future_complete(this, result) == rclcpp::executor::FutureReturnCode::SUCCESS;

        if (success) std::cout << "ololoo\n";
        else std::cout << "aalalalal\n";

        std::cout << "a.18\n";

        return success;

    };

    while(!exit) {

        std::cout << "1\n";

        if (goal_handle->is_canceling()) {

            result->success = false;

            goal_handle->canceled(result);

            state_ = idle;

            cancel_all_action_requests();

            return;

        }

        switch(state_) {
        default:
        case idle:

            std::cout << "2\n";

            powerline = getPowerline();

            std::cout << "2.1\n";

            if (set_general_target_yaw(powerline, first_cable_id_, second_cable_id_) 
                    && compute_pose_under_cable(powerline, first_cable_id_, under_cable_pose)) {

            std::cout << "2.2\n";
                
                state_ = fly_under_first_cable;

            } else {

            std::cout << "2.3\n";

                abort_goal();

            }

            break;

        case fly_under_first_cable:

        std::cout << "3\n";

            if (fly_to_under_cable_blocking(under_cable_pose)) {

                state_ = land_on_first_cable;

            } else {

                abort_goal();

            }

            break;

        case land_on_first_cable:

        std::cout << "4\n";

            if (land_on_cable_blocking(first_cable_id_)) {

                std::cout << "4.1\n";

                set_cable_drum_manual();

                state_ = leave_first_cable;

            } else {

                std::cout << "4.2\n";

                abort_goal();

            }

            break;

        case leave_first_cable:

        std::cout << "5\n";

            if (leave_cable_blocking()) {

                std::cout << "5.1\n";

                set_cable_drum_tracking();

                std::cout << "5.2\n";

                powerline = getPowerline();

                std::cout << "5.3\n";

                if (compute_pose_under_cable(powerline, second_cable_id_, under_cable_pose)) {

                std::cout << "5.4\n";

                    state_ = fly_under_second_cable;

                } else {

                std::cout << "5.5\n";

                    abort_goal();

                }


            } else {

                std::cout << "5.6\n";

                abort_goal();

            }

            break;

        case fly_under_second_cable:

        std::cout << "6\n";

            if (fly_to_under_cable_blocking(under_cable_pose)) {

                state_ = land_on_second_cable;

            } else {

                abort_goal();

            }

            break;

        case land_on_second_cable:

        std::cout << "7\n";

            if (land_on_cable_blocking(second_cable_id_)) {

                set_cable_drum_off();

                state_ = on_second_cable;

                result->success = true;

                goal_handle->succeed(result);

                exit = true;

            } else {

                abort_goal();

            }

            break;

        }
    }
}

iii_interfaces::msg::Powerline DoubleCableLander::getPowerline() {

    iii_interfaces::msg::Powerline powerline;

    powerline_mutex_.lock(); {

        powerline = powerline_;

    } powerline_mutex_.unlock(); 

    return powerline;

}

void DoubleCableLander::setPowerline(iii_interfaces::msg::Powerline powerline) {

    powerline_mutex_.lock(); {

        powerline_ = powerline;

    } powerline_mutex_.unlock();

}

void DoubleCableLander::powerlineCallback(iii_interfaces::msg::Powerline::SharedPtr msg) {

    setPowerline(*msg);

}

void DoubleCableLander::flyToPositionGoalResponseCallback(std::shared_future<GoalHandleFlyToPosition::SharedPtr> future) {

    std::cout << "What do asd\n";

    auto goal_handle = future.get();

    std::cout << "What do asd\n";

    if (goal_handle) {

        trajectory_goal_response_ = 1;

        std::cout << "success goal\n";
    }
    else {

        trajectory_goal_response_ = -1;
        std::cout << "fail goal\n";
    }

}

void DoubleCableLander::flyToPositionFeedbackCallback(GoalHandleFlyToPosition::SharedPtr, 
                    const std::shared_ptr<const FlyToPosition::Feedback> feedback) {

    //

}

void DoubleCableLander::flyToPositionResultCallback(const GoalHandleFlyToPosition::WrappedResult &result) {

    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        trajectory_goal_result_ = 1;
    else   
        trajectory_goal_result_ = -1;

}

void DoubleCableLander::cableLandingGoalResponseCallback(std::shared_future<GoalHandleCableLanding::SharedPtr> future) {

    auto goal_handle = future.get();

    if (goal_handle)
        trajectory_goal_response_ = 1;
    else
        trajectory_goal_response_ = -1;

}

void DoubleCableLander::cableLandingFeedbackCallback(GoalHandleCableLanding::SharedPtr, 
                    const std::shared_ptr<const CableLanding::Feedback> feedback) {

    //

}

void DoubleCableLander::cableLandingResultCallback(const GoalHandleCableLanding::WrappedResult &result) {

    std::cout << "Cable landing result callback\n";

    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        trajectory_goal_result_ = 1;
    else
        trajectory_goal_result_ = -1;

}

void DoubleCableLander::cableTakeoffGoalResponseCallback(std::shared_future<GoalHandleCableTakeoff::SharedPtr> future) {

    auto goal_handle = future.get();

    if (goal_handle)
        trajectory_goal_response_ = 1;
    else
        trajectory_goal_response_ = -1;

}

void DoubleCableLander::cableTakeoffFeedbackCallback(GoalHandleCableTakeoff::SharedPtr, 
                    const std::shared_ptr<const CableTakeoff::Feedback> feedback) {

    //

}

void DoubleCableLander::cableTakeoffResultCallback(const GoalHandleCableTakeoff::WrappedResult &result) {

    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        trajectory_goal_result_ = 1;
    else   
        trajectory_goal_result_ = -1;

}

void DoubleCableLander::drumManualRollGoalResponseCallback(std::shared_future<GoalHandleDrumManualRoll::SharedPtr> future) {

    auto goal_handle = future.get();

    if (goal_handle)
        cable_drum_goal_response_ = 1;
    else
        cable_drum_goal_response_ = -1;

}

void DoubleCableLander::drumManualRollFeedbackCallback(GoalHandleDrumManualRoll::SharedPtr, 
                    const std::shared_ptr<const DrumManualRoll::Feedback> feedback) {

    //

}

void DoubleCableLander::drumManualRollResultCallback(const GoalHandleDrumManualRoll::WrappedResult &result) {

    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        cable_drum_goal_result_ = 1;
    else   
        cable_drum_goal_result_ = -1;

}

void DoubleCableLander::publishCallback() {

    std_msgs::msg::String msg;
    
    switch(state_) {

    default:
    case idle:
        msg.data = "idle";
        break;

    case fly_under_first_cable:
        msg.data = "fly under first cable";
        break;

    case land_on_first_cable:
        msg.data = "land on first cable";
        break;

    case leave_first_cable:
        msg.data = "leave first cable";
        break;

    case fly_under_second_cable:
        msg.data = "fly under second cable";
        break;

    case land_on_second_cable:
        msg.data = "land on second cable";
        break;

    case on_second_cable:
        msg.data = "on second cable";
        break;

    }

    state_pub_->publish(msg);

}

int main(int argc, char* argv[]) {
	 std::cout << "Starting DoubleCableLander node..." << std::endl;

	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DoubleCableLander>());

	rclcpp::shutdown();
	return 0;
}