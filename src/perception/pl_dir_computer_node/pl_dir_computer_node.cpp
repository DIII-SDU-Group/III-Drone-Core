/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/perception/pl_dir_computer_node/pl_dir_computer_node.hpp>

using namespace iii_drone::perception::pl_dir_computer_node;
using namespace iii_drone::types;
using namespace iii_drone::math;

namespace {

using LifecycleConfigurator = iii_drone::configuration::Configurator<rclcpp_lifecycle::LifecycleNode>;
using ParameterType = rclcpp::ParameterType;
using ConfigurationEntry = iii_drone::configuration::configuration_entry_t;

void DeclareManagedParameters(LifecycleConfigurator & configurator)
{
    const auto bool_t = ParameterType::PARAMETER_BOOL;
    const auto int_t = ParameterType::PARAMETER_INTEGER;
    const auto double_t = ParameterType::PARAMETER_DOUBLE;
    const auto string_t = ParameterType::PARAMETER_STRING;

    configurator.DeclareParameter("/perception/begin_running", bool_t);
    configurator.DeclareParameter("/perception/pl_dir_computer/kf_r", double_t);
    configurator.DeclareParameter("/perception/pl_dir_computer/kf_q", double_t);
    configurator.DeclareParameter("/perception/pl_dir_computer/odometry_callback_period_ms", int_t);
    configurator.DeclareParameter("/perception/pl_mapper/min_point_dist", double_t);
    configurator.DeclareParameter("/perception/pl_mapper/max_point_dist", double_t);
    configurator.DeclareParameter("/perception/pl_mapper/view_cone_slope", double_t);
    configurator.DeclareParameter("/tf/drone_frame_id", string_t);
    configurator.DeclareParameter("/tf/world_frame_id", string_t);
    configurator.DeclareParameter("/tf/cable_gripper_frame_id", string_t);
    configurator.DeclareParameter("/tf/mmwave_frame_id", string_t);

    configurator.CreateConfiguration("powerline_direction", {
        ConfigurationEntry("/tf/drone_frame_id", string_t),
        ConfigurationEntry("/perception/pl_dir_computer/kf_r", double_t),
        ConfigurationEntry("/perception/pl_dir_computer/kf_q", double_t),
    });
}

}  // namespace

/*****************************************************************************/
// Implementation
/*****************************************************************************/

PowerlineDirectionComputerNode::PowerlineDirectionComputerNode(
    const std::string & node_name, 
    const std::string & node_namespace,
    const rclcpp::NodeOptions & options
) : rclcpp_lifecycle::LifecycleNode(
    node_name, 
    node_namespace,
    options
) {
    auto set_logger_level = [this](int severity) {
        const rcutils_ret_t ret = rcutils_logging_set_logger_level(this->get_logger().get_name(), severity);
        if (ret != RCUTILS_RET_OK) {
            RCLCPP_WARN(this->get_logger(), "Failed to set logger level, rcutils_ret_t=%d", static_cast<int>(ret));
        }
    };

	const char * log_level_env = std::getenv("PL_DIR_COMPUTER_LOG_LEVEL");
	std::string log_level = log_level_env == nullptr ? "" : log_level_env;

	if (log_level != "") {

		// Convert to upper case:
		std::transform(
			log_level.begin(), 
			log_level.end(), 
			log_level.begin(), 
			[](unsigned char c){ return std::toupper(c); }
		);

		if (log_level == "DEBUG") {
			set_logger_level(RCUTILS_LOG_SEVERITY_DEBUG);
		} else if (log_level == "INFO") {
			set_logger_level(RCUTILS_LOG_SEVERITY_INFO);
		} else if (log_level == "WARN") {
			set_logger_level(RCUTILS_LOG_SEVERITY_WARN);
		} else if (log_level == "ERROR") {
			set_logger_level(RCUTILS_LOG_SEVERITY_ERROR);
		} else if (log_level == "FATAL") {
			set_logger_level(RCUTILS_LOG_SEVERITY_FATAL);
		}

	}

    pl_direction_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "powerline_direction_pose", 
        10
    );

    pl_direction_quat_pub_ = this->create_publisher<geometry_msgs::msg::QuaternionStamped>(
        "powerline_direction_quat", 
        10
    );

    status_pub_ = this->create_publisher<iii_drone_interfaces::msg::StringStamped>(
        "status", 
        10
    );

    status_timer_ = this->create_wall_timer(
        std::chrono::seconds(1), 
        [this]() {
            iii_drone_interfaces::msg::StringStamped status_stamped_msg;
            status_stamped_msg.stamp = this->now();
            status_stamped_msg.data = running_ ? "Running" : "Stopped";
            status_pub_->publish(status_stamped_msg);
        }
    );

    RCLCPP_DEBUG(this->get_logger(), "PowerlineDirectionComputerNode::PowerlineDirectionComputerNode(): PL dir computer");

}

PowerlineDirectionComputerNode::~PowerlineDirectionComputerNode() {

    RCLCPP_INFO(this->get_logger(),  "Shutting down powerline direction computer node..");

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PowerlineDirectionComputerNode::on_configure(const rclcpp_lifecycle::State & state) {

    RCLCPP_INFO(
        this->get_logger(), 
        "PowerlineDirectionComputerNode::on_configure()"
    );

    CallbackReturn parent_return = rclcpp_lifecycle::LifecycleNode::on_configure(state);

    if (parent_return != CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(
            this->get_logger(), 
            "PowerlineDirectionComputerNode::on_configure(): Failed to configure parent class."
        );
        return parent_return;
    }

    // Configurator:
    RCLCPP_DEBUG(
        this->get_logger(), 
        "PowerlineDirectionComputerNode::on_configure(): Initializing configurator object."
    );

    configurator_ = std::make_shared<iii_drone::configuration::Configurator<rclcpp_lifecycle::LifecycleNode>>(this, "pl_dir_computer");
    DeclareManagedParameters(*configurator_);
    configurator_->validate();

    // Powerline direction object:
    RCLCPP_DEBUG(
        this->get_logger(), 
        "PowerlineDirectionComputerNode::on_configure(): Initializing powerline direction object."
    );

    pl_direction_ = std::make_shared<PowerlineDirection>(
        configurator_->GetConfiguration("powerline_direction")
    );

    // Tf
    RCLCPP_DEBUG(
        this->get_logger(), 
        "PowerlineDirectionComputerNode::on_configure(): Initializing tf buffer and transform listener."
    );

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    return CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PowerlineDirectionComputerNode::on_cleanup(const rclcpp_lifecycle::State & state) {

    RCLCPP_INFO(
        this->get_logger(), 
        "PowerlineDirectionComputerNode::on_cleanup()"
    );

    CallbackReturn parent_return = rclcpp_lifecycle::LifecycleNode::on_cleanup(state);

    if (parent_return != CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(
            this->get_logger(), 
            "PowerlineDirectionComputerNode::on_cleanup(): Failed to cleanup parent class."
        );
        return parent_return;
    }

    RCLCPP_DEBUG(
        this->get_logger(), 
        "PowerlineDirectionComputerNode::on_cleanup(): Cleaning up tf buffer and transform listener."
    );

    transform_listener_.reset();
    transform_listener_ = nullptr;

    tf_buffer_->clear();
    tf_buffer_.reset();
    tf_buffer_ = nullptr;

    RCLCPP_DEBUG(
        this->get_logger(), 
        "PowerlineDirectionComputerNode::on_cleanup(): Cleaning up powerline direction object"
    );

    pl_direction_.reset();
    pl_direction_ = nullptr;

    RCLCPP_DEBUG(
        this->get_logger(), 
        "PowerlineDirectionComputerNode::on_cleanup(): Cleaning up configurator object"
    );

    configurator_.reset();
    configurator_ = nullptr;

    return CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PowerlineDirectionComputerNode::on_activate(const rclcpp_lifecycle::State & state) {

    RCLCPP_INFO(
        this->get_logger(), 
        "PowerlineDirectionComputerNode::on_activate()"
    );

    CallbackReturn parent_return = rclcpp_lifecycle::LifecycleNode::on_activate(state);

    if (parent_return != CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(
            this->get_logger(), 
            "PowerlineDirectionComputerNode::on_activate(): Failed to activate parent class."
        );
        return parent_return;
    }

    RCLCPP_DEBUG(
        this->get_logger(), 
        "PowerlineDirectionComputerNode::on_activate(): Initializing subscribers."
    );

    pl_angle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/perception/hough_transformer/cable_yaw_angle", 
        10, 
        [this](const std_msgs::msg::Float32::SharedPtr msg) {
            if (!running_) {
                return;
            }

            RCLCPP_DEBUG(this->get_logger(), "Received powerline angle: %f", msg->data);
            pl_direction_->Update(msg->data);
        }
    );

    pl_sub_ = this->create_subscription<iii_drone_interfaces::msg::Powerline>(
        "/perception/pl_mapper/powerline", 
        10, 
        [this](const iii_drone_interfaces::msg::Powerline::SharedPtr msg) {
            if (!running_) {
                return;
            }

            RCLCPP_DEBUG(this->get_logger(), "Received powerline");
            pl_direction_->UpdatePowerlineAdapter(msg);
        }
    );

    RCLCPP_DEBUG(
        this->get_logger(), 
        "PowerlineDirectionComputerNode::on_activate(): Initializing odometry callback timer."
    );

    std::chrono::milliseconds odom_callback_ms(
        configurator_->GetParameter("/perception/pl_dir_computer/odometry_callback_period_ms").as_int()
    );

    drone_tf_timer_ = this->create_wall_timer(
        odom_callback_ms, 
        std::bind(
            &PowerlineDirectionComputerNode::odometryCallback, 
            this
        )
    );

    RCLCPP_DEBUG(
        this->get_logger(), 
        "PowerlineDirectionComputerNode::on_activate(): Initializing command service."
    );

    command_srv_ = this->create_service<iii_drone_interfaces::srv::SystemCommand>(
        "command", 
        std::bind(
            &PowerlineDirectionComputerNode::commandCallback, 
            this, 
            std::placeholders::_1, 
            std::placeholders::_2
        )
    );

    running_ = configurator_->GetParameter("/perception/begin_running").as_bool();

    return CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PowerlineDirectionComputerNode::on_deactivate(const rclcpp_lifecycle::State & state) {

    RCLCPP_INFO(
        this->get_logger(), 
        "PowerlineDirectionComputerNode::on_deactivate()"
    );

    CallbackReturn parent_return = rclcpp_lifecycle::LifecycleNode::on_deactivate(state);

    if (parent_return != CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(
            this->get_logger(), 
            "PowerlineDirectionComputerNode::on_deactivate(): Failed to deactivate parent class."
        );
        return parent_return;
    }

    RCLCPP_DEBUG(
        this->get_logger(), 
        "PowerlineDirectionComputerNode::on_deactivate(): Cleaning up odom callback timer."
    );

    drone_tf_timer_->cancel();
    drone_tf_timer_.reset();
    drone_tf_timer_ = nullptr;

    RCLCPP_DEBUG(
        this->get_logger(), 
        "PowerlineDirectionComputerNode::on_deactivate(): Cleaning up subscribers."
    );

    pl_angle_sub_->clear_on_new_message_callback();
    pl_angle_sub_.reset();
    pl_angle_sub_ = nullptr;

    pl_sub_->clear_on_new_message_callback();
    pl_sub_.reset();
    pl_sub_ = nullptr;

    RCLCPP_DEBUG(
        this->get_logger(), 
        "PowerlineDirectionComputerNode::on_deactivate(): Cleaning up command service."
    );

    command_srv_->clear_on_new_request_callback();
    command_srv_.reset();
    command_srv_ = nullptr;

    return CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PowerlineDirectionComputerNode::on_shutdown(const rclcpp_lifecycle::State & state) {

    RCLCPP_INFO(
        this->get_logger(), 
        "PowerlineDirectionComputerNode::on_shutdown()"
    );

    CallbackReturn parent_return = rclcpp_lifecycle::LifecycleNode::on_shutdown(state);

    if (parent_return != CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(
            this->get_logger(), 
            "PowerlineDirectionComputerNode::on_shutdown(): Failed to shutdown parent class."
        );
        return parent_return;
    }

    // Create and start thread detached which sleeps for 1 second, then shuts down rclcpp
    std::thread shutdown_thread([this](){
        std::this_thread::sleep_for(std::chrono::seconds(1));
        rclcpp::shutdown();
    });
    shutdown_thread.detach();

    return CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PowerlineDirectionComputerNode::on_error(const rclcpp_lifecycle::State & state) {
    RCLCPP_FATAL(
		this->get_logger(), 
		"PowerlineDirectionComputerNode::on_error(): Lifecycle transition failed."
	);

	return rclcpp_lifecycle::LifecycleNode::on_error(state);

}

void PowerlineDirectionComputerNode::commandCallback(
	const std::shared_ptr<iii_drone_interfaces::srv::SystemCommand::Request> request,
	std::shared_ptr<iii_drone_interfaces::srv::SystemCommand::Response> response
) {

	uint8_t cmd = request->command;

	if (cmd == iii_drone_interfaces::srv::SystemCommand::Request::SYSTEM_COMMAND_START) {

		RCLCPP_INFO(
			this->get_logger(), 
			"PowerlineDirectionComputerNode::commandCallback(): Starting pl dir computer node"
		);

		running_ = true;

		response->ack = response->SYSTEM_ACK_OK;

	} else if (cmd == iii_drone_interfaces::srv::SystemCommand::Request::SYSTEM_COMMAND_STOP) {

		RCLCPP_INFO(
			this->get_logger(), 
			"PowerlineDirectionComputerNode::commandCallback(): Stopping pl dir computer node"
		);

		running_ = false;

		response->ack = response->SYSTEM_ACK_OK;

	} else {

		RCLCPP_WARN(
			this->get_logger(), 
			"PowerlineDirectionComputerNode::commandCallback(): Unknown command"
		);

		response->ack = response->SYSTEM_ACK_INVALID_CMD;

	}
}

void PowerlineDirectionComputerNode::odometryCallback() {

    if (!running_) {
        return;
    }

    RCLCPP_DEBUG(this->get_logger(), "Fetching odometry transform");

    geometry_msgs::msg::TransformStamped tf;

    try {

        tf = tf_buffer_->lookupTransform(
            configurator_->GetParameter("/tf/drone_frame_id").as_string(),
            configurator_->GetParameter("/tf/world_frame_id").as_string(), 
            tf2::TimePointZero
        );

    } catch(tf2::TransformException & ex) {

        RCLCPP_DEBUG(this->get_logger(), "Could not get odometry transform, frame world to drone");
        return;

    }

    quaternion_t quat = quaternionFromTransformMsg(tf.transform);

    pl_direction_->Predict(quat);

    publishPowerlineDirection();

}

void PowerlineDirectionComputerNode::publishPowerlineDirection() {

    RCLCPP_DEBUG(this->get_logger(), "Publishing powerline direction");

    geometry_msgs::msg::PoseStamped pose_msg = pl_direction_->ToPoseStampedMsg(
        configurator_->GetParameter("/tf/drone_frame_id").as_string()
    );
    geometry_msgs::msg::QuaternionStamped quat_msg = pl_direction_->ToQuaternionStampedMsg(
        configurator_->GetParameter("/tf/drone_frame_id").as_string()
    );

    pl_direction_pose_pub_->publish(pose_msg);
    pl_direction_quat_pub_->publish(quat_msg);

    RCLCPP_DEBUG(this->get_logger(), "Finished publishing powerline direction");

}

int main(int argc, char *argv[]) {

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;

    auto node = std::make_shared<PowerlineDirectionComputerNode>();

    executor.add_node(node->get_node_base_interface());

    try {
        
        executor.spin();

    } catch(const std::exception& e) {
        RCLCPP_FATAL(node->get_logger(), "PowerlineDirectionComputerNode main loop failed: %s", e.what());
        node.reset();

    }
    
	if (rclcpp::ok()) {
		node.reset();
		rclcpp::shutdown();
	}

    return 0;

}
