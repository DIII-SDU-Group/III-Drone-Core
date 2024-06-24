/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/perception/hough_transformer_node/hough_transformer_node.hpp>

using namespace iii_drone::perception::hough_transformer_node;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

HoughTransformerNode::HoughTransformerNode(
	const std::string & node_name, 
	const std::string & node_namespace,
	const rclcpp::NodeOptions & options
) : LifecycleNode(
	node_name, 
	node_namespace,
	options
) {

	std::string log_level = std::getenv("HOUGH_TRANSFORMER_LOG_LEVEL");

	if (log_level != "") {

		// Convert to upper case:
		std::transform(
			log_level.begin(), 
			log_level.end(), 
			log_level.begin(), 
			[](unsigned char c){ return std::toupper(c); }
		);

		if (log_level == "DEBUG") {
			rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
		} else if (log_level == "INFO") {
			rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);
		} else if (log_level == "WARN") {
			rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_WARN);
		} else if (log_level == "ERROR") {
			rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_ERROR);
		} else if (log_level == "FATAL") {
			rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_FATAL);
		}

	}

	// Topics:
	cable_yaw_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
		"cable_yaw_angle", 
		10
	);

}

HoughTransformerNode::~HoughTransformerNode() {

	RCLCPP_INFO(this->get_logger(),  "Shutting down hough transformer node..");

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn HoughTransformerNode::on_configure(
	const rclcpp_lifecycle::State & state
) {

	RCLCPP_INFO(
		this->get_logger(), 
		"HoughTransformerNode::on_configure()"
	);

	CallbackReturn parent_return = rclcpp_lifecycle::LifecycleNode::on_configure(state);

	if (parent_return != CallbackReturn::SUCCESS) {
		RCLCPP_ERROR(
			this->get_logger(), 
			"Failed to configure parent class."
		);
		return parent_return;
	}

	// Configurator object
	configurator_ = std::make_shared<iii_drone::configuration::Configurator<rclcpp_lifecycle::LifecycleNode>>(this);

	// HoughTransformer object
	hough_transformer_ = std::make_shared<HoughTransformer>(
		configurator_->GetParameterBundle("hough_transformer")
	);

	return CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn HoughTransformerNode::on_cleanup(
	const rclcpp_lifecycle::State & state
) {

	RCLCPP_INFO(
		this->get_logger(), 
		"HoughTransformerNode::on_cleanup()"
	);

	CallbackReturn parent_return = rclcpp_lifecycle::LifecycleNode::on_cleanup(state);

	if (parent_return != CallbackReturn::SUCCESS) {
		RCLCPP_ERROR(
			this->get_logger(), 
			"Failed to cleanup parent class."
		);
		return parent_return;
	}

	RCLCPP_DEBUG(
		this->get_logger(), 
		"Cleaning up hough transformer object..."
	);

	hough_transformer_.reset();
	hough_transformer_ = nullptr;

	RCLCPP_DEBUG(
		this->get_logger(),
		"Cleaning up configurator object..."
	);

	configurator_.reset();
	configurator_ = nullptr;

	RCLCPP_DEBUG(
		this->get_logger(),
		"Finished cleaning up."
	);

	return CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn HoughTransformerNode::on_activate(
	const rclcpp_lifecycle::State & state
) {

	RCLCPP_INFO(
		this->get_logger(), 
		"HoughTransformerNode::on_activate()"
	);

	CallbackReturn parent_return = rclcpp_lifecycle::LifecycleNode::on_activate(state);

	if (parent_return != CallbackReturn::SUCCESS) {
		RCLCPP_ERROR(
			this->get_logger(), 
			"Failed to activate parent class."
		);
		return parent_return;
	}

	rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1));
	qos.best_effort();
	qos.durability_volatile();

	camera_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
		"/sensor/cable_camera/image_raw",	
		qos,
		std::bind(
			&HoughTransformerNode::onCameraMsg, 
			this, 
			std::placeholders::_1
		)
	);

	command_service_ = this->create_service<iii_drone_interfaces::srv::SystemCommand>(
		"command",
		std::bind(
			&HoughTransformerNode::commandCallback, 
			this, 
			std::placeholders::_1, 
			std::placeholders::_2
		)
	);

	running_ = configurator_->GetParameter("begin_running").as_bool();

	return CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn HoughTransformerNode::on_deactivate(
	const rclcpp_lifecycle::State & state
) {

	RCLCPP_INFO(
		this->get_logger(), 
		"HoughTransformerNode::on_deactivate()"
	);

	CallbackReturn parent_return = rclcpp_lifecycle::LifecycleNode::on_deactivate(state);

	if (parent_return != CallbackReturn::SUCCESS) {
		RCLCPP_ERROR(
			this->get_logger(), 
			"Failed to deactivate parent class."
		);
		return parent_return;
	}

	camera_subscription_->clear_on_new_message_callback();
	camera_subscription_.reset();
	camera_subscription_ = nullptr;

	command_service_->clear_on_new_request_callback();
	command_service_.reset();
	command_service_ = nullptr;

	return CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn HoughTransformerNode::on_shutdown(
	const rclcpp_lifecycle::State & state
) {

	RCLCPP_INFO(
		this->get_logger(), 
		"HoughTransformerNode::on_shutdown()"
	);

	CallbackReturn parent_return = rclcpp_lifecycle::LifecycleNode::on_shutdown(state);

	if (parent_return != CallbackReturn::SUCCESS) {
		RCLCPP_ERROR(
			this->get_logger(), 
			"Failed to shutdown parent class."
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

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn HoughTransformerNode::on_error(
	const rclcpp_lifecycle::State & state
) {

	RCLCPP_FATAL(
		this->get_logger(), 
		"HoughTransformerNode::on_error(): An error occured"
	);

	throw std::runtime_error("An error occured");

	return CallbackReturn::ERROR;

}

void HoughTransformerNode::commandCallback(
	const std::shared_ptr<iii_drone_interfaces::srv::SystemCommand::Request> request,
	std::shared_ptr<iii_drone_interfaces::srv::SystemCommand::Response> response
) {

	uint8_t cmd = request->command;

	if (cmd == iii_drone_interfaces::srv::SystemCommand::Request::SYSTEM_COMMAND_START) {

		RCLCPP_INFO(
			this->get_logger(), 
			"HoughTransformerNode::commandCallback(): Starting hough transformer node"
		);

		running_ = true;

		response->ack = response->SYSTEM_ACK_OK;

	} else if (cmd == iii_drone_interfaces::srv::SystemCommand::Request::SYSTEM_COMMAND_STOP) {

		RCLCPP_INFO(
			this->get_logger(), 
			"HoughTransformerNode::commandCallback(): Stopping hough transformer node"
		);

		running_ = false;

		response->ack = response->SYSTEM_ACK_OK;

	} else {

		RCLCPP_WARN(
			this->get_logger(), 
			"HoughTransformerNode::commandCallback(): Unknown command"
		);

		response->ack = response->SYSTEM_ACK_INVALID_CMD;

	}
}

// mmwave message callback function
void HoughTransformerNode::onCameraMsg(const sensor_msgs::msg::Image::SharedPtr _msg){

	if (!running_) {
		return;
	}

	RCLCPP_DEBUG(this->get_logger(), "Received image message, running hough transform...");

	int a = 0;

	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
		_msg, 
		_msg->encoding
	);

	cv::Mat img = cv_ptr->image;

	float angle;
	bool success = hough_transformer_->ComputeAngle(
		img, 
		angle
	);

	if (success){

		std_msgs::msg::Float32 pl_direction;
		pl_direction.data = angle;
		cable_yaw_publisher_->publish(pl_direction);

		// RCLCPP debug published hough angle
		RCLCPP_DEBUG(this->get_logger(), "Hough angle: %f", angle);

	} else {

		// RCLCPP debug no lines detected
		RCLCPP_DEBUG(this->get_logger(), "No lines detected");

	}
}

int main(int argc, char *argv[])
{
	std::cout << "Starting hough_tf_pub node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);

	rclcpp::executors::MultiThreadedExecutor exec;

	auto node = std::make_shared<HoughTransformerNode>();

	exec.add_node(node->get_node_base_interface());

	try {

		exec.spin();

	} catch (const std::exception & e) {

		// std::cerr << e.what() << std::endl;
		node.reset();
	}
	
	if (rclcpp::ok()) {
		node.reset();
		rclcpp::shutdown();
	}

	return 0;
}
