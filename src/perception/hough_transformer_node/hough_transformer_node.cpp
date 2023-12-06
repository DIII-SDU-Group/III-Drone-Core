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
) : Node(
	node_name, 
	node_namespace,
	options
), configurator_(this),
hough_transformer_(configurator_.hough_transformer_parameters()) {

	// Topics:
	cable_yaw_publisher_ = this->create_publisher<iii_drone_interfaces::msg::PowerlineDirection>(
		"cable_yaw_angle", 
		10
	);

	rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10));
	qos.best_effort();
	qos.durability_volatile();

	camera_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
		"/sensor/cable_camera/image_raw",	
		qos,
		std::bind(
			&HoughTransformerNode::OnCameraMsg, 
			this, 
			std::placeholders::_1
		)
	);

}

HoughTransformerNode::~HoughTransformerNode() {

	RCLCPP_INFO(this->get_logger(),  "Shutting down hough transformer node..");

}

// mmwave message callback function
void HoughTransformerNode::OnCameraMsg(const sensor_msgs::msg::Image::SharedPtr _msg){

	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
		_msg, 
		_msg->encoding
	);

	cv::Mat img = cv_ptr->image;

	std::vector<cv::Vec2f> lines = hough_transformer_.GetHoughLines(img);

	if (lines.size() > 0){

		PowerlineDirection pl_direction(
			lines, 
			img.rows, 
			img.cols
		);

		cable_yaw_publisher_->publish(pl_direction.ToMsg());

		// RCLCPP debug published hough angle
		RCLCPP_DEBUG(this->get_logger(), "Hough angle: %f", pl_direction.angle());

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

	rclcpp::spin(std::make_shared<HoughTransformerNode>());

	rclcpp::shutdown();
	return 0;
}
