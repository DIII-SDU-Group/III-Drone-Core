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
) {

	// Params
	this->declare_parameter<int>("canny_low_threshold", 50);
	this->declare_parameter<int>("canny_ratio", 4);
	this->declare_parameter<int>("canny_kernel_size", 3);

	cable_yaw_publisher_ = this->create_publisher<iii_drone_interfaces::msg::PowerlineDirection>(
		"cable_yaw_angle", 
		10
	);

	rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10));
	qos.best_effort();
	qos.durability_volatile();

	camera_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
		"/cable_camera/image_raw",	
		qos,
		std::bind(
			&HoughTransformerNode::OnCameraMsg, 
			this, 
			std::placeholders::_1
		)
	);

}

HoughTransformerNode::~HoughTransformerNode() {

	RCLCPP_INFO(this->get_logger(),  "Shutting down hough_tf_pub..");

}

int HoughTransformerNode::getBestLineIndex(
	std::vector<cv::Vec2f> lines, 
	int img_height, 
	int img_width
) {

	int best_idx = -1;
	float best_dist = 100000.;

	float x0 = img_width/2;
	float y0 = img_height/2;

    for( size_t i = 0; i < lines.size(); i++ ) {
		
		float a = -cos(lines[i][1])/sin(lines[i][1]);
		float b = lines[i][0]/sin(lines[i][1]);

		float dist = abs(a*x0 - y0 + b) / sqrt(a*a+1);

		if (dist < best_dist) {
			best_idx = i;
		}
	}
}


// mmwave message callback function
void HoughTransformerNode::OnCameraMsg(const sensor_msgs::msg::Image::SharedPtr _msg){

	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(_msg, _msg->encoding);
	cv::Mat img = cv_ptr->image;

	cv::Mat edge;
	this->get_parameter("canny_low_threshold", canny_low_threshold_);
	this->get_parameter("canny_ratio", canny_ratio_);
	this->get_parameter("canny_kernel_size", canny_kernel_size_);
	cv::Canny(img, edge, canny_low_threshold_, canny_low_threshold_*canny_ratio_, canny_kernel_size_); // edge detection

	// Standard Hough Line Transform
    std::vector<cv::Vec2f> lines; // will hold the results of the detection
    cv::HoughLines(edge, lines, 1, M_PI/180, 150, 0, 0 ); // runs the actual detection

	float avg_theta_tmp = 0.0;

	if (lines.size() > 0){

		int idx = getBestLineIndex(lines, img.rows, img.cols);

		avg_theta_tmp = lines[idx][1];

		avg_theta_ = -avg_theta_tmp;

		iii_drone_interfaces::msg::PowerlineDirection pl_msg;

		pl_msg.angle = avg_theta_;
		cable_yaw_publisher_->publish(pl_msg);

		// RCLCPP debug published hough angle
		RCLCPP_DEBUG(this->get_logger(), "Hough angle: %f", avg_theta_);

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
