/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "hough_transformer_HW.h"

/*****************************************************************************/
// Implementation
/*****************************************************************************/

HoughInterfacerNode::HoughInterfacerNode(const std::string & node_name, const std::string & node_namespace) 
    : Node(node_name, node_namespace)
{

    // Params
    this->declare_parameter<int>("canny_low_threshold", 50);
    this->declare_parameter<int>("canny_ratio", 4);
    this->declare_parameter<int>("canny_kernel_size", 3);
    this->declare_parameter<int>("n_lines_include", 1); // only use first value to accomodate edge case ±PI flip

    this->get_parameter("canny_low_threshold", canny_low_threshold_);
    this->get_parameter("canny_ratio", canny_ratio_);
    this->get_parameter("canny_kernel_size", canny_kernel_size_);
    this->get_parameter("n_lines_include", n_lines_include_);

    initIPs();
	    
    RCLCPP_INFO(this->get_logger(), "Successfully initialized IPs");

    rclcpp::QoS video_qos(10);
    video_qos.keep_last(10);
    video_qos.reliable();
    video_qos.durability_volatile();

    // subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
    //         "/usb_cam/image_raw", video_qos, std::bind(&HoughInterfacerNode::imageRecvCallback, this, std::placeholders::_1));
    subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/cable_camera/image_raw", video_qos, std::bind(&HoughInterfacerNode::imageRecvCallback, this, std::placeholders::_1));
    
    cable_yaw_publisher_ = this->create_publisher<iii_interfaces::msg::PowerlineDirection>(
        "cable_yaw_angle", 10);

}

HoughInterfacerNode::~HoughInterfacerNode()
{
    XHoughlines_accel_Release(&xhl);
}

void HoughInterfacerNode::initIPs()
{    
    
    int success = XHoughlines_accel_Initialize(&xhl, HOUGHLINES_ACCEL_NAME);

	if (success == XST_DEVICE_NOT_FOUND)
	{
		RCLCPP_FATAL(this->get_logger(), "Device not found");
		
        rclcpp::shutdown();
	}

	if (success == XST_OPEN_DEVICE_FAILED)
	{
		RCLCPP_FATAL(this->get_logger(), "Open device failed");
		
        rclcpp::shutdown();
	}

    if (success != XST_SUCCESS)
    {
        RCLCPP_FATAL(this->get_logger(), "Component initialization failed ");
    
        rclcpp::shutdown();
    }        
    else{
        RCLCPP_INFO(this->get_logger(), "Component initialization successful");
    }
}

void HoughInterfacerNode::imageRecvCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    
    RCLCPP_DEBUG(this->get_logger(), "Image received");

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);

    cv::Mat img = cv_ptr->image;

	cv::Mat edge;
	cv::Canny(img, edge, canny_low_threshold_, canny_low_threshold_*canny_ratio_, canny_kernel_size_); // edge detection

    float lines[32]; // will hold the results of the detection
    int status = getHoughLines(edge, &(lines[0]));

    if (!status)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed getting hough lines");

        return;
    }

    RCLCPP_DEBUG(this->get_logger(), "Successfully obtained hough lines");

	float avg_theta_tmp = 0.0;

    for( size_t i = 0; i < n_lines_include_; i++ )
    {
        float theta = lines[i];
		// Fix nan
		if (theta != theta){
			theta = 0;
		}
		// Convert from {0,180} deg to {-90,90} deg
		if (theta > M_PI/2)
		{
			theta = -(M_PI - theta);
		}
		avg_theta_tmp = avg_theta_tmp + theta;
    }

    if (avg_theta_tmp == 0.0) {
        // RCLCPP_INFO(this->get_logger(), "No lines detected");
        return;
    }

	
    // Make compatible with right hand rule
    float avg_theta_ = - avg_theta_tmp / n_lines_include_;

    iii_interfaces::msg::PowerlineDirection pl_msg;
    pl_msg.angle = avg_theta_;

    RCLCPP_DEBUG(this->get_logger(),  "Publishing cable yaw");
    cable_yaw_publisher_->publish(pl_msg);

	// RCLCPP_INFO(this->get_logger(),  "Theta: %f %f %f %f %f %f %f %f", lines[0], lines[1], lines[2], lines[3], lines[4], lines[5], lines[6], lines[7]);
	// RCLCPP_INFO(this->get_logger(),  "Theta avg: %f", avg_theta_);
}


int HoughInterfacerNode::getHoughLines(const cv::Mat img, float *lines)
{
    RCLCPP_DEBUG(this->get_logger(), "Calling houghlines_accel");

    if (img.total() != SIZE)
    {
        RCLCPP_ERROR(this->get_logger(), "Expected image of size %d, got size %d, aborting", SIZE, img.total());

        return 0;
    }

    std::vector<uint8_t> img_vec_in;

    img_vec_in.assign(img.data, img.data + img.total());

    int success = callHoughIP(&img_vec_in[0], (uint8_t *)lines);

    if (!success)
    {
        RCLCPP_ERROR(this->get_logger(), "Unsuccessful call to IP");

        return 0;
    }
    
    RCLCPP_DEBUG(this->get_logger(), "Successful call to stream_to_finn IP");

    return 1;
}

int HoughInterfacerNode::callHoughIP(uint8_t *ptr_img_data_in, uint8_t *ptr_lines_out)
{
    int length;

    RCLCPP_DEBUG(this->get_logger(), "Polling for houghlines_accel IP ready");

    while(!XHoughlines_accel_IsReady(&xhl)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    length = XHoughlines_accel_Write_img_in_Bytes(&xhl, 0, (char *)ptr_img_data_in, SIZE);

    if(length == SIZE)
    {
        RCLCPP_DEBUG(this->get_logger(), "Wrote batch to houghlines_accel IP");
    } else
    {
        RCLCPP_ERROR(this->get_logger(), "Could not write batch to houghlines_accel IP");
        
        return 0;
    }

    RCLCPP_DEBUG(this->get_logger(), "Polling for houghlines_accel IP idle");

    while(!XHoughlines_accel_IsIdle(&xhl)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    RCLCPP_DEBUG(this->get_logger(), "Starting houghlines_accel IP");

    XHoughlines_accel_Start(&xhl);

    RCLCPP_DEBUG(this->get_logger(), "Started houghlines_accel IP");

    while(!XHoughlines_accel_IsIdle(&xhl)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    RCLCPP_DEBUG(this->get_logger(), "IP houghlines_accel is idle, reading out bytes");

    XHoughlines_accel_Read_theta_array_Bytes(&xhl, 0, (char*)(&ptr_lines_out[0]), LINES_OUT);

    return 1;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor executor;

    auto node = std::make_shared<HoughInterfacerNode>();

    executor.add_node(node);

    executor.spin();

    rclcpp::shutdown();

    return 0;
}
