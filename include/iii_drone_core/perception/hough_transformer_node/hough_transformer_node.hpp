#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <rclcpp/qos.hpp>

#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "iii_drone_interfaces/msg/powerline_direction.hpp"

#include <algorithm>
#include <cstdlib>
#include <stdlib.h> 
#include <iostream>   
#include <math.h>  
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <cmath>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace perception {
namespace hough_transformer_node {

	class HoughTransformerNode : public rclcpp::Node {
	public:
		HoughTransformerNode(
			const std::string & node_name="hough_transformer", 
			const std::string & node_namespace="/perception/hough_transformer",
			const rclcpp::NodeOptions & options = rclcpp::NodeOptions()
		);

		~HoughTransformerNode();

	private:

		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;
		rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_subscription_;
		rclcpp::Publisher<iii_drone_interfaces::msg::PowerlineDirection>::SharedPtr cable_yaw_publisher_;
		void OnCameraMsg(const sensor_msgs::msg::Image::SharedPtr _msg);

		int getBestLineIndex(
			std::vector<cv::Vec2f> lines, 
			int img_height, 
			int img_width
		);

		float avg_theta_;

		int canny_low_threshold_;
		int canny_ratio_;
		int canny_kernel_size_;

	};

} // namespace hough_transformer_node
} // namespace perception
} // namespace iii_drone

/*****************************************************************************/
// Main
/*****************************************************************************/
			
int main(int argc, char *argv[]);