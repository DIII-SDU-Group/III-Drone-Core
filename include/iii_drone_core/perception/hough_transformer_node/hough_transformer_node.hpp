#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/qos.hpp>

/*****************************************************************************/
// CV:

#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include "iii_drone_interfaces/msg/powerline_direction.hpp"

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/perception/hough_transformer_node/hough_transformer_node_configurator.hpp>
#include <iii_drone_core/perception/hough_transformer_parameters.hpp>
#include <iii_drone_core/perception/hough_transformer.hpp>

#include <iii_drone_core/perception/powerline_direction.hpp>

/*****************************************************************************/
// Std:

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

	/**
	 * @brief Node for computing hough transform on images to detect powerline direction.
	*/
	class HoughTransformerNode : public rclcpp::Node {
	public:
		/**
		 *	@brief Constructor
		 *
		 * @param node_name Name of the node
		 * @param node_namespace Namespace of the node
		 * @param options Options for the node
		 */
		HoughTransformerNode(
			const std::string & node_name="hough_transformer", 
			const std::string & node_namespace="/perception/hough_transformer",
			const rclcpp::NodeOptions & options = rclcpp::NodeOptions()
		);

		/**
		 *	@brief Destructor
		 */
		~HoughTransformerNode();

	private:
		/**
		 * @brief Configurator object
		 */
		HoughTransformerConfigurator configurator_;

		/**
		 * @brief HoughTransformer object
		 */
		HoughTransformer hough_transformer_;

		/**
		 *	@brief Subscription object for the image topic
		 */
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;

		/**
		 * @brief Callback function for the image topic
		 *
		 * @param _msg Message containing the image
		 */
		void OnCameraMsg(const sensor_msgs::msg::Image::SharedPtr _msg);

		/**
		 * @brief Publisher object for the cable yaw angle
		 */
		rclcpp::Publisher<iii_drone_interfaces::msg::PowerlineDirection>::SharedPtr cable_yaw_publisher_;

	};

} // namespace hough_transformer_node
} // namespace perception
} // namespace iii_drone

/*****************************************************************************/
// Main
/*****************************************************************************/
			
int main(int argc, char *argv[]);