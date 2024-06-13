#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <std_msgs/msg/float32.hpp>

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
// III-Drone-Configuration:

#include <iii_drone_configuration/configurator.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/perception/hough_transformer.hpp>

#include <iii_drone_core/utils/atomic.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/srv/system_command.hpp>

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
	class HoughTransformerNode : public rclcpp_lifecycle::LifecycleNode {
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

		// Lifecycle callbacks

		/**
		 * @brief Callback for configuration of the node
		 * 
		 * @param state State of the node
		 * 
		 * @return Callback return
		 */
		rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
		on_configure(const rclcpp_lifecycle::State & state);

		/**
		 * @brief Callback for cleanup of the node
		 * 
		 * @param state State of the node
		 * 
		 * @return Callback return
		 */
		rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
		on_cleanup(const rclcpp_lifecycle::State & state);

		/**
		 * @brief Callback for activation of the node
		 * 
		 * @param state State of the node
		 * 
		 * @return Callback return
		 */
		rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
		on_activate(const rclcpp_lifecycle::State & state);

		/**
		 * @brief Callback for deactivation of the node
		 * 
		 * @param state State of the node
		 * 
		 * @return Callback return
		 */
		rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
		on_deactivate(const rclcpp_lifecycle::State & state);

		/**
		 * @brief Callback for shutdown of the node
		 * 
		 * @param state State of the node
		 * 
		 * @return Callback return
		 */
		rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
		on_shutdown(const rclcpp_lifecycle::State & state);

		/**
		 * @brief Callback for error of the node
		 * 
		 * @param state State of the node
		 * 
		 * @return Callback return
		 */
		rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
		on_error(const rclcpp_lifecycle::State & state);

	private:
		/**
		 * @brief Configurator object
		 */
		iii_drone::configuration::Configurator<rclcpp_lifecycle::LifecycleNode>::SharedPtr configurator_;

		/**
		 * @brief HoughTransformer object
		 */
		HoughTransformer::SharedPtr hough_transformer_;

		/**
		 *	@brief Subscription object for the image topic
		 */
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;

		/**
		 * @brief Command service
		 */
		rclcpp::Service<iii_drone_interfaces::srv::SystemCommand>::SharedPtr command_service_;

		/**
		 * @brief Callback function for the command service
		 * 
		 * @param request Request message
		 * @param response Response message
		 */
		void commandCallback(
			const std::shared_ptr<iii_drone_interfaces::srv::SystemCommand::Request> request,
			std::shared_ptr<iii_drone_interfaces::srv::SystemCommand::Response> response
		);

		/**
		 * @brief Running flag
		 * 
		 * @details Flag to indicate if the node is running
		 */
		utils::Atomic<bool> running_;

		/**
		 * @brief Callback function for the image topic
		 *
		 * @param _msg Message containing the image
		 */
		void onCameraMsg(const sensor_msgs::msg::Image::SharedPtr _msg);

		/**
		 * @brief Publisher object for the cable yaw angle
		 */
		rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>::SharedPtr cable_yaw_publisher_;

	};

} // namespace hough_transformer_node
} // namespace perception
} // namespace iii_drone

/*****************************************************************************/
// Main
/*****************************************************************************/
			
int main(int argc, char *argv[]);