#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "iii_interfaces/msg/powerline_direction.hpp"

#include <algorithm>
#include <cstdlib>
#include <stdlib.h> 
#include <iostream>   
#include <math.h>  
#include <vector>
#include <string>
#include <thread>
#include <mutex>


#define PI 3.14159265


using namespace std::chrono_literals;

//creates a HoughTFPub class that subclasses the generic rclcpp::Node base class.
class HoughTFPub : public rclcpp::Node
{
	public:
		HoughTFPub(const std::string & node_name="hough_transformer", const std::string & node_namespace="/hough_transformer") 
						: Node(node_name, node_namespace) {

			// Params
			this->declare_parameter<int>("canny_low_threshold", 50);
			this->declare_parameter<int>("canny_ratio", 4);
			this->declare_parameter<int>("canny_kernel_size", 3);

			this->get_parameter("canny_low_threshold", canny_low_threshold_);
			this->get_parameter("canny_ratio", canny_ratio_);
			this->get_parameter("canny_kernel_size", canny_kernel_size_);

			cable_yaw_publisher_ = this->create_publisher<iii_interfaces::msg::PowerlineDirection>(
				"cable_yaw_angle", 10);


			// hough_yaw_publisher_ = this->create_publisher<iii_interfaces::msg::PowerlineDirection>(
			// 	"hough_yaw_angle", 10);
						

			camera_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
				"/cable_camera/image_raw",	10,
				std::bind(&HoughTFPub::OnCameraMsg, this, std::placeholders::_1));


			odometry_subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>( 
				"/fmu/vehicle_odometry/out",	10,
				std::bind(&HoughTFPub::OnOdoMsg, this, std::placeholders::_1));


		}

		~HoughTFPub() {
			RCLCPP_INFO(this->get_logger(),  "Shutting down hough_tf_pub..");
		}


	private:

		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;
		rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_subscription_;
		rclcpp::Publisher<iii_interfaces::msg::PowerlineDirection>::SharedPtr cable_yaw_publisher_;
		// rclcpp::Publisher<iii_interfaces::msg::PowerlineDirection>::SharedPtr hough_yaw_publisher_;
		void OnCameraMsg(const sensor_msgs::msg::Image::SharedPtr _msg);
		void OnOdoMsg(const px4_msgs::msg::VehicleOdometry::SharedPtr _msg);
		void KF_predict();
		void KF_update(); 

		int getBestLineIndex(std::vector<cv::Vec2f> lines, int img_height, int img_width);

		float yaw_prev_;
		float yaw_curr_;
		float yaw_diff_;

		float Q_ = 0.5;
		float R_ = 0.5;
		float x_hat_;
		float P_hat_;
		std::mutex x_hat_mutex_;
		std::mutex P_hat_mutex_;

		float avg_theta_;

		int canny_low_threshold_;
		int canny_ratio_;
		int canny_kernel_size_;
};


/* Kalman filter predict step */
void HoughTFPub::KF_predict(){

	std::lock_guard<std::mutex> guard1(x_hat_mutex_);
	x_hat_ = x_hat_ + yaw_diff_;

	std::lock_guard<std::mutex> guard2(P_hat_mutex_);
	P_hat_ = P_hat_ + Q_;
}


/* Kalman filter update step */
void HoughTFPub::KF_update(){

	std::lock_guard<std::mutex> guard1(x_hat_mutex_);
	float Y_bar = avg_theta_ - x_hat_;

	std::lock_guard<std::mutex> guard2(P_hat_mutex_);
	float S_k = P_hat_ + R_;

	float K_k = P_hat_ / S_k;

	x_hat_ = x_hat_ + K_k*Y_bar;

	P_hat_ = (1-K_k) * P_hat_; 
}


void HoughTFPub::OnOdoMsg(const px4_msgs::msg::VehicleOdometry::SharedPtr _msg){

	RCLCPP_DEBUG(this->get_logger(),  "On odometry msg");

	//yaw_prev_ = yaw_curr_;

	//float yaw = atan2(2.0 * (_msg->q[3] * _msg->q[0] + _msg->q[1] * _msg->q[2]) , - 1.0 + 2.0 * (_msg->q[0] * _msg->q[0] + _msg->q[1] * _msg->q[1]));
	//// convert to {0, 2PI}
	//if (yaw > 0){
	//	yaw_curr_ = yaw;
	//} else {
	//	yaw_curr_ = 2*PI + yaw; // + because yaw_ is negative
	//}

	//yaw_diff_ = yaw_curr_ - yaw_prev_;
	//// Fix 2PI-0 crossing
	//if (yaw_diff_ > PI)
	//{
	//	yaw_diff_ = yaw_curr_ - (yaw_prev_ + 2*PI);
	//}
	//else if (yaw_diff_ < -PI)
	//{
	//	yaw_diff_ = (yaw_curr_ + 2*PI) - yaw_prev_;
	//}
	

	//KF_predict();

	//iii_interfaces::msg::PowerlineDirection pl_msg;
	//std::lock_guard<std::mutex> guard(x_hat_mutex_);

	//pl_msg.angle = x_hat_;
	//RCLCPP_DEBUG(this->get_logger(),  "Publishing cable yaw");
	//cable_yaw_publisher_->publish(pl_msg);
}


int HoughTFPub::getBestLineIndex(std::vector<cv::Vec2f> lines, int img_height, int img_width) {

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
void HoughTFPub::OnCameraMsg(const sensor_msgs::msg::Image::SharedPtr _msg){

	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(_msg, _msg->encoding);
	cv::Mat img = cv_ptr->image;

	cv::Mat edge;
	cv::Canny(img, edge, canny_low_threshold_, canny_low_threshold_*canny_ratio_, canny_kernel_size_); // edge detection

	// Standard Hough Line Transform
    std::vector<cv::Vec2f> lines; // will hold the results of the detection
    cv::HoughLines(edge, lines, 1, PI/180, 150, 0, 0 ); // runs the actual detection



	float avg_theta_tmp = 0.0;

    // for( size_t i = 0; i < lines.size(); i++ )
    // {
    //     float theta = lines[i][1];
	// 	// Fix nan
	// 	if (theta != theta){
	// 		theta = 0;
	// 	}
	// 	// Convert from {0,180} deg to {-90,90} deg
	// 	if (theta > PI/2)
	// 	{
	// 		theta = -(PI - theta);
	// 	}
	// 	avg_theta_tmp = avg_theta_tmp + theta;

	// 	break;
    // }

	
	if (lines.size() > 0){

		int idx = getBestLineIndex(lines, img.rows, img.cols);

		avg_theta_tmp = lines[idx][1];

		// Make compatible with right hand rule
		//avg_theta_ = - (avg_theta_tmp / (float)lines.size());
		avg_theta_ = -avg_theta_tmp;

		iii_interfaces::msg::PowerlineDirection pl_msg;
		std::lock_guard<std::mutex> guard(x_hat_mutex_);

		pl_msg.angle = avg_theta_;
		//RCLCPP_INFO(this->get_logger(),  "Publishing cable yaw: %f", avg_theta_);
		cable_yaw_publisher_->publish(pl_msg);
		//hough_yaw_publisher_->publish(pl_msg); // 


		// Only update KF when there is a new valid angle
		//KF_update();
	}
		/*
	RCLCPP_INFO(this->get_logger(),  "Theta avg: %f", avg_theta_);
	RCLCPP_INFO(this->get_logger(),  "Hough lines: %d", lines.size());
	RCLCPP_INFO(this->get_logger(),  "Yaw_diff: %f", yaw_diff_);
	RCLCPP_INFO(this->get_logger(),  "Pred. angle: %f \n", x_hat_);
	*/
}

			
int main(int argc, char *argv[])
{
	std::cout << "Starting hough_tf_pub node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<HoughTFPub>());

	rclcpp::shutdown();
	return 0;
}
