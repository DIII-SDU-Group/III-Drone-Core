// Copyright 2021, Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

// inspired from: https://answers.ros.org/question/230476/how-to-subscribe-to-sensor_msgscompressedimage-without-the-raw-image/ (ROS1)

class ImageDecompress : public rclcpp::Node {
public:
	ImageDecompress() : Node("image_decompressor") {

		compress_img_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
			"/cable_camera/image_raw/compressed",
			10,

			// [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
			// img_ = cv::imdecode(cv::Mat(msg->data),1);  //convert compressed image data to cv::Mat
			// });

			std::bind(&ImageDecompress::imageCallback, this, std::placeholders::_1));

  }

  ~ImageDecompress() {
		RCLCPP_INFO(this->get_logger(),  "Shutting down image decompressor..");
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}

  private:
	rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compress_img_sub_;

	cv::Mat img_;

  void imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr _msg);

};


void ImageDecompress::imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr _msg)
{
  try
  {
    cv::Mat image = cv::imdecode(cv::Mat(_msg->data),1);//convert compressed image data to cv::Mat
    cv::imshow("view", image);

    cv::waitKey(2);
  }
  catch (cv_bridge::Exception& e)
  {
    //RCLCPP_ERROR(logger, "Could not convert to image!");
  }
}

int main(int argc, char ** argv)
{
  std::cout << "Starting ImageDecompress node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);

  cv::namedWindow("view");
  cv::startWindowThread();

	rclcpp::spin(std::make_shared<ImageDecompress>());

  cv::destroyWindow("view");

	rclcpp::shutdown();
	return 0;
}


