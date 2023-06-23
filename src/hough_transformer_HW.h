/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <queue>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rate.hpp>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "iii_interfaces/msg/powerline_direction.hpp"

extern "C" {
#include "xhoughlines_accel.h"
}

/*****************************************************************************/
// Defines
/*****************************************************************************/

#define	IMAGE_X		640
#define IMAGE_Y		480
#define SIZE 		IMAGE_X*IMAGE_Y

#define LINES_OUT   32

#define HOUGHLINES_ACCEL_NAME   "houghlines_accel"

using namespace std::chrono_literals;

/*****************************************************************************/
// Class
/*****************************************************************************/

class HoughInterfacerNode : public rclcpp::Node
{
public:
    HoughInterfacerNode(const std::string & node_name="hough_transformer", const std::string & node_namespace="/hough_transformer");
    ~HoughInterfacerNode();

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;

    XHoughlines_accel xhl;

    //std::chrono::time_point<std::chrono::system_clock> start_time;
    //std::chrono::time_point<std::chrono::system_clock> resize_ts;
    //std::chrono::time_point<std::chrono::system_clock> stf_ts;
    //std::chrono::time_point<std::chrono::system_clock> finn_done_ts;

    //bool start_time_set = false;
    //bool timing = false;

    void initIPs();

    void imageRecvCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    int getHoughLines(const cv::Mat img, float *lines);
    int callHoughIP(uint8_t *ptr_img_data_in, uint8_t *lines);

    //void timer_callback();

    //rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<iii_interfaces::msg::PowerlineDirection>::SharedPtr cable_yaw_publisher_;

    int canny_low_threshold_;
    int canny_ratio_;
    int canny_kernel_size_;
    int n_lines_include_;

};

/*****************************************************************************/
// Main
/*****************************************************************************/

int main(int argc, char *argv[]);
