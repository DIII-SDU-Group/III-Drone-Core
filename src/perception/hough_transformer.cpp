/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "iii_drone_core/perception/hough_transformer.hpp"

using namespace iii_drone::perception;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

HoughTransformer::HoughTransformer(std::shared_ptr<HoughTransformerParameters> parameters) : parameters_(parameters) { }

HoughTransformer::~HoughTransformer() { }

const std::vector<cv::Vec2f> HoughTransformer::GetHoughLines(const cv::Mat img) const {

	cv::Mat edge;

    const int canny_low_threshold = parameters_->canny_low_threshold();
    const int canny_ratio = parameters_->canny_ratio();
    const int canny_kernel_size = parameters_->canny_kernel_size();

	cv::Canny(
		img, 
		edge, 
		canny_low_threshold,
		canny_low_threshold * canny_ratio,
		canny_kernel_size
	); // edge detection

	// Standard Hough Line Transform
    std::vector<cv::Vec2f> lines; // will hold the results of the detection
    cv::HoughLines(
		edge, 
		lines, 
		1, 
		M_PI/180, 
		150, 
		0, 
		0
	); // runs the actual detection

    return lines;

}