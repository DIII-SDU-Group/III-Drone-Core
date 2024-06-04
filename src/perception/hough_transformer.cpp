/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "iii_drone_core/perception/hough_transformer.hpp"

using namespace iii_drone::perception;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

HoughTransformer::HoughTransformer(iii_drone::configuration::ParameterBundle::SharedPtr parameters) : parameters_(parameters) { }

HoughTransformer::~HoughTransformer() {

	parameters_.reset();

 }

const std::vector<cv::Vec2f> HoughTransformer::GetHoughLines(const cv::Mat img) const {

	cv::Mat edge;

    int canny_low_threshold = parameters_->GetParameter("canny_low_threshold").as_int();
    int canny_ratio = parameters_->GetParameter("canny_ratio").as_int();
    int canny_kernel_size = parameters_->GetParameter("canny_kernel_size").as_int();

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

float HoughTransformer::ComputeAngle(
    const std::vector<cv::Vec2f> lines,
    int rows,
    int cols
) const {

    if (lines.size() == 0) {
        throw std::runtime_error("HoughTransformer::GetHoughLineAngle(): Empty lines vector.");
    }

	float theta = 0.0;

    int idx = getBestLineIndex(
        lines, 
        rows,
        cols
    );

    theta = lines[idx][1];

    return theta;

}

bool HoughTransformer::ComputeAngle(
	const cv::Mat img,
	float & angle
) const {

	const std::vector<cv::Vec2f> lines = GetHoughLines(img);

	if (lines.size() == 0) {
		return false;
	}

	angle = ComputeAngle(
		lines,
		img.rows,
		img.cols
	);

	return true;

}

int HoughTransformer::getBestLineIndex(
    const std::vector<cv::Vec2f> lines,
    int rows,
    int cols
) const {

	int best_idx = -1;
	float best_dist = 100000.;

	float x0 = cols/2;
	float y0 = rows/2;

    for( size_t i = 0; i < lines.size(); i++ ) {
		
		float a = -cos(lines[i][1])/sin(lines[i][1]);
		float b = lines[i][0]/sin(lines[i][1]);

		float dist = abs(a*x0 - y0 + b) / sqrt(a*a+1);

		if (dist < best_dist) {
			best_idx = i;
		}
	}

    return best_idx;

}
