/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "iii_drone_core/perception/powerline_direction.hpp"

using namespace iii_drone::perception;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

PowerlineDirection::PowerlineDirection(const float angle) : angle_(angle) { }

PowerlineDirection::PowerlineDirection(
    const std::vector<cv::Vec2f> lines,
    const int rows,
    const int cols
) {

    if (lines.size() == 0) {
        throw std::runtime_error("PowerlineDirection::PowerlineDirection(): Empty lines vector.");
    }

    angle_ = computeAngle(
        lines, 
        rows, 
        cols
    );

}

PowerlineDirection::PowerlineDirection(const iii_drone_interfaces::msg::PowerlineDirection::SharedPtr msg) : angle_(msg->angle) { }

PowerlineDirection::~PowerlineDirection() { }

const iii_drone_interfaces::msg::PowerlineDirection PowerlineDirection::ToMsg() const {

    iii_drone_interfaces::msg::PowerlineDirection msg;
    msg.angle = angle_;

    return msg;

}

const float & PowerlineDirection::angle() const {
    return angle_;
}

const float PowerlineDirection::computeAngle(
    const std::vector<cv::Vec2f> lines,
    const int rows,
    const int cols
) const {

    if (lines.size() == 0) {
        throw std::runtime_error("HoughTransformer::GetHoughLineAngle(): Empty lines vector.");
    }

	float theta = 0.0;

    const int idx = getBestLineIndex(
        lines, 
        rows,
        cols
    );

    theta = -lines[idx][1];

    return theta;

}

const int PowerlineDirection::getBestLineIndex(
    const std::vector<cv::Vec2f> lines,
    const int rows,
    const int cols
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