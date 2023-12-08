/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/perception/powerline_direction_parameters.hpp>

/*****************************************************************************/
// Implementation
/*****************************************************************************/

using namespace iii_drone::perception;

PowerlineDirectionParameters::PowerlineDirectionParameters(
    std::string drone_frame_id,
    float kf_r,
    float kf_q
) : drone_frame_id_(drone_frame_id),
    kf_r_(kf_r),
    kf_q_(kf_q) {}

const std::string & PowerlineDirectionParameters::drone_frame_id() const {

    return drone_frame_id_;

}

std::string & PowerlineDirectionParameters::drone_frame_id() {

    return drone_frame_id_;

}

const float & PowerlineDirectionParameters::kf_r() const {

    return kf_r_;

}

float & PowerlineDirectionParameters::kf_r() {

    return kf_r_;

}

const float & PowerlineDirectionParameters::kf_q() const {

    return kf_q_;

}

float & PowerlineDirectionParameters::kf_q() {

    return kf_q_;

}