/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/perception/powerline_parameters.hpp>

/*****************************************************************************/
// Implementation
/*****************************************************************************/

using namespace iii_drone::perception;

PowerlineParameters::PowerlineParameters(
    const float kf_r,
    const float kf_q,
    const int alive_cnt_low_thresh,
    const int alive_cnt_high_thresh,
    const int alive_cnt_ceiling,
    const float matching_line_max_dist,
    const int max_lines,
    const float min_point_dist,
    const float max_point_dist,
    const float view_cone_slope,
    const float strict_min_point_dist,
    const float strict_max_point_dist,
    const float strict_view_cone_slope,
    const int inter_pos_window_size,
    const std::string drone_frame_id,
    const std::string mmwave_frame_id,
    const bool simulation
) {

    kf_r_ = kf_r;
    kf_q_ = kf_q;
    alive_cnt_low_thresh_ = alive_cnt_low_thresh;
    alive_cnt_high_thresh_ = alive_cnt_high_thresh;
    alive_cnt_ceiling_ = alive_cnt_ceiling;
    matching_line_max_dist_ = matching_line_max_dist;
    max_lines_ = max_lines;
    min_point_dist_ = min_point_dist;
    max_point_dist_ = max_point_dist;
    view_cone_slope_ = view_cone_slope;
    strict_min_point_dist_ = strict_min_point_dist;
    strict_max_point_dist_ = strict_max_point_dist;
    strict_view_cone_slope_ = strict_view_cone_slope;
    inter_pos_window_size_ = inter_pos_window_size;
    drone_frame_id_ = drone_frame_id;
    mmwave_frame_id_ = mmwave_frame_id;
    simulation_ = simulation;

}

const float & PowerlineParameters::kf_r() const {

    std::shared_lock<std::shared_mutex> lock(parameter_mutex_);

    return kf_r_;

}

float & PowerlineParameters::kf_r() {

    std::unique_lock<std::shared_mutex> lock(parameter_mutex_);

    return kf_r_;

}

const float & PowerlineParameters::kf_q() const {

    std::shared_lock<std::shared_mutex> lock(parameter_mutex_);

    return kf_q_;

}

float & PowerlineParameters::kf_q() {

    std::unique_lock<std::shared_mutex> lock(parameter_mutex_);

    return kf_q_;

}

const int & PowerlineParameters::alive_cnt_low_thresh() const {

    std::shared_lock<std::shared_mutex> lock(parameter_mutex_);

    return alive_cnt_low_thresh_;

}

int & PowerlineParameters::alive_cnt_low_thresh() {

    std::unique_lock<std::shared_mutex> lock(parameter_mutex_);

    return alive_cnt_low_thresh_;

}

const int & PowerlineParameters::alive_cnt_high_thresh() const {

    std::shared_lock<std::shared_mutex> lock(parameter_mutex_);

    return alive_cnt_high_thresh_;

}

int & PowerlineParameters::alive_cnt_high_thresh() {

    std::unique_lock<std::shared_mutex> lock(parameter_mutex_);

    return alive_cnt_high_thresh_;

}

const int & PowerlineParameters::alive_cnt_ceiling() const {

    std::shared_lock<std::shared_mutex> lock(parameter_mutex_);

    return alive_cnt_ceiling_;

}

int & PowerlineParameters::alive_cnt_ceiling() {

    std::unique_lock<std::shared_mutex> lock(parameter_mutex_);

    return alive_cnt_ceiling_;

}

const float & PowerlineParameters::matching_line_max_dist() const {

    std::shared_lock<std::shared_mutex> lock(parameter_mutex_);

    return matching_line_max_dist_;

}

float & PowerlineParameters::matching_line_max_dist() {

    std::unique_lock<std::shared_mutex> lock(parameter_mutex_);

    return matching_line_max_dist_;

}

const int & PowerlineParameters::max_lines() const {

    std::shared_lock<std::shared_mutex> lock(parameter_mutex_);

    return max_lines_;

}

int & PowerlineParameters::max_lines() {

    std::unique_lock<std::shared_mutex> lock(parameter_mutex_);

    return max_lines_;

}

const float & PowerlineParameters::min_point_dist() const {

    std::shared_lock<std::shared_mutex> lock(parameter_mutex_);

    return min_point_dist_;

}

float & PowerlineParameters::min_point_dist() {

    std::unique_lock<std::shared_mutex> lock(parameter_mutex_);

    return min_point_dist_;

}

const float & PowerlineParameters::max_point_dist() const {

    std::shared_lock<std::shared_mutex> lock(parameter_mutex_);

    return max_point_dist_;

}

float & PowerlineParameters::max_point_dist() {

    std::unique_lock<std::shared_mutex> lock(parameter_mutex_);

    return max_point_dist_;

}

const float & PowerlineParameters::view_cone_slope() const {

    std::shared_lock<std::shared_mutex> lock(parameter_mutex_);

    return view_cone_slope_;

}

float & PowerlineParameters::view_cone_slope() {

    std::unique_lock<std::shared_mutex> lock(parameter_mutex_);

    return view_cone_slope_;

}

const float & PowerlineParameters::strict_min_point_dist() const {

    std::shared_lock<std::shared_mutex> lock(parameter_mutex_);

    return strict_min_point_dist_;

}

float & PowerlineParameters::strict_min_point_dist() {

    std::unique_lock<std::shared_mutex> lock(parameter_mutex_);

    return strict_min_point_dist_;

}

const float & PowerlineParameters::strict_max_point_dist() const {

    std::shared_lock<std::shared_mutex> lock(parameter_mutex_);

    return strict_max_point_dist_;

}

float & PowerlineParameters::strict_max_point_dist() {

    std::unique_lock<std::shared_mutex> lock(parameter_mutex_);

    return strict_max_point_dist_;

}

const float & PowerlineParameters::strict_view_cone_slope() const {

    std::shared_lock<std::shared_mutex> lock(parameter_mutex_);

    return strict_view_cone_slope_;

}

float & PowerlineParameters::strict_view_cone_slope() {

    std::unique_lock<std::shared_mutex> lock(parameter_mutex_);

    return strict_view_cone_slope_;

}

const int & PowerlineParameters::inter_pos_window_size() const {

    std::shared_lock<std::shared_mutex> lock(parameter_mutex_);
    
    return inter_pos_window_size_;

}

int & PowerlineParameters::inter_pos_window_size() {

    std::unique_lock<std::shared_mutex> lock(parameter_mutex_);

    return inter_pos_window_size_;

}

const std::string & PowerlineParameters::drone_frame_id() const {

    std::shared_lock<std::shared_mutex> lock(parameter_mutex_);

    return drone_frame_id_;

}

const std::string & PowerlineParameters::mmwave_frame_id() const {

    std::shared_lock<std::shared_mutex> lock(parameter_mutex_);

    return mmwave_frame_id_;

}

const bool & PowerlineParameters::simulation() const {

    std::shared_lock<std::shared_mutex> lock(parameter_mutex_);

    return simulation_;

}
