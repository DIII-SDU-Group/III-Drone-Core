/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "iii_drone_core/perception/powerline.hpp"

using namespace iii_drone::perception;
using namespace iii_drone::types;
using namespace iii_drone::math;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

Powerline::Powerline(
    iii_drone::configuration::ParameterBundle::SharedPtr powerline_parameters,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer
) 
    : parameters_(powerline_parameters),
    tf_buffer_(tf_buffer),
    pl_dir_history_(1),
    drone_pose_history_(2) {

    Reset();

}

const iii_drone::adapters::PowerlineAdapter Powerline::ToAdapter(bool only_visible) const {

    std::shared_lock<std::shared_mutex> lines_lock(lines_mutex_);

    std::vector<iii_drone::adapters::SingleLineAdapter> line_adapters;

    for (unsigned int i = 0; i < lines_.size(); i++) {

        if (only_visible && !lines_[i].IsVisible()) {

            continue;

        }

        line_adapters.push_back(lines_[i].ToAdapter());

    }

    lines_lock.unlock();

    plane_t projection_plane = projection_plane_;

    iii_drone::adapters::PowerlineAdapter adapter(
        stamp_,
        line_adapters,
        projection_plane
    );

    return adapter;

}

const iii_drone::adapters::PointCloudAdapter Powerline::ToPointCloudAdapter(bool only_visible) const {

    std::shared_lock<std::shared_mutex> lines_lock(lines_mutex_);

    std::vector<point_t> points;

    if (!only_visible) {

        for (unsigned int i = 0; i < lines_.size(); i++) {

            points.push_back(lines_[i].position());

        }

    } else {

        for (unsigned int i = 0; i < lines_.size(); i++) {

            if (lines_[i].IsVisible()) {

                points.push_back(lines_[i].position());

            }
        }
    }

    lines_lock.unlock();

    iii_drone::adapters::PointCloudAdapter adapter(
        stamp_,
        parameters_->GetParameter("drone_frame_id").as_string(),
        points
    );

    return adapter;

}

const std::vector<SingleLine> Powerline::GetVisibleLines() const {

    std::shared_lock<std::shared_mutex> lock(lines_mutex_);

    std::vector<SingleLine> ret_lines;

    for (unsigned int i = 0; i < lines_.size(); i++) {

        if (lines_[i].IsVisible()) {

            ret_lines.push_back(SingleLine(lines_[i]));

        }

    }

    return ret_lines;

}

int Powerline::GetLinesCount() const {

    std::shared_lock<std::shared_mutex> lock(lines_mutex_);

    return lines_.size();

}

point_t Powerline::UpdateLine(const point_t & point) {

    if (!pl_dir_history_.full() || !drone_pose_history_.full()) {

        return point;

    }

    point_t projected_point = projectPoint(point);

    int match_index = findMatchingLine(projected_point);

    int line_count;

    {

        std::shared_lock<std::shared_mutex> shared_lines_lock(lines_mutex_);

        line_count = lines_.size();

    }

    if (match_index == -1 && line_count >= parameters_->GetParameter("max_lines").as_int()) {

        return projected_point;


    } else if (match_index == -1) {

        registerNewLine(projected_point);

    } else {

        std::unique_lock<std::shared_mutex> unique_lines_lock(lines_mutex_);

        lines_[match_index].Update(projected_point);

    }

    stamp_.Update();

    return projected_point;

}

void Powerline::UpdateDirection(const quaternion_t & pl_direction) {

    pl_dir_history_ = pl_direction;

    {
            
        std::unique_lock<std::shared_mutex> lock(lines_mutex_);

        for (unsigned int i = 0; i < lines_.size(); i++) {

            lines_[i].SetDirection(pl_direction);

        }

    }

    if (!drone_pose_history_.full()) {

        return;

    }

    updateProjectionPlane();

    stamp_.Update();

}

void Powerline::UpdateOdometry(const pose_t & drone_pose) {

    drone_pose_history_ = drone_pose;

    if (!drone_pose_history_.full()) {

        return;
    }

    if (!pl_dir_history_.full()) {

        return;

    }

    predictLines();

    stamp_.Update();

}

void Powerline::CleanupLines() {

    std::unique_lock<std::shared_mutex> lines_lock(lines_mutex_);

    lines_.erase(
        std::remove_if(
            lines_.begin(), 
            lines_.end(),
            [](SingleLine& line) { 
                return !line.IsAlive(); 
            }
        ),
        lines_.end()
    );

    std::vector<inter_line_positions_t> new_pos_vec;

    for (unsigned int i = 0; i < inter_line_positions_.size(); i++) {

        bool line_1_found = false;
        bool line_2_found = false;

        for (unsigned int j = 0; j < lines_.size(); j++) {

            if (lines_[j].id() == inter_line_positions_[i].line_id_1) {

                line_1_found = true;

            } else if (lines_[j].id() == inter_line_positions_[i].line_id_2) {

                line_2_found = true;

            }

            if (line_1_found && line_2_found) {

                new_pos_vec.push_back(inter_line_positions_[i]);

                break;

            }
        }

    }

    inter_line_positions_ = new_pos_vec;

    stamp_.Update();

}

void Powerline::ComputeInterLinePositions() {

    quaternion_t direction = pl_dir_history_[0];

    quaternion_t q_pl_to_drone = quatInv(direction);
    rotation_matrix_t R_pl_to_drone = quatToMat(q_pl_to_drone);

    std::unique_lock<std::shared_mutex> lines_lock(lines_mutex_);

    if (lines_.empty()) {
            
        return;

    }

    for (unsigned int i = 0; i < lines_.size()-1; i++) {

        bool in_fov = lines_[i].IsInFOV();

        if (!in_fov) {

            continue;

        }

        for (unsigned int j = i+1; j < lines_.size(); j++) {

            if (!lines_[j].IsInFOV()) {

                continue;

            }

            bool ilp_found = false;

            for (unsigned int k = 0; k < inter_line_positions_.size(); k++) {

                if (inter_line_positions_[k].line_id_1 == lines_[i].id() && inter_line_positions_[k].line_id_2 == lines_[j].id()) {

                    vector_t vec = lines_[j].position() - lines_[i].position();
                    vec = R_pl_to_drone * vec;

                    inter_line_positions_[k].inter_line_position_window.push_back(vec);

                    while(inter_line_positions_[k].inter_line_position_window.size() > (unsigned int)parameters_->GetParameter("inter_pos_window_size").as_int()) {

                        inter_line_positions_[k].inter_line_position_window.erase(inter_line_positions_[k].inter_line_position_window.begin());

                    }

                    ilp_found = true;
                    break;

                } else if (inter_line_positions_[k].line_id_1 == lines_[j].id() && inter_line_positions_[k].line_id_2 == lines_[i].id()) {

                    vector_t vec = lines_[i].position() - lines_[j].position();
                    vec = R_pl_to_drone * vec;

                    inter_line_positions_[k].inter_line_position_window.push_back(vec);

                    while(inter_line_positions_[k].inter_line_position_window.size() > (unsigned int)parameters_->GetParameter("inter_pos_window_size").as_int()) {

                        inter_line_positions_[k].inter_line_position_window.erase(inter_line_positions_[k].inter_line_position_window.begin());

                    }

                    ilp_found = true;
                    break;

                }

            }

            if (!ilp_found) {

                throw std::runtime_error("Inter line position not found.");

            }
        }
    }

    stamp_.Update();

}

void Powerline::Reset() {

    {

        std::unique_lock<std::shared_mutex> lock(lines_mutex_);

        lines_.clear();

        inter_line_positions_.clear();

    }

    pl_dir_history_.clear();
    drone_pose_history_.clear();

    projection_plane_ = createPlane(point_t(0, 0, 0), vector_t(1, 0, 0));

    id_cnt_ = 0;

    stamp_.Update();

}

const rclcpp::Time Powerline::stamp() const {

    return stamp_;

}

const quaternion_t Powerline::powerline_direction() const {

    if (pl_dir_history_.empty()) {
            
        return quaternion_t::Identity();

    } else {

        return pl_dir_history_[0];

    }

}

const plane_t Powerline::projection_plane() const {

    return projection_plane_;

}

const pose_t Powerline::drone_pose() const {

    if (drone_pose_history_.empty()) {
            
        pose_t pose;
        pose.position = point_t::Zero();
        pose.orientation = quaternion_t::Identity();
        
        return pose;

    } else {

        return drone_pose_history_[0];

    }

}

void Powerline::updateProjectionPlane() {

    quaternion_t direction_tmp = pl_dir_history_[0];

    point_t plane_p(0, 0, 0);

    vector_t unit_x(1, 0, 0);

    euler_angles_t eul = quatToEul(direction_tmp);

    vector_t plane_normal = rotateVector(eulToMat(eul), unit_x);

    projection_plane_ = createPlane(plane_p, plane_normal);

}

int Powerline::findMatchingLine(const point_t & point) const { 

    std::shared_lock<std::shared_mutex> lock(lines_mutex_);

    int best_idx = -1;
    float best_dist = std::numeric_limits<float>::infinity();

    for (unsigned int i = 0; i < lines_.size(); i++) {

        vector_t vec = (vector_t)(point - lines_[i].position());

        float dist = sqrt(vec.dot(vec));

        if (dist < parameters_->GetParameter("matching_line_max_dist").as_double() && dist < best_dist) {

            best_dist = dist;
            best_idx = i;

        }
    }

    return best_idx;

}

void Powerline::registerNewLine(const point_t & point) {

    quaternion_t pl_quat = pl_dir_history_[0];

    int new_id = id_cnt_ + 1;
    id_cnt_ = new_id;

    auto new_line = SingleLine(
        new_id,
        point,
        pl_quat,
        tf_buffer_,
        parameters_
    );

    std::unique_lock<std::shared_mutex> lines_lock(lines_mutex_);

    for (unsigned int i = 0; i < lines_.size(); i++) {

        inter_line_positions_t ilp;
        ilp.line_id_1 = lines_[i].id();
        ilp.line_id_2 = new_id;

        inter_line_positions_.push_back(ilp);

    }


    lines_.push_back(new_line);

}

const point_t Powerline::projectPoint(const point_t & point) const {

    point_t projected_point;

    projected_point = projectPointOnPlane(point, projection_plane_);

    return projected_point;

}

void Powerline::predictLines() {

    vector_t delta_position;
    quaternion_t delta_quat, q_drone_to_pl;

    pose_t drone_pose = drone_pose_history_[0];
    pose_t last_drone_pose = drone_pose_history_[-1];

    rotation_matrix_t W_R_D1 = quatToMat(last_drone_pose.orientation);
    rotation_matrix_t W_R_D2 = quatToMat(drone_pose.orientation);
    rotation_matrix_t D2_R_W = W_R_D2.transpose();

    delta_quat = matToQuat(D2_R_W*W_R_D1);

    delta_position = drone_pose.position - last_drone_pose.position;
    delta_position = D2_R_W * delta_position;

    q_drone_to_pl = pl_dir_history_[0];

    rotation_matrix_t R_drone_to_pl = quatToMat(q_drone_to_pl);

    plane_t projection_plane = projection_plane_;

    {

        std::unique_lock<std::shared_mutex> lock(lines_mutex_);

        std::vector<unsigned int> non_visible_line_indices;

        for (unsigned int i = 0; i < lines_.size(); i++) {

            if (lines_[i].IsInFOV()) {

                lines_[i].Predict(delta_position, delta_quat, projection_plane);

            } else {

                non_visible_line_indices.push_back(i);

            }
        }

        for (unsigned int i = 0; i < non_visible_line_indices.size(); i++) {

            unsigned int idx = non_visible_line_indices[i];

            std::vector<point_t> expected_positions;

            for (unsigned int j = 0; j < inter_line_positions_.size(); j++) {

                if (inter_line_positions_[j].inter_line_position_window.size() < 1)
                    continue;

                int id1 = inter_line_positions_[j].line_id_1;
                int id2 = inter_line_positions_[j].line_id_2;

                bool id1_match = id1 == lines_[idx].id();
                bool id2_match = id2 == lines_[idx].id();

                if (id1_match || id2_match) {

                    int ref_line_id = id1_match ? id2 : id1;

                    point_t reference_line_point;
                    bool ref_line_point_found = false;

                    for (unsigned int k = 0; k < lines_.size(); k++) {

                        if (k == idx)
                            continue;

                        if (lines_[k].id() == ref_line_id && lines_[k].IsInFOV()) {

                            reference_line_point = lines_[k].position();
                            ref_line_point_found = true;

                            break;

                        }
                    }

                    if (ref_line_point_found) {

                        vector_t mean_vec = inter_line_positions_[j].inter_line_position_window[0];

                        for (unsigned int k = 1; k < inter_line_positions_[j].inter_line_position_window.size(); k++) {

                            mean_vec += inter_line_positions_[j].inter_line_position_window[k];

                        }

                        mean_vec /= inter_line_positions_[j].inter_line_position_window.size();

                        float mult = id1_match ? -1. : 1.;

                        mean_vec *= mult;

                        mean_vec = R_drone_to_pl * mean_vec;

                        point_t expected_pos = reference_line_point + mean_vec;

                        expected_positions.push_back(expected_pos);

                    }
                }
            }

            if (expected_positions.size() > 0) {

                point_t mean_pos = expected_positions[0];

                for (unsigned int j = 1; j < expected_positions.size(); j++) {

                    mean_pos += expected_positions[j];

                }

                mean_pos /= expected_positions.size();

                lines_[idx].SetPosition(mean_pos);

            } else {

                lines_[idx].Predict(delta_position, delta_quat, projection_plane);

            }
        }
    }
}