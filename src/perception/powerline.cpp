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
    rclcpp::Logger logger, 
    bool simulation
) : logger_(logger) {

    simulation_ = simulation;

    direction_ = quat_t(1,0,0,0);
    position_ = point_t(0,0,0);
    quat_ = quat_t(0, 0, 0, 0);
    last_position_ = point_t(0, 0, 0);
    last_quat_ = quat_t(0, 0, 0, 0);
    projection_plane_ = { .p=point_t(0,0,0), .normal=vector_t(0,0,0) };

    id_cnt_ = 0;

}

void Powerline::SetParams(
    float r, 
    float q, 
    int alive_cnt_low_thresh, 
    int alive_cnt_high_thresh, 
    int alive_cnt_ceiling, 
    float matching_line_max_dist,
    std::string drone_frame_id, 
    std::string mmwave_frame_id, 
    int max_lines
) {

    max_lines_ = max_lines;

    r_ = r;
    q_ = q;

    alive_cnt_low_thresh_ = alive_cnt_low_thresh;
    alive_cnt_high_thresh_ = alive_cnt_high_thresh;
    alive_cnt_ceiling_ = alive_cnt_ceiling;

    matching_line_max_dist_ = matching_line_max_dist;

    drone_frame_id_ = drone_frame_id;
    mmwave_frame_id_ = mmwave_frame_id;

}

std::vector<SingleLine> Powerline::GetVisibleLines() {

    std::vector<SingleLine> ret_lines;

    lines_mutex_.lock(); {

        for (int i = 0; i < lines_.size(); i++) {

            if (lines_[i].IsVisible()) {

                ret_lines.push_back(lines_[i].GetCopy());

            }

        }

    } lines_mutex_.unlock();

    return ret_lines;

}

quat_t Powerline::GetDirection() {

    quat_t direction;

    direction_mutex_.lock(); {

        direction = direction_;

    } direction_mutex_.unlock();

    return direction;

}

plane_t Powerline::GetProjectionPlane() {

    plane_t plane;

    projection_plane_mutex_.lock(); {

        plane = projection_plane_;

    } projection_plane_mutex_.unlock();

    return plane;

}

quat_t Powerline::GetQuat() {

    quat_t quat;

    odometry_mutex_.lock(); {

        quat = quat_;

    } odometry_mutex_.unlock();

    return quat;

}

point_t Powerline::UpdateLine(point_t point) {

    if (!received_pl_dir || !received_first_odom || !received_second_odom) {

        return point;

    }

    //RCLCPP_INFO(logger_, "Updating line");

    point_t projected_point = projectPoint(point);

    //RCLCPP_INFO(logger_, "Point: [%f, %f, %f] \t projected point: [%f, %f, %f]", point(0), point(1), point(2), projected_point(0), projected_point(1), projected_point(2));

    int match_index = findMatchingLine(projected_point);

    if (match_index == -1 && lines_.size() < max_lines_) {

        //RCLCPP_INFO(logger_, "No matching line found");

        lines_mutex_.lock(); {

            int new_id = id_cnt_++;

            //RCLCPP_INFO(logger_, "Creating new line");

            auto new_line = SingleLine(new_id, projected_point, r_, q_, 
                    logger_, alive_cnt_low_thresh_, alive_cnt_high_thresh_, alive_cnt_ceiling_,
                    drone_frame_id_, mmwave_frame_id_, simulation_);

            for (int i = 0; i < lines_.size(); i++) {
                // //RCLCPP_INFO(logger_, "Creating inter line position");

                inter_line_positions_t ilp;
                ilp.line_id_1 = lines_[i].GetId();
                ilp.line_id_2 = new_id;

                inter_line_positions_.push_back(ilp);

            }


            //RCLCPP_INFO(logger_, "Pushing back new line");
            lines_.push_back(new_line);

        } lines_mutex_.unlock();

    } else {

        lines_mutex_.lock(); {

            //RCLCPP_INFO(logger_, "Found matching line, updating the line");

            lines_[match_index].Update(projected_point);

        } lines_mutex_.unlock();

    }

    return projected_point;

}

void Powerline::UpdateDirection(quat_t pl_direction) {

    direction_mutex_.lock(); {
        direction_ = pl_direction;
    } direction_mutex_.unlock();

    if (!received_first_odom || !received_second_odom) {

        return;

    }

    //quat_t quat;

    //odometry_mutex_.lock(); {

    //    quat = quat_;

    //} odometry_mutex_.unlock();

    //direction_mutex_.lock(); {

    //    float direction2 = (direction > 0) ? direction - M_PI : direction + M_PI;
    //    float new_direction = (abs(direction-direction_) < abs(direction2-direction_)) ? direction : direction2;

    //    orientation_t yaw_eul = quatToEul(quat);

    //    float new_global_input_direction = new_direction - yaw_eul(2);

    //    float new_global_output_direction = 0.1 * new_global_input_direction 
    //            + 0.45 * last_global_output_direction_ + 0.45 * last_last_global_output_direction_;

    //    last_global_input_direction_ = new_global_input_direction;
    //    last_last_global_output_direction_ = last_global_output_direction_;
    //    last_global_output_direction_ = new_global_output_direction;

    //    direction_ = new_global_output_direction + yaw_eul(2);

    //} direction_mutex_.unlock();

    updateProjectionPlane();

    received_pl_dir = true;

}

void Powerline::UpdateOdometry(
    point_t position, 
    quat_t quat, 
    std::unique_ptr<tf2_ros::Buffer> &tf_buffer,
    float min_point_dist, 
    float max_point_dist, 
    float view_cone_slope
) {

    //RCLCPP_INFO(logger_, "Updating odometry");

    odometry_mutex_.lock(); {

        last_quat_ = quat_;
        last_position_ = position_;

        quat_ = quat;
        position_ = position;

    } odometry_mutex_.unlock();

    if (!received_first_odom) {

        received_first_odom = true;

        return;

    }

    if (!received_second_odom) {


        received_second_odom = true;

        return;
    }

    if (!received_pl_dir) {

        return;

    }

    // updateProjectionPlane();

    predictLines(tf_buffer, min_point_dist, max_point_dist, view_cone_slope);

}

void Powerline::CleanupLines(
    std::unique_ptr<tf2_ros::Buffer> &tf_buffer, 
    float min_point_dist, 
    float max_point_dist, 
    float view_cone_slope
) {

    //RCLCPP_INFO(logger_, "Cleaning up lines");

    lines_mutex_.lock(); {

        std::vector<SingleLine> new_vec;

        // //RCLCPP_INFO(logger_, "Looping through all lines");

        for (int i = 0; i < lines_.size(); i++) {

            // //RCLCPP_INFO(logger_, "At line number %d", i);

            if (lines_[i].IsAlive(tf_buffer, min_point_dist, max_point_dist, view_cone_slope)) {

                // //RCLCPP_INFO(logger_, "Line is alive, pushing back to new vector");

                new_vec.push_back(lines_[i]);

            }
        }

        // //RCLCPP_INFO(logger_, "Assigning new_vec to lines_");
        lines_ = new_vec;

        std::vector<inter_line_positions_t> new_pos_vec;

        // //RCLCPP_INFO(logger_, "Looping through inter line positions");

        for (int i = 0; i < inter_line_positions_.size(); i++) {

            // //RCLCPP_INFO(logger_, "at ilp number %d", i);

            bool line_1_found = false;
            bool line_2_found = false;

            for (int j = 0; j < lines_.size(); j++) {

                // //RCLCPP_INFO(logger_, "At line number %d", j);

                if (lines_[j].GetId() == inter_line_positions_[i].line_id_1) {

                    // //RCLCPP_INFO(logger_, "Found line 1 id match");

                    line_1_found = true;

                } else if (lines_[j].GetId() == inter_line_positions_[i].line_id_2) {

                    // //RCLCPP_INFO(logger_, "Found line 2 id match");

                    line_2_found = true;

                }

                if (line_1_found && line_2_found) {

                    // //RCLCPP_INFO(logger_, "Both lines are matched, pushing back and breaking");

                    new_pos_vec.push_back(inter_line_positions_[i]);

                    break;

                }
            }

        }

        // //RCLCPP_INFO(logger_, "Assigning new_pos_vec to inter_line_positions_");

        inter_line_positions_ = new_pos_vec;

    } lines_mutex_.unlock();

    // //RCLCPP_INFO(logger_, "New lines_ length: %d \t New ILP length: %d", lines_.size(), inter_line_positions_.size());

}

void Powerline::ComputeInterLinePositions(
    std::unique_ptr<tf2_ros::Buffer> &tf_buffer, 
    float min_point_dist, 
    float max_point_dist, 
    float view_cone_slope, 
    int inter_pos_window_size
) {

    //RCLCPP_INFO(logger_, "Computing inter line positions");

    quat_t direction;

    direction_mutex_.lock(); {

        direction = direction_;

    } direction_mutex_.unlock();

    // //RCLCPP_INFO(logger_, "a2");

    quat_t q_pl_to_drone = quatInv(direction);
    rotation_matrix_t R_pl_to_drone = quatToMat(q_pl_to_drone);

    lines_mutex_.lock(); {

        // //RCLCPP_INFO(logger_, "a3");

        // //RCLCPP_INFO(logger_, "Looping through lines");

        for (int i = 0; i < ((int)lines_.size())-1; i++) {

            // //RCLCPP_INFO(logger_, "At line number %d", i);

            // //RCLCPP_INFO(logger_, "a4");
            // //RCLCPP_INFO(logger_, "a4aaaaa");
            // //RCLCPP_INFO(logger_, "%d - %d", i, lines_.size()-1);

            // //RCLCPP_INFO(logger_, "a4a");

            if (!lines_[i].IsInFOV(tf_buffer, min_point_dist, max_point_dist, view_cone_slope)) {

                // //RCLCPP_INFO(logger_, "Line is not in FOV, not computing inter line position for line");

                // //RCLCPP_INFO(logger_, "a5");

                continue;

            }

            // //RCLCPP_INFO(logger_, "Line is in FOV, going through other lines");

            for (int j = i+1; j < lines_.size(); j++) {

                // //RCLCPP_INFO(logger_, "At second line number %d", j);

                // //RCLCPP_INFO(logger_, "a6");

                if (!lines_[j].IsInFOV(tf_buffer, min_point_dist, max_point_dist, view_cone_slope)) {

                    // //RCLCPP_INFO(logger_, "Second line is not in FOV, not computing inter line distance between the lines");

                    continue;

                }

                // //RCLCPP_INFO(logger_, "Second line is in FOV");

                bool ilp_found = false;

                // //RCLCPP_INFO(logger_, "Attempting to find matching ILP, going through ILPs");

                for (int k = 0; k < inter_line_positions_.size(); k++) {

                    // //RCLCPP_INFO(logger_, "At ilp number %d", k);

                    // //RCLCPP_INFO(logger_, "a7");

                    if (inter_line_positions_[k].line_id_1 == lines_[i].GetId() && inter_line_positions_[k].line_id_2 == lines_[j].GetId()) {

                        // //RCLCPP_INFO(logger_, "ILP matches lines 1-2, computing distance");

                        // //RCLCPP_INFO(logger_, "a8");

                        vector_t vec = lines_[j].GetPoint() - lines_[i].GetPoint();
                        vec = R_pl_to_drone * vec;

                        // //RCLCPP_INFO(logger_, "Pushing back new ilp");

                        inter_line_positions_[k].inter_line_position_window.push_back(vec);

                        while(inter_line_positions_[k].inter_line_position_window.size() > inter_pos_window_size) {

                            // //RCLCPP_INFO(logger_, "ILP larger than window, removing element");

                            // //RCLCPP_INFO(logger_, "a9");

                            inter_line_positions_[k].inter_line_position_window.erase(inter_line_positions_[k].inter_line_position_window.begin());

                        }

                        ilp_found = true;
                        break;

                    } else if (inter_line_positions_[k].line_id_1 == lines_[j].GetId() && inter_line_positions_[k].line_id_2 == lines_[i].GetId()) {

                        // //RCLCPP_INFO(logger_, "ILP matches lines 2-1, computing distance");

                        // //RCLCPP_INFO(logger_, "a10");

                        vector_t vec = lines_[i].GetPoint() - lines_[j].GetPoint();
                        vec = R_pl_to_drone * vec;

                        // //RCLCPP_INFO(logger_, "Pushing back new ilp");

                        inter_line_positions_[k].inter_line_position_window.push_back(vec);

                        while(inter_line_positions_[k].inter_line_position_window.size() > inter_pos_window_size) {

                            // //RCLCPP_INFO(logger_, "ILP larger than window, removing element");

                            // //RCLCPP_INFO(logger_, "a11");

                            inter_line_positions_[k].inter_line_position_window.erase(inter_line_positions_[k].inter_line_position_window.begin());

                        }

                        ilp_found = true;
                        break;

                    }

                    // //RCLCPP_INFO(logger_, "ilp didn't match the lines");

                }

                if (!ilp_found) {

                    RCLCPP_FATAL(logger_, "Could not find matchin ILP entry, fatal");

                }
            }
        }

    } lines_mutex_.unlock();

}

int Powerline::GetLinesCount() {

    int cnt;

    lines_mutex_.lock(); {

        cnt = lines_.size();

    } lines_mutex_.unlock();

    return cnt;

}

void Powerline::updateProjectionPlane() {

    // //RCLCPP_INFO(logger_, "Updating projection plane");

    quat_t direction_tmp;
    //quat_t quat_tmp;

    direction_mutex_.lock(); {

        direction_tmp = direction_;

    } direction_mutex_.unlock();

    //odometry_mutex_.lock(); {

    //    quat_tmp = quat_t(quat_[0],quat_[1],quat_[2],quat_[3]);

    //} odometry_mutex_.unlock();

    point_t plane_p(0, 0, 0);

    vector_t unit_x(1, 0, 0);

    orientation_t eul = quatToEul(direction_tmp);

    //orientation_t rotation(0, -eul[1], direction_tmp);

    vector_t plane_normal = rotateVector(eulToR(eul), unit_x);

    projection_plane_mutex_.lock(); {

        projection_plane_ = {

            .p = plane_p,
            .normal = plane_normal

        };

        //plane_orientation_ = rotation;

    } projection_plane_mutex_.unlock();

}

point_t Powerline::projectPoint(point_t point) {

    point_t projected_point;

    projection_plane_mutex_.lock(); {

        projected_point = projectPointOnPlane(point, projection_plane_);

    } projection_plane_mutex_.unlock();

    return projected_point;

}

int Powerline::findMatchingLine(point_t point) { 

    int best_idx = -1;
    float best_dist = std::numeric_limits<float>::infinity();

    //RCLCPP_INFO(logger_, "Trying to find matching line to point [%f, %f, %f]", point(0), point(1), point(2));

    lines_mutex_.lock(); {
        
        for (int i = 0; i < lines_.size(); i++) {

            //RCLCPP_INFO(logger_, "At line number %d [%f, %f, %f]", i, lines_[i].GetPoint()(0), lines_[i].GetPoint()(1), lines_[i].GetPoint()(2));

            vector_t vec = (vector_t)(point - lines_[i].GetPoint());

            float dist = sqrt(vec.dot(vec));

            //RCLCPP_INFO(logger_, "Distance between point and line is %f", dist);

            if (dist < matching_line_max_dist_ && dist < best_dist) {

                //RCLCPP_INFO(logger_, "Found line candidate");

                best_dist = dist;
                best_idx = i;

            }
        }

    } lines_mutex_.unlock();

    return best_idx;

}

void Powerline::predictLines(
    std::unique_ptr<tf2_ros::Buffer> &tf_buffer, 
    float min_point_dist, 
    float max_point_dist, 
    float view_cone_slope
) {

    // //RCLCPP_INFO(logger_, "Predicting lines");

    vector_t delta_position;
    quat_t delta_quat, q_drone_to_pl;

    odometry_mutex_.lock(); {

        quat_t inv_last_quat = quatInv(last_quat_);
        quat_t inv_quat = quatInv(quat_);

        rotation_matrix_t W_R_D1 = quatToMat(last_quat_);
        rotation_matrix_t W_R_D2 = quatToMat(quat_);
        rotation_matrix_t D2_R_W = W_R_D2.transpose();

        delta_quat = matToQuat(D2_R_W*W_R_D1);

        // RCLCPP_INFO(logger_, "position: [%f,%f,%f]", position_(0), position_(1), position_(2));
        // RCLCPP_INFO(logger_, "last position: [%f,%f,%f]", last_position_(0), last_position_(1), last_position_(2));
        delta_position = position_ - last_position_;
        // RCLCPP_INFO(logger_, "delta position: [%f,%f,%f]", delta_position(0), delta_position(1), delta_position(2));
        delta_position = D2_R_W * delta_position;
        // RCLCPP_INFO(logger_, "W delta position: [%f,%f,%f]", delta_position(0), delta_position(1), delta_position(2));
        // RCLCPP_INFO(logger_, "\n\n");

        // delta_quat = quatMultiply(last_quat_, quat_);
        // delta_quat = quatMultiply(inv_last_quat, quat_);
        // delta_quat = quatMultiply(last_quat_, inv_quat);
        // delta_quat = quatMultiply(inv_last_quat, inv_quat);
        // delta_quat = quatMultiply(quat_, last_quat_);
        // delta_quat = quatMultiply(quat_, inv_last_quat);
        // delta_quat = quatMultiply(inv_quat, last_quat_);
        // delta_quat = quatMultiply(inv_quat, inv_last_quat);

    } odometry_mutex_.unlock();

    direction_mutex_.lock(); {

        q_drone_to_pl = direction_;

    } direction_mutex_.unlock();

    rotation_matrix_t R_drone_to_pl = quatToMat(q_drone_to_pl);

    plane_t projection_plane;

    projection_plane_mutex_.lock(); {

        projection_plane = projection_plane_;

    } projection_plane_mutex_.unlock();

    lines_mutex_.lock(); {

        // //RCLCPP_INFO(logger_, "Going through %d lines", lines_.size());

        std::vector<int> non_visible_line_indices;

        for (int i = 0; i < lines_.size(); i++) {

            // //RCLCPP_INFO(logger_, "At line number %d", i);

            if (lines_[i].IsInFOV(tf_buffer, min_point_dist, max_point_dist, view_cone_slope)) {

                // RCLCPP_INFO(logger_, "Line %d is in FOV, predicting", i);

                lines_[i].Predict(delta_position, delta_quat, projection_plane, tf_buffer);

            } else {

                // RCLCPP_INFO(logger_, "Line %d is not in FOV, putting index into non-visible indices list", i);

                non_visible_line_indices.push_back(i);

            }
        }

        //RCLCPP_INFO(logger_, "Going through %d non-visible indices", non_visible_line_indices.size());

        for (int i = 0; i < non_visible_line_indices.size(); i++) {

            int idx = non_visible_line_indices[i];

            std::vector<point_t> expected_positions;

            for (int j = 0; j < inter_line_positions_.size(); j++) {

                if (inter_line_positions_[j].inter_line_position_window.size() < 1)
                    continue;

                int id1 = inter_line_positions_[j].line_id_1;
                int id2 = inter_line_positions_[j].line_id_2;

                bool id1_match = id1 == lines_[idx].GetId();
                bool id2_match = id2 == lines_[idx].GetId();

                if (id1_match || id2_match) {

                    int ref_line_id = id1_match ? id2 : id1;

                    point_t reference_line_point;
                    bool ref_line_point_found = false;

                    for (int k = 0; k < lines_.size(); k++) {

                        if (k == idx)
                            continue;

                        if (lines_[k].GetId() == ref_line_id && lines_[k].IsInFOV(tf_buffer, min_point_dist, max_point_dist, view_cone_slope)) {

                            reference_line_point = lines_[k].GetPoint();
                            ref_line_point_found = true;

                            break;

                        }
                    }

                    if (ref_line_point_found) {

                        vector_t mean_vec = inter_line_positions_[j].inter_line_position_window[0];

                        for (int k = 1; k < inter_line_positions_[j].inter_line_position_window.size(); k++) {

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

                for (int j = 1; j < expected_positions.size(); j++) {

                    mean_pos += expected_positions[j];

                }

                mean_pos /= expected_positions.size();

                lines_[idx].SetPoint(mean_pos);

            } else {

                lines_[idx].Predict(delta_position, delta_quat, projection_plane, tf_buffer);

            }
        }

        //RCLCPP_INFO(logger_, "lines_.size(): %d", lines_.size());

    } lines_mutex_.unlock();

}

void Powerline::SetSimulation(bool simulation) {

    if (lines_.size() > 0) {

        RCLCPP_FATAL(logger_, "Cannot change simulation mode after lines have been created");
        
        exit(1);

    }

    simulation_ = simulation;

}