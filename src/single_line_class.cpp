/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "single_line_class.h"

/*****************************************************************************/
// Implementation
/*****************************************************************************/

SingleLine::SingleLine(int id, point_t initial_point, float r, float q, 
    rclcpp::Logger logger, int alive_cnt_low_thresh, int alive_cnt_high_thresh, int alive_cnt_ceiling,
    std::string drone_frame_id, std::string mmwave_frame_id) : logger_(logger) {

    id_ = id;

    projected_point_ = initial_point;

    pl_point_ = point_t(
        initial_point(0),
        initial_point(1),
        initial_point(2)
    );

    for (int i = 0; i < 3; i++) {

        estimates[i] = { .state_est = initial_point[i], .var_est = 0};

    }

    r_ = r;
    q_ = q;

    alive_cnt_low_thresh_ = alive_cnt_low_thresh;
    alive_cnt_high_thresh_ = alive_cnt_high_thresh;
    alive_cnt_ceiling_ = alive_cnt_ceiling;

    alive_cnt_ = (alive_cnt_low_thresh_ + alive_cnt_high_thresh_) / 2;

    drone_frame_id_ = drone_frame_id;
    mmwave_frame_id_ = mmwave_frame_id;

}

int SingleLine::GetId() {

    return id_;

}

SingleLine SingleLine::GetCopy() {

    SingleLine sl(id_, pl_point_, r_, q_, 
        logger_, alive_cnt_low_thresh_, alive_cnt_high_thresh_, alive_cnt_ceiling_, drone_frame_id_, mmwave_frame_id_);

    return sl;

}

point_t SingleLine::GetPoint() {

    return pl_point_;

}

void SingleLine::SetPoint(point_t point) {

    pl_point_ = point;
    estimates[0].state_est = point(0);
    estimates[1].state_est = point(1);
    estimates[2].state_est = point(2);

}

bool SingleLine::IsAlive(std::unique_ptr<tf2_ros::Buffer> &tf_buffer, float min_point_dist, float max_point_dist, float view_cone_slope) {

    if (IsInFOV(tf_buffer, min_point_dist, max_point_dist, view_cone_slope) && --alive_cnt_ <= alive_cnt_low_thresh_) {

        return false;

    } else {

        return true;

    }
}

bool SingleLine::IsVisible() {

    return alive_cnt_ >= alive_cnt_high_thresh_;

}

bool SingleLine::IsInFOV(point_t point, float min_point_dist, float max_point_dist, float view_cone_slope) {

    bool in_FOV = true;

    float dist = point.norm();

    in_FOV &= dist <= max_point_dist;
    in_FOV &= dist >= min_point_dist;

    float yz_dist = sqrt(point(1)*point(1)+point(2)*point(2));
    in_FOV &= point(0) > view_cone_slope*yz_dist;

    return in_FOV;

}

bool SingleLine::IsInFOV(std::unique_ptr<tf2_ros::Buffer> &tf_buffer, float min_point_dist, float max_point_dist, float view_cone_slope) {

    // //RCLCPP_INFO(logger_, "b1");

    geometry_msgs::msg::PointStamped point_stamped;
    point_stamped.header.frame_id = drone_frame_id_;
    point_stamped.point.x = pl_point_(0);
    point_stamped.point.y = pl_point_(1);
    point_stamped.point.z = pl_point_(2);

    // //RCLCPP_INFO(logger_, "b2");

    geometry_msgs::msg::PointStamped mmwave_point_stamped = tf_buffer->transform(point_stamped, mmwave_frame_id_);

    // //RCLCPP_INFO(logger_, "b3");

    point_t mmwave_point(
        mmwave_point_stamped.point.x,
        mmwave_point_stamped.point.y,
        mmwave_point_stamped.point.z
    );

    // //RCLCPP_INFO(logger_, "b4");

    // return IsInFOV(pl_point_, min_point_dist, max_point_dist, view_cone_slope);
    return IsInFOV(mmwave_point, min_point_dist, max_point_dist, view_cone_slope);

}

void SingleLine::Update(point_t point) {

    projected_point_ = point;

    for (int i = 0; i < 3; i++) {

        float y_bar = point(i) - estimates[i].state_est;
        float s = estimates[i].var_est + r_;

        float k = estimates[i].var_est / s;

        estimates[i].state_est += k*y_bar;
        estimates[i].var_est *= 1-k;

        pl_point_(i) = estimates[i].state_est;

    }

    alive_cnt_ += 2;

    if (alive_cnt_ > alive_cnt_ceiling_) {

        alive_cnt_ = alive_cnt_ceiling_;

    }

}

void SingleLine::Predict(vector_t delta_position, quat_t delta_quat, plane_t projection_plane, std::unique_ptr<tf2_ros::Buffer> &tf_buffer) {

    //RCLCPP_INFO(logger_, "Predicting line");

    orientation_t eul = quatToEul(delta_quat);

    // eul(0) = 0;
    // eul(1) = 0;
    // eul(2) = 0;

    rotation_matrix_t R = eulToR(eul);
    // rotation_matrix_t R = quatToMat(delta_quat);

    //RCLCPP_INFO(logger_, "Point before: (%f, %f, %f)", pl_point_(0), pl_point_(1), pl_point_(2));

    delta_position = projectPointOnPlane(delta_position, projection_plane);
    // delta_position(0) = 0;
    // delta_position(1) = 0;

    pl_point_ = (R * pl_point_) - delta_position;
    // pl_point_ = (R.transpose() * pl_point_) + delta_position;

    // pl_point_ = projectPointOnPlane(pl_point_, projection_plane);

    // //RCLCPP_INFO(logger_, "Delta position: (%f, %f, %f)", delta_position(0), delta_position(1), delta_position(2));

    //RCLCPP_INFO(logger_, "Point after: (%f, %f, %f)", pl_point_(0), pl_point_(1), pl_point_(2));

    for (int i = 0; i < 3; i++) {

        estimates[i].state_est = pl_point_(i);
        estimates[i].var_est += q_;
    }

}