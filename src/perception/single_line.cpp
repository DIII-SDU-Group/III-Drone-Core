/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/perception/single_line.hpp>

using namespace iii_drone::perception;
using namespace iii_drone::types;
using namespace iii_drone::math;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

SingleLine::SingleLine(const SingleLine & other) : mutex_() {

    id_ = other.id();
    position_ = other.position();
    projected_position_ = other.projected_position();
    quaternion_ = other.quaternion();
    frame_id_ = other.frame_id();
    alive_cnt_ = other.alive_cnt();
    stamp_ = other.stamp();
    tf_buffer_ = other.tf_buffer();
    parameters_ = other.parameters();

    for (int i = 0; i < 3; i++) {

        estimates[i].state_est = other.state_est(i);
        estimates[i].var_est = other.var_est(i);

    }
}

SingleLine::SingleLine(
    const int & id, 
    const geometry_msgs::msg::Pose & pose,
    const std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
    const std::shared_ptr<PowerlineParameters> & parameters
) : tf_buffer_(tf_buffer), parameters_(parameters), mutex_() {

    id_ = id;
    position_ = pointFromPointMsg(pose.position);
    projected_position_ = position_;
    quaternion_ = quaternionFromQuaternionMsg(pose.orientation);
    frame_id_ = parameters_->drone_frame_id();

    alive_cnt_ = (parameters_->alive_cnt_low_thresh() + parameters_->alive_cnt_high_thresh()) / 2;

    resetKalmanFilter();

}

SingleLine::SingleLine(
    const int & id, 
    const point_t & position,
    const quaternion_t & quaternion,
    const std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
    const std::shared_ptr<PowerlineParameters> & parameters
) : tf_buffer_(tf_buffer), parameters_(parameters), mutex_() {

    id_ = id;
    position_ = position;
    projected_position_ = position_;
    quaternion_ = quaternion;
    frame_id_ = parameters_->drone_frame_id();

    alive_cnt_ = (parameters_->alive_cnt_low_thresh() + parameters_->alive_cnt_high_thresh()) / 2;

    resetKalmanFilter();

}

SingleLine::SingleLine(
    const int & id, 
    const point_t & position,
    const quaternion_t & quaternion,
    const std::string & frame_id,
    const std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
    const std::shared_ptr<PowerlineParameters> & parameters
) : tf_buffer_(tf_buffer), parameters_(parameters), mutex_() {

    id_ = id;
    position_ = position;
    projected_position_ = position_;
    quaternion_ = quaternion;
    frame_id_ = frame_id;

    alive_cnt_ = (parameters_->alive_cnt_low_thresh() + parameters_->alive_cnt_high_thresh()) / 2;

    resetKalmanFilter();

}

SingleLine::SingleLine(
    const iii_drone::adapters::SingleLineAdapter & adapter,
    const std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
    const std::shared_ptr<PowerlineParameters> & parameters
) : tf_buffer_(tf_buffer), parameters_(parameters), mutex_() {

    id_ = adapter.id();
    position_ = adapter.position();
    projected_position_ = adapter.projected_position();
    quaternion_ = adapter.quaternion();

    alive_cnt_ = (parameters_->alive_cnt_low_thresh() + parameters_->alive_cnt_high_thresh()) / 2;

    resetKalmanFilter();

}

const iii_drone::adapters::SingleLineAdapter SingleLine::ToAdapter() const {

    std::shared_lock<std::shared_mutex> lock(mutex_);

    iii_drone::adapters::SingleLineAdapter adapter(
        stamp_,
        frame_id_,
        id_, 
        position_, 
        projected_position_, 
        quaternion_, 
        IsInFOV()
    );

    return adapter;

}

void SingleLine::OverwriteStamp(const rclcpp::Time & stamp) {

    stamp_ = stamp;

}

bool SingleLine::IsAlive() {

    std::shared_lock<std::shared_mutex> lock(mutex_);

    if (IsInFOV() && --alive_cnt_ <= parameters_->alive_cnt_low_thresh()) {

        return false;

    } else {

        return true;

    }
}

bool SingleLine::IsVisible() const {

    std::shared_lock<std::shared_mutex> lock(mutex_);

    return alive_cnt_ >= parameters_->alive_cnt_high_thresh();

}

bool SingleLine::IsInFOV(
    const point_t & position,
    const float min_point_dist,
    const float max_point_dist,
    const float view_cone_slope
) const {

    bool in_FOV = true;

    float dist = position.norm();

    in_FOV &= dist <= max_point_dist;
    in_FOV &= dist >= min_point_dist;

    if (parameters_->simulation()) {

        float yz_dist = sqrt(position(1)*position(1)+position(2)*position(2));
        in_FOV &= position(0) > view_cone_slope*yz_dist;

    } else {

        float xz_dist = sqrt(position(0)*position(0)+position(2)*position(2));
        in_FOV &= position(1) > view_cone_slope*xz_dist;

    }

    return in_FOV;

}

bool SingleLine::IsInFOV() const {

    std::shared_lock<std::shared_mutex> lock(mutex_);

    geometry_msgs::msg::PointStamped point_stamped;
    point_stamped.header.frame_id = frame_id_;
    point_stamped.point = pointMsgFromPoint(position_);

    geometry_msgs::msg::PointStamped mmwave_point_stamped = tf_buffer_->transform(
        point_stamped, 
        parameters_->mmwave_frame_id()
    );

    point_t mmwave_point = pointFromPointMsg(mmwave_point_stamped.point);

    return IsInFOV(
        mmwave_point, 
        parameters_->min_point_dist(), 
        parameters_->max_point_dist(), 
        parameters_->view_cone_slope()
    );

}

#include <iostream>

bool SingleLine::IsInFOVStrict() const {

    std::shared_lock<std::shared_mutex> lock(mutex_);

    geometry_msgs::msg::PointStamped point_stamped;
    point_stamped.header.frame_id = frame_id_;
    point_stamped.point = pointMsgFromPoint(position_);

    geometry_msgs::msg::PointStamped mmwave_point_stamped = tf_buffer_->transform(
        point_stamped, 
        parameters_->mmwave_frame_id()
    );

    point_t mmwave_point = pointFromPointMsg(mmwave_point_stamped.point);

    return IsInFOV(
        mmwave_point, 
        parameters_->strict_min_point_dist(), 
        parameters_->strict_max_point_dist(), 
        parameters_->strict_view_cone_slope()
    );

}

void SingleLine::Update(const point_t & projected_position) {

    std::unique_lock<std::shared_mutex> lock(mutex_);

    projected_position_ = projected_position;

    for (int i = 0; i < 3; i++) {

        float y_bar = projected_position(i) - estimates[i].state_est;
        float s = estimates[i].var_est + parameters_->kf_r();

        float k = estimates[i].var_est / s;

        estimates[i].state_est += k*y_bar;
        estimates[i].var_est *= 1-k;

        position_(i) = estimates[i].state_est;

    }

    alive_cnt_ += 2;

    if (alive_cnt_ > parameters_->alive_cnt_ceiling()) {

        alive_cnt_ = parameters_->alive_cnt_ceiling();

    }

    lock.unlock();

    stamp_.Update();

}

void SingleLine::Predict(
    const vector_t & delta_drone_position, 
    const quaternion_t & delta_drone_quat, 
    const plane_t & projection_plane
) {

    std::unique_lock<std::shared_mutex> lock(mutex_);

    rotation_matrix_t R = quatToMat(delta_drone_quat);

    vector_t projected_delta_drone_position = projectPointOnPlane(
        delta_drone_position, 
        projection_plane
    );

    position_ = (R * position_) - projected_delta_drone_position;

    for (int i = 0; i < 3; i++) {

        estimates[i].state_est = position_(i);
        estimates[i].var_est += parameters_->kf_q();
    }

    lock.unlock();

    stamp_.Update();

}

void SingleLine::SetPosition(const point_t & position) {

    std::unique_lock<std::shared_mutex> lock(mutex_);

    position_ = position;

    for (int i = 0; i < 3; i++) {

        estimates[i].state_est = position_(i);
        estimates[i].var_est += parameters_->kf_q();

    }

    lock.unlock();

    stamp_.Update();

}

const point_t SingleLine::position() const {

    std::shared_lock<std::shared_mutex> lock(mutex_);

    return position_;

}

int SingleLine::id() const {

    std::shared_lock<std::shared_mutex> lock(mutex_);

    return id_;

}

rclcpp::Time SingleLine::stamp() const {

    return stamp_;

}

std::string SingleLine::frame_id() const {

    return frame_id_;

}

quaternion_t SingleLine::quaternion() const {

    std::shared_lock<std::shared_mutex> lock(mutex_);

    return quaternion_;

}

point_t SingleLine::projected_position() const {

    std::shared_lock<std::shared_mutex> lock(mutex_);

    return projected_position_;

}

std::shared_ptr<tf2_ros::Buffer> SingleLine::tf_buffer() const {

    return tf_buffer_;

}

std::shared_ptr<PowerlineParameters> SingleLine::parameters() const {

    return parameters_;

}

int SingleLine::alive_cnt() const {

    std::shared_lock<std::shared_mutex> lock(mutex_);

    return alive_cnt_;

}

float SingleLine::state_est(const int & i) const {

    std::shared_lock<std::shared_mutex> lock(mutex_);

    return estimates[i].state_est;

}

float SingleLine::var_est(const int & i) const {

    std::shared_lock<std::shared_mutex> lock(mutex_);

    return estimates[i].var_est;

}

void SingleLine::resetKalmanFilter() {

    for (int i = 0; i < 3; i++) {

        estimates[i].state_est = position_(i);
        estimates[i].var_est = 0.01;

    }

}

SingleLine & SingleLine::operator=(const SingleLine & other) {

    if (this != &other) {

        id_ = other.id();
        position_ = other.position();
        projected_position_ = other.projected_position();
        quaternion_ = other.quaternion();
        frame_id_ = other.frame_id();
        alive_cnt_ = other.alive_cnt();
        stamp_ = other.stamp();
        tf_buffer_ = other.tf_buffer();
        parameters_ = other.parameters();

        for (int i = 0; i < 3; i++) {

            estimates[i].state_est = other.state_est(i);
            estimates[i].var_est = other.var_est(i);

        }
    }

    return *this;

}