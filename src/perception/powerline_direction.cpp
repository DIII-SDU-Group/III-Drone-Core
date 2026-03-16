/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "iii_drone_core/perception/powerline_direction.hpp"

using namespace iii_drone::perception;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

PowerlineDirection::PowerlineDirection(iii_drone::configuration::Configuration::SharedPtr parameters) 
    : configuration_(parameters),
    drone_quat_history_(2),
    pl_quat_history_(1) {

    resetKalmanFilter();

    stamp_.Update();

}

PowerlineDirection::~PowerlineDirection() { }

const geometry_msgs::msg::PoseStamped PowerlineDirection::ToPoseStampedMsg(const std::string & drone_frame_id) const {

    geometry_msgs::msg::PoseStamped msg;

    msg.header.stamp = (rclcpp::Time)stamp_;
    msg.header.frame_id = drone_frame_id;

    iii_drone::types::quaternion_t pl_quat;

    if (pl_quat_history_.empty()) {

        pl_quat = iii_drone::types::quaternion_t::Identity();

    } else {

        pl_quat = pl_quat_history_[0];

    }

    msg.pose = iii_drone::types::poseMsgFromPose(
        iii_drone::types::point_t::Zero(),
        pl_quat
    );

    return msg;

}

const geometry_msgs::msg::QuaternionStamped PowerlineDirection::ToQuaternionStampedMsg(const std::string & drone_frame_id) const {

    geometry_msgs::msg::QuaternionStamped msg;

    msg.header.stamp = (rclcpp::Time)stamp_;
    msg.header.frame_id = drone_frame_id;

    if (pl_quat_history_.empty()) {

        msg.quaternion = iii_drone::types::quaternionMsgFromQuaternion(iii_drone::types::quaternion_t::Identity());

    } else {

        msg.quaternion = iii_drone::types::quaternionMsgFromQuaternion(pl_quat_history_[0]);

    }

    return msg;

}

void PowerlineDirection::Update(float pl_yaw) {

    using namespace iii_drone::types;
    using namespace iii_drone::math;

    if (drone_quat_history_.empty()) {

        return;

    }

    quaternion_t drone_quat = drone_quat_history_[0];

    if (pl_quat_history_.empty()) {

        euler_angles_t pl_eul = computePowerlineEulerAngles(pl_yaw, drone_quat);
        quaternion_t pl_quat = eulToQuat(pl_eul);

        pl_quat_history_ = pl_quat;

        resetKalmanFilter();

        stamp_.Update();

        return;

    }

    if (!anyCableInFOV()) {

        return;

    }


    euler_angles_t D_eul_P = computePowerlineEulerAngles(pl_yaw, drone_quat);

    for (int i = 0; i < 3; i++) {

        float angle = false ? mapAngle2(pl_angle_est_[i].state_est, D_eul_P(i)) : D_eul_P(i);

        // if (!received_angle_) {

        //     pl_angle_est_[i].state_est = backmapAngle(angle);

        //     if (i==2) {

        //         received_angle_ = true;

        //     }

        // } else {

            float y_bar = angle - pl_angle_est_[i].state_est;
            float s = pl_angle_est_[i].var_est + configuration_->GetParameter("/perception/pl_dir_computer/kf_r").as_double();

            float k = pl_angle_est_[i].var_est / s;

            pl_angle_est_[i].state_est += k*y_bar;
            pl_angle_est_[i].var_est *= 1-k;

            pl_angle_est_[i].state_est = backmapAngle(pl_angle_est_[i].state_est);

        // }

        D_eul_P(i) = pl_angle_est_[i].state_est;

        pl_quat_history_ = eulToQuat(D_eul_P);

    }

    stamp_.Update();

}

void PowerlineDirection::Predict(const iii_drone::types::quaternion_t & drone_quat) {

    using namespace iii_drone::types;
    using namespace iii_drone::math;

    drone_quat_history_ = drone_quat;

    if (!drone_quat_history_.full()) {

        return;

    }

    if (!pl_quat_history_.full()) {

        return;

    }

    quaternion_t inv_last_drone_quat = quatInv(drone_quat_history_[-1]);
    quaternion_t delta_drone_quat = quatMultiply(drone_quat, inv_last_drone_quat); /// Works!!!!! #impericalmethod

    quaternion_t pl_quat = pl_quat_history_[0];

    pl_quat = quatMultiply(
        delta_drone_quat, 
        pl_quat
    );   //// WORKS!!! #impericalmethodAKAnoideawhyitworks

    pl_quat_history_ = pl_quat;

    euler_angles_t eul = quatToEul(pl_quat);

    for (int i = 0; i < 3; i++) { 
        
        pl_angle_est_[i].var_est += configuration_->GetParameter("/perception/pl_dir_computer/kf_q").as_double(); 
        pl_angle_est_[i].state_est = backmapAngle(eul(i)); 
        
    }

    stamp_.Update();

}

void PowerlineDirection::UpdatePowerlineAdapter(const iii_drone_interfaces::msg::Powerline::SharedPtr msg) {

    std::unique_lock<std::shared_mutex> lock(pl_mutex_);

    pl_.UpdateFromMsg(*msg);

}

bool PowerlineDirection::anyCableInFOV() const {

    std::shared_lock<std::shared_mutex> lock(pl_mutex_);

    if (pl_.single_line_adapters().size() == 0) {

        return true;

    }

    if (pl_.GetVisibleLineAdapters().size() == 0) {

        return false;

    }

    return true;

}

const iii_drone::types::quaternion_t PowerlineDirection::quaternion() const {

    if (pl_quat_history_.empty()) {

        return iii_drone::types::quaternion_t::Identity();

    }

    return pl_quat_history_[0];

}

const rclcpp::Time PowerlineDirection::stamp() const {

    return stamp_;

}

const iii_drone::types::euler_angles_t PowerlineDirection::computePowerlineEulerAngles(
    const float & pl_yaw,
    const iii_drone::types::quaternion_t & drone_quat
) {

    using namespace iii_drone::types;
    using namespace iii_drone::math;

    static float W_pl_yaw_ = 0;

    quaternion_t W_Q_D = drone_quat;

    rotation_matrix_t W_R_D = quatToMat(W_Q_D);

    vector_t unit_x(1,0,0);
    euler_angles_t pl_eul(0,0,pl_yaw);

    vector_t D_v = eulToMat(pl_eul) * unit_x;

    vector_t W_v(
        -(D_v(1)*W_R_D(0,1) - D_v(0)*W_R_D(1,1))/(W_R_D(0,0)*W_R_D(1,1) - W_R_D(0,1)*W_R_D(1,0)),
        (D_v(1)*W_R_D(0,0) - D_v(0)*W_R_D(1,0))/(W_R_D(0,0)*W_R_D(1,1) - W_R_D(0,1)*W_R_D(1,0)),
        0
    );

    float pl_angle = atan2(W_v[1], W_v[0]);
    pl_angle = mapAngle(W_pl_yaw_, pl_angle);
    W_pl_yaw_ = pl_angle;

    euler_angles_t W_pl_eul(0,0,pl_angle);
    W_v = eulToMat(W_pl_eul) * unit_x;

    W_v /= W_v.norm();

    euler_angles_t pi_2_yaw(0,0,M_PI_2);

    vector_t W_P_x = W_v;
    vector_t W_P_y = eulToMat(pi_2_yaw)*W_P_x;
    vector_t W_P_z(0,0,1);

    rotation_matrix_t W_R_P;

    for (int i = 0; i < 3; i++) {

        W_R_P(i,0) = W_P_x(i);
        W_R_P(i,1) = W_P_y(i);
        W_R_P(i,2) = W_P_z(i);

    }

    rotation_matrix_t D_R_P = W_R_D * W_R_P;

    quaternion_t D_Q_P = matToQuat(D_R_P);

    euler_angles_t D_eul_P = quatToEul(D_Q_P);

    return D_eul_P;

}

float PowerlineDirection::mapAngle(
    float curr_angle, 
    float new_angle
) const {

    float angle_candidates[5];
    angle_candidates[0] = new_angle;

    if (new_angle > 0) {

        angle_candidates[1] = new_angle - M_PI;
        angle_candidates[2] = new_angle - 2*M_PI;
        angle_candidates[3] = new_angle + M_PI;
        angle_candidates[4] = new_angle + 2*M_PI;

    } else {

        angle_candidates[1] = new_angle + M_PI;
        angle_candidates[2] = new_angle + 2*M_PI;
        angle_candidates[3] = new_angle - M_PI;
        angle_candidates[4] = new_angle - 2*M_PI;

    }

    float best_angle = angle_candidates[0];
    float best_angle_diff = abs(angle_candidates[0]-curr_angle);

    for (int i = 0; i < 5; i++) {

        float diff = abs(angle_candidates[i]-curr_angle);
        if (diff < best_angle_diff) {
            best_angle_diff = diff;
            best_angle = angle_candidates[i];
        }

    }

    return best_angle;

}

float PowerlineDirection::mapAngle2(
    float curr_angle, 
    float new_angle
) const {

    float angle_candidates[3];
    angle_candidates[0] = new_angle;
    angle_candidates[1] = new_angle + M_PI;
    angle_candidates[2] = new_angle - M_PI;

    float best_angle = angle_candidates[0];
    float best_angle_diff = abs(angle_candidates[0]-curr_angle);

    for (int i = 0; i < 3; i++) {

        float diff = abs(angle_candidates[i]-curr_angle);
        if (diff < best_angle_diff) {
            best_angle_diff = diff;
            best_angle = angle_candidates[i];
        }

    }

    return best_angle;

}

float PowerlineDirection::backmapAngle(float angle) const {

    if (angle > M_PI) {
        return angle-2*M_PI;
    } else if (angle < -M_PI) {
        return angle+2*M_PI;
    } else {
        return angle;
    }

}


void PowerlineDirection::resetKalmanFilter() {

    iii_drone::types::euler_angles_t pl_eul;
    
    if (pl_quat_history_.empty()) {

        pl_eul = iii_drone::types::euler_angles_t(0,0,0);
    
    } else {
    
        pl_eul = iii_drone::math::quatToEul(pl_quat_history_[0]);
    
    }

    pl_angle_est_[0].state_est = pl_eul(0);
    pl_angle_est_[0].var_est = 0.1;

    pl_angle_est_[1].state_est = pl_eul(1);
    pl_angle_est_[1].var_est = 0.1;

    pl_angle_est_[2].state_est = pl_eul(2);
    pl_angle_est_[2].var_est = 0.1;

}
