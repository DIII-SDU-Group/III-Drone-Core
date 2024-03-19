/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/state.hpp>

using namespace iii_drone::control;
using namespace iii_drone::types;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

State::State(
    const point_t position,
    const vector_t velocity,
    const quaternion_t quaternion,
    const vector_t angular_velocity,
    const rclcpp::Time stamp
) {

    position_ = position;
    velocity_ = velocity;
    quaternion_ = quaternion;
    euler_angles_t eul = iii_drone::math::quatToEul(quaternion);
    yaw_ = eul(2);
    angular_velocity_ = angular_velocity;
    stamp_ = stamp;

}

State::State(
    const point_t position,
    const vector_t velocity,
    double yaw,
    const vector_t angular_velocity,
    const rclcpp::Time stamp
) {

    position_ = position;
    velocity_ = velocity;
    yaw_ = yaw;
    quaternion_ = iii_drone::math::eulToQuat(euler_angles_t(0, 0, yaw));
    angular_velocity_ = angular_velocity;
    stamp_ = stamp;

}

State::State() {

    position_ = point_t::Zero();
    velocity_ = vector_t::Zero();
    quaternion_ = quaternion_t::Identity();
    angular_velocity_ = vector_t::Zero();
    stamp_ = rclcpp::Clock().now();

}

const point_t State::position() const {
    return position_;
}

const vector_t State::velocity() const {
    return velocity_;
}

const quaternion_t State::quaternion() const {
    return quaternion_;
}

double State::yaw() const {
    return yaw_;
}

const vector_t State::angular_velocity() const {
    return angular_velocity_;
}

const rclcpp::Time State::stamp() const {
    return stamp_;
}