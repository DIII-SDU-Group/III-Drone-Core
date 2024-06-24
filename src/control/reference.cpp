/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/reference.hpp>

using namespace iii_drone::control;
using namespace iii_drone::types;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

Reference::Reference(
    const point_t position,
    double yaw,
    const vector_t velocity,
    double yaw_rate,
    const vector_t acceleration,
    double yaw_acceleration,
    const rclcpp::Time stamp
) {
    position_ = position;
    yaw_ = yaw;
    velocity_ = velocity;
    yaw_rate_ = yaw_rate;
    acceleration_ = acceleration;
    yaw_acceleration_ = yaw_acceleration;
    stamp_ = stamp;
}

Reference::Reference() {
    position_ = point_t::Zero();
    yaw_ = 0.0;
    velocity_ = vector_t::Zero();
    yaw_rate_ = 0.0;
    acceleration_ = vector_t::Zero();
    yaw_acceleration_ = 0.0;
    stamp_ = rclcpp::Clock().now();
}

Reference::Reference(
    const State& state,
    bool nans_velocity,
    bool nans_acceleration
) {
    position_ = state.position();

    euler_angles_t euler = iii_drone::math::quatToEul(state.quaternion());

    yaw_ = euler(2);

    velocity_ = nans_velocity ? vector_t::Constant(NAN) : state.velocity();
    yaw_rate_ = nans_velocity ? NAN : 0.0;

    acceleration_ = nans_acceleration ? vector_t::Constant(NAN) : vector_t::Zero();
    yaw_acceleration_ = nans_acceleration ? NAN : 0.0;

    stamp_ = state.stamp();

}

Reference Reference::CopyWithNewStamp(const rclcpp::Time stamp) const {
    return Reference(
        position_,
        yaw_,
        velocity_,
        yaw_rate_,
        acceleration_,
        yaw_acceleration_,
        stamp
    );
}

Reference Reference::CopyWithNans(bool nan_velocity, bool nan_acceleration) const {
    return Reference(
        position_,
        yaw_,
        nan_velocity ? vector_t::Constant(NAN) : velocity_,
        nan_velocity ? NAN : yaw_rate_,
        nan_acceleration ? vector_t::Constant(NAN) : acceleration_,
        nan_acceleration ? NAN : yaw_acceleration_,
        stamp_
    );
}

const point_t Reference::position() const {
    return position_;
}

double Reference::yaw() const {
    return yaw_;
}

const vector_t Reference::velocity() const {
    return velocity_;
}

double Reference::yaw_rate() const {
    return yaw_rate_;
}

const vector_t Reference::acceleration() const {
    return acceleration_;
}

double Reference::yaw_acceleration() const {
    return yaw_acceleration_;
}

const rclcpp::Time Reference::stamp() const {
    return stamp_;
}