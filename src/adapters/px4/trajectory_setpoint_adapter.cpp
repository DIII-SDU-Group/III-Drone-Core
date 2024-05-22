#include <iii_drone_core/adapters/px4/trajectory_setpoint_adapter.hpp>

#include <iostream>

using namespace iii_drone::adapters::px4;
using namespace iii_drone::control;
using namespace iii_drone::types;
using namespace iii_drone::math;

TrajectorySetpointAdapter::TrajectorySetpointAdapter() { 

    init(
        rclcpp::Time(0, 0, RCL_ROS_TIME),
        {NAN, NAN, NAN},
        {NAN, NAN, NAN},
        {NAN, NAN, NAN},
        NAN,
        NAN
    );

}

TrajectorySetpointAdapter::TrajectorySetpointAdapter(const rclcpp::Time & stamp) {

    init(
        stamp,
        {NAN, NAN, NAN},
        {NAN, NAN, NAN},
        {NAN, NAN, NAN},
        NAN,
        NAN
    );

}

TrajectorySetpointAdapter::TrajectorySetpointAdapter(const Reference & reference) {

    init(
        reference.stamp(),
        reference.position(),
        reference.velocity(),
        reference.acceleration(),
        reference.yaw(),
        reference.yaw_rate()
    );

}

TrajectorySetpointAdapter::TrajectorySetpointAdapter(const State & state) {

    quaternion_t q = state.quaternion();
    euler_angles_t euler_orientation = quatToEul(q);

    init(
        state.stamp(),
        state.position(),
        state.velocity(),
        {NAN, NAN, NAN},
        euler_orientation[2],
        state.angular_velocity()[2]
    );

}

void TrajectorySetpointAdapter::OnlyPosition() {
    
    velocity_ = {NAN, NAN, NAN};
    acceleration_ = {NAN, NAN, NAN};
    yawspeed_ = NAN;
}

px4_msgs::msg::TrajectorySetpoint TrajectorySetpointAdapter::ToMsg() const {

    px4_msgs::msg::TrajectorySetpoint trajectory_setpoint_msg;

    trajectory_setpoint_msg.timestamp = stamp_.nanoseconds() / 1000;

    for (int i = 0; i < 3; i++) {

        double multiplier = 1.0;

        if (i == 1 || i == 2) {
            multiplier = -1.0;
        }

        trajectory_setpoint_msg.position[i] = multiplier*position_[i];
        trajectory_setpoint_msg.velocity[i] = multiplier*velocity_[i];
        trajectory_setpoint_msg.acceleration[i] = multiplier*acceleration_[i];

    }

    trajectory_setpoint_msg.yaw = -yaw_;
    trajectory_setpoint_msg.yawspeed = -yawspeed_;

    return trajectory_setpoint_msg;

}

const rclcpp::Time & TrajectorySetpointAdapter::stamp() const {
    return stamp_;
}

const iii_drone::types::point_t & TrajectorySetpointAdapter::position() const {
    return position_;
}

const iii_drone::types::vector_t & TrajectorySetpointAdapter::velocity() const {
    return velocity_;
}

const iii_drone::types::vector_t & TrajectorySetpointAdapter::acceleration() const {
    return acceleration_;
}

const double & TrajectorySetpointAdapter::yaw() const {
    return yaw_;
}

const double & TrajectorySetpointAdapter::yawspeed() const {
    return yawspeed_;
}

void TrajectorySetpointAdapter::init(
    const rclcpp::Time & stamp,
    const point_t & position,
    const vector_t & velocity,
    const vector_t & acceleration,
    const double & yaw,
    const double & yawspeed
) {

    stamp_ = stamp;

    position_ = position;
    velocity_ = velocity;
    acceleration_ = acceleration;

    yaw_ = yaw;
    yawspeed_ = yawspeed;

}