#include <iii_ros2/px4/trajectory_setpoint.hpp>

#include <iostream>

using namespace iii_ros2::px4;
using namespace iii_ros2::control;
using namespace iii::control;
using namespace iii::types;

TrajectorySetpoint::TrajectorySetpoint() { 

    init(
        rclcpp::Time(0, 0, RCL_ROS_TIME),
        {NAN, NAN, NAN},
        {NAN, NAN, NAN},
        {NAN, NAN, NAN},
        NAN,
        NAN
    );

}

TrajectorySetpoint::TrajectorySetpoint(const rclcpp::Time & stamp) {

    init(
        stamp,
        {NAN, NAN, NAN},
        {NAN, NAN, NAN},
        {NAN, NAN, NAN},
        NAN,
        NAN
    );

}

TrajectorySetpoint::TrajectorySetpoint(
    const rclcpp::Time & stamp,
    const Reference & reference
) {

    init(
        stamp,
        reference.position(),
        {NAN, NAN, NAN},
        {NAN, NAN, NAN},
        reference.yaw(),
        NAN
    );

}

TrajectorySetpoint::TrajectorySetpoint(
    const rclcpp::Time & stamp,
    const FlightControlReference & flight_control_reference
) {

    Reference reference = flight_control_reference.ToReference();

    init(
        stamp,
        reference.position(),
        {NAN, NAN, NAN},
        {NAN, NAN, NAN},
        reference.yaw(),
        NAN
    );

}

TrajectorySetpoint::TrajectorySetpoint(
    const rclcpp::Time & stamp,
    const State & state
) {

    quaternion_t q = state.quaternion();
    euler_orientation_t euler_orientation = iii::math::QuaternionToEuler(q);

    init(
        stamp,
        state.position(),
        state.velocity(),
        {NAN, NAN, NAN},
        euler_orientation[2],
        state.angular_velocity()[2]
    );

}

void TrajectorySetpoint::OnlyPosition() {
    
    velocity_ = {NAN, NAN, NAN};
    acceleration_ = {NAN, NAN, NAN};
    yawspeed_ = NAN;
}

px4_msgs::msg::TrajectorySetpoint TrajectorySetpoint::ToMsg() const {

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

const rclcpp::Time & TrajectorySetpoint::stamp() const {
    return stamp_;
}

const iii::types::position_t & TrajectorySetpoint::position() const {
    return position_;
}

const iii::types::velocity_t & TrajectorySetpoint::velocity() const {
    return velocity_;
}

const iii::types::acceleration_t & TrajectorySetpoint::acceleration() const {
    return acceleration_;
}

const double & TrajectorySetpoint::yaw() const {
    return yaw_;
}

const double & TrajectorySetpoint::yawspeed() const {
    return yawspeed_;
}

void TrajectorySetpoint::init(
    const rclcpp::Time & stamp,
    const position_t & position,
    const velocity_t & velocity,
    const acceleration_t & acceleration,
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