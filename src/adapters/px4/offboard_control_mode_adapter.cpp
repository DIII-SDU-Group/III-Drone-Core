#include <iii_drone_core/adapters/px4/offboard_control_mode_adapter.hpp>

using namespace iii_drone::adapters::px4;

OffboardControlModeAdapter::OffboardControlModeAdapter(
    const rclcpp::Time & stamp,
    const bool & position,
    const bool & velocity,
    const bool & acceleration,
    const bool & attitude,
    const bool & body_rate,
    const bool & actuator
) {

    stamp_ = stamp;

    position_ = position;
    velocity_ = velocity;
    acceleration_ = acceleration;
    attitude_ = attitude;
    body_rate_ = body_rate;
    actuator_ = actuator;

}

px4_msgs::msg::OffboardControlMode OffboardControlModeAdapter::ToMsg() const {

    px4_msgs::msg::OffboardControlMode msg{};
    msg.timestamp = stamp_.nanoseconds() / 1000;
    msg.position = position_;
    msg.velocity = velocity_;
    msg.acceleration = acceleration_;
    msg.attitude = attitude_;
    msg.body_rate = body_rate_;;	
    msg.actuator = actuator_;
    
    return msg;

}

const rclcpp::Time & OffboardControlModeAdapter::stamp() const {
    return stamp_;
}

const bool & OffboardControlModeAdapter::position() const {
    return position_;
}

const bool & OffboardControlModeAdapter::velocity() const {
    return velocity_;
}

const bool & OffboardControlModeAdapter::acceleration() const {
    return acceleration_;
}

const bool & OffboardControlModeAdapter::attitude() const {
    return attitude_;
}

const bool & OffboardControlModeAdapter::body_rate() const {
    return body_rate_;
}

const bool & OffboardControlModeAdapter::actuator() const {
    return actuator_;
}
