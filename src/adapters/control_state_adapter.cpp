/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/adapters/control_state_adapter.hpp>

/*****************************************************************************/
// Implementation
/*****************************************************************************/

using namespace iii_drone::adapters;

ControlStateAdapter::ControlStateAdapter() {

    control_state_ = CONTROL_STATE_INIT;

    stamp_ = rclcpp::Clock().now();

}

ControlStateAdapter::ControlStateAdapter(const control_state_t & control_state) {

    control_state_ = control_state;

    stamp_ = rclcpp::Clock().now();

}

ControlStateAdapter::ControlStateAdapter(const iii_drone_interfaces::msg::ControlState & msg) {

    UpdateFromMsg(msg);

}

void ControlStateAdapter::UpdateFromMsg(const iii_drone_interfaces::msg::ControlState & msg) {

    control_state_ = (control_state_t) msg.state;

    stamp_ = msg.timestamp;

}

iii_drone_interfaces::msg::ControlState ControlStateAdapter::ToMsg() const {

    iii_drone_interfaces::msg::ControlState msg{};
    msg.timestamp = stamp_;
    msg.state = (iii_drone_interfaces::msg::ControlState::_state_type) control_state_;

    return msg;

}

const control_state_t & ControlStateAdapter::control_state() const {
    return control_state_;
}

const rclcpp::Time & ControlStateAdapter::stamp() const {
    return stamp_;
}