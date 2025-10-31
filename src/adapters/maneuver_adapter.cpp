/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/adapters/maneuver_adapter.hpp>

using namespace iii_drone::adapters;
using namespace iii_drone::control::maneuver;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

ManeuverAdapter::ManeuverAdapter() {
    maneuver_type_ = MANEUVER_TYPE_NONE;
    start_time_ = rclcpp::Time(0, 0);
    terminated_ = false;
    success_ = false;
}

ManeuverAdapter::ManeuverAdapter(const ManeuverAdapter &other) {
    maneuver_type_ = other.maneuver_type_;
    start_time_ = other.start_time_;
    terminated_ = other.terminated_;
    success_ = other.success_;
}

ManeuverAdapter::ManeuverAdapter(const iii_drone_interfaces::msg::Maneuver &msg) {
    maneuver_type_ = static_cast<maneuver_type_t>(msg.maneuver_type);
    start_time_ = rclcpp::Time(msg.start_time);
    terminated_ = msg.terminated;
    success_ = msg.success;
}

ManeuverAdapter::ManeuverAdapter(const iii_drone::control::maneuver::Maneuver &maneuver) {
    maneuver_type_ = maneuver.maneuver_type();
    start_time_ = maneuver.start_time();
    terminated_ = maneuver.terminated();
    success_ = maneuver.success();
}

iii_drone_interfaces::msg::Maneuver ManeuverAdapter::ToMsg() const {
    iii_drone_interfaces::msg::Maneuver msg;

    msg.maneuver_type = static_cast<int8_t>(maneuver_type_);
    msg.start_time = start_time_;
    msg.terminated = terminated_;
    msg.success = success_;

    return msg;

}

iii_drone::control::maneuver::maneuver_type_t ManeuverAdapter::maneuver_type() const {
    return maneuver_type_;
}

rclcpp::Time ManeuverAdapter::start_time() const {
    return start_time_;
}

bool ManeuverAdapter::terminated() const {
    return terminated_;
}

bool ManeuverAdapter::success() const {
    return success_;
}

ManeuverAdapter & ManeuverAdapter::operator=(const ManeuverAdapter &other) {
    maneuver_type_ = other.maneuver_type_;
    start_time_ = other.start_time_;
    terminated_ = other.terminated_;
    success_ = other.success_;
    return *this;
}