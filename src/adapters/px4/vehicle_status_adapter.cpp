#include <iii_ros2/px4/vehicle_status.hpp>

using namespace iii_ros2::px4;

VehicleStatus::VehicleStatus() { 

    stamp_ = rclcpp::Time(0);

}

VehicleStatus::VehicleStatus(const px4_msgs::msg::VehicleStatus & vehicle_status_msg) {

    UpdateFromMsg(vehicle_status_msg);

}

void VehicleStatus::UpdateFromMsg(const px4_msgs::msg::VehicleStatus & vehicle_status_msg) {

    stamp_ = rclcpp::Time(vehicle_status_msg.timestamp * 1000);

    armed_time_ = rclcpp::Time(vehicle_status_msg.armed_time * 1000);   
    takeoff_time_ = rclcpp::Time(vehicle_status_msg.takeoff_time * 1000);

    arming_state_ = (arming_state_t)vehicle_status_msg.arming_state;

    latest_arming_reason_ = (arm_disarm_reason_t)vehicle_status_msg.latest_arming_reason;
    latest_disarming_reason_ = (arm_disarm_reason_t)vehicle_status_msg.latest_disarming_reason;

    nav_state_timestamp_ = rclcpp::Time(vehicle_status_msg.nav_state_timestamp * 1000);

    nav_state_user_intention_ = (navigation_state_t)vehicle_status_msg.nav_state_user_intention;
    nav_state_ = (navigation_state_t)vehicle_status_msg.nav_state;

    failure_detector_status_.empty();

    if (vehicle_status_msg.failure_detector_status & FAILURE_NONE) {
        failure_detector_status_.push_back(FAILURE_NONE);
    }

    if (vehicle_status_msg.failure_detector_status & FAILURE_ROLL) {
        failure_detector_status_.push_back(FAILURE_ROLL);
    }

    if (vehicle_status_msg.failure_detector_status & FAILURE_PITCH) {
        failure_detector_status_.push_back(FAILURE_PITCH);
    }

    if (vehicle_status_msg.failure_detector_status & FAILURE_ALT) {
        failure_detector_status_.push_back(FAILURE_ALT);
    }

    if (vehicle_status_msg.failure_detector_status & FAILURE_EXT) {
        failure_detector_status_.push_back(FAILURE_EXT);
    }

    if (vehicle_status_msg.failure_detector_status & FAILURE_ARM_ESC) {
        failure_detector_status_.push_back(FAILURE_ARM_ESC);
    }

    if (vehicle_status_msg.failure_detector_status & FAILURE_ARM_BATTERY) {
        failure_detector_status_.push_back(FAILURE_ARM_BATTERY);
    }

    if (vehicle_status_msg.failure_detector_status & FAILURE_IMBALANCED_PROP) {
        failure_detector_status_.push_back(FAILURE_IMBALANCED_PROP);
    }

    if (vehicle_status_msg.failure_detector_status & FAILURE_MOTOR) {
        failure_detector_status_.push_back(FAILURE_MOTOR);
    }

    hil_state_ = (hil_state_t)vehicle_status_msg.hil_state;

    vehicle_type_ = (vehicle_type_t)vehicle_status_msg.vehicle_type;

    failsafe_ = vehicle_status_msg.failsafe;
    failsafe_and_user_took_over_ = vehicle_status_msg.failsafe_and_user_took_over;

    gcs_connection_lost_ = vehicle_status_msg.gcs_connection_lost;
    gcs_connection_lost_counter_ = vehicle_status_msg.gcs_connection_lost_counter;
    high_latency_data_link_lost_ = vehicle_status_msg.high_latency_data_link_lost;

    system_type_ = vehicle_status_msg.system_type;
    system_id_ = vehicle_status_msg.system_id;
    component_id_ = vehicle_status_msg.component_id;

    safety_button_available_ = vehicle_status_msg.safety_button_available;
    safety_off_ = vehicle_status_msg.safety_off;

    power_input_valid_ = vehicle_status_msg.power_input_valid;
    usb_connected_ = vehicle_status_msg.usb_connected;

    pre_flight_checks_pass_ = vehicle_status_msg.pre_flight_checks_pass;

}

const rclcpp::Time & VehicleStatus::stamp() const {

    return stamp_;

}

const rclcpp::Time & VehicleStatus::armed_time() const {

    return armed_time_;

}

const rclcpp::Time & VehicleStatus::takeoff_time() const {

    return takeoff_time_;

}

const arming_state_t & VehicleStatus::arming_state() const {

    return arming_state_;

}

const arm_disarm_reason_t & VehicleStatus::latest_arming_reason() const {

    return latest_arming_reason_;

}

const arm_disarm_reason_t & VehicleStatus::latest_disarming_reason() const {

    return latest_disarming_reason_;

}

const rclcpp::Time & VehicleStatus::nav_state_timestamp() const {

    return nav_state_timestamp_;

}

const navigation_state_t & VehicleStatus::nav_state_user_intention() const {

    return nav_state_user_intention_;

}

const navigation_state_t & VehicleStatus::nav_state() const {

    return nav_state_;

}

const std::vector<vehicle_status_failure_t> & VehicleStatus::failure_detector_status() const {

    return failure_detector_status_;

}

const hil_state_t & VehicleStatus::hil_state() const {

    return hil_state_;

}

const vehicle_type_t & VehicleStatus::vehicle_type() const {

    return vehicle_type_;

}

const bool & VehicleStatus::failsafe() const {

    return failsafe_;

}

const bool & VehicleStatus::failsafe_and_user_took_over() const {

    return failsafe_and_user_took_over_;

}

const bool & VehicleStatus::gcs_connection_lost() const {

    return gcs_connection_lost_;

}

const uint8_t & VehicleStatus::gcs_connection_lost_counter() const {

    return gcs_connection_lost_counter_;

}

const bool & VehicleStatus::high_latency_data_link_lost() const {

    return high_latency_data_link_lost_;

}

const uint8_t & VehicleStatus::system_type() const {

    return system_type_;

}

const uint8_t & VehicleStatus::system_id() const {

    return system_id_;

}

const uint8_t & VehicleStatus::component_id() const {

    return component_id_;

}

const bool & VehicleStatus::safety_button_available() const {

    return safety_button_available_;

}

const bool & VehicleStatus::safety_off() const {

    return safety_off_;

}

const bool & VehicleStatus::power_input_valid() const {

    return power_input_valid_;

}

const bool & VehicleStatus::usb_connected() const {

    return usb_connected_;

}

const bool & VehicleStatus::pre_flight_checks_pass() const {

    return pre_flight_checks_pass_;

}