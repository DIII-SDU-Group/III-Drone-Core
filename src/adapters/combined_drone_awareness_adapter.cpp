/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/adapters/combined_drone_awareness_adapter.hpp>

using namespace iii_drone::adapters;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

CombinedDroneAwarenessAdapter::CombinedDroneAwarenessAdapter() :
    state_(),
    armed_(false),
    offboard_(false),
    target_adapter_(),
    target_position_known_(false),
    drone_location_(DRONE_LOCATION_UNKNOWN),
    on_cable_id_(0),
    ground_altitude_estimate_(0.0),
    ground_altitude_estimate_amsl_(0.0),
    gripper_open_(false) {}

CombinedDroneAwarenessAdapter::CombinedDroneAwarenessAdapter(const iii_drone_interfaces::msg::CombinedDroneAwareness & msg) {

    state_ = StateAdapter(msg.state).state();
    armed_ = msg.armed;
    offboard_ = msg.offboard;
    target_adapter_ = TargetAdapter(msg.target);
    target_position_known_ = msg.target_position_known;
    drone_location_ = (drone_location_t)msg.drone_location;
    on_cable_id_ = msg.on_cable_id;
    ground_altitude_estimate_ = msg.ground_altitude_estimate;
    ground_altitude_estimate_amsl_ = msg.ground_altitude_estimate_amsl;
    gripper_open_ = msg.gripper_open;

}

CombinedDroneAwarenessAdapter::CombinedDroneAwarenessAdapter(
    const iii_drone::control::State & state,
    bool armed,
    bool offboard,
    const iii_drone::adapters::TargetAdapter & target_adapter,
    bool target_position_known,
    drone_location_t drone_location,
    int on_cable_id,
    float ground_altitude_estimate,
    float ground_altitude_estimate_amsl,
    bool gripper_open
) : state_(state),
    armed_(armed),
    offboard_(offboard),
    target_adapter_(target_adapter),
    target_position_known_(target_position_known),
    drone_location_(drone_location),
    on_cable_id_(on_cable_id),
    ground_altitude_estimate_(ground_altitude_estimate),
    ground_altitude_estimate_amsl_(ground_altitude_estimate_amsl),
    gripper_open_(gripper_open) {

}

const iii_drone_interfaces::msg::CombinedDroneAwareness CombinedDroneAwarenessAdapter::ToMsg() const {

    iii_drone_interfaces::msg::CombinedDroneAwareness msg;

    msg.state = StateAdapter(state_).ToMsg();
    msg.armed = armed_;
    msg.offboard = offboard_;
    msg.has_target = has_target();
    msg.target = target_adapter_.ToMsg();
    msg.target_position_known = target_position_known_;
    msg.drone_location = drone_location_;
    msg.on_cable_id = on_cable_id_;
    msg.ground_altitude_estimate = ground_altitude_estimate_;
    msg.ground_altitude_estimate_amsl = ground_altitude_estimate_amsl_;
    msg.gripper_open = gripper_open_;

    return msg;

}

const iii_drone::control::State CombinedDroneAwarenessAdapter::state() const {
    return state_;
}

iii_drone::control::State & CombinedDroneAwarenessAdapter::state() {
    return state_;
}

bool CombinedDroneAwarenessAdapter::armed() const {
    return armed_;
}

bool & CombinedDroneAwarenessAdapter::armed() {
    return armed_;
}

bool CombinedDroneAwarenessAdapter::offboard() const {
    return offboard_;
}

bool & CombinedDroneAwarenessAdapter::offboard() {
    return offboard_;
}

bool CombinedDroneAwarenessAdapter::has_target() const {
    return target_adapter_.target_type() != iii_drone::adapters::TARGET_TYPE_NONE;
}

const iii_drone::adapters::TargetAdapter CombinedDroneAwarenessAdapter::target_adapter() const {
    return target_adapter_;
}

iii_drone::adapters::TargetAdapter & CombinedDroneAwarenessAdapter::target_adapter() {
    return target_adapter_;
}

bool CombinedDroneAwarenessAdapter::target_position_known() const {
    return target_position_known_;
}

bool & CombinedDroneAwarenessAdapter::target_position_known() {
    return target_position_known_;
}

drone_location_t CombinedDroneAwarenessAdapter::drone_location() const {
    return drone_location_;
}

drone_location_t & CombinedDroneAwarenessAdapter::drone_location() {
    return drone_location_;
}

bool CombinedDroneAwarenessAdapter::on_ground() const {
    return drone_location_ == DRONE_LOCATION_ON_GROUND;
}

bool CombinedDroneAwarenessAdapter::in_flight() const {
    return drone_location_ == DRONE_LOCATION_IN_FLIGHT;
}

bool CombinedDroneAwarenessAdapter::on_cable() const {
    return drone_location_ == DRONE_LOCATION_ON_CABLE;
}

int CombinedDroneAwarenessAdapter::on_cable_id() const {
    return on_cable_id_;
}

int & CombinedDroneAwarenessAdapter::on_cable_id() {
    return on_cable_id_;
}

float CombinedDroneAwarenessAdapter::ground_altitude_estimate() const {
    return ground_altitude_estimate_;
}

float & CombinedDroneAwarenessAdapter::ground_altitude_estimate() {
    return ground_altitude_estimate_;
}

float CombinedDroneAwarenessAdapter::ground_altitude_estimate_amsl() const {
    return ground_altitude_estimate_amsl_;
}

float & CombinedDroneAwarenessAdapter::ground_altitude_estimate_amsl() {
    return ground_altitude_estimate_amsl_;
}

bool CombinedDroneAwarenessAdapter::gripper_open() const {
    return gripper_open_;
}

bool & CombinedDroneAwarenessAdapter::gripper_open() {
    return gripper_open_;
}