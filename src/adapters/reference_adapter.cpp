/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/adapters/reference_adapter.hpp>

using namespace iii_drone::adapters;
using namespace iii_drone::types;
using namespace iii_drone::math;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

ReferenceAdapter::ReferenceAdapter() : reference_(iii_drone::control::Reference()) {}

ReferenceAdapter::ReferenceAdapter(const iii_drone::control::Reference &reference) : reference_(reference) {}

ReferenceAdapter::ReferenceAdapter(const iii_drone_interfaces::msg::Reference &msg) {
    
    point_t position = pointFromPointMsg(msg.position);
    double yaw = msg.yaw;

    vector_t velocity = vectorFromVectorMsg(msg.velocity);
    double yaw_rate = msg.yaw_rate;

    vector_t acceleration = vectorFromVectorMsg(msg.acceleration);
    double yaw_acceleration = msg.yaw_acceleration;

    reference_ = iii_drone::control::Reference(
        position,
        yaw,
        velocity,
        yaw_rate,
        acceleration,
        yaw_acceleration,
        msg.stamp
    );

}

iii_drone_interfaces::msg::Reference ReferenceAdapter::ToMsg() const {
    iii_drone_interfaces::msg::Reference msg;

    msg.position = pointMsgFromPoint(reference_.position());
    msg.yaw = reference_.yaw();

    msg.velocity = vectorMsgFromVector(reference_.velocity());
    msg.yaw_rate = reference_.yaw_rate();

    msg.acceleration = vectorMsgFromVector(reference_.acceleration());
    msg.yaw_acceleration = reference_.yaw_acceleration();

    msg.stamp = reference_.stamp();

    return msg;
}

iii_drone::control::Reference ReferenceAdapter::reference() const {
    return reference_;
}