/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/adapters/state_adapter.hpp>

using namespace iii_drone::adapters;
using namespace iii_drone::types;
using namespace iii_drone::math;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

StateAdapter::StateAdapter() : state_(iii_drone::control::State()) {}

StateAdapter::StateAdapter(const iii_drone::control::State &state) : state_(state) {}

StateAdapter::StateAdapter(const iii_drone_interfaces::msg::State &msg) {

    pose_t pose = poseFromPoseMsg(msg.pose);
    vector_t velocity = vectorFromVectorMsg(msg.velocity);
    vector_t angular_velocity = vectorFromVectorMsg(msg.angular_velocity);

    state_ = iii_drone::control::State(
        pose.position,
        velocity,
        pose.orientation,
        angular_velocity,
        msg.stamp
    );

}

StateAdapter::StateAdapter(const iii_drone_interfaces::msg::State::SharedPtr &msg) {

    pose_t pose = poseFromPoseMsg(msg->pose);
    vector_t velocity = vectorFromVectorMsg(msg->velocity);
    vector_t angular_velocity = vectorFromVectorMsg(msg->angular_velocity);

    state_ = iii_drone::control::State(
        pose.position,
        velocity,
        pose.orientation,
        angular_velocity,
        msg->stamp
    );

}

iii_drone_interfaces::msg::State StateAdapter::ToMsg() const {

    iii_drone_interfaces::msg::State msg;

    msg.pose = poseMsgFromPose(
        poseFromPose(
            state_.position(),
            state_.quaternion()
        )
    );
    msg.velocity = vectorMsgFromVector(state_.velocity());
    msg.angular_velocity = vectorMsgFromVector(state_.angular_velocity());

    msg.stamp = state_.stamp();

    return msg;

}

iii_drone::control::State StateAdapter::state() const {
    return state_;
}