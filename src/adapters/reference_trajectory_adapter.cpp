/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/adapters/reference_trajectory_adapter.hpp>

using namespace iii_drone::adapters;
using namespace iii_drone::types;
using namespace iii_drone::math;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

ReferenceTrajectoryAdapter::ReferenceTrajectoryAdapter() : reference_trajectory_(iii_drone::control::ReferenceTrajectory()) {}

ReferenceTrajectoryAdapter::ReferenceTrajectoryAdapter(const iii_drone::control::ReferenceTrajectory &reference_trajectory) : reference_trajectory_(reference_trajectory) {}

ReferenceTrajectoryAdapter::ReferenceTrajectoryAdapter(const iii_drone_interfaces::msg::ReferenceTrajectory &msg) {

    std::vector<iii_drone::control::Reference> references;

    for (auto reference_msg : msg.references) {
        references.push_back(ReferenceAdapter(reference_msg).reference());
    }

    reference_trajectory_ = iii_drone::control::ReferenceTrajectory(references);

}

ReferenceTrajectoryAdapter::ReferenceTrajectoryAdapter(const iii_drone_interfaces::msg::ReferenceTrajectory::SharedPtr &msg) {

    std::vector<iii_drone::control::Reference> references;

    for (auto reference_msg : msg->references) {
        references.push_back(ReferenceAdapter(reference_msg).reference());
    }

    reference_trajectory_ = iii_drone::control::ReferenceTrajectory(references);

}

iii_drone_interfaces::msg::ReferenceTrajectory ReferenceTrajectoryAdapter::ToMsg() const {
    iii_drone_interfaces::msg::ReferenceTrajectory msg;

    for (auto reference : reference_trajectory_.references()) {
        msg.references.push_back(ReferenceAdapter(reference).ToMsg());
    }

    return msg;
}

iii_drone::control::ReferenceTrajectory ReferenceTrajectoryAdapter::reference_trajectory() const {
    return reference_trajectory_;
}
