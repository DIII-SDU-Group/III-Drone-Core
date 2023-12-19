/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/reference_trajectory.hpp>

using namespace iii_drone::control;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

ReferenceTrajectory::ReferenceTrajectory() {
    references_ = std::vector<Reference>();
}

ReferenceTrajectory::ReferenceTrajectory(const std::vector<Reference>& references) {
    references_ = references;
}

ReferenceTrajectory::ReferenceTrajectory(const std::vector<State>& states) {
    references_ = std::vector<Reference>();

    for (const State& state : states) {
        references_.push_back(Reference(state));
    }
}

const std::vector<Reference>& ReferenceTrajectory::references() const {
    return references_;
}