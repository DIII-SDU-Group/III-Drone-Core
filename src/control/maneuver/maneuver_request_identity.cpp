#include <iii_drone_core/control/maneuver/maneuver_request_identity.hpp>

namespace iii_drone {
namespace control {
namespace maneuver {

namespace {

ManeuverRequestIdentityGenerator & processGenerator() {
    static ManeuverRequestIdentityGenerator generator;
    return generator;
}

}  // namespace

std::string nextProcessManeuverRequestIdentity() {
    return processGenerator().next();
}

ManeuverRequestScope processManeuverRequestScope() {
    ManeuverRequestScope scope;
    scope.epoch = processGenerator().epochLabel();
    scope.last_counter = processGenerator().lastIssuedCounter();
    return scope;
}

}  // namespace maneuver
}  // namespace control
}  // namespace iii_drone
