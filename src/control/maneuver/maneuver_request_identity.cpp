#include <iii_drone_core/control/maneuver/maneuver_request_identity.hpp>

namespace iii_drone {
namespace control {
namespace maneuver {

std::string nextProcessManeuverRequestIdentity() {
    static ManeuverRequestIdentityGenerator generator;
    return generator.next();
}

}  // namespace maneuver
}  // namespace control
}  // namespace iii_drone
