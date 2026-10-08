/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/maneuver/maneuver_profile_policy.hpp>
#include <iii_drone_core/utils/runtime_profile.hpp>

/*****************************************************************************/
// Implementation
/*****************************************************************************/

bool iii_drone::control::maneuver::ManeuverAvailableInProfile(
    maneuver_type_t maneuver_type,
    const std::string & runtime_profile
) {
    if (runtime_profile != iii_drone::utils::kOptiTrackRuntimeProfile) {
        return true;
    }

    switch (maneuver_type) {
        case MANEUVER_TYPE_HOVER:
        case MANEUVER_TYPE_FLY_TO_POSITION:
        case MANEUVER_TYPE_FOLLOW_WAYPOINT_PATH:
            return true;
        default:
            return false;
    }
}

std::string iii_drone::control::maneuver::ManeuverUnavailableMessage(
    const std::string & maneuver_name,
    const std::string & runtime_profile
) {
    return "Maneuver " + maneuver_name + " is not available in the " + runtime_profile + " profile";
}
