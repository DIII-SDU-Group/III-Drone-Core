#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <string>

#include <iii_drone_core/control/maneuver/maneuver_types.hpp>

/*****************************************************************************/
// Functions
/*****************************************************************************/

namespace iii_drone {
namespace control {
namespace maneuver {

    /**
     * @brief Whether a maneuver may run in a runtime profile (see
     * iii_drone::utils::ResolveRuntimeProfile). The opti_track profile has no
     * cable, payload or perception and allows only hover, fly_to_position and
     * follow_waypoint_path. It is an allowlist: a maneuver added later stays
     * unavailable there until it is listed. Every other profile, including an
     * empty or unknown one, is unrestricted.
     *
     * @param maneuver_type The maneuver type.
     * @param runtime_profile The normalised runtime profile name.
     *
     * @return bool True if goals of this maneuver may be served.
     */
    bool ManeuverAvailableInProfile(
        maneuver_type_t maneuver_type,
        const std::string & runtime_profile
    );

    /**
     * @brief The rejection text for a maneuver that is unavailable in a
     * profile: "Maneuver <name> is not available in the <profile> profile".
     *
     * @param maneuver_name The maneuver's action name, e.g. cable_landing.
     * @param runtime_profile The normalised runtime profile name.
     */
    std::string ManeuverUnavailableMessage(
        const std::string & maneuver_name,
        const std::string & runtime_profile
    );

} // namespace maneuver
} // namespace control
} // namespace iii_drone
