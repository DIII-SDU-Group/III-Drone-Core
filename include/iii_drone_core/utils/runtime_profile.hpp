#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <string>

/*****************************************************************************/
// Functions
/*****************************************************************************/

namespace iii_drone {
namespace utils {

    /**
     * @brief ROS parameter that names the runtime profile of a node. A non-empty
     * value takes precedence over the environment variable.
     */
    constexpr const char * kRuntimeProfileParameter = "iii_runtime_profile";

    /**
     * @brief Environment variable that Supervision sets to the boot profile for
     * every entity process.
     */
    constexpr const char * kRuntimeProfileEnvironmentVariable = "III_SYSTEM_PROFILE";

    /**
     * @brief The reduced flight-basics profile of the OptiTrack lab: no cable,
     * payload, perception or corridor.
     */
    constexpr const char * kOptiTrackRuntimeProfile = "opti_track";

    /**
     * @brief Resolves a node's runtime profile: the parameter value if it is
     * non-empty, else the environment value (nullptr when unset). The result is
     * trimmed and lower-cased; it is empty when neither names a profile. An
     * empty or unknown profile is unrestricted.
     *
     * @param parameter_value Value of the iii_runtime_profile parameter.
     * @param environment_value Value of III_SYSTEM_PROFILE, or nullptr.
     *
     * @return std::string The normalised profile name.
     */
    std::string ResolveRuntimeProfile(
        const std::string & parameter_value,
        const char * environment_value
    );

    /**
     * @brief Resolves the runtime profile from the parameter value and the
     * process environment (III_SYSTEM_PROFILE).
     */
    std::string ResolveRuntimeProfile(const std::string & parameter_value);

} // namespace utils
} // namespace iii_drone
