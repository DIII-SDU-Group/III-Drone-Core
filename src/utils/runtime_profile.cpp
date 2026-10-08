/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/utils/runtime_profile.hpp>

#include <algorithm>
#include <cctype>
#include <cstdlib>

/*****************************************************************************/
// Implementation
/*****************************************************************************/

namespace {

std::string normalised(const std::string & value) {
    const auto is_space = [](unsigned char character) { return std::isspace(character) != 0; };
    const auto begin = std::find_if_not(value.begin(), value.end(), is_space);
    const auto end = std::find_if_not(value.rbegin(), value.rend(), is_space).base();
    std::string result = begin < end ? std::string(begin, end) : std::string();
    std::transform(result.begin(), result.end(), result.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return result;
}

}  // namespace

std::string iii_drone::utils::ResolveRuntimeProfile(
    const std::string & parameter_value,
    const char * environment_value
) {
    const std::string from_parameter = normalised(parameter_value);
    if (!from_parameter.empty()) {
        return from_parameter;
    }
    return environment_value == nullptr ? std::string() : normalised(environment_value);
}

std::string iii_drone::utils::ResolveRuntimeProfile(const std::string & parameter_value) {
    return ResolveRuntimeProfile(
        parameter_value,
        std::getenv(kRuntimeProfileEnvironmentVariable)
    );
}
