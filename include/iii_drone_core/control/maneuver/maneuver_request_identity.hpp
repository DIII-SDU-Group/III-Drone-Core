#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <random>
#include <sstream>
#include <string>

namespace iii_drone {
namespace control {
namespace maneuver {

/**
 * The native request identity is opaque to consumers. Restricting its wire
 * representation makes missing, legacy, and malformed goals fail closed
 * before they can claim a reference generation.
 *
 * A process-local random epoch prevents a restarted Mission process at ROS
 * time zero from recreating a prior token. The counter is monotonic for the
 * lifetime of the generator and shared by all action types through its one
 * Mission-side instance.
 */
class ManeuverRequestIdentityGenerator {
public:
    using Epoch = std::array<std::uint64_t, 2>;

    ManeuverRequestIdentityGenerator()
    : ManeuverRequestIdentityGenerator(randomEpoch()) {}

    explicit ManeuverRequestIdentityGenerator(const Epoch & epoch)
    : epoch_(epoch) {}

    std::string next() {
        return format(epoch_, counter_.fetch_add(1) + 1);
    }

    static std::string format(const Epoch & epoch, std::uint64_t counter) {
        std::ostringstream identity;
        identity << "mri1-" << std::hex << std::setfill('0')
                 << std::setw(16) << epoch[0]
                 << std::setw(16) << epoch[1]
                 << '-' << std::setw(16) << counter;
        return identity.str();
    }

private:
    static Epoch randomEpoch() {
        std::random_device source;
        Epoch epoch{
            randomWord(source),
            randomWord(source),
        };
        if (epoch[0] == 0 && epoch[1] == 0) {
            epoch[1] = 1;
        }
        return epoch;
    }

    static std::uint64_t randomWord(std::random_device & source) {
        return (static_cast<std::uint64_t>(source()) << 32U) ^
            static_cast<std::uint64_t>(source());
    }

    Epoch epoch_;
    std::atomic<std::uint64_t> counter_{0};
};

/**
 * Return the next opaque maneuver-request identity for this process.
 *
 * The definition lives in iii_drone_core, so every Mission producer in the
 * process shares the same random epoch and monotonic counter.  Keep the
 * deterministic generator class above for isolated protocol tests.
 */
std::string nextProcessManeuverRequestIdentity();

inline bool isValidManeuverRequestIdentity(const std::string & identity) {
    constexpr std::size_t kLength = 54;
    constexpr std::size_t kEpochCounterSeparator = 37;
    if (
        identity.size() != kLength ||
        identity.compare(0, 5, "mri1-") != 0 ||
        identity[kEpochCounterSeparator] != '-'
    ) {
        return false;
    }
    for (std::size_t index = 5; index < identity.size(); ++index) {
        if (index == kEpochCounterSeparator) {
            continue;
        }
        const char value = identity[index];
        if (!(
            (value >= '0' && value <= '9') ||
            (value >= 'a' && value <= 'f')
        )) {
            return false;
        }
    }
    return true;
}

}  // namespace maneuver
}  // namespace control
}  // namespace iii_drone
