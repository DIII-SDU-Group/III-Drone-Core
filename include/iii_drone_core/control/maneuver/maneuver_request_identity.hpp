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

    /** "mri1-" plus the 32 hex digit epoch shared by every identity minted here. */
    std::string epochLabel() const {
        return format(epoch_, 0).substr(0, 37);
    }

    /** Counter of the most recently minted identity; zero before the first. */
    std::uint64_t lastIssuedCounter() const {
        return counter_.load();
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

/**
 * Identities minted by one producer up to (and including) one counter.
 *
 * Mission Exit releases exactly this set: a later run of the same process
 * mints larger counters and is never affected by an earlier release.
 */
struct ManeuverRequestScope {
    std::string epoch;
    std::uint64_t last_counter = 0;

    bool valid() const;
    bool contains(const std::string & request_identity) const;
};

/** The epoch of this process' generator and its latest minted counter. */
ManeuverRequestScope processManeuverRequestScope();

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

/** "mri1-<32 hex>" prefix of a valid identity, empty otherwise. */
inline std::string maneuverRequestIdentityEpoch(const std::string & identity) {
    return isValidManeuverRequestIdentity(identity) ? identity.substr(0, 37) : std::string();
}

/** Monotonic counter suffix of a valid identity, zero otherwise. */
inline std::uint64_t maneuverRequestIdentityCounter(const std::string & identity) {
    if (!isValidManeuverRequestIdentity(identity)) {
        return 0;
    }
    return std::stoull(identity.substr(38), nullptr, 16);
}

inline bool isValidManeuverRequestEpoch(const std::string & epoch) {
    return epoch.size() == 37 &&
        isValidManeuverRequestIdentity(epoch + "-0000000000000001");
}

inline bool ManeuverRequestScope::valid() const {
    return last_counter != 0 && isValidManeuverRequestEpoch(epoch);
}

inline bool ManeuverRequestScope::contains(const std::string & request_identity) const {
    if (!valid() || maneuverRequestIdentityEpoch(request_identity) != epoch) {
        return false;
    }
    const std::uint64_t counter = maneuverRequestIdentityCounter(request_identity);
    return counter != 0 && counter <= last_counter;
}

}  // namespace maneuver
}  // namespace control
}  // namespace iii_drone
