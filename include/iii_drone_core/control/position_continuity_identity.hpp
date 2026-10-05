#pragma once

#include <cstdint>

namespace iii_drone::control {

// Longest ended gap between consecutive PX4 odometry samples that consumers
// treat as continuous. HIL showed single gaps of ~0.3 s on the SITL -> XRCE ->
// Pi path; sample ages are still bounded separately by each consumer's
// freshness limit (0.25 s), so this only stops an ended gap from being
// mistaken for a discontinuity.
inline constexpr double kMaximumOdometrySampleGapS = 0.5;

// The raw aggregate remains independent: only two source-qualified samples
// may bridge a change in VehicleOdometry::reset_counter.
struct PositionContinuityIdentity {
    uint64_t source_epoch = 0;
    uint64_t position_epoch = 0;
    uint8_t raw_reset_counter = 0;
    bool source_qualified = false;
};

inline bool SamePositionContinuity(
    const PositionContinuityIdentity & earlier,
    const PositionContinuityIdentity & later) {
    if (earlier.source_epoch != later.source_epoch ||
        earlier.position_epoch != later.position_epoch) return false;
    if (earlier.raw_reset_counter == later.raw_reset_counter) return true;
    return earlier.source_qualified && later.source_qualified;
}

}  // namespace iii_drone::control
