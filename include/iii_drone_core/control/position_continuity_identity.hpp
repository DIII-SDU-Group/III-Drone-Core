#pragma once

#include <cstdint>

namespace iii_drone::control {

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
