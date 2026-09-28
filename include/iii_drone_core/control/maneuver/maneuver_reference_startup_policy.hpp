#pragma once

#include <cmath>

#include <iii_drone_core/control/reference.hpp>

namespace iii_drone::control::maneuver {

/**
 * Tracks the one continuity-baseline allowance at the start of a stream
 * generation. The scheduler's initialization hold deliberately has NaN
 * derivatives, so it must not consume the allowance reserved for the first
 * planned reference. A planned reference may be fully finite, use the
 * velocity-only shape used by HoverOnCable, or the MPC shape with NaN yaw
 * derivatives.
 */
class ManeuverReferenceStartupPolicy {
public:
    void arm() { first_planned_baseline_pending_ = true; }
    void reset() { first_planned_baseline_pending_ = false; }

    bool consumeFirstPlannedBaseline(const Reference & reference) {
        if (
            !first_planned_baseline_pending_ ||
            (!fullyFinite(reference) &&
                !validVelocityOnly(reference) &&
                !validMpcReference(reference))
        ) {
            return false;
        }
        first_planned_baseline_pending_ = false;
        return true;
    }

    bool firstPlannedBaselinePending() const {
        return first_planned_baseline_pending_;
    }

private:
    static bool fullyFinite(const Reference & reference) {
        return reference.position().allFinite() &&
            reference.velocity().allFinite() &&
            reference.acceleration().allFinite() &&
            std::isfinite(reference.yaw()) &&
            std::isfinite(reference.yaw_rate()) &&
            std::isfinite(reference.yaw_acceleration());
    }

    static bool allNaN(const iii_drone::types::vector_t & value) {
        return std::isnan(value.x()) && std::isnan(value.y()) && std::isnan(value.z());
    }

    static bool validVelocityOnly(const Reference & reference) {
        return allNaN(reference.position()) &&
            std::isnan(reference.yaw()) &&
            reference.velocity().allFinite() &&
            std::isfinite(reference.yaw_rate()) &&
            allNaN(reference.acceleration()) &&
            std::isnan(reference.yaw_acceleration());
    }

    static bool validMpcReference(const Reference & reference) {
        return reference.position().allFinite() &&
            reference.velocity().allFinite() &&
            reference.acceleration().allFinite() &&
            std::isfinite(reference.yaw()) &&
            std::isnan(reference.yaw_rate()) &&
            std::isnan(reference.yaw_acceleration());
    }

    bool first_planned_baseline_pending_ = false;
};

}  // namespace iii_drone::control::maneuver
