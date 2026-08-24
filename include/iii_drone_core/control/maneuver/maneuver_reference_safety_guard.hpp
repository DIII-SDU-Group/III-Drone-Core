#pragma once

#include <chrono>
#include <optional>
#include <string>

#include <iii_drone_core/control/reference.hpp>

namespace iii_drone::control::maneuver {

struct ManeuverReferenceSafetyConfig {
    std::chrono::milliseconds loss_timeout{500};
    double max_jerk_m_s3 = 1.0;
    double max_yaw_jerk_rad_s3 = 1.5;
    double position_tolerance_m = 0.25;
    double velocity_tolerance_m_s = 0.5;
    double acceleration_tolerance_m_s2 = 0.75;
    double yaw_tolerance_rad = 0.35;
    double yaw_rate_tolerance_rad_s = 0.5;
    double yaw_acceleration_tolerance_rad_s2 = 0.75;
};

enum class ManeuverReferenceSafetyDecision {
    ACCEPT,
    HOLD_LAST,
    BEGIN_STOP,
    REJECT_LATCHED,
};

struct ManeuverReferenceSafetyEvaluation {
    ManeuverReferenceSafetyDecision decision = ManeuverReferenceSafetyDecision::HOLD_LAST;
    std::string reason;
    double reference_age_s = 0.0;
    double position_error_m = 0.0;
    double position_limit_m = 0.0;
    double velocity_error_m_s = 0.0;
    double velocity_limit_m_s = 0.0;
    double acceleration_error_m_s2 = 0.0;
    double acceleration_limit_m_s2 = 0.0;
    double yaw_error_rad = 0.0;
    double yaw_limit_rad = 0.0;
    double yaw_rate_error_rad_s = 0.0;
    double yaw_rate_limit_rad_s = 0.0;
    double yaw_acceleration_error_rad_s2 = 0.0;
    double yaw_acceleration_limit_rad_s2 = 0.0;
};

class ManeuverReferenceSafetyGuard {
public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    explicit ManeuverReferenceSafetyGuard(ManeuverReferenceSafetyConfig config);

    void reset();
    ManeuverReferenceSafetyEvaluation observeReference(
        const Reference & reference,
        TimePoint received_at = Clock::now()
    );
    ManeuverReferenceSafetyEvaluation observeMiss(TimePoint observed_at = Clock::now());
    bool hasAcceptedReference() const;
    bool faultLatched() const;

private:
    ManeuverReferenceSafetyEvaluation latchFault(
        ManeuverReferenceSafetyEvaluation evaluation,
        std::string reason
    );

    ManeuverReferenceSafetyConfig config_;
    std::optional<Reference> last_reference_;
    std::optional<TimePoint> last_reference_received_at_;
    bool fault_latched_ = false;
};

}  // namespace iii_drone::control::maneuver
