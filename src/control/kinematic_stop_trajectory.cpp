#include <iii_drone_core/control/kinematic_stop_trajectory.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

using namespace iii_drone::control;
using namespace iii_drone::types;

namespace {

constexpr double kMinimumDurationS = 0.05;
constexpr double kMaximumDurationS = 60.0;
constexpr int kLimitSamples = 200;
constexpr double kLimitTolerance = 1.0e-9;

bool positiveFinite(double value) {
    return std::isfinite(value) && value > 0.0;
}

bool finiteReference(const Reference & reference) {
    return reference.position().allFinite() &&
        reference.velocity().allFinite() &&
        reference.acceleration().allFinite() &&
        std::isfinite(reference.yaw()) &&
        std::isfinite(reference.yaw_rate()) &&
        std::isfinite(reference.yaw_acceleration());
}

struct HermiteTerms {
    double velocity_from_velocity;
    double velocity_from_acceleration;
    double acceleration_from_velocity;
    double acceleration_from_acceleration;
    double jerk_from_velocity;
    double jerk_from_acceleration;
    double position_from_velocity;
    double position_from_acceleration;
};

HermiteTerms terms(double u, double duration_s) {
    const double u2 = u * u;
    const double u3 = u2 * u;
    const double u4 = u3 * u;
    return {
        2.0 * u3 - 3.0 * u2 + 1.0,
        (u3 - 2.0 * u2 + u) * duration_s,
        (6.0 * u2 - 6.0 * u) / duration_s,
        3.0 * u2 - 4.0 * u + 1.0,
        (12.0 * u - 6.0) / (duration_s * duration_s),
        (6.0 * u - 4.0) / duration_s,
        (0.5 * u4 - u3 + u) * duration_s,
        (0.25 * u4 - (2.0 / 3.0) * u3 + 0.5 * u2) * duration_s * duration_s,
    };
}

}  // namespace

KinematicStopTrajectory::KinematicStopTrajectory(
    const Reference & initial,
    const KinematicStopLimits & limits
) : initial_(initial), limits_(limits) {
    if (!finiteReference(initial_)) {
        throw std::invalid_argument("kinematic stop initial reference must be finite");
    }
    if (
        !positiveFinite(limits_.max_acceleration_m_s2) ||
        !positiveFinite(limits_.max_jerk_m_s3) ||
        !positiveFinite(limits_.max_yaw_acceleration_rad_s2) ||
        !positiveFinite(limits_.max_yaw_jerk_rad_s3)
    ) {
        throw std::invalid_argument("kinematic stop limits must be finite and positive");
    }
    if (
        initial_.velocity().norm() <= kLimitTolerance &&
        initial_.acceleration().norm() <= kLimitTolerance &&
        std::abs(initial_.yaw_rate()) <= kLimitTolerance &&
        std::abs(initial_.yaw_acceleration()) <= kLimitTolerance
    ) {
        return;
    }

    duration_s_ = kMinimumDurationS;
    while (!satisfiesLimits(duration_s_) && duration_s_ < kMaximumDurationS) {
        duration_s_ *= 1.05;
    }
    if (duration_s_ > kMaximumDurationS || !satisfiesLimits(duration_s_)) {
        throw std::runtime_error("unable to construct bounded kinematic stop trajectory");
    }
}

Reference KinematicStopTrajectory::sample(double elapsed_s, const rclcpp::Time & stamp) const {
    if (duration_s_ <= 0.0 || elapsed_s >= duration_s_) {
        return terminalReference(stamp);
    }

    const double u = std::clamp(elapsed_s / duration_s_, 0.0, 1.0);
    const HermiteTerms h = terms(u, duration_s_);
    const vector_t velocity =
        h.velocity_from_velocity * initial_.velocity() +
        h.velocity_from_acceleration * initial_.acceleration();
    const vector_t acceleration =
        h.acceleration_from_velocity * initial_.velocity() +
        h.acceleration_from_acceleration * initial_.acceleration();
    const point_t position =
        initial_.position() +
        h.position_from_velocity * initial_.velocity() +
        h.position_from_acceleration * initial_.acceleration();
    const double yaw_rate =
        h.velocity_from_velocity * initial_.yaw_rate() +
        h.velocity_from_acceleration * initial_.yaw_acceleration();
    const double yaw_acceleration =
        h.acceleration_from_velocity * initial_.yaw_rate() +
        h.acceleration_from_acceleration * initial_.yaw_acceleration();
    const double yaw =
        initial_.yaw() +
        h.position_from_velocity * initial_.yaw_rate() +
        h.position_from_acceleration * initial_.yaw_acceleration();

    return Reference(position, yaw, velocity, yaw_rate, acceleration, yaw_acceleration, stamp);
}

Reference KinematicStopTrajectory::terminalReference(const rclcpp::Time & stamp) const {
    if (duration_s_ <= 0.0) {
        return Reference(
            initial_.position(), initial_.yaw(), vector_t::Zero(), 0.0,
            vector_t::Zero(), 0.0, stamp
        );
    }
    const HermiteTerms h = terms(1.0, duration_s_);
    return Reference(
        initial_.position() +
            h.position_from_velocity * initial_.velocity() +
            h.position_from_acceleration * initial_.acceleration(),
        initial_.yaw() +
            h.position_from_velocity * initial_.yaw_rate() +
            h.position_from_acceleration * initial_.yaw_acceleration(),
        vector_t::Zero(),
        0.0,
        vector_t::Zero(),
        0.0,
        stamp
    );
}

double KinematicStopTrajectory::durationS() const {
    return duration_s_;
}

bool KinematicStopTrajectory::satisfiesLimits(double duration_s) const {
    // An incoming acceleration above the configured stop limit cannot be removed
    // instantaneously without violating the jerk limit. Preserve it at t=0,
    // never increase its magnitude, and jerk-limit the transition to zero.
    const double acceleration_limit = std::max(
        limits_.max_acceleration_m_s2,
        static_cast<double>(initial_.acceleration().norm())
    );
    const double yaw_acceleration_limit = std::max(
        limits_.max_yaw_acceleration_rad_s2,
        std::abs(initial_.yaw_acceleration())
    );
    for (int index = 0; index <= kLimitSamples; ++index) {
        const double u = static_cast<double>(index) / kLimitSamples;
        const HermiteTerms h = terms(u, duration_s);
        const vector_t acceleration =
            h.acceleration_from_velocity * initial_.velocity() +
            h.acceleration_from_acceleration * initial_.acceleration();
        const vector_t jerk =
            h.jerk_from_velocity * initial_.velocity() +
            h.jerk_from_acceleration * initial_.acceleration();
        const double yaw_acceleration =
            h.acceleration_from_velocity * initial_.yaw_rate() +
            h.acceleration_from_acceleration * initial_.yaw_acceleration();
        const double yaw_jerk =
            h.jerk_from_velocity * initial_.yaw_rate() +
            h.jerk_from_acceleration * initial_.yaw_acceleration();
        if (
            acceleration.norm() > acceleration_limit + kLimitTolerance ||
            jerk.norm() > limits_.max_jerk_m_s3 + kLimitTolerance ||
            std::abs(yaw_acceleration) > yaw_acceleration_limit + kLimitTolerance ||
            std::abs(yaw_jerk) > limits_.max_yaw_jerk_rad_s3 + kLimitTolerance
        ) {
            return false;
        }
    }
    return true;
}
