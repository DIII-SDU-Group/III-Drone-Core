#include <iii_drone_core/control/maneuver/maneuver_reference_safety_guard.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

using namespace iii_drone::control;
using namespace iii_drone::control::maneuver;

namespace {

constexpr double kTwoPi = 6.28318530717958647692;

bool nonNegativeFinite(double value) {
    return std::isfinite(value) && value >= 0.0;
}

bool finiteOrNan(double value) {
    return std::isfinite(value) || std::isnan(value);
}

bool validReference(const Reference & reference) {
    for (int axis = 0; axis < 3; ++axis) {
        if (
            !finiteOrNan(reference.position()(axis)) ||
            !finiteOrNan(reference.velocity()(axis)) ||
            !finiteOrNan(reference.acceleration()(axis))
        ) {
            return false;
        }
        if (
            !std::isfinite(reference.position()(axis)) &&
            !std::isfinite(reference.velocity()(axis)) &&
            !std::isfinite(reference.acceleration()(axis))
        ) {
            return false;
        }
    }
    if (
        !finiteOrNan(reference.yaw()) || !finiteOrNan(reference.yaw_rate()) ||
        !finiteOrNan(reference.yaw_acceleration())
    ) {
        return false;
    }
    return std::isfinite(reference.yaw()) || std::isfinite(reference.yaw_rate());
}

template <typename Expected>
double commonFiniteVectorError(
    const iii_drone::types::vector_t & current,
    const iii_drone::types::vector_t & previous,
    Expected expected
) {
    double squared_error = 0.0;
    for (int axis = 0; axis < 3; ++axis) {
        if (!std::isfinite(current(axis)) || !std::isfinite(previous(axis))) {
            continue;
        }
        const double delta = current(axis) - expected(axis);
        squared_error += delta * delta;
    }
    return std::sqrt(squared_error);
}

double angleError(double actual, double expected) {
    return std::abs(std::remainder(actual - expected, kTwoPi));
}

}  // namespace

ManeuverReferenceSafetyGuard::ManeuverReferenceSafetyGuard(
    ManeuverReferenceSafetyConfig config
) : config_(std::move(config)) {
    if (config_.loss_timeout.count() <= 0) {
        throw std::invalid_argument("reference loss timeout must be positive");
    }
    if (
        !nonNegativeFinite(config_.max_jerk_m_s3) ||
        !nonNegativeFinite(config_.max_yaw_jerk_rad_s3) ||
        !nonNegativeFinite(config_.position_tolerance_m) ||
        !nonNegativeFinite(config_.velocity_tolerance_m_s) ||
        !nonNegativeFinite(config_.acceleration_tolerance_m_s2) ||
        !nonNegativeFinite(config_.yaw_tolerance_rad) ||
        !nonNegativeFinite(config_.yaw_rate_tolerance_rad_s) ||
        !nonNegativeFinite(config_.yaw_acceleration_tolerance_rad_s2)
    ) {
        throw std::invalid_argument("reference continuity limits must be finite and non-negative");
    }
}

void ManeuverReferenceSafetyGuard::reset() {
    last_reference_.reset();
    last_reference_received_at_.reset();
    fault_latched_ = false;
}

ManeuverReferenceSafetyEvaluation ManeuverReferenceSafetyGuard::observeReference(
    const Reference & reference,
    TimePoint received_at
) {
    ManeuverReferenceSafetyEvaluation evaluation;
    if (fault_latched_) {
        evaluation.decision = ManeuverReferenceSafetyDecision::REJECT_LATCHED;
        evaluation.reason = "reference safety fault is latched";
        return evaluation;
    }
    if (!validReference(reference)) {
        return latchFault(std::move(evaluation), "received invalid maneuver reference channels");
    }
    if (!last_reference_ || !last_reference_received_at_) {
        last_reference_ = reference;
        last_reference_received_at_ = received_at;
        evaluation.decision = ManeuverReferenceSafetyDecision::ACCEPT;
        return evaluation;
    }

    const double elapsed_s = std::max(
        0.0,
        std::chrono::duration<double>(received_at - *last_reference_received_at_).count()
    );
    evaluation.reference_age_s = elapsed_s;
    if (received_at - *last_reference_received_at_ >= config_.loss_timeout) {
        return latchFault(std::move(evaluation), "reference response exceeded delivery deadline");
    }

    const Reference & previous = *last_reference_;
    const double elapsed_s2 = elapsed_s * elapsed_s;
    const double elapsed_s3 = elapsed_s2 * elapsed_s;
    iii_drone::types::point_t expected_position = previous.position();
    iii_drone::types::vector_t expected_velocity = previous.velocity();
    for (int axis = 0; axis < 3; ++axis) {
        if (std::isfinite(expected_position(axis))) {
            expected_position(axis) +=
                (std::isfinite(previous.velocity()(axis)) ? previous.velocity()(axis) * elapsed_s : 0.0) +
                (std::isfinite(previous.acceleration()(axis))
                    ? previous.acceleration()(axis) * (0.5 * elapsed_s2) : 0.0);
        }
        if (std::isfinite(expected_velocity(axis)) && std::isfinite(previous.acceleration()(axis))) {
            expected_velocity(axis) += previous.acceleration()(axis) * elapsed_s;
        }
    }
    const double expected_yaw = std::isfinite(previous.yaw())
        ? previous.yaw() +
            (std::isfinite(previous.yaw_rate()) ? previous.yaw_rate() * elapsed_s : 0.0) +
            (std::isfinite(previous.yaw_acceleration())
                ? previous.yaw_acceleration() * (0.5 * elapsed_s2) : 0.0)
        : previous.yaw();
    const double expected_yaw_rate = std::isfinite(previous.yaw_rate())
        ? previous.yaw_rate() +
            (std::isfinite(previous.yaw_acceleration())
                ? previous.yaw_acceleration() * elapsed_s : 0.0)
        : previous.yaw_rate();

    evaluation.position_error_m = commonFiniteVectorError(
        reference.position(), previous.position(), expected_position
    );
    evaluation.position_limit_m =
        config_.position_tolerance_m + config_.max_jerk_m_s3 * elapsed_s3 / 6.0;
    evaluation.velocity_error_m_s = commonFiniteVectorError(
        reference.velocity(), previous.velocity(), expected_velocity
    );
    evaluation.velocity_limit_m_s =
        config_.velocity_tolerance_m_s + config_.max_jerk_m_s3 * elapsed_s2 / 2.0;
    evaluation.acceleration_error_m_s2 = commonFiniteVectorError(
        reference.acceleration(), previous.acceleration(), previous.acceleration()
    );
    evaluation.acceleration_limit_m_s2 =
        config_.acceleration_tolerance_m_s2 + config_.max_jerk_m_s3 * elapsed_s;
    evaluation.yaw_error_rad =
        std::isfinite(reference.yaw()) && std::isfinite(expected_yaw)
            ? angleError(reference.yaw(), expected_yaw) : 0.0;
    evaluation.yaw_limit_rad =
        config_.yaw_tolerance_rad + config_.max_yaw_jerk_rad_s3 * elapsed_s3 / 6.0;
    evaluation.yaw_rate_error_rad_s =
        std::isfinite(reference.yaw_rate()) && std::isfinite(expected_yaw_rate)
            ? std::abs(reference.yaw_rate() - expected_yaw_rate) : 0.0;
    evaluation.yaw_rate_limit_rad_s =
        config_.yaw_rate_tolerance_rad_s + config_.max_yaw_jerk_rad_s3 * elapsed_s2 / 2.0;
    evaluation.yaw_acceleration_error_rad_s2 =
        std::isfinite(reference.yaw_acceleration()) &&
            std::isfinite(previous.yaw_acceleration())
            ? std::abs(reference.yaw_acceleration() - previous.yaw_acceleration()) : 0.0;
    evaluation.yaw_acceleration_limit_rad_s2 =
        config_.yaw_acceleration_tolerance_rad_s2 + config_.max_yaw_jerk_rad_s3 * elapsed_s;

    if (
        evaluation.position_error_m > evaluation.position_limit_m ||
        evaluation.velocity_error_m_s > evaluation.velocity_limit_m_s ||
        evaluation.acceleration_error_m_s2 > evaluation.acceleration_limit_m_s2 ||
        evaluation.yaw_error_rad > evaluation.yaw_limit_rad ||
        evaluation.yaw_rate_error_rad_s > evaluation.yaw_rate_limit_rad_s ||
        evaluation.yaw_acceleration_error_rad_s2 >
            evaluation.yaw_acceleration_limit_rad_s2
    ) {
        return latchFault(std::move(evaluation), "reference violates continuity envelope");
    }

    last_reference_ = reference;
    last_reference_received_at_ = received_at;
    evaluation.decision = ManeuverReferenceSafetyDecision::ACCEPT;
    return evaluation;
}

ManeuverReferenceSafetyEvaluation ManeuverReferenceSafetyGuard::observeMiss(TimePoint observed_at) {
    ManeuverReferenceSafetyEvaluation evaluation;
    if (fault_latched_) {
        evaluation.decision = ManeuverReferenceSafetyDecision::REJECT_LATCHED;
        evaluation.reason = "reference safety fault is latched";
        return evaluation;
    }
    if (!last_reference_received_at_) {
        evaluation.decision = ManeuverReferenceSafetyDecision::HOLD_LAST;
        evaluation.reason = "waiting for first maneuver reference";
        return evaluation;
    }
    evaluation.reference_age_s = std::max(
        0.0,
        std::chrono::duration<double>(observed_at - *last_reference_received_at_).count()
    );
    if (observed_at - *last_reference_received_at_ >= config_.loss_timeout) {
        return latchFault(std::move(evaluation), "reference delivery deadline expired");
    }
    evaluation.decision = ManeuverReferenceSafetyDecision::HOLD_LAST;
    evaluation.reason = "transient reference miss within delivery deadline";
    return evaluation;
}

bool ManeuverReferenceSafetyGuard::hasAcceptedReference() const {
    return last_reference_.has_value();
}

bool ManeuverReferenceSafetyGuard::faultLatched() const {
    return fault_latched_;
}

ManeuverReferenceSafetyEvaluation ManeuverReferenceSafetyGuard::latchFault(
    ManeuverReferenceSafetyEvaluation evaluation,
    std::string reason
) {
    fault_latched_ = true;
    evaluation.decision = ManeuverReferenceSafetyDecision::BEGIN_STOP;
    evaluation.reason = std::move(reason);
    return evaluation;
}
