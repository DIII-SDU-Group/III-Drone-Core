#include <iii_drone_core/control/terminal_position_tracking_controller.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

using iii_drone::control::TerminalPositionTrackingController;
using iii_drone::control::Reference;
using iii_drone::control::State;
using iii_drone::types::vector_t;

namespace {

bool finiteVector(const vector_t & value) {
    return value.allFinite();
}

bool nonnegativeFinite(double value) {
    return std::isfinite(value) && value >= 0.0;
}

double secondsBetween(const rclcpp::Time & later, const rclcpp::Time & earlier) {
    return (later - earlier).seconds();
}

std::string timingFailureDetails(
    const rclcpp::Time & receipt, const rclcpp::Time & previous_receipt,
    const rclcpp::Time & emission, const rclcpp::Time & previous_emission,
    double measured_age_s, double maximum_sample_interval_s
) {
    // Format only on failure. A clock-type change must remain a failure without
    // attempting an invalid subtraction between different clock domains.
    const auto signedInterval = [](const rclcpp::Time & current, const rclcpp::Time & previous) {
        return current.get_clock_type() == previous.get_clock_type()
            ? std::to_string(secondsBetween(current, previous))
            : std::string("unavailable_mixed_clock");
    };
    return " (sample_dt_s=" + signedInterval(receipt, previous_receipt) +
        ", emission_dt_s=" + signedInterval(emission, previous_emission) +
        ", current_receipt_ns=" + std::to_string(receipt.nanoseconds()) +
        ", previous_receipt_ns=" + std::to_string(previous_receipt.nanoseconds()) +
        ", current_emission_ns=" + std::to_string(emission.nanoseconds()) +
        ", previous_emission_ns=" + std::to_string(previous_emission.nanoseconds()) +
        ", measured_age_s=" + std::to_string(measured_age_s) +
        ", current_receipt_clock_type=" + std::to_string(static_cast<int>(receipt.get_clock_type())) +
        ", previous_receipt_clock_type=" + std::to_string(static_cast<int>(previous_receipt.get_clock_type())) +
        ", current_emission_clock_type=" + std::to_string(static_cast<int>(emission.get_clock_type())) +
        ", previous_emission_clock_type=" + std::to_string(static_cast<int>(previous_emission.get_clock_type())) +
        ", maximum_sample_interval_s=" + std::to_string(maximum_sample_interval_s) + ")";
}

}  // namespace

TerminalPositionTrackingController::TerminalPositionTrackingController(Reference nominal_reference)
    : TerminalPositionTrackingController(std::move(nominal_reference), Limits{}) {}

TerminalPositionTrackingController::TerminalPositionTrackingController(
    Reference nominal_reference,
    Limits limits
) : nominal_reference_(std::move(nominal_reference)),
    limits_(limits),
    last_output_(nominal_reference_) {
    if (
        !nominal_reference_.position().allFinite() || !std::isfinite(nominal_reference_.yaw()) ||
        !nominal_reference_.velocity().allFinite() || !nominal_reference_.acceleration().allFinite() ||
        !std::isfinite(nominal_reference_.yaw_rate()) ||
        !std::isfinite(nominal_reference_.yaw_acceleration()) ||
        nominal_reference_.velocity().norm() > 1.0e-6 ||
        nominal_reference_.acceleration().norm() > 1.0e-6 ||
        std::abs(nominal_reference_.yaw_rate()) > 1.0e-6 ||
        std::abs(nominal_reference_.yaw_acceleration()) > 1.0e-6
    ) {
        throw std::invalid_argument("terminal position tracking requires a finite stationary nominal reference");
    }
    if (
        !nonnegativeFinite(limits_.max_offset_m) ||
        !std::isfinite(limits_.max_offset_speed_m_s) || limits_.max_offset_speed_m_s <= 0.0 ||
        !std::isfinite(limits_.max_offset_acceleration_m_s2) || limits_.max_offset_acceleration_m_s2 <= 0.0 ||
        !std::isfinite(limits_.max_offset_jerk_m_s3) || limits_.max_offset_jerk_m_s3 <= 0.0 ||
        !std::isfinite(limits_.integral_gain_per_s) || limits_.integral_gain_per_s <= 0.0 ||
        !nonnegativeFinite(limits_.arrival_tolerance_m) ||
        !std::isfinite(limits_.maximum_odometry_age_s) || limits_.maximum_odometry_age_s <= 0.0 ||
        !std::isfinite(limits_.maximum_sample_interval_s) || limits_.maximum_sample_interval_s <= 0.0 ||
        !std::isfinite(limits_.maximum_sample_gap_s) || limits_.maximum_sample_gap_s < limits_.maximum_sample_interval_s ||
        !nonnegativeFinite(limits_.maximum_future_stamp_s) ||
        !std::isfinite(limits_.maximum_tracking_time_s) || limits_.maximum_tracking_time_s <= 0.0 ||
        !std::isfinite(limits_.authority_exhaustion_time_s) || limits_.authority_exhaustion_time_s <= 0.0
    ) {
        throw std::invalid_argument("terminal position tracking limits must be finite and positive where required");
    }
}

bool TerminalPositionTrackingController::Update(
    const State & state,
    const rclcpp::Time & odometry_stamp,
    uint8_t reset_counter,
    const rclcpp::Time & emission_stamp,
    double safe_offset_radius_m,
    Reference & output,
    std::string & failure_reason
) {
    return Update(state, odometry_stamp,
        PositionContinuityIdentity{0, 0, reset_counter, false}, emission_stamp,
        safe_offset_radius_m, output, failure_reason);
}

bool TerminalPositionTrackingController::Update(
    const State & state,
    const rclcpp::Time & odometry_stamp,
    const PositionContinuityIdentity & position_continuity,
    const rclcpp::Time & emission_stamp,
    double safe_offset_radius_m,
    Reference & output,
    std::string & failure_reason
) {
    output = last_output_;
    failure_reason.clear();
    if (committed_stop_active_) {
        return fail("terminal fault stop already owns the command", output, failure_reason);
    }
    if (!std::isfinite(safe_offset_radius_m) || safe_offset_radius_m < 0.0) {
        return fail("cable clearance leaves no valid terminal correction authority", output, failure_reason);
    }
    const vector_t measured_position = state.position();
    const vector_t measured_velocity = state.velocity();
    if (!finiteVector(measured_position) || !finiteVector(measured_velocity)) {
        return fail("terminal tracking received non-finite odometry", output, failure_reason);
    }
    if (emission_stamp.get_clock_type() != odometry_stamp.get_clock_type()) {
        return fail("terminal tracking odometry and command clocks differ", output, failure_reason);
    }
    const double age_s = secondsBetween(emission_stamp, odometry_stamp);
    if (!std::isfinite(age_s) || age_s < -limits_.maximum_future_stamp_s || age_s > limits_.maximum_odometry_age_s) {
        const char * cause = !std::isfinite(age_s) ? "nonfinite" :
            (age_s < -limits_.maximum_future_stamp_s ? "future" : "stale");
        return fail("terminal tracking odometry is stale or from the future"
            " (cause=" + std::string(cause) +
            ", age_s=" + std::to_string(age_s) +
            ", receipt_ns=" + std::to_string(odometry_stamp.nanoseconds()) +
            ", emission_ns=" + std::to_string(emission_stamp.nanoseconds()) +
            ", maximum_odometry_age_s=" +
                std::to_string(limits_.maximum_odometry_age_s) +
            ", maximum_future_stamp_s=" +
                std::to_string(limits_.maximum_future_stamp_s) + ")",
            output, failure_reason);
    }
    if (!finiteVector(nominal_reference_.position())) {
        return fail("terminal tracking nominal target is invalid", output, failure_reason);
    }

    double dt_s = 0.0;
    if (initialized_) {
        if (!SamePositionContinuity(position_continuity_, position_continuity)) {
            return fail("terminal tracking rejected an odometry reset-counter change"
                " (raw=" + std::to_string(position_continuity_.raw_reset_counter) +
                "->" + std::to_string(position_continuity.raw_reset_counter) +
                ", source_epoch=" + std::to_string(position_continuity_.source_epoch) +
                "->" + std::to_string(position_continuity.source_epoch) +
                ", position_epoch=" + std::to_string(position_continuity_.position_epoch) +
                "->" + std::to_string(position_continuity.position_epoch) + ")",
                output, failure_reason);
        }
        if (rebase_sample_timing_) {
            rebase_sample_timing_ = false;
            // Only a forward pause is forgiven; older samples still fail.
            if (odometry_stamp.get_clock_type() == previous_odometry_stamp_.get_clock_type() &&
                emission_stamp.get_clock_type() == previous_emission_stamp_.get_clock_type() &&
                secondsBetween(odometry_stamp, previous_odometry_stamp_) >= 0.0 &&
                secondsBetween(emission_stamp, previous_emission_stamp_) >= 0.0) {
                previous_odometry_stamp_ = odometry_stamp;
                previous_emission_stamp_ = emission_stamp;
            }
        }
        if (emission_stamp.get_clock_type() != previous_emission_stamp_.get_clock_type() ||
            secondsBetween(emission_stamp, previous_emission_stamp_) < 0.0) {
            return fail("terminal tracking command-emission clock is discontinuous" +
                timingFailureDetails(odometry_stamp, previous_odometry_stamp_,
                    emission_stamp, previous_emission_stamp_, age_s,
                    limits_.maximum_sample_interval_s), output, failure_reason);
        }
        dt_s = secondsBetween(odometry_stamp, previous_odometry_stamp_);
        if (std::isfinite(dt_s) && dt_s > limits_.maximum_sample_interval_s &&
            dt_s <= limits_.maximum_sample_gap_s) {
            // An odometry gap that has ended: re-anchor without integrating
            // across it (the sample's age is checked above).
            previous_odometry_stamp_ = odometry_stamp;
            dt_s = 0.0;
        } else if (!std::isfinite(dt_s) || dt_s < 0.0 || dt_s > limits_.maximum_sample_interval_s) {
            return fail("terminal tracking sample interval is discontinuous" +
                timingFailureDetails(odometry_stamp, previous_odometry_stamp_,
                    emission_stamp, previous_emission_stamp_, age_s,
                    limits_.maximum_sample_interval_s), output, failure_reason);
        }
        if (safe_offset_radius_m + 1.0e-9 < emitted_offset_.norm() ||
            (segment_.active && safe_offset_radius_m + 1.0e-9 < segment_.target.norm())) {
            return fail("terminal cable clearance shrank below the active correction path", output, failure_reason);
        }
        // The reference service and stream publisher can sample the same
        // ROS-clock tick. Validate the current measurement and geometry above,
        // then return the committed command without a second integral step.
        if (secondsBetween(emission_stamp, previous_emission_stamp_) == 0.0) {
            position_continuity_ = position_continuity;
            output = last_output_;
            return true;
        }
    } else {
        initialized_ = true;
        position_continuity_ = position_continuity;
        previous_odometry_stamp_ = odometry_stamp;
    }

    if (dt_s > 0.0) {
        previous_odometry_stamp_ = odometry_stamp;
    }
    position_continuity_ = position_continuity;
    previous_emission_stamp_ = emission_stamp;

    const double available_offset_m = std::min(limits_.max_offset_m, safe_offset_radius_m);
    const vector_t error = nominal_reference_.position() - measured_position;
    if (!quiescence_requested_ && dt_s > 0.0 && available_offset_m > 0.0) {
        const vector_t requested = integral_target_offset_ + limits_.integral_gain_per_s * dt_s * error;
        const double requested_norm = requested.norm();
        integral_target_offset_ = requested_norm > available_offset_m
            ? requested * (available_offset_m / requested_norm)
            : requested;
    }

    if (!quiescence_requested_) {
        if (error.norm() <= limits_.arrival_tolerance_m) {
            nonconvergence_active_ = false;
        } else if (!nonconvergence_active_) {
            nonconvergence_active_ = true;
            nonconvergence_start_stamp_ = emission_stamp;
        } else if (secondsBetween(emission_stamp, nonconvergence_start_stamp_) >= limits_.maximum_tracking_time_s) {
            return fail("terminal tracking did not converge within its bounded settling time", output, failure_reason);
        }

        const bool saturated = !segment_.active &&
            emitted_offset_.norm() >= available_offset_m - 1.0e-5 &&
            error.norm() > limits_.arrival_tolerance_m;
        if (saturated) {
            if (!authority_saturated_) {
                authority_saturated_ = true;
                authority_saturation_start_stamp_ = emission_stamp;
            } else if (secondsBetween(emission_stamp, authority_saturation_start_stamp_) >= limits_.authority_exhaustion_time_s) {
                return fail("terminal tracking exhausted cable-safe correction authority", output, failure_reason);
            }
        } else {
            authority_saturated_ = false;
        }
    } else {
        authority_saturated_ = false;
    }

    if (!segment_.active && !quiescence_requested_) {
        beginSegment(emission_stamp);
    }
    if (segment_.active && dt_s >= 0.0) {
        sampleSegment(emission_stamp);
        if (secondsBetween(emission_stamp, segment_.start_stamp) >= segment_.duration_s) {
            emitted_offset_ = segment_.target;
            emitted_velocity_.setZero();
            emitted_acceleration_.setZero();
            segment_.active = false;
            // Do not preempt an active polynomial. A newer integral target is
            // picked up only after this rest-to-rest segment completes.
            if (!quiescence_requested_) {
                beginSegment(emission_stamp);
            }
        }
    }
    buildOutput(emission_stamp);
    output = last_output_;
    return true;
}

void TerminalPositionTrackingController::RequestQuiescence() {
    if (quiescence_requested_) {
        return;
    }
    quiescence_requested_ = true;
    // The active segment is immutable and will reach its fixed endpoint; no
    // subsequent segment may be planned while quiescent. Freeze all pending
    // integral intent at the correction that has actually been emitted now.
    integral_target_offset_ = emitted_offset_;
    nonconvergence_active_ = false;
    authority_saturated_ = false;
}

bool TerminalPositionTrackingController::isQuiescent() const {
    constexpr double rest_tolerance = 1.0e-9;
    return quiescence_requested_ && !segment_.active &&
        emitted_velocity_.norm() <= rest_tolerance &&
        emitted_acceleration_.norm() <= rest_tolerance;
}

void TerminalPositionTrackingController::ResumeAfterHandover() {
    if (initialized_) rebase_sample_timing_ = true;
}

void TerminalPositionTrackingController::ResumeTracking() {
    if (!quiescence_requested_ || committed_stop_active_) {
        return;
    }
    quiescence_requested_ = false;
    integral_target_offset_ = emitted_offset_;
    nonconvergence_active_ = false;
    authority_saturated_ = false;
}

bool TerminalPositionTrackingController::ContinueCommittedStop(
    const rclcpp::Time & emission_stamp,
    Reference & output,
    std::string & failure_reason
) {
    output = last_output_;
    failure_reason.clear();
    // A fault latches ownership even if its first sampling attempt has an
    // invalid clock. No subsequent feedback update may restart integration.
    committed_stop_active_ = true;
    RequestQuiescence();
    const auto previous = stop_previous_emission_stamp_
        ? stop_previous_emission_stamp_
        : (initialized_ ? std::optional<rclcpp::Time>(previous_emission_stamp_) : std::nullopt);
    if (previous && (emission_stamp.get_clock_type() != previous->get_clock_type() ||
        secondsBetween(emission_stamp, *previous) < 0.0)) {
        return fail("terminal fault-stop command clock is discontinuous", output, failure_reason);
    }
    if (segment_.active) {
        sampleSegment(emission_stamp);
        if (secondsBetween(emission_stamp, segment_.start_stamp) >= segment_.duration_s) {
            emitted_offset_ = segment_.target;
            emitted_velocity_.setZero();
            emitted_acceleration_.setZero();
            segment_.active = false;
        }
    }
    // In particular, do not replace this committed segment with a generic
    // braking curve: such a curve can overshoot the certified offset ball.
    buildOutput(emission_stamp);
    stop_previous_emission_stamp_ = emission_stamp;
    output = last_output_;
    return true;
}

bool TerminalPositionTrackingController::fail(
    const std::string & reason,
    Reference & output,
    std::string & failure_reason
) {
    failure_reason = reason;
    output = last_output_;
    return false;
}

void TerminalPositionTrackingController::beginSegment(const rclcpp::Time & stamp) {
    const vector_t difference = integral_target_offset_ - emitted_offset_;
    const double distance = difference.norm();
    if (distance <= 1.0e-8) {
        segment_.active = false;
        emitted_velocity_.setZero();
        emitted_acceleration_.setZero();
        return;
    }

    double duration_s = 0.0;
    constexpr double maximum_first_derivative = 35.0 / 16.0;
    constexpr double maximum_second_derivative = 84.0 / (5.0 * 2.2360679774997896964);
    constexpr double maximum_third_derivative = 52.5;
    if (limits_.max_offset_speed_m_s > 0.0) {
        duration_s = std::max(duration_s, maximum_first_derivative * distance / limits_.max_offset_speed_m_s);
    }
    if (limits_.max_offset_acceleration_m_s2 > 0.0) {
        duration_s = std::max(duration_s, std::sqrt(maximum_second_derivative * distance / limits_.max_offset_acceleration_m_s2));
    }
    if (limits_.max_offset_jerk_m_s3 > 0.0) {
        duration_s = std::max(duration_s, std::cbrt(maximum_third_derivative * distance / limits_.max_offset_jerk_m_s3));
    }
    if (!std::isfinite(duration_s) || duration_s <= 0.0) {
        return;
    }
    segment_.start = emitted_offset_;
    segment_.target = integral_target_offset_;
    segment_.start_stamp = stamp;
    segment_.duration_s = duration_s;
    segment_.active = true;
}

void TerminalPositionTrackingController::sampleSegment(const rclcpp::Time & stamp) {
    const double elapsed_s = std::clamp(secondsBetween(stamp, segment_.start_stamp), 0.0, segment_.duration_s);
    const double u = elapsed_s / segment_.duration_s;
    const double u2 = u * u;
    const double u3 = u2 * u;
    const double u4 = u3 * u;
    const double u5 = u4 * u;
    const double u6 = u5 * u;
    const double u7 = u6 * u;
    const double s = 35.0 * u4 - 84.0 * u5 + 70.0 * u6 - 20.0 * u7;
    const double ds = 140.0 * u3 - 420.0 * u4 + 420.0 * u5 - 140.0 * u6;
    const double d2s = 420.0 * u2 - 1680.0 * u3 + 2100.0 * u4 - 840.0 * u5;
    const vector_t difference = segment_.target - segment_.start;
    emitted_offset_ = segment_.start + difference * s;
    emitted_velocity_ = difference * (ds / segment_.duration_s);
    emitted_acceleration_ = difference * (d2s / (segment_.duration_s * segment_.duration_s));
}

void TerminalPositionTrackingController::buildOutput(const rclcpp::Time & stamp) {
    last_output_ = Reference(
        nominal_reference_.position() + emitted_offset_,
        nominal_reference_.yaw(),
        nominal_reference_.velocity() + emitted_velocity_,
        nominal_reference_.yaw_rate(),
        nominal_reference_.acceleration() + emitted_acceleration_,
        nominal_reference_.yaw_acceleration(),
        stamp
    );
}

const Reference & TerminalPositionTrackingController::lastOutput() const {
    return last_output_;
}

double TerminalPositionTrackingController::integralTargetOffsetNorm() const {
    return integral_target_offset_.norm();
}

double TerminalPositionTrackingController::emittedOffsetNorm() const {
    return emitted_offset_.norm();
}
