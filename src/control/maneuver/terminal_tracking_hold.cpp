#include <iii_drone_core/control/maneuver/terminal_tracking_hold.hpp>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

using iii_drone::control::Reference;
using iii_drone::control::maneuver::TerminalTrackingHold;

TerminalTrackingHold::TerminalTrackingHold(
    Reference nominal,
    iii_drone::control::CombinedDroneAwarenessHandler::SharedPtr awareness,
    rclcpp::Clock::SharedPtr clock,
    Clearance minimum_cable_distance,
    double required_clearance_m,
    iii_drone::control::TerminalPositionTrackingController::Limits limits
) : nominal_(std::move(nominal)), awareness_(std::move(awareness)),
    clock_(std::move(clock)), minimum_cable_distance_(std::move(minimum_cable_distance)),
    required_clearance_m_(required_clearance_m), controller_(nominal_, limits),
    last_command_(nominal_) {
    if (!awareness_ || !clock_ ||
        (minimum_cable_distance_ &&
         (!std::isfinite(required_clearance_m_) || required_clearance_m_ <= 0.0))) {
        throw std::invalid_argument("terminal hold requires measured odometry and valid geometry policy");
    }
}

Reference TerminalTrackingHold::GetReference() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (phase_ == Phase::Stopping) {
        const auto now = clock_->now();
        Reference command;
        std::string reason;
        if (!controller_.ContinueCommittedStop(now, command, reason)) {
            phase_ = Phase::Unrecoverable;
            failure_reason_ += "; committed stop unavailable: " + reason;
            return last_command_;
        }
        last_command_ = command;
        if (controller_.isQuiescent()) phase_ = Phase::Degraded;
        return last_command_;
    }
    if (phase_ == Phase::Degraded) {
        return last_command_.CopyWithNewStamp(clock_->now());
    }
    if (phase_ == Phase::Unrecoverable) return last_command_;

    // Snapshot the measured sample before sampling emission time. A newer
    // odometry callback between these operations will then be handled on the
    // next call, rather than appearing spuriously future-dated in this one.
    const auto capture_started = std::chrono::steady_clock::now();
    const auto measured = awareness_->GetMeasuredOdometry();
    const auto capture_finished = std::chrono::steady_clock::now();
    const auto now = clock_->now();
    const auto clock_finished = std::chrono::steady_clock::now();
    if (!measured) {
        beginFailure("measured odometry unavailable", now);
        return last_command_;
    }
    double radius_m = 0.4;
    if (minimum_cable_distance_) {
        constexpr double numeric_margin_m = 1.0e-3;
        const double nominal_clearance_m = minimum_cable_distance_(nominal_.position());
        const double measured_clearance_m = minimum_cable_distance_(measured->state.position());
        if (!std::isfinite(nominal_clearance_m) || !std::isfinite(measured_clearance_m) ||
            measured_clearance_m < required_clearance_m_ + numeric_margin_m) {
            beginFailure("measured cable clearance is invalid or exhausted", now);
            return last_command_;
        }
        radius_m = std::min(
            radius_m, nominal_clearance_m - required_clearance_m_ - numeric_margin_m
        );
    }
    Reference command;
    std::string reason;
    if (!controller_.Update(
            measured->state, measured->receipt_stamp,
            measured->position_continuity.source_qualified ||
                measured->position_continuity.source_epoch != 0 ||
                measured->position_continuity.position_epoch != 0
                ? measured->position_continuity
                : PositionContinuityIdentity{0, 0, measured->reset_counter, false},
            now, radius_m, command, reason)) {
        if (reason.rfind("terminal tracking odometry is stale or from the future", 0) == 0 ||
            reason.rfind("terminal tracking sample interval is discontinuous", 0) == 0 ||
            reason.rfind("terminal tracking command-emission clock is discontinuous", 0) == 0) {
            const auto failure_observed = std::chrono::steady_clock::now();
            const auto ingress = awareness_->TryGetOdometryIngressDiagnostics();
            const auto ingress_copy_finished = std::chrono::steady_clock::now();
            const auto ns = [](const auto duration) {
                return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
            };
            std::ostringstream details;
            details << "controller_reason=" << reason
                    << " captured_source_us=" << measured->source_sample_timestamp_us
                    << " captured_raw_reset=" << static_cast<unsigned>(measured->reset_counter)
                    << " captured_receipt_ros_ns=" << measured->receipt_stamp.nanoseconds()
                    << " emission_ros_ns=" << now.nanoseconds()
                    << " capture_started_steady_ns=" << ns(capture_started.time_since_epoch())
                    << " capture_finished_steady_ns=" << ns(capture_finished.time_since_epoch())
                    << " clock_finished_steady_ns=" << ns(clock_finished.time_since_epoch())
                    << " failure_observed_steady_ns=" << ns(failure_observed.time_since_epoch())
                    << " ingress_copy_finished_steady_ns=" << ns(ingress_copy_finished.time_since_epoch())
                    << " capture_read_steady_ns=" << ns(capture_finished - capture_started)
                    << " capture_to_clock_steady_ns=" << ns(clock_finished - capture_started)
                    << " clock_to_failure_steady_ns=" << ns(failure_observed - clock_finished)
                    << " capture_to_failure_steady_ns=" << ns(failure_observed - capture_started);
            if (!ingress.available) {
                details << " ingress=" << (ingress.busy ? "busy" : "unavailable");
            } else {
                details << " ingress=available"
                        << " latest_available=" << ingress.latest_available
                        << " latest_source_us=" << ingress.latest_source_sample_timestamp_us
                        << " latest_raw_reset=" << static_cast<unsigned>(ingress.latest_reset_counter)
                        << " latest_receipt_ros_ns=" << ingress.latest_receipt_ros_ns
                        << " latest_accepted_steady_ns=" << ingress.latest_accepted_steady_ns
                        << " ingress_total=" << ingress.total_callbacks
                        << " ingress_count=" << ingress.history_count
                        << " ingress_fields=source_us,raw_reset,callback_ros_ns,entry_steady_ns,lock_steady_ns,accepted_steady_ns,done_steady_ns,accepted,pending_before,pending_after"
                        << " ingress_history=[";
                for (size_t i = 0; i < ingress.history_count; ++i) {
                    const auto & event = ingress.history[i];
                    if (i) details << ';';
                    details << event.source_sample_timestamp_us << ','
                            << static_cast<unsigned>(event.reset_counter) << ','
                            << event.callback_receipt_ros_ns << ','
                            << event.callback_entry_steady_ns << ','
                            << event.lock_acquired_steady_ns << ','
                            << event.accepted_steady_ns << ','
                            << event.completed_steady_ns << ','
                            << event.accepted << ','
                            << event.pending_before << ',' << event.pending_after;
                }
                details << ']';
            }
            last_freshness_diagnostic_ = details.str();
            RCLCPP_ERROR(rclcpp::get_logger("terminal_tracking_hold"),
                "Terminal tracking first-fault timing evidence: %s",
                last_freshness_diagnostic_.c_str());
        }
        beginFailure(reason, now);
        return last_command_;
    }
    last_command_ = command;
    return command;
}

void TerminalTrackingHold::beginFailure(const std::string & reason, const rclcpp::Time & now) {
    if (phase_ != Phase::Tracking) {
        return;
    }
    failure_reason_ = reason;
    Reference command;
    std::string stop_reason;
    if (!controller_.ContinueCommittedStop(now, command, stop_reason)) {
        failure_reason_ += "; committed stop unavailable: " + stop_reason;
        phase_ = Phase::Unrecoverable;
        return;
    }
    last_command_ = command;
    phase_ = controller_.isQuiescent() ? Phase::Degraded : Phase::Stopping;
}

bool TerminalTrackingHold::RequestQuiescence() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (phase_ != Phase::Tracking) return false;
    controller_.RequestQuiescence();
    return true;
}

bool TerminalTrackingHold::isQuiescent() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return phase_ == Phase::Tracking && controller_.isQuiescent();
}

bool TerminalTrackingHold::ResumeTracking() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (phase_ != Phase::Tracking) return false;
    controller_.ResumeTracking();
    return true;
}

void TerminalTrackingHold::Fail(const std::string & reason) {
    std::lock_guard<std::mutex> lock(mutex_);
    beginFailure(reason, clock_->now());
}

TerminalTrackingHold::Phase TerminalTrackingHold::phase() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return phase_;
}

std::string TerminalTrackingHold::failureReason() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return failure_reason_;
}

Reference TerminalTrackingHold::lastCommand() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_command_;
}

Reference TerminalTrackingHold::nominalReference() const {
    return nominal_;
}
