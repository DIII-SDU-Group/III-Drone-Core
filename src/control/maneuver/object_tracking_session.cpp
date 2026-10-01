#include <iii_drone_core/control/maneuver/object_tracking_session.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

using iii_drone::control::Reference;
using iii_drone::control::maneuver::ObjectTrackingSession;
using iii_drone::types::vector_t;

namespace {
bool stopAboveFloor(const iii_drone::control::KinematicStopTrajectory & stop,
                    const Reference & initial, double floor_m) {
    const double duration = stop.durationS();
    const auto safe = [&stop, floor_m](double elapsed) {
        return stop.sample(elapsed).position()(2) >= floor_m - 1.0e-5;
    };
    if (!safe(0.0) || !safe(duration)) return false;
    const double velocity = initial.velocity()(2);
    const double denominator = 2.0 * velocity + duration * initial.acceleration()(2);
    if (duration > 0.0 && std::abs(denominator) > 1.0e-12) {
        const double u = -velocity / denominator;
        if (u > 0.0 && u < 1.0 && !safe(u * duration)) return false;
    }
    return true;
}
}

ObjectTrackingSession::ObjectTrackingSession(
    Planner planner,
    Reference initial_command,
    std::string request_identity,
    uint64_t execution_id,
    const rclcpp::Time & started_at,
    double command_floor_m,
    Limits limits
) : planner_(std::move(planner)), limits_(limits),
    initial_command_(initial_command.CopyWithNewStamp(started_at)),
    last_command_(initial_command_), last_output_(initial_command_),
    last_command_emission_(started_at), command_floor_m_(command_floor_m),
    request_identity_(std::move(request_identity)), execution_id_(execution_id) {
    if (!planner_ || request_identity_.empty() || execution_id_ == 0 ||
        !finiteCommand(initial_command_) || !std::isfinite(command_floor_m_) ||
        initial_command_.position()(2) < command_floor_m_ - 1.0e-5 ||
        !std::isfinite(limits_.gain_per_s) || limits_.gain_per_s <= 0.0 ||
        !std::isfinite(limits_.max_offset_m) || limits_.max_offset_m <= 0.0 ||
        !std::isfinite(limits_.maximum_sample_interval_s) ||
        limits_.maximum_sample_interval_s <= 0.0 ||
        !std::isfinite(limits_.maximum_sample_gap_s) ||
        limits_.maximum_sample_gap_s < limits_.maximum_sample_interval_s ||
        !std::isfinite(limits_.maximum_odometry_age_s) ||
        limits_.maximum_odometry_age_s <= 0.0 ||
        !std::isfinite(limits_.maximum_command_age_s) ||
        limits_.maximum_command_age_s <= 0.0 ||
        !std::isfinite(limits_.maximum_future_stamp_s) ||
        limits_.maximum_future_stamp_s < 0.0 ||
        !std::isfinite(limits_.authority_exhaustion_time_s) ||
        limits_.authority_exhaustion_time_s <= 0.0 ||
        !std::isfinite(limits_.tracking_tolerance_m) ||
        limits_.tracking_tolerance_m < 0.0 ||
        !std::isfinite(limits_.maximum_uncorrected_error_time_s) ||
        limits_.maximum_uncorrected_error_time_s <= 0.0) {
        throw std::invalid_argument("object tracking requires a finite seed, owner, planner and limits");
    }
    certified_stop_.emplace(initial_command_, limits_.cancellation_config.limits);
    if (!certifiedStopAboveFloor(*certified_stop_, initial_command_)) {
        throw std::invalid_argument("object tracking initial command lacks a safe bounded stop");
    }
}

bool ObjectTrackingSession::finiteCommand(const Reference & command) {
    return command.position().allFinite() && command.velocity().allFinite() &&
        command.acceleration().allFinite() && std::isfinite(command.yaw()) &&
        std::isfinite(command.yaw_rate()) && std::isfinite(command.yaw_acceleration());
}

bool ObjectTrackingSession::CanCertifyInitialSeed(
    const Reference & command, double command_floor_m,
    const ControlledCancellationConfig & config, std::string & reason) {
    if (!finiteCommand(command) || !std::isfinite(command_floor_m) ||
        command.position()(2) < command_floor_m - 1.0e-5) {
        reason = "object tracking initial command or floor is invalid";
        return false;
    }
    try {
        const iii_drone::control::KinematicStopTrajectory stop(command, config.limits);
        if (!stopAboveFloor(stop, command, command_floor_m)) {
            reason = "object tracking initial command lacks a floor-safe bounded stop";
            return false;
        }
    } catch (const std::exception & error) {
        reason = error.what();
        return false;
    }
    reason.clear();
    return true;
}

bool ObjectTrackingSession::failLocked(
    const std::string & reason, const rclcpp::Time & stamp,
    Reference & output, std::string & failure_reason
) {
    failed_ = true;
    failure_reason_ = reason;
    failure_reason = reason;
    output = sampleFailureStopLocked(stamp);
    return false;
}

bool ObjectTrackingSession::certifiedStopAboveFloor(
    const iii_drone::control::KinematicStopTrajectory & stop,
    const Reference & initial) const {
    return stopAboveFloor(stop, initial, command_floor_m_);
}

Reference ObjectTrackingSession::sampleFailureStopLocked(const rclcpp::Time & stamp) {
    if (!certified_stop_ || stamp.get_clock_type() != last_command_emission_.get_clock_type()) {
        unrecoverable_ = true;
        return last_output_;
    }
    if (!stop_started_at_) stop_started_at_ = stamp;
    const double elapsed = (stamp - *stop_started_at_).seconds();
    if (!std::isfinite(elapsed) || elapsed < 0.0) {
        unrecoverable_ = true;
        return last_output_;
    }
    last_output_ = certified_stop_->sample(elapsed, stamp);
    stop_complete_ = elapsed >= certified_stop_->durationS();
    return last_output_;
}

bool ObjectTrackingSession::Compute(
    const Reference & nominal_target,
    const iii_drone::control::MeasuredOdometrySnapshot & measured,
    const rclcpp::Time & emission_stamp,
    const std::string & request_identity,
    uint64_t execution_id,
    double minimum_altitude_m,
    double maximum_offset_m,
    Reference & output,
    std::string & reason,
    bool * first_timing_fault
) {
    if (first_timing_fault) *first_timing_fault = false;
    std::lock_guard<std::mutex> lock(mutex_);
    output = last_output_;
    reason.clear();
    if (request_identity != request_identity_ || execution_id != execution_id_) {
        reason = "object tracking callback does not own this request/execution";
        return false;
    }
    if (failed_) {
        reason = failure_reason_;
        output = sampleFailureStopLocked(emission_stamp);
        return false;
    }
    if (transition_stop_) {
        output = sampleFailureStopLocked(emission_stamp);
        if (unrecoverable_) {
            reason = "object transition stop cannot be sampled";
            return false;
        }
        return true;
    }
    if (!finiteCommand(nominal_target) ||
        !measured.state.position().allFinite() || !measured.state.velocity().allFinite() ||
        measured.source_sample_timestamp_us == 0 ||
        !std::isfinite(minimum_altitude_m) ||
        !std::isfinite(maximum_offset_m) || maximum_offset_m < 0.0 ||
        maximum_offset_m > limits_.max_offset_m) {
        return failLocked("object tracking input or geometry authority is invalid",
            emission_stamp, output, reason);
    }
    if (nominal_target.position()(2) < minimum_altitude_m - 1.0e-5) {
        return failLocked("object nominal target is below minimum altitude",
            emission_stamp, output, reason);
    }
    if (emission_stamp.get_clock_type() != measured.receipt_stamp.get_clock_type() ||
        emission_stamp.get_clock_type() != last_command_emission_.get_clock_type()) {
        if (first_timing_fault) *first_timing_fault = true;
        return failLocked("object tracking command and odometry clocks differ",
            emission_stamp, output, reason);
    }
    const double age_s = (emission_stamp - measured.receipt_stamp).seconds();
    const double command_age_s = (emission_stamp - last_command_emission_).seconds();
    const bool odometry_stale = !std::isfinite(age_s) ||
        age_s < -limits_.maximum_future_stamp_s ||
        age_s > limits_.maximum_odometry_age_s;
    const bool command_stale = !std::isfinite(command_age_s) ||
        command_age_s < 0.0 ||
        command_age_s > limits_.maximum_command_age_s;
    if (odometry_stale || command_stale) {
        std::ostringstream diagnostic;
        diagnostic << std::fixed << std::setprecision(6)
            << "object tracking odometry or owned command is stale"
            << " (odometry_stale=" << (odometry_stale ? "true" : "false")
            << " command_stale=" << (command_stale ? "true" : "false")
            << " odometry_age_s=" << age_s
            << " command_age_s=" << command_age_s
            << " emission_ros_ns=" << emission_stamp.nanoseconds()
            << " emission_clock_type=" << static_cast<int>(emission_stamp.get_clock_type())
            << " last_command_ros_ns=" << last_command_emission_.nanoseconds()
            << " receipt_ros_ns=" << measured.receipt_stamp.nanoseconds()
            << " source_sample_us=" << measured.source_sample_timestamp_us
            << " reset_counter=" << static_cast<unsigned>(measured.reset_counter)
            << " prior_planner_rpc_ms=";
        if (last_planner_rpc_elapsed_ms_) {
            diagnostic << *last_planner_rpc_elapsed_ms_;
        } else {
            diagnostic << "none";
        }
        diagnostic << " prior_stop_certification_ms=";
        if (last_stop_certification_elapsed_ms_) {
            diagnostic << *last_stop_certification_elapsed_ms_;
        } else {
            diagnostic << "none";
        }
        diagnostic << ')';
        if (first_timing_fault) *first_timing_fault = true;
        return failLocked(diagnostic.str(), emission_stamp, output, reason);
    }
    double sample_dt_s = 0.0;
    const auto continuity = measured.position_continuity.source_qualified ||
        measured.position_continuity.source_epoch != 0 ||
        measured.position_continuity.position_epoch != 0
            ? measured.position_continuity
            : iii_drone::control::PositionContinuityIdentity{
                0, 0, measured.reset_counter, false};
    if (last_sample_timestamp_us_) {
        if (!iii_drone::control::SamePositionContinuity(
                *position_continuity_, continuity) ||
            measured.source_sample_timestamp_us < *last_sample_timestamp_us_) {
            return failLocked("object tracking odometry reset or reversed sample"
                " (raw=" + std::to_string(position_continuity_->raw_reset_counter) +
                "->" + std::to_string(continuity.raw_reset_counter) +
                ", source_epoch=" + std::to_string(position_continuity_->source_epoch) +
                "->" + std::to_string(continuity.source_epoch) +
                ", position_epoch=" + std::to_string(position_continuity_->position_epoch) +
                "->" + std::to_string(continuity.position_epoch) +
                ", sample_us=" + std::to_string(*last_sample_timestamp_us_) +
                "->" + std::to_string(measured.source_sample_timestamp_us) + ")",
                emission_stamp, output, reason);
        }
        if (measured.source_sample_timestamp_us > *last_sample_timestamp_us_) {
            const uint64_t source_interval_us =
                measured.source_sample_timestamp_us - *last_sample_timestamp_us_;
            sample_dt_s = static_cast<double>(source_interval_us) * 1.0e-6;
            // HIL: PX4 odometry occasionally arrives after a ~0.3 s gap. The
            // fresh sample is valid (its age is checked above); only the
            // correction must not integrate across the gap.
            if (std::isfinite(sample_dt_s) && sample_dt_s > limits_.maximum_sample_interval_s &&
                sample_dt_s <= limits_.maximum_sample_gap_s) {
                sample_dt_s = 0.0;
            } else if (!std::isfinite(sample_dt_s) || sample_dt_s <= 0.0 ||
                sample_dt_s > limits_.maximum_sample_interval_s) {
                const double receipt_interval_s =
                    (measured.receipt_stamp - *last_sample_receipt_).seconds();
                std::ostringstream diagnostic;
                diagnostic << std::fixed << std::setprecision(6)
                    << "object tracking odometry sample interval is discontinuous"
                    << " (source_interval_s=" << sample_dt_s
                    << " receipt_interval_s=" << receipt_interval_s
                    << " source_sample_us=" << measured.source_sample_timestamp_us
                    << " prior_source_sample_us=" << *last_sample_timestamp_us_
                    << " receipt_ros_ns=" << measured.receipt_stamp.nanoseconds()
                    << " prior_receipt_ros_ns=" << last_sample_receipt_->nanoseconds()
                    << " emission_ros_ns=" << emission_stamp.nanoseconds()
                    << " reset_counter=" << static_cast<unsigned>(measured.reset_counter)
                    << " request_identity=" << request_identity_
                    << " execution_id=" << execution_id_ << ')';
                if (first_timing_fault) *first_timing_fault = true;
                return failLocked(diagnostic.str(),
                    emission_stamp, output, reason);
            }
        }
    } else {
        position_continuity_ = continuity;
    }
    if (!last_sample_timestamp_us_ ||
        measured.source_sample_timestamp_us > *last_sample_timestamp_us_) {
        last_sample_timestamp_us_ = measured.source_sample_timestamp_us;
        last_sample_receipt_ = measured.receipt_stamp;
    }
    // Matching metadata can qualify the already accepted source sample later.
    // Remember its proof without advancing the sample clock or integrating twice.
    position_continuity_ = continuity;

    const vector_t tracking_error = last_command_.position() - measured.state.position();
    const vector_t uncompensated_error = tracking_error - correction_;
    if (sample_dt_s > 0.0) {
        if (uncompensated_error.norm() > limits_.tracking_tolerance_m) {
            if (!uncorrected_error_started_) uncorrected_error_started_ = emission_stamp;
            if ((emission_stamp - *uncorrected_error_started_).seconds() >=
                limits_.maximum_uncorrected_error_time_s) {
                return failLocked("object tracking error did not converge",
                    emission_stamp, output, reason);
            }
        } else {
            uncorrected_error_started_.reset();
        }
    }
    if (sample_dt_s > 0.0) {
        correction_ += limits_.gain_per_s * sample_dt_s * (tracking_error - correction_);
    }
    // Geometry can change even when the odometry sample has not. Project on
    // every call; duplicate samples may not advance the estimator itself.
    const vector_t unconstrained = correction_;
    const double available_radius = std::min(limits_.max_offset_m, maximum_offset_m);
    const double floor = minimum_altitude_m - nominal_target.position()(2);
    if (floor > available_radius + 1.0e-6) {
        return failLocked("object tracking has no feasible correction above minimum altitude",
            emission_stamp, output, reason);
    }
    const double norm = correction_.norm();
    if (norm > available_radius && norm > 0.0) {
        correction_ *= available_radius / norm;
    }
    if (correction_(2) < floor) {
        correction_(2) = floor;
        const double horizontal_radius = std::sqrt(std::max(
            0.0, available_radius * available_radius - correction_(2) * correction_(2)));
        const double horizontal_norm = correction_.head<2>().norm();
        if (horizontal_norm > horizontal_radius && horizontal_norm > 0.0) {
            correction_.head<2>() *= horizontal_radius / horizontal_norm;
        }
    }
    const bool geometry_limited = (correction_ - unconstrained).norm() > 1.0e-6;
    saturated_ = uncompensated_error.norm() > limits_.tracking_tolerance_m &&
        (geometry_limited || correction_.norm() >= available_radius - 1.0e-5);
    if (saturated_) {
        if (!saturation_started_) saturation_started_ = emission_stamp;
        if ((emission_stamp - *saturation_started_).seconds() >=
            limits_.authority_exhaustion_time_s) {
            return failLocked("object tracking exhausted safe correction authority",
                emission_stamp, output, reason);
        }
    } else {
        saturation_started_.reset();
    }

    const Reference corrected_target(
        nominal_target.position() + correction_, nominal_target.yaw(),
        nominal_target.velocity(), nominal_target.yaw_rate(),
        nominal_target.acceleration(), nominal_target.yaw_acceleration(),
        emission_stamp);
    try {
        const auto planner_started = std::chrono::steady_clock::now();
        const Reference command = planner_(initial_command_, corrected_target, !planner_initialized_);
        const auto planner_finished = std::chrono::steady_clock::now();
        if (!finiteCommand(command)) {
            return failLocked("object tracking planner returned a nonfinite command",
                emission_stamp, output, reason);
        }
        // A bounded polynomial can overshoot a safe endpoint when the start
        // has nonzero acceleration. Never publish its first unsafe sample.
        if (command.position()(2) < command_floor_m_ - 1.0e-5) {
            return failLocked("object tracking generated a command below minimum altitude",
                emission_stamp, output, reason);
        }
        const auto certification_started = std::chrono::steady_clock::now();
        iii_drone::control::KinematicStopTrajectory proposed_stop(
            command, limits_.cancellation_config.limits);
        if (!certifiedStopAboveFloor(proposed_stop, command)) {
            return failLocked("object tracking command lacks a safe bounded stop",
                emission_stamp, output, reason);
        }
        const auto certification_finished = std::chrono::steady_clock::now();
        planner_initialized_ = true;
        last_command_ = command.CopyWithNewStamp(emission_stamp);
        last_output_ = last_command_;
        certified_stop_ = std::move(proposed_stop);
        last_command_emission_ = emission_stamp;
        last_planner_rpc_elapsed_ms_ =
            std::chrono::duration<double, std::milli>(planner_finished - planner_started).count();
        last_stop_certification_elapsed_ms_ =
            std::chrono::duration<double, std::milli>(
                certification_finished - certification_started).count();
        output = last_command_;
        return true;
    } catch (const std::exception & error) {
        return failLocked(std::string("object tracking planner failed: ") + error.what(),
            emission_stamp, output, reason);
    }
}

bool ObjectTrackingSession::Adopt(
    const std::string & old_request_identity,
    uint64_t old_execution_id,
    std::string new_request_identity,
    uint64_t new_execution_id,
    const Reference & applied_command,
    const rclcpp::Time & now,
    std::string & reason
) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (failed_ || old_request_identity != request_identity_ ||
        old_execution_id != execution_id_ || new_request_identity.empty() ||
        new_execution_id == 0 || !finiteCommand(applied_command) ||
        now.get_clock_type() != last_command_emission_.get_clock_type() ||
        (now - last_command_emission_).seconds() < 0.0 ||
        (now - last_command_emission_).seconds() > limits_.maximum_command_age_s) {
        reason = "object tracking adoption lacks an exact fresh finite owner/command";
        return false;
    }
    std::optional<iii_drone::control::KinematicStopTrajectory> adopted_stop;
    try {
        adopted_stop.emplace(applied_command, limits_.cancellation_config.limits);
        if (!certifiedStopAboveFloor(*adopted_stop, applied_command)) {
            reason = "object tracking applied seed lacks a safe bounded stop";
            return false;
        }
    } catch (const std::exception & error) {
        reason = std::string("object tracking applied seed stop failed: ") + error.what();
        return false;
    }
    request_identity_ = std::move(new_request_identity);
    execution_id_ = new_execution_id;
    initial_command_ = applied_command.CopyWithNewStamp(now);
    last_command_ = applied_command.CopyWithNewStamp(now);
    last_output_ = last_command_;
    certified_stop_ = std::move(adopted_stop);
    last_command_emission_ = now;
    last_planner_rpc_elapsed_ms_.reset();
    last_stop_certification_elapsed_ms_.reset();
    planner_initialized_ = false;
    transition_stop_ = false;
    stop_started_at_.reset();
    stop_complete_ = false;
    cancellation_motion_proof_.reset();
    cancellation_proof_started_.reset();
    cancellation_proof_failed_ = false;
    reason.clear();
    return true;
}

void ObjectTrackingSession::Fail(const std::string & reason) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!failed_) {
        failed_ = true;
        failure_reason_ = reason;
    }
}

Reference ObjectTrackingSession::FailureReference(
    const rclcpp::Time & emission_stamp, const std::string & reason) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!failed_) {
        failed_ = true;
        failure_reason_ = reason;
    }
    return sampleFailureStopLocked(emission_stamp);
}

bool ObjectTrackingSession::failed() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return failed_;
}

bool ObjectTrackingSession::stopComplete() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return failed_ && stop_complete_ && !unrecoverable_;
}

bool ObjectTrackingSession::unrecoverable() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return unrecoverable_;
}

std::string ObjectTrackingSession::failureReason() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return failure_reason_;
}

Reference ObjectTrackingSession::lastCommand() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_output_;
}

vector_t ObjectTrackingSession::correction() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return correction_;
}

bool ObjectTrackingSession::saturated() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return saturated_;
}

bool ObjectTrackingSession::owns(const std::string & request_identity, uint64_t execution_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return request_identity == request_identity_ && execution_id == execution_id_;
}

iii_drone::control::ControlledCancellationConfig
ObjectTrackingSession::cancellationConfig() const {
    return limits_.cancellation_config;
}

bool ObjectTrackingSession::CertifiesCancellationStop(
    const Reference & initial,
    const iii_drone::control::KinematicStopTrajectory & candidate,
    std::string & reason) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!finiteCommand(initial) ||
        (initial.position() - last_output_.position()).norm() > 1.0e-4 ||
        (initial.velocity() - last_output_.velocity()).norm() > 1.0e-4 ||
        (initial.acceleration() - last_output_.acceleration()).norm() > 1.0e-4 ||
        std::abs(initial.yaw() - last_output_.yaw()) > 1.0e-4 ||
        std::abs(initial.yaw_rate() - last_output_.yaw_rate()) > 1.0e-4 ||
        std::abs(initial.yaw_acceleration() - last_output_.yaw_acceleration()) > 1.0e-4) {
        reason = "cancellation stop is not anchored to the current owned command";
        return false;
    }
    if (!certifiedStopAboveFloor(candidate, initial)) {
        reason = "cancellation stop would cross object session command floor";
        return false;
    }
    reason.clear();
    return true;
}

void ObjectTrackingSession::RejectUnsafeCancellation(const std::string & reason) {
    std::lock_guard<std::mutex> lock(mutex_);
    failed_ = true;
    unrecoverable_ = true;
    cancellation_proof_failed_ = true;
    failure_reason_ = reason;
}

bool ObjectTrackingSession::ObserveCancellationProof(
    bool profile_complete, bool exact_applied_rest,
    const std::optional<iii_drone::control::MeasuredOdometrySnapshot> & measured,
    const rclcpp::Time & now) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (cancellation_proof_failed_ || unrecoverable_) return false;
    if (!profile_complete) {
        cancellation_motion_proof_.reset();
        return false;
    }
    if (!cancellation_proof_started_) {
        cancellation_proof_started_ = std::chrono::steady_clock::now();
    }
    if (exact_applied_rest && measured && cancellation_motion_proof_.observe(
            true, *measured, limits_.cancellation_config, now)) return true;
    if (!exact_applied_rest || !measured) cancellation_motion_proof_.reset();
    if (std::chrono::steady_clock::now() - *cancellation_proof_started_ >
        std::chrono::seconds(10)) {
        cancellation_proof_failed_ = true;
        failure_reason_ = "object cancellation lacked an exact applied rest and estimated stop";
    }
    return false;
}

bool ObjectTrackingSession::cancellationProofFailed() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return cancellation_proof_failed_;
}

bool ObjectTrackingSession::RequestTransitionStop(
    const std::string & request_identity, uint64_t execution_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (failed_ || unrecoverable_ || !certified_stop_ ||
        request_identity != request_identity_ || execution_id != execution_id_) return false;
    transition_stop_ = true;
    return true;
}

Reference ObjectTrackingSession::TransitionReference(const rclcpp::Time & emission_stamp) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!transition_stop_) return last_output_;
    return sampleFailureStopLocked(emission_stamp);
}

bool ObjectTrackingSession::transitionStopping() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return transition_stop_;
}

bool ObjectTrackingSession::transitionRest() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return transition_stop_ && stop_complete_ && !unrecoverable_;
}
