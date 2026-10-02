#pragma once

#include <iii_drone_core/control/position_continuity_identity.hpp>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>

#include <iii_drone_core/control/combined_drone_awareness_handler.hpp>
#include <iii_drone_core/control/kinematic_stop_trajectory.hpp>
#include <iii_drone_core/control/estimated_position_stop_proof.hpp>
#include <iii_drone_core/control/reference.hpp>

namespace iii_drone::control::maneuver {

/** One measured-position correction and bounded planner continuation for an object approach. */
class ObjectTrackingSession {
public:
    struct Limits {
        double gain_per_s = 0.15;
        double max_offset_m = 0.4;
        double maximum_sample_interval_s = 0.25;
        /** An ended source gap up to this (the odometry age limit plus emission
         * jitter) is ridden through without integrating across it; longer gaps
         * end tracking. Pauses are bounded by maximum_odometry_age_s. */
        double maximum_sample_gap_s = iii_drone::control::kMaximumOdometrySampleGapS;
        double maximum_odometry_age_s = 0.25;
        double maximum_command_age_s = 0.25;
        double maximum_future_stamp_s = 0.02;
        double authority_exhaustion_time_s = 3.0;
        double tracking_tolerance_m = 0.1;
        double maximum_uncorrected_error_time_s = 90.0;
        iii_drone::control::ControlledCancellationConfig cancellation_config;
    };

    using Planner = std::function<Reference(
        const Reference & start, const Reference & corrected_target, bool reset)>;

    ObjectTrackingSession(
        Planner planner,
        Reference initial_command,
        std::string request_identity,
        uint64_t execution_id,
        const rclcpp::Time & started_at,
        double command_floor_m,
        Limits limits
    );
    static bool CanCertifyInitialSeed(const Reference & command,
        double command_floor_m, const ControlledCancellationConfig & config,
        std::string & reason);

    /** Rebind only after the new generation has an exact applied finite seed. */
    bool Adopt(
        const std::string & old_request_identity,
        uint64_t old_execution_id,
        std::string new_request_identity,
        uint64_t new_execution_id,
        const Reference & applied_command,
        const rclcpp::Time & now,
        std::string & reason
    );

    /** Sample one original live target. Output remains the last finite command on failure. */
    bool Compute(
        const Reference & nominal_target,
        const MeasuredOdometrySnapshot & measured,
        const rclcpp::Time & emission_stamp,
        const std::string & request_identity,
        uint64_t execution_id,
        double minimum_altitude_m,
        double maximum_offset_m,
        Reference & output,
        std::string & reason,
        bool * first_timing_fault = nullptr
    );

    void Fail(const std::string & reason);
    Reference FailureReference(const rclcpp::Time & emission_stamp, const std::string & reason);
    bool failed() const;
    bool stopComplete() const;
    bool unrecoverable() const;
    std::string failureReason() const;
    Reference lastCommand() const;
    iii_drone::types::vector_t correction() const;
    bool saturated() const;
    bool owns(const std::string & request_identity, uint64_t execution_id) const;
    iii_drone::control::ControlledCancellationConfig cancellationConfig() const;
    bool CertifiesCancellationStop(const Reference & initial,
        const iii_drone::control::KinematicStopTrajectory & candidate,
        std::string & reason) const;
    void RejectUnsafeCancellation(const std::string & reason);
    bool ObserveCancellationProof(bool profile_complete, bool exact_applied_rest,
        const std::optional<MeasuredOdometrySnapshot> & measured,
        const rclcpp::Time & now);
    bool cancellationProofFailed() const;
    bool RequestTransitionStop(const std::string & request_identity, uint64_t execution_id);
    Reference TransitionReference(const rclcpp::Time & emission_stamp);
    bool transitionStopping() const;
    bool transitionRest() const;

private:
    bool failLocked(const std::string & reason, const rclcpp::Time & stamp,
        Reference & output, std::string & failure_reason);
    Reference sampleFailureStopLocked(const rclcpp::Time & stamp);
    bool certifiedStopAboveFloor(const iii_drone::control::KinematicStopTrajectory & stop,
        const Reference & initial) const;
    static bool finiteCommand(const Reference & command);

    mutable std::mutex mutex_;
    Planner planner_;
    Limits limits_;
    Reference initial_command_;
    Reference last_command_;
    Reference last_output_;
    rclcpp::Time last_command_emission_;
    std::optional<double> last_planner_rpc_elapsed_ms_;
    std::optional<double> last_stop_certification_elapsed_ms_;
    double command_floor_m_;
    std::optional<iii_drone::control::KinematicStopTrajectory> certified_stop_;
    std::optional<rclcpp::Time> stop_started_at_;
    bool stop_complete_ = false;
    bool transition_stop_ = false;
    bool unrecoverable_ = false;
    std::string request_identity_;
    uint64_t execution_id_;
    bool planner_initialized_ = false;
    bool failed_ = false;
    std::string failure_reason_;
    iii_drone::types::vector_t correction_ = iii_drone::types::vector_t::Zero();
    std::optional<uint64_t> last_sample_timestamp_us_;
    std::optional<rclcpp::Time> last_sample_receipt_;
    std::optional<iii_drone::control::PositionContinuityIdentity> position_continuity_;
    std::optional<rclcpp::Time> saturation_started_;
    std::optional<rclcpp::Time> uncorrected_error_started_;
    bool saturated_ = false;
    iii_drone::control::EstimatedPositionStopProof cancellation_motion_proof_;
    std::optional<std::chrono::steady_clock::time_point> cancellation_proof_started_;
    bool cancellation_proof_failed_ = false;
};

}  // namespace iii_drone::control::maneuver
