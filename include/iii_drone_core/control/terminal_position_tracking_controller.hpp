#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include <iii_drone_core/control/reference.hpp>
#include <iii_drone_core/control/state.hpp>
#include <iii_drone_core/control/position_continuity_identity.hpp>

namespace iii_drone::control {

/**
 * Bounded terminal position disturbance rejection for a fixed nominal target.
 *
 * The internal position-error integral selects a correction target. The emitted
 * correction is a non-preempted seventh-order rest-to-rest segment so position,
 * velocity, acceleration, and jerk remain mutually consistent and bounded.
 */
class TerminalPositionTrackingController {
public:
    struct Limits {
        double max_offset_m = 0.4;
        double max_offset_speed_m_s = 0.1;
        double max_offset_acceleration_m_s2 = 0.2;
        double max_offset_jerk_m_s3 = 0.5;
        double integral_gain_per_s = 0.15;
        /** Only the position error beyond this radius is integrated. The
         * vehicle's own position hold wanders a few centimetres about its
         * setpoint; chasing that wander turns a steady hover into a series of
         * correction segments. Capped at half the arrival tolerance, so an
         * error beyond the tolerance always integrates. */
        double correction_deadband_m = 0.02;
        /** A pending integral correction is applied only once it reaches this
         * size (capped like the deadband). Smaller ones keep the reference at
         * rest: otherwise position noise in a steady hover becomes a stream of
         * millimetre rest-to-rest segments whose feedforward (up to
         * 0.045 m/s^2), sampled at the control rate, shakes the vehicle. */
        double minimum_correction_step_m = 0.01;
        double arrival_tolerance_m = 0.1;
        double maximum_odometry_age_s = 0.25;
        double maximum_sample_interval_s = 0.25;
        /** An ended odometry gap up to this (the odometry age limit plus
         * emission jitter) is ridden through without integrating across it;
         * longer gaps end tracking. Pauses are bounded by maximum_odometry_age_s. */
        double maximum_sample_gap_s = iii_drone::control::kMaximumOdometrySampleGapS;
        double maximum_future_stamp_s = 0.02;
        double maximum_tracking_time_s = 90.0;
        double authority_exhaustion_time_s = 3.0;
    };

    explicit TerminalPositionTrackingController(Reference nominal_reference);
    TerminalPositionTrackingController(Reference nominal_reference, Limits limits);

    /**
     * Update using one measured sample and an independent command-emission time.
     * Returns false on invalid/stale/reset input or exhausted safe authority.
     * On failure, output remains the last accepted command for controlled-stop
     * ownership to resolve; the caller must not report success.
     */
    bool Update(
        const State & state,
        const rclcpp::Time & odometry_stamp,
        uint8_t reset_counter,
        const rclcpp::Time & emission_stamp,
        double safe_offset_radius_m,
        Reference & output,
        std::string & failure_reason
    );
    bool Update(
        const State & state,
        const rclcpp::Time & odometry_stamp,
        const PositionContinuityIdentity & position_continuity,
        const rclcpp::Time & emission_stamp,
        double safe_offset_radius_m,
        Reference & output,
        std::string & failure_reason
    );

    /** Freeze correction growth and finish any already-started segment. */
    void RequestQuiescence();
    /** True only after requested quiescence has reached a full-rest reference. */
    bool isQuiescent() const;
    /** Resume correction from the actually emitted offset without resetting P/V/A. */
    void ResumeTracking();
    // A maneuver handover deliberately pauses evaluation (token transfer,
    // execution start). The next update re-anchors its sample timing instead
    // of rejecting the pause as an odometry gap or integrating across it; the
    // continuity identity and odometry-age checks still apply.
    void ResumeAfterHandover();

    /**
     * Irreversible fault stop: finish only the already committed segment.
     * No measured feedback is consumed or refreshed. The original correction
     * ball and derivative bounds remain valid through its stationary endpoint.
     * Equal emission times are idempotent; clock reversal/mismatch returns the
     * last accepted command and false, never a fabricated stopped reference.
     */
    bool ContinueCommittedStop(
        const rclcpp::Time & emission_stamp,
        Reference & output,
        std::string & failure_reason
    );

    const Reference & lastOutput() const;
    double integralTargetOffsetNorm() const;
    double emittedOffsetNorm() const;

private:
    struct Segment {
        iii_drone::types::vector_t start = iii_drone::types::vector_t::Zero();
        iii_drone::types::vector_t target = iii_drone::types::vector_t::Zero();
        rclcpp::Time start_stamp{0, 0, RCL_ROS_TIME};
        double duration_s = 0.0;
        bool active = false;
    };

    bool fail(const std::string & reason, Reference & output, std::string & failure_reason);
    void beginSegment(const rclcpp::Time & stamp);
    void sampleSegment(const rclcpp::Time & stamp);
    void buildOutput(const rclcpp::Time & stamp);

    Reference nominal_reference_;
    Limits limits_;
    Reference last_output_;
    iii_drone::types::vector_t integral_target_offset_ = iii_drone::types::vector_t::Zero();
    iii_drone::types::vector_t emitted_offset_ = iii_drone::types::vector_t::Zero();
    iii_drone::types::vector_t emitted_velocity_ = iii_drone::types::vector_t::Zero();
    iii_drone::types::vector_t emitted_acceleration_ = iii_drone::types::vector_t::Zero();
    Segment segment_;
    bool initialized_ = false;
    bool rebase_sample_timing_ = false;
    PositionContinuityIdentity position_continuity_;
    rclcpp::Time previous_odometry_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time previous_emission_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time nonconvergence_start_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time authority_saturation_start_stamp_{0, 0, RCL_ROS_TIME};
    bool nonconvergence_active_ = false;
    bool authority_saturated_ = false;
    bool quiescence_requested_ = false;
    bool committed_stop_active_ = false;
    std::optional<rclcpp::Time> stop_previous_emission_stamp_;
};

}  // namespace iii_drone::control
