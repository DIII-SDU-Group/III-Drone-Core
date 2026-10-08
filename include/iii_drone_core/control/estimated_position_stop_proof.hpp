#pragma once

#include <cmath>
#include <cstdint>
#include <deque>
#include <optional>

#include <iii_drone_core/control/combined_drone_awareness_handler.hpp>
#include <iii_drone_core/control/kinematic_stop_trajectory.hpp>

namespace iii_drone::control {

/**
 * Terminal estimated-motion consistency check for an acknowledged rest command.
 * A biased EKF velocity can coexist with stationary estimated positions. Use
 * the length of the measured position history, not its net displacement, over
 * a full second and retain the caller's existing speed/yaw/dwell limits.
 * This is not independent evidence of physical rest or absolute GNSS accuracy.
 */
class EstimatedPositionStopProof {
public:
    void reset() {
        samples_.clear();
        settled_since_us_.reset();
        previous_receipt_.reset();
        previous_now_.reset();
        path_speed_m_s_.reset();
        settled_ = false;
    }

    bool observe(
        bool nominal_pose_reached_and_rest_command_acknowledged,
        const MeasuredOdometrySnapshot & measured,
        const ControlledCancellationConfig & config,
        const rclcpp::Time & now
    ) {
        const auto reject = [this]() { reset(); return false; };
        if (!nominal_pose_reached_and_rest_command_acknowledged ||
            !measured.state.position().allFinite() ||
            !measured.state.velocity().allFinite() ||
            !measured.state.angular_velocity().allFinite() ||
            !std::isfinite(measured.state.yaw()) ||
            measured.source_sample_timestamp_us == 0 ||
            !std::isfinite(config.velocity_threshold_m_s) ||
            config.velocity_threshold_m_s <= 0.0 ||
            !std::isfinite(config.yaw_rate_threshold_rad_s) ||
            config.yaw_rate_threshold_rad_s <= 0.0 ||
            !std::isfinite(config.settle_time_s) || config.settle_time_s < 0.0 ||
            std::abs(measured.state.angular_velocity().z()) > config.yaw_rate_threshold_rad_s ||
            now.get_clock_type() != measured.receipt_stamp.get_clock_type()) {
            return reject();
        }
        const double age_s = (now - measured.receipt_stamp).seconds();
        if (age_s < -0.02) return reject();
        if (previous_now_ &&
            (previous_now_->get_clock_type() != now.get_clock_type() || now < *previous_now_)) {
            return reject();
        }
        previous_now_ = now;
        const uint64_t stamp = measured.source_sample_timestamp_us;
        if (!samples_.empty()) {
            if (measured.reset_counter != reset_counter_ || stamp < samples_.back().stamp_us ||
                measured.receipt_stamp.get_clock_type() != previous_receipt_->get_clock_type() ||
                measured.receipt_stamp < *previous_receipt_) return reject();
            // Repeated polling/publication cannot advance the history or
            // dwell, nor withdraw the verdict on this sample while it is
            // fresh: the action loop rechecks success right after certifying
            // it, and a false here held FollowWaypointPath completion until
            // a new PX4 sample arrived between the two checks (HIL
            // 2026-10-05: 79 s, then more than 560 s).
            if (stamp == samples_.back().stamp_us) return settled_ && age_s <= 0.25;
            const auto max_gap_us = static_cast<uint64_t>(kMaximumOdometrySampleGapS * 1.0e6);
            if (stamp - samples_.back().stamp_us > max_gap_us ||
                (measured.receipt_stamp - *previous_receipt_).seconds() > kMaximumOdometrySampleGapS) return reject();
        }
        reset_counter_ = measured.reset_counter;
        previous_receipt_ = measured.receipt_stamp;
        samples_.push_back({stamp, measured.state.position()});
        // Preserve the segment crossing the exact one-second window boundary.
        while (samples_.size() > 2 && stamp - samples_[1].stamp_us >= window_us_) {
            samples_.pop_front();
        }
        if (samples_.size() > 512) return reject();
        // A stale sample cannot certify rest now, but a pause shorter than the
        // odometry gap bound does not restart the dwell (HIL: ~0.3 s gaps).
        settled_ = false;
        if (age_s > 0.25) return false;
        if (stamp - samples_.front().stamp_us < window_us_) return false;

        double length_m = 0.0;
        const uint64_t window_begin = stamp - window_us_;
        for (std::size_t i = 1; i < samples_.size(); ++i) {
            const auto & before = samples_[i - 1];
            const auto & after = samples_[i];
            double fraction = 1.0;
            if (before.stamp_us < window_begin) {
                fraction = static_cast<double>(after.stamp_us - window_begin) /
                    static_cast<double>(after.stamp_us - before.stamp_us);
            }
            length_m += fraction * (after.position - before.position).norm();
        }
        path_speed_m_s_ = length_m;  // Exactly one second of source-sample time.
        if (!std::isfinite(length_m) || length_m > config.velocity_threshold_m_s + 1.0e-12) {
            settled_since_us_.reset();
            return false;
        }
        if (!settled_since_us_) settled_since_us_ = stamp;
        settled_ = static_cast<double>(stamp - *settled_since_us_) * 1.0e-6 + 1.0e-12 >=
            config.settle_time_s;
        return settled_;
    }

    std::optional<double> pathSpeedMS() const { return path_speed_m_s_; }

private:
    struct Sample {
        uint64_t stamp_us;
        iii_drone::types::point_t position;
    };
    static constexpr uint64_t window_us_ = 1000000;
    std::deque<Sample> samples_;
    std::optional<uint64_t> settled_since_us_;
    std::optional<rclcpp::Time> previous_receipt_;
    std::optional<rclcpp::Time> previous_now_;
    std::optional<double> path_speed_m_s_;
    // Verdict on the newest sample.
    bool settled_ = false;
    uint8_t reset_counter_ = 0;
};

}  // namespace iii_drone::control
