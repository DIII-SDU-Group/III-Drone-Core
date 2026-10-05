/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/utils/opti_track_pose_relay.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

using namespace iii_drone::utils::opti_track_pose_relay;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

namespace {

constexpr int64_t kMaxRosDomainId = 232;
constexpr double kMinOutputRateHz = 1.0;
constexpr double kMaxOutputRateHz = 200.0;
constexpr double kMaxStaleTimeoutS = 1.0;
constexpr double kMaxVariance = 1.0;
constexpr double kMinAltitudeM = -500.0;
constexpr double kMaxAltitudeM = 9000.0;

std::string number(double value) {
    std::ostringstream stream;
    stream << value;
    return stream.str();
}

std::string milliseconds(double value_ms) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(0) << value_ms << " ms";
    return stream.str();
}

// False for NaN.
bool within(double value, double lower, double upper) {
    return value >= lower && value <= upper;
}

void requireWithin(
    std::vector<std::string> & errors,
    const char * name,
    double value,
    double lower,
    double upper,
    bool lower_inclusive,
    const char * unit
) {
    const bool valid = within(value, lower, upper) && (lower_inclusive || value > lower);
    if (!valid) {
        errors.push_back(
            std::string(kParameterPrefix) + name + " must be in " + (lower_inclusive ? "[" : "(") +
            number(lower) + ", " + number(upper) + "]" + unit + ", got " + number(value)
        );
    }
}

int64_t nanoseconds(double seconds) {
    return static_cast<int64_t>(std::llround(seconds * 1.0e9));
}

}  // namespace

std::vector<std::string> iii_drone::utils::opti_track_pose_relay::ValidatePoseRelayParameters(
    const PoseRelayParameters & parameters
) {
    std::vector<std::string> errors;

    if (parameters.rigid_body_id == -1) {
        errors.push_back(
            std::string(kParameterPrefix) + "rigid_body_id is not configured (-1): set it to the "
            "Motive rigid-body ID, whose pose the lab gateway publishes on /body_splitter/body_<id>/pose"
        );
    } else if (parameters.rigid_body_id < 0) {
        errors.push_back(
            std::string(kParameterPrefix) + "rigid_body_id must be >= 0, got " +
            std::to_string(parameters.rigid_body_id)
        );
    }
    if (parameters.lab_ros_domain_id < 0 || parameters.lab_ros_domain_id > kMaxRosDomainId) {
        errors.push_back(
            std::string(kParameterPrefix) + "lab_ros_domain_id must be in [0, " +
            std::to_string(kMaxRosDomainId) + "], got " + std::to_string(parameters.lab_ros_domain_id)
        );
    }
    requireWithin(errors, "output_rate_hz", parameters.output_rate_hz,
        kMinOutputRateHz, kMaxOutputRateHz, true, " Hz");
    requireWithin(errors, "stale_timeout_s", parameters.stale_timeout_s,
        0.0, kMaxStaleTimeoutS, false, " s");
    requireWithin(errors, "position_variance_m2", parameters.position_variance_m2,
        0.0, kMaxVariance, false, " m^2");
    requireWithin(errors, "orientation_variance_rad2", parameters.orientation_variance_rad2,
        0.0, kMaxVariance, false, " rad^2");
    requireWithin(errors, "origin_latitude_deg", parameters.origin_latitude_deg,
        -90.0, 90.0, true, " deg");
    requireWithin(errors, "origin_longitude_deg", parameters.origin_longitude_deg,
        -180.0, 180.0, true, " deg");
    requireWithin(errors, "origin_altitude_m", parameters.origin_altitude_m,
        kMinAltitudeM, kMaxAltitudeM, true, " m");

    return errors;
}

std::string iii_drone::utils::opti_track_pose_relay::LabPoseTopic(int64_t rigid_body_id) {
    return "/body_splitter/body_" + std::to_string(rigid_body_id) + "/pose";
}

std::optional<NedPose> iii_drone::utils::opti_track_pose_relay::LabPoseToNed(
    const LabPose & pose,
    std::string * rejection_reason
) {
    const auto reject = [rejection_reason](const std::string & reason) -> std::optional<NedPose> {
        if (rejection_reason != nullptr) {
            *rejection_reason = reason;
        }
        return std::nullopt;
    };

    const auto & p = pose.position;
    const auto & q = pose.orientation;
    if (!std::all_of(p.begin(), p.end(), [](double value) { return std::isfinite(value); })) {
        return reject("non-finite position");
    }
    if (!std::all_of(q.begin(), q.end(), [](double value) { return std::isfinite(value); })) {
        return reject("non-finite orientation");
    }
    const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (!(std::abs(norm - 1.0) <= kQuaternionNormTolerance)) {
        return reject("degenerate orientation: quaternion norm " + number(norm));
    }

    // Lab world (Z up) and body (FLU) to NED and FRD: rotate both by 180
    // degrees about x. Conjugating the rotation by it negates y and z.
    NedPose ned;
    ned.position = {p[0], -p[1], -p[2]};
    ned.orientation = {q[0] / norm, q[1] / norm, -q[2] / norm, -q[3] / norm};
    return ned;
}

OutputGate::OutputGate(double output_rate_hz, double stale_timeout_s)
: period_ns_(nanoseconds(1.0 / output_rate_hz)),
  min_spacing_ns_(period_ns_ / 2),
  stale_timeout_ns_(nanoseconds(stale_timeout_s)) { }

bool OutputGate::Offer(int64_t arrival_ns, int64_t now_ns) {
    if (now_ns - arrival_ns > stale_timeout_ns_) {
        return false;
    }
    if (forwarded_any_) {
        if (arrival_ns <= last_forwarded_arrival_ns_) {
            return false;
        }
        if (now_ns < next_due_ns_ || now_ns - last_forward_ns_ < min_spacing_ns_) {
            return false;
        }
    }

    // Keep the schedule while on time; restart it after a gap instead of
    // catching up.
    if (!forwarded_any_ || now_ns - next_due_ns_ >= period_ns_) {
        next_due_ns_ = now_ns + period_ns_;
    } else {
        next_due_ns_ += period_ns_;
    }
    forwarded_any_ = true;
    last_forward_ns_ = now_ns;
    last_forwarded_arrival_ns_ = arrival_ns;
    return true;
}

RelayHealthMonitor::RelayHealthMonitor(double output_rate_hz, double stale_timeout_s, int64_t start_ns)
: output_rate_hz_(output_rate_hz),
  stale_timeout_ns_(nanoseconds(stale_timeout_s)),
  period_start_ns_(start_ns) { }

void RelayHealthMonitor::RecordInput(int64_t arrival_ns, double lab_stamp_age_ms) {
    if (has_input_) {
        period_max_gap_ns_ = std::max(period_max_gap_ns_, arrival_ns - last_input_ns_);
    }
    has_input_ = true;
    last_input_ns_ = arrival_ns;
    lab_stamp_age_ms_ = lab_stamp_age_ms;
    ++period_inputs_;
}

void RelayHealthMonitor::RecordRejected() {
    ++rejected_total_;
    ++period_rejected_;
}

void RelayHealthMonitor::RecordOutput(int64_t forwarded_ns) {
    has_output_ = true;
    last_output_ns_ = forwarded_ns;
    ++period_outputs_;
}

HealthReport RelayHealthMonitor::Report(int64_t now_ns) {
    HealthReport report;
    const double period_s = static_cast<double>(now_ns - period_start_ns_) * 1.0e-9;
    if (period_s > 0.0) {
        report.input_rate_hz = static_cast<double>(period_inputs_) / period_s;
        report.output_rate_hz = static_cast<double>(period_outputs_) / period_s;
    }
    report.lab_stamp_age_ms = lab_stamp_age_ms_;
    report.rejected_samples = rejected_total_;
    report.forwarding = has_output_ && now_ns - last_output_ns_ <= stale_timeout_ns_;

    int64_t max_gap_ns = 0;
    if (has_input_) {
        const int64_t age_ns = std::max<int64_t>(now_ns - last_input_ns_, 0);
        max_gap_ns = std::max(period_max_gap_ns_, age_ns);
        report.last_input_age_ms = static_cast<double>(age_ns) * 1.0e-6;
        report.max_input_gap_ms = static_cast<double>(max_gap_ns) * 1.0e-6;
        report.stale = age_ns > stale_timeout_ns_;
    }

    if (!has_input_) {
        report.level = HealthLevel::ERROR;
        report.message = "no pose received yet";
    } else if (report.stale) {
        report.level = HealthLevel::ERROR;
        report.message = "stale: last pose " + milliseconds(report.last_input_age_ms) +
            " ago (timeout " + milliseconds(static_cast<double>(stale_timeout_ns_) * 1.0e-6) + ")";
    } else {
        std::vector<std::string> warnings;
        if (max_gap_ns > stale_timeout_ns_) {
            warnings.push_back("input gap " + milliseconds(report.max_input_gap_ms));
        }
        if (report.input_rate_hz < 0.5 * output_rate_hz_) {
            std::ostringstream stream;
            stream << std::fixed << std::setprecision(1) << "input rate " << report.input_rate_hz
                   << " Hz below half the output rate " << output_rate_hz_ << " Hz";
            warnings.push_back(stream.str());
        }
        if (period_rejected_ > 0) {
            warnings.push_back(std::to_string(period_rejected_) + " rejected poses");
        }
        report.level = warnings.empty() ? HealthLevel::OK : HealthLevel::WARN;
        for (std::size_t i = 0; i < warnings.size(); ++i) {
            report.message += (i == 0 ? "" : "; ") + warnings[i];
        }
        if (warnings.empty()) {
            report.message = "ok";
        }
    }

    period_start_ns_ = now_ns;
    period_inputs_ = 0;
    period_outputs_ = 0;
    period_rejected_ = 0;
    period_max_gap_ns_ = 0;
    return report;
}

OriginSender::OriginSender(bool enabled, int64_t resend_interval_ns, int64_t input_timeout_ns)
: enabled_(enabled),
  resend_interval_ns_(resend_interval_ns),
  input_timeout_ns_(input_timeout_ns) { }

void OriginSender::UpdateDisarmed(bool disarmed, int64_t now_ns) {
    disarmed_ = Sample{disarmed, now_ns};
}

void OriginSender::UpdateGlobalOrigin(bool xy_global, int64_t now_ns) {
    xy_global_ = Sample{xy_global, now_ns};
}

void OriginSender::UpdateVisionPositionFusion(bool cs_ev_pos, int64_t now_ns) {
    cs_ev_pos_ = Sample{cs_ev_pos, now_ns};
}

bool OriginSender::fresh(const std::optional<Sample> & sample, int64_t now_ns) const {
    return sample.has_value() && now_ns - sample->received_ns <= input_timeout_ns_;
}

bool OriginSender::Due(int64_t now_ns) const {
    if (!enabled_ || !fresh(disarmed_, now_ns) || !fresh(xy_global_, now_ns) ||
        !fresh(cs_ev_pos_, now_ns)) {
        return false;
    }
    if (!disarmed_->value || xy_global_->value || !cs_ev_pos_->value) {
        return false;
    }
    return !last_sent_ns_.has_value() || now_ns - *last_sent_ns_ >= resend_interval_ns_;
}

void OriginSender::MarkSent(int64_t now_ns) {
    last_sent_ns_ = now_ns;
}

bool OriginSender::sent() const {
    return last_sent_ns_.has_value();
}
