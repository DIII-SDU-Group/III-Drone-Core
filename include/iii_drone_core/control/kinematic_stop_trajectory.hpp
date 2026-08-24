#pragma once

#include <iii_drone_core/control/reference.hpp>

namespace iii_drone::control {

struct KinematicStopLimits {
    double max_acceleration_m_s2 = 0.5;
    double max_jerk_m_s3 = 1.0;
    double max_yaw_acceleration_rad_s2 = 0.75;
    double max_yaw_jerk_rad_s3 = 1.5;
};

struct ControlledCancellationConfig {
    KinematicStopLimits limits;
    double velocity_threshold_m_s = 0.08;
    double yaw_rate_threshold_rad_s = 0.08;
    double settle_time_s = 0.2;
};

/**
 * A C2 stop trajectory. Velocity is a cubic Hermite curve from the supplied
 * velocity/acceleration to zero velocity/acceleration. Duration is increased
 * until the configured acceleration and jerk limits are satisfied.
 */
class KinematicStopTrajectory {
public:
    KinematicStopTrajectory(const Reference & initial, const KinematicStopLimits & limits);

    Reference sample(double elapsed_s, const rclcpp::Time & stamp = rclcpp::Clock().now()) const;
    Reference terminalReference(const rclcpp::Time & stamp = rclcpp::Clock().now()) const;
    double durationS() const;

private:
    bool satisfiesLimits(double duration_s) const;

    Reference initial_;
    KinematicStopLimits limits_;
    double duration_s_ = 0.0;
};

}  // namespace iii_drone::control
