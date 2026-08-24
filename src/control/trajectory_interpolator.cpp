/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/trajectory_interpolator.hpp>

#include <algorithm>
#include <cmath>

using namespace iii_drone::control;
using namespace iii_drone::configuration;
using namespace iii_drone::adapters;
using namespace iii_drone::types;

namespace {

double shortestYawError(double current_yaw, double target_yaw) {
    return std::atan2(std::sin(target_yaw - current_yaw), std::cos(target_yaw - current_yaw));
}

Reference withYawClosestTo(const Reference & reference, double anchor_yaw) {
    return Reference(
        reference.position(),
        anchor_yaw + shortestYawError(anchor_yaw, reference.yaw()),
        reference.velocity(),
        reference.yaw_rate(),
        reference.acceleration(),
        reference.yaw_acceleration(),
        reference.stamp()
    );
}

}  // namespace

/*****************************************************************************/
// Implementation
/*****************************************************************************/

TrajectoryInterpolator::TrajectoryInterpolator(
    Configuration::SharedPtr params,
    rclcpp_lifecycle::LifecycleNode * node
) : configuration_(params), node_(node) { }

TrajectoryInterpolator::~TrajectoryInterpolator() { }

ReferenceTrajectory TrajectoryInterpolator::ComputeReferenceTrajectory(
    const State &start_state,
    const Reference &end_reference,
    bool set_reference,
    bool reset
) {

    // State feedback can retain small tracking velocities after a maneuver has
    // reached its waypoint. Treating those as multi-second endpoint
    // constraints bends an otherwise straight point-to-point trajectory.
    const Reference stationary_start(start_state.position(), start_state.yaw());
    return ComputeReferenceTrajectory(
        stationary_start,
        end_reference,
        set_reference,
        reset
    );

}

ReferenceTrajectory TrajectoryInterpolator::ComputeReferenceTrajectory(
    const Reference &start_reference,
    const Reference &end_reference,
    bool set_reference,
    bool reset
) {

    double t = (rclcpp::Clock().now() - start_time_).seconds();

    if (first_ || reset) {

        first_ = false;
        reference_trajectory_ = ReferenceTrajectory();
        reference_ = withYawClosestTo(end_reference, start_reference.yaw());

        start_time_ = rclcpp::Clock().now();

        double T = computeInterpolation(
            start_reference,
            reference_
        );

        end_time_ = start_time_ + rclcpp::Duration::from_seconds(T);

        t = 0;

    } else if (set_reference) {

        Reference new_start_reference = referenceFunction(t);
        reference_ = withYawClosestTo(end_reference, new_start_reference.yaw());

        start_time_ = rclcpp::Clock().now();

        double T = computeInterpolation(
            new_start_reference,
            reference_
        );

        end_time_ = start_time_ + rclcpp::Duration::from_seconds(T);

        t = 0;

    }

    reference_trajectory_ = referenceTrajectoryFunction(t);

    return reference_trajectory_;

}

double TrajectoryInterpolator::computeInterpolation(
    const Reference &start_reference,
    const Reference &end_reference
) {

    auto rescale_yaw = [](double yaw) {
        while (yaw < -M_PI) yaw += 2 * M_PI;
        while (yaw > M_PI) yaw -= 2 * M_PI;
        return yaw;
    };

    const point_t p0 = start_reference.position();
    vector_t v0 = start_reference.velocity();
    // A streamed acceleration is an instantaneous feed-forward value, not a
    // constraint that should shape the entire next segment. Holding it as a
    // quintic endpoint derivative makes longer durations amplify corner
    // handoffs into arbitrarily large spatial excursions.
    const vector_t a0 = vector_t::Zero();
    const double yaw0 = rescale_yaw(start_reference.yaw());
    const double yaw_rate_0 = start_reference.yaw_rate();
    const double yaw_acceleration_0 = 0.0;

    const point_t pT = end_reference.position();
    const vector_t vT = end_reference.velocity(); 
    const vector_t aT = end_reference.acceleration();
    const double yawT = yaw0 + shortestYawError(yaw0, end_reference.yaw());
    const double yaw_rate_T = end_reference.yaw_rate();
    const double yaw_acceleration_T = 0;

    const double position_duration = (pT - p0).norm()
        / configuration_->GetParameter("/control/trajectory_interpolator/interpolation_avg_velocity_m_s").as_double();
    const double yaw_error = std::abs(yawT - yaw0);
    const double yaw_duration = yaw_error
        / configuration_->GetParameter("/control/trajectory_interpolator/interpolation_avg_yaw_rate_rad_s").as_double();

    const double max_velocity = configuration_->GetParameter(
        "/control/trajectory_interpolator/interpolation_max_velocity_m_s"
    ).as_double();
    const double max_acceleration = configuration_->GetParameter(
        "/control/trajectory_interpolator/interpolation_max_acceleration_m_s2"
    ).as_double();
    const double max_yaw_rate = configuration_->GetParameter(
        "/control/trajectory_interpolator/interpolation_max_yaw_rate_rad_s"
    ).as_double();
    const double max_yaw_acceleration = configuration_->GetParameter(
        "/control/trajectory_interpolator/interpolation_max_yaw_acceleration_rad_s2"
    ).as_double();

    const double position_distance = (pT - p0).norm();
    const double t0 = 0.0;
    constexpr double rest_to_rest_peak_velocity_coeff = 1.875;
    constexpr double rest_to_rest_peak_acceleration_coeff = 5.773502691896258;

    const double T_velocity = rest_to_rest_peak_velocity_coeff * position_distance / max_velocity;
    const double T_acceleration = std::sqrt(
        rest_to_rest_peak_acceleration_coeff * position_distance / max_acceleration
    );
    const double T_yaw_rate = rest_to_rest_peak_velocity_coeff * yaw_error / max_yaw_rate;
    const double T_yaw_acceleration = std::sqrt(
        rest_to_rest_peak_acceleration_coeff * yaw_error / max_yaw_acceleration
    );

    double T = std::max({
        position_duration,
        yaw_duration,
        T_velocity,
        T_acceleration,
        T_yaw_rate,
        T_yaw_acceleration,
        1.0e-3
    });

    const auto limit_start_velocity_for_duration = [&]() {
        constexpr double monotonic_slope_factor = 3.0;
        constexpr double minimum_axis_displacement = 1.0e-4;
        const vector_t displacement = pT - p0;

        for (int axis = 0; axis < 3; ++axis) {
            const double delta = displacement(axis);
            if (
                std::abs(delta) < minimum_axis_displacement ||
                v0(axis) * delta <= 0.0
            ) {
                v0(axis) = 0.0;
                continue;
            }

            const double maximum_velocity =
                monotonic_slope_factor * std::abs(delta) / T;
            v0(axis) = std::copysign(
                std::min(std::abs(static_cast<double>(v0(axis))), maximum_velocity),
                delta
            );
        }
    };

    auto solve_quintic = [&]() {
        Eigen::Matrix<double, 6, 6> A;

        Eigen::Matrix<double, 1, 6> A_p0;
        Eigen::Matrix<double, 1, 6> A_v0;
        Eigen::Matrix<double, 1, 6> A_a0;

        A_p0 << 1, t0, t0*t0, t0*t0*t0, t0*t0*t0*t0, t0*t0*t0*t0*t0;
        A_v0 << 0, 1, 2*t0, 3*t0*t0, 4*t0*t0*t0, 5*t0*t0*t0*t0;
        A_a0 << 0, 0, 2, 6*t0, 12*t0*t0, 20*t0*t0*t0;

        Eigen::Matrix<double, 1, 6> A_vT;
        Eigen::Matrix<double, 1, 6> A_pT;
        Eigen::Matrix<double, 1, 6> A_aT;

        A_vT << 0, 1, 2*T, 3*T*T, 4*T*T*T, 5*T*T*T*T;
        A_pT << 1, T, T*T, T*T*T, T*T*T*T, T*T*T*T*T;
        A_aT << 0, 0, 2, 6*T, 12*T*T, 20*T*T*T;

        A.row(0) = A_p0;
        A.row(1) = A_v0;
        A.row(2) = A_a0;
        A.row(3) = A_pT;
        A.row(4) = A_vT;
        A.row(5) = A_aT;

        for (int i = 0; i < 3; i++) {

            Eigen::Matrix<double, 6, 1> b;

            b << p0(i), v0(i), a0(i), pT(i), vT(i), aT(i);

            q.col(i) = A.colPivHouseholderQr().solve(b);

        }

        Eigen::Matrix<double, 6, 1> b_yaw;
        b_yaw << yaw0, yaw_rate_0, yaw_acceleration_0, yawT, yaw_rate_T, yaw_acceleration_T;

        q_yaw = A.colPivHouseholderQr().solve(b_yaw);
    };

    auto yawAccelerationFunction = [this](double t) {
        Eigen::Matrix<double, 1, 6> A_t;

        A_t << 0, 0, 2, 6*t, 12*t*t, 20*t*t*t;

        return (double)(A_t * q_yaw);
    };

    constexpr int max_duration_adjustments = 4;
    constexpr int sample_count = 50;
    constexpr double duration_margin = 1.05;

    for (int adjustment = 0; adjustment < max_duration_adjustments; ++adjustment) {
        limit_start_velocity_for_duration();
        solve_quintic();

        double observed_velocity = 0.0;
        double observed_acceleration = 0.0;
        double observed_yaw_rate = 0.0;
        double observed_yaw_acceleration = 0.0;

        for (int sample = 0; sample <= sample_count; ++sample) {
            const double sample_t = T * static_cast<double>(sample) / sample_count;

            observed_velocity = std::max(observed_velocity, static_cast<double>(velocityFunction(sample_t).norm()));
            observed_acceleration = std::max(observed_acceleration, static_cast<double>(accelerationFunction(sample_t).norm()));
            observed_yaw_rate = std::max(observed_yaw_rate, std::abs(yawRateFunction(sample_t)));
            observed_yaw_acceleration = std::max(observed_yaw_acceleration, std::abs(yawAccelerationFunction(sample_t)));
        }

        const double duration_scale = std::max({
            1.0,
            observed_velocity / max_velocity,
            std::sqrt(observed_acceleration / max_acceleration),
            observed_yaw_rate / max_yaw_rate,
            std::sqrt(observed_yaw_acceleration / max_yaw_acceleration)
        });

        if (!std::isfinite(duration_scale) || duration_scale <= 1.001) {
            return T;
        }

        T *= duration_scale * duration_margin;
    }

    limit_start_velocity_for_duration();
    solve_quintic();
    return T;

}

point_t TrajectoryInterpolator::positionFunction(double t) {

    Eigen::Matrix<double, 1, 6> A_t;

    A_t << 1, t, t*t, t*t*t, t*t*t*t, t*t*t*t*t;

    Eigen::Matrix<double, 1, 3> p = A_t * q;

    return point_t(p(0), p(1), p(2));

}

vector_t TrajectoryInterpolator::velocityFunction(double t) {

    Eigen::Matrix<double, 1, 6> A_t;

    A_t << 0, 1, 2*t, 3*t*t, 4*t*t*t, 5*t*t*t*t;

    Eigen::Matrix<double, 1, 3> v = A_t * q;

    return vector_t(v(0), v(1), v(2));

}

vector_t TrajectoryInterpolator::accelerationFunction(double t) {

    Eigen::Matrix<double, 1, 6> A_t;

    A_t << 0, 0, 2, 6*t, 12*t*t, 20*t*t*t;

    Eigen::Matrix<double, 1, 3> a = A_t * q;

    return vector_t(a(0), a(1), a(2));

}

double TrajectoryInterpolator::yawFunction(double t) {

    Eigen::Matrix<double, 1, 6> A_t;

    A_t << 1, t, t*t, t*t*t, t*t*t*t, t*t*t*t*t;

    return (double)(A_t * q_yaw);

}

double TrajectoryInterpolator::yawRateFunction(double t) {

    Eigen::Matrix<double, 1, 6> A_t;

    A_t << 0, 1, 2*t, 3*t*t, 4*t*t*t, 5*t*t*t*t;

    return (double)(A_t * q_yaw);

}

Reference TrajectoryInterpolator::referenceFunction(double t) {

    if (start_time_ + rclcpp::Duration::from_seconds(t) > end_time_) {

        return reference_;

    }

    point_t p = positionFunction(t);
    vector_t v = velocityFunction(t);
    vector_t a = accelerationFunction(t);
    double yaw = yawFunction(t);
    double yaw_rate = yawRateFunction(t);

    return Reference(
        p,
        yaw,
        v,
        yaw_rate,
        a,
        0,
        start_time_ + rclcpp::Duration::from_seconds(t)
    );

}

ReferenceTrajectory TrajectoryInterpolator::referenceTrajectoryFunction(double t) {

    int N = configuration_->GetParameter("/control/trajectory_interpolator/reference_trajectory_length_N").as_int();
    double dt = configuration_->GetParameter("/control/dt").as_double();

    std::vector<Reference> reference_trajectory;

    for (int i = 0; i < N; i++) {

        reference_trajectory.push_back(referenceFunction(t + i * dt));

    }

    return ReferenceTrajectory(reference_trajectory);

}
