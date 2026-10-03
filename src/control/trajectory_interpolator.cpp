/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/trajectory_interpolator.hpp>

#include <sstream>

#include <algorithm>
#include <cmath>
#include <limits>

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
    rclcpp_lifecycle::LifecycleNode * node,
    std::function<rclcpp::Time()> clock_now
) : configuration_(params), node_(node),
    clock_now_(clock_now ? std::move(clock_now) : [] { return rclcpp::Clock().now(); }) { }

TrajectoryInterpolator::~TrajectoryInterpolator() { }

TrajectoryInterpolator::BoundedDerivativeSample
TrajectoryInterpolator::boundedDerivativeSample(double t) const {
    BoundedDerivativeSample sample;
    // The bounded certificate evaluates these fixed quintic derivatives at
    // 513 points. Horner form avoids constructing six tiny Eigen matrices at
    // every point; keep double arithmetic and the existing vector_t cast.
    for (int axis = 0; axis < 3; ++axis) {
        const double c1 = q(1, axis);
        const double c2 = q(2, axis);
        const double c3 = q(3, axis);
        const double c4 = q(4, axis);
        const double c5 = q(5, axis);
        sample.velocity(axis) = static_cast<float>(
            ((((5.0 * c5 * t + 4.0 * c4) * t + 3.0 * c3) * t +
                2.0 * c2) * t + c1));
        sample.acceleration(axis) = static_cast<float>(
            (((20.0 * c5 * t + 12.0 * c4) * t + 6.0 * c3) * t +
                2.0 * c2));
        sample.jerk(axis) = static_cast<float>(
            ((60.0 * c5 * t + 24.0 * c4) * t + 6.0 * c3));
    }
    const double y1 = q_yaw(1);
    const double y2 = q_yaw(2);
    const double y3 = q_yaw(3);
    const double y4 = q_yaw(4);
    const double y5 = q_yaw(5);
    sample.yaw_rate = ((((5.0 * y5 * t + 4.0 * y4) * t + 3.0 * y3) * t +
        2.0 * y2) * t + y1);
    sample.yaw_acceleration = (((20.0 * y5 * t + 12.0 * y4) * t + 6.0 * y3) * t +
        2.0 * y2);
    sample.yaw_jerk = ((60.0 * y5 * t + 24.0 * y4) * t + 6.0 * y3);
    return sample;
}

ReferenceTrajectory TrajectoryInterpolator::ComputeBoundedPositionalTrajectory(
    const Reference &start_reference,
    const Reference &end_reference,
    bool set_reference,
    bool reset
) {
    return ComputeReferenceTrajectory(
        start_reference, end_reference, set_reference, reset, true);
}

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
    bool reset,
    bool bounded_interpolation
) {

    const rclcpp::Time now = clock_now_();
    double t = (now - start_time_).seconds();

    if (first_ || reset) {

        first_ = false;
        reference_trajectory_ = ReferenceTrajectory();
        reference_ = withYawClosestTo(end_reference, start_reference.yaw());
        bounded_interpolation_ = bounded_interpolation;
        if (bounded_interpolation_) {
            reference_ = Reference(
                reference_.position(), reference_.yaw(), vector_t::Zero(), 0.0,
                vector_t::Zero(), 0.0, reference_.stamp()
            );
        }

        start_time_ = now;

        double T = computeInterpolation(
            start_reference,
            reference_,
            bounded_interpolation
        );

        end_time_ = start_time_ + rclcpp::Duration::from_seconds(T);

        t = 0;

    } else if (set_reference) {

        Reference new_start_reference = referenceFunction(t);
        reference_ = withYawClosestTo(end_reference, new_start_reference.yaw());
        bounded_interpolation_ = bounded_interpolation;
        if (bounded_interpolation_) {
            reference_ = Reference(
                reference_.position(), reference_.yaw(), vector_t::Zero(), 0.0,
                vector_t::Zero(), 0.0, reference_.stamp()
            );
        }

        start_time_ = now;

        double T = computeInterpolation(
            new_start_reference,
            reference_,
            bounded_interpolation
        );

        end_time_ = start_time_ + rclcpp::Duration::from_seconds(T);

        t = 0;

    }

    reference_trajectory_ = referenceTrajectoryFunction(t);

    return reference_trajectory_;

}

double TrajectoryInterpolator::computeInterpolation(
    const Reference &start_reference,
    const Reference &end_reference,
    bool bounded_interpolation
) {

    auto rescale_yaw = [](double yaw) {
        while (yaw < -M_PI) yaw += 2 * M_PI;
        while (yaw > M_PI) yaw -= 2 * M_PI;
        return yaw;
    };

    if (bounded_interpolation && (
        !start_reference.position().allFinite() ||
        !start_reference.velocity().allFinite() ||
        !start_reference.acceleration().allFinite() ||
        !std::isfinite(start_reference.yaw()) ||
        !std::isfinite(start_reference.yaw_rate()) ||
        !std::isfinite(start_reference.yaw_acceleration()) ||
        !end_reference.position().allFinite() ||
        !std::isfinite(end_reference.yaw())
    )) {
        throw std::runtime_error(
            "Bounded interpolation requires finite start and target values"
        );
    }

    const point_t p0 = start_reference.position();
    vector_t v0 = start_reference.velocity();
    // A streamed acceleration is an instantaneous feed-forward value, not a
    // constraint that should shape the entire next segment. Holding it as a
    // quintic endpoint derivative makes longer durations amplify corner
    // handoffs into arbitrarily large spatial excursions.
    const vector_t a0 = bounded_interpolation ?
        start_reference.acceleration() : vector_t::Zero();
    const double yaw0 = bounded_interpolation ?
        std::remainder(start_reference.yaw(), 2.0 * M_PI) :
        rescale_yaw(start_reference.yaw());
    const double yaw_rate_0 = start_reference.yaw_rate();
    const double yaw_acceleration_0 = bounded_interpolation ?
        start_reference.yaw_acceleration() : 0.0;

    const point_t pT = end_reference.position();
    const vector_t vT = bounded_interpolation ? vector_t::Zero() : end_reference.velocity();
    const vector_t aT = bounded_interpolation ? vector_t::Zero() : end_reference.acceleration();
    const double end_yaw = bounded_interpolation ?
        std::remainder(end_reference.yaw(), 2.0 * M_PI) : end_reference.yaw();
    const double yawT = yaw0 + shortestYawError(yaw0, end_yaw);
    const double yaw_rate_T = bounded_interpolation ? 0.0 : end_reference.yaw_rate();
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
    // Every segment honours the configured jerk limits. Without them a short
    // segment is timed only by its acceleration limit, and its quintic swings
    // from +a_max to -a_max within a fraction of a second: a 13 mm FlyToPosition
    // produced ~13 m/s^3 against the 1 m/s^3 limit the consumer's continuity
    // envelope assumes.
    const double max_jerk = configuration_->GetParameter(
        "/control/trajectory_interpolator/interpolation_max_jerk_m_s3"
    ).as_double();
    const double max_yaw_jerk = configuration_->GetParameter(
        "/control/maneuver_controller/controlled_cancel_max_yaw_jerk_rad_s3"
    ).as_double();

    if (bounded_interpolation) {
        const double avg_velocity = configuration_->GetParameter(
            "/control/trajectory_interpolator/interpolation_avg_velocity_m_s"
        ).as_double();
        const double avg_yaw_rate = configuration_->GetParameter(
            "/control/trajectory_interpolator/interpolation_avg_yaw_rate_rad_s"
        ).as_double();
        const bool finite_start = p0.allFinite() && v0.allFinite() && a0.allFinite() &&
            std::isfinite(yaw0) && std::isfinite(yaw_rate_0) &&
            std::isfinite(yaw_acceleration_0) && pT.allFinite() &&
            std::isfinite(yawT);
        const bool finite_positive_limits = std::isfinite(avg_velocity) && avg_velocity > 0.0 &&
            std::isfinite(avg_yaw_rate) && avg_yaw_rate > 0.0 &&
            std::isfinite(max_velocity) && max_velocity > 0.0 &&
            std::isfinite(max_acceleration) && max_acceleration > 0.0 &&
            std::isfinite(max_jerk) && max_jerk > 0.0 &&
            std::isfinite(max_yaw_jerk) && max_yaw_jerk > 0.0 &&
            std::isfinite(max_yaw_rate) && max_yaw_rate > 0.0 &&
            std::isfinite(max_yaw_acceleration) && max_yaw_acceleration > 0.0;
        if (!finite_start || !finite_positive_limits) {
            throw std::runtime_error(
                "Bounded interpolation requires finite start/target data and positive finite limits"
            );
        }
        if (v0.norm() > max_velocity || a0.norm() > max_acceleration ||
            std::abs(yaw_rate_0) > max_yaw_rate ||
            std::abs(yaw_acceleration_0) > max_yaw_acceleration) {
            throw std::runtime_error(
                "Bounded interpolation start derivatives exceed configured interpolation limits"
            );
        }
    }

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
    const double T_jerk = std::cbrt(60.0 * position_distance / max_jerk);
    const double T_yaw_jerk = std::cbrt(60.0 * yaw_error / max_yaw_jerk);

    double T = std::max({
        position_duration,
        yaw_duration,
        T_velocity,
        T_acceleration,
        T_yaw_rate,
        T_yaw_acceleration,
        T_jerk,
        T_yaw_jerk,
        1.0e-3
    });

    constexpr double bounded_max_duration_s = 600.0;
    if (bounded_interpolation &&
        (!std::isfinite(T) || T <= 0.0 || T > bounded_max_duration_s)) {
        throw std::runtime_error(
            "Bounded interpolation has an invalid initial duration"
        );
    }

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

    if (bounded_interpolation) {
        constexpr int sample_count = 512;
        constexpr int max_duration_adjustments = 64;
        constexpr double duration_margin = 1.02;
        constexpr double max_duration_s = bounded_max_duration_s;

        auto polynomialDerivativeBound = [](const auto & coefficients, int order, double duration) {
            double bound = 0.0;
            for (int power = order; power < coefficients.rows(); ++power) {
                double factor = 1.0;
                for (int derivative = 0; derivative < order; ++derivative) {
                    factor *= static_cast<double>(power - derivative);
                }
                bound += coefficients.row(power).norm() * factor *
                    std::pow(duration, power - order);
            }
            return bound;
        };

        auto yawDerivativeBound = [](const auto & coefficients, int order, double duration) {
            double bound = 0.0;
            for (int power = order; power < coefficients.rows(); ++power) {
                double factor = 1.0;
                for (int derivative = 0; derivative < order; ++derivative) {
                    factor *= static_cast<double>(power - derivative);
                }
                bound += std::abs(coefficients(power)) * factor *
                    std::pow(duration, power - order);
            }
            return bound;
        };

        for (int adjustment = 0; adjustment < max_duration_adjustments; ++adjustment) {
            if (!std::isfinite(T) || T <= 0.0 || T > max_duration_s) {
                throw std::runtime_error(
                    "Bounded interpolation exceeded the duration search bound"
                );
            }
            solve_quintic();
            if (!q.allFinite() || !q_yaw.allFinite()) {
                throw std::runtime_error(
                    "Bounded interpolation produced non-finite coefficients"
                );
            }

            double sampled_velocity = 0.0;
            double sampled_acceleration = 0.0;
            double sampled_jerk = 0.0;
            double sampled_yaw_rate = 0.0;
            double sampled_yaw_acceleration = 0.0;
            double sampled_yaw_jerk = 0.0;
            for (int sample = 0; sample <= sample_count; ++sample) {
                const double time = T * static_cast<double>(sample) / sample_count;
                const BoundedDerivativeSample derivatives = boundedDerivativeSample(time);
                if (!derivatives.velocity.allFinite() ||
                    !derivatives.acceleration.allFinite() ||
                    !derivatives.jerk.allFinite() ||
                    !std::isfinite(derivatives.yaw_rate) ||
                    !std::isfinite(derivatives.yaw_acceleration) ||
                    !std::isfinite(derivatives.yaw_jerk)) {
                    throw std::runtime_error(
                        "Bounded interpolation produced non-finite derivatives"
                    );
                }
                sampled_velocity = std::max(sampled_velocity,
                    static_cast<double>(derivatives.velocity.norm()));
                sampled_acceleration = std::max(sampled_acceleration,
                    static_cast<double>(derivatives.acceleration.norm()));
                sampled_jerk = std::max(sampled_jerk,
                    static_cast<double>(derivatives.jerk.norm()));
                sampled_yaw_rate = std::max(sampled_yaw_rate,
                    std::abs(derivatives.yaw_rate));
                sampled_yaw_acceleration = std::max(
                    sampled_yaw_acceleration, std::abs(derivatives.yaw_acceleration)
                );
                sampled_yaw_jerk = std::max(
                    sampled_yaw_jerk, std::abs(derivatives.yaw_jerk)
                );
            }

            const double half_sample_step = T / (2.0 * sample_count);
            const double certified_velocity = sampled_velocity +
                polynomialDerivativeBound(q, 2, T) * half_sample_step;
            const double certified_acceleration = sampled_acceleration +
                polynomialDerivativeBound(q, 3, T) * half_sample_step;
            const double certified_jerk = sampled_jerk +
                polynomialDerivativeBound(q, 4, T) * half_sample_step;
            const double certified_yaw_rate = sampled_yaw_rate +
                yawDerivativeBound(q_yaw, 2, T) * half_sample_step;
            const double certified_yaw_acceleration = sampled_yaw_acceleration +
                yawDerivativeBound(q_yaw, 3, T) * half_sample_step;
            const double certified_yaw_jerk = sampled_yaw_jerk +
                yawDerivativeBound(q_yaw, 4, T) * half_sample_step;
            if (!std::isfinite(certified_velocity) || !std::isfinite(certified_acceleration) ||
                !std::isfinite(certified_jerk) || !std::isfinite(certified_yaw_rate) ||
                !std::isfinite(certified_yaw_acceleration) ||
                !std::isfinite(certified_yaw_jerk)) {
                throw std::runtime_error(
                    "Bounded interpolation could not certify finite derivatives"
                );
            }

            const double duration_scale = std::max({
                certified_velocity / max_velocity,
                std::sqrt(certified_acceleration / max_acceleration),
                std::cbrt(certified_jerk / max_jerk),
                certified_yaw_rate / max_yaw_rate,
                std::sqrt(certified_yaw_acceleration / max_yaw_acceleration),
                std::cbrt(certified_yaw_jerk / max_yaw_jerk)
            });

            if (!std::isfinite(duration_scale)) {
                throw std::runtime_error(
                    "Bounded interpolation could not certify finite derivatives"
                );
            }
            if (duration_scale <= 1.0) {
                return T;
            }

            const double certified_duration = T;
            T *= std::max(duration_margin, duration_scale * duration_margin);
            if (!std::isfinite(T) || T > max_duration_s) {
                // Name the inputs and the limiting derivative so a rare
                // divergence is diagnosable from the log alone.
                std::ostringstream reason;
                reason << "Bounded interpolation exceeded the duration search bound"
                    << " (adjustment " << adjustment << ", T " << certified_duration << " s"
                    << ", scale v=" << certified_velocity / max_velocity
                    << " a=" << std::sqrt(certified_acceleration / max_acceleration)
                    << " j=" << std::cbrt(certified_jerk / max_jerk)
                    << " yr=" << certified_yaw_rate / max_yaw_rate
                    << " ya=" << std::sqrt(certified_yaw_acceleration / max_yaw_acceleration)
                    << " yj=" << std::cbrt(certified_yaw_jerk / max_yaw_jerk)
                    << "; p0=[" << p0.transpose() << "] v0=[" << v0.transpose()
                    << "] a0=[" << a0.transpose() << "] yaw0=" << yaw0
                    << " yaw_rate0=" << yaw_rate_0 << " yaw_acc0=" << yaw_acceleration_0
                    << " pT=[" << pT.transpose() << "] yawT=" << yawT
                    << " vT=[" << vT.transpose() << "] aT=[" << aT.transpose() << "])";
                throw std::runtime_error(reason.str());
            }
        }

        throw std::runtime_error(
            "Bounded interpolation could not satisfy derivative limits"
        );
    }

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

vector_t TrajectoryInterpolator::jerkFunction(double t) {

    Eigen::Matrix<double, 1, 6> A_t;
    A_t << 0, 0, 0, 6, 24 * t, 60 * t * t;
    const Eigen::Matrix<double, 1, 3> jerk = A_t * q;
    return vector_t(jerk(0), jerk(1), jerk(2));

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

double TrajectoryInterpolator::yawAccelerationFunction(double t) {
    Eigen::Matrix<double, 1, 6> A_t;
    A_t << 0, 0, 2, 6 * t, 12 * t * t, 20 * t * t * t;
    return static_cast<double>((A_t * q_yaw)(0));
}

double TrajectoryInterpolator::yawJerkFunction(double t) {
    Eigen::Matrix<double, 1, 6> A_t;
    A_t << 0, 0, 0, 6, 24 * t, 60 * t * t;
    return static_cast<double>((A_t * q_yaw)(0));
}

Reference TrajectoryInterpolator::referenceFunction(double t) {

    if (start_time_ + rclcpp::Duration::from_seconds(t) > end_time_) {

        // Holding the endpoint still produces a new sample. Keep its sample
        // time advancing just as it does during interpolation.
        return reference_.CopyWithNewStamp(start_time_ + rclcpp::Duration::from_seconds(t));

    }

    point_t p = positionFunction(t);
    vector_t v = velocityFunction(t);
    vector_t a = accelerationFunction(t);
    double yaw = yawFunction(t);
    double yaw_rate = yawRateFunction(t);
    double yaw_acceleration = bounded_interpolation_ ? yawAccelerationFunction(t) : 0.0;

    return Reference(
        p,
        yaw,
        v,
        yaw_rate,
        a,
        yaw_acceleration,
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
