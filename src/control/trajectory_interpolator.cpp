/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/trajectory_interpolator.hpp>

using namespace iii_drone::control;
using namespace iii_drone::configuration;
using namespace iii_drone::adapters;
using namespace iii_drone::types;

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

    return ComputeReferenceTrajectory(
        Reference(start_state),
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
        reference_ = end_reference;

        start_time_ = rclcpp::Clock().now();

        double T = computeInterpolation(
            start_reference,
            end_reference
        );

        end_time_ = start_time_ + rclcpp::Duration::from_seconds(T);

        t = 0;

    } else if (set_reference) {

        reference_ = end_reference;

        Reference new_start_reference = referenceFunction(t);

        start_time_ = rclcpp::Clock().now();

        double T = computeInterpolation(
            new_start_reference,
            end_reference
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
    // const vector_t v0 = start_reference.velocity();
    // const vector_t a0 = start_reference.acceleration();
    const vector_t v0 = vector_t::Zero();
    const vector_t a0 = vector_t::Zero();
    const double yaw0 = rescale_yaw(start_reference.yaw());
    const double yaw_rate_0 = start_reference.yaw_rate();
    const double yaw_acceleration_0 = 0;

    const point_t pT = end_reference.position();
    const vector_t vT = end_reference.velocity(); 
    const vector_t aT = end_reference.acceleration();
    const double yawT = rescale_yaw(end_reference.yaw());
    const double yaw_rate_T = end_reference.yaw_rate();
    const double yaw_acceleration_T = 0;

    const double t0 = 0.0;
    const double T = (pT - p0).norm() / configuration_->GetParameter("/control/trajectory_interpolator/interpolation_avg_velocity_m_s").as_double();

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
