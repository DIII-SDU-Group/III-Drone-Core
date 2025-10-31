#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// III-Drone-Configuration:

#include <iii_drone_configuration/parameter_bundle.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/types.hpp>
#include <iii_drone_core/utils/math.hpp>

#include <iii_drone_core/control/state.hpp>
#include <iii_drone_core/control/reference.hpp>
#include <iii_drone_core/control/reference_trajectory.hpp>

#include <iii_drone_core/adapters/state_adapter.hpp>
#include <iii_drone_core/adapters/reference_adapter.hpp>
#include <iii_drone_core/adapters/reference_trajectory_adapter.hpp>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>


/*****************************************************************************/
// Std:

#include <vector>
#include <memory>

/*****************************************************************************/
// Eigen:

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/QR>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace control {

    class TrajectoryInterpolator {
    public:
        TrajectoryInterpolator(
            iii_drone::configuration::ParameterBundle::SharedPtr params,
            rclcpp_lifecycle::LifecycleNode * node
        );

        ~TrajectoryInterpolator();

        ReferenceTrajectory ComputeReferenceTrajectory(
            const State &start_state,
            const Reference &end_reference,
            bool set_reference,
            bool reset
        );

        ReferenceTrajectory ComputeReferenceTrajectory(
            const Reference &start_reference,
            const Reference &end_reference,
            bool set_reference,
            bool reset
        );

        typedef std::shared_ptr<TrajectoryInterpolator> SharedPtr;

    private:
        iii_drone::configuration::ParameterBundle::SharedPtr params_;
        rclcpp_lifecycle::LifecycleNode * node_;

        Reference reference_;
        ReferenceTrajectory reference_trajectory_;

        State state_;

        bool first_ = true;

        rclcpp::Time start_time_;
        rclcpp::Time end_time_;

        Eigen::Matrix<double, 6, 3> q;
        Eigen::Matrix<double, 6, 1> q_yaw;

        double computeInterpolation(
            const Reference &start_reference,
            const Reference &end_reference
        );

        iii_drone::types::point_t positionFunction(double t);
        iii_drone::types::vector_t velocityFunction(double t);
        iii_drone::types::vector_t accelerationFunction(double t);

        double yawFunction(double t);
        double yawRateFunction(double t);

        Reference referenceFunction(double t);

        ReferenceTrajectory referenceTrajectoryFunction(double t);
    
    };

} // namespace control
} // namespace iii_drone