#pragma once

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <iii_drone_configuration/configuration.hpp>

#include <iii_drone_core/adapters/powerline_adapter.hpp>
#include <iii_drone_core/control/reference.hpp>
#include <iii_drone_core/control/reference_trajectory.hpp>
#include <iii_drone_core/control/state.hpp>

namespace iii_drone {
namespace control {

    class CableAwareTrajectoryPlanner {
    public:
        CableAwareTrajectoryPlanner(
            iii_drone::configuration::Configuration::SharedPtr configuration,
            rclcpp_lifecycle::LifecycleNode * node
        );

        ReferenceTrajectory ComputeReferenceTrajectory(
            const State & start_state,
            const Reference & goal_reference,
            const iii_drone::adapters::PowerlineAdapter & powerline,
            bool reset
        );

        typedef std::shared_ptr<CableAwareTrajectoryPlanner> SharedPtr;

    private:
        iii_drone::configuration::Configuration::SharedPtr configuration_;
        rclcpp_lifecycle::LifecycleNode * node_;

        ReferenceTrajectory active_trajectory_;
        Reference goal_reference_;
        rclcpp::Time start_time_;
        double duration_s_{0.0};
        bool has_active_trajectory_{false};
        bool active_trajectory_stream_started_{false};

        std::vector<iii_drone::types::point_t> planAStarPath(
            const iii_drone::types::point_t & start,
            const iii_drone::types::point_t & goal,
            const iii_drone::adapters::PowerlineAdapter & powerline
        ) const;

        ReferenceTrajectory smoothPathLeastSquares(
            const std::vector<iii_drone::types::point_t> & waypoints,
            const State & start_state,
            const Reference & goal_reference
        );

        ReferenceTrajectory buildPiecewiseLinearTrajectory(
            const std::vector<iii_drone::types::point_t> & waypoints,
            const State & start_state,
            const Reference & goal_reference
        );

        bool trajectoryMeetsBoundaryContract(
            const ReferenceTrajectory & trajectory,
            const State & start_state,
            const Reference & goal_reference
        ) const;

        bool trajectoryIsSafe(
            const ReferenceTrajectory & trajectory,
            const iii_drone::adapters::PowerlineAdapter & powerline,
            const iii_drone::types::point_t & start,
            const iii_drone::types::point_t & goal,
            bool start_requires_terminal_exception,
            bool goal_requires_terminal_exception
        ) const;

        bool segmentIsSafe(
            const iii_drone::types::point_t & a,
            const iii_drone::types::point_t & b,
            const iii_drone::adapters::PowerlineAdapter & powerline
        ) const;

        bool segmentIsSafeForPlanning(
            const iii_drone::types::point_t & a,
            const iii_drone::types::point_t & b,
            const iii_drone::adapters::PowerlineAdapter & powerline,
            const iii_drone::types::point_t & start,
            const iii_drone::types::point_t & goal,
            bool start_requires_terminal_exception,
            bool goal_requires_terminal_exception
        ) const;

        bool pointIsSafe(
            const iii_drone::types::point_t & point,
            const iii_drone::adapters::PowerlineAdapter & powerline
        ) const;

        bool pointIsSafeForPlanning(
            const iii_drone::types::point_t & point,
            const iii_drone::adapters::PowerlineAdapter & powerline,
            const iii_drone::types::point_t & start,
            const iii_drone::types::point_t & goal,
            bool start_requires_terminal_exception,
            bool goal_requires_terminal_exception
        ) const;

        bool pointIsAllowedByTerminalException(
            const iii_drone::types::point_t & point,
            const iii_drone::types::point_t & terminal,
            const iii_drone::adapters::PowerlineAdapter & powerline,
            const iii_drone::adapters::SingleLineAdapter & line
        ) const;

        iii_drone::types::vector_t cableDirection(
            const iii_drone::adapters::PowerlineAdapter & powerline,
            const iii_drone::adapters::SingleLineAdapter & line
        ) const;

        double clearance() const;

        double distanceToCable(
            const iii_drone::types::point_t & point,
            const iii_drone::adapters::PowerlineAdapter & powerline,
            const iii_drone::adapters::SingleLineAdapter & line
        ) const;

        ReferenceTrajectory sampleActiveTrajectory(double elapsed_s) const;
    };

} // namespace control
} // namespace iii_drone
