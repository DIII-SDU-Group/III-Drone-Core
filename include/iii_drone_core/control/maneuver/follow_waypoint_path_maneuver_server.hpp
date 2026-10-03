#pragma once

#include <chrono>
#include <functional>
#include <mutex>
#include <optional>
#include <string>

#include <iii_drone_configuration/configuration.hpp>
#include <iii_drone_interfaces/action/follow_waypoint_path.hpp>

#include <iii_drone_core/control/maneuver/maneuver_server.hpp>
#include <iii_drone_core/control/maneuver/terminal_tracking_hold.hpp>
#include <iii_drone_core/control/estimated_position_stop_proof.hpp>
#include <iii_drone_core/control/waypoint_path_planner.hpp>

namespace iii_drone::control::maneuver {

// Tracks a measured terminal stop across completion polls using monotonic time.
class WaypointPathTerminalStopProof {
public:
    using Clock = std::chrono::steady_clock;

    void reset();
    bool observe(
        bool path_finished_and_at_target,
        const State & measured_state,
        const ControlledCancellationConfig & config,
        Clock::time_point now
    );

private:
    std::optional<Clock::time_point> settled_since_;
};

class FollowWaypointPathManeuverServer : public ManeuverServer {
    using FollowWaypointPath = iii_drone_interfaces::action::FollowWaypointPath;
    using GoalHandle = rclcpp_action::ServerGoalHandle<FollowWaypointPath>;

public:
    FollowWaypointPathManeuverServer(
        rclcpp_lifecycle::LifecycleNode * node,
        CombinedDroneAwarenessHandler::SharedPtr awareness_handler,
        const std::string & action_name,
        unsigned int wait_for_execute_poll_ms,
        unsigned int evaluate_done_poll_ms,
        iii_drone::configuration::Configuration::SharedPtr configuration
    );

    bool CanExecuteManeuver(
        const Maneuver & maneuver,
        const iii_drone::adapters::CombinedDroneAwarenessAdapter & awareness
    ) const override;

    iii_drone::adapters::CombinedDroneAwarenessAdapter ExpectedAwarenessAfterExecution(
        const Maneuver & maneuver
    ) override;
    void RegisterAppliedRestReferenceCallback(
        std::function<bool(const std::string &, const Reference &)> callback);

private:
    maneuver_type_t maneuver_type() const override;
    void startExecution(Maneuver & maneuver) override;
    bool canCancel() override;
    std::optional<ControlledCancellationConfig> controlledCancellationConfig() const override;
    bool controlledCancellationComplete(const ControlledCancellationConfig & config) override;
    bool controlledCancellationFailure() const override;
    bool rebaseExecution(const State & stopped_state, std::string & reason) override;
    Reference computeReference(const State & state) override;
    bool hasSucceeded(Maneuver & maneuver) override;
    bool hasFailed(Maneuver & maneuver) override;
    std::shared_ptr<void> getFeedback(Maneuver & maneuver) override;
    void publishResultAndFinalize(
        Maneuver & maneuver,
        maneuver_result_type_t result_type
    ) override;
    void registerReferenceCallbackOnSuccess(const Maneuver & maneuver) override;

    bool validateManeuverParameters(
        const follow_waypoint_path_maneuver_params_t & params
    ) const;
    std::vector<WaypointPathWaypoint> transformWaypoints(
        const follow_waypoint_path_maneuver_params_t & params
    ) const;
    WaypointPathConstraints resolveConstraints(
        const follow_waypoint_path_maneuver_params_t & params
    ) const;
    std::shared_ptr<TerminalTrackingHold> createTerminalHold(
        const Reference & nominal, const std::string & request_identity, bool quiescent);

    iii_drone::configuration::Configuration::SharedPtr configuration_;
    WaypointPathPlanner planner_;

    mutable std::mutex plan_mutex_;
    WaypointPathPlan plan_;
    WaypointPathSample active_sample_;
    rclcpp::Time execution_start_time_;
    bool has_failed_ = false;
    WaypointPathTerminalStopProof terminal_stop_proof_;
    EstimatedPositionStopProof terminal_motion_proof_;
    EstimatedPositionStopProof cancellation_motion_proof_;
    std::shared_ptr<TerminalTrackingHold> terminal_hold_;
    std::function<bool(const std::string &, const Reference &)> applied_rest_reference_;
    std::optional<std::chrono::steady_clock::time_point> cancellation_proof_started_;
    bool cancellation_proof_failed_ = false;
    std::vector<WaypointPathWaypoint> active_waypoints_;
    WaypointPathConstraints active_constraints_;
    bool active_repeat_ = false;
    uint32_t active_repeat_from_index_ = 0;
};

}  // namespace iii_drone::control::maneuver
