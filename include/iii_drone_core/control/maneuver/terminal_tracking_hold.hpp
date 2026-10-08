#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <iii_drone_core/control/combined_drone_awareness_handler.hpp>
#include <iii_drone_core/control/terminal_position_tracking_controller.hpp>

namespace iii_drone::control::maneuver {

/** One Core-owned command source shared by terminal approach and its hover. */
class TerminalTrackingHold {
public:
    enum class Phase { Tracking, Stopping, Degraded, Unrecoverable };
    using Clearance = std::function<double(const iii_drone::types::point_t &)>;

    TerminalTrackingHold(
        Reference nominal,
        CombinedDroneAwarenessHandler::SharedPtr awareness,
        rclcpp::Clock::SharedPtr clock,
        Clearance minimum_cable_distance,
        double required_clearance_m,
        TerminalPositionTrackingController::Limits limits = {}
    );

    Reference GetReference();
    bool RequestQuiescence();
    bool isQuiescent() const;
    bool ResumeTracking();
    // Called by a maneuver adopting this hold after a handover pause; see
    // TerminalPositionTrackingController::ResumeAfterHandover().
    void ResumeAfterHandover();
    void Fail(const std::string & reason);
    Phase phase() const;
    std::string failureReason() const;
    Reference lastCommand() const;
    Reference nominalReference() const;

private:
    void beginFailure(const std::string & reason, const rclcpp::Time & now);

    mutable std::mutex mutex_;
    Reference nominal_;
    CombinedDroneAwarenessHandler::SharedPtr awareness_;
    rclcpp::Clock::SharedPtr clock_;
    Clearance minimum_cable_distance_;
    double required_clearance_m_;
    TerminalPositionTrackingController controller_;
    Reference last_command_;
    Phase phase_ = Phase::Tracking;
    std::string failure_reason_;
    // One fault-only record; never copied into the published reference stream.
    std::string last_freshness_diagnostic_;
};

}  // namespace iii_drone::control::maneuver
