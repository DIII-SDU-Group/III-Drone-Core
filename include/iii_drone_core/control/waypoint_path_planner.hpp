#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include <iii_drone_core/control/reference.hpp>
#include <iii_drone_core/utils/types.hpp>

namespace iii_drone::control {

enum class WaypointTransition : uint8_t {
    Stop = 0,
    Blend = 1,
};

struct WaypointPathWaypoint {
    iii_drone::types::point_t position;
    double yaw = 0.0;
    WaypointTransition transition = WaypointTransition::Stop;
    double blend_radius_m = 0.0;
    double speed_limit_m_s = 0.0;
};

struct WaypointPathConstraints {
    double nominal_speed_m_s = 1.0;
    double max_acceleration_m_s2 = 0.5;
    double max_jerk_m_s3 = 0.5;
    double geometry_sample_spacing_m = 0.03;
};

struct TimedWaypointPath {
    std::vector<double> times_s;
    std::vector<Reference> references;
    std::vector<uint32_t> waypoint_indices;
    std::vector<uint32_t> primitive_indices;

    double duration_s() const;
    bool empty() const;
};

struct WaypointPathSample {
    Reference reference;
    uint32_t waypoint_index = 0;
    uint32_t primitive_index = 0;
    double progress = 0.0;
};

class WaypointPathPlan {
public:
    TimedWaypointPath prefix;
    TimedWaypointPath loop;
    bool repeating = false;

    double prefixDurationS() const;
    double loopDurationS() const;
    WaypointPathSample sample(double elapsed_s) const;
    std::vector<Reference> previewReferences() const;
};

class WaypointPathPlanner {
public:
    WaypointPathPlan plan(
        const Reference & start_reference,
        const std::vector<WaypointPathWaypoint> & waypoints,
        bool repeat,
        uint32_t repeat_from_index,
        const WaypointPathConstraints & constraints
    ) const;
};

}  // namespace iii_drone::control
