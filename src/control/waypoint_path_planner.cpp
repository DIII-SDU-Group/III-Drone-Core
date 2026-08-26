#include <iii_drone_core/control/waypoint_path_planner.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <eigen3/Eigen/Geometry>

#include <iii_drone_core/utils/math.hpp>

using iii_drone::types::point_t;
using iii_drone::types::vector_t;

namespace iii_drone::control {
namespace {

constexpr double kEpsilon = 1.0e-6;
constexpr double kTangentialAccelerationFraction = 0.65;
constexpr double kLateralAccelerationFraction = 0.65;
constexpr double kAccelerationSmoothingWindowS = 0.25;

struct GeometrySample {
    point_t position;
    double yaw = 0.0;
    double speed_limit_m_s = 0.0;
    uint32_t waypoint_index = 0;
    uint32_t primitive_index = 0;
    bool stop = false;
};

double interpolateYaw(double from, double to, double fraction) {
    double error = std::fmod(to - from + M_PI, 2.0 * M_PI);
    if (error < 0.0) {
        error += 2.0 * M_PI;
    }
    return from + (error - M_PI) * fraction;
}

point_t quinticBezier(const std::array<point_t, 6> & controls, double u) {
    const double v = 1.0 - u;
    return
        std::pow(v, 5) * controls[0] +
        5.0 * std::pow(v, 4) * u * controls[1] +
        10.0 * std::pow(v, 3) * u * u * controls[2] +
        10.0 * v * v * u * u * u * controls[3] +
        5.0 * v * std::pow(u, 4) * controls[4] +
        std::pow(u, 5) * controls[5];
}

void appendLine(
    std::vector<GeometrySample> & output,
    const point_t & from,
    const point_t & to,
    double from_yaw,
    double to_yaw,
    double speed_limit,
    uint32_t waypoint_index,
    uint32_t primitive_index,
    double spacing
) {
    const double length = (to - from).norm();
    if (length <= kEpsilon) {
        return;
    }
    const int steps = std::max(1, static_cast<int>(std::ceil(length / spacing)));
    for (int step = 1; step <= steps; ++step) {
        const double u = static_cast<double>(step) / static_cast<double>(steps);
        output.push_back({
            from + u * (to - from),
            interpolateYaw(from_yaw, to_yaw, u),
            speed_limit,
            waypoint_index,
            primitive_index,
            false,
        });
    }
}

void appendBlend(
    std::vector<GeometrySample> & output,
    const point_t & entry,
    const point_t & corner,
    const point_t & exit,
    const vector_t & incoming_direction,
    const vector_t & outgoing_direction,
    double yaw,
    double speed_limit,
    uint32_t waypoint_index,
    uint32_t primitive_index,
    double spacing
) {
    const double entry_distance = (corner - entry).norm();
    const double exit_distance = (exit - corner).norm();
    const double handle = 0.24 * std::min(entry_distance, exit_distance);
    const std::array<point_t, 6> controls{
        entry,
        entry + handle * incoming_direction,
        entry + 2.0 * handle * incoming_direction,
        exit - 2.0 * handle * outgoing_direction,
        exit - handle * outgoing_direction,
        exit,
    };

    double control_polygon_length = 0.0;
    for (std::size_t index = 1; index < controls.size(); ++index) {
        control_polygon_length += (controls[index] - controls[index - 1]).norm();
    }
    const int steps = std::max(
        2,
        static_cast<int>(std::ceil(control_polygon_length / spacing))
    );
    for (int step = 1; step <= steps; ++step) {
        const double u = static_cast<double>(step) / static_cast<double>(steps);
        output.push_back({
            quinticBezier(controls, u),
            yaw,
            speed_limit,
            waypoint_index,
            primitive_index,
            false,
        });
    }
}

std::vector<GeometrySample> buildOpenGeometry(
    const point_t & start_position,
    double start_yaw,
    const std::vector<WaypointPathWaypoint> & route,
    const std::optional<WaypointPathWaypoint> & final_successor,
    double nominal_speed,
    double spacing
) {
    std::vector<GeometrySample> output;
    output.push_back({start_position, start_yaw, nominal_speed, 0, 0, false});

    point_t current_position = start_position;
    double current_yaw = start_yaw;
    uint32_t primitive_index = 0;

    for (std::size_t index = 0; index < route.size(); ++index) {
        const WaypointPathWaypoint & waypoint = route[index];
        const point_t incoming_origin = index == 0
            ? start_position
            : route[index - 1].position;
        const WaypointPathWaypoint * successor = nullptr;
        if (index + 1 < route.size()) {
            successor = &route[index + 1];
        } else if (final_successor) {
            successor = &*final_successor;
        }

        const double speed_limit = waypoint.speed_limit_m_s > 0.0
            ? std::min(waypoint.speed_limit_m_s, nominal_speed)
            : nominal_speed;
        const vector_t incoming = waypoint.position - incoming_origin;
        const bool can_blend =
            waypoint.transition == WaypointTransition::Blend &&
            successor != nullptr &&
            waypoint.blend_radius_m > 0.0 &&
            incoming.norm() > kEpsilon &&
            (successor->position - waypoint.position).norm() > kEpsilon;

        if (can_blend) {
            const vector_t outgoing = successor->position - waypoint.position;
            const vector_t incoming_direction = incoming.normalized();
            const vector_t outgoing_direction = outgoing.normalized();
            const double entry_distance = std::min(
                waypoint.blend_radius_m,
                0.45 * incoming.norm()
            );
            const double exit_distance = std::min(
                waypoint.blend_radius_m,
                0.45 * outgoing.norm()
            );
            const point_t entry = waypoint.position - entry_distance * incoming_direction;
            const point_t exit = waypoint.position + exit_distance * outgoing_direction;

            appendLine(
                output,
                current_position,
                entry,
                current_yaw,
                waypoint.yaw,
                speed_limit,
                static_cast<uint32_t>(index),
                primitive_index++,
                spacing
            );
            appendBlend(
                output,
                entry,
                waypoint.position,
                exit,
                incoming_direction,
                outgoing_direction,
                waypoint.yaw,
                speed_limit,
                static_cast<uint32_t>(index),
                primitive_index++,
                spacing
            );
            current_position = exit;
            current_yaw = waypoint.yaw;
        } else {
            appendLine(
                output,
                current_position,
                waypoint.position,
                current_yaw,
                waypoint.yaw,
                speed_limit,
                static_cast<uint32_t>(index),
                primitive_index++,
                spacing
            );
            current_position = waypoint.position;
            current_yaw = waypoint.yaw;
            if (!output.empty() && waypoint.transition == WaypointTransition::Stop) {
                output.back().stop = true;
            }
        }
    }
    return output;
}

double curvatureAt(const std::vector<GeometrySample> & geometry, std::size_t index) {
    if (index == 0 || index + 1 >= geometry.size()) {
        return 0.0;
    }
    const vector_t a = geometry[index].position - geometry[index - 1].position;
    const vector_t b = geometry[index + 1].position - geometry[index].position;
    const vector_t chord = geometry[index + 1].position - geometry[index - 1].position;
    const double denominator = a.norm() * b.norm() * chord.norm();
    if (denominator <= kEpsilon) {
        return 0.0;
    }
    return 2.0 * a.cross(b).norm() / denominator;
}

struct BaseTimedPath {
    std::vector<double> times;
    std::vector<point_t> positions;
    std::vector<double> yaws;
    std::vector<vector_t> velocities;
    std::vector<vector_t> accelerations;
    std::vector<double> yaw_rates;
    std::vector<uint32_t> waypoint_indices;
    std::vector<uint32_t> primitive_indices;
    double required_time_scale = 1.0;
};

BaseTimedPath parameterize(
    const std::vector<GeometrySample> & geometry,
    const WaypointPathConstraints & constraints,
    double start_speed,
    double end_speed,
    bool periodic = false
) {
    if (geometry.size() < 2) {
        throw std::invalid_argument("Waypoint path geometry must contain at least two samples");
    }

    const std::size_t count = geometry.size();
    const double tangential_acceleration =
        constraints.max_acceleration_m_s2 * kTangentialAccelerationFraction;
    std::vector<double> speed(count, constraints.nominal_speed_m_s);
    for (std::size_t index = 0; index < count; ++index) {
        speed[index] = std::min(
            constraints.nominal_speed_m_s,
            geometry[index].speed_limit_m_s > 0.0
                ? geometry[index].speed_limit_m_s
                : constraints.nominal_speed_m_s
        );
        const double curvature = curvatureAt(geometry, index);
        if (curvature > kEpsilon) {
            speed[index] = std::min(
                speed[index],
                std::sqrt(
                    kLateralAccelerationFraction *
                    constraints.max_acceleration_m_s2 /
                    curvature
                )
            );
        }
        if (geometry[index].stop) {
            speed[index] = 0.0;
        }
    }
    speed.front() = std::min(speed.front(), std::max(0.0, start_speed));
    speed.back() = std::min(speed.back(), std::max(0.0, end_speed));

    for (std::size_t index = 1; index < count; ++index) {
        const double distance = (geometry[index].position - geometry[index - 1].position).norm();
        speed[index] = std::min(
            speed[index],
            std::sqrt(speed[index - 1] * speed[index - 1] + 2.0 * tangential_acceleration * distance)
        );
    }
    for (std::size_t index = count - 1; index-- > 0;) {
        const double distance = (geometry[index + 1].position - geometry[index].position).norm();
        speed[index] = std::min(
            speed[index],
            std::sqrt(speed[index + 1] * speed[index + 1] + 2.0 * tangential_acceleration * distance)
        );
    }

    BaseTimedPath result;
    result.times.resize(count, 0.0);
    result.positions.reserve(count);
    result.yaws.reserve(count);
    result.velocities.resize(count, vector_t::Zero());
    result.accelerations.resize(count, vector_t::Zero());
    result.yaw_rates.resize(count, 0.0);
    result.waypoint_indices.reserve(count);
    result.primitive_indices.reserve(count);

    for (std::size_t index = 0; index < count; ++index) {
        result.positions.push_back(geometry[index].position);
        result.yaws.push_back(geometry[index].yaw);
        result.waypoint_indices.push_back(geometry[index].waypoint_index);
        result.primitive_indices.push_back(geometry[index].primitive_index);
        if (index == 0) {
            continue;
        }
        const double distance = (geometry[index].position - geometry[index - 1].position).norm();
        const double speed_sum = speed[index - 1] + speed[index];
        const double interval = speed_sum > kEpsilon
            ? 2.0 * distance / speed_sum
            : 2.0 * std::sqrt(distance / std::max(tangential_acceleration, kEpsilon));
        result.times[index] = result.times[index - 1] + std::max(interval, 1.0e-4);
    }

    for (std::size_t index = 0; index < count; ++index) {
        vector_t tangent;
        if (index == 0) {
            tangent = result.positions[1] - result.positions[0];
        } else if (index + 1 == count) {
            tangent = result.positions[index] - result.positions[index - 1];
        } else {
            tangent = result.positions[index + 1] - result.positions[index - 1];
        }
        if (tangent.norm() > kEpsilon) {
            result.velocities[index] = speed[index] * tangent.normalized();
        }
    }
    if (periodic) {
        const double seam_speed = std::min(
            static_cast<double>(result.velocities.front().norm()),
            static_cast<double>(result.velocities.back().norm())
        );
        vector_t seam_direction = result.velocities.front() + result.velocities.back();
        if (seam_direction.norm() <= kEpsilon) {
            seam_direction = result.positions[1] - result.positions[0];
        }
        seam_direction.normalize();
        result.velocities.front() = seam_speed * seam_direction;
        result.velocities.back() = result.velocities.front();
    }

    for (std::size_t index = 1; index + 1 < count; ++index) {
        const double interval = result.times[index + 1] - result.times[index - 1];
        result.accelerations[index] =
            (result.velocities[index + 1] - result.velocities[index - 1]) /
            std::max(interval, kEpsilon);
        result.yaw_rates[index] =
            (interpolateYaw(result.yaws[index - 1], result.yaws[index + 1], 1.0) -
                result.yaws[index - 1]) /
            std::max(interval, kEpsilon);
    }
    if (periodic) {
        const double seam_interval =
            (result.times[1] - result.times[0]) +
            (result.times[count - 1] - result.times[count - 2]);
        result.accelerations.front() =
            (result.velocities[1] - result.velocities[count - 2]) /
            std::max(seam_interval, kEpsilon);
        result.accelerations.back() = result.accelerations.front();
    } else {
        result.accelerations.front() =
            (result.velocities[1] - result.velocities[0]) /
            std::max(result.times[1] - result.times[0], kEpsilon);
        result.accelerations.back() =
            (result.velocities[count - 1] - result.velocities[count - 2]) /
            std::max(result.times[count - 1] - result.times[count - 2], kEpsilon);
    }

    // Dense geometric sampling makes a point-to-point acceleration derivative
    // overly sensitive to chord noise. Average over the controller's physical
    // response horizon before evaluating jerk and publishing feed-forward terms.
    const std::vector<vector_t> raw_accelerations = result.accelerations;
    const std::size_t unique_count = periodic ? count - 1 : count;
    for (std::size_t index = 0; index < unique_count; ++index) {
        vector_t sum = vector_t::Zero();
        double weight_sum = 0.0;
        for (std::size_t other = 0; other < unique_count; ++other) {
            double time_distance = std::abs(result.times[other] - result.times[index]);
            if (periodic) {
                time_distance = std::min(
                    time_distance,
                    result.times.back() - time_distance
                );
            }
            if (time_distance > kAccelerationSmoothingWindowS) {
                continue;
            }
            const double weight = 1.0 - time_distance / kAccelerationSmoothingWindowS;
            sum += weight * raw_accelerations[other];
            weight_sum += weight;
        }
        result.accelerations[index] = sum / std::max(weight_sum, kEpsilon);
    }
    if (periodic) {
        result.accelerations.back() = result.accelerations.front();
    }

    // Acceleration is a feed-forward term. Project it onto the configured jerk
    // envelope directly instead of slowing the entire geometric path because of
    // one sampled derivative spike.
    const auto project_jerk_edge = [&](std::size_t from, std::size_t to, double interval) {
        const vector_t delta = result.accelerations[to] - result.accelerations[from];
        const double limit = constraints.max_jerk_m_s3 * std::max(interval, kEpsilon);
        if (delta.norm() <= limit) {
            return;
        }
        const vector_t correction = 0.5 * (1.0 - limit / delta.norm()) * delta;
        result.accelerations[from] += correction;
        result.accelerations[to] -= correction;
    };
    for (int iteration = 0; iteration < 64; ++iteration) {
        for (std::size_t index = 0; index + 1 < unique_count; ++index) {
            project_jerk_edge(
                index,
                index + 1,
                result.times[index + 1] - result.times[index]
            );
        }
        if (periodic) {
            project_jerk_edge(
                unique_count - 1,
                0,
                result.times.back() - result.times[unique_count - 1]
            );
        }
        for (std::size_t index = unique_count - 1; index > 0; --index) {
            project_jerk_edge(
                index - 1,
                index,
                result.times[index] - result.times[index - 1]
            );
        }
    }
    if (periodic) {
        result.accelerations.back() = result.accelerations.front();
    }

    double observed_velocity = 0.0;
    double observed_acceleration = 0.0;
    double observed_jerk = 0.0;
    for (std::size_t index = 0; index < count; ++index) {
        observed_velocity = std::max(observed_velocity, static_cast<double>(result.velocities[index].norm()));
        observed_acceleration = std::max(observed_acceleration, static_cast<double>(result.accelerations[index].norm()));
        if (index > 0) {
            const double interval = result.times[index] - result.times[index - 1];
            observed_jerk = std::max(
                observed_jerk,
                static_cast<double>(((result.accelerations[index] - result.accelerations[index - 1]) /
                    std::max(interval, kEpsilon)).norm())
            );
        }
    }
    result.required_time_scale = 1.01 * std::max({
        1.0,
        observed_velocity / constraints.nominal_speed_m_s,
        std::sqrt(observed_acceleration / constraints.max_acceleration_m_s2),
        std::cbrt(observed_jerk / constraints.max_jerk_m_s3),
    });
    return result;
}

TimedWaypointPath applyTimeScale(const BaseTimedPath & base, double scale) {
    TimedWaypointPath result;
    result.times_s.reserve(base.times.size());
    result.references.reserve(base.times.size());
    result.waypoint_indices = base.waypoint_indices;
    result.primitive_indices = base.primitive_indices;

    for (std::size_t index = 0; index < base.times.size(); ++index) {
        result.times_s.push_back(base.times[index] * scale);
        result.references.emplace_back(
            base.positions[index],
            base.yaws[index],
            base.velocities[index] / scale,
            base.yaw_rates[index] / scale,
            base.accelerations[index] / (scale * scale)
        );
    }
    return result;
}

WaypointPathSample sampleSection(const TimedWaypointPath & section, double time_s) {
    if (section.empty()) {
        throw std::runtime_error("Cannot sample an empty waypoint path section");
    }
    const double clamped_time = std::clamp(time_s, 0.0, section.duration_s());
    const auto upper = std::upper_bound(section.times_s.begin(), section.times_s.end(), clamped_time);
    if (upper == section.times_s.begin()) {
        return {section.references.front(), section.waypoint_indices.front(), section.primitive_indices.front(), 0.0};
    }
    if (upper == section.times_s.end()) {
        return {section.references.back(), section.waypoint_indices.back(), section.primitive_indices.back(), 1.0};
    }

    const std::size_t next = static_cast<std::size_t>(upper - section.times_s.begin());
    const std::size_t previous = next - 1;
    const double interval = section.times_s[next] - section.times_s[previous];
    const double u = (clamped_time - section.times_s[previous]) / std::max(interval, kEpsilon);
    const Reference & a = section.references[previous];
    const Reference & b = section.references[next];
    const Reference interpolated(
        a.position() + u * (b.position() - a.position()),
        interpolateYaw(a.yaw(), b.yaw(), u),
        a.velocity() + u * (b.velocity() - a.velocity()),
        a.yaw_rate() + u * (b.yaw_rate() - a.yaw_rate()),
        a.acceleration() + u * (b.acceleration() - a.acceleration()),
        0.0
    );
    return {
        interpolated,
        section.waypoint_indices[next],
        section.primitive_indices[next],
        section.duration_s() > 0.0 ? clamped_time / section.duration_s() : 1.0,
    };
}

}  // namespace

double TimedWaypointPath::duration_s() const {
    return times_s.empty() ? 0.0 : times_s.back();
}

bool TimedWaypointPath::empty() const {
    return references.empty();
}

double WaypointPathPlan::prefixDurationS() const {
    return prefix.duration_s();
}

double WaypointPathPlan::loopDurationS() const {
    return loop.duration_s();
}

WaypointPathSample WaypointPathPlan::sample(double elapsed_s) const {
    if (!repeating || loop.empty() || elapsed_s <= prefix.duration_s()) {
        return sampleSection(prefix, elapsed_s);
    }
    const double loop_time = std::fmod(
        std::max(0.0, elapsed_s - prefix.duration_s()),
        loop.duration_s()
    );
    return sampleSection(loop, loop_time);
}

std::vector<Reference> WaypointPathPlan::previewReferences() const {
    std::vector<Reference> result = prefix.references;
    if (!loop.empty()) {
        result.insert(result.end(), loop.references.begin(), loop.references.end());
    }
    return result;
}

WaypointPathPlan WaypointPathPlanner::plan(
    const Reference & start_reference,
    const std::vector<WaypointPathWaypoint> & waypoints,
    bool repeat,
    uint32_t repeat_from_index,
    const WaypointPathConstraints & constraints
) const {
    if (waypoints.empty()) {
        throw std::invalid_argument("Waypoint path requires at least one waypoint");
    }
    if (
        constraints.nominal_speed_m_s <= 0.0 ||
        constraints.max_acceleration_m_s2 <= 0.0 ||
        constraints.max_jerk_m_s3 <= 0.0 ||
        constraints.geometry_sample_spacing_m <= 0.0
    ) {
        throw std::invalid_argument("Waypoint path constraints must be positive");
    }
    if (repeat && (repeat_from_index >= waypoints.size() || waypoints.size() - repeat_from_index < 2)) {
        throw std::invalid_argument("Repeating waypoint path requires at least two loop waypoints");
    }

    std::vector<WaypointPathWaypoint> prefix_route = waypoints;
    std::optional<WaypointPathWaypoint> prefix_successor;
    std::vector<GeometrySample> loop_geometry;

    if (repeat) {
        prefix_route.push_back(waypoints[repeat_from_index]);
        prefix_successor = waypoints[repeat_from_index + 1];

        const WaypointPathWaypoint & loop_corner = waypoints[repeat_from_index];
        const WaypointPathWaypoint & predecessor = waypoints.back();
        const WaypointPathWaypoint & successor = waypoints[repeat_from_index + 1];
        const vector_t incoming = loop_corner.position - predecessor.position;
        const vector_t outgoing = successor.position - loop_corner.position;
        if (incoming.norm() <= kEpsilon || outgoing.norm() <= kEpsilon) {
            throw std::invalid_argument("Loop seam contains duplicate waypoint positions");
        }
        const double exit_distance = std::min(loop_corner.blend_radius_m, 0.45 * outgoing.norm());
        const point_t seam = loop_corner.position + exit_distance * outgoing.normalized();

        std::vector<WaypointPathWaypoint> loop_route(
            waypoints.begin() + repeat_from_index + 1,
            waypoints.end()
        );
        loop_route.push_back(loop_corner);
        loop_geometry = buildOpenGeometry(
            seam,
            loop_corner.yaw,
            loop_route,
            successor,
            constraints.nominal_speed_m_s,
            constraints.geometry_sample_spacing_m
        );
        const uint32_t first_successor_index = repeat_from_index + 1;
        for (auto & sample : loop_geometry) {
            const uint32_t goal_index = first_successor_index + sample.waypoint_index;
            sample.waypoint_index = goal_index < waypoints.size()
                ? goal_index
                : repeat_from_index;
        }
    }

    std::vector<GeometrySample> prefix_geometry = buildOpenGeometry(
        start_reference.position(),
        start_reference.yaw(),
        prefix_route,
        prefix_successor,
        constraints.nominal_speed_m_s,
        constraints.geometry_sample_spacing_m
    );
    if (repeat) {
        // prefix_route contains one synthetic copy of the loop-start waypoint
        // to generate a continuous seam. Feedback must still reference the
        // caller's original waypoint array, where that target is repeat_from_index.
        for (auto & sample : prefix_geometry) {
            if (sample.waypoint_index >= waypoints.size()) {
                sample.waypoint_index = repeat_from_index;
            }
        }
    }

    BaseTimedPath loop_base;
    double seam_speed = 0.0;
    if (repeat) {
        seam_speed = constraints.nominal_speed_m_s;
        loop_base = parameterize(loop_geometry, constraints, seam_speed, seam_speed, true);
        seam_speed = loop_base.velocities.front().norm();
    }
    BaseTimedPath prefix_base = parameterize(
        prefix_geometry,
        constraints,
        std::max(0.0, static_cast<double>(start_reference.velocity().norm())),
        repeat ? seam_speed : 0.0
    );

    const double time_scale = repeat
        ? std::max(prefix_base.required_time_scale, loop_base.required_time_scale)
        : prefix_base.required_time_scale;

    WaypointPathPlan result;
    result.prefix = applyTimeScale(prefix_base, time_scale);
    result.repeating = repeat;
    if (repeat) {
        result.loop = applyTimeScale(loop_base, time_scale);
    }
    return result;
}

}  // namespace iii_drone::control
