#include <iii_drone_core/control/cable_aware_trajectory_planner.hpp>

#include <iii_drone_core/utils/math.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/QR>

using namespace iii_drone::adapters;
using namespace iii_drone::configuration;
using namespace iii_drone::control;
using namespace iii_drone::types;

namespace {

struct GridIndex {
    int x;
    int y;
    int z;

    bool operator==(const GridIndex & other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct GridIndexHash {
    std::size_t operator()(const GridIndex & index) const {
        std::size_t seed = 0;
        seed ^= std::hash<int>{}(index.x) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= std::hash<int>{}(index.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= std::hash<int>{}(index.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
};

struct QueueEntry {
    GridIndex index;
    double f_score;

    bool operator<(const QueueEntry & other) const {
        return f_score > other.f_score;
    }
};

double getDouble(const Configuration::SharedPtr & configuration, const std::string & name, double fallback) {
    if (!configuration) {
        return fallback;
    }
    if (configuration->HasParameter(name)) {
        return configuration->GetParameter(name).as_double();
    }
    return fallback;
}

int getInt(const Configuration::SharedPtr & configuration, const std::string & name, int fallback) {
    if (!configuration) {
        return fallback;
    }
    if (configuration->HasParameter(name)) {
        return configuration->GetParameter(name).as_int();
    }
    return fallback;
}

point_t minPoint(const point_t & a, const point_t & b) {
    return point_t(std::min(a.x(), b.x()), std::min(a.y(), b.y()), std::min(a.z(), b.z()));
}

point_t maxPoint(const point_t & a, const point_t & b) {
    return point_t(std::max(a.x(), b.x()), std::max(a.y(), b.y()), std::max(a.z(), b.z()));
}

double yawLerp(double yaw0, double yaw1, double alpha) {
    const double delta = std::atan2(std::sin(yaw1 - yaw0), std::cos(yaw1 - yaw0));
    return yaw0 + alpha * delta;
}

} // namespace

CableAwareTrajectoryPlanner::CableAwareTrajectoryPlanner(
    Configuration::SharedPtr configuration,
    rclcpp_lifecycle::LifecycleNode * node
) : configuration_(configuration), node_(node) { }

ReferenceTrajectory CableAwareTrajectoryPlanner::ComputeReferenceTrajectory(
    const State & start_state,
    const Reference & goal_reference,
    const PowerlineAdapter & powerline,
    bool reset
) {
    if (!has_active_trajectory_ || reset || (goal_reference.position() - goal_reference_.position()).norm() > 1.0e-3) {
        if (!pointIsSafe(goal_reference.position(), powerline)) {
            throw std::runtime_error("CableAwareTrajectoryPlanner: goal position violates cable clearance.");
        }
        const auto waypoints = planAStarPath(start_state.position(), goal_reference.position(), powerline);
        active_trajectory_ = smoothPathLeastSquares(waypoints, start_state, goal_reference);
        const bool start_requires_terminal_exception = !pointIsSafe(start_state.position(), powerline);
        const bool goal_requires_terminal_exception = false;
        const bool smoothing_boundary_valid = trajectoryMeetsBoundaryContract(
            active_trajectory_, start_state, goal_reference);
        if (!smoothing_boundary_valid || !trajectoryIsSafe(
                active_trajectory_,
                powerline,
                start_state.position(),
                goal_reference.position(),
                start_requires_terminal_exception,
                goal_requires_terminal_exception
            )) {
            RCLCPP_WARN(
                node_->get_logger(),
                "CableAwareTrajectoryPlanner::ComputeReferenceTrajectory(): LLS-smoothed trajectory violates cable clearance or terminal constraints; using piecewise-linear A* trajectory."
            );
            active_trajectory_ = buildPiecewiseLinearTrajectory(waypoints, start_state, goal_reference);
        }
        if (!trajectoryIsSafe(
                active_trajectory_,
                powerline,
                start_state.position(),
                goal_reference.position(),
                start_requires_terminal_exception,
                goal_requires_terminal_exception
            )) {
            throw std::runtime_error("CableAwareTrajectoryPlanner: generated trajectory violates cable clearance.");
        }
        goal_reference_ = goal_reference;
        start_time_ = start_state.stamp();
        has_active_trajectory_ = true;
        active_trajectory_stream_started_ = false;
    } else if (!active_trajectory_stream_started_) {
        start_time_ = start_state.stamp();
        active_trajectory_stream_started_ = true;
        RCLCPP_DEBUG(
            node_->get_logger(),
            "CableAwareTrajectoryPlanner::ComputeReferenceTrajectory(): Starting active cable-aware trajectory stream from current state timestamp."
        );
    }

    const double elapsed_s = std::max(0.0, (start_state.stamp() - start_time_).seconds());
    return sampleActiveTrajectory(elapsed_s);
}

std::vector<point_t> CableAwareTrajectoryPlanner::planAStarPath(
    const point_t & start,
    const point_t & goal,
    const PowerlineAdapter & powerline
) const {
    if (powerline.single_line_adapters().empty()) {
        throw std::runtime_error("CableAwareTrajectoryPlanner: no powerline map available.");
    }
    const bool start_requires_terminal_exception = !pointIsSafe(start, powerline);
    const bool goal_requires_terminal_exception = false;
    if (!pointIsSafe(goal, powerline)) {
        throw std::runtime_error("CableAwareTrajectoryPlanner: goal position violates cable clearance.");
    }
    if (start_requires_terminal_exception) {
        RCLCPP_WARN(
            node_->get_logger(),
            "CableAwareTrajectoryPlanner::planAStarPath(): Start position violates cable clearance; allowing bounded terminal exception while planning escape path."
        );
    }

    const double resolution = getDouble(configuration_, "/control/trajectory_generator/cable_aware_grid_resolution_m", 0.5);
    const double margin = getDouble(configuration_, "/control/trajectory_generator/cable_aware_grid_margin_m", 3.0);
    const int max_expansions = getInt(configuration_, "/control/trajectory_generator/cable_aware_max_astar_expansions", 50000);

    point_t bounds_min = minPoint(start, goal);
    point_t bounds_max = maxPoint(start, goal);
    double max_line_z = -std::numeric_limits<double>::infinity();
    double min_cross_corridor_coordinate = std::numeric_limits<double>::infinity();
    double max_cross_corridor_coordinate = -std::numeric_limits<double>::infinity();

    vector_t powerline_direction = powerline.projection_plane().normal;
    powerline_direction.z() = 0.0;
    if (powerline_direction.norm() < 1.0e-6) {
        const vector_t first_direction = iii_drone::math::quatToMat(powerline.single_line_adapters().front().quaternion()).col(0);
        powerline_direction = first_direction;
        powerline_direction.z() = 0.0;
    }
    if (powerline_direction.norm() < 1.0e-6) {
        powerline_direction = vector_t(1.0, 0.0, 0.0);
    }
    powerline_direction.normalize();
    const vector_t cross_corridor_direction(-powerline_direction.y(), powerline_direction.x(), 0.0);

    for (const auto & line : powerline.single_line_adapters()) {
        bounds_min = minPoint(bounds_min, line.position());
        bounds_max = maxPoint(bounds_max, line.position());
        max_line_z = std::max(max_line_z, static_cast<double>(line.position().z()));
        const double cross_corridor_coordinate = line.position().dot(cross_corridor_direction);
        min_cross_corridor_coordinate = std::min(min_cross_corridor_coordinate, cross_corridor_coordinate);
        max_cross_corridor_coordinate = std::max(max_cross_corridor_coordinate, cross_corridor_coordinate);
    }
    bounds_min -= point_t(margin, margin, margin);
    bounds_max += point_t(margin, margin, margin);

    auto toIndex = [&](const point_t & point) {
        return GridIndex{
            static_cast<int>(std::round((point.x() - bounds_min.x()) / resolution)),
            static_cast<int>(std::round((point.y() - bounds_min.y()) / resolution)),
            static_cast<int>(std::round((point.z() - bounds_min.z()) / resolution)),
        };
    };

    auto toPoint = [&](const GridIndex & index) {
        return point_t(
            bounds_min.x() + index.x * resolution,
            bounds_min.y() + index.y * resolution,
            bounds_min.z() + index.z * resolution
        );
    };

    auto inBounds = [&](const GridIndex & index) {
        const point_t point = toPoint(index);
        return point.x() >= bounds_min.x() && point.y() >= bounds_min.y() && point.z() >= bounds_min.z()
            && point.x() <= bounds_max.x() && point.y() <= bounds_max.y() && point.z() <= bounds_max.z();
    };

    auto heuristic = [&](const GridIndex & index) {
        return (toPoint(index) - goal).norm();
    };

    const GridIndex start_index = toIndex(start);
    const GridIndex goal_index = toIndex(goal);

    if (start_index == goal_index) {
        if (!segmentIsSafeForPlanning(
                start,
                goal,
                powerline,
                start,
                goal,
                start_requires_terminal_exception,
                goal_requires_terminal_exception
            )) {
            throw std::runtime_error("CableAwareTrajectoryPlanner: direct path within one grid cell violates cable clearance.");
        }
        return {start, goal};
    }

    if (segmentIsSafeForPlanning(
            start,
            goal,
            powerline,
            start,
            goal,
            start_requires_terminal_exception,
            goal_requires_terminal_exception
        )) {
        return {start, goal};
    }

    std::priority_queue<QueueEntry> open_set;
    std::unordered_map<GridIndex, GridIndex, GridIndexHash> came_from;
    std::unordered_map<GridIndex, double, GridIndexHash> g_score;
    std::unordered_set<GridIndex, GridIndexHash> closed;

    open_set.push({start_index, heuristic(start_index)});
    g_score[start_index] = 0.0;

    const std::array<GridIndex, 26> neighbors = []() {
        std::array<GridIndex, 26> result{};
        int idx = 0;
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    if (dx == 0 && dy == 0 && dz == 0) {
                        continue;
                    }
                    result[idx++] = GridIndex{dx, dy, dz};
                }
            }
        }
        return result;
    }();

    int expansions = 0;
    while (!open_set.empty() && expansions < max_expansions) {
        const GridIndex current = open_set.top().index;
        open_set.pop();

        if (closed.contains(current)) {
            continue;
        }
        closed.insert(current);
        ++expansions;

        if (current == goal_index) {
            std::vector<point_t> path;
            GridIndex cursor = current;
            path.push_back(goal);
            while (!(cursor == start_index)) {
                cursor = came_from.at(cursor);
                path.push_back(toPoint(cursor));
            }
            path.back() = start;
            std::reverse(path.begin(), path.end());
            if (path.size() >= 2 && !segmentIsSafeForPlanning(
                    start,
                    path[1],
                    powerline,
                    start,
                    goal,
                    start_requires_terminal_exception,
                    goal_requires_terminal_exception
                )) {
                throw std::runtime_error("CableAwareTrajectoryPlanner: exact initial segment violates cable clearance.");
            }
            if (path.size() >= 2 && !segmentIsSafeForPlanning(
                    path[path.size() - 2],
                    goal,
                    powerline,
                    start,
                    goal,
                    start_requires_terminal_exception,
                    goal_requires_terminal_exception
                )) {
                throw std::runtime_error("CableAwareTrajectoryPlanner: exact final segment violates cable clearance.");
            }
            return path;
        }

        const point_t current_point = toPoint(current);
        for (const auto & delta : neighbors) {
            const GridIndex next{current.x + delta.x, current.y + delta.y, current.z + delta.z};
            if (!inBounds(next) || closed.contains(next)) {
                continue;
            }

            const point_t next_point = toPoint(next);
            if (!pointIsSafeForPlanning(
                    next_point,
                    powerline,
                    start,
                    goal,
                    start_requires_terminal_exception,
                    goal_requires_terminal_exception
                )
                || !segmentIsSafeForPlanning(
                    current_point,
                    next_point,
                    powerline,
                    start,
                    goal,
                    start_requires_terminal_exception,
                    goal_requires_terminal_exception
                )) {
                continue;
            }

            const double tentative_g = g_score[current] + (next_point - current_point).norm();
            const auto known_g = g_score.find(next);
            if (known_g == g_score.end() || tentative_g < known_g->second) {
                came_from[next] = current;
                g_score[next] = tentative_g;
                open_set.push({next, tentative_g + heuristic(next)});
            }
        }
    }

    auto deterministicDetour = [&](double side_sign, bool use_top_detour) -> std::optional<std::vector<point_t>> {
        const double side_coordinate = side_sign < 0.0
            ? min_cross_corridor_coordinate - clearance() - margin
            : max_cross_corridor_coordinate + clearance() + margin;
        auto moveToSide = [&](const point_t & point) {
            const double delta = side_coordinate - point.dot(cross_corridor_direction);
            return point + delta * cross_corridor_direction;
        };

        std::vector<point_t> candidate;
        candidate.push_back(start);
        candidate.push_back(moveToSide(start));

        if (use_top_detour) {
            const double top_z = max_line_z + clearance() + margin;
            point_t start_side_top = candidate.back();
            start_side_top.z() = static_cast<float>(std::max(static_cast<double>(start_side_top.z()), top_z));
            point_t goal_side_top = moveToSide(goal);
            goal_side_top.z() = static_cast<float>(std::max(static_cast<double>(goal_side_top.z()), top_z));
            point_t goal_side = moveToSide(goal);
            candidate.push_back(start_side_top);
            candidate.push_back(goal_side_top);
            candidate.push_back(goal_side);
        } else {
            candidate.push_back(moveToSide(goal));
        }

        candidate.push_back(goal);

        std::vector<point_t> compact;
        compact.reserve(candidate.size());
        for (const point_t & point : candidate) {
            if (compact.empty() || (point - compact.back()).norm() > std::max(0.05, resolution * 0.25)) {
                compact.push_back(point);
            }
        }

        for (std::size_t i = 0; i < compact.size(); ++i) {
            if (!pointIsSafeForPlanning(
                    compact[i],
                    powerline,
                    start,
                    goal,
                    start_requires_terminal_exception,
                    goal_requires_terminal_exception
                )) {
                return std::nullopt;
            }
            if (i > 0 && !segmentIsSafeForPlanning(
                    compact[i - 1],
                    compact[i],
                    powerline,
                    start,
                    goal,
                    start_requires_terminal_exception,
                    goal_requires_terminal_exception
                )) {
                return std::nullopt;
            }
        }
        return compact;
    };

    for (const double side_sign : {-1.0, 1.0}) {
        for (const bool use_top_detour : {false, true}) {
            std::optional<std::vector<point_t>> detour = deterministicDetour(side_sign, use_top_detour);
            if (detour.has_value()) {
                RCLCPP_WARN(
                    node_->get_logger(),
                    "CableAwareTrajectoryPlanner::planAStarPath(): A* failed after %d expansions; using validated deterministic %s-side %sdetour with %zu waypoint(s).",
                    expansions,
                    side_sign < 0.0 ? "negative" : "positive",
                    use_top_detour ? "top " : "",
                    detour->size()
                );
                return detour.value();
            }
        }
    }

    throw std::runtime_error("CableAwareTrajectoryPlanner: A* failed to find a cable-safe path.");
}

ReferenceTrajectory CableAwareTrajectoryPlanner::smoothPathLeastSquares(
    const std::vector<point_t> & waypoints,
    const State & start_state,
    const Reference & goal_reference
) {
    if (waypoints.size() < 2) {
        return ReferenceTrajectory({goal_reference.CopyWithNewStamp(start_state.stamp())});
    }

    const double avg_velocity = getDouble(configuration_, "/control/trajectory_interpolator/interpolation_avg_velocity_m_s", 0.5);
    const int horizon_count = getInt(configuration_, "/control/trajectory_interpolator/reference_trajectory_length_N", 10);
    const double dt = getDouble(configuration_, "/control/dt", 0.2);

    double path_length = 0.0;
    std::vector<double> cumulative{0.0};
    for (std::size_t i = 1; i < waypoints.size(); ++i) {
        path_length += (waypoints[i] - waypoints[i - 1]).norm();
        cumulative.push_back(path_length);
    }

    duration_s_ = std::max(path_length / std::max(avg_velocity, 1.0e-3), dt * (horizon_count - 1));
    const int sample_count = std::max(horizon_count, static_cast<int>(std::ceil(duration_s_ / dt)) + horizon_count);

    const int degree = std::min<int>(5, static_cast<int>(waypoints.size()) - 1);
    Eigen::MatrixXd A(waypoints.size() + 4, degree + 1);
    Eigen::MatrixXd bx(waypoints.size() + 4, 3);
    A.setZero();
    bx.setZero();

    for (std::size_t i = 0; i < waypoints.size(); ++i) {
        const double t = path_length > 1.0e-6 ? (cumulative[i] / path_length) * duration_s_ : 0.0;
        double pow_t = 1.0;
        for (int j = 0; j <= degree; ++j) {
            A(i, j) = pow_t;
            pow_t *= t;
        }
        bx(i, 0) = waypoints[i].x();
        bx(i, 1) = waypoints[i].y();
        bx(i, 2) = waypoints[i].z();
    }

    const std::size_t velocity_start_row = waypoints.size();
    const std::size_t acceleration_start_row = waypoints.size() + 1;
    const std::size_t velocity_end_row = waypoints.size() + 2;
    const std::size_t acceleration_end_row = waypoints.size() + 3;

    for (int j = 1; j <= degree; ++j) {
        A(velocity_start_row, j) = j;
        A(acceleration_start_row, j) = j >= 2 ? j * (j - 1) : 0.0;
        A(velocity_end_row, j) = j * std::pow(duration_s_, j - 1);
        A(acceleration_end_row, j) = j >= 2 ? j * (j - 1) * std::pow(duration_s_, j - 2) : 0.0;
    }

    bx(velocity_start_row, 0) = start_state.velocity().x();
    bx(velocity_start_row, 1) = start_state.velocity().y();
    bx(velocity_start_row, 2) = start_state.velocity().z();
    bx(velocity_end_row, 0) = goal_reference.velocity().x();
    bx(velocity_end_row, 1) = goal_reference.velocity().y();
    bx(velocity_end_row, 2) = goal_reference.velocity().z();

    const Eigen::MatrixXd coeff = A.colPivHouseholderQr().solve(bx);

    std::vector<Reference> references;
    references.reserve(sample_count);
    for (int i = 0; i < sample_count; ++i) {
        const double t = std::min(duration_s_, i * dt);
        Eigen::RowVectorXd basis(degree + 1);
        Eigen::RowVectorXd dbasis(degree + 1);
        Eigen::RowVectorXd ddbasis(degree + 1);
        for (int j = 0; j <= degree; ++j) {
            basis(j) = std::pow(t, j);
            dbasis(j) = j == 0 ? 0.0 : j * std::pow(t, j - 1);
            ddbasis(j) = j < 2 ? 0.0 : j * (j - 1) * std::pow(t, j - 2);
        }
        const Eigen::RowVector3d p = basis * coeff;
        const Eigen::RowVector3d v = dbasis * coeff;
        const Eigen::RowVector3d a = ddbasis * coeff;
        const double alpha = duration_s_ > 1.0e-6 ? std::clamp(t / duration_s_, 0.0, 1.0) : 1.0;
        references.emplace_back(
            point_t(p.x(), p.y(), p.z()),
            yawLerp(start_state.yaw(), goal_reference.yaw(), alpha),
            vector_t(v.x(), v.y(), v.z()),
            0.0,
            vector_t(a.x(), a.y(), a.z()),
            0.0,
            start_state.stamp() + rclcpp::Duration::from_seconds(t)
        );
    }

    return ReferenceTrajectory(references);
}

ReferenceTrajectory CableAwareTrajectoryPlanner::buildPiecewiseLinearTrajectory(
    const std::vector<point_t> & waypoints,
    const State & start_state,
    const Reference & goal_reference
) {
    if (waypoints.size() < 2) {
        return ReferenceTrajectory({goal_reference.CopyWithNewStamp(start_state.stamp())});
    }

    const double avg_velocity = getDouble(configuration_, "/control/trajectory_interpolator/interpolation_avg_velocity_m_s", 0.5);
    const int horizon_count = getInt(configuration_, "/control/trajectory_interpolator/reference_trajectory_length_N", 10);
    const double dt = getDouble(configuration_, "/control/dt", 0.2);

    double path_length = 0.0;
    std::vector<double> cumulative{0.0};
    for (std::size_t i = 1; i < waypoints.size(); ++i) {
        path_length += (waypoints[i] - waypoints[i - 1]).norm();
        cumulative.push_back(path_length);
    }

    duration_s_ = std::max(path_length / std::max(avg_velocity, 1.0e-3), dt * (horizon_count - 1));
    const int sample_count = std::max(horizon_count, static_cast<int>(std::ceil(duration_s_ / dt)) + horizon_count);

    std::vector<Reference> references;
    references.reserve(sample_count);
    for (int i = 0; i < sample_count; ++i) {
        const double t = std::min(duration_s_, i * dt);
        const double distance = duration_s_ > 1.0e-6 ? (t / duration_s_) * path_length : path_length;

        std::size_t segment = 1;
        while (segment < cumulative.size() - 1 && cumulative[segment] < distance) {
            ++segment;
        }

        const double segment_length = std::max(cumulative[segment] - cumulative[segment - 1], 1.0e-6);
        const double segment_alpha = std::clamp((distance - cumulative[segment - 1]) / segment_length, 0.0, 1.0);
        const vector_t segment_delta = waypoints[segment] - waypoints[segment - 1];
        const point_t position = waypoints[segment - 1] + segment_alpha * segment_delta;
        vector_t velocity = vector_t::Zero();
        if (segment_delta.norm() > 1.0e-6) {
            velocity = segment_delta.normalized() * avg_velocity;
        }
        const double alpha = duration_s_ > 1.0e-6 ? std::clamp(t / duration_s_, 0.0, 1.0) : 1.0;

        const rclcpp::Time stamp = start_state.stamp() + rclcpp::Duration::from_seconds(t);
        if (t >= duration_s_) {
            references.push_back(goal_reference.CopyWithNewStamp(stamp));
        } else {
            references.emplace_back(
                position,
                yawLerp(start_state.yaw(), goal_reference.yaw(), alpha),
                i == 0 ? start_state.velocity() : velocity,
                0.0,
                vector_t::Zero(),
                0.0,
                stamp
            );
        }
    }

    return ReferenceTrajectory(references);
}

bool CableAwareTrajectoryPlanner::trajectoryMeetsBoundaryContract(
    const ReferenceTrajectory & trajectory,
    const State & start_state,
    const Reference & goal_reference
) const {
    constexpr double tolerance = 1.0e-3;
    const auto & references = trajectory.references();
    return !references.empty()
        && (references.front().position() - start_state.position()).norm() <= tolerance
        && (references.front().velocity() - start_state.velocity()).norm() <= tolerance
        && references.front().acceleration().norm() <= tolerance
        && (references.back().position() - goal_reference.position()).norm() <= tolerance
        && (references.back().velocity() - goal_reference.velocity()).norm() <= tolerance
        && (references.back().acceleration() - goal_reference.acceleration()).norm() <= tolerance;
}

bool CableAwareTrajectoryPlanner::trajectoryIsSafe(
    const ReferenceTrajectory & trajectory,
    const PowerlineAdapter & powerline,
    const point_t & start,
    const point_t & goal,
    bool start_requires_terminal_exception,
    bool goal_requires_terminal_exception
) const {
    const auto & references = trajectory.references();
    for (std::size_t i = 0; i < references.size(); ++i) {
        const auto & reference = references[i];
        if (!pointIsSafeForPlanning(
                reference.position(),
                powerline,
                start,
                goal,
                start_requires_terminal_exception,
                goal_requires_terminal_exception
            )) {
            return false;
        }
        if (i > 0 && !segmentIsSafeForPlanning(
                references[i - 1].position(),
                reference.position(),
                powerline,
                start,
                goal,
                start_requires_terminal_exception,
                goal_requires_terminal_exception
            )) {
            return false;
        }
    }
    return true;
}

bool CableAwareTrajectoryPlanner::segmentIsSafe(
    const point_t & a,
    const point_t & b,
    const PowerlineAdapter & powerline
) const {
    const double resolution = getDouble(configuration_, "/control/trajectory_generator/cable_aware_grid_resolution_m", 0.5);
    const int samples = std::max(2, static_cast<int>(std::ceil((b - a).norm() / std::max(resolution * 0.5, 1.0e-3))));
    for (int i = 0; i <= samples; ++i) {
        const double alpha = static_cast<double>(i) / samples;
        if (!pointIsSafe(a + alpha * (b - a), powerline)) {
            return false;
        }
    }
    return true;
}

bool CableAwareTrajectoryPlanner::segmentIsSafeForPlanning(
    const point_t & a,
    const point_t & b,
    const PowerlineAdapter & powerline,
    const point_t & start,
    const point_t & goal,
    bool start_requires_terminal_exception,
    bool goal_requires_terminal_exception
) const {
    const double resolution = getDouble(configuration_, "/control/trajectory_generator/cable_aware_grid_resolution_m", 0.5);
    const int samples = std::max(2, static_cast<int>(std::ceil((b - a).norm() / std::max(resolution * 0.5, 1.0e-3))));
    for (int i = 0; i <= samples; ++i) {
        const double alpha = static_cast<double>(i) / samples;
        if (!pointIsSafeForPlanning(
                a + alpha * (b - a),
                powerline,
                start,
                goal,
                start_requires_terminal_exception,
                goal_requires_terminal_exception
            )) {
            return false;
        }
    }
    return true;
}

bool CableAwareTrajectoryPlanner::pointIsSafe(
    const point_t & point,
    const PowerlineAdapter & powerline
) const {
    const double clearance_m = clearance();
    for (const auto & line : powerline.single_line_adapters()) {
        if (distanceToCable(point, powerline, line) < clearance_m) {
            return false;
        }
    }
    return true;
}

bool CableAwareTrajectoryPlanner::pointIsSafeForPlanning(
    const point_t & point,
    const PowerlineAdapter & powerline,
    const point_t & start,
    const point_t & goal,
    bool start_requires_terminal_exception,
    bool goal_requires_terminal_exception
) const {
    const double clearance_m = clearance();
    for (const auto & line : powerline.single_line_adapters()) {
        if (distanceToCable(point, powerline, line) >= clearance_m) {
            continue;
        }

        const bool allowed_by_start = start_requires_terminal_exception
            && pointIsAllowedByTerminalException(point, start, powerline, line);
        const bool allowed_by_goal = goal_requires_terminal_exception
            && pointIsAllowedByTerminalException(point, goal, powerline, line);
        if (!allowed_by_start && !allowed_by_goal) {
            return false;
        }
    }
    return true;
}

bool CableAwareTrajectoryPlanner::pointIsAllowedByTerminalException(
    const point_t & point,
    const point_t & terminal,
    const PowerlineAdapter & powerline,
    const SingleLineAdapter & line
) const {
    const double resolution = getDouble(configuration_, "/control/trajectory_generator/cable_aware_grid_resolution_m", 0.5);
    const double clearance_m = clearance();
    const double terminal_distance_to_line = distanceToCable(terminal, powerline, line);
    if (terminal_distance_to_line >= clearance_m + resolution) {
        return false;
    }

    const vector_t direction = cableDirection(powerline, line);
    const vector_t delta = point - terminal;
    const double along_error = std::abs(delta.dot(direction));
    const vector_t cross_delta = delta - delta.dot(direction) * direction;

    const double cross_exception_radius = clearance_m + resolution;
    const double along_exception_radius = 2.0 * clearance_m + resolution;
    return cross_delta.norm() <= cross_exception_radius
        && along_error <= along_exception_radius;
}

vector_t CableAwareTrajectoryPlanner::cableDirection(
    const PowerlineAdapter & powerline,
    const SingleLineAdapter & line
) const {
    vector_t direction = powerline.projection_plane().normal;
    direction.z() = 0.0;

    if (direction.norm() > 1.0e-6) {
        return direction.normalized();
    }

    direction = iii_drone::math::quatToMat(line.quaternion()).col(0);
    if (direction.norm() < 1.0e-6) {
        return vector_t(1.0, 0.0, 0.0);
    }
    return direction.normalized();
}

double CableAwareTrajectoryPlanner::clearance() const {
    return getDouble(configuration_, "/control/trajectory_generator/cable_aware_clearance_m", 1.0);
}

double CableAwareTrajectoryPlanner::distanceToCable(
    const point_t & point,
    const PowerlineAdapter & powerline,
    const SingleLineAdapter & line
) const {
    const vector_t direction = cableDirection(powerline, line);
    const vector_t delta = point - line.position();
    return (delta - delta.dot(direction) * direction).norm();
}

ReferenceTrajectory CableAwareTrajectoryPlanner::sampleActiveTrajectory(double elapsed_s) const {
    if (active_trajectory_.references().empty()) {
        return active_trajectory_;
    }

    const int N = getInt(configuration_, "/control/trajectory_interpolator/reference_trajectory_length_N", 10);
    const double dt = getDouble(configuration_, "/control/dt", 0.2);
    const auto & base = active_trajectory_.references();
    std::vector<Reference> sampled;
    sampled.reserve(N);

    for (int i = 0; i < N; ++i) {
        const double t = std::min(duration_s_, elapsed_s + i * dt);
        if (t >= duration_s_ - 1.0e-9) {
            sampled.push_back(base.back().CopyWithNewStamp(
                rclcpp::Clock().now() + rclcpp::Duration::from_seconds(i * dt)));
            continue;
        }
        const std::size_t index = std::min<std::size_t>(
            base.size() - 1,
            static_cast<std::size_t>(std::round(t / dt))
        );
        sampled.push_back(base[index].CopyWithNewStamp(rclcpp::Clock().now() + rclcpp::Duration::from_seconds(i * dt)));
    }

    return ReferenceTrajectory(sampled);
}
