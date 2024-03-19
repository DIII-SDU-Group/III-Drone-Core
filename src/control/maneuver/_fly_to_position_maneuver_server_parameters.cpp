/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/maneuver/fly_to_position_maneuver_server_parameters.hpp>

using namespace iii_drone::control::maneuver;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

FlyToPositionManeuverServerParameters::FlyToPositionManeuverServerParameters(
    double reached_position_euclidean_distance_threshold,
    double minimum_target_altitude,
    bool generate_trajectories_asynchronously_with_delay,
    unsigned int generate_trajectories_poll_period_ms,
    const std::string & world_frame_id
) : reached_position_euclidean_distance_threshold_(reached_position_euclidean_distance_threshold),
    minimum_target_altitude_(minimum_target_altitude),
    generate_trajectories_asynchronously_with_delay_(generate_trajectories_asynchronously_with_delay),
    generate_trajectories_poll_period_ms_(generate_trajectories_poll_period_ms),
    world_frame_id_(world_frame_id) { }

double FlyToPositionManeuverServerParameters::reached_position_euclidean_distance_threshold() const {
    std::shared_lock lock(parameters_mutex_);

    return reached_position_euclidean_distance_threshold_;
}

double & FlyToPositionManeuverServerParameters::reached_position_euclidean_distance_threshold() {
    std::unique_lock lock(parameters_mutex_);

    return reached_position_euclidean_distance_threshold_;
}

double FlyToPositionManeuverServerParameters::minimum_target_altitude() const {
    std::shared_lock lock(parameters_mutex_);

    return minimum_target_altitude_;
}

double & FlyToPositionManeuverServerParameters::minimum_target_altitude() {
    std::unique_lock lock(parameters_mutex_);

    return minimum_target_altitude_;
}

bool FlyToPositionManeuverServerParameters::generate_trajectories_asynchronously_with_delay() const {
    std::shared_lock lock(parameters_mutex_);

    return generate_trajectories_asynchronously_with_delay_;
}

unsigned int FlyToPositionManeuverServerParameters::generate_trajectories_poll_period_ms() const {
    std::shared_lock lock(parameters_mutex_);

    return generate_trajectories_poll_period_ms_;
}

std::string FlyToPositionManeuverServerParameters::world_frame_id() const {
    std::shared_lock lock(parameters_mutex_);

    return world_frame_id_;
}