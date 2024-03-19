/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/combined_drone_awareness_handler_parameters.hpp>

using namespace iii_drone::control;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

CombinedDroneAwarenessHandlerParameters::CombinedDroneAwarenessHandlerParameters(
    int ground_estimate_window_size,
    int ground_estimate_update_period_ms,
    double landed_altitude_threshold,
    double on_cable_max_euc_distance,
    bool fail_on_unable_to_locate,
    const std::string & cable_gripper_frame_id,
    const std::string & drone_frame_id,
    const std::string & world_frame_id,
    const std::string & ground_frame_id
) :
    ground_estimate_window_size_(ground_estimate_window_size),
    ground_estimate_update_period_ms_(ground_estimate_update_period_ms),
    landed_altitude_threshold_(landed_altitude_threshold),
    on_cable_max_euc_distance_(on_cable_max_euc_distance),
    fail_on_unable_to_locate_(fail_on_unable_to_locate),
    cable_gripper_frame_id_(cable_gripper_frame_id),
    drone_frame_id_(drone_frame_id),
    world_frame_id_(world_frame_id),
    ground_frame_id_(ground_frame_id) {}

int CombinedDroneAwarenessHandlerParameters::ground_estimate_window_size() const {
    std::shared_lock lock(parameters_mutex_);

    return ground_estimate_window_size_;
}

int CombinedDroneAwarenessHandlerParameters::ground_estimate_update_period_ms() const {
    std::shared_lock lock(parameters_mutex_);

    return ground_estimate_update_period_ms_;
}

double CombinedDroneAwarenessHandlerParameters::landed_altitude_threshold() const {
    std::shared_lock lock(parameters_mutex_);

    return landed_altitude_threshold_;
}

double & CombinedDroneAwarenessHandlerParameters::landed_altitude_threshold() {
    std::unique_lock lock(parameters_mutex_);

    return landed_altitude_threshold_;
}

double CombinedDroneAwarenessHandlerParameters::on_cable_max_euc_distance() const {
    std::shared_lock lock(parameters_mutex_);

    return on_cable_max_euc_distance_;
}

double & CombinedDroneAwarenessHandlerParameters::on_cable_max_euc_distance() {
    std::unique_lock lock(parameters_mutex_);

    return on_cable_max_euc_distance_;
}

bool CombinedDroneAwarenessHandlerParameters::fail_on_unable_to_locate() const {
    std::shared_lock lock(parameters_mutex_);

    return fail_on_unable_to_locate_;
}

std::string CombinedDroneAwarenessHandlerParameters::cable_gripper_frame_id() const {
    std::shared_lock lock(parameters_mutex_);

    return cable_gripper_frame_id_;
}

std::string CombinedDroneAwarenessHandlerParameters::drone_frame_id() const {
    std::shared_lock lock(parameters_mutex_);

    return drone_frame_id_;
}

std::string CombinedDroneAwarenessHandlerParameters::world_frame_id() const {
    std::shared_lock lock(parameters_mutex_);

    return world_frame_id_;
}

std::string CombinedDroneAwarenessHandlerParameters::ground_frame_id() const {
    std::shared_lock lock(parameters_mutex_);

    return ground_frame_id_;
}