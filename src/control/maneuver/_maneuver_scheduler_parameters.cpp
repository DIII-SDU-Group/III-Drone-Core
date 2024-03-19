/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/maneuver/maneuver_scheduler_parameters.hpp>

using namespace iii_drone::control::maneuver;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

ManeuverSchedulerParameters::ManeuverSchedulerParameters(
    double maneuver_register_update_timeout_s,
    double maneuver_start_timeout_s,
    unsigned int maneuver_queue_size,
    unsigned int maneuver_execution_period_ms,
    bool use_nans_when_hovering,
    double no_maneuver_idle_cnt_s,
    double hover_by_object_max_euc_dist,
    double hover_on_cable_default_z_velocity,
    double hover_on_cable_default_yaw_rate
) : 
    maneuver_register_update_timeout_s_(maneuver_register_update_timeout_s),
    maneuver_start_timeout_s_(maneuver_start_timeout_s),
    maneuver_queue_size_(maneuver_queue_size),
    maneuver_execution_period_ms_(maneuver_execution_period_ms),
    use_nans_when_hovering_(use_nans_when_hovering),
    no_maneuver_idle_cnt_s_(no_maneuver_idle_cnt_s),
    hover_by_object_max_euc_dist_(hover_by_object_max_euc_dist),
    hover_on_cable_default_z_velocity_(hover_on_cable_default_z_velocity),
    hover_on_cable_default_yaw_rate_(hover_on_cable_default_yaw_rate) { }

double ManeuverSchedulerParameters::maneuver_register_update_timeout_s() const {
    std::shared_lock<std::shared_mutex> lck(parameters_mutex_);

    return maneuver_register_update_timeout_s_;
}

double ManeuverSchedulerParameters::maneuver_start_timeout_s() const {
    std::shared_lock<std::shared_mutex> lck(parameters_mutex_);

    return maneuver_start_timeout_s_;
}

unsigned int ManeuverSchedulerParameters::maneuver_queue_size() const {
    std::shared_lock<std::shared_mutex> lck(parameters_mutex_);

    return maneuver_queue_size_;
}

unsigned int ManeuverSchedulerParameters::maneuver_execution_period_ms() const {
    std::shared_lock<std::shared_mutex> lck(parameters_mutex_);

    return maneuver_execution_period_ms_;
}

bool ManeuverSchedulerParameters::use_nans_when_hovering() const {
    std::shared_lock<std::shared_mutex> lck(parameters_mutex_);

    return use_nans_when_hovering_;
}

double ManeuverSchedulerParameters::no_maneuver_idle_cnt_s() const {
    std::shared_lock<std::shared_mutex> lck(parameters_mutex_);

    return no_maneuver_idle_cnt_s_;
}

double ManeuverSchedulerParameters::hover_by_object_max_euc_dist() const {
    std::shared_lock<std::shared_mutex> lck(parameters_mutex_);

    return hover_by_object_max_euc_dist_;
}

double &ManeuverSchedulerParameters::hover_by_object_max_euc_dist() {
    std::unique_lock<std::shared_mutex> lck(parameters_mutex_);

    return hover_by_object_max_euc_dist_;
}

double ManeuverSchedulerParameters::hover_on_cable_default_z_velocity() const {
    std::shared_lock<std::shared_mutex> lck(parameters_mutex_);

    return hover_on_cable_default_z_velocity_;
}

double &ManeuverSchedulerParameters::hover_on_cable_default_z_velocity() {
    std::unique_lock<std::shared_mutex> lck(parameters_mutex_);

    return hover_on_cable_default_z_velocity_;
}

double ManeuverSchedulerParameters::hover_on_cable_default_yaw_rate() const {
    std::shared_lock<std::shared_mutex> lck(parameters_mutex_);

    return hover_on_cable_default_yaw_rate_;
}

double &ManeuverSchedulerParameters::hover_on_cable_default_yaw_rate() {
    std::unique_lock<std::shared_mutex> lck(parameters_mutex_);

    return hover_on_cable_default_yaw_rate_;
}
