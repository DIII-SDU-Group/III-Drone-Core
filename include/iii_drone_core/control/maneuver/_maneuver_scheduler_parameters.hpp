#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/types.hpp>

/*****************************************************************************/
// Std:

#include <mutex>
#include <shared_mutex>
#include <memory>
#include <string>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {

namespace control {

namespace maneuver {

    /**
     * @brief Class for handling parameters for the maneuver scheduler object.
     */
    class ManeuverSchedulerParameters {
    public:
        /**
         * @brief Constructor
         * 
         * @param maneuver_register_update_timeout_s The timeout for updating a maneuver after registering it.
         * @param maneuver_start_timeout_s The timeout for the server to start executing a maneuver after Start() has been called.
         * @param maneuver_queue_size The size of the maneuver queue.
         * @param maneuver_execution_period_ms The period for executing maneuvers.
         * @param use_nans_when_hovering Whether to use NaNs for velocity and acceleration when hovering, otherwise uses zeros.
         * @param no_maneuver_idle_cnt_s The number of seconds without a maneuver before the scheduler goes from hovering to idle.
         * @param hover_by_object_max_euc_dist The maximum euclidean distance to the object before the fail callback is called.
         * @param hover_on_cable_default_z_velocity The default z velocity for hovering on a cable.
         * @param hover_on_cable_default_yaw_rate The default yaw rate for hovering on a cable.
         */
        ManeuverSchedulerParameters(
            double maneuver_register_update_timeout_s,
            double maneuver_start_timeout_s,
            unsigned int maneuver_queue_size,
            unsigned int maneuver_execution_period_ms,
            bool use_nans_when_hovering,
            double no_maneuver_idle_cnt_s,
            double hover_by_object_max_euc_dist,
            double hover_on_cable_default_z_velocity,
            double hover_on_cable_default_yaw_rate
        );

        /**
         * @brief Get the timeout duration for updating a maneuver after registering.
         * 
         * @return double The timeout duration.
         */
        double maneuver_register_update_timeout_s() const;

        /**
         * @brief Get the timeout duration for the server to start executing a maneuver after Start() has been called.
         * 
         * @return double The timeout duration.
         */
        double maneuver_start_timeout_s() const;

        /**
         * @brief Get the size of the maneuver queue.
         * 
         * @return unsigned int The size of the maneuver queue.
         */
        unsigned int maneuver_queue_size() const;

        /**
         * @brief Get the period for executing maneuvers.
         * 
         * @return unsigned int The period for executing maneuvers.
         */
        unsigned int maneuver_execution_period_ms() const;

        /**
         * @brief Get whether to use NaNs for velocity and acceleration when hovering, otherwise uses zeros.
         * 
         * @return bool Whether to use NaNs for velocity and acceleration when hovering, otherwise uses zeros.
         */
        bool use_nans_when_hovering() const;

        /**
         * @brief Get the number of seconds without a maneuver before the scheduler goes from hovering to idle.
         * 
         * @return unsigned int The number of seconds without a maneuver before the scheduler goes from hovering to idle.
         */
        double no_maneuver_idle_cnt_s() const;

        /**
         * @brief Get the maximum euclidean distance to the object before the fail callback is called.
         * 
         * @return double The maximum euclidean distance to the object before the fail callback is called.
         */
        double hover_by_object_max_euc_dist() const;

        /**
         * @brief Set the maximum euclidean distance to the object before the fail callback is called.
         * 
         * @param hover_by_object_max_euc_dist The maximum euclidean distance to the object before the fail callback is called.
         */
        double &hover_by_object_max_euc_dist();

        /**
         * @brief Get the default z velocity for hovering on a cable.
         * 
         * @return double The default z velocity for hovering on a cable.
         */
        double hover_on_cable_default_z_velocity() const;

        /**
         * @brief Set the default z velocity for hovering on a cable.
         * 
         * @param hover_on_cable_default_z_velocity The default z velocity for hovering on a cable.
         */
        double &hover_on_cable_default_z_velocity();

        /**
         * @brief Get the default yaw rate for hovering on a cable.
         * 
         * @return double The default yaw rate for hovering on a cable.
         */
        double hover_on_cable_default_yaw_rate() const;

        /**
         * @brief Set the default yaw rate for hovering on a cable.
         * 
         * @param hover_on_cable_default_yaw_rate The default yaw rate for hovering on a cable.
         */
        double &hover_on_cable_default_yaw_rate();

        /**
         * @brief Shared ptr type.
         */
        typedef std::shared_ptr<ManeuverSchedulerParameters> SharedPtr;

    private:
        /**
         * @brief Shared mutex protecting the parameter access
         */
        mutable std::shared_mutex parameters_mutex_;

        /**
         * @brief The timeout for updating the maneuver after registering.
         */
        double maneuver_register_update_timeout_s_;

        /**
         * @brief The timeout for the server to start executing a maneuver after Start() has been called.
         */
        double maneuver_start_timeout_s_;

        /**
         * @brief The size of the maneuver queue.
         */
        unsigned int maneuver_queue_size_;

        /**
         * @brief The period for executing maneuvers.
         */
        unsigned int maneuver_execution_period_ms_;

        /**
         * @brief Whether to use NaNs for velocity and acceleration when hovering, otherwise uses zeros.
         */
        bool use_nans_when_hovering_;

        /**
         * @brief The number of seconds without a maneuver before the scheduler goes from hovering to idle.
         */
        double no_maneuver_idle_cnt_s_;

        /**
         * @brief The maximum euclidean distance to the object before the fail callback is called.
         */
        double hover_by_object_max_euc_dist_;

        /**
         * @brief The default z velocity for hovering on a cable.
         */
        double hover_on_cable_default_z_velocity_;

        /**
         * @brief The default yaw rate for hovering on a cable.
         */
        double hover_on_cable_default_yaw_rate_;

    };

} // namespace maneuver
} // namespace control
} // namespace iii_drone