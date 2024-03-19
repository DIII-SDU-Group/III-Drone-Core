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
     * @brief Class for handling parameters for the FlyToObjectManeuverServer object.
     */
    class FlyToObjectManeuverServerParameters {
    public:
        /**
         * @brief Constructor
         * 
         * @param reached_position_euclidean_distance_threshold The threshold for the euclidean distance to the target position for the maneuver to be considered reached.
         * @param minimum_target_altitude The minimum altitude for the target position.
         * @param generate_trajectories_asynchronously_with_delay Whether to generate trajectories asynchronously with a one time step delay.
         * @param generate_trajectories_poll_period_ms The period for polling the trajectory generator for new trajectories.
         * @param world_frame_id The world frame id.
         */
        FlyToObjectManeuverServerParameters(
            double reached_position_euclidean_distance_threshold,
            double minimum_target_altitude,
            bool generate_trajectories_asynchronously_with_delay,
            unsigned int generate_trajectories_poll_period_ms,
            const std::string & world_frame_id
        );

        /**
         * @brief Get the reached position euclidean distance threshold.
         * 
         * @return The reached position euclidean distance threshold.
         */
        double reached_position_euclidean_distance_threshold() const;

        /**
         * @brief Set the reached position euclidean distance threshold.
         * 
         * @return The reached position euclidean distance threshold.
         */
        double & reached_position_euclidean_distance_threshold();

        /**
         * @brief Get the minimum target altitude.
         * 
         * @return The minimum target altitude.
         */
        double minimum_target_altitude() const;

        /**
         * @brief Set the minimum target altitude.
         * 
         * @return The minimum target altitude.
         */
        double & minimum_target_altitude();

        /**
         * @brief Get the flag for whether to generate trajectories asynchronously with a one time step delay.
         * 
         * @return The flag for whether to generate trajectories asynchronously with a one time step delay.
         */
        bool generate_trajectories_asynchronously_with_delay() const;

        /**
         * @brief Get the period for polling the trajectory generator for new trajectories.
         * 
         * @return The period for polling the trajectory generator for new trajectories.
         */
        unsigned int generate_trajectories_poll_period_ms() const;

        /**
         * @brief Get the world frame id.
         * 
         * @return The world frame id.
         */
        std::string world_frame_id() const;

        /**
         * @brief Shared ptr type.
         */
        typedef std::shared_ptr<FlyToObjectManeuverServerParameters> SharedPtr;

    private:
        /**
         * @brief Shared mutex protecting the parameter access
         */
        mutable std::shared_mutex parameters_mutex_;

        /**
         * @brief The reached position euclidean distance threshold.
         */
        double reached_position_euclidean_distance_threshold_;

        /**
         * @brief The minimum target altitude.
         */
        double minimum_target_altitude_;

        /**
         * @brief Whether to generate trajectories asynchronously with a one time step delay.
         */
        bool generate_trajectories_asynchronously_with_delay_;

        /**
         * @brief The period for polling the trajectory generator for new trajectories.
         */
        unsigned int generate_trajectories_poll_period_ms_;

        /**
         * @brief The world frame id.
         */
        std::string world_frame_id_;

    };

} // namespace maneuver
} // namespace control
} // namespace iii_drone