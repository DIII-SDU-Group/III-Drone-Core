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

    /**
     * @brief Class for handling parameters for the combined drone awareness handler object.
     */
    class CombinedDroneAwarenessHandlerParameters {
    public:
        /**
         * @brief Constructor
         * 
         * @param ground_estimate_window_size The window size for the ground estimate history.
         * @param ground_estimate_update_period_ms The period for updating the ground estimate.
         * @param landed_altitude_threshold The altitude threshold for the drone to be considered landed.
         * @param on_cable_max_euc_distance The on_cable_max_euc_distance value.
         * @param fail_on_unable_to_locate Whether to fail if unable to locate the drone.
         * @param cable_gripper_frame_id The cable gripper frame id.
         * @param drone_frame_id The drone frame id.
         * @param world_frame_id The world frame id.
         * @param ground_frame_id The ground frame id.
         */
        CombinedDroneAwarenessHandlerParameters(
            int ground_estimate_window_size,
            int ground_estimate_update_period_ms,
            double landed_altitude_threshold,
            double on_cable_max_euc_distance,
            bool fail_on_unable_to_locate,
            const std::string & cable_gripper_frame_id,
            const std::string & drone_frame_id,
            const std::string & world_frame_id,
            const std::string & ground_frame_id
        );

        /**
         * @brief Get the window size for the ground estimate history.
         * 
         * @return The window size for the ground estimate history.
         */
        int ground_estimate_window_size() const;

        /**
         * @brief Get the period for updating the ground estimate.
         * 
         * @return The period for updating the ground estimate.
         */
        int ground_estimate_update_period_ms() const;

        /**
         * @brief Get the altitude threshold for the drone to be considered landed.
         * 
         * @return The altitude threshold for the drone to be considered landed.
         */
        double landed_altitude_threshold() const;

        /**
         * @brief Set the altitude threshold for the drone to be considered landed.
         * 
         * @return The altitude threshold for the drone to be considered landed.
         */
        double & landed_altitude_threshold();

        /**
         * @brief Get the on_cable_max_euc_distance value.
         * 
         * @return The on_cable_max_euc_distance value.
         */
        double on_cable_max_euc_distance() const;

        /**
         * @brief Set the on_cable_max_euc_distance value.
         * 
         * @return The on_cable_max_euc_distance value.
         */
        double & on_cable_max_euc_distance();

        /**
         * @brief Get the fail_on_unable_to_locate value.
         * 
         * @return The fail_on_unable_to_locate value.
         */
        bool fail_on_unable_to_locate() const;

        /**
         * @brief Get the cable gripper frame id.
         * 
         * @return The cable gripper frame id.
         */
        std::string cable_gripper_frame_id() const;

        /**
         * @brief Get the drone frame id.
         * 
         * @return The drone frame id.
         */
        std::string drone_frame_id() const;

        /**
         * @brief Get the world frame id.
         * 
         * @return The world frame id.
         */
        std::string world_frame_id() const;

        /**
         * @brief Get the ground frame id.
         * 
         * @return The ground frame id.
         */
        std::string ground_frame_id() const;

        /**
         * @brief Shared ptr type.
         */
        typedef std::shared_ptr<CombinedDroneAwarenessHandlerParameters> SharedPtr;

    private:
        /**
         * @brief Shared mutex protecting the parameter access
         */
        mutable std::shared_mutex parameters_mutex_;

        /**
         * @brief The window size for the ground estimate history.
         */
        int ground_estimate_window_size_;

        /**
         * @brief The period for updating the ground estimate.
         */
        int ground_estimate_update_period_ms_;

        /**
         * @brief The altitude threshold for the drone to be considered landed.
         */
        double landed_altitude_threshold_;

        /**
         * @brief The on_cable_max_euc_distance value.
         */
        double on_cable_max_euc_distance_;

        /**
         * @brief The fail_on_unable_to_locate value.
         */
        bool fail_on_unable_to_locate_;

        /**
         * @brief The cable gripper frame id.
         */
        std::string cable_gripper_frame_id_;

        /**
         * @brief The drone frame id.
         */
        std::string drone_frame_id_;

        /**
         * @brief The world frame id.
         */
        std::string world_frame_id_;

        /**
         * @brief The ground frame id.
         */
        std::string ground_frame_id_;

    };

} // namespace control
} // namespace iii_drone