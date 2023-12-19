#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/types.hpp>
#include <iii_drone_core/utils/math.hpp>

#include <iii_drone_core/control/reference_trajectory.hpp>

#include <iii_drone_core/adapters/reference_adapter.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/msg/reference_trajectory.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {

namespace adapters {

    /**
     * @brief Class for adapting the III-Drone-Core::ReferenceTrajectory to the III-Drone-Interfaces::ReferenceTrajectory.
     */
    class ReferenceTrajectoryAdapter {
    public:
        /**
         * @brief Default constructor.
         */
        ReferenceTrajectoryAdapter();

        /**
         * @brief Constructor from III-Drone-Core::ReferenceTrajectory.
         * 
         * @param reference The III-Drone-Core::ReferenceTrajectory.
         */
        ReferenceTrajectoryAdapter(const iii_drone::control::ReferenceTrajectory &reference_trajectory);

        /**
         * @brief Constructor from III-Drone-Interfaces::ReferenceTrajectory msg.
         * 
         * @param reference The III-Drone-Interfaces::ReferenceTrajectory msg.
         */
        ReferenceTrajectoryAdapter(const iii_drone_interfaces::msg::ReferenceTrajectory &reference_trajectory);

        /**
         * @brief Constructor from III-Drone-Interfaces::ReferenceTrajectory msg shared pointer.
         * 
         * @param reference The III-Drone-Interfaces::ReferenceTrajectory msg shared pointer.
         */
        ReferenceTrajectoryAdapter(const iii_drone_interfaces::msg::ReferenceTrajectory::SharedPtr &reference_trajectory);

        /**
         * @brief Converts to msg.
         * 
         * @return The III-Drone-Interfaces::ReferenceTrajectory msg.
         */
        iii_drone_interfaces::msg::ReferenceTrajectory ToMsg() const;

        /**
         * @brief ReferenceTrajectory getter.
         * 
         * @return The III-Drone-Core::ReferenceTrajectory.
         */
        iii_drone::control::ReferenceTrajectory reference_trajectory() const;

    private:
        iii_drone::control::ReferenceTrajectory reference_trajectory_;

    };

} // namespace adapters

} // namespace iii_drone