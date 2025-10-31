#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <vector>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/types.hpp>

#include <iii_drone_core/control/state.hpp>
#include <iii_drone_core/control/reference.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {

namespace control {

    class ReferenceTrajectory {

    public:
        /**
         * @brief Default constructor
         */
        ReferenceTrajectory();

        /**
         * @brief Constructor from vector of references
         * 
         * @param references The vector of references
         */
        ReferenceTrajectory(const std::vector<Reference>& references);

        /**
         * @brief Constructor from vector of states
         * 
         * @param states The vector of states
         */
        ReferenceTrajectory(const std::vector<State>& states);

        /**
         * @brief References getter.
         */
        const std::vector<Reference>& references() const;

    private:
        /**
         * @brief The vector of references
         */
        std::vector<Reference> references_;

    };

} // namespace iii_drone::control
} // namespace iii_drone