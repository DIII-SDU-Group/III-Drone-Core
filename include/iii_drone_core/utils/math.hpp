#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <eigen3/Eigen/Core>

#include <iii_drone_core/utils/types.hpp>

/*****************************************************************************/
// Function declarations
/*****************************************************************************/

namespace iii_drone {
namespace math {

    iii_drone::types::rotation_matrix_t eulToR(iii_drone::types::orientation_t eul);

    iii_drone::types::vector_t rotateVector(
        iii_drone::types::rotation_matrix_t R, 
        iii_drone::types::vector_t v
    );

    iii_drone::types::point_t projectPointOnPlane(
        iii_drone::types::point_t point, 
        iii_drone::types::plane_t plane
    );

    iii_drone::types::orientation_t quatToEul(iii_drone::types::quat_t quat);

    iii_drone::types::quat_t quatInv(iii_drone::types::quat_t quat);

    iii_drone::types::quat_t quatMultiply(
        iii_drone::types::quat_t quat1,
        iii_drone::types::quat_t quat2
    );

    iii_drone::types::rotation_matrix_t quatToMat(iii_drone::types::quat_t quat);

    iii_drone::types::quat_t matToQuat(iii_drone::types::rotation_matrix_t R);

    iii_drone::types::quat_t eulToQuat(iii_drone::types::orientation_t eul);

    iii_drone::types::transform_t getTransformMatrix(
        iii_drone::types::vector_t vec, 
        iii_drone::types::quat_t quat
    );

} // namespace math
} // namespace iii_drone