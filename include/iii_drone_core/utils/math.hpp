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

    const iii_drone::types::vector_t rotateVector(
        const iii_drone::types::rotation_matrix_t & R, 
        const iii_drone::types::vector_t & v
    );

    const iii_drone::types::point_t projectPointOnPlane(
        const iii_drone::types::point_t & point, 
        const iii_drone::types::plane_t & plane
    );

    const iii_drone::types::euler_angles_t quatToEul(const iii_drone::types::quaternion_t & quat);

    const iii_drone::types::quaternion_t quatInv(const iii_drone::types::quaternion_t & quat);

    const iii_drone::types::quaternion_t quatMultiply(
        const iii_drone::types::quaternion_t & quat1,
        const iii_drone::types::quaternion_t & quat2
    );

    const iii_drone::types::rotation_matrix_t eulToMat(const iii_drone::types::euler_angles_t & eul);

    const iii_drone::types::rotation_matrix_t quatToMat(const iii_drone::types::quaternion_t & quat);

    const iii_drone::types::quaternion_t matToQuat(const iii_drone::types::rotation_matrix_t & R);

    const iii_drone::types::quaternion_t eulToQuat(const iii_drone::types::euler_angles_t & eul);

    const iii_drone::types::transform_matrix_t createTransformMatrix(
        const iii_drone::types::vector_t & vector, 
        const iii_drone::types::quaternion_t & quaternion
    );

} // namespace math
} // namespace iii_drone