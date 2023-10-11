#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <eigen3/Eigen/Core>

/*****************************************************************************/
// Defines
/*****************************************************************************/

namespace iii_drone {
namespace types {

    typedef Eigen::Vector3f point_t;

    typedef Eigen::Vector3f orientation_t;

    typedef Eigen::Vector4f quat_t;

    typedef Eigen::Vector3f vector_t;

    typedef struct {

        point_t p;
        vector_t v;

    } line_t;

    typedef struct {

        point_t p;
        vector_t normal;

    } plane_t;

    typedef Eigen::Matrix3f rotation_matrix_t;

    typedef Eigen::Matrix4f transform_t;


} // namespace types
} // namespace iii_drone