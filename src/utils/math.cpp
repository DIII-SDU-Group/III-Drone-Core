/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "iii_drone_core/utils/math.hpp"
#include "iii_drone_core/utils/types.hpp"

using namespace iii_drone::types;
using namespace iii_drone::math;

/*****************************************************************************/
// Function implementations
/*****************************************************************************/

const rotation_matrix_t iii_drone::math::eulToMat(const euler_angles_t & eul) {

    float cos_yaw = cos(eul[2]);
    float cos_pitch = cos(eul[1]);
    float cos_roll = cos(eul[0]);
    float sin_yaw = sin(eul[2]);
    float sin_pitch = sin(eul[1]);
    float sin_roll = sin(eul[0]);

    rotation_matrix_t mat;

    mat(0,0) = cos_pitch*cos_yaw;
    mat(0,1) = sin_roll*sin_pitch*cos_yaw-cos_roll*sin_yaw;
    mat(0,2) = cos_roll*sin_pitch*cos_yaw+sin_roll*sin_yaw;
    mat(1,0) = cos_pitch*sin_yaw;
    mat(1,1) = sin_roll*sin_pitch*sin_yaw+cos_roll*cos_yaw;
    mat(1,2) = cos_roll*sin_pitch*sin_yaw-sin_roll*cos_pitch; // wrong? cos_roll*sin_pitch*sin_yaw-sin_roll*cos_yaw
    mat(2,0) = -sin_pitch;
    mat(2,1) = sin_roll*cos_pitch;
    mat(2,2) = cos_roll*cos_pitch;

    return mat;

}

const vector_t iii_drone::math::rotateVector(
    const rotation_matrix_t & R, 
    const vector_t & v
) {

    vector_t ret_vec = R*v;

    return ret_vec;
}

const point_t iii_drone::math::projectPointOnPlane(
    const point_t & point, 
    const plane_t & plane
) {

    float t = (plane.normal.dot(plane.p) - plane.normal.dot(point)) / plane.normal.dot(plane.normal);

    point_t proj_point = point + (point_t)(t*plane.normal);

    return proj_point;

}

const euler_angles_t iii_drone::math::quatToEul(const quaternion_t & quat) {

    euler_angles_t eul(
        atan2(2*(quat[0]*quat[1] + quat[2]*quat[3]), 1-2*(quat[1]*quat[1] + quat[2]*quat[2])),
        asin(2*(quat[0]*quat[2] - quat[3]*quat[1])),
        atan2(2*(quat[0]*quat[3] + quat[1]*quat[2]), 1-2*(quat[2]*quat[2]+quat[3]*quat[3]))
    );

    return eul;

}

const quaternion_t iii_drone::math::quatInv(const quaternion_t & quat) {

    quaternion_t ret_quat(quat[0], -quat[1], -quat[2], -quat[3]);

    return ret_quat;

}

const quaternion_t iii_drone::math::quatMultiply(
    const quaternion_t & quat1, 
    const quaternion_t & quat2
) {

    quaternion_t ret_quat(
        quat1[0]*quat2[0] - quat1[1]*quat2[1] - quat1[2]*quat2[2] - quat1[3]*quat2[3],
        quat1[0]*quat2[1] + quat1[1]*quat2[0] + quat1[2]*quat2[3] - quat1[3]*quat2[2],
        quat1[0]*quat2[2] - quat1[1]*quat2[3] + quat1[2]*quat2[0] + quat1[3]*quat2[1],
        quat1[0]*quat2[3] + quat1[1]*quat2[2] - quat1[2]*quat2[1] + quat1[3]*quat2[0]
    );

    return ret_quat;

}

const rotation_matrix_t iii_drone::math::quatToMat(const quaternion_t & quat) {

    euler_angles_t eul = quatToEul(quat);
    rotation_matrix_t mat = eulToMat(eul);

    return mat;

}

const quaternion_t iii_drone::math::matToQuat(const rotation_matrix_t & R) {

    float tr = R(0,0) + R(1,1) + R(2,2);

    float qw, qx, qy, qz;

    if (tr > 0) { 

        float S = sqrt(tr+1.0) * 2; // S=4*qw 
        qw = 0.25 * S;
        qx = (R(2,1) - R(1,2)) / S;
        qy = (R(0,2) - R(2,0)) / S; 
        qz = (R(1,0) - R(0,1)) / S; 

    } else if ((R(0,0) > R(1,1)) && (R(0,0) > R(2,2))) { 

        float S = sqrt(1.0 + R(0,0) - R(1,1) - R(2,2)) * 2; // S=4*qx 
        qw = (R(2,1) - R(1,2)) / S;
        qx = 0.25 * S;
        qy = (R(0,1) + R(1,0)) / S; 
        qz = (R(0,2) + R(2,0)) / S; 

    } else if (R(1,1) > R(2,2)) { 

        float S = sqrt(1.0 + R(1,1) - R(0,0) - R(2,2)) * 2; // S=4*qy
        qw = (R(0,2) - R(2,0)) / S;
        qx = (R(0,1) + R(1,0)) / S; 
        qy = 0.25 * S;
        qz = (R(1,2) + R(2,1)) / S; 

    } else { 

        float S = sqrt(1.0 + R(2,2) - R(0,0) - R(1,1)) * 2; // S=4*qz
        qw = (R(1,0) - R(0,1)) / S;
        qx = (R(0,2) + R(2,0)) / S;
        qy = (R(1,2) + R(2,1)) / S;
        qz = 0.25 * S;

    }

    quaternion_t quat(qw, qx, qy, qz);

    return quat;

}

const quaternion_t iii_drone::math::eulToQuat(const euler_angles_t & eul) {

    // Abbreviations for the various angular functions
    float cy = cos(eul(2) * 0.5);
    float sy = sin(eul(2) * 0.5);
    float cp = cos(eul(1) * 0.5);
    float sp = sin(eul(1) * 0.5);
    float cr = cos(eul(0) * 0.5);
    float sr = sin(eul(0) * 0.5);

    quaternion_t q;
    q(0) = cr * cp * cy + sr * sp * sy;
    q(1) = sr * cp * cy - cr * sp * sy;
    q(2) = cr * sp * cy + sr * cp * sy;
    q(3) = cr * cp * sy - sr * sp * cy;

    return q;

}

const transform_matrix_t iii_drone::math::createTransformMatrix(
    const vector_t & vec, 
    const quaternion_t & quat
) {

    transform_matrix_t T;

    rotation_matrix_t R = iii_drone::math::quatToMat(quat);

    for ( int i = 0; i < 3; i++) {

        for (int j = 0; j < 3; j++) {

            T(i,j) = R(i,j);
        }

        T(i,3) = vec(i);

    }

    T(3,0) = 0;
    T(3,1) = 0;
    T(3,2) = 0;
    T(3,3) = 1;

    return T;

}

const transform_matrix_t iii_drone::types::transformMatrixFromTransformMsg(const geometry_msgs::msg::Transform & transform_msg) {

    transform_matrix_t transform_matrix;

    transform_matrix.block<3, 1>(0, 3) = vectorFromVectorMsg(transform_msg.translation);
    transform_matrix.block<3, 3>(0, 0) = quatToMat(quaternionFromTransformMsg(transform_msg));

    transform_matrix(3, 0) = 0.0f;
    transform_matrix(3, 1) = 0.0f;
    transform_matrix(3, 2) = 0.0f;
    transform_matrix(3, 3) = 1.0f;

    return transform_matrix;

}

const quaternion_t iii_drone::types::quaternionFromTransformMatrix(const transform_matrix_t & transform_matrix) {

    return matToQuat(transform_matrix.block<3, 3>(0, 0));

}

const pose_t iii_drone::types::poseFromTransformMatrix(const transform_matrix_t & transform_matrix) {

    pose_t pose;

    pose.position = transform_matrix.block<3, 1>(0, 3);
    pose.orientation = quaternionFromTransformMatrix(transform_matrix);

    return pose;

}