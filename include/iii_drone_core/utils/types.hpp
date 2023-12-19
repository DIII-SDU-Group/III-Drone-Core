#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <eigen3/Eigen/Core>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/vector3.hpp>

/*****************************************************************************/
// Defines
/*****************************************************************************/

namespace iii_drone {
namespace types {

    typedef Eigen::Vector3f point_t;

    typedef Eigen::Vector4f point4_t;

    typedef Eigen::Vector3f euler_angles_t;

    typedef Eigen::Vector4f quaternion_t;

    typedef Eigen::Vector3f vector_t;

	// /**
	//  * @brief State in 3D space with first derivative
	// */
	// typedef Eigen::Matrix<float, 6, 1> state3_t;

	// /**
	//  * @brief State in 4D space with yaw and second derivative
	// */
	// typedef Eigen::Matrix<float, 12, 1> state4_t;


    typedef struct {

        point_t p;
        vector_t v;

    } line_t;

    typedef struct {

        point_t p;
        vector_t normal;

    } plane_t;

    typedef struct pose_t {

        point_t position;
        quaternion_t orientation;

        pose_t();

    } pose_t;

    typedef Eigen::Matrix3f rotation_matrix_t;

    typedef Eigen::Matrix4f transform_matrix_t;

    const plane_t createPlane(const point_t & p, const vector_t & normal);

    const geometry_msgs::msg::Point pointMsgFromPoint(const point_t & point);

    const point_t pointFromPointMsg(const geometry_msgs::msg::Point & point_msg);

    const geometry_msgs::msg::Vector3 vectorMsgFromVector(const vector_t & vector);

    const vector_t vectorFromVectorMsg(const geometry_msgs::msg::Vector3 & vector_msg);

    const geometry_msgs::msg::Quaternion quaternionMsgFromQuaternion(const quaternion_t & quaternion);

    const quaternion_t quaternionFromQuaternionMsg(const geometry_msgs::msg::Quaternion & quaternion_msg);

    const geometry_msgs::msg::Pose poseMsgFromPose(const point_t & position, const quaternion_t & quaternion);

    const geometry_msgs::msg::Pose poseMsgFromPose(const pose_t & pose);

    const pose_t poseFromPoseMsg(const geometry_msgs::msg::Pose & pose_msg);

    const pose_t poseFromPose(const point_t & position, const quaternion_t & orientation);

    const pose_t poseFromTransformMsg(const geometry_msgs::msg::Transform & transform_msg);

    const quaternion_t quaternionFromPoseMsg(const geometry_msgs::msg::Pose & pose_msg);

    const point_t pointFromPoseMsg(const geometry_msgs::msg::Pose & pose_msg);

    const quaternion_t quaternionFromTransformMsg(const geometry_msgs::msg::Transform & transform_msg);

    const vector_t vectorFromTransformMsg(const geometry_msgs::msg::Transform & transform_msg);

    const geometry_msgs::msg::Transform transformMsgFromTransform(const vector_t & vector, const quaternion_t & quaternion);


} // namespace types
} // namespace iii_drone