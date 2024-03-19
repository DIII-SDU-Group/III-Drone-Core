/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/utils/types.hpp>

/*****************************************************************************/
// Implementation
/*****************************************************************************/

using namespace iii_drone::types;

iii_drone::types::pose_t::pose_t() {

    position = point_t::Zero();
    orientation = quaternion_t::Zero();
    orientation(0) = 1.0f;

}

const plane_t iii_drone::types::createPlane(const point_t & p, const vector_t & normal) {

    plane_t plane;

    plane.p = p;
    plane.normal = normal;

    return plane;


}

const geometry_msgs::msg::Point iii_drone::types::pointMsgFromPoint(const point_t & point) {
    
    geometry_msgs::msg::Point point_msg;

    point_msg.x = point(0);
    point_msg.y = point(1);
    point_msg.z = point(2);

    return point_msg;
    
}

const point_t iii_drone::types::pointFromPointMsg(const geometry_msgs::msg::Point & point_msg) {

    point_t point;

    point(0) = point_msg.x;
    point(1) = point_msg.y;
    point(2) = point_msg.z;

    return point;

}

const geometry_msgs::msg::Vector3 iii_drone::types::vectorMsgFromVector(const vector_t & vector) {
        
    geometry_msgs::msg::Vector3 vector_msg;

    vector_msg.x = vector(0);
    vector_msg.y = vector(1);
    vector_msg.z = vector(2);

    return vector_msg;

}

const vector_t iii_drone::types::vectorFromVectorMsg(const geometry_msgs::msg::Vector3 & vector_msg) {
                
    vector_t vector;

    vector(0) = vector_msg.x;
    vector(1) = vector_msg.y;
    vector(2) = vector_msg.z;

    return vector;
    
}

const geometry_msgs::msg::Quaternion iii_drone::types::quaternionMsgFromQuaternion(const quaternion_t & quaternion) {
    
    geometry_msgs::msg::Quaternion quaternion_msg;

    quaternion_msg.w = quaternion(0);
    quaternion_msg.x = quaternion(1);
    quaternion_msg.y = quaternion(2);
    quaternion_msg.z = quaternion(3);

    return quaternion_msg;

}

const quaternion_t iii_drone::types::quaternionFromQuaternionMsg(const geometry_msgs::msg::Quaternion & quaternion_msg) {
    
    quaternion_t quaternion;

    quaternion(0) = quaternion_msg.w;
    quaternion(1) = quaternion_msg.x;
    quaternion(2) = quaternion_msg.y;
    quaternion(3) = quaternion_msg.z;

    return quaternion;

}

#include <iostream>

const geometry_msgs::msg::Pose iii_drone::types::poseMsgFromPose(const point_t & position, const quaternion_t & quaternion) {
    
    geometry_msgs::msg::Pose pose_msg;

    pose_msg.position = pointMsgFromPoint(position);
    pose_msg.orientation = quaternionMsgFromQuaternion(quaternion);

    return pose_msg;

}

const geometry_msgs::msg::Pose iii_drone::types::poseMsgFromPose(const pose_t & pose) {
    
    geometry_msgs::msg::Pose pose_msg;

    pose_msg.position = pointMsgFromPoint(pose.position);
    pose_msg.orientation = quaternionMsgFromQuaternion(pose.orientation);

    return pose_msg;

}

const pose_t iii_drone::types::poseFromPoseMsg(const geometry_msgs::msg::Pose & pose_msg) {
    
    pose_t pose;

    pose.position = pointFromPointMsg(pose_msg.position);
    pose.orientation = quaternionFromQuaternionMsg(pose_msg.orientation);

    return pose;

}

const pose_t iii_drone::types::poseFromPose(const point_t & position, const quaternion_t & orientation) {
    
    pose_t pose;

    pose.position = position;
    pose.orientation = orientation;

    return pose;

}

const pose_t iii_drone::types::poseFromTransformMsg(const geometry_msgs::msg::Transform & transform_msg) {
    
    pose_t pose;

    pose.position = vectorFromVectorMsg(transform_msg.translation);
    pose.orientation = quaternionFromQuaternionMsg(transform_msg.rotation);

    return pose;

}

const quaternion_t iii_drone::types::quaternionFromPoseMsg(const geometry_msgs::msg::Pose & pose_msg) {
    
    quaternion_t quaternion;

    quaternion = quaternionFromQuaternionMsg(pose_msg.orientation);

    return quaternion;

}

const point_t iii_drone::types::pointFromPoseMsg(const geometry_msgs::msg::Pose & pose_msg) {
        
    point_t point;

    point = pointFromPointMsg(pose_msg.position);

    return point;

}

const quaternion_t iii_drone::types::quaternionFromTransformMsg(const geometry_msgs::msg::Transform & transform_msg) {
        
    quaternion_t quaternion;

    quaternion = quaternionFromQuaternionMsg(transform_msg.rotation);

    return quaternion;

}

const vector_t iii_drone::types::vectorFromTransformMsg(const geometry_msgs::msg::Transform & transform_msg) {
            
    vector_t vector;

    vector = vectorFromVectorMsg(transform_msg.translation);

    return vector;

}

const geometry_msgs::msg::Transform iii_drone::types::transformMsgFromTransform(const vector_t & vector, const quaternion_t & quaternion) {
                    
    geometry_msgs::msg::Transform transform_msg;

    transform_msg.translation = vectorMsgFromVector(vector);
    transform_msg.rotation = quaternionMsgFromQuaternion(quaternion);

    return transform_msg;
}

const geometry_msgs::msg::Transform iii_drone::types::transformMsgFromTransformMatrix(const transform_matrix_t & transform_matrix) {

    geometry_msgs::msg::Transform transform_msg;

    transform_msg.translation = vectorMsgFromVector(transform_matrix.block<3, 1>(0, 3));
    transform_msg.rotation = quaternionMsgFromQuaternion(quaternionFromTransformMatrix(transform_matrix));

    return transform_msg;

}

const quaternion_t iii_drone::types::quaternionFromTransformMatrix(const transform_matrix_t & transform_matrix) {

    return quaternionFromTransformMsg(transformMsgFromTransformMatrix(transform_matrix));

}

const pose_t iii_drone::types::poseFromTransformMatrix(const transform_matrix_t & transform_matrix) {

    pose_t pose;

    pose.position = transform_matrix.block<3, 1>(0, 3);
    pose.orientation = quaternionFromTransformMatrix(transform_matrix);

    return pose;

}