/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/adapters/single_line_adapter.hpp>

/*****************************************************************************/
// Implementation:
/*****************************************************************************/

using namespace iii_drone::adapters;

SingleLineAdapter::SingleLineAdapter() {

    stamp_ = rclcpp::Clock().now();
    frame_id_ = "drone";
    id_ = -1;
    position_ = iii_drone::types::point_t::Zero();
    projected_position_ = iii_drone::types::point_t::Zero();
    quaternion_ = iii_drone::types::quaternion_t::Zero();
    quaternion_(0) = 1.0;
    in_fov_ = false;

}

SingleLineAdapter::SingleLineAdapter(const iii_drone_interfaces::msg::SingleLine& msg) {

    stamp_ = msg.header.stamp;
    frame_id_ = msg.header.frame_id;
    id_ = msg.id;
    position_ = iii_drone::types::pointFromPoseMsg(msg.pose);
    projected_position_ = iii_drone::types::pointFromPointMsg(msg.projected_position);
    quaternion_ = iii_drone::types::quaternionFromPoseMsg(msg.pose);
    in_fov_ = msg.in_field_of_view;

}

SingleLineAdapter::SingleLineAdapter(
    const rclcpp::Time & stamp, 
    const std::string & frame_id, 
    const int & id, 
    const iii_drone::types::point_t & position, 
    const iii_drone::types::point_t & projected_position,
    const iii_drone::types::quaternion_t & quaternion, 
    const bool & in_fov
) {

    stamp_ = stamp;
    frame_id_ = frame_id;
    id_ = id;
    position_ = position;
    projected_position_ = projected_position;
    quaternion_ = quaternion;
    in_fov_ = in_fov;

}

SingleLineAdapter::SingleLineAdapter(
    const rclcpp::Time & stamp, 
    const std::string & frame_id, 
    const int & id, 
    const geometry_msgs::msg::Pose & msg,
    const bool & in_fov
) {

    stamp_ = stamp;
    frame_id_ = frame_id;
    id_ = id;
    position_ = iii_drone::types::pointFromPoseMsg(msg);
    projected_position_ = position_;
    quaternion_ = iii_drone::types::quaternionFromPoseMsg(msg);
    in_fov_ = in_fov;

}

SingleLineAdapter::SingleLineAdapter(
    const int & id, 
    const geometry_msgs::msg::PoseStamped & msg, 
    const bool & in_fov
) {

    stamp_ = msg.header.stamp;
    frame_id_ = msg.header.frame_id;
    id_ = id;
    position_ = iii_drone::types::pointFromPoseMsg(msg.pose);
    projected_position_ = position_;
    quaternion_ = iii_drone::types::quaternionFromPoseMsg(msg.pose);
    in_fov_ = in_fov;

}

const iii_drone_interfaces::msg::SingleLine SingleLineAdapter::ToMsg() const {

    iii_drone_interfaces::msg::SingleLine msg;

    msg.header.stamp = stamp_;
    msg.header.frame_id = frame_id_;
    msg.id = id_;
    msg.pose = iii_drone::types::poseMsgFromPose(position_, quaternion_);
    msg.projected_position = iii_drone::types::pointMsgFromPoint(projected_position_);
    msg.in_field_of_view = in_fov_;

    return msg;

}

const geometry_msgs::msg::Pose SingleLineAdapter::ToPoseMsg() const {

    geometry_msgs::msg::Pose msg;

    msg.position = iii_drone::types::pointMsgFromPoint(position_);
    msg.orientation = iii_drone::types::quaternionMsgFromQuaternion(quaternion_);

    return msg;

}

const geometry_msgs::msg::PoseStamped SingleLineAdapter::ToPoseStampedMsg() const {

    geometry_msgs::msg::PoseStamped msg;

    msg.header.stamp = stamp_;
    msg.header.frame_id = frame_id_;
    msg.pose = ToPoseMsg();

    return msg;

}

void SingleLineAdapter::Transform(
    const std::string & target_frame_id,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer
) {

    geometry_msgs::msg::PoseStamped pose_stamped_msg = ToPoseStampedMsg();
    pose_stamped_msg = tf_buffer->transform(pose_stamped_msg, target_frame_id);

    position_ = iii_drone::types::pointFromPoseMsg(pose_stamped_msg.pose);
    quaternion_ = iii_drone::types::quaternionFromPoseMsg(pose_stamped_msg.pose);

    geometry_msgs::msg::PointStamped point_stamped_msg;
    point_stamped_msg.header.stamp = stamp_;
    point_stamped_msg.header.frame_id = frame_id_;
    point_stamped_msg.point = iii_drone::types::pointMsgFromPoint(projected_position_);

    point_stamped_msg = tf_buffer->transform(point_stamped_msg, target_frame_id);

    projected_position_ = iii_drone::types::pointFromPointMsg(point_stamped_msg.point);

    frame_id_ = target_frame_id;


}

const rclcpp::Time & SingleLineAdapter::stamp() const {

    return stamp_;

}

const std::string & SingleLineAdapter::frame_id() const {

    return frame_id_;

}

const int & SingleLineAdapter::id() const {

    return id_;

}

const iii_drone::types::point_t & SingleLineAdapter::position() const {

    return position_;

}

const iii_drone::types::point_t & SingleLineAdapter::projected_position() const {

    return projected_position_;

}

const iii_drone::types::quaternion_t & SingleLineAdapter::quaternion() const {

    return quaternion_;

}

const bool & SingleLineAdapter::in_fov() const {

    return in_fov_;

}
