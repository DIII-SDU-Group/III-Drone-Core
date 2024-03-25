/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/adapters/target_adapter.hpp>

using namespace iii_drone::adapters;
using namespace iii_drone::types;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

TargetAdapter::TargetAdapter() {
    
    target_type_ = TARGET_TYPE_NONE;
    target_id_ = -1;
    reference_frame_id_ = "";
    target_transform_ = iii_drone::types::transform_matrix_t::Identity();

}

TargetAdapter::TargetAdapter(const TargetAdapter & other) {
    
    target_type_ = other.target_type_;
    target_id_ = other.target_id_;
    reference_frame_id_ = other.reference_frame_id_;
    target_transform_ = other.target_transform_;

}

TargetAdapter::TargetAdapter(const iii_drone_interfaces::msg::Target & target_msg) {
    
    target_type_ = (target_type_t)target_msg.target_type;
    target_id_ = target_msg.target_id;
    reference_frame_id_ = target_msg.reference_frame_id;
    target_transform_ = iii_drone::types::transformMatrixFromTransformMsg(target_msg.target_transform);

}

TargetAdapter::TargetAdapter(
    target_type_t target_type,
    int target_id,
    const std::string & reference_frame_id,
    const iii_drone::types::transform_matrix_t & target_transform
) {
    
    target_type_ = target_type;
    target_id_ = target_id;
    reference_frame_id_ = reference_frame_id;
    target_transform_ = target_transform;

}

TargetAdapter::~TargetAdapter() {

}

iii_drone_interfaces::msg::Target TargetAdapter::ToMsg() const {
    
    iii_drone_interfaces::msg::Target target_msg;

    target_msg.target_type = target_type_;
    target_msg.target_id = target_id_;
    target_msg.reference_frame_id = reference_frame_id_;
    target_msg.target_transform = iii_drone::types::transformMsgFromTransformMatrix(target_transform_);

    return target_msg;

}

target_type_t TargetAdapter::target_type() const {
    return target_type_;
}

int TargetAdapter::target_id() const {
    return target_id_;
}

std::string TargetAdapter::reference_frame_id() const {
    return reference_frame_id_;
}

iii_drone::types::transform_matrix_t TargetAdapter::target_transform() const {
    return target_transform_;
}

TargetAdapter & TargetAdapter::operator=(const TargetAdapter & other) {
    
    target_type_ = other.target_type_;
    target_id_ = other.target_id_;
    reference_frame_id_ = other.reference_frame_id_;
    target_transform_ = other.target_transform_;

    return *this;

}

bool TargetAdapter::operator==(const TargetAdapter & other) const {
    
    return (
        target_type_ == other.target_type_ &&
        target_id_ == other.target_id_ &&
        reference_frame_id_ == other.reference_frame_id_ &&
        target_transform_ == other.target_transform_
    );

}

bool TargetAdapter::operator!=(const TargetAdapter & other) const {
    return !(*this == other);
}