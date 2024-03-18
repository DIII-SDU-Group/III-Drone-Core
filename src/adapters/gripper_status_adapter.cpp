/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/adapters/gripper_status_adapter.hpp>

using namespace iii_drone::adapters;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

GripperStatusAdapter::GripperStatusAdapter() {
    
    open_ = false;

}

GripperStatusAdapter::GripperStatusAdapter(const GripperStatusAdapter & other) {
    
    open_ = other.open_;

}

GripperStatusAdapter::GripperStatusAdapter(const iii_drone_interfaces::msg::GripperStatus & msg) {
    
    UpdateFromMsg(msg);

}

void GripperStatusAdapter::UpdateFromMsg(const iii_drone_interfaces::msg::GripperStatus & msg) {
    
    open_ = msg.gripper_status == msg.GRIPPER_STATUS_OPEN;

}

GripperStatusAdapter & GripperStatusAdapter::operator=(const GripperStatusAdapter & other) {
    
    open_ = other.open_;

    return *this;

}

bool GripperStatusAdapter::open() const {
    return open_;
}