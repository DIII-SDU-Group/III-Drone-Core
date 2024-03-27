/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/maneuver/maneuver.hpp>

using namespace iii_drone::control::maneuver;
using namespace iii_drone::control;
using namespace iii_drone::types;
using namespace iii_drone::adapters;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

Maneuver::Maneuver() : 
    goal_handle_(nullptr),
    maneuver_type_(MANEUVER_TYPE_NONE), 
    maneuver_params_(nullptr) { 

    creation_time_ = rclcpp::Clock().now();
    terminated_ = true;
    success_ = true;

}

Maneuver::Maneuver(
    maneuver_type_t maneuver_type,
    const rclcpp_action::GoalUUID & uuid
) : 
    goal_handle_(nullptr),
    uuid_(uuid),
    maneuver_type_(maneuver_type), 
    maneuver_params_(nullptr) {

    creation_time_ = rclcpp::Clock().now();

}

Maneuver::Maneuver(const Maneuver & other) : 
    goal_handle_(other.goal_handle_),
    uuid_(other.uuid_),
    maneuver_type_(other.maneuver_type_),
    maneuver_params_(other.maneuver_params_) {

    creation_time_ = rclcpp::Clock().now();

}
      
template <typename ActionT>
Maneuver Maneuver::FromGoalHandle(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle) {

    Maneuver m;
    m.goal_handle_ = goal_handle;
    m.uuid_ = goal_handle->get_goal_id();
    m.SetFromGoal<ActionT>(goal_handle->get_goal());
    m.creation_time_ = rclcpp::Clock().now();

    return m;

}

template <typename ActionT>
void Maneuver::SetFromGoal(const std::shared_ptr<const typename ActionT::Goal> goal) {

    if constexpr (std::is_same<ActionT, iii_drone_interfaces::action::FlyToPosition>::value) {

        maneuver_type_ = MANEUVER_TYPE_FLY_TO_POSITION;
        maneuver_params_ = std::make_shared<fly_to_position_maneuver_params_t>();

        std::shared_ptr<fly_to_position_maneuver_params_t> p = std::static_pointer_cast<fly_to_position_maneuver_params_t>(maneuver_params_);

        if (p == nullptr) {
            std::string msg = "Maneuver::setFromGoal(): Failed to cast maneuver params to fly_to_position_maneuver_params_t";
            RCLCPP_FATAL(rclcpp::get_logger("rclcpp"),msg.c_str());
            throw std::runtime_error(msg);
        }

        *p = fly_to_position_maneuver_params_t(
            goal->frame_id,
            iii_drone::types::pointFromPointMsg(goal->target_position),
            goal->target_yaw
        );

    } else if constexpr (std::is_same<ActionT, iii_drone_interfaces::action::FlyToObject>::value) {

        maneuver_type_ = MANEUVER_TYPE_FLY_TO_OBJECT;

        maneuver_params_ = std::make_shared<fly_to_object_maneuver_params_t>();

        std::shared_ptr<fly_to_object_maneuver_params_t> p = std::static_pointer_cast<fly_to_object_maneuver_params_t>(maneuver_params_);

        if (p == nullptr) {
            std::string msg = "Maneuver::setFromGoal(): Failed to cast maneuver params to fly_to_object_maneuver_params_t";
            RCLCPP_FATAL(rclcpp::get_logger("rclcpp"),msg.c_str());
            throw std::runtime_error(msg);
        }

        *p = fly_to_object_maneuver_params_t(TargetAdapter(goal->target));

    } else if constexpr (std::is_same<ActionT, iii_drone_interfaces::action::CableLanding>::value) {

        maneuver_type_ = MANEUVER_TYPE_CABLE_LANDING;

        maneuver_params_ = nullptr;

    } else if constexpr (std::is_same<ActionT, iii_drone_interfaces::action::CableLanding>::value) {

        maneuver_type_ = MANEUVER_TYPE_CABLE_TAKEOFF;
        maneuver_params_ = std::make_shared<cable_takeoff_maneuver_params_t>();

        std::shared_ptr<cable_takeoff_maneuver_params_t> p = std::static_pointer_cast<cable_takeoff_maneuver_params_t>(maneuver_params_);

        if (p == nullptr) {
            std::string msg = "Maneuver::setFromGoal(): Failed to cast maneuver params to cable_takeoff_maneuver_params_t";
            RCLCPP_FATAL(rclcpp::get_logger("rclcpp"),msg.c_str());
            throw std::runtime_error(msg);
        }

        *p = cable_takeoff_maneuver_params_t(goal->target_cable_distance);

    } else if constexpr (std::is_same<ActionT, iii_drone_interfaces::action::Hover>::value) {

        maneuver_type_ = MANEUVER_TYPE_HOVER;
        maneuver_params_ = std::make_shared<hover_maneuver_params_t>();

        std::shared_ptr<hover_maneuver_params_t> p = std::static_pointer_cast<hover_maneuver_params_t>(maneuver_params_);

        if (p == nullptr) {
            std::string msg = "Maneuver::setFromGoal(): Failed to cast maneuver params to hover_maneuver_params_t";
            RCLCPP_FATAL(rclcpp::get_logger("rclcpp"),msg.c_str());
            throw std::runtime_error(msg);
        }

        *p = hover_maneuver_params_t(goal->duration_s);

    } else if constexpr (std::is_same<ActionT, iii_drone_interfaces::action::HoverByObject>::value) {

        maneuver_type_ = MANEUVER_TYPE_HOVER_BY_OBJECT;
        maneuver_params_ = std::make_shared<hover_by_object_maneuver_params_t>();

        std::shared_ptr<hover_by_object_maneuver_params_t> p = std::static_pointer_cast<hover_by_object_maneuver_params_t>(maneuver_params_);

        if (p == nullptr) {
            std::string msg = "Maneuver::setFromGoal(): Failed to cast maneuver params to hover_by_object_maneuver_params_t";
            RCLCPP_FATAL(rclcpp::get_logger("rclcpp"),msg.c_str());
            throw std::runtime_error(msg);
        }

        *p = hover_by_object_maneuver_params_t(
            TargetAdapter(goal->target),
            goal->duration_s
        );

    } else if constexpr (std::is_same<ActionT, iii_drone_interfaces::action::HoverOnCable>::value) {

        maneuver_type_ = MANEUVER_TYPE_HOVER_ON_CABLE;
        maneuver_params_ = std::make_shared<hover_on_cable_maneuver_params_t>();

        std::shared_ptr<hover_on_cable_maneuver_params_t> p = std::static_pointer_cast<hover_on_cable_maneuver_params_t>(maneuver_params_);

        if (p == nullptr) {
            std::string msg = "Maneuver::setFromGoal(): Failed to cast maneuver params to hover_on_cable_maneuver_params_t";
            RCLCPP_FATAL(rclcpp::get_logger("rclcpp"),msg.c_str());
            throw std::runtime_error(msg);
        }

        *p = hover_on_cable_maneuver_params_t(
            goal->target_cable_id,
            goal->target_z_velocity,
            goal->target_yaw_rate,
            goal->duration_s
        );

    // } else if constexpr (std::is_same<ActionT, iii_drone_interfaces::action::DisarmOnCable>::value) {

    //     maneuver_type_ = MANEUVER_TYPE_DISARM_ON_CABLE;
    //     maneuver_params_ = nullptr;

    // } else if constexpr (std::is_same<ActionT, iii_drone_interfaces::action::ArmOnCable>::value) {

    //     maneuver_type_ = MANEUVER_TYPE_ARM_ON_CABLE;
    //     maneuver_params_ = nullptr;

    // } else {

    //     maneuver_type_ = MANEUVER_TYPE_NONE;
    //     maneuver_params_ = nullptr;

    }
}

void Maneuver::Start() {

    terminated_ = false;
    success_ = false;

    switch(maneuver_type_) {
        case MANEUVER_TYPE_FLY_TO_POSITION: {
            auto goal_handle = std::static_pointer_cast<rclcpp_action::ServerGoalHandle<iii_drone_interfaces::action::FlyToPosition>>(goal_handle_);
            if (!goal_handle->is_executing()) goal_handle->execute();
            break;
        }

        case MANEUVER_TYPE_FLY_TO_OBJECT: {
            auto goal_handle = std::static_pointer_cast<rclcpp_action::ServerGoalHandle<iii_drone_interfaces::action::FlyToObject>>(goal_handle_);
            if (!goal_handle->is_executing()) goal_handle->execute();
            break;
        }

        case MANEUVER_TYPE_CABLE_LANDING: {
            auto goal_handle = std::static_pointer_cast<rclcpp_action::ServerGoalHandle<iii_drone_interfaces::action::CableLanding>>(goal_handle_);
            if (!goal_handle->is_executing()) goal_handle->execute();
            break;
        }

        case MANEUVER_TYPE_CABLE_TAKEOFF: {
            auto goal_handle = std::static_pointer_cast<rclcpp_action::ServerGoalHandle<iii_drone_interfaces::action::CableTakeoff>>(goal_handle_);
            if (!goal_handle->is_executing()) goal_handle->execute();
            break;
        }

        case MANEUVER_TYPE_HOVER: {
            auto goal_handle = std::static_pointer_cast<rclcpp_action::ServerGoalHandle<iii_drone_interfaces::action::Hover>>(goal_handle_);
            if (!goal_handle->is_executing()) goal_handle->execute();
            break;
        }

        case MANEUVER_TYPE_HOVER_BY_OBJECT: {
            auto goal_handle = std::static_pointer_cast<rclcpp_action::ServerGoalHandle<iii_drone_interfaces::action::HoverByObject>>(goal_handle_);
            if (!goal_handle->is_executing()) goal_handle->execute();
            break;
        }

        case MANEUVER_TYPE_HOVER_ON_CABLE: {
            auto goal_handle = std::static_pointer_cast<rclcpp_action::ServerGoalHandle<iii_drone_interfaces::action::HoverOnCable>>(goal_handle_);
            if (!goal_handle->is_executing()) goal_handle->execute();
            break;
        }

        default:
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Maneuver::Start(): Unknown maneuver type");
            break;
        
    }

    start_time_ = rclcpp::Clock().now();

}

void Maneuver::Terminate(bool success) {

    terminated_ = true;
    success_ = success;
    goal_handle_ = nullptr;

}

template <typename ActionT>
void Maneuver::PublishFeedback(const std::shared_ptr<void> feedback) {

    auto goal_handle = std::static_pointer_cast<rclcpp_action::ServerGoalHandle<ActionT>>(goal_handle_);
    auto feedback_casted = std::static_pointer_cast<typename ActionT::Feedback>(feedback);
    goal_handle->publish_feedback(feedback_casted);

}

bool Maneuver::operator==(const Maneuver & rhs) const {

    return uuid_ == rhs.uuid_ && maneuver_type_ == rhs.maneuver_type_;

}
bool Maneuver::operator!=(const Maneuver & rhs) const {
    return !(*this == rhs);
}
Maneuver & Maneuver::operator=(const Maneuver & rhs) {
    goal_handle_ = rhs.goal_handle_;
    uuid_ = rhs.uuid_;
    maneuver_type_ = rhs.maneuver_type_;
    maneuver_params_ = rhs.maneuver_params_;

    return *this;
}

const std::shared_ptr<void> Maneuver::goal_handle() const {
    return goal_handle_;
}

const rclcpp_action::GoalUUID Maneuver::uuid() const {
    return uuid_;
}

maneuver_type_t Maneuver::maneuver_type() const {
    return maneuver_type_;
}

const std::shared_ptr<void> Maneuver::maneuver_params() const {
    return maneuver_params_;
}

const rclcpp::Time Maneuver::creation_time() const {
    return creation_time_;
}

const rclcpp::Time Maneuver::start_time() const {
    return start_time_;
}

bool Maneuver::terminated() const {
    return terminated_;
}

bool Maneuver::success() const {
    return success_;
}

/*****************************************************************************/
// Explicit instantiation
/*****************************************************************************/

template Maneuver Maneuver::FromGoalHandle(const std::shared_ptr<rclcpp_action::ServerGoalHandle<iii_drone_interfaces::action::FlyToPosition>> goal_handle);
template Maneuver Maneuver::FromGoalHandle(const std::shared_ptr<rclcpp_action::ServerGoalHandle<iii_drone_interfaces::action::FlyToObject>> goal_handle);
template Maneuver Maneuver::FromGoalHandle(const std::shared_ptr<rclcpp_action::ServerGoalHandle<iii_drone_interfaces::action::CableLanding>> goal_handle);
template Maneuver Maneuver::FromGoalHandle(const std::shared_ptr<rclcpp_action::ServerGoalHandle<iii_drone_interfaces::action::CableTakeoff>> goal_handle);
template Maneuver Maneuver::FromGoalHandle(const std::shared_ptr<rclcpp_action::ServerGoalHandle<iii_drone_interfaces::action::Hover>> goal_handle);
template Maneuver Maneuver::FromGoalHandle(const std::shared_ptr<rclcpp_action::ServerGoalHandle<iii_drone_interfaces::action::HoverByObject>> goal_handle);
template Maneuver Maneuver::FromGoalHandle(const std::shared_ptr<rclcpp_action::ServerGoalHandle<iii_drone_interfaces::action::HoverOnCable>> goal_handle);

template void Maneuver::SetFromGoal<iii_drone_interfaces::action::FlyToPosition>(const std::shared_ptr<const iii_drone_interfaces::action::FlyToPosition::Goal> goal);
template void Maneuver::SetFromGoal<iii_drone_interfaces::action::FlyToObject>(const std::shared_ptr<const iii_drone_interfaces::action::FlyToObject::Goal> goal);
template void Maneuver::SetFromGoal<iii_drone_interfaces::action::CableLanding>(const std::shared_ptr<const iii_drone_interfaces::action::CableLanding::Goal> goal);
template void Maneuver::SetFromGoal<iii_drone_interfaces::action::CableTakeoff>(const std::shared_ptr<const iii_drone_interfaces::action::CableTakeoff::Goal> goal);
template void Maneuver::SetFromGoal<iii_drone_interfaces::action::Hover>(const std::shared_ptr<const iii_drone_interfaces::action::Hover::Goal> goal);
template void Maneuver::SetFromGoal<iii_drone_interfaces::action::HoverByObject>(const std::shared_ptr<const iii_drone_interfaces::action::HoverByObject::Goal> goal);
template void Maneuver::SetFromGoal<iii_drone_interfaces::action::HoverOnCable>(const std::shared_ptr<const iii_drone_interfaces::action::HoverOnCable::Goal> goal);

template void Maneuver::PublishFeedback<iii_drone_interfaces::action::FlyToPosition>(const std::shared_ptr<void> feedback);
template void Maneuver::PublishFeedback<iii_drone_interfaces::action::FlyToObject>(const std::shared_ptr<void> feedback);
template void Maneuver::PublishFeedback<iii_drone_interfaces::action::CableLanding>(const std::shared_ptr<void> feedback);
template void Maneuver::PublishFeedback<iii_drone_interfaces::action::CableTakeoff>(const std::shared_ptr<void> feedback);
template void Maneuver::PublishFeedback<iii_drone_interfaces::action::Hover>(const std::shared_ptr<void> feedback);
template void Maneuver::PublishFeedback<iii_drone_interfaces::action::HoverByObject>(const std::shared_ptr<void> feedback);
template void Maneuver::PublishFeedback<iii_drone_interfaces::action::HoverOnCable>(const std::shared_ptr<void> feedback);