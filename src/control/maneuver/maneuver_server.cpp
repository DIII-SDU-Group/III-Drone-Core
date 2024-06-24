/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/maneuver/maneuver_server.hpp>

using namespace iii_drone::control::maneuver;
using namespace iii_drone::control;
using namespace iii_drone::utils;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

#include <iostream>

ManeuverServer::ManeuverServer(
    rclcpp_lifecycle::LifecycleNode * node,
    CombinedDroneAwarenessHandler::SharedPtr awareness_handler,
    const std::string & action_name,
    unsigned int wait_for_execute_poll_ms,
    unsigned int evaluate_done_poll_ms
) : node_(node),
    awareness_handler_(awareness_handler),
    action_name_(action_name),
    wait_for_execute_poll_ms_(wait_for_execute_poll_ms),
    evaluate_done_poll_ms_(evaluate_done_poll_ms),
    server_(nullptr) { }

void ManeuverServer::Start(
    std::function<bool(Maneuver, bool &)> register_maneuver_function,
    std::function<bool(Maneuver)> update_manuever_function,
    std::function<bool(Maneuver)> cancel_maneuver_function,
    std::function<bool(Maneuver)> verify_maneuver_in_queue_function,
    std::function<bool(Maneuver)> verify_maneuver_active_function,
    std::function<void(Maneuver)> done_callback,
    ReferenceCallbackToken::SharedPtr reference_callback_token,
    std::map<iii_drone::control::maneuver::maneuver_type_t, std::shared_ptr<ManeuverServer>> registered_maneuvers
) {

    std::string action_name_ = action_name();

    RCLCPP_DEBUG(node_->get_logger(), "ManeuverServer::Start(): %s", action_name_.c_str());

    RCLCPP_DEBUG(node_->get_logger(), "ManeuverServer::Start(): %s: Setting callbacks", action_name_.c_str());

    register_maneuver_ = register_maneuver_function;
    update_maneuver_ = update_manuever_function;
    cancel_maneuver_ = cancel_maneuver_function;
    verify_maneuver_in_queue_ = verify_maneuver_in_queue_function;
    verify_maneuver_active_ = verify_maneuver_active_function;
    done_callback_ = done_callback;

    RCLCPP_DEBUG(node_->get_logger(), "ManeuverServer::Start(): %s: Setting reference callback token", action_name_.c_str());

    reference_callback_token_ = reference_callback_token;

    RCLCPP_DEBUG(node_->get_logger(), "ManeuverServer::Start(): %s: Setting registered maneuvers map", action_name_.c_str());

    registered_maneuvers_ = registered_maneuvers;

    running_ = true;

    RCLCPP_DEBUG(node_->get_logger(), "ManeuverServer::Start(): %s: Finished", action_name_.c_str());
}

void ManeuverServer::Stop() {
    running_ = false;

    reference_callback_token_.reset();
    reference_callback_token_ = nullptr;
}

bool ManeuverServer::running() const {
    return running_;
}

std::string ManeuverServer::action_name() const {
    return action_name_;
}

const CombinedDroneAwarenessHandler::SharedPtr &ManeuverServer::awareness_handler() const {
    return awareness_handler_;
}

const iii_drone::utils::Atomic<iii_drone::control::maneuver::Maneuver> & ManeuverServer::current_maneuver() const {
    return current_maneuver_;
}

std::map<iii_drone::control::maneuver::maneuver_type_t, std::shared_ptr<ManeuverServer>> ManeuverServer::registered_maneuvers() const {
    return registered_maneuvers_;
}

rclcpp_lifecycle::LifecycleNode * ManeuverServer::node() const {
    return node_;
}

std::shared_ptr<void> ManeuverServer::getFeedback(Maneuver &) {
    
    return nullptr;
}

template <typename ActionT>
rclcpp_action::GoalResponse ManeuverServer::handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const typename ActionT::Goal> goal
) {

    std::string action_name_ = action_name();

    RCLCPP_DEBUG(node_->get_logger(), "ManeuverServer::handleGoal(): %s: Received goal", action_name_.c_str());

    if (!running_) {
        return rclcpp_action::GoalResponse::REJECT;
    }

    (void)uuid;

    Maneuver maneuver(
        maneuver_type(),
        uuid
    );

    maneuver.SetFromGoal<ActionT>(goal);

    bool executing_instantly;
    bool success = register_maneuver_(maneuver, executing_instantly);

    if (!success) {

        RCLCPP_WARN(node_->get_logger(), "ManeuverServer::handleGoal(): %s: Could not register maneuver, rejecting goal", action_name_.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (executing_instantly) {
        RCLCPP_INFO(node_->get_logger(), "ManeuverServer::handleGoal(): %s: Accepting and executing goal", action_name_.c_str());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    RCLCPP_INFO(node_->get_logger(), "ManeuverServer::handleGoal(): %s: Accepting and deferring goal", action_name_.c_str());

    return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;

}

template <typename ActionT>
rclcpp_action::CancelResponse ManeuverServer::handleCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle
) {

    std::string action_name_ = action_name();

    RCLCPP_DEBUG(node_->get_logger(), "ManeuverServer::handleCancel(): %s: Received cancel request", action_name_.c_str());

    (void)goal_handle;

    if (canCancel()) {
        RCLCPP_INFO(node_->get_logger(), "ManeuverServer::handleCancel(): %s: Accepting cancel", action_name_.c_str());
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    RCLCPP_WARN(node_->get_logger(), "ManeuverServer::handleCancel(): %s: Rejecting cancel", action_name_.c_str());
    return rclcpp_action::CancelResponse::REJECT;

}

template <typename ActionT>
void ManeuverServer::handleAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle
) {

    std::string action_name_ = action_name();

    RCLCPP_DEBUG(node_->get_logger(), "ManeuverServer::handleAccepted(): %s: Received accepted", action_name_.c_str());

    Maneuver maneuver = Maneuver::FromGoalHandle<ActionT>(goal_handle);

    RCLCPP_DEBUG(node_->get_logger(), "ManeuverServer::handleAccepted(): %s: Updating maneuver with the maneuver scheduler", action_name_.c_str());

    if (!update_maneuver_(maneuver)) {
        RCLCPP_WARN(node_->get_logger(), "ManeuverServer::handleAccepted(): %s: Could not update maneuver, aborting accepted", action_name_.c_str());
        goal_handle->abort(std::make_shared<typename ActionT::Result>());
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "ManeuverServer::handleAccepted(): %s: Starting async execution", action_name_.c_str());

    std::thread{
        std::bind(
            &ManeuverServer::asyncExecute<ActionT>,
            this, 
            std::placeholders::_1
        ),
        goal_handle
    }.detach();

}

template <typename ActionT>
void ManeuverServer::asyncExecute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle
) {

    std::string action_name_ = action_name();

    RCLCPP_INFO(node_->get_logger(), "ManeuverServer::asyncExecute(): %s", action_name_.c_str());

    Maneuver maneuver = Maneuver::FromGoalHandle<ActionT>(goal_handle);

    rclcpp::Rate rate = rclcpp::Rate(std::chrono::milliseconds(wait_for_execute_poll_ms_));

    auto abort_maneuver = [this, &maneuver, &goal_handle]() {
        if (!goal_handle->is_canceling())
            goal_handle->abort(std::make_shared<typename ActionT::Result>());
        maneuver.Terminate(false);
        cancel_maneuver_(maneuver);
        current_maneuver_ = Maneuver();
    };

    auto cancel_maneuver = [this, &maneuver, &goal_handle]() {
        if (!goal_handle->is_canceling())
            goal_handle->canceled(std::make_shared<typename ActionT::Result>());
        maneuver.Terminate(false);
        cancel_maneuver_(maneuver);
        current_maneuver_ = Maneuver();
    };

    while(!goal_handle->is_executing()) {
        // RCLCPP_DEBUG(node_->get_logger(), "ManeuverServer::asyncExecute(): Waiting for goal to start executing");
        rate.sleep();
        if (!verify_maneuver_in_queue_(maneuver)) {
            RCLCPP_WARN(
                node_->get_logger(), 
                "ManeuverServer::asyncExecute(): %s: Goal was removed from queue, aborting goal and cancelling maneuver with the scheduler", 
                action_name_.c_str()
            );
            abort_maneuver();
            return;
        }

        if (maneuver.canceling()) {
            RCLCPP_WARN(
                node_->get_logger(), 
                "ManeuverServer::asyncExecute(): %s: Goal is canceling, aborting goal and cancelling maneuver with the scheduler",
                action_name_.c_str()
            );
            cancel_maneuver();
            return;
        }
    }

    RCLCPP_DEBUG(
        node_->get_logger(), 
        "ManeuverServer::asyncExecute(): %s: Starting execution",
        action_name_.c_str()
    
    );

    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);

    if (!lock.owns_lock()) {
        RCLCPP_ERROR(
            node_->get_logger(), 
            "ManeuverServer::asyncExecute(): %s: Could not acquire lock, aborting goal",
            action_name_.c_str()
        );
        abort_maneuver();
        return;
    }

    current_maneuver_ = maneuver;

    if (!reference_callback_token_->Acquire()) {
        RCLCPP_ERROR(
            node_->get_logger(), 
            "ManeuverServer::asyncExecute(): %s: Could not acquire reference callback token, aborting goal",
            action_name_.c_str()
        );
        abort_maneuver();

        return;
    }

    startExecution(maneuver);

    RCLCPP_DEBUG(
        node_->get_logger(), 
        "ManeuverServer::asyncExecute(): %s: Storing reference callback",
        action_name_.c_str()
    );

    reference_callback_token_->resource().set(
        std::bind(
            &ManeuverServer::computeReference, 
            this, 
            std::placeholders::_1
        ),
        action_name()
    );

    bool success = false;
    bool active = true;
    bool canceling = false;
    bool failed = false;

    while(true) {

        if (!verify_maneuver_active_(maneuver) || !running_) {

            RCLCPP_WARN(
                node_->get_logger(), 
                "ManeuverServer::asyncExecute(): %s: Goal was removed from active maneuvers, cancelling goal",
                action_name_.c_str()
            );
            publishResultAndFinalize(
                maneuver,
                MANEUVER_RESULT_TYPE_CANCEL
            );
            cancel_maneuver_(maneuver);
            current_maneuver_ = Maneuver();

            return;
        }

        if (goal_handle->is_canceling()) {
            success = false;
            canceling = true;
            break;
        }

        if (hasSucceeded(maneuver)) {
            success = true;
            break;
        }

        if (hasFailed(maneuver)) {
            success = false;
            break;
        }

        // RCLCPP_DEBUG(node_->get_logger(), "ManeuverServer::asyncExecute(): Publishing feedback");

        auto feedback = getFeedback(maneuver);

        if (feedback == nullptr) {
            
            feedback = std::static_pointer_cast<void>(
                std::make_shared<typename ActionT::Feedback>()
            );
        
        }

        maneuver.PublishFeedback<ActionT>(feedback);

        rate.sleep();

    }

    if (canceling) {

        RCLCPP_WARN(
            node_->get_logger(), 
            "ManeuverServer::asyncExecute(): Goal is canceling, cancelling maneuver",
            action_name_.c_str()
        );
        
        publishResultAndFinalize(
            maneuver,
            MANEUVER_RESULT_TYPE_CANCEL
        );

    } else if (success) {

        RCLCPP_INFO(
            node_->get_logger(), 
            "ManeuverServer::asyncExecute(): %s: Maneuver succeeded",
            action_name_.c_str()
        );

        registerReferenceCallbackOnSuccess(maneuver);

        publishResultAndFinalize(
            maneuver,
            MANEUVER_RESULT_TYPE_SUCCEED
        );

    } else {

        RCLCPP_WARN(
            node_->get_logger(), 
            "ManeuverServer::asyncExecute(): %s: Maneuver failed",
            action_name_.c_str()
        );
        
        publishResultAndFinalize(
            maneuver,
            MANEUVER_RESULT_TYPE_ABORT
        );

    }

    RCLCPP_DEBUG(
        node_->get_logger(), 
        "ManeuverServer::asyncExecute(): %s: Terminating maneuver.",
        action_name_.c_str()
    );

    maneuver.Terminate(success);

    done_callback_(maneuver);

    current_maneuver_ = Maneuver();

    RCLCPP_DEBUG(
        node_->get_logger(), 
        "ManeuverServer::asyncExecute(): %s: Releasing reference callback token",
        action_name_.c_str()
    );

    reference_callback_token_->Release();

    RCLCPP_INFO(
        node_->get_logger(), 
        "ManeuverServer::asyncExecute(): %s: Finished",
        action_name_.c_str()
    );

}

void ManeuverServer::registerCallback(const ReferenceCallback &callback) {
    reference_callback_token_->resource().set(
        callback,
        action_name()
    );
}

template <typename ActionT>
void ManeuverServer::createServer() {
    
    // Create action server
    std::shared_ptr<rclcpp_action::Server<ActionT>> server = rclcpp_action::create_server<ActionT>(
        node_,
        action_name_,
        std::bind(
            &ManeuverServer::handleGoal<ActionT>,
            this, 
            std::placeholders::_1, 
            std::placeholders::_2
        ),
        std::bind(
            &ManeuverServer::handleCancel<ActionT>,
            this, 
            std::placeholders::_1
        ),
        std::bind(
            &ManeuverServer::handleAccepted<ActionT>,
            this, 
            std::placeholders::_1
        )
    );

    server_ = std::static_pointer_cast<void>(server);

}

const rclcpp_lifecycle::LifecycleNode & ManeuverServer::node_handle() const {
    return *node_;
}



/*****************************************************************************/
// Explicit instantiation
/*****************************************************************************/

template rclcpp_action::GoalResponse iii_drone::control::maneuver::ManeuverServer::handleGoal<iii_drone_interfaces::action::FlyToPosition>(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const iii_drone_interfaces::action::FlyToPosition::Goal>
);
template rclcpp_action::GoalResponse iii_drone::control::maneuver::ManeuverServer::handleGoal<iii_drone_interfaces::action::FlyToObject>(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const iii_drone_interfaces::action::FlyToObject::Goal>
);
template rclcpp_action::GoalResponse iii_drone::control::maneuver::ManeuverServer::handleGoal<iii_drone_interfaces::action::CableLanding>(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const iii_drone_interfaces::action::CableLanding::Goal>
);
template rclcpp_action::GoalResponse iii_drone::control::maneuver::ManeuverServer::handleGoal<iii_drone_interfaces::action::CableTakeoff>(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const iii_drone_interfaces::action::CableTakeoff::Goal>
);
template rclcpp_action::GoalResponse iii_drone::control::maneuver::ManeuverServer::handleGoal<iii_drone_interfaces::action::Hover>(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const iii_drone_interfaces::action::Hover::Goal>
);
template rclcpp_action::GoalResponse iii_drone::control::maneuver::ManeuverServer::handleGoal<iii_drone_interfaces::action::HoverByObject>(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const iii_drone_interfaces::action::HoverByObject::Goal>
);
template rclcpp_action::GoalResponse iii_drone::control::maneuver::ManeuverServer::handleGoal<iii_drone_interfaces::action::HoverOnCable>(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const iii_drone_interfaces::action::HoverOnCable::Goal>
);

template rclcpp_action::CancelResponse iii_drone::control::maneuver::ManeuverServer::handleCancel<iii_drone_interfaces::action::FlyToPosition>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<iii_drone_interfaces::action::FlyToPosition>>
);
template rclcpp_action::CancelResponse iii_drone::control::maneuver::ManeuverServer::handleCancel<iii_drone_interfaces::action::FlyToObject>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<iii_drone_interfaces::action::FlyToObject>>
);
template rclcpp_action::CancelResponse iii_drone::control::maneuver::ManeuverServer::handleCancel<iii_drone_interfaces::action::CableLanding>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<iii_drone_interfaces::action::CableLanding>>
);
template rclcpp_action::CancelResponse iii_drone::control::maneuver::ManeuverServer::handleCancel<iii_drone_interfaces::action::CableTakeoff>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<iii_drone_interfaces::action::CableTakeoff>>
);
template rclcpp_action::CancelResponse iii_drone::control::maneuver::ManeuverServer::handleCancel<iii_drone_interfaces::action::Hover>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<iii_drone_interfaces::action::Hover>>
);
template rclcpp_action::CancelResponse iii_drone::control::maneuver::ManeuverServer::handleCancel<iii_drone_interfaces::action::HoverByObject>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<iii_drone_interfaces::action::HoverByObject>>
);
template rclcpp_action::CancelResponse iii_drone::control::maneuver::ManeuverServer::handleCancel<iii_drone_interfaces::action::HoverOnCable>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<iii_drone_interfaces::action::HoverOnCable>>
);

template void iii_drone::control::maneuver::ManeuverServer::handleAccepted<iii_drone_interfaces::action::FlyToPosition>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<iii_drone_interfaces::action::FlyToPosition>>
);
template void iii_drone::control::maneuver::ManeuverServer::handleAccepted<iii_drone_interfaces::action::FlyToObject>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<iii_drone_interfaces::action::FlyToObject>>
);
template void iii_drone::control::maneuver::ManeuverServer::handleAccepted<iii_drone_interfaces::action::CableLanding>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<iii_drone_interfaces::action::CableLanding>>
);
template void iii_drone::control::maneuver::ManeuverServer::handleAccepted<iii_drone_interfaces::action::CableTakeoff>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<iii_drone_interfaces::action::CableTakeoff>>
);
template void iii_drone::control::maneuver::ManeuverServer::handleAccepted<iii_drone_interfaces::action::Hover>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<iii_drone_interfaces::action::Hover>>
);
template void iii_drone::control::maneuver::ManeuverServer::handleAccepted<iii_drone_interfaces::action::HoverByObject>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<iii_drone_interfaces::action::HoverByObject>>
);
template void iii_drone::control::maneuver::ManeuverServer::handleAccepted<iii_drone_interfaces::action::HoverOnCable>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<iii_drone_interfaces::action::HoverOnCable>>
);

template void iii_drone::control::maneuver::ManeuverServer::asyncExecute<iii_drone_interfaces::action::FlyToPosition>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<iii_drone_interfaces::action::FlyToPosition>>
);
template void iii_drone::control::maneuver::ManeuverServer::asyncExecute<iii_drone_interfaces::action::FlyToObject>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<iii_drone_interfaces::action::FlyToObject>>
);
template void iii_drone::control::maneuver::ManeuverServer::asyncExecute<iii_drone_interfaces::action::CableLanding>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<iii_drone_interfaces::action::CableLanding>>
);
template void iii_drone::control::maneuver::ManeuverServer::asyncExecute<iii_drone_interfaces::action::CableTakeoff>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<iii_drone_interfaces::action::CableTakeoff>>
);
template void iii_drone::control::maneuver::ManeuverServer::asyncExecute<iii_drone_interfaces::action::Hover>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<iii_drone_interfaces::action::Hover>>
);
template void iii_drone::control::maneuver::ManeuverServer::asyncExecute<iii_drone_interfaces::action::HoverByObject>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<iii_drone_interfaces::action::HoverByObject>>
);
template void iii_drone::control::maneuver::ManeuverServer::asyncExecute<iii_drone_interfaces::action::HoverOnCable>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<iii_drone_interfaces::action::HoverOnCable>>
);

template void iii_drone::control::maneuver::ManeuverServer::createServer<iii_drone_interfaces::action::FlyToPosition>();
template void iii_drone::control::maneuver::ManeuverServer::createServer<iii_drone_interfaces::action::FlyToObject>();
template void iii_drone::control::maneuver::ManeuverServer::createServer<iii_drone_interfaces::action::CableLanding>();
template void iii_drone::control::maneuver::ManeuverServer::createServer<iii_drone_interfaces::action::CableTakeoff>();
template void iii_drone::control::maneuver::ManeuverServer::createServer<iii_drone_interfaces::action::Hover>();
template void iii_drone::control::maneuver::ManeuverServer::createServer<iii_drone_interfaces::action::HoverByObject>();
template void iii_drone::control::maneuver::ManeuverServer::createServer<iii_drone_interfaces::action::HoverOnCable>();