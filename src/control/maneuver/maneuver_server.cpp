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

    RCLCPP_DEBUG(node_->get_logger(), "ManeuverServer::Start()");

    RCLCPP_DEBUG(node_->get_logger(), "ManeuverServer::Start(): Setting callbacks");

    register_maneuver_ = register_maneuver_function;
    update_maneuver_ = update_manuever_function;
    cancel_maneuver_ = cancel_maneuver_function;
    verify_maneuver_in_queue_ = verify_maneuver_in_queue_function;
    verify_maneuver_active_ = verify_maneuver_active_function;
    done_callback_ = done_callback;

    RCLCPP_DEBUG(node_->get_logger(), "ManeuverServer::Start(): Setting reference callback token");

    reference_callback_token_ = reference_callback_token;

    RCLCPP_DEBUG(node_->get_logger(), "ManeuverServer::Start(): Setting registered maneuvers map");

    registered_maneuvers_ = registered_maneuvers;

    running_ = true;

    RCLCPP_DEBUG(node_->get_logger(), "ManeuverServer::Start(): Finished");
}

void ManeuverServer::Stop() {
    running_ = false;
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

    RCLCPP_DEBUG(node_->get_logger(), "ManeuverServer::handleGoal()");

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

        RCLCPP_DEBUG(node_->get_logger(), "ManeuverServer::handleGoal(): Could not register maneuver, rejecting goal");
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (executing_instantly) {
        RCLCPP_DEBUG(node_->get_logger(), "ManeuverServer::handleGoal(): Accepting and executing goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    RCLCPP_DEBUG(node_->get_logger(), "ManeuverServer::handleGoal(): Accepting and deferring goal");

    return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;

}

template <typename ActionT>
rclcpp_action::CancelResponse ManeuverServer::handleCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle
) {

    RCLCPP_DEBUG(node_->get_logger(), "ManeuverServer::handleCancel()");

    (void)goal_handle;

    if (canCancel()) {
        RCLCPP_DEBUG(node_->get_logger(), "ManeuverServer::handleCancel(): Accepting cancel");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    RCLCPP_DEBUG(node_->get_logger(), "ManeuverServer::handleCancel(): Rejecting cancel");
    return rclcpp_action::CancelResponse::REJECT;

}

template <typename ActionT>
void ManeuverServer::handleAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle
) {

    RCLCPP_DEBUG(node_->get_logger(), "ManeuverServer::handleAccepted()");

    Maneuver maneuver = Maneuver::FromGoalHandle<ActionT>(goal_handle);

    RCLCPP_DEBUG(node_->get_logger(), "ManeuverServer::handleAccepted(): Updating maneuver with the maneuver scheduler");

    if (!update_maneuver_(maneuver)) {
        RCLCPP_DEBUG(node_->get_logger(), "ManeuverServer::handleAccepted(): Could not update maneuver, aborting goal");
        goal_handle->abort(std::make_shared<typename ActionT::Result>());
        return;
    }

    RCLCPP_DEBUG(node_->get_logger(), "ManeuverServer::handleAccepted(): Starting async execution");

    std::thread{std::bind(
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

    RCLCPP_DEBUG(node_->get_logger(), "ManeuverServer::asyncExecute()");

    Maneuver maneuver = Maneuver::FromGoalHandle<ActionT>(goal_handle);

    rclcpp::Rate rate = rclcpp::Rate(std::chrono::milliseconds(wait_for_execute_poll_ms_));

    while(!goal_handle->is_executing()) {
        // RCLCPP_DEBUG(node_->get_logger(), "ManeuverServer::asyncExecute(): Waiting for goal to start executing");
        rate.sleep();
        if (!verify_maneuver_in_queue_(maneuver)) {
            RCLCPP_DEBUG(node_->get_logger(), "ManeuverServer::asyncExecute(): Goal was removed from queue, aborting goal and cancelling maneuver with the scheduler");
            goal_handle->abort(std::make_shared<typename ActionT::Result>());
            maneuver.Terminate(false);
            cancel_maneuver_(maneuver);
            current_maneuver_ = Maneuver();

            return;
        }
    }

    RCLCPP_DEBUG(node_->get_logger(), "ManeuverServer::asyncExecute(): Starting execution");

    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);

    if (!lock.owns_lock()) {
        RCLCPP_ERROR(node_->get_logger(), "ManeuverServer::asyncExecute(): Could not acquire lock, aborting goal");
        goal_handle->canceled(std::make_shared<typename ActionT::Result>());
        return;
    }

    current_maneuver_ = maneuver;

    if (!reference_callback_token_->Acquire()) {
        RCLCPP_ERROR(node_->get_logger(), "ManeuverServer::asyncExecute(): Could not acquire reference callback token, aborting goal");
        goal_handle->canceled(std::make_shared<typename ActionT::Result>());
        maneuver.Terminate(false);
        cancel_maneuver_(maneuver);
        current_maneuver_ = Maneuver();

        return;
    }

    startExecution(maneuver);

    RCLCPP_DEBUG(node_->get_logger(), "ManeuverServer::asyncExecute(): Storing reference callback");

    Atomic<std::function<Reference(const State &)>> &get_reference_callback = reference_callback_token_->resource();

    get_reference_callback.Store(
        std::bind(
            &ManeuverServer::computeReference, 
            this, 
            std::placeholders::_1
        )
    );

    // reference_callback_token_->resource().Store(
    //     std::bind(
    //         &ManeuverServer::computeReference, 
    //         this, 
    //         std::placeholders::_1
    //     )
    // );

    bool success;

    while(true) {

        if (!verify_maneuver_active_(maneuver) || !running_) {
            RCLCPP_DEBUG(node_->get_logger(), "ManeuverServer::asyncExecute(): Goal was removed from active maneuvers, cancelling goal");
            publishResultAndFinalize(
                maneuver,
                MANEUVER_RESULT_TYPE_CANCEL
            );
            maneuver.Terminate(false);
            cancel_maneuver_(maneuver);
            current_maneuver_ = Maneuver();
            return;
        }

        if (goal_handle->is_canceling()) {
            RCLCPP_DEBUG(node_->get_logger(), "ManeuverServer::asyncExecute(): Goal is canceling, cancelling maneuver");
            success = false;
            publishResultAndFinalize(
                maneuver,
                MANEUVER_RESULT_TYPE_CANCEL
            );
            break;
        }

        if (hasSucceeded(maneuver)) {
            RCLCPP_DEBUG(node_->get_logger(), "ManeuverServer::asyncExecute(): Maneuver succeeded");
            success = true;
            publishResultAndFinalize(
                maneuver,
                MANEUVER_RESULT_TYPE_SUCCEED
            );
            break;
        }

        if (hasFailed(maneuver)) {
            RCLCPP_DEBUG(node_->get_logger(), "ManeuverServer::asyncExecute(): Maneuver failed");
            success = false;
            publishResultAndFinalize(
                maneuver,
                MANEUVER_RESULT_TYPE_ABORT
            );
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

    if (success) {
        RCLCPP_DEBUG(node_->get_logger(), "ManeuverServer::asyncExecute(): Goal succeeded");
        registerReferenceCallbackOnSuccess(maneuver);
    }

    RCLCPP_DEBUG(node_->get_logger(), "ManeuverServer::asyncExecute(): Terminating maneuver and releasing reference callback token");

    maneuver.Terminate(success);

    done_callback_(maneuver);

    current_maneuver_ = Maneuver();

    reference_callback_token_->Release();

}

void ManeuverServer::registerCallback(const std::function<Reference(const State &)> &callback) {
    reference_callback_token_->resource() = callback;
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