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

#include <algorithm>
#include <cmath>
#include <iostream>

namespace {

enum class GoalTerminalState {
    Abort,
    Cancel
};

template <typename ActionT>
bool finalizeGoalSafely(
    rclcpp_lifecycle::LifecycleNode * node,
    const std::string & action_name,
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> & goal_handle,
    GoalTerminalState terminal_state,
    const char * context
) {
    auto result = std::make_shared<typename ActionT::Result>();

    try {
        const bool was_executing = goal_handle->is_executing();
        const bool was_canceling = goal_handle->is_canceling();

        if (!was_executing && !was_canceling) {
            // Deferred action goals must be executing before ROS accepts a terminal transition.
            RCLCPP_WARN(
                node->get_logger(),
                "ManeuverServer::finalizeGoalSafely(): %s: Goal was accepted but not executing during %s; executing before terminal transition",
                action_name.c_str(),
                context
            );
            goal_handle->execute();
        }

        if (terminal_state == GoalTerminalState::Cancel || goal_handle->is_canceling()) {
            RCLCPP_WARN(
                node->get_logger(),
                "ManeuverServer::finalizeGoalSafely(): %s: Finalizing goal as canceled during %s (was_executing=%s, was_canceling=%s)",
                action_name.c_str(),
                context,
                was_executing ? "true" : "false",
                was_canceling ? "true" : "false"
            );
            goal_handle->canceled(result);
        } else {
            RCLCPP_WARN(
                node->get_logger(),
                "ManeuverServer::finalizeGoalSafely(): %s: Finalizing goal as aborted during %s (was_executing=%s, was_canceling=%s)",
                action_name.c_str(),
                context,
                was_executing ? "true" : "false",
                was_canceling ? "true" : "false"
            );
            goal_handle->abort(result);
        }

        return true;
    } catch (const rclcpp::exceptions::RCLError & e) {
        RCLCPP_ERROR(
            node->get_logger(),
            "ManeuverServer::finalizeGoalSafely(): %s: ROS action transition failed during %s: %s",
            action_name.c_str(),
            context,
            e.what()
        );
    } catch (const std::exception & e) {
        RCLCPP_ERROR(
            node->get_logger(),
            "ManeuverServer::finalizeGoalSafely(): %s: Goal finalization failed during %s: %s",
            action_name.c_str(),
            context,
            e.what()
        );
    }

    return false;
}

} // namespace

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

    registered_maneuvers_.clear();

    reference_callback_token_.reset();
    reference_callback_token_ = nullptr;

    done_callback_ = nullptr;
    verify_maneuver_active_ = nullptr;
    verify_maneuver_in_queue_ = nullptr;
    cancel_maneuver_ = nullptr;
    update_maneuver_ = nullptr;
    register_maneuver_ = nullptr;

}

bool ManeuverServer::running() const {
    return running_;
}

void ManeuverServer::PauseReferenceStream() {
    reference_stream_paused_.Store(true);
}

ReferenceStreamRecoveryDisposition ManeuverServer::PrepareReferenceStreamRecovery(
    const State & stopped_state,
    std::string & reason
) {
    if (!reference_stream_paused_.Load()) {
        reason = "reference stream must be paused before rebase";
        return ReferenceStreamRecoveryDisposition::REJECT;
    }
    const Maneuver maneuver = current_maneuver_;
    if (!running_ || !verify_maneuver_active_ || !verify_maneuver_active_(maneuver)) {
        // Scheduler state is authoritative. This private copy is reconstructed
        // after action execution begins, so its local started flag is stale.
        reason = "maneuver is no longer active in the scheduler";
        return ReferenceStreamRecoveryDisposition::REJECT;
    }
    resetControlledCancellation();
    return referenceLossRecoveryDisposition(stopped_state, reason);
}

void ManeuverServer::CommitReferenceStreamRebase() {
    reference_stream_paused_.Store(false);
}

void ManeuverServer::AbortAfterReferenceLoss() {
    abort_after_reference_loss_.Store(true);
    reference_stream_paused_.Store(false);
}

bool ManeuverServer::referenceStreamPaused() const {
    return reference_stream_paused_.Load();
}

bool ManeuverServer::rebaseExecution(const State &, std::string & reason) {
    reason = action_name_ + " does not support safe transparent rebase";
    return false;
}

ReferenceStreamRecoveryDisposition ManeuverServer::referenceLossRecoveryDisposition(
    const State & stopped_state,
    std::string & reason
) {
    return rebaseExecution(stopped_state, reason)
        ? ReferenceStreamRecoveryDisposition::REBASE
        : ReferenceStreamRecoveryDisposition::REJECT;
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

std::optional<ControlledCancellationConfig> ManeuverServer::controlledCancellationConfig() const {
    return std::nullopt;
}

ControlledCancellationConfig ManeuverServer::controlledCancellationConfigFrom(
    const iii_drone::configuration::Configuration::SharedPtr & configuration
) {
    ControlledCancellationConfig config;
    config.limits.max_acceleration_m_s2 = configuration->GetParameter(
        "/control/maneuver_controller/controlled_cancel_max_deceleration_m_s2"
    ).as_double();
    config.limits.max_jerk_m_s3 = configuration->GetParameter(
        "/control/maneuver_controller/controlled_cancel_max_jerk_m_s3"
    ).as_double();
    config.limits.max_yaw_acceleration_rad_s2 = configuration->GetParameter(
        "/control/maneuver_controller/controlled_cancel_max_yaw_deceleration_rad_s2"
    ).as_double();
    config.limits.max_yaw_jerk_rad_s3 = configuration->GetParameter(
        "/control/maneuver_controller/controlled_cancel_max_yaw_jerk_rad_s3"
    ).as_double();
    config.velocity_threshold_m_s = configuration->GetParameter(
        "/control/maneuver_controller/controlled_cancel_velocity_threshold_m_s"
    ).as_double();
    config.yaw_rate_threshold_rad_s = configuration->GetParameter(
        "/control/maneuver_controller/controlled_cancel_yaw_rate_threshold_rad_s"
    ).as_double();
    config.settle_time_s = configuration->GetParameter(
        "/control/maneuver_controller/controlled_cancel_settle_time_s"
    ).as_double();
    return config;
}

Reference ManeuverServer::computeManagedReference(const State & state) {
    {
        std::lock_guard<std::mutex> lock(controlled_cancellation_mutex_);
        if (controlled_stop_trajectory_.has_value()) {
            const double elapsed_s = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - controlled_stop_start_time_
            ).count();
            return controlled_stop_trajectory_->sample(
                elapsed_s,
                node_->now()
            );
        }
    }

    Reference reference = computeReference(state);
    {
        std::lock_guard<std::mutex> lock(controlled_cancellation_mutex_);
        // Cancellation may have started while the derived callback was running.
        if (controlled_stop_trajectory_.has_value()) {
            const double elapsed_s = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - controlled_stop_start_time_
            ).count();
            return controlled_stop_trajectory_->sample(
                elapsed_s,
                node_->now()
            );
        }
        latest_managed_reference_ = reference;
    }
    return reference;
}

void ManeuverServer::resetControlledCancellation() {
    std::lock_guard<std::mutex> lock(controlled_cancellation_mutex_);
    latest_managed_reference_.reset();
    controlled_stop_trajectory_.reset();
    controlled_stop_below_threshold_since_.reset();
}

bool ManeuverServer::startControlledCancellation(const ControlledCancellationConfig & config) {
    std::lock_guard<std::mutex> lock(controlled_cancellation_mutex_);
    if (controlled_stop_trajectory_.has_value()) {
        return true;
    }

    const State state = awareness_handler_->GetState();
    Reference initial = latest_managed_reference_.value_or(Reference(state));
    const auto finite_or = [](double value, double fallback) {
        return std::isfinite(value) ? value : fallback;
    };
    const auto finite_vector_or = [](const iii_drone::types::vector_t & value,
                                     const iii_drone::types::vector_t & fallback) {
        return value.allFinite() ? value : fallback;
    };
    iii_drone::types::vector_t initial_acceleration = finite_vector_or(
        initial.acceleration(), iii_drone::types::vector_t::Zero()
    );
    double initial_yaw_acceleration = finite_or(initial.yaw_acceleration(), 0.0);
    initial = Reference(
        finite_vector_or(initial.position(), state.position()),
        finite_or(initial.yaw(), state.yaw()),
        finite_vector_or(initial.velocity(), state.velocity()),
        finite_or(initial.yaw_rate(), state.angular_velocity()(2)),
        initial_acceleration,
        initial_yaw_acceleration,
        node_->now()
    );

    try {
        controlled_stop_trajectory_.emplace(initial, config.limits);
    } catch (const std::exception & error) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "ManeuverServer::startControlledCancellation(): %s: Cannot construct bounded stop: %s",
            action_name_.c_str(),
            error.what()
        );
        return false;
    }
    controlled_stop_start_time_ = std::chrono::steady_clock::now();
    controlled_stop_below_threshold_since_.reset();
    RCLCPP_INFO(
        node_->get_logger(),
        "ManeuverServer::startControlledCancellation(): %s: Retaining control for %.3f s bounded stop from speed %.3f m/s",
        action_name_.c_str(),
        controlled_stop_trajectory_->durationS(),
        initial.velocity().norm()
    );
    return true;
}

bool ManeuverServer::controlledCancellationComplete(const ControlledCancellationConfig & config) {
    std::lock_guard<std::mutex> lock(controlled_cancellation_mutex_);
    if (!controlled_stop_trajectory_.has_value()) {
        return false;
    }
    const auto now = std::chrono::steady_clock::now();
    if (
        std::chrono::duration<double>(now - controlled_stop_start_time_).count() <
        controlled_stop_trajectory_->durationS()
    ) {
        controlled_stop_below_threshold_since_.reset();
        return false;
    }

    const State state = awareness_handler_->GetState();
    const bool below_threshold =
        state.velocity().allFinite() &&
        state.angular_velocity().allFinite() &&
        state.velocity().norm() <= config.velocity_threshold_m_s &&
        std::abs(state.angular_velocity()(2)) <= config.yaw_rate_threshold_rad_s;
    if (!below_threshold) {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            5000,
            "ManeuverServer::controlledCancellationComplete(): %s: Holding terminal stop; measured speed %.3f m/s (limit %.3f), yaw rate %.3f rad/s (limit %.3f)",
            action_name_.c_str(),
            state.velocity().norm(),
            config.velocity_threshold_m_s,
            std::abs(state.angular_velocity()(2)),
            config.yaw_rate_threshold_rad_s
        );
        controlled_stop_below_threshold_since_.reset();
        return false;
    }
    if (!controlled_stop_below_threshold_since_.has_value()) {
        controlled_stop_below_threshold_since_ = now;
        return config.settle_time_s <= 0.0;
    }
    return std::chrono::duration<double>(
        now - controlled_stop_below_threshold_since_.value()
    ).count() >= config.settle_time_s;
}

std::optional<Reference> ManeuverServer::controlledCancellationFinalReference() const {
    std::lock_guard<std::mutex> lock(controlled_cancellation_mutex_);
    if (!controlled_stop_trajectory_.has_value()) {
        return std::nullopt;
    }
    return controlled_stop_trajectory_->terminalReference(node_->now());
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
        finalizeGoalSafely<ActionT>(
            node_,
            action_name_,
            goal_handle,
            GoalTerminalState::Abort,
            "accepted-goal scheduler update failure"
        );
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
        finalizeGoalSafely<ActionT>(
            node_,
            action_name(),
            goal_handle,
            GoalTerminalState::Abort,
            "pre-execution abort"
        );
        maneuver.Terminate(false);
        cancel_maneuver_(maneuver);
        current_maneuver_ = Maneuver();
    };

    auto cancel_maneuver = [this, &maneuver, &goal_handle]() {
        finalizeGoalSafely<ActionT>(
            node_,
            action_name(),
            goal_handle,
            GoalTerminalState::Cancel,
            "pre-execution cancel"
        );
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

    resetControlledCancellation();
    reference_stream_paused_.Store(false);
    abort_after_reference_loss_.Store(false);

    if (!reference_callback_token_->Acquire()) {
        RCLCPP_ERROR(
            node_->get_logger(), 
            "ManeuverServer::asyncExecute(): %s: Could not acquire reference callback token, aborting goal",
            action_name_.c_str()
        );
        abort_maneuver();

        return;
    }

    // Maneuver initialization may perform synchronous planning. Publish a
    // current-pose hold under this maneuver's provider identity immediately
    // after acquiring the token, then replace it with computeReference below.
    // This prevents clients from seeing either the previous maneuver's
    // callback or an invalid reference while initialization is in progress.
    const Reference initialization_hold =
        Reference(awareness_handler_->GetState()).CopyWithNans();
    reference_callback_token_->resource().set(
        [initialization_hold](const State &) {
            return initialization_hold.CopyWithNewStamp();
        },
        action_name()
    );

    startExecution(maneuver);

    RCLCPP_DEBUG(
        node_->get_logger(), 
        "ManeuverServer::asyncExecute(): %s: Storing reference callback",
        action_name_.c_str()
    );

    reference_callback_token_->resource().set(
        std::bind(
            &ManeuverServer::computeManagedReference,
            this, 
            std::placeholders::_1
        ),
        action_name()
    );

    bool success = false;
    bool canceling = false;
    bool controlled_cancel_started = false;

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
            maneuver.Terminate(false);
            cancel_maneuver_(maneuver);
            current_maneuver_ = Maneuver();
            reference_callback_token_->Release();

            return;
        }

        if (goal_handle->is_canceling()) {
            success = false;
            const auto cancel_config = controlledCancellationConfig();
            if (!cancel_config.has_value()) {
                canceling = true;
                break;
            }
            if (!controlled_cancel_started) {
                controlled_cancel_started = startControlledCancellation(cancel_config.value());
                if (!controlled_cancel_started) {
                    RCLCPP_ERROR(
                        node_->get_logger(),
                        "ManeuverServer::asyncExecute(): %s: Rejecting unsafe immediate control release after stop generation failure",
                        action_name_.c_str()
                    );
                }
            }
            if (
                controlled_cancel_started &&
                controlledCancellationComplete(cancel_config.value())
            ) {
                canceling = true;
                break;
            }

            auto feedback = getFeedback(maneuver);
            if (feedback == nullptr) {
                feedback = std::static_pointer_cast<void>(
                    std::make_shared<typename ActionT::Feedback>()
                );
            }
            maneuver.PublishFeedback<ActionT>(feedback);
            rate.sleep();
            continue;
        }

        if (reference_stream_paused_.Load()) {
            auto feedback = getFeedback(maneuver);
            if (feedback == nullptr) {
                feedback = std::static_pointer_cast<void>(
                    std::make_shared<typename ActionT::Feedback>()
                );
            }
            maneuver.PublishFeedback<ActionT>(feedback);
            rate.sleep();
            continue;
        }

        if (abort_after_reference_loss_.Load()) {
            RCLCPP_WARN(
                node_->get_logger(),
                "ManeuverServer::asyncExecute(): %s: Aborting after bounded reference-loss stop",
                action_name_.c_str()
            );
            success = false;
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

    maneuver_result_type_t maneuver_result_type = MANEUVER_RESULT_TYPE_ABORT;

    if (canceling) {

        RCLCPP_WARN(
            node_->get_logger(), 
            "ManeuverServer::asyncExecute(): %s: Goal is canceling, cancelling maneuver",
            action_name_.c_str()
        );
        maneuver_result_type = MANEUVER_RESULT_TYPE_CANCEL;

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
        maneuver_result_type = MANEUVER_RESULT_TYPE_ABORT;

    }

    if (!success) {

        publishResultAndFinalize(
            maneuver,
            maneuver_result_type
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

    reference_callback_token_->Release();

    RCLCPP_DEBUG(
        node_->get_logger(), 
        "ManeuverServer::asyncExecute(): %s: Released reference callback token",
        action_name_.c_str()
    );

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
template rclcpp_action::GoalResponse iii_drone::control::maneuver::ManeuverServer::handleGoal<iii_drone_interfaces::action::FollowWaypointPath>(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const iii_drone_interfaces::action::FollowWaypointPath::Goal>
);
template rclcpp_action::GoalResponse iii_drone::control::maneuver::ManeuverServer::handleGoal<iii_drone_interfaces::action::CableAwareFlyToPosition>(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const iii_drone_interfaces::action::CableAwareFlyToPosition::Goal>
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
template rclcpp_action::CancelResponse iii_drone::control::maneuver::ManeuverServer::handleCancel<iii_drone_interfaces::action::FollowWaypointPath>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<iii_drone_interfaces::action::FollowWaypointPath>>
);
template rclcpp_action::CancelResponse iii_drone::control::maneuver::ManeuverServer::handleCancel<iii_drone_interfaces::action::CableAwareFlyToPosition>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<iii_drone_interfaces::action::CableAwareFlyToPosition>>
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
template void iii_drone::control::maneuver::ManeuverServer::handleAccepted<iii_drone_interfaces::action::FollowWaypointPath>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<iii_drone_interfaces::action::FollowWaypointPath>>
);
template void iii_drone::control::maneuver::ManeuverServer::handleAccepted<iii_drone_interfaces::action::CableAwareFlyToPosition>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<iii_drone_interfaces::action::CableAwareFlyToPosition>>
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
template void iii_drone::control::maneuver::ManeuverServer::asyncExecute<iii_drone_interfaces::action::FollowWaypointPath>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<iii_drone_interfaces::action::FollowWaypointPath>>
);
template void iii_drone::control::maneuver::ManeuverServer::asyncExecute<iii_drone_interfaces::action::CableAwareFlyToPosition>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<iii_drone_interfaces::action::CableAwareFlyToPosition>>
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
template void iii_drone::control::maneuver::ManeuverServer::createServer<iii_drone_interfaces::action::FollowWaypointPath>();
template void iii_drone::control::maneuver::ManeuverServer::createServer<iii_drone_interfaces::action::CableAwareFlyToPosition>();
template void iii_drone::control::maneuver::ManeuverServer::createServer<iii_drone_interfaces::action::FlyToObject>();
template void iii_drone::control::maneuver::ManeuverServer::createServer<iii_drone_interfaces::action::CableLanding>();
template void iii_drone::control::maneuver::ManeuverServer::createServer<iii_drone_interfaces::action::CableTakeoff>();
template void iii_drone::control::maneuver::ManeuverServer::createServer<iii_drone_interfaces::action::Hover>();
template void iii_drone::control::maneuver::ManeuverServer::createServer<iii_drone_interfaces::action::HoverByObject>();
template void iii_drone::control::maneuver::ManeuverServer::createServer<iii_drone_interfaces::action::HoverOnCable>();
