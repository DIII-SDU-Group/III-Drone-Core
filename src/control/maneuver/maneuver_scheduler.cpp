/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/maneuver/maneuver_scheduler.hpp>

using namespace iii_drone::control;
using namespace iii_drone::control::maneuver;
using namespace iii_drone::types;
using namespace iii_drone::utils;
using namespace iii_drone::adapters;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

ManeuverScheduler::ManeuverScheduler(
    rclcpp::Node *node,
    const CombinedDroneAwarenessHandler::SharedPtr combined_drone_awareness_handler,
    const iii_drone::configuration::ParameterBundle::SharedPtr parameters,
    rclcpp::CallbackGroup::SharedPtr maneuver_execution_callback_group
) : node_(node),
    combined_drone_awareness_handler_(combined_drone_awareness_handler),
    parameters_(parameters),
    maneuver_execution_callback_group_(maneuver_execution_callback_group),
    reference_callback_token_(
        reference_callback_,
        std::bind(
            &ManeuverScheduler::onReferenceCallbackTokenReacquired,
            this
        )
    ) {

    maneuver_queue_ = std::make_unique<ManeuverQueue>(parameters_->GetParameter("maneuver_queue_size").as_int());
    
    current_maneuver_ = Maneuver();

    reference_publisher_ = node_->create_publisher<iii_drone_interfaces::msg::Reference>(
        "reference",
        10 // Fix QoS
    );

    maneuver_execution_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(parameters_->GetParameter("maneuver_execution_period_ms").as_int()),
        std::bind(
            &ManeuverScheduler::maneuverExecutionTimerCallback,
            this
        ),
        maneuver_execution_callback_group_
    );

    maneuver_execution_timer_->cancel();

}

void ManeuverScheduler::RegisterManeuverServer(
    maneuver_type_t maneuver_type,
    ManeuverServer::SharedPtr maneuver_server
) {
    
    // Check if the maneuver type is already registered
    if (registered_maneuvers_.find(maneuver_type) != registered_maneuvers_.end()) {

        std::string msg = "ManeuverScheduler::RegisterManeuverServer(): maneuver type " + std::to_string(maneuver_type) + " already registered, but was attempted registered again.";

        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), msg.c_str());

        return;

    }

    registered_maneuvers_.insert(
        std::pair<maneuver_type_t, ManeuverServer::SharedPtr>(
            maneuver_type,
            maneuver_server
        )
    );

    maneuver_server->Start(
        std::bind(
            &ManeuverScheduler::RegisterManeuver,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        ),
        std::bind(
            &ManeuverScheduler::UpdateManeuver,
            this,
            std::placeholders::_1
        ),
        std::bind(
            &ManeuverScheduler::CancelManeuver,
            this,
            std::placeholders::_1
        ),
        [this](Maneuver maneuver) -> bool {
            std::shared_lock<std::shared_mutex> lck(maneuver_mutex_);

            Maneuver mn = maneuver_queue_->Find(maneuver.uuid());

            return mn == maneuver || maneuver == *current_maneuver_;
        },
        [this](Maneuver maneuver) -> bool {
            std::shared_lock<std::shared_mutex> lck(maneuver_mutex_);

            return maneuver == *current_maneuver_;
        },
        std::bind(
            &ManeuverScheduler::onManeuverCompleted,
            this,
            std::placeholders::_1
        ),
        std::make_shared<ReferenceCallbackToken>(
            reference_callback_token_.CreateSlaveHandle(
                maneuver_server->action_name()
            )
        ),
        registered_maneuvers_
    );

    if (maneuver_type == MANEUVER_TYPE_HOVER_BY_OBJECT) {

        std::static_pointer_cast<HoverByObjectManeuverServer>(maneuver_server)->RegisterOnFailCallback(
            std::bind(
                &ManeuverScheduler::onHoveringFail,
                this
            )
        );

    } else if (maneuver_type == MANEUVER_TYPE_HOVER_ON_CABLE) {

        std::static_pointer_cast<HoverOnCableManeuverServer>(maneuver_server)->RegisterOnFailCallback(
            std::bind(
                &ManeuverScheduler::onHoveringFail,
                this
            )
        );

    }

}

combined_drone_awareness_t ManeuverScheduler::ProjectExpectedAwarenessFull(const Maneuver & maneuver) const {

    std::vector<Maneuver> maneuver_queue = maneuver_queue_->vector();

    Maneuver current_maneuver = current_maneuver_.Load();

    if (maneuver_queue.size() == 0 || maneuver_queue[0] != current_maneuver) {

        maneuver_queue.insert(
            maneuver_queue.begin(), 
            current_maneuver
        );

    }

    maneuver_queue.push_back(maneuver);

    combined_drone_awareness_t awareness = combined_drone_awareness_handler_->combined_drone_awareness();

    for (unsigned int i = 0; i < maneuver_queue.size(); i++) {

        combined_drone_awareness_t next_awareness;

        if (
            !ProjectExpectedAwarenessSingle(
                maneuver_queue[i], 
                awareness,
                next_awareness
            )
        ) {

            throw std::runtime_error("ManeuverScheduler::ProjectExpectedAwarenessFull(): maneuver queue is invalid");

        }

        awareness = next_awareness;

    }

    return awareness;

}

bool ManeuverScheduler::ProjectExpectedAwarenessSingle(
    const Maneuver & maneuver,
    const combined_drone_awareness_t & awareness_before,
    combined_drone_awareness_t & awareness_after
) const {

    if (maneuver.maneuver_type() == MANEUVER_TYPE_NONE) {

        awareness_after = awareness_before;

        return true;

    }

    if (!maneuverCanExecute(maneuver, awareness_before)) {

        return false;

    }

    // Find maneuver server:
    auto registered_maneuver = registered_maneuvers_.find(maneuver.maneuver_type());
    
    awareness_after = registered_maneuver->second->ExpectedAwarenessAfterExecution(maneuver);

    // switch(maneuver.maneuver_type()) {

        // case maneuver::MANEUVER_TYPE_CABLE_LANDING:
        //     awareness_after.armed = true;
        //     awareness_after.offboard = true;
        //     awareness_after.has_target_cable = true;
        //     awareness_after.drone_location = DRONE_LOCATION_ON_CABLE;
        //     break;

        // case maneuver::MANEUVER_TYPE_CABLE_TAKEOFF:
        //     awareness_after.armed = true;
        //     awareness_after.offboard = true;
        //     awareness_after.has_target_cable = true;
        //     awareness_after.drone_location = DRONE_LOCATION_IN_FLIGHT;
        //     break;

    // }

    return true;

}

bool ManeuverScheduler::CanExecute(const Maneuver & maneuver) const {

    if (maneuverIsExecutingOrPending()) {

        return false;

    }

    return maneuverCanExecute(
        maneuver,
        combined_drone_awareness_handler_->combined_drone_awareness()
    );

}

bool ManeuverScheduler::CanSchedule(const Maneuver & maneuver) const {

    try {
            
        ProjectExpectedAwarenessFull(maneuver);

    } catch (std::runtime_error & e) {

        return false;
    
    }

    return true;

}

bool ManeuverScheduler::RegisterManeuver(
    Maneuver maneuver,
    bool & will_execute_immediately
) {

    std::shared_lock<std::shared_mutex> lck(maneuver_mutex_);

    if (!CanSchedule(maneuver)) {

        return false;

    }

    if (!maneuver_queue_->Push(maneuver)) {

        return false;

    }

    will_execute_immediately = CanExecute(maneuver);

    if (maneuver_execution_timer_->is_canceled()) {

        maneuver_execution_timer_->reset();

    }

    return true;

}

bool ManeuverScheduler::UpdateManeuver(Maneuver maneuver) {

    std::shared_lock<std::shared_mutex> lck(maneuver_mutex_);

    Maneuver old_maneuver = maneuver_queue_->Find(maneuver.uuid());

    if (old_maneuver == Maneuver()) {

        return false;

    }

    rclcpp::Time maneuver_creation_time = old_maneuver.creation_time();

    rclcpp::Time current_time = node_->now();

    if ((current_time - maneuver_creation_time).seconds() > parameters_->GetParameter("maneuver_register_update_timeout_s").as_double()) {

        maneuver.Terminate(false);

        maneuver_queue_->Update(maneuver);

        return false;

    }

    if(!maneuver_queue_->Update(maneuver)) {

        if (maneuver == *current_maneuver_) {

            current_maneuver_ = maneuver;

            return true;

        } else {

            return false;

        }

    } else {

        return true;

    }

}

bool ManeuverScheduler::CancelManeuver(Maneuver maneuver) {

    if (!maneuver.terminated()) {

        std::string msg = "ManeuverScheduler::CancelManeuver(): maneuver was not terminated before canceling with the scheduler.";

        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), msg.c_str());

        throw std::runtime_error(msg);

    }

    std::shared_lock<std::shared_mutex> lck(maneuver_mutex_);

    if (!maneuver_queue_->Update(maneuver)) {

        if (maneuver == *current_maneuver_) {

            current_maneuver_ = maneuver;

            return true;

        } else {

            return false;

        }

    } else {

        return true;

    }
}

iii_drone::control::maneuver::Maneuver ManeuverScheduler::current_maneuver() const {

    return current_maneuver_;

}

bool ManeuverScheduler::maneuverIsExecutingOrPending() const {

    if (!maneuver_queue_->empty()) {

        return true;

    }

    return ((Maneuver)current_maneuver_).maneuver_type() != iii_drone::control::maneuver::MANEUVER_TYPE_NONE;

}

bool ManeuverScheduler::maneuverCanExecute(
    const Maneuver & maneuver,
    const combined_drone_awareness_t & awareness
) const {

    // Find registered maneuver coresponding to the maneuver type
    auto registered_maneuver = registered_maneuvers_.find(maneuver.maneuver_type());

    if (registered_maneuver == registered_maneuvers_.end()) {

        // Log error
        std::string msg = "ManeuverScheduler::maneuverCanExecute(): maneuver type " + std::to_string(maneuver.maneuver_type()) + " not registered.";

        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), msg.c_str());

        return false;

    }

    // Check if the maneuver can execute
    return registered_maneuver->second->CanExecuteManeuver(
        maneuver,
        awareness
    );

    // switch(maneuver.type) {

    //     case maneuver::MANEUVER_TYPE_CABLE_LANDING:
    //         return behavior_state.in_flight() && 
    //             behavior_state.offboard && 
    //             behavior_state.has_target_cable && 
    //             behavior_state.target_cable_position_known;

    //     case maneuver::MANEUVER_TYPE_CABLE_TAKEOFF:
    //         return behavior_state.on_cable() && 
    //             behavior_state.offboard && 
    //             behavior_state.armed && 
    //             behavior_state.has_target_cable && 
    //             behavior_state.target_cable_position_known;

    //     default:
    //         return true;

    // }
}

void ManeuverScheduler::onManeuverCompleted(Maneuver maneuver) {

    std::shared_lock<std::shared_mutex> lck(maneuver_mutex_);

    if (!maneuver.terminated()) {

        std::string msg = "ManeuverScheduler::onManeuverCompleted(): maneuver was not terminated before completing.";
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), msg.c_str());

    }

    current_maneuver_ = maneuver;

}

void ManeuverScheduler::onReferenceCallbackTokenReacquired() {

    if (!reference_callback_token_.master_has_token()) {

        std::string msg = "ManeuverScheduler::onReferenceCallbackTokenReacquired(): master does not have token.";

        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), msg.c_str());

        throw std::runtime_error(msg);

    }

    if (!current_maneuver_->terminated()) {

        std::string msg = "ManeuverScheduler::onReferenceCallbackTokenReacquired(): maneuver is not terminated.";

        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), msg.c_str());

        throw std::runtime_error(msg);

    }

    if (!current_maneuver_->success()) {

        auto registered_maneuver = registered_maneuvers_.find(MANEUVER_TYPE_HOVER);

        std::shared_ptr<HoverManeuverServer> maneuver_server = std::static_pointer_cast<HoverManeuverServer>(registered_maneuver->second);

        reference_callback_token_.resource() = std::bind(
            &HoverManeuverServer::GetReference,
            &(*maneuver_server),
            std::placeholders::_1
        );

    }


    // switch(maneuver_type) {
    //     case MANEUVER_TYPE_CABLE_LANDING: {
    //         auto registered_maneuver = registered_maneuvers_.find(MANEUVER_TYPE_HOVER_ON_CABLE);

    //         std::shared_ptr<HoverOnCableManeuverServer> maneuver_server = std::static_pointer_cast<HoverOnCableManeuverServer>(registered_maneuver->second);

    //         maneuver_server->Update(
    //             parameters_->GetParameter("hover_on_cable_default_z_velocity").as_string(),
    //             parameters_->GetParameter("hover_on_cable_default_yaw_rate").as_string()
    //         );

    //         reference_callback_token_.resource() = std::bind(
    //             &HoverOnCableManeuverServer::GetReference,
    //             &(*maneuver_server),
    //             std::placeholders::_1
    //         );

    //         break;
    //     }
    // }
}

void ManeuverScheduler::onHoveringFail() {

    cancelAllPendingManeuvers();

}

void ManeuverScheduler::maneuverExecutionTimerCallback() {

    progressScheduler();

    fetchNextReferenceAndPublish();

}

void ManeuverScheduler::progressScheduler() {

    static bool waiting_for_maneuver_to_start = false;
    static std::string waiting_for_maneuver_to_start_action_name;

    std::unique_lock<std::shared_mutex> lck(maneuver_mutex_);

    auto on_no_maneuver = [this](Maneuver previous_maneuver) {

        if (!previous_maneuver.success()) {

            no_maneuver_idle_cnt_ = parameters_->GetParameter("no_maneuver_idle_cnt_s").as_double() * 1000 / parameters_->GetParameter("maneuver_execution_period_ms").as_int();

        } else {

            switch (previous_maneuver.maneuver_type()) {

                case MANEUVER_TYPE_HOVER: {
                    hover_maneuver_params_t maneuver_params(previous_maneuver.maneuver_params());

                    no_maneuver_idle_cnt_ = maneuver_params.duration_s * 1000 / parameters_->GetParameter("maneuver_execution_period_ms").as_int();

                    break;
                }
                case MANEUVER_TYPE_HOVER_BY_OBJECT: {
                    hover_by_object_maneuver_params_t maneuver_params(previous_maneuver.maneuver_params());

                    no_maneuver_idle_cnt_ = maneuver_params.duration_s * 1000 / parameters_->GetParameter("maneuver_execution_period_ms").as_int();

                    break;
                }
                case MANEUVER_TYPE_HOVER_ON_CABLE: {
                    hover_on_cable_maneuver_params_t maneuver_params(previous_maneuver.maneuver_params());

                    no_maneuver_idle_cnt_ = maneuver_params.duration_s * 1000 / parameters_->GetParameter("maneuver_execution_period_ms").as_int();

                    break;
                }
                case MANEUVER_TYPE_NONE:
                    if (*no_maneuver_idle_cnt_ >= 0)
                        (*no_maneuver_idle_cnt_)--;
                    break;

                default:
                    no_maneuver_idle_cnt_ = parameters_->GetParameter("no_maneuver_idle_cnt_s").as_double() * 1000 / parameters_->GetParameter("maneuver_execution_period_ms").as_int();
                    break;
            
            }

        }

        if (*no_maneuver_idle_cnt_ < 0) {

            maneuver_execution_timer_->cancel();

        }
    };

    auto on_failure = [this, &on_no_maneuver](Maneuver previous_maneuver) {

        previous_maneuver.Terminate(false);

        cancelAllPendingManeuvers();

        reference_callback_token_.DenyAll();

        on_no_maneuver(previous_maneuver);

    };

    // Check if a maneuver is executing
    if (maneuverIsExecutingOrPending()) {

        // Check if the current maneuver is completed
        if(current_maneuver_->terminated()) {

            if (!reference_callback_token_.master_has_token()) {

                std::string msg = "ManeuverScheduler::progressScheduler(): master does not have token.";

                RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), msg.c_str());

                throw std::runtime_error(msg);

            }

            // Check if the maneuver was successful
            if (current_maneuver_->success()) {

                // Check if there are more maneuvers in the queue
                Maneuver maneuver;
                if (maneuver_queue_->Pop(maneuver)) {

                    // Progress to the next maneuver
                    current_maneuver_ = maneuver;

                    // Start the maneuver execution:
                    current_maneuver_->Start();

                    // Find the maneuver server:
                    auto registered_maneuver = registered_maneuvers_.find(maneuver.maneuver_type());

                    // Get the action name:
                    waiting_for_maneuver_to_start_action_name = registered_maneuver->second->action_name();

                    // Set waiting for start:
                    waiting_for_maneuver_to_start = true;

                    // Reset no_maneuver_idle_cnt:
                    *no_maneuver_idle_cnt_ = -1;


                } else {

                    on_no_maneuver(current_maneuver_);

                    current_maneuver_ = Maneuver();

                }

            } else {

                on_failure(current_maneuver_);

            }

        } else {
            // Current maneuver has not terminated.

            if (waiting_for_maneuver_to_start) {

                // Check that the maneuver has a goal handle:
                if (current_maneuver_->goal_handle() == nullptr) {

                    // Check that the update timeout has elapsed:
                    rclcpp::Time maneuver_creation_time = current_maneuver_->creation_time();
                    rclcpp::Time current_time = node_->now();

                    if ((current_time - maneuver_creation_time).seconds() > parameters_->GetParameter("maneuver_register_update_timeout_s").as_double()) {

                        on_failure(current_maneuver_);

                    }

                    return;

                }

                if (reference_callback_token_.has_requested_token(waiting_for_maneuver_to_start_action_name)) {

                    // Give the token:
                    reference_callback_token_.Give(waiting_for_maneuver_to_start_action_name);

                    // Reset the flag:
                    waiting_for_maneuver_to_start = false;

                } else {

                    rclcpp::Time maneuver_start_time = current_maneuver_->start_time();
                    rclcpp::Time current_time = node_->now();

                    if ((current_time - maneuver_start_time).seconds() > parameters_->GetParameter("maneuver_start_timeout_s").as_double()) {

                        on_failure(current_maneuver_);

                    }
                }
            }
        }

    } else {

        if (current_maneuver_->maneuver_type() != MANEUVER_TYPE_NONE) {

            std::string msg = "ManeuverScheduler::progressScheduler(): maneuver is not executing or pending, but current maneuver is not MANEUVER_TYPE_NONE.";

            RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), msg.c_str());

            throw std::runtime_error(msg);

        }

        on_no_maneuver(current_maneuver_);

    }
}

void ManeuverScheduler::fetchNextReferenceAndPublish() {

    Reference ref = (*reference_callback_)(combined_drone_awareness_handler_->GetState());

    ReferenceAdapter ref_adapter(ref);

    reference_publisher_->publish(ref_adapter.ToMsg());

}

void ManeuverScheduler::cancelAllPendingManeuvers() {

    std::unique_lock<std::shared_mutex> lck(maneuver_mutex_);

    maneuver_queue_->Clear();

    current_maneuver_ = Maneuver();

}