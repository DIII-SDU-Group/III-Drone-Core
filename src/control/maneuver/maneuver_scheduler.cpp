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
    reference_callback_(std::make_shared<Atomic<std::function<Reference(const State &)>>>()),
    reference_callback_token_(
        reference_callback_,
        std::bind(
            &ManeuverScheduler::onReferenceCallbackTokenReacquired,
            this
        ),
        node_->get_logger()
    ) {

    // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::ManeuverScheduler()");

    // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::ManeuverScheduler(): Initializing reference callback");

    *reference_callback_ = std::bind(
        &ManeuverScheduler::getPassthroughReference,
        this,
        std::placeholders::_1
    );

    reference_publisher_ = node_->create_publisher<iii_drone_interfaces::msg::Reference>(
        "reference",
        10 // Fix QoS
    );

    // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::ManeuverScheduler(): Initializing maneuver queue");

    maneuver_queue_ = std::make_unique<ManeuverQueue>(parameters_->GetParameter("maneuver_queue_size").as_int());
    
    current_maneuver_ = Maneuver();

    // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::ManeuverScheduler(): Initializing maneuver publisher and timer");

    current_maneuver_publisher_ = node_->create_publisher<iii_drone_interfaces::msg::Maneuver>(
        "current_maneuver",
        10 // Fix QoS
    );

    maneuver_queue_publisher_ = node_->create_publisher<iii_drone_interfaces::msg::ManeuverQueue>(
        "maneuver_queue",
        10 // Fix QoS
    );

    maneuver_publish_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(parameters_->GetParameter("maneuver_publish_period_ms").as_int()),
        [this]() -> void {

            Maneuver current_maneuver;
            std::vector<Maneuver> maneuver_queue;

            {
                std::shared_lock<std::shared_mutex> lck(maneuver_mutex_);

                current_maneuver = current_maneuver_;
                maneuver_queue = maneuver_queue_->vector();
            }

            iii_drone_interfaces::msg::Maneuver current_maneuver_msg = ManeuverAdapter(current_maneuver).ToMsg();

            iii_drone_interfaces::msg::ManeuverQueue maneuver_queue_msg;

            maneuver_queue_msg.current_maneuver = current_maneuver_msg;

            for (Maneuver maneuver : maneuver_queue) {

                maneuver_queue_msg.scheduled_maneuvers.push_back(ManeuverAdapter(maneuver).ToMsg());

            }

            current_maneuver_publisher_->publish(current_maneuver_msg);
            maneuver_queue_publisher_->publish(maneuver_queue_msg);
            
        }
    );

    // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::ManeuverScheduler(): Initializing maneuver execution timer");

    maneuver_execution_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(parameters_->GetParameter("maneuver_execution_period_ms").as_int()),
        std::bind(
            &ManeuverScheduler::maneuverExecutionTimerCallback,
            this
        ),
        maneuver_execution_callback_group_
    );

    maneuver_execution_timer_->cancel();

    // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::ManeuverScheduler(): Initializing get reference service");

    get_reference_service_ = node_->create_service<iii_drone_interfaces::srv::GetReference>(
        "get_reference",
        std::bind(
            &ManeuverScheduler::getReferenceServiceCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::ManeuverScheduler(): Finished initializing");

}

void ManeuverScheduler::RegisterManeuverServer(
    maneuver_type_t maneuver_type,
    ManeuverServer::SharedPtr maneuver_server
) {

    // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::RegisterManeuverServer(): registering maneuver type %d", maneuver_type);
    
    // Check if the maneuver type is already registered
    if (registered_maneuvers_.find(maneuver_type) != registered_maneuvers_.end()) {

        std::string msg = "ManeuverScheduler::RegisterManeuverServer(): maneuver type " + std::to_string(maneuver_type) + " already registered, but was attempted registered again.";

        RCLCPP_ERROR(node_->get_logger(), msg.c_str());

        return;

    }

    // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::RegisterManeuverServer(): Inserting maneuver server");

    registered_maneuvers_.insert(
        std::pair<maneuver_type_t, ManeuverServer::SharedPtr>(
            maneuver_type,
            maneuver_server
        )
    );

    // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::RegisterManeuverServer(): Starting maneuver server");

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

        // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::RegisterManeuverServer(): Registering on fail callback for hover by object");

        std::static_pointer_cast<HoverByObjectManeuverServer>(maneuver_server)->RegisterOnFailCallback(
            std::bind(
                &ManeuverScheduler::onHoveringFail,
                this
            )
        );

    } else if (maneuver_type == MANEUVER_TYPE_HOVER_ON_CABLE) {

        // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::RegisterManeuverServer(): Registering on fail callback for hover on cable");

        std::static_pointer_cast<HoverOnCableManeuverServer>(maneuver_server)->RegisterOnFailCallback(
            std::bind(
                &ManeuverScheduler::onHoveringFail,
                this
            )
        );

    }

    // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::RegisterManeuverServer(): Finished registering maneuver type %d", maneuver_type);

}

bool ManeuverScheduler::ProjectExpectedAwarenessFull(
    const Maneuver & maneuver,
    combined_drone_awareness_t & awareness
) const {

    // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::ProjectExpectedAwarenessFull(): projecting expected awareness");

    std::vector<Maneuver> maneuver_queue = maneuver_queue_->vector();

    Maneuver current_maneuver = current_maneuver_.Load();

    // RCLCPP_DEBUG(
    //     node_->get_logger(), 
    //     "ManeuverScheduler::ProjectExpectedAwarenessFull(): Current manuever: type = %d, terminated = %s, success = %s",
    //     current_maneuver.maneuver_type(),
    //     current_maneuver.terminated() ? "true" : "false",
    //     current_maneuver.success() ? "true" : "false"
    // );

    if (maneuver_queue.size() == 0 || maneuver_queue[0] != current_maneuver) {

        maneuver_queue.insert(
            maneuver_queue.begin(), 
            current_maneuver
        );

    }

    maneuver_queue.push_back(maneuver);

    awareness = combined_drone_awareness_handler_->combined_drone_awareness();

    for (unsigned int i = 0; i < maneuver_queue.size(); i++) {

        combined_drone_awareness_t next_awareness;

        if (
            !ProjectExpectedAwarenessSingle(
                maneuver_queue[i], 
                awareness,
                next_awareness
            )
        ) {

            // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::ProjectExpectedAwarenessFull(): maneuver type %d cannot execute", maneuver_queue[i].maneuver_type());

            return false;

        }

        awareness = next_awareness;

    }

    // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::ProjectExpectedAwarenessFull(): Successfully projected expected awareness");

    return true;

}

bool ManeuverScheduler::ProjectExpectedAwarenessSingle(
    const Maneuver & maneuver,
    const combined_drone_awareness_t & awareness_before,
    combined_drone_awareness_t & awareness_after
) const {

    // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::ProjectExpectedAwarenessSingle(): projecting expected awareness for maneuver type %d", maneuver.maneuver_type());

    if (maneuver.maneuver_type() == MANEUVER_TYPE_NONE) {

        // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::ProjectExpectedAwarenessSingle(): maneuver type is NONE, returning awareness unaltered awareness");

        awareness_after = awareness_before;

        return true;

    }

    if (!maneuverCanExecute(maneuver, awareness_before)) {

        return false;

    }

    // Find maneuver server:
    auto registered_maneuver = registered_maneuvers_.find(maneuver.maneuver_type());
    
    awareness_after = registered_maneuver->second->ExpectedAwarenessAfterExecution(maneuver);

    return true;

}

bool ManeuverScheduler::CanExecute(const Maneuver & maneuver) const {

    // // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::CanExecute(): checking if maneuver can execute");

    if (maneuverIsExecutingOrPending()) {

        // // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::CanExecute(): A maneuver is executing or pending");

        return false;

    }

    //// RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::CanExecute(): No maneuver is executing or pending");

    bool maneuver_can_execute = maneuverCanExecute(
        maneuver,
        combined_drone_awareness_handler_->combined_drone_awareness()
    );

    if (!maneuver_can_execute) {

        //// RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::CanExecute(): maneuver cannot execute");

        return false;

    }

    //// RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::CanExecute(): maneuver can execute");

    return true;

}

bool ManeuverScheduler::CanSchedule(const Maneuver & maneuver) const {

    // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::CanSchedule(): checking if maneuver can be scheduled");

    combined_drone_awareness_t awareness;

    if(ProjectExpectedAwarenessFull(
        maneuver,
        awareness
    )) {

        // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::CanSchedule(): maneuver can be scheduled");

        return true;

    }

    // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::CanSchedule(): maneuver cannot be scheduled");

    return false;

}

bool ManeuverScheduler::RegisterManeuver(
    Maneuver maneuver,
    bool & will_execute_immediately
) {

    // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::RegisterManeuver(): registering maneuver");

    std::shared_lock<std::shared_mutex> lck(maneuver_mutex_);

    if (!CanSchedule(maneuver)) {

        // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::RegisterManeuver(): maneuver cannot be scheduled, returning false");

        return false;

    }

    will_execute_immediately = CanExecute(maneuver);

    if (!maneuver_queue_->Push(maneuver)) {

        // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::RegisterManeuver(): maneuver could not be pushed to queue, returning false");

        return false;

    }

    if (maneuver_execution_timer_->is_canceled()) {

        // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::RegisterManeuver(): maneuver execution timer is canceled, starting timer");

        maneuver_execution_timer_->reset();

    }

    // RCLCPP_DEBUG(
    //     node_->get_logger(), 
    //     "ManeuverScheduler::RegisterManeuver(): maneuver registered, returning true with will_execute_immediately = %s", 
    //     will_execute_immediately ? "true" : "false"
    // );

    return true;

}

bool ManeuverScheduler::UpdateManeuver(Maneuver maneuver) {

    // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::UpdateManeuver(): Updating maneuver type %d", maneuver.maneuver_type());

    if (!&maneuver_mutex_) {

        RCLCPP_FATAL(node_->get_logger(), "ManeuverScheduler::UpdateManeuver(): maneuver_mutex_ is null");
        throw std::runtime_error("ManeuverScheduler::UpdateManeuver(): maneuver_mutex_ is null");

    } else {

        // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::UpdateManeuver(): maneuver_mutex_ is not null");

    }

    auto update_timeout_elapsed = [this](Maneuver & maneuver) -> bool {

        rclcpp::Time maneuver_creation_time = maneuver.creation_time();

        rclcpp::Time current_time = rclcpp::Clock().now();

        return (current_time - maneuver_creation_time).seconds() > parameters_->GetParameter("maneuver_register_update_timeout_s").as_double();

    };

    std::shared_lock<std::shared_mutex> lck(maneuver_mutex_);

    if (maneuver == *current_maneuver_) {

        if (update_timeout_elapsed(maneuver)) {

            // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::UpdateManeuver(): Manuever is current maneuver, but maneuver update timeout elapsed, terminating current maneuver and returning false");

            maneuver.Terminate(false);

            current_maneuver_ = maneuver;

            return false;

        }

        // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::UpdateManeuver(): Manuever is current maneuver, updating current maneuver, returning true");

        current_maneuver_ = maneuver;

        return true;

    }

    Maneuver old_maneuver = maneuver_queue_->Find(maneuver.uuid());

    if (old_maneuver == Maneuver()) {

        // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::UpdateManeuver(): Old maneuver not found, returning false");

        return false;

    }

    if (update_timeout_elapsed(old_maneuver)) {

        // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::UpdateManeuver(): Old maneuver found in queue, but update timeout elapsed, terminating old maneuver and returning false");

        old_maneuver.Terminate(false);

        return false;

    }

    if(!maneuver_queue_->Update(maneuver)) {

        std::string msg = "ManeuverScheduler::UpdateManeuver(): maneuver could not be updated in queue, but was already checked previously. This should not happen.";

        RCLCPP_FATAL(node_->get_logger(), msg.c_str());

        throw std::runtime_error(msg);

    } else {

        // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::UpdateManeuver(): Updated maneuver in queue, returning true");

        return true;

    }

}

bool ManeuverScheduler::CancelManeuver(Maneuver maneuver) {

    if (!maneuver.terminated()) {

        std::string msg = "ManeuverScheduler::CancelManeuver(): maneuver was not terminated before canceling with the scheduler.";

        RCLCPP_FATAL(node_->get_logger(), msg.c_str());

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

    // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::maneuverCanExecute(): checking if maneuver can execute with given awareness");

    // Find registered maneuver coresponding to the maneuver type
    auto registered_maneuver = registered_maneuvers_.find(maneuver.maneuver_type());

    if (registered_maneuver == registered_maneuvers_.end()) {

        // Log error
        std::string msg = "ManeuverScheduler::maneuverCanExecute(): maneuver type " + std::to_string(maneuver.maneuver_type()) + " not registered.";

        RCLCPP_ERROR(node_->get_logger(), msg.c_str());

        return false;

    }

    // Check if the maneuver can execute
    return registered_maneuver->second->CanExecuteManeuver(
        maneuver,
        awareness
    );

}

void ManeuverScheduler::onManeuverCompleted(Maneuver maneuver) {

    std::shared_lock<std::shared_mutex> lck(maneuver_mutex_);

    if (!maneuver.terminated()) {

        std::string msg = "ManeuverScheduler::onManeuverCompleted(): maneuver was not terminated before completing.";
        RCLCPP_ERROR(node_->get_logger(), msg.c_str());

    }

    current_maneuver_ = maneuver;

}

void ManeuverScheduler::onReferenceCallbackTokenReacquired() {

    if (!reference_callback_token_.master_has_token()) {

        std::string msg = "ManeuverScheduler::onReferenceCallbackTokenReacquired(): master does not have token.";

        RCLCPP_FATAL(node_->get_logger(), msg.c_str());

        throw std::runtime_error(msg);

    }

    if (!current_maneuver_->terminated()) {

        std::string msg = "ManeuverScheduler::onReferenceCallbackTokenReacquired(): maneuver is not terminated.";

        RCLCPP_FATAL(node_->get_logger(), msg.c_str());

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

}

Reference ManeuverScheduler::getPassthroughReference(const State & state) const {

    //// RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::getPassthroughReference(): Getting passthrough reference");

    return Reference(state);

}

void ManeuverScheduler::onHoveringFail() {

    std::unique_lock<std::shared_mutex> lck(maneuver_mutex_);

    cancelAllPendingManeuvers();

}

void ManeuverScheduler::getReferenceServiceCallback(
    const std::shared_ptr<iii_drone_interfaces::srv::GetReference::Request>,
    std::shared_ptr<iii_drone_interfaces::srv::GetReference::Response> response
) {

    //// RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::getReferenceServiceCallback(): Received request");

    iii_drone_interfaces::msg::Reference ref_msg = fetchNextReferenceAndPublish();

    response->reference = ref_msg;
    bool is_valid = maneuverIsExecutingOrPending();

    if (is_valid) {

        //// RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::getReferenceServiceCallback(): Maneuver is executing or pending");

    } else {

        //// RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::getReferenceServiceCallback(): Maneuver is not executing or pending");

    }

    response->is_valid = is_valid;

}

void ManeuverScheduler::maneuverExecutionTimerCallback() {

    progressScheduler();

}

void ManeuverScheduler::progressScheduler() {

    static bool waiting_for_maneuver_to_start = false;
    static std::string waiting_for_maneuver_to_start_action_name;

    std::unique_lock<std::shared_mutex> lck(maneuver_mutex_);

    auto on_no_maneuver = [this](Maneuver previous_maneuver) {

        if (!previous_maneuver.success()) {

            no_maneuver_idle_cnt_ = parameters_->GetParameter("no_maneuver_idle_cnt_s").as_double() * 1000 / parameters_->GetParameter("maneuver_execution_period_ms").as_int();

        } else {

            auto set_default_no_maneuver_idle_cnt = [this]() {

                no_maneuver_idle_cnt_ = parameters_->GetParameter("no_maneuver_idle_cnt_s").as_double() * 1000 / parameters_->GetParameter("maneuver_execution_period_ms").as_int();

            };

            switch (previous_maneuver.maneuver_type()) {

                case MANEUVER_TYPE_HOVER: {
                    hover_maneuver_params_t maneuver_params(previous_maneuver.maneuver_params());

                    if (maneuver_params.sustain_action) {

                        set_default_no_maneuver_idle_cnt();

                        break;

                    }

                    no_maneuver_idle_cnt_ = maneuver_params.duration_s * 1000 / parameters_->GetParameter("maneuver_execution_period_ms").as_int();

                    break;
                }
                case MANEUVER_TYPE_HOVER_BY_OBJECT: {
                    hover_by_object_maneuver_params_t maneuver_params(previous_maneuver.maneuver_params());

                    if (maneuver_params.sustain_action) {

                        set_default_no_maneuver_idle_cnt();

                        break;

                    }

                    no_maneuver_idle_cnt_ = maneuver_params.duration_s * 1000 / parameters_->GetParameter("maneuver_execution_period_ms").as_int();

                    break;
                }
                case MANEUVER_TYPE_HOVER_ON_CABLE: {
                    hover_on_cable_maneuver_params_t maneuver_params(previous_maneuver.maneuver_params());

                    if (maneuver_params.sustain_action) {

                        set_default_no_maneuver_idle_cnt();

                        break;

                    }

                    no_maneuver_idle_cnt_ = maneuver_params.duration_s * 1000 / parameters_->GetParameter("maneuver_execution_period_ms").as_int();

                    break;
                }
                case MANEUVER_TYPE_NONE:
                    if (*no_maneuver_idle_cnt_ >= 0)
                        (*no_maneuver_idle_cnt_)--;
                    break;

                default:
                    set_default_no_maneuver_idle_cnt();
                    break;
            
            }

        }

        if (*no_maneuver_idle_cnt_ < 0) {

            *reference_callback_ = std::bind(
                &ManeuverScheduler::getPassthroughReference,
                this,
                std::placeholders::_1
            );

            maneuver_execution_timer_->cancel();

        }
    };

    auto on_failure = [this, &on_no_maneuver](Maneuver previous_maneuver) {

        // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::progressScheduler()::on_failure(): maneuver failed");

        previous_maneuver.Terminate(false);

        // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::progressScheduler()::on_failure(): maneuver terminated");

        cancelAllPendingManeuvers();

        // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::progressScheduler()::on_failure(): all pending maneuvers canceled, denying all tokens");

        reference_callback_token_.DenyAll();

        // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::progressScheduler()::on_failure(): all tokens denied, calling on_no_maneuver()");

        on_no_maneuver(previous_maneuver);

    };

    // Check if a maneuver is executing
    if (maneuverIsExecutingOrPending()) {

        // Check if the current maneuver is completed
        if(current_maneuver_->terminated()) {

            // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::progressScheduler(): maneuver type %d is terminated", current_maneuver_->maneuver_type());

            if (!reference_callback_token_.master_has_token()) {

                std::string msg = "ManeuverScheduler::progressScheduler(): master does not have token.";

                RCLCPP_FATAL(node_->get_logger(), msg.c_str());

                throw std::runtime_error(msg);

            }

            // Check if the maneuver was successful
            if (current_maneuver_->success()) {

                // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::progressScheduler(): maneuver was successful, checking maneuver queue");

                // Check if there are more maneuvers in the queue
                Maneuver maneuver;
                if (maneuver_queue_->Pop(maneuver)) {

                    // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::progressScheduler(): maneuver type %d popped from queue, setting current maneuver", maneuver.maneuver_type());

                    // Progress to the next maneuver
                    current_maneuver_ = maneuver;

                } else {

                    // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::progressScheduler(): maneuver queue is empty, calling on_no_maneuver()");

                    on_no_maneuver(current_maneuver_);

                    current_maneuver_ = Maneuver();

                }

            } else {

                // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::progressScheduler(): maneuver was not successful");

                on_failure(current_maneuver_);

            }

        } else {
            // Current maneuver has not terminated.
            // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::progressScheduler(): maneuver type %d is not terminated", current_maneuver_->maneuver_type());

            // Check that the maneuver has a goal handle:
            if (current_maneuver_->goal_handle() == nullptr) {

                // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::progressScheduler(): Maneuver does not have a goal handle");

                // Check that the update timeout has elapsed:
                rclcpp::Time maneuver_creation_time = current_maneuver_->creation_time();
                rclcpp::Time current_time = rclcpp::Clock().now();

                if ((current_time - maneuver_creation_time).seconds() > parameters_->GetParameter("maneuver_register_update_timeout_s").as_double()) {

                    // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::progressScheduler(): Maneuver update timeout elapsed, calling on_failure()");

                    on_failure(current_maneuver_);

                }

                return;

            }

            if (!current_maneuver_->started()) {

                // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::progressScheduler(): Starting maneuver");

                // Start the maneuver execution:
                current_maneuver_->Start();

                // Find the maneuver server:
                auto registered_maneuver = registered_maneuvers_.find(current_maneuver_->maneuver_type());

                // Get the action name:
                waiting_for_maneuver_to_start_action_name = registered_maneuver->second->action_name();

                // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::progressScheduler(): Waiting for maneuver server %s to start", waiting_for_maneuver_to_start_action_name.c_str());

                // Set waiting for start:
                waiting_for_maneuver_to_start = true;

                // Reset no_maneuver_idle_cnt:
                *no_maneuver_idle_cnt_ = -1;

            }

            if (waiting_for_maneuver_to_start) {

                // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::progressScheduler(): Waiting for maneuver server %s to start", waiting_for_maneuver_to_start_action_name.c_str());

                if (reference_callback_token_.has_requested_token(waiting_for_maneuver_to_start_action_name)) {

                    // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::progressScheduler(): Giving token to maneuver server %s", waiting_for_maneuver_to_start_action_name.c_str());

                    // Give the token:
                    reference_callback_token_.Give(waiting_for_maneuver_to_start_action_name);

                    // Reset the flag:
                    waiting_for_maneuver_to_start = false;

                } else {

                    rclcpp::Time maneuver_start_time = current_maneuver_->start_time();
                    rclcpp::Time current_time = rclcpp::Clock().now();

                    if ((current_time - maneuver_start_time).seconds() > parameters_->GetParameter("maneuver_start_timeout_s").as_double()) {

                        // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::progressScheduler(): Maneuver start timeout elapsed, calling on_failure()");

                        on_failure(current_maneuver_);

                    }
                }
            }
        }

    } else {

        if (current_maneuver_->maneuver_type() != MANEUVER_TYPE_NONE) {

            std::string msg = "ManeuverScheduler::progressScheduler(): maneuver is not executing or pending, but current maneuver is not MANEUVER_TYPE_NONE.";

            RCLCPP_FATAL(node_->get_logger(), msg.c_str());

            throw std::runtime_error(msg);

        }

        on_no_maneuver(current_maneuver_);

    }
}

iii_drone_interfaces::msg::Reference ManeuverScheduler::fetchNextReferenceAndPublish() {

    //// RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::fetchNextReferenceAndPublish(): Calling stored reference callback");

    Reference ref = (**reference_callback_)(combined_drone_awareness_handler_->GetState());

    ReferenceAdapter ref_adapter(ref);

    iii_drone_interfaces::msg::Reference ref_msg = ref_adapter.ToMsg();

    reference_publisher_->publish(ref_msg);

    return ref_msg;

}

void ManeuverScheduler::cancelAllPendingManeuvers() {

    // RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::cancelAllPendingManeuvers(): clearing maneuver queue and resetting current maneuver");

    maneuver_queue_->Clear();

    current_maneuver_ = Maneuver();

}