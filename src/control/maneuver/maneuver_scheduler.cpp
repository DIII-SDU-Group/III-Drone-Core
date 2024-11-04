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
    rclcpp_lifecycle::LifecycleNode *node,
    const CombinedDroneAwarenessHandler::SharedPtr combined_drone_awareness_handler,
    const iii_drone::configuration::ParameterBundle::SharedPtr parameters,
    rclcpp::CallbackGroup::SharedPtr maneuver_execution_callback_group
) : node_(node),
    combined_drone_awareness_handler_(combined_drone_awareness_handler),
    parameters_(parameters),
    maneuver_execution_callback_group_(maneuver_execution_callback_group),
    reference_callback_struct_(std::make_shared<ReferenceCallbackStruct>()),
    reference_callback_token_(
        reference_callback_struct_,
        std::bind(
            &ManeuverScheduler::onReferenceCallbackTokenReacquired,
            this
        ),
        node_->get_logger()
    ) {

    reference_callback_struct_->set(
        std::bind(
            &ManeuverScheduler::getPassthroughReference,
            this,
            std::placeholders::_1
        ),
        "passthrough"
    );

    reference_publisher_ = node_->create_publisher<iii_drone_interfaces::msg::Reference>(
        "reference",
        10 // Fix QoS
    );

    current_maneuver_publisher_ = node_->create_publisher<iii_drone_interfaces::msg::Maneuver>(
        "current_maneuver",
        10 // Fix QoS
    );

    maneuver_queue_publisher_ = node_->create_publisher<iii_drone_interfaces::msg::ManeuverQueue>(
        "maneuver_queue",
        10 // Fix QoS
    );

    reference_callback_provider_publisher_ = node_->create_publisher<iii_drone_interfaces::msg::StringStamped>(
        "reference_callback_provider",
        10 // Fix QoS
    );

}

ManeuverScheduler::~ManeuverScheduler() {

    RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::~ManeuverScheduler()");

    if (is_started_)
        Stop();

}

void ManeuverScheduler::Start() {

    if (is_started_) {

        std::string msg = "ManeuverScheduler::Start(): maneuver scheduler is already started.";

        RCLCPP_ERROR(node_->get_logger(), msg.c_str());

        return;

    }

    maneuver_queue_ = std::make_unique<ManeuverQueue>(parameters_->GetParameter("maneuver_queue_size").as_int());
    
    current_maneuver_ = Maneuver();

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

    maneuver_execution_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(parameters_->GetParameter("maneuver_execution_period_ms").as_int()),
        std::bind(
            &ManeuverScheduler::maneuverExecutionTimerCallback,
            this
        ),
        maneuver_execution_callback_group_
    );

    maneuver_execution_timer_->cancel();

    // reference_callback_provider_publish_timer_ = node_->create_wall_timer(
    //     std::chrono::milliseconds(parameters_->GetParameter("reference_callback_provider_publish_period_ms").as_int()),
    //     [this]() -> void {

    //         std_msgs::msg::String msg;

    //         msg.data = reference_callback_struct_->reference_provider_name;

    //         reference_callback_provider_publisher_->publish(msg);

    //     }
    // );

    get_reference_service_ = node_->create_service<iii_drone_interfaces::srv::GetReference>(
        "get_reference",
        std::bind(
            &ManeuverScheduler::getReferenceServiceCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    is_started_ = true;

}

void ManeuverScheduler::Stop() {

    if (!is_started_) {

        std::string msg = "ManeuverScheduler::Stop(): maneuver scheduler is not started.";

        RCLCPP_ERROR(node_->get_logger(), msg.c_str());

        return;

    }

    is_started_ = false;

    cancelAllPendingManeuvers();

    maneuver_execution_timer_->cancel();
    maneuver_execution_timer_.reset();
    maneuver_execution_timer_ = nullptr;

    get_reference_service_->clear_on_new_request_callback();
    get_reference_service_.reset();
    get_reference_service_ = nullptr;

    maneuver_publish_timer_->cancel();
    maneuver_publish_timer_.reset();
    maneuver_publish_timer_ = nullptr;

    // reference_callback_provider_publish_timer_->cancel();
    // reference_callback_provider_publish_timer_.reset();
    // reference_callback_provider_publish_timer_ = nullptr;

    maneuver_queue_.reset();
    maneuver_queue_ = nullptr;

    // Loop through manuever servers, remove all
    for (auto registered_maneuver : registered_maneuvers_) {

        UnregisterManeuverServer(registered_maneuver.first);

    }

}

void ManeuverScheduler::RegisterManeuverServer(
    maneuver_type_t maneuver_type,
    ManeuverServer::SharedPtr maneuver_server
) {

    if (!is_started_) {

        std::string msg = "ManeuverScheduler::RegisterManeuverServer(): maneuver scheduler is not started.";

        RCLCPP_ERROR(node_->get_logger(), msg.c_str());

        return;

    }

    // Check if the maneuver type is already registered
    if (registered_maneuvers_.find(maneuver_type) != registered_maneuvers_.end()) {

        std::string msg = "ManeuverScheduler::RegisterManeuverServer(): maneuver type " + std::to_string(maneuver_type) + " already registered, but was attempted registered again.";

        RCLCPP_ERROR(node_->get_logger(), msg.c_str());

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

void ManeuverScheduler::UnregisterManeuverServer(maneuver_type_t maneuver_type) {

    if (!is_started_) {

        std::string msg = "ManeuverScheduler::UnregisterManeuverServer(): maneuver scheduler is not started.";

        RCLCPP_ERROR(node_->get_logger(), msg.c_str());

        return;

    }

    auto registered_maneuver = registered_maneuvers_.find(maneuver_type);

    if (registered_maneuver == registered_maneuvers_.end()) {

        std::string msg = "ManeuverScheduler::UnregisterManeuverServer(): maneuver type " + std::to_string(maneuver_type) + " not registered.";

        RCLCPP_ERROR(node_->get_logger(), msg.c_str());

        return;

    }

    registered_maneuver->second->Stop();

    registered_maneuvers_.erase(registered_maneuver);

}

bool ManeuverScheduler::ProjectExpectedAwarenessFull(
    const Maneuver & maneuver,
    CombinedDroneAwarenessAdapter & awareness
) const {

    if (!is_started_) {

        std::string msg = "ManeuverScheduler::ProjectExpectedAwarenessFull(): maneuver scheduler is not started.";

        RCLCPP_ERROR(node_->get_logger(), msg.c_str());

        return false;

    }

    std::vector<Maneuver> maneuver_queue = maneuver_queue_->vector();

    Maneuver current_maneuver = current_maneuver_.Load();

    if (maneuver_queue.size() == 0 || maneuver_queue[0] != current_maneuver) {

        maneuver_queue.insert(
            maneuver_queue.begin(), 
            current_maneuver
        );

    }

    maneuver_queue.push_back(maneuver);

    awareness = combined_drone_awareness_handler_->adapter();

    for (unsigned int i = 0; i < maneuver_queue.size(); i++) {

        CombinedDroneAwarenessAdapter next_awareness;

        if (
            !ProjectExpectedAwarenessSingle(
                maneuver_queue[i], 
                awareness,
                next_awareness
            )
        ) {

            return false;

        }

        awareness = next_awareness;

    }

    return true;

}

bool ManeuverScheduler::ProjectExpectedAwarenessSingle(
    const Maneuver & maneuver,
    const CombinedDroneAwarenessAdapter & awareness_before,
    CombinedDroneAwarenessAdapter & awareness_after
) const {

    if (!is_started_) {

        std::string msg = "ManeuverScheduler::ProjectExpectedAwarenessSingle(): maneuver scheduler is not started.";

        RCLCPP_ERROR(node_->get_logger(), msg.c_str());

        return false;

    }

    if (maneuver.maneuver_type() == MANEUVER_TYPE_NONE) {

        awareness_after = awareness_before;

        return true;

    }

    if (maneuver.terminated()) {

        if (!maneuver.success()) {

            return false;

        }

    } else if (!maneuverCanExecute(maneuver, awareness_before)) {

        return false;

    }

    // Find maneuver server:
    auto registered_maneuver = registered_maneuvers_.find(maneuver.maneuver_type());
    
    try {

        awareness_after = registered_maneuver->second->ExpectedAwarenessAfterExecution(maneuver);

    } catch (const std::exception & e) {

        RCLCPP_ERROR(node_->get_logger(), e.what());

        return false;

    }

    return true;

}

bool ManeuverScheduler::CanExecute(const Maneuver & maneuver) const {

    if (!is_started_) {

        std::string msg = "ManeuverScheduler::CanExecute(): maneuver scheduler is not started.";

        RCLCPP_ERROR(node_->get_logger(), msg.c_str());

        return false;

    }

    if (maneuverIsExecutingOrPending()) {

        return false;

    }

    bool maneuver_can_execute = maneuverCanExecute(
        maneuver,
        combined_drone_awareness_handler_->adapter()
    );

    if (!maneuver_can_execute) {

        return false;

    }

    return true;

}

bool ManeuverScheduler::CanSchedule(const Maneuver & maneuver) const {

    RCLCPP_DEBUG(
        node_->get_logger(),
        "ManeuverScheduler::CanSchedule(): maneuver type %d",
        maneuver.maneuver_type()
    );

    if (!is_started_) {

        std::string msg = "ManeuverScheduler::CanSchedule(): maneuver scheduler is not started.";

        RCLCPP_ERROR(node_->get_logger(), msg.c_str());

        return false;

    }

    CombinedDroneAwarenessAdapter awareness;

    if(ProjectExpectedAwarenessFull(
        maneuver,
        awareness
    )) {

        return true;

    }

    return false;

}

bool ManeuverScheduler::RegisterManeuver(
    Maneuver maneuver,
    bool & will_execute_immediately
) {

    RCLCPP_DEBUG(
        node_->get_logger(),
        "ManeuverScheduler::RegisterManeuver(): maneuver type %d",
        maneuver.maneuver_type()
    );

    if (!is_started_) {

        std::string msg = "ManeuverScheduler::RegisterManeuver(): maneuver scheduler is not started.";

        RCLCPP_ERROR(node_->get_logger(), msg.c_str());

        return false;

    }

    std::shared_lock<std::shared_mutex> lck(maneuver_mutex_);

    if (!CanSchedule(maneuver)) {

        return false;

    }

    will_execute_immediately = CanExecute(maneuver);

    if (!maneuver_queue_->Push(maneuver)) {

        return false;

    }

    if (maneuver_execution_timer_->is_canceled()) {

        maneuver_execution_timer_->reset();

    }

    if (will_execute_immediately) {

        RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::RegisterManeuver(): maneuver will execute immediately.");

    } else {

        RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::RegisterManeuver(): maneuver will be scheduled.");

    }

    return true;

}

bool ManeuverScheduler::UpdateManeuver(Maneuver maneuver) {

    if (!is_started_) {

        std::string msg = "ManeuverScheduler::UpdateManeuver(): maneuver scheduler is not started.";

        RCLCPP_ERROR(node_->get_logger(), msg.c_str());

        return false;

    }

    if (!&maneuver_mutex_) {

        RCLCPP_FATAL(node_->get_logger(), "ManeuverScheduler::UpdateManeuver(): maneuver_mutex_ is null");
        throw std::runtime_error("ManeuverScheduler::UpdateManeuver(): maneuver_mutex_ is null");

    } else {

    }

    auto update_timeout_elapsed = [this](Maneuver & maneuver) -> bool {

        rclcpp::Time maneuver_creation_time = maneuver.creation_time();

        rclcpp::Time current_time = rclcpp::Clock().now();

        return (current_time - maneuver_creation_time).seconds() > parameters_->GetParameter("maneuver_register_update_timeout_s").as_double();

    };

    std::shared_lock<std::shared_mutex> lck(maneuver_mutex_);

    if (maneuver == *current_maneuver_) {

        if (update_timeout_elapsed(maneuver)) {

            maneuver.Terminate(false);

            current_maneuver_ = maneuver;

            return false;

        }

        current_maneuver_ = maneuver;

        return true;

    }

    Maneuver old_maneuver = maneuver_queue_->Find(maneuver.uuid());

    if (old_maneuver == Maneuver()) {

        return false;

    }

    if (update_timeout_elapsed(old_maneuver)) {

        old_maneuver.Terminate(false);

        return false;

    }

    if(!maneuver_queue_->Update(maneuver)) {

        std::string msg = "ManeuverScheduler::UpdateManeuver(): maneuver could not be updated in queue, but was already checked previously. This should not happen.";

        RCLCPP_FATAL(node_->get_logger(), msg.c_str());

        throw std::runtime_error(msg);

    } else {

        return true;

    }

}

bool ManeuverScheduler::CancelManeuver(Maneuver maneuver) {

    if (!is_started_) {

        std::string msg = "ManeuverScheduler::CancelManeuver(): maneuver scheduler is not started.";

        RCLCPP_ERROR(node_->get_logger(), msg.c_str());

        return false;

    }

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
    const CombinedDroneAwarenessAdapter & awareness
) const {

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

    if (maneuver != *current_maneuver_) {

        std::string fatal_msg = "ManeuverScheduler::onManeuverCompleted(): Completed maneuver was not the current maneuver.";

        RCLCPP_FATAL(node_->get_logger(), fatal_msg.c_str());

        throw std::runtime_error(fatal_msg);

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

        reference_callback_token_.resource().set(
            std::bind(
                &HoverManeuverServer::GetReference,
                &(*maneuver_server),
                std::placeholders::_1
            ),
            maneuver_server->action_name()
        );

    }

}

Reference ManeuverScheduler::getPassthroughReference(const State & state) const {

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

    iii_drone_interfaces::msg::Reference ref_msg = fetchNextReferenceAndPublish();

    response->reference = ref_msg;
    bool is_valid = maneuverIsExecutingOrPending() || maneuver_server_get_reference_callback_still_registered_;

    response->is_valid = is_valid;

    iii_drone_interfaces::msg::StringStamped reference_callback_provider_msg;

    reference_callback_provider_msg.data = reference_callback_struct_->reference_provider_name;
    reference_callback_provider_msg.stamp = rclcpp::Clock().now();

    reference_callback_provider_publisher_->publish(reference_callback_provider_msg);

}

void ManeuverScheduler::maneuverExecutionTimerCallback() {

    progressScheduler();

}

void ManeuverScheduler::progressScheduler() {

    static bool waiting_for_maneuver_to_start = false;
    static std::string waiting_for_maneuver_to_start_action_name;

    std::unique_lock<std::shared_mutex> lck(maneuver_mutex_);

    auto on_no_maneuver = [this](Maneuver previous_maneuver) {

        auto set_default_no_maneuver_idle_cnt = [this]() {

            no_maneuver_idle_cnt_ = parameters_->GetParameter("no_maneuver_idle_cnt_s").as_double() * 1000 / parameters_->GetParameter("maneuver_execution_period_ms").as_int();

            maneuver_server_get_reference_callback_still_registered_ = false;


        };


        if (!previous_maneuver.success()) {

            set_default_no_maneuver_idle_cnt();

        } else {

            switch (previous_maneuver.maneuver_type()) {

                case MANEUVER_TYPE_HOVER: {
                    hover_maneuver_params_t maneuver_params(previous_maneuver.maneuver_params());

                    if (maneuver_params.sustain_action) {

                        set_default_no_maneuver_idle_cnt();

                        break;

                    }

                    no_maneuver_idle_cnt_ = maneuver_params.duration_s * 1000 / parameters_->GetParameter("maneuver_execution_period_ms").as_int();

                    maneuver_server_get_reference_callback_still_registered_ = true;

                    break;
                }
                case MANEUVER_TYPE_HOVER_BY_OBJECT: {
                    hover_by_object_maneuver_params_t maneuver_params(previous_maneuver.maneuver_params());

                    if (maneuver_params.sustain_action) {

                        set_default_no_maneuver_idle_cnt();

                        break;

                    }

                    no_maneuver_idle_cnt_ = maneuver_params.duration_s * 1000 / parameters_->GetParameter("maneuver_execution_period_ms").as_int();

                    maneuver_server_get_reference_callback_still_registered_ = true;

                    break;
                }
                case MANEUVER_TYPE_HOVER_ON_CABLE: {
                    hover_on_cable_maneuver_params_t maneuver_params(previous_maneuver.maneuver_params());

                    if (maneuver_params.sustain_action) {

                        set_default_no_maneuver_idle_cnt();

                        break;

                    }

                    no_maneuver_idle_cnt_ = maneuver_params.duration_s * 1000 / parameters_->GetParameter("maneuver_execution_period_ms").as_int();

                    maneuver_server_get_reference_callback_still_registered_ = true;

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

            RCLCPP_DEBUG(node_->get_logger(), "ManeuverScheduler::progressScheduler(): no maneuver idle count reached, setting passthrough reference.");

            reference_callback_token_.resource().set(
                std::bind(
                    &ManeuverScheduler::getPassthroughReference,
                    this,
                    std::placeholders::_1
                ),
                "passthrough"
            );

            maneuver_server_get_reference_callback_still_registered_ = false;

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

                rclcpp::Time now = rclcpp::Clock().now();

                float elapsed_seconds = (now - current_maneuver_->termination_time()).seconds();

                float timeout_seconds = parameters_->GetParameter("maneuver_completion_token_acquisition_timeout_s").as_double();

                if (elapsed_seconds > timeout_seconds) {

                    std::string msg = "ManeuverScheduler::progressScheduler(): master does not have token.";

                    RCLCPP_FATAL(node_->get_logger(), msg.c_str());

                    throw std::runtime_error(msg);

                }

                return;

            }

            // Check if the maneuver was successful
            if (current_maneuver_->success()) {

                RCLCPP_DEBUG(
                    node_->get_logger(),
                    "ManeuverScheduler::progressScheduler(): maneuver %d completed successfully.",
                    current_maneuver_->maneuver_type()
                );

                // Check if there are more maneuvers in the queue
                Maneuver maneuver;
                if (maneuver_queue_->Pop(maneuver)) {

                    RCLCPP_DEBUG(
                        node_->get_logger(),
                        "ManeuverScheduler::progressScheduler(): maneuver %d popped from queue as new current maneuver.",
                        maneuver.maneuver_type()
                    );

                    // Progress to the next maneuver
                    current_maneuver_ = maneuver;

                } else {

                    RCLCPP_DEBUG(
                        node_->get_logger(),
                        "ManeuverScheduler::progressScheduler(): no more maneuvers in queue."
                    );

                    on_no_maneuver(current_maneuver_);

                    current_maneuver_ = Maneuver();

                }

            } else {

                RCLCPP_DEBUG(
                    node_->get_logger(),
                    "ManeuverScheduler::progressScheduler(): maneuver %d failed.",
                    current_maneuver_->maneuver_type()
                );

                on_failure(current_maneuver_);

            }

        } else {
            // Current maneuver has not terminated.

            // Check that the maneuver has a goal handle:
            if (current_maneuver_->goal_handle() == nullptr) {

                // Check that the update timeout has elapsed:
                rclcpp::Time maneuver_creation_time = current_maneuver_->creation_time();
                rclcpp::Time current_time = rclcpp::Clock().now();

                if ((current_time - maneuver_creation_time).seconds() > parameters_->GetParameter("maneuver_register_update_timeout_s").as_double()) {

                    on_failure(current_maneuver_);

                }

                return;

            }

            if (!current_maneuver_->started()) {

                // Start the maneuver execution:
                current_maneuver_->Start();

                // Find the maneuver server:
                auto registered_maneuver = registered_maneuvers_.find(current_maneuver_->maneuver_type());

                // Get the action name:
                waiting_for_maneuver_to_start_action_name = registered_maneuver->second->action_name();

                // Set waiting for start:
                waiting_for_maneuver_to_start = true;

                // Reset no_maneuver_idle_cnt:
                *no_maneuver_idle_cnt_ = -1;

            }

            if (waiting_for_maneuver_to_start) {

                if (reference_callback_token_.has_requested_token(waiting_for_maneuver_to_start_action_name)) {

                    // Give the token:
                    reference_callback_token_.Give(waiting_for_maneuver_to_start_action_name);

                    // Reset the flag:
                    waiting_for_maneuver_to_start = false;

                } else {

                    rclcpp::Time maneuver_start_time = current_maneuver_->start_time();
                    rclcpp::Time current_time = rclcpp::Clock().now();

                    if ((current_time - maneuver_start_time).seconds() > parameters_->GetParameter("maneuver_start_timeout_s").as_double()) {

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

    Reference ref = (*reference_callback_struct_)(combined_drone_awareness_handler_->GetState());

    ReferenceAdapter ref_adapter(ref);

    iii_drone_interfaces::msg::Reference ref_msg = ref_adapter.ToMsg();

    reference_publisher_->publish(ref_msg);

    return ref_msg;

}

void ManeuverScheduler::cancelAllPendingManeuvers() {

    maneuver_queue_->Clear();

    current_maneuver_ = Maneuver();

}