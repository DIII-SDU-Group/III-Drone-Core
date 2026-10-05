/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/maneuver/maneuver_scheduler.hpp>
#include <iii_drone_core/control/maneuver/fly_to_position_maneuver_server.hpp>
#include <iii_drone_core/control/maneuver/fly_to_object_maneuver_server.hpp>
#include <iii_drone_core/control/maneuver/follow_waypoint_path_maneuver_server.hpp>
#include <iii_drone_core/control/maneuver/maneuver_request_identity.hpp>
#include <iii_drone_core/diagnostics/hil_trace.hpp>

#include <iii_drone_core/adapters/state_adapter.hpp>

#include <algorithm>
#include <cctype>

using namespace iii_drone::control;
using namespace iii_drone::control::maneuver;
using namespace iii_drone::types;
using namespace iii_drone::utils;
using namespace iii_drone::adapters;

namespace {
// VehicleStatus normally arrives at approximately 2 Hz. Three periods allow
// callback scheduling jitter; this is a proof freshness bound, not an ACK grace.
constexpr auto kNativeHoldStatusFreshness = std::chrono::milliseconds(1500);

bool freshNavigationSample(
    const std::optional<VehicleNavigationSample> & sample,
    std::chrono::steady_clock::time_point now
) {
    return sample && sample->source_timestamp_us != 0 &&
        sample->receipt <= now && now - sample->receipt <= kNativeHoldStatusFreshness;
}

// PX4 applies Core commands only in OFFBOARD or an external mode. Any other
// fresh navigation state (Hold, Land, RTL, manual, ...) is PX4-native control.
bool freshNativeNavigation(
    const VehicleNavigationEvidence & navigation,
    std::chrono::steady_clock::time_point now
) {
    if (!freshNavigationSample(navigation.latest, now) ||
        navigation.latest->nav_state_timestamp_us == 0 ||
        (navigation.last_external &&
         navigation.last_external->source_timestamp_us ==
            navigation.latest->source_timestamp_us)) return false;
    const auto nav_state = navigation.latest->nav_state;
    return nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD &&
        (nav_state < px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_EXTERNAL1 ||
         nav_state > px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_EXTERNAL8);
}
}

/*****************************************************************************/
// Implementation
/*****************************************************************************/

ManeuverScheduler::ManeuverScheduler(
    rclcpp_lifecycle::LifecycleNode *node,
    const CombinedDroneAwarenessHandler::SharedPtr combined_drone_awareness_handler,
    const iii_drone::configuration::Configuration::SharedPtr parameters,
    rclcpp::CallbackGroup::SharedPtr maneuver_execution_callback_group
) : node_(node),
    combined_drone_awareness_handler_(combined_drone_awareness_handler),
    configuration_(parameters),
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

    get_reference_callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive
    );

    reference_publisher_ = node_->create_publisher<iii_drone_interfaces::msg::Reference>(
        "reference",
        10 // Fix QoS
    );

    const auto stream_period = std::chrono::milliseconds(
        configuration_->GetParameter(
            "/control/maneuver_controller/maneuver_execution_period_ms"
        ).as_int()
    );
    const auto stream_timeout = std::chrono::milliseconds(
        configuration_->GetParameter(
            "/control/maneuver_controller/reference_stream_timeout_ms"
        ).as_int()
    );
    // Reference delivery is safety critical and remains local to the vehicle
    // runtime.  Keep a short reliable window so a brief executor/DDS scheduling
    // stall cannot erase an entire successor generation before the consumer
    // has observed and acknowledged its first sample.
    rclcpp::QoS stream_qos(rclcpp::KeepLast(5));
    stream_qos.reliable().durability_volatile();
    stream_qos.deadline(stream_period * 2);
    stream_qos.lifespan(stream_timeout);
    reference_stream_publisher_ =
        node_->create_publisher<iii_drone_interfaces::msg::ManeuverReferenceStream>(
            "reference_stream", stream_qos
        );

    rclcpp::QoS ack_qos(rclcpp::KeepLast(10));
    ack_qos.reliable().durability_volatile();
    rclcpp::SubscriptionOptions ack_options;
    ack_options.callback_group = get_reference_callback_group_;
    reference_ack_subscription_ =
        node_->create_subscription<iii_drone_interfaces::msg::ManeuverReferenceAck>(
            "reference_ack",
            ack_qos,
            callback_lifetime_.Guard<const iii_drone_interfaces::msg::ManeuverReferenceAck::SharedPtr>(
                std::bind(&ManeuverScheduler::acknowledgeReferenceStream, this, std::placeholders::_1)),
            ack_options
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

    // The ack subscription outlives Stop(); end every ROS callback that
    // captures `this` before the scheduler is destroyed.
    callback_lifetime_.Close();

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

    maneuver_queue_ = std::make_unique<ManeuverQueue>(configuration_->GetParameter("/control/maneuver_controller/maneuver_queue_size").as_int());
    
    current_maneuver_ = Maneuver();

    maneuver_publish_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(configuration_->GetParameter("/control/maneuver_controller/maneuver_publish_period_ms").as_int()),
        callback_lifetime_.Guard<>([this]() { publishManeuverStatus(); })
    );

    maneuver_execution_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(
            configuration_->GetParameter(
                "/control/maneuver_controller/maneuver_execution_period_ms"
            ).as_int()
        ),
        callback_lifetime_.Guard<>(std::bind(
            &ManeuverScheduler::maneuverExecutionTimerCallback,
            this
        )),
        maneuver_execution_callback_group_
    );

    maneuver_execution_timer_->cancel();

    // reference_callback_provider_publish_timer_ = node_->create_wall_timer(
    //     std::chrono::milliseconds(configuration_->GetParameter("/control/maneuver_controller/reference_callback_provider_publish_period_ms").as_int()),
    //     [this]() -> void {

    //         std_msgs::msg::String msg;

    //         msg.data = reference_callback_struct_->reference_provider_name;

    //         reference_callback_provider_publisher_->publish(msg);

    //     }
    // );

    get_reference_service_ = node_->create_service<iii_drone_interfaces::srv::GetReference>(
        "get_reference",
        callback_lifetime_.Guard<
            const std::shared_ptr<iii_drone_interfaces::srv::GetReference::Request>, std::shared_ptr<iii_drone_interfaces::srv::GetReference::Response>>(
            std::bind(
            &ManeuverScheduler::getReferenceServiceCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )),
        rclcpp::ServicesQoS(),
        get_reference_callback_group_
    );

    pause_reference_stream_service_ =
        node_->create_service<iii_drone_interfaces::srv::PauseReferenceStream>(
            "pause_reference_stream",
            callback_lifetime_.Guard<
            const std::shared_ptr<iii_drone_interfaces::srv::PauseReferenceStream::Request>, std::shared_ptr<iii_drone_interfaces::srv::PauseReferenceStream::Response>>(
            std::bind(
                &ManeuverScheduler::pauseReferenceStream,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )),
            rclcpp::ServicesQoS(),
            get_reference_callback_group_
        );
    rebase_reference_stream_service_ =
        node_->create_service<iii_drone_interfaces::srv::RebaseReferenceStream>(
            "rebase_reference_stream",
            callback_lifetime_.Guard<
            const std::shared_ptr<iii_drone_interfaces::srv::RebaseReferenceStream::Request>, std::shared_ptr<iii_drone_interfaces::srv::RebaseReferenceStream::Response>>(
            std::bind(
                &ManeuverScheduler::rebaseReferenceStream,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )),
            rclcpp::ServicesQoS(),
            get_reference_callback_group_
        );
    commit_reference_stream_service_ =
        node_->create_service<iii_drone_interfaces::srv::CommitReferenceStream>(
            "commit_reference_stream",
            callback_lifetime_.Guard<
            const std::shared_ptr<iii_drone_interfaces::srv::CommitReferenceStream::Request>, std::shared_ptr<iii_drone_interfaces::srv::CommitReferenceStream::Response>>(
            std::bind(
                &ManeuverScheduler::commitReferenceStream,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )),
            rclcpp::ServicesQoS(),
            get_reference_callback_group_
        );
    terminal_hold_transfer_service_ =
        node_->create_service<iii_drone_interfaces::srv::TerminalHoldTransfer>(
            "terminal_hold_transfer",
            callback_lifetime_.Guard<
            const std::shared_ptr<iii_drone_interfaces::srv::TerminalHoldTransfer::Request>, std::shared_ptr<iii_drone_interfaces::srv::TerminalHoldTransfer::Response>>(
            std::bind(&ManeuverScheduler::terminalHoldTransfer, this,
                std::placeholders::_1, std::placeholders::_2)),
            rclcpp::ServicesQoS(), get_reference_callback_group_
        );

    release_consumer_control_service_ =
        node_->create_service<iii_drone_interfaces::srv::ReleaseConsumerControl>(
            "release_consumer_control",
            callback_lifetime_.Guard<
            const std::shared_ptr<iii_drone_interfaces::srv::ReleaseConsumerControl::Request>, std::shared_ptr<iii_drone_interfaces::srv::ReleaseConsumerControl::Response>>(
            std::bind(&ManeuverScheduler::releaseConsumerControl, this,
                std::placeholders::_1, std::placeholders::_2))
        );

    clear_maneuver_queue_service_ = node_->create_service<iii_drone_interfaces::srv::ClearManeuverQueue>(
        "clear_maneuver_queue",
        callback_lifetime_.Guard<
            const std::shared_ptr<iii_drone_interfaces::srv::ClearManeuverQueue::Request>, std::shared_ptr<iii_drone_interfaces::srv::ClearManeuverQueue::Response>>(
            std::bind(
            &ManeuverScheduler::clearManeuverQueueServiceCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        ))
    );

    is_started_ = true;

}

void ManeuverScheduler::publishManeuverStatus() {
    Maneuver current_maneuver;
    std::vector<Maneuver> maneuver_queue;

    {
        // Status is observational. Never hold the default callback group while
        // waiting for a control transition: odometry ingress uses that group.
        std::shared_lock<std::shared_mutex> lck(maneuver_mutex_, std::try_to_lock);
        if (!lck.owns_lock()) return;
        current_maneuver = current_maneuver_;
        maneuver_queue = maneuver_queue_->vector();
    }

    iii_drone_interfaces::msg::Maneuver current_maneuver_msg =
        ManeuverAdapter(current_maneuver).ToMsg();
    iii_drone_interfaces::msg::ManeuverQueue maneuver_queue_msg;
    maneuver_queue_msg.current_maneuver = current_maneuver_msg;
    for (Maneuver maneuver : maneuver_queue) {
        maneuver_queue_msg.scheduled_maneuvers.push_back(ManeuverAdapter(maneuver).ToMsg());
    }
    current_maneuver_publisher_->publish(current_maneuver_msg);
    maneuver_queue_publisher_->publish(maneuver_queue_msg);
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
    terminal_hold_transfer_service_.reset();
    release_consumer_control_service_.reset();

    if (clear_maneuver_queue_service_) {
        clear_maneuver_queue_service_->clear_on_new_request_callback();
        clear_maneuver_queue_service_.reset();
        clear_maneuver_queue_service_ = nullptr;
    }

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

    if (maneuver_type == MANEUVER_TYPE_FLY_TO_POSITION) {
        std::static_pointer_cast<FlyToPositionManeuverServer>(maneuver_server)
            ->RegisterBlendReferenceAppliedCallback(
                [this](const std::string & request_identity) {
                    return blendedReferenceApplied(request_identity);
                }
            );
    }
    if (maneuver_type == MANEUVER_TYPE_FLY_TO_OBJECT) {
        std::static_pointer_cast<FlyToObjectManeuverServer>(maneuver_server)
            ->RegisterFirstReferenceAppliedCallback(
                [this](const std::string & request_identity) {
                    return firstObjectReferenceApplied(request_identity);
                });
    }
    if (maneuver_type == MANEUVER_TYPE_HOVER_BY_OBJECT) {
        std::static_pointer_cast<HoverByObjectManeuverServer>(maneuver_server)
            ->RegisterFirstReferenceAppliedCallback(
                [this](const std::string & request_identity) {
                    return firstManeuverReferenceApplied(
                        MANEUVER_TYPE_HOVER_BY_OBJECT, request_identity);
                });
        std::static_pointer_cast<HoverByObjectManeuverServer>(maneuver_server)
            ->RegisterAppliedRestReferenceCallback(
                [this](const std::string & request_identity, const Reference & command) {
                    return appliedFiniteRestReference(request_identity, command);
                });
    }
    if (maneuver_type == MANEUVER_TYPE_FLY_TO_OBJECT) {
        std::static_pointer_cast<FlyToObjectManeuverServer>(maneuver_server)
            ->RegisterAppliedRestReferenceCallback(
                [this](const std::string & request_identity, const Reference & command) {
                    return appliedFiniteRestReference(request_identity, command);
                });
    }
    if (maneuver_type == MANEUVER_TYPE_HOVER) {
        std::static_pointer_cast<HoverManeuverServer>(maneuver_server)
            ->RegisterFirstReferenceAppliedCallback(
                [this](const std::string & request_identity) {
                    return firstTerminalHoverReferenceApplied(request_identity);
                });
    }
    if (maneuver_type == MANEUVER_TYPE_FOLLOW_WAYPOINT_PATH) {
        std::static_pointer_cast<FollowWaypointPathManeuverServer>(maneuver_server)
            ->RegisterAppliedRestReferenceCallback(
                [this](const std::string & request_identity, const Reference & command) {
                    return appliedFiniteRestReference(request_identity, command);
                });
    }

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

    auto update_timeout_elapsed = [this](Maneuver & maneuver) -> bool {

        rclcpp::Time maneuver_creation_time = maneuver.creation_time();

        rclcpp::Time current_time = rclcpp::Clock().now();

        return (current_time - maneuver_creation_time).seconds() > configuration_->GetParameter("/control/maneuver_controller/maneuver_register_update_timeout_s").as_double();

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

    std::unique_lock<std::shared_mutex> lck(maneuver_mutex_);
    const Maneuver queued = maneuver_queue_->Find(maneuver.uuid());
    if (queued.maneuver_type() != MANEUVER_TYPE_NONE) {
        // The queue has not started this goal. Preserve that state, but never
        // let an old or mismatched request replace another queued identity.
        if (queued != maneuver ||
            queued.requestIdentity() != maneuver.requestIdentity()) return false;
        Maneuver canceled = queued;
        canceled.Terminate(false);
        return maneuver_queue_->Update(canceled);
    }

    const Maneuver current = current_maneuver_.Load();
    if (maneuver != current ||
        maneuver.requestIdentity() != current.requestIdentity() ||
        current.terminated()) return false;

    // The action worker retained its own pre-Start value. Cancellation is a
    // terminal report, not authority to replace the scheduler's Start state.
    Maneuver canceled = current;
    canceled.Terminate(false);
    current_maneuver_.Store(canceled);
    return true;
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

uint32_t ManeuverScheduler::ClearManeuverQueue(const std::string & request_identity) {

    if (
        !request_identity.empty() &&
        !isValidManeuverRequestIdentity(request_identity)
    ) {
        RCLCPP_WARN(node_->get_logger(), "ManeuverScheduler::ClearManeuverQueue(): Refusing malformed scoped request identity.");
        return 0;
    }

    if (!is_started_) {
        RCLCPP_WARN(node_->get_logger(), "ManeuverScheduler::ClearManeuverQueue(): maneuver scheduler is not started.");
        return 0;
    }

    std::unique_lock<std::shared_mutex> lck(maneuver_mutex_);

    const uint32_t cleared_count = request_identity.empty()
        ? static_cast<uint32_t>(maneuver_queue_->size())
        : maneuver_queue_->ClearRequestIdentity(request_identity);
    if (request_identity.empty()) {
        maneuver_queue_->Clear();
    }

    RCLCPP_INFO(
        node_->get_logger(),
        "ManeuverScheduler::ClearManeuverQueue(): Cleared %u queued maneuver(s) for %s. Current maneuver was not cancelled.",
        cleared_count,
        request_identity.empty() ? "the explicit global request" : "the scoped request identity"
    );

    return cleared_count;

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

    std::unique_lock<std::shared_mutex> lck(maneuver_mutex_);

    if (!maneuver.terminated()) {

        std::string msg = "ManeuverScheduler::onManeuverCompleted(): maneuver was not terminated before completing.";
        RCLCPP_ERROR(node_->get_logger(), msg.c_str());
        return;

    }

    const Maneuver current = current_maneuver_.Load();
    if (maneuver != current ||
        maneuver.requestIdentity() != current.requestIdentity()) {

        std::string fatal_msg = "ManeuverScheduler::onManeuverCompleted(): Completed maneuver was not the current maneuver.";

        RCLCPP_FATAL(node_->get_logger(), fatal_msg.c_str());

        throw std::runtime_error(fatal_msg);

    }

    if (!current.started() || current.terminated()) {
        RCLCPP_ERROR(node_->get_logger(),
            "ManeuverScheduler::onManeuverCompleted(): rejecting completion without an active scheduler Start");
        return;
    }

    // Start belongs to the scheduler's queue/timer copy. The action worker's
    // independent value only reports the terminal outcome.
    Maneuver completed = current;
    completed.Terminate(maneuver.success());
    current_maneuver_.Store(completed);

}

void ManeuverScheduler::onReferenceCallbackTokenReacquired() {

    if (!reference_callback_token_.master_has_token()) {

        std::string msg = "ManeuverScheduler::onReferenceCallbackTokenReacquired(): master does not have token.";

        RCLCPP_ERROR(
            node_->get_logger(),
            "%s Current token holder: %s",
            msg.c_str(),
            reference_callback_token_.token_holder().c_str()
        );

        return;

    }

    if (!current_maneuver_->terminated()) {

        std::string msg = "ManeuverScheduler::onReferenceCallbackTokenReacquired(): maneuver is not terminated.";

        RCLCPP_WARN(
            node_->get_logger(),
            "%s Current maneuver type: %d. Keeping released token with existing callback until scheduler advances.",
            msg.c_str(),
            current_maneuver_->maneuver_type()
        );

        return;

    }

    const Maneuver completed = current_maneuver_;
    const auto hover_entry = registered_maneuvers_.find(MANEUVER_TYPE_HOVER);
    if (hover_entry == registered_maneuvers_.end()) return;
    const auto hover = std::static_pointer_cast<HoverManeuverServer>(hover_entry->second);
    const auto owner = hover->terminalHoldBinding();
    const auto binding = reference_callback_struct_->snapshot();
    const auto source_entry = registered_maneuvers_.find(completed.maneuver_type());
    const bool exact_retained_owner = owner.hold && binding.callback &&
        isValidManeuverRequestIdentity(owner.request_identity) &&
        owner.request_identity == completed.requestIdentity() &&
        binding.request_identity == owner.request_identity &&
        binding.execution_id != 0 &&
        binding.execution_id == current_reference_execution_id_.Load() &&
        source_entry != registered_maneuvers_.end() &&
        binding.reference_provider_name == source_entry->second->action_name();
    if (exact_retained_owner) {
        // The ROS action result can reach a client before the next scheduler
        // tick. Install the exact retained command under its existing request
        // and execution generation before exposing post-action validity. This
        // shares the stream lock with successor begin and native-Hold retire.
        std::lock_guard<std::mutex> lock(reference_stream_mutex_);
        const auto current_owner = hover->terminalHoldBinding();
        const auto current_binding = reference_callback_struct_->snapshot();
        const Maneuver current = current_maneuver_;
        if (current_owner.hold != owner.hold ||
            current_owner.request_identity != owner.request_identity ||
            !current_binding.callback ||
            current_binding.request_identity != binding.request_identity ||
            current_binding.execution_id != binding.execution_id ||
            current_binding.reference_provider_name != binding.reference_provider_name ||
            current_reference_execution_id_.Load() != binding.execution_id ||
            !current.started() || !current.terminated() ||
            current.requestIdentity() != completed.requestIdentity() ||
            current.maneuver_type() != completed.maneuver_type()) return;
        reference_callback_token_.resource().set(
            std::bind(&HoverManeuverServer::GetReference, hover, std::placeholders::_1),
            completed.success() ? binding.reference_provider_name : hover->action_name(),
            binding.execution_id, binding.request_identity);
        auto & epoch = retained_native_hold_epoch_;
        const auto & stream = reference_stream_state_;
        if (stream.valid && !stream.stream_id.empty() &&
            stream.request_identity == owner.request_identity &&
            stream.execution_id == binding.execution_id &&
            epoch.request_identity == owner.request_identity &&
            epoch.execution_id == binding.execution_id) {
            epoch.stream_id = stream.stream_id;
            epoch.completed = true;
            epoch.succeeded = completed.success();
        }
        // Degraded holds keep their finite stop command on this exact owner;
        // terminalHoldTransfer separately refuses to offer a degraded hold.
        maneuver_server_get_reference_callback_still_registered_ = true;
        return;
    }

    if (completed.success() &&
        (completed.maneuver_type() == MANEUVER_TYPE_FLY_TO_OBJECT ||
         completed.maneuver_type() == MANEUVER_TYPE_HOVER_BY_OBJECT)) {
        const auto object_entry = registered_maneuvers_.find(MANEUVER_TYPE_HOVER_BY_OBJECT);
        if (object_entry != registered_maneuvers_.end() &&
            source_entry != registered_maneuvers_.end() &&
            binding.reference_provider_name == source_entry->second->action_name() &&
            binding.request_identity == completed.requestIdentity() &&
            binding.execution_id == current_reference_execution_id_.Load() &&
            std::static_pointer_cast<HoverByObjectManeuverServer>(object_entry->second)
                ->RetainsTrackedSource(binding)) {
            std::lock_guard<std::mutex> lock(reference_stream_mutex_);
            const auto current_binding = reference_callback_struct_->snapshot();
            const Maneuver current = current_maneuver_;
            if (current.started() && current.terminated() && current.success() &&
                current.requestIdentity() == completed.requestIdentity() &&
                current.maneuver_type() == completed.maneuver_type() &&
                current_binding.callback &&
                current_binding.request_identity == binding.request_identity &&
                current_binding.execution_id == binding.execution_id &&
                current_binding.reference_provider_name == binding.reference_provider_name &&
                current_reference_execution_id_.Load() == binding.execution_id &&
                std::static_pointer_cast<HoverByObjectManeuverServer>(object_entry->second)
                    ->RetainsTrackedSource(current_binding)) {
                auto & epoch = retained_native_hold_epoch_;
                const auto & stream = reference_stream_state_;
                if (stream.valid && !stream.stream_id.empty() &&
                    stream.request_identity == current_binding.request_identity &&
                    stream.execution_id == current_binding.execution_id &&
                    epoch.request_identity == current_binding.request_identity &&
                    epoch.execution_id == current_binding.execution_id) {
                    epoch.stream_id = stream.stream_id;
                    epoch.completed = true;
                    epoch.succeeded = true;
                }
                maneuver_server_get_reference_callback_still_registered_ = true;
                return;
            }
        }
    }

    const bool object_goal = completed.maneuver_type() == MANEUVER_TYPE_FLY_TO_OBJECT ||
        completed.maneuver_type() == MANEUVER_TYPE_HOVER_BY_OBJECT;
    const bool exact_failed_object_binding = !completed.success() && object_goal &&
        source_entry != registered_maneuvers_.end() &&
        binding.request_identity == completed.requestIdentity() &&
        binding.execution_id != 0 &&
        binding.execution_id == current_reference_execution_id_.Load() &&
        binding.reference_provider_name == source_entry->second->action_name();
    const bool failed_object_source = exact_failed_object_binding &&
        (source_entry->second->startupRejected(binding) ||
         (completed.maneuver_type() == MANEUVER_TYPE_FLY_TO_OBJECT
            ? std::static_pointer_cast<FlyToObjectManeuverServer>(source_entry->second)
                ->TrackedSourceUnrecoverable(binding)
            : std::static_pointer_cast<HoverByObjectManeuverServer>(source_entry->second)
                ->TrackedSourceUnrecoverable(binding)));
    if (failed_object_source) {
        std::lock_guard<std::mutex> lock(reference_stream_mutex_);
        const auto current_binding = reference_callback_struct_->snapshot();
        const Maneuver current = current_maneuver_;
        if (current.requestIdentity() != completed.requestIdentity() ||
            !current.terminated() || current.success() ||
            current_binding.request_identity != binding.request_identity ||
            current_binding.execution_id != binding.execution_id ||
            current_binding.reference_provider_name != binding.reference_provider_name ||
            current_reference_execution_id_.Load() != binding.execution_id) return;
        const auto & stream = reference_stream_state_;
        const bool accepted_finite = stream.valid &&
            stream.request_identity == binding.request_identity &&
            stream.execution_id == binding.execution_id &&
            stream.ack_seen && stream.last_ack_sequence != 0 &&
            stream.last_consumer_status ==
                iii_drone_interfaces::msg::ManeuverReferenceAck::STATUS_APPLIED &&
            stream.last_ack_reference_valid &&
            stream.last_ack_reference.position().allFinite() &&
            stream.last_ack_reference.velocity().allFinite() &&
            stream.last_ack_reference.acceleration().allFinite() &&
            std::isfinite(stream.last_ack_reference.yaw()) &&
            std::isfinite(stream.last_ack_reference.yaw_rate()) &&
            std::isfinite(stream.last_ack_reference.yaw_acceleration());
        if (accepted_finite) {
            const Reference command = stream.last_ack_reference;
            reference_callback_token_.resource().set(
                [this, command](const State &) {
                    return command.CopyWithNewStamp(node_->now());
                }, binding.reference_provider_name, binding.execution_id,
                binding.request_identity);
            maneuver_server_get_reference_callback_still_registered_ = true;
        } else {
            // No command in this failed generation was actually applied.
            // Leave the client's existing finite predecessor/hover in charge.
            reference_callback_token_.resource().set(nullptr,
                binding.reference_provider_name, binding.execution_id,
                binding.request_identity);
            reference_stream_state_.valid = false;
            maneuver_server_get_reference_callback_still_registered_ = false;
        }
        return;
    }

    if (!completed.success()) {

        auto maneuver_server = hover;

        // A canceled maneuver may have inherited an old hover target from a
        // previous successful maneuver. Refresh it before exposing the
        // fallback callback so a mode handoff cannot command that stale pose.
        if (!maneuver_server->terminalHold()) {
            maneuver_server->Update(Reference(combined_drone_awareness_handler_->GetState()));
        }

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

    Maneuver current_maneuver = current_maneuver_;

    if (
        current_maneuver.terminated()
        || (
            current_maneuver.maneuver_type() != MANEUVER_TYPE_HOVER_BY_OBJECT
            && current_maneuver.maneuver_type() != MANEUVER_TYPE_HOVER_ON_CABLE
        )
    ) {
        // Retained hover callbacks can fire after a successor has taken over; those must not clear handoff work.
        RCLCPP_DEBUG(
            node_->get_logger(),
            "ManeuverScheduler::onHoveringFail(): Ignoring stale hover failure callback while current maneuver type is %d and terminated=%s.",
            current_maneuver.maneuver_type(),
            current_maneuver.terminated() ? "true" : "false"
        );
        return;
    }

    const uint32_t cleared_count = static_cast<uint32_t>(maneuver_queue_->size());
    maneuver_queue_->Clear();

    RCLCPP_WARN(
        node_->get_logger(),
        "ManeuverScheduler::onHoveringFail(): Active hover failed; cleared %u queued maneuver(s). Current hover maneuver was not cleared.",
        cleared_count
    );

}

void ManeuverScheduler::getReferenceServiceCallback(
    const std::shared_ptr<iii_drone_interfaces::srv::GetReference::Request>,
    std::shared_ptr<iii_drone_interfaces::srv::GetReference::Response> response
) {

    const ReferenceCallbackBinding binding = reference_callback_struct_->snapshot();
    const auto ref_msg = fetchNextReferenceAndPublish(binding);
    if (ref_msg) response->reference = *ref_msg;

    // RegisterManeuver() makes a goal pending before its server has acquired
    // the callback token. During that gap the callback can still belong to a
    // previous maneuver. Keep the response invalid until the current
    // maneuver's provider is installed; the client will hold its live hover
    // reference while waiting.
    response->is_valid = ref_msg.has_value();

    iii_drone_interfaces::msg::StringStamped reference_callback_provider_msg;

    reference_callback_provider_msg.data = binding.reference_provider_name;
    reference_callback_provider_msg.stamp = rclcpp::Clock().now();

    reference_callback_provider_publisher_->publish(reference_callback_provider_msg);

}

void ManeuverScheduler::clearManeuverQueueServiceCallback(
    const std::shared_ptr<iii_drone_interfaces::srv::ClearManeuverQueue::Request> request,
    std::shared_ptr<iii_drone_interfaces::srv::ClearManeuverQueue::Response> response
) {

    if (
        !request->request_identity.empty() &&
        !isValidManeuverRequestIdentity(request->request_identity)
    ) {
        RCLCPP_WARN(
            node_->get_logger(),
            "ManeuverScheduler::clearManeuverQueueServiceCallback(): Refusing malformed scoped clear. Reason: %s",
            request->reason.c_str()
        );
        response->cleared_count = 0;
        response->success = false;
        return;
    }

    RCLCPP_INFO(
        node_->get_logger(),
        "ManeuverScheduler::clearManeuverQueueServiceCallback(): Clearing maneuver queue. Reason: %s",
        request->reason.c_str()
    );

    response->cleared_count = ClearManeuverQueue(request->request_identity);
    response->success = true;

}

void ManeuverScheduler::maneuverExecutionTimerCallback() {

    const auto callback_start = std::chrono::steady_clock::now();

    // This check must precede every scheduler transition. After executor
    // congestion, queued timer callbacks may otherwise evaluate an old
    // trajectory at a much later time and retire the maneuver before the
    // consumer can stop and rebase it.
    if (!pauseReferenceStreamIfRequired()) {
        progressScheduler();
    }
    publishReferenceStream();

    const auto callback_end = std::chrono::steady_clock::now();
    auto event = iii_drone::diagnostics::HilTrace::event("maneuver_execution_timer");
    event.number(
        "duration_ns",
        static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
            callback_end - callback_start).count()));
    event.commit();

}

ManeuverServer::SharedPtr ManeuverScheduler::activeManeuverServer() const {
    const Maneuver maneuver = current_maneuver_;
    if (
        maneuver.maneuver_type() == MANEUVER_TYPE_NONE ||
        !maneuver.started() || maneuver.terminated()
    ) {
        return nullptr;
    }
    const auto entry = registered_maneuvers_.find(maneuver.maneuver_type());
    return entry == registered_maneuvers_.end() ? nullptr : entry->second;
}

bool ManeuverScheduler::currentReferenceValid(
    const ReferenceCallbackBinding & binding
) const {
    if (
        !binding.callback ||
        !isValidManeuverRequestIdentity(binding.request_identity) ||
        binding.execution_id != current_reference_execution_id_.Load()
    ) {
        return false;
    }

    const Maneuver maneuver = current_maneuver_;
    if (
        maneuver.maneuver_type() != MANEUVER_TYPE_NONE && maneuver.started() &&
        !maneuver.terminated() &&
        binding.request_identity == maneuver.requestIdentity()
    ) {
        const auto server = registered_maneuvers_.find(maneuver.maneuver_type());
        return server != registered_maneuvers_.end() &&
            binding.reference_provider_name == server->second->action_name();
    }
    return maneuver_server_get_reference_callback_still_registered_;
}

std::string ManeuverScheduler::nextReferenceStreamId(const std::string & provider) {
    std::string safe_provider = provider;
    std::replace_if(
        safe_provider.begin(), safe_provider.end(),
        [](unsigned char value) { return !std::isalnum(value); }, '_'
    );
    return safe_provider + ":" + std::to_string(node_->now().nanoseconds()) + ":" +
        std::to_string(++reference_stream_state_.generation);
}

void ManeuverScheduler::beginReferenceExecution(
    const std::string & provider,
    const std::string & request_identity,
    std::optional<Reference> initial_command
) {
    ReferenceCallback initial_callback;
    if (initial_command) {
        initial_callback = [this, seed = *initial_command](const State &) {
            return seed.CopyWithNewStamp(node_->now());
        };
    }
    // Retire the predecessor before the successor can request its token. This
    // rejects old ACK/pause/rebase traffic during the initialization gap and
    // prevents an old same-provider callable from being published as the new
    // execution's first sample. Binding, execution and stream become visible
    // as one generation to native-Hold retirement and transfer.
    std::lock_guard<std::mutex> lock(reference_stream_mutex_);
    maneuver_server_get_reference_callback_still_registered_ = false;
    seeded_successor_execution_.reset();
    const uint64_t execution_id = reference_callback_struct_->beginExecution(
        provider, request_identity, std::move(initial_callback));
    current_reference_execution_id_.Store(execution_id);
    const auto navigation = combined_drone_awareness_handler_->GetVehicleNavigationEvidence();
    const auto steady_now = std::chrono::steady_clock::now();
    retained_native_hold_epoch_ = RetainedNativeHoldEpoch{};
    retained_native_hold_epoch_.request_identity = request_identity;
    retained_native_hold_epoch_.execution_id = execution_id;
    retained_native_hold_epoch_.status_source_epoch = navigation.source_epoch;
    retained_native_hold_epoch_.owner_started = steady_now;
    if (freshNavigationSample(navigation.latest, steady_now) &&
        navigation.last_external &&
        navigation.latest->source_timestamp_us ==
            navigation.last_external->source_timestamp_us) {
        retained_native_hold_epoch_.external_status_timestamp_us =
            navigation.last_external->source_timestamp_us;
        retained_native_hold_epoch_.external_nav_transition_us =
            navigation.last_external->nav_state_timestamp_us;
    }
    reference_stream_state_.valid = false;
    reference_stream_state_.paused = false;
    reference_stream_state_.prepared = false;
    reference_stream_state_.committed_waiting_for_applied = false;
    reference_stream_state_.abort_waiting_for_consumer_ready = false;
    reference_stream_state_.ack_seen = false;
    reference_stream_state_.claimed_consumer_identity.clear();
    reference_stream_state_.claim_ack_pending = false;
    reference_stream_state_.claim_source_ack_sequence = 0;
    reference_stream_state_.offer_consumer_identity.clear();
    reference_stream_state_.offer_ack_sequence = 0;
    reference_stream_state_.last_ack_reference_valid = false;
    reference_stream_state_.recent_references.clear();
    reference_stream_state_.object_tracking_sequences.clear();
    reference_stream_state_.object_stop_requested_sequence = 0;
}

void ManeuverScheduler::beginReferenceExecution(const std::string & provider) {
    beginReferenceExecution(provider, "");
}

void ManeuverScheduler::beginSeededSuccessorExecution(
    const std::string & provider,
    const std::string & request_identity,
    const Reference & seed
) {
    beginReferenceExecution(provider, request_identity, seed);
    std::lock_guard<std::mutex> lock(reference_stream_mutex_);
    const auto binding = reference_callback_struct_->snapshot();
    seeded_successor_execution_ = detail::SeededSuccessorExecution{
        binding.request_identity, binding.execution_id, binding.revision};
}

bool ManeuverScheduler::installUnexecutedSuccessorHoldLocked() {
    if (!seeded_successor_execution_) return false;
    const auto hover_entry = registered_maneuvers_.find(MANEUVER_TYPE_HOVER);
    if (hover_entry == registered_maneuvers_.end()) return false;
    const auto hover = std::static_pointer_cast<HoverManeuverServer>(hover_entry->second);
    const auto owner = hover->terminalHoldBinding();
    const Maneuver successor = current_maneuver_;
    const auto binding = reference_callback_struct_->snapshot();
    detail::UnexecutedSuccessor state;
    state.seeded = *seeded_successor_execution_;
    state.request_identity = successor.requestIdentity();
    state.started = successor.started();
    state.terminated = successor.terminated();
    state.succeeded = successor.success();
    state.binding_request_identity = binding.request_identity;
    state.binding_execution_id = binding.execution_id;
    state.binding_revision = binding.revision;
    state.current_execution_id = current_reference_execution_id_.Load();
    state.master_has_token = reference_callback_token_.master_has_token();
    state.hold_tracking = owner.hold &&
        owner.hold->phase() == TerminalTrackingHold::Phase::Tracking;
    state.hold_owner = owner.request_identity;
    if (!detail::UnexecutedSuccessorOwnsRetainedHold(state)) return false;

    // As for an executed owner that ended unsuccessfully (token return):
    // the hold continues under the successor's request and execution.
    hover->AdoptTerminalHold(owner.hold, successor.requestIdentity());
    reference_callback_token_.resource().set(
        std::bind(&HoverManeuverServer::GetReference, hover, std::placeholders::_1),
        hover->action_name(), binding.execution_id, binding.request_identity);
    auto & epoch = retained_native_hold_epoch_;
    const auto & stream = reference_stream_state_;
    if (stream.valid && !stream.stream_id.empty() &&
        stream.request_identity == binding.request_identity &&
        stream.execution_id == binding.execution_id &&
        epoch.request_identity == binding.request_identity &&
        epoch.execution_id == binding.execution_id) {
        epoch.stream_id = stream.stream_id;
        epoch.completed = true;
        epoch.succeeded = false;
    }
    maneuver_server_get_reference_callback_still_registered_ = true;
    seeded_successor_execution_.reset();
    RCLCPP_INFO(node_->get_logger(),
        "Terminal hold: request %s ended before its server took over; it keeps the retained hold of request %s",
        successor.requestIdentity().c_str(), owner.request_identity.c_str());
    return true;
}

bool ManeuverScheduler::pauseReferenceStreamIfRequired() {
    const auto steady_now = std::chrono::steady_clock::now();
    ManeuverServer::SharedPtr server;
    bool newly_paused = false;
    bool progression_blocked = false;

    {
        std::lock_guard<std::mutex> lock(reference_stream_mutex_);
        if (!reference_stream_state_.valid) {
            return false;
        }

        if (
            reference_stream_state_.execution_id !=
            current_reference_execution_id_.Load()
        ) {
            return false;
        }

        server = activeManeuverServer();
        if (!server) {
            return false;
        }

        if (!reference_stream_state_.paused) {
            const auto acknowledgement_timeout = std::chrono::milliseconds(
                configuration_->GetParameter(
                    "/control/maneuver_controller/reference_stream_timeout_ms"
                ).as_int()
            );
            const auto acknowledgement_age = reference_stream_state_.ack_seen
                ? steady_now - reference_stream_state_.last_ack
                : steady_now - reference_stream_state_.generation_started;
            if (acknowledgement_age > acknowledgement_timeout) {
                reference_stream_state_.paused = true;
                newly_paused = true;
                auto event = iii_drone::diagnostics::HilTrace::event("reference_stream_pause_timeout");
                event.text("stream_id", reference_stream_state_.stream_id);
                event.number("sequence", reference_stream_state_.sequence);
                event.boolean("ack_seen", reference_stream_state_.ack_seen);
                event.number("acknowledgement_age_ms", static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::milliseconds>(acknowledgement_age).count()));
                event.commit();
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "Reference stream %s missed consumer acknowledgements; blocking scheduler "
                    "progression and pausing producer at sequence %lu.",
                    reference_stream_state_.stream_id.c_str(),
                    static_cast<unsigned long>(reference_stream_state_.sequence)
                );
            }
        }
        progression_blocked = reference_stream_state_.paused;
    }

    if (newly_paused) {
        server->PauseReferenceStream();
    }
    return progression_blocked;
}

void ManeuverScheduler::publishReferenceStream() {
    const ReferenceCallbackBinding binding = reference_callback_struct_->snapshot();
    const bool valid = currentReferenceValid(binding);
    const Maneuver completed_maneuver = current_maneuver_;
    const auto fly_object_entry = registered_maneuvers_.find(MANEUVER_TYPE_FLY_TO_OBJECT);
    const auto hover_object_entry = registered_maneuvers_.find(MANEUVER_TYPE_HOVER_BY_OBJECT);
    const bool fly_source = fly_object_entry != registered_maneuvers_.end() &&
        binding.reference_provider_name == fly_object_entry->second->action_name();
    const bool hover_source = hover_object_entry != registered_maneuvers_.end() &&
        binding.reference_provider_name == hover_object_entry->second->action_name();
    // A completed object source may still have an entered fallback invocation
    // inside ObjectTrackingSession::Compute. RetainsTrackedSource() takes that
    // session's mutex, so even inspecting it here can stall the timer after
    // progressScheduler() has returned. Keep the exact existing stream intact
    // until the invocation drains; the next tick can sample and publish it.
    // This also covers an expired successor: its rejection resumes the same
    // lease, but the entered invocation is still active until it exits.
    if (valid && binding.lease && !binding.lease->drained() &&
        (fly_source || hover_source) &&
        hover_object_entry != registered_maneuvers_.end() &&
        std::static_pointer_cast<HoverByObjectManeuverServer>(hover_object_entry->second)
            ->HasTrackedSourceIdentity(binding)) {
        std::lock_guard<std::mutex> stream_lock(reference_stream_mutex_);
        std::lock_guard<std::mutex> binding_lock(
            reference_callback_struct_->publication_mutex_);
        const auto current_binding = reference_callback_struct_->snapshot();
        const Maneuver current = current_maneuver_;
        const bool active_source = current.started() && !current.terminated() &&
            current.requestIdentity() == binding.request_identity;
        const auto & stream = reference_stream_state_;
        if (!active_source && current_binding.revision == binding.revision &&
            current_binding.callback &&
            current_binding.request_identity == binding.request_identity &&
            current_binding.execution_id == binding.execution_id &&
            current_binding.reference_provider_name == binding.reference_provider_name &&
            binding.execution_id == current_reference_execution_id_.Load() &&
            stream.valid && !stream.stream_id.empty() &&
            stream.provider == binding.reference_provider_name &&
            stream.request_identity == binding.request_identity &&
            stream.execution_id == binding.execution_id &&
            !binding.lease->drained()) {
            return;
        }
    }
    const bool fly_tracking = fly_source &&
        std::static_pointer_cast<FlyToObjectManeuverServer>(fly_object_entry->second)
            ->RetainsTrackedSource(binding);
    const bool hover_tracking = (fly_source || hover_source) &&
        hover_object_entry != registered_maneuvers_.end() &&
        std::static_pointer_cast<HoverByObjectManeuverServer>(hover_object_entry->second)
            ->RetainsTrackedSource(binding);
    const auto source_entry = registered_maneuvers_.find(completed_maneuver.maneuver_type());
    const bool object_startup_rejected = source_entry != registered_maneuvers_.end() &&
        (completed_maneuver.maneuver_type() == MANEUVER_TYPE_FLY_TO_OBJECT ||
         completed_maneuver.maneuver_type() == MANEUVER_TYPE_HOVER_BY_OBJECT) &&
        source_entry->second->startupRejected(binding);
    const bool object_tracking = !object_startup_rejected &&
        binding.callback && binding.execution_id != 0 &&
        binding.execution_id == current_reference_execution_id_.Load() &&
        (fly_tracking || hover_tracking);
    const bool object_unrecoverable = object_startup_rejected || (object_tracking &&
        ((fly_tracking && std::static_pointer_cast<FlyToObjectManeuverServer>(
            fly_object_entry->second)->TrackedSourceUnrecoverable(binding)) ||
         (hover_tracking && std::static_pointer_cast<HoverByObjectManeuverServer>(
            hover_object_entry->second)->TrackedSourceUnrecoverable(binding))));
    const auto navigation = combined_drone_awareness_handler_->GetVehicleNavigationEvidence();
    const std::string provider = binding.reference_provider_name;
    const auto steady_now = std::chrono::steady_clock::now();
    ManeuverServer::SharedPtr server_to_pause;
    HoverManeuverServer::TerminalHoldBinding terminal_binding;
    if (const auto hover_entry = registered_maneuvers_.find(MANEUVER_TYPE_HOVER);
        hover_entry != registered_maneuvers_.end()) {
        terminal_binding = std::static_pointer_cast<HoverManeuverServer>(
            hover_entry->second)->terminalHoldBinding();
    }
    const auto completed_source = source_entry;
    const bool exact_owner_binding = terminal_binding.hold && binding.callback &&
        isValidManeuverRequestIdentity(terminal_binding.request_identity) &&
        binding.request_identity == terminal_binding.request_identity &&
        binding.execution_id != 0 &&
        binding.execution_id == current_reference_execution_id_.Load();
    const bool exact_completion_binding = exact_owner_binding &&
        completed_maneuver.started() && completed_maneuver.terminated() &&
        terminal_binding.request_identity == completed_maneuver.requestIdentity() &&
        completed_source != registered_maneuvers_.end() &&
        (provider == completed_source->second->action_name() ||
         (!completed_maneuver.success() &&
          provider == registered_maneuvers_.at(MANEUVER_TYPE_HOVER)->action_name()));
    // The hold can survive a successor's seed generation, but only its own
    // request may receive terminal status or terminal watchdog handling.
    auto terminal_hold = terminal_binding.request_identity == binding.request_identity
        ? terminal_binding.hold : nullptr;
    bool terminal_ack_failed = false;
    std::string stream_id;
    bool prepared = false;
    bool paused = false;
    bool committed_waiting_for_applied = false;
    Reference reference;

    {
        std::lock_guard<std::mutex> lock(reference_stream_mutex_);
        std::lock_guard<std::mutex> binding_lock(
            reference_callback_struct_->publication_mutex_);
        if (reference_callback_struct_->snapshot().revision != binding.revision) {
            return;
        }
        auto & native_epoch = retained_native_hold_epoch_;
        if (completed_maneuver.started() && !completed_maneuver.terminated() &&
            completed_maneuver.requestIdentity() == binding.request_identity &&
            native_epoch.request_identity == binding.request_identity &&
            native_epoch.execution_id == binding.execution_id &&
            native_epoch.status_source_epoch == navigation.source_epoch &&
            freshNavigationSample(navigation.last_external, steady_now) &&
            navigation.last_external->receipt > native_epoch.owner_started &&
            navigation.last_external->source_timestamp_us >
                native_epoch.external_status_timestamp_us &&
            navigation.last_external->nav_state_timestamp_us >
                native_epoch.minimum_external_transition_us) {
            native_epoch.external_status_timestamp_us =
                navigation.last_external->source_timestamp_us;
            native_epoch.external_nav_transition_us =
                navigation.last_external->nav_state_timestamp_us;
        }
        if (!valid) {
            if (reference_stream_state_.valid &&
                reference_stream_state_.execution_id == binding.execution_id &&
                reference_stream_state_.request_identity == binding.request_identity &&
                !reference_stream_state_.stream_id.empty() &&
                binding.execution_id == current_reference_execution_id_.Load()) {
                const auto object_entry = registered_maneuvers_.find(
                    MANEUVER_TYPE_HOVER_BY_OBJECT);
                const auto current_binding = reference_callback_struct_->snapshot();
                const Maneuver current = current_maneuver_;
                if (object_entry != registered_maneuvers_.end() &&
                    current_binding.callback &&
                    current_binding.request_identity == binding.request_identity &&
                    current_binding.execution_id == binding.execution_id &&
                    current_binding.reference_provider_name == binding.reference_provider_name &&
                    std::static_pointer_cast<HoverByObjectManeuverServer>(object_entry->second)
                        ->RetainsTrackedSource(current_binding) &&
                    ((current.started() && current.terminated() && current.success() &&
                      current.requestIdentity() == binding.request_identity &&
                      registered_maneuvers_.count(current.maneuver_type()) != 0 &&
                      current_binding.reference_provider_name ==
                          registered_maneuvers_.at(current.maneuver_type())->action_name()) ||
                     currentReferenceValid(current_binding))) {
                    return;
                }
            }
            // Terminate may precede Token::Release, and Release exposes the
            // master token before its reacquire callback installs the retained
            // Hover callable. Keep only this exact already-applied stream
            // intact during that transition; no new command is published.
            if (exact_owner_binding && reference_stream_state_.valid &&
                reference_stream_state_.execution_id == binding.execution_id &&
                reference_stream_state_.request_identity == binding.request_identity &&
                !reference_stream_state_.stream_id.empty() &&
                current_reference_execution_id_.Load() == binding.execution_id) {
                const auto current_owner = std::static_pointer_cast<HoverManeuverServer>(
                    registered_maneuvers_.at(MANEUVER_TYPE_HOVER))->terminalHoldBinding();
                const auto current_binding = reference_callback_struct_->snapshot();
                const Maneuver current_maneuver = current_maneuver_;
                const bool same_owner = current_owner.hold == terminal_binding.hold &&
                    current_owner.request_identity == terminal_binding.request_identity &&
                    current_binding.callback &&
                    current_binding.request_identity == binding.request_identity &&
                    current_binding.execution_id == binding.execution_id;
                const bool source_finalizing = exact_completion_binding && same_owner &&
                    current_maneuver.started() && current_maneuver.terminated() &&
                    current_maneuver.maneuver_type() == completed_maneuver.maneuver_type() &&
                    current_maneuver.requestIdentity() == completed_maneuver.requestIdentity() &&
                    (current_binding.reference_provider_name ==
                         completed_source->second->action_name() ||
                     (!completed_maneuver.success() &&
                      current_binding.reference_provider_name ==
                          registered_maneuvers_.at(MANEUVER_TYPE_HOVER)->action_name())) &&
                    !maneuver_server_get_reference_callback_still_registered_.Load();
                const bool callback_rebound = same_owner &&
                    currentReferenceValid(current_binding);
                if (source_finalizing || callback_rebound) return;
            }
            reference_stream_state_.valid = false;
            reference_stream_state_.abort_waiting_for_consumer_ready = false;
            return;
        }
        // The callback may have been retired or a successor may have begun
        // after the pre-lock snapshot. Never recreate an old stream from it.
        const auto locked_binding = reference_callback_struct_->snapshot();
        if (!currentReferenceValid(binding) || !locked_binding.callback ||
            locked_binding.execution_id != binding.execution_id ||
            locked_binding.request_identity != binding.request_identity ||
            locked_binding.reference_provider_name != binding.reference_provider_name) {
            return;
        }
        if (
            !reference_stream_state_.valid ||
            reference_stream_state_.execution_id != binding.execution_id
        ) {
            reference_stream_state_.stream_id = nextReferenceStreamId(provider);
            reference_stream_state_.provider = provider;
            reference_stream_state_.request_identity = binding.request_identity;
            reference_stream_state_.execution_id = binding.execution_id;
            reference_stream_state_.sequence = 0;
            reference_stream_state_.last_ack_sequence = 0;
            reference_stream_state_.valid = true;
            reference_stream_state_.paused = false;
            reference_stream_state_.prepared = false;
            reference_stream_state_.committed_waiting_for_applied = false;
            reference_stream_state_.abort_waiting_for_consumer_ready = false;
            reference_stream_state_.ack_seen = false;
            reference_stream_state_.offer_consumer_identity.clear();
            reference_stream_state_.offer_ack_sequence = 0;
            reference_stream_state_.last_ack_reference_valid = false;
            reference_stream_state_.recent_references.clear();
            reference_stream_state_.object_tracking_sequences.clear();
            reference_stream_state_.object_stop_requested_sequence = 0;
            reference_stream_state_.generation_started = steady_now;
            auto event = iii_drone::diagnostics::HilTrace::event("reference_stream_generation_created");
            event.text("stream_id", reference_stream_state_.stream_id);
            event.text("provider", provider);
            event.commit();
        }
        const auto ack_timeout = std::chrono::milliseconds(
            configuration_->GetParameter(
                "/control/maneuver_controller/reference_stream_timeout_ms"
            ).as_int()
        );
        const auto acknowledgement_age = reference_stream_state_.ack_seen
            ? steady_now - reference_stream_state_.last_ack
            : steady_now - reference_stream_state_.generation_started;
        const bool claim_grace = reference_stream_state_.claim_ack_pending &&
            steady_now < reference_stream_state_.claim_deadline;
        const auto applicable_ack_timeout = terminal_hold
            ? detail::TerminalHoldAckTimeout(ack_timeout)
            : ack_timeout;
        if (!reference_stream_state_.paused && !claim_grace &&
            acknowledgement_age > applicable_ack_timeout) {
            if (terminal_hold) {
                terminal_ack_failed = true;
            } else {
            reference_stream_state_.paused = true;
            server_to_pause = activeManeuverServer();
            auto event = iii_drone::diagnostics::HilTrace::event("reference_stream_paused");
            event.text("stream_id", reference_stream_state_.stream_id);
            event.number("sequence", reference_stream_state_.sequence);
            event.boolean("ack_seen", reference_stream_state_.ack_seen);
            event.number("acknowledgement_age_ms", static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::milliseconds>(acknowledgement_age).count()));
            event.commit();
            RCLCPP_ERROR(
                node_->get_logger(),
                "Reference stream %s missed consumer acknowledgements; pausing producer at sequence %lu.",
                reference_stream_state_.stream_id.c_str(),
                static_cast<unsigned long>(reference_stream_state_.sequence)
            );
            }
        }
        stream_id = reference_stream_state_.stream_id;
        prepared = reference_stream_state_.prepared;
        paused = reference_stream_state_.paused;
        committed_waiting_for_applied =
            reference_stream_state_.committed_waiting_for_applied;
        if (prepared || committed_waiting_for_applied) {
            reference = reference_stream_state_.prepared_reference.CopyWithNewStamp(node_->now());
        } else if (paused) {
            reference = reference_stream_state_.latest_reference.CopyWithNewStamp(node_->now());
        }
    }

    if (server_to_pause) {
        server_to_pause->PauseReferenceStream();
    }
    if (terminal_ack_failed) {
        terminal_hold->Fail("terminal hold lost its applied consumer acknowledgement");
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
            "Terminal hold reference stream %s failed: %s (phase=%u)",
            stream_id.c_str(), terminal_hold->failureReason().c_str(),
            static_cast<unsigned>(terminal_hold->phase()));
    }
    if (!prepared && !paused && !committed_waiting_for_applied) {
        try {
            reference = binding.callback(combined_drone_awareness_handler_->GetState());
        } catch (const RetiredReferenceCallback &) {
            // A copied callable was superseded before invocation. The owner
            // will publish its next valid sample; this one has no authority.
            return;
        }
    }

    // The stop state describes the command sampled above, not merely a stop
    // request received since the previous publication.
    const bool sampled_object_command = !prepared && !paused &&
        !committed_waiting_for_applied;
    const bool object_stopping = sampled_object_command && object_tracking &&
        ((fly_tracking && std::static_pointer_cast<FlyToObjectManeuverServer>(
            fly_object_entry->second)->TrackedTransitionStopping(binding)) ||
         (hover_tracking && std::static_pointer_cast<HoverByObjectManeuverServer>(
            hover_object_entry->second)->TrackedTransitionStopping(binding)));
    const auto object_rest = !object_stopping ? std::optional<Reference>{} :
        (hover_tracking
            ? std::static_pointer_cast<HoverByObjectManeuverServer>(
                hover_object_entry->second)->TrackedTransitionRest(binding)
            : std::static_pointer_cast<FlyToObjectManeuverServer>(
                fly_object_entry->second)->TrackedTransitionRest(binding));
    const bool object_stopped = object_rest &&
        reference.position().allFinite() && reference.velocity().allFinite() &&
        reference.acceleration().allFinite() && std::isfinite(reference.yaw()) &&
        std::isfinite(reference.yaw_rate()) &&
        std::isfinite(reference.yaw_acceleration()) &&
        reference.velocity().norm() <= 1.0e-5 &&
        reference.acceleration().norm() <= 1.0e-5 &&
        std::abs(reference.yaw_rate()) <= 1.0e-5 &&
        std::abs(reference.yaw_acceleration()) <= 1.0e-5;

    iii_drone_interfaces::msg::ManeuverReferenceStream message;
    {
        std::lock_guard<std::mutex> lock(reference_stream_mutex_);
        std::lock_guard<std::mutex> binding_lock(
            reference_callback_struct_->publication_mutex_);
        const auto current_binding = reference_callback_struct_->snapshot();
        if (
            !reference_stream_state_.valid ||
            reference_stream_state_.stream_id != stream_id ||
            reference_stream_state_.execution_id != binding.execution_id ||
            reference_stream_state_.request_identity != binding.request_identity ||
            current_binding.execution_id != binding.execution_id ||
            current_binding.request_identity != binding.request_identity ||
            current_binding.revision != binding.revision ||
            (binding.lease && binding.lease->retired())
        ) {
            return;
        }
        if (!prepared && !paused && reference_stream_state_.paused) {
            return;
        }
        reference_stream_state_.latest_reference = reference;
        message.stream_id = stream_id;
        message.request_identity = binding.request_identity;
        message.sequence = ++reference_stream_state_.sequence;
        reference_stream_state_.recent_references.emplace_back(message.sequence, reference);
        while (reference_stream_state_.recent_references.size() > 32) {
            reference_stream_state_.recent_references.pop_front();
        }
        while (reference_stream_state_.object_tracking_sequences.size() > 32) {
            reference_stream_state_.object_tracking_sequences.pop_front();
        }
        const auto now = node_->now();
        message.produced_at = now;
        message.valid_until = now + rclcpp::Duration::from_nanoseconds(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::milliseconds(configuration_->GetParameter(
                    "/control/maneuver_controller/reference_stream_timeout_ms"
                ).as_int())
            ).count()
        );
        message.trajectory_time_s = std::chrono::duration<double>(
            steady_now - reference_stream_state_.generation_started
        ).count();
        message.provider = binding.reference_provider_name;
        message.terminal_hold_active = static_cast<bool>(terminal_hold);
        message.object_tracking_active = object_tracking && !object_unrecoverable;
        message.state = object_unrecoverable
            ? iii_drone_interfaces::msg::ManeuverReferenceStream::STATE_TERMINAL_UNRECOVERABLE
            : (object_stopped && !reference_stream_state_.prepared &&
                !reference_stream_state_.paused &&
                !reference_stream_state_.committed_waiting_for_applied
            ? iii_drone_interfaces::msg::ManeuverReferenceStream::STATE_OBJECT_STOPPED
            : (object_stopping && !reference_stream_state_.prepared &&
                !reference_stream_state_.paused &&
                !reference_stream_state_.committed_waiting_for_applied
            ? iii_drone_interfaces::msg::ManeuverReferenceStream::STATE_OBJECT_STOPPING
            : (terminal_hold &&
            terminal_hold->phase() == TerminalTrackingHold::Phase::Unrecoverable
            ? iii_drone_interfaces::msg::ManeuverReferenceStream::STATE_TERMINAL_UNRECOVERABLE
            : (terminal_hold &&
            terminal_hold->phase() == TerminalTrackingHold::Phase::Degraded
            ? iii_drone_interfaces::msg::ManeuverReferenceStream::STATE_TERMINAL_DEGRADED
            : (reference_stream_state_.prepared
            ? iii_drone_interfaces::msg::ManeuverReferenceStream::STATE_PREPARED
            : (reference_stream_state_.committed_waiting_for_applied
                ? iii_drone_interfaces::msg::ManeuverReferenceStream::STATE_ACTIVE
            : (reference_stream_state_.paused
                ? iii_drone_interfaces::msg::ManeuverReferenceStream::STATE_PAUSED
                : iii_drone_interfaces::msg::ManeuverReferenceStream::STATE_ACTIVE)))))));
        if (object_tracking && message.state ==
                iii_drone_interfaces::msg::ManeuverReferenceStream::STATE_ACTIVE) {
            // STOPPING and STOPPED cannot prove first active object ownership.
            reference_stream_state_.object_tracking_sequences.push_back(message.sequence);
        }
        message.is_valid = true;
        message.reference = ReferenceAdapter(reference).ToMsg();
        // Stream state, both reference topics, and replacement of this
        // callable share the stream -> binding publication boundary.
        reference_stream_publisher_->publish(message);
        reference_publisher_->publish(message.reference);
    }
    auto event = iii_drone::diagnostics::HilTrace::event("reference_stream_published");
    event.text("stream_id", message.stream_id);
    event.number("sequence", message.sequence);
    event.number("state", message.state);
    event.signed_number(
        "produced_at_ns",
        static_cast<int64_t>(message.produced_at.sec) * 1000000000LL + message.produced_at.nanosec);
    event.signed_number(
        "valid_until_ns",
        static_cast<int64_t>(message.valid_until.sec) * 1000000000LL + message.valid_until.nanosec);
    event.decimal("trajectory_time_s", message.trajectory_time_s);
    event.text("provider", message.provider);
    event.commit();
}

void ManeuverScheduler::acknowledgeReferenceStream(
    const iii_drone_interfaces::msg::ManeuverReferenceAck::SharedPtr message
) {
    auto received = iii_drone::diagnostics::HilTrace::event("reference_ack_received");
    received.text("stream_id", message->stream_id);
    received.number("last_applied_sequence", message->last_applied_sequence);
    received.number("consumer_status", message->consumer_status);
    received.commit();
    if (message->consumer_status ==
            iii_drone_interfaces::msg::ManeuverReferenceAck::STATUS_OBJECT_STOP_REQUESTED) {
        // A stop request is control intent, not evidence that this sequence
        // was newly applied. Never advance the ordinary ACK clock or status.
        const auto binding = reference_callback_struct_->snapshot();
        const auto hover_entry = registered_maneuvers_.find(MANEUVER_TYPE_HOVER_BY_OBJECT);
        const auto fly_entry = registered_maneuvers_.find(MANEUVER_TYPE_FLY_TO_OBJECT);
        const bool hover_owner = hover_entry != registered_maneuvers_.end() &&
            std::static_pointer_cast<HoverByObjectManeuverServer>(hover_entry->second)
                ->RetainsTrackedSource(binding);
        const bool fly_owner = !hover_owner && fly_entry != registered_maneuvers_.end() &&
            std::static_pointer_cast<FlyToObjectManeuverServer>(fly_entry->second)
                ->RetainsTrackedSource(binding);
        if (!binding.callback || (!hover_owner && !fly_owner)) return;
        const auto now = std::chrono::steady_clock::now();
        const auto max_ack_age = std::chrono::milliseconds(configuration_->GetParameter(
            "/control/maneuver_controller/reference_stream_timeout_ms").as_int());
        bool accepted = false;
        {
            std::lock_guard<std::mutex> lock(reference_stream_mutex_);
            auto & stream = reference_stream_state_;
            if (stream.valid && !stream.paused && !stream.prepared &&
                !stream.committed_waiting_for_applied && !stream.claim_ack_pending &&
                stream.stream_id == message->stream_id &&
                stream.request_identity == binding.request_identity &&
                stream.execution_id == binding.execution_id &&
                stream.execution_id == current_reference_execution_id_.Load() &&
                (stream.claimed_consumer_identity.empty() ||
                 stream.claimed_consumer_identity == message->consumer_identity) &&
                message->last_applied_sequence != 0 &&
                stream.last_ack_sequence == message->last_applied_sequence &&
                stream.last_consumer_status ==
                    iii_drone_interfaces::msg::ManeuverReferenceAck::STATUS_APPLIED &&
                stream.last_ack_reference_valid && now >= stream.last_ack &&
                now - stream.last_ack <= max_ack_age &&
                std::find(stream.object_tracking_sequences.begin(),
                    stream.object_tracking_sequences.end(),
                    message->last_applied_sequence) !=
                        stream.object_tracking_sequences.end()) {
                if (stream.object_stop_requested_sequence == message->last_applied_sequence) {
                    return;  // duplicate for this exact accepted request
                }
                stream.object_stop_requested_sequence = message->last_applied_sequence;
                accepted = true;
            }
        }
        if (!accepted) return;
        const auto current = reference_callback_struct_->snapshot();
        const bool same_owner = current.callback &&
            current.request_identity == binding.request_identity &&
            current.execution_id == binding.execution_id &&
            current.reference_provider_name == binding.reference_provider_name &&
            current.execution_id == current_reference_execution_id_.Load();
        const bool requested = same_owner && (hover_owner
            ? std::static_pointer_cast<HoverByObjectManeuverServer>(hover_entry->second)
                ->RequestTrackedTransitionStop(current)
            : std::static_pointer_cast<FlyToObjectManeuverServer>(fly_entry->second)
                ->RequestTrackedTransitionStop(current));
        if (!requested) {
            std::lock_guard<std::mutex> lock(reference_stream_mutex_);
            if (reference_stream_state_.stream_id == message->stream_id &&
                reference_stream_state_.execution_id == binding.execution_id &&
                reference_stream_state_.object_stop_requested_sequence ==
                    message->last_applied_sequence) {
                reference_stream_state_.object_stop_requested_sequence = 0;
            }
        }
        return;
    }
    ManeuverServer::SharedPtr server_to_pause;
    ManeuverServer::SharedPtr server_to_abort;
    {
        std::lock_guard<std::mutex> lock(reference_stream_mutex_);
        if (
            !reference_stream_state_.valid ||
            message->stream_id != reference_stream_state_.stream_id ||
            message->last_applied_sequence > reference_stream_state_.sequence ||
            (reference_stream_state_.ack_seen &&
                message->last_applied_sequence < reference_stream_state_.last_ack_sequence)
        ) {
            auto rejected = iii_drone::diagnostics::HilTrace::event("reference_ack_rejected");
            rejected.text("stream_id", message->stream_id);
            rejected.number("last_applied_sequence", message->last_applied_sequence);
            rejected.number("producer_sequence", reference_stream_state_.sequence);
            rejected.number("producer_last_ack_sequence", reference_stream_state_.last_ack_sequence);
            rejected.commit();
            return;
        }
        if (!reference_stream_state_.claimed_consumer_identity.empty() &&
            (message->consumer_identity != reference_stream_state_.claimed_consumer_identity ||
             (reference_stream_state_.claim_ack_pending &&
              message->last_applied_sequence <= reference_stream_state_.claim_source_ack_sequence))) {
            return;
        }
        std::optional<Reference> acknowledged_reference;
        for (const auto & [sequence, command] : reference_stream_state_.recent_references) {
            if (sequence == message->last_applied_sequence) {
                acknowledged_reference = command;
                break;
            }
        }
        if (message->consumer_status ==
                iii_drone_interfaces::msg::ManeuverReferenceAck::STATUS_APPLIED &&
            !acknowledged_reference) {
            return;
        }
        const uint64_t previous_ack_sequence = reference_stream_state_.last_ack_sequence;
        reference_stream_state_.ack_seen = true;
        reference_stream_state_.last_ack_sequence = message->last_applied_sequence;
        reference_stream_state_.last_ack_reference_valid = acknowledged_reference.has_value();
        if (acknowledged_reference) reference_stream_state_.last_ack_reference = *acknowledged_reference;
        reference_stream_state_.last_consumer_status = message->consumer_status;
        reference_stream_state_.last_ack = std::chrono::steady_clock::now();
        const Maneuver acknowledged_maneuver = current_maneuver_;
        if (message->consumer_status ==
                iii_drone_interfaces::msg::ManeuverReferenceAck::STATUS_APPLIED &&
            acknowledged_maneuver.started() && !acknowledged_maneuver.terminated() &&
            acknowledged_maneuver.requestIdentity() == reference_stream_state_.request_identity) {
            const auto hover_entry = registered_maneuvers_.find(MANEUVER_TYPE_HOVER);
            if (hover_entry != registered_maneuvers_.end()) {
                auto hover = std::static_pointer_cast<HoverManeuverServer>(hover_entry->second);
                const auto owner = hover->terminalHoldBinding();
                if (owner.hold && owner.request_identity != reference_stream_state_.request_identity) {
                    hover->ClearTerminalHold();
                }
            }
        }
        if (reference_stream_state_.claim_ack_pending &&
            message->consumer_status == iii_drone_interfaces::msg::ManeuverReferenceAck::STATUS_APPLIED) {
            reference_stream_state_.claim_ack_pending = false;
            auto & native_epoch = retained_native_hold_epoch_;
            if (native_epoch.completed &&
                native_epoch.stream_id == reference_stream_state_.stream_id &&
                native_epoch.request_identity == reference_stream_state_.request_identity &&
                native_epoch.execution_id == reference_stream_state_.execution_id &&
                native_epoch.claimed_consumer_identity == message->consumer_identity) {
                native_epoch.claimed_applied = true;
            }
        }
        auto processed = iii_drone::diagnostics::HilTrace::event("reference_ack_processed");
        processed.text("stream_id", message->stream_id);
        processed.number("last_applied_sequence", message->last_applied_sequence);
        processed.number("previous_ack_sequence", previous_ack_sequence);
        processed.boolean("sequence_advanced", message->last_applied_sequence > previous_ack_sequence);
        processed.number("consumer_status", message->consumer_status);
        processed.commit();
        if (
            reference_stream_state_.abort_waiting_for_consumer_ready &&
            message->consumer_status ==
                iii_drone_interfaces::msg::ManeuverReferenceAck::STATUS_ACTION_ABORT_READY
        ) {
            reference_stream_state_.abort_waiting_for_consumer_ready = false;
            reference_stream_state_.paused = false;
            server_to_abort = activeManeuverServer();
        }
        if (reference_stream_state_.committed_waiting_for_applied) {
            if (
                message->consumer_status ==
                    iii_drone_interfaces::msg::ManeuverReferenceAck::STATUS_APPLIED
            ) {
                const ManeuverServer::SharedPtr server = activeManeuverServer();
                if (server) {
                    auto event = iii_drone::diagnostics::HilTrace::event("reference_rebase_commit_begin");
                    event.text("stream_id", message->stream_id);
                    event.number("sequence", message->last_applied_sequence);
                    event.commit();
                    server->CommitReferenceStreamRebase();
                    auto result = iii_drone::diagnostics::HilTrace::event("reference_rebase_commit_end");
                    result.text("stream_id", message->stream_id);
                    result.number("sequence", message->last_applied_sequence);
                    result.commit();
                    reference_stream_state_.committed_waiting_for_applied = false;
                    reference_stream_state_.paused = false;
                    reference_stream_state_.generation_started =
                        std::chrono::steady_clock::now();
                    RCLCPP_INFO(
                        node_->get_logger(),
                        "Reference stream %s received first applied acknowledgement at "
                        "sequence %lu; releasing rebased maneuver.",
                        reference_stream_state_.stream_id.c_str(),
                        static_cast<unsigned long>(message->last_applied_sequence)
                    );
                }
            }
            return;
        }
        if (
            !server_to_abort &&
            message->consumer_status !=
                iii_drone_interfaces::msg::ManeuverReferenceAck::STATUS_APPLIED &&
            !reference_stream_state_.paused
        ) {
            reference_stream_state_.paused = true;
            server_to_pause = activeManeuverServer();
        }
    }
    if (server_to_pause) {
        auto event = iii_drone::diagnostics::HilTrace::event("reference_pause_server_call");
        event.text("reason", "ack_status_not_applied");
        event.commit();
        server_to_pause->PauseReferenceStream();
    }
    if (server_to_abort) {
        auto event = iii_drone::diagnostics::HilTrace::event("reference_abort_server_call");
        event.commit();
        server_to_abort->AbortAfterReferenceLoss();
    }
}

bool ManeuverScheduler::blendedReferenceApplied(const std::string & request_identity) {
    // Read the execution ID before the stream lock, matching the ordering in
    // beginReferenceExecution. Do not acquire callback or maneuver locks here.
    // The first ACK may cover the current goal's initialization hold; that is
    // enough to establish consumer ownership of this request and generation.
    const uint64_t current_execution_id = current_reference_execution_id_.Load();
    std::lock_guard<std::mutex> lock(reference_stream_mutex_);
    return !request_identity.empty() &&
        reference_stream_state_.valid &&
        reference_stream_state_.request_identity == request_identity &&
        reference_stream_state_.execution_id == current_execution_id &&
        reference_stream_state_.ack_seen &&
        reference_stream_state_.last_ack_sequence > 0 &&
        reference_stream_state_.last_consumer_status ==
            iii_drone_interfaces::msg::ManeuverReferenceAck::STATUS_APPLIED &&
        !reference_stream_state_.paused &&
        !reference_stream_state_.prepared &&
        !reference_stream_state_.committed_waiting_for_applied;
}

bool ManeuverScheduler::firstObjectReferenceApplied(const std::string & request_identity) {
    return firstManeuverReferenceApplied(MANEUVER_TYPE_FLY_TO_OBJECT, request_identity);
}

bool ManeuverScheduler::firstTerminalHoverReferenceApplied(
    const std::string & request_identity
) {
    const auto entry = registered_maneuvers_.find(MANEUVER_TYPE_HOVER);
    if (entry == registered_maneuvers_.end()) return false;
    const auto owner = std::static_pointer_cast<HoverManeuverServer>(entry->second)
        ->terminalHoldBinding();
    if (!owner.hold || owner.request_identity != request_identity ||
        owner.hold->phase() != TerminalTrackingHold::Phase::Tracking) return false;
    return firstManeuverReferenceApplied(MANEUVER_TYPE_HOVER, request_identity);
}

bool ManeuverScheduler::firstManeuverReferenceApplied(
    maneuver_type_t maneuver_type, const std::string & request_identity
) {
    if (!isValidManeuverRequestIdentity(request_identity)) return false;
    const Maneuver maneuver = current_maneuver_;
    if (maneuver.maneuver_type() != maneuver_type ||
        !maneuver.started() || maneuver.terminated() ||
        maneuver.requestIdentity() != request_identity) return false;
    const auto server = registered_maneuvers_.find(maneuver_type);
    if (server == registered_maneuvers_.end()) return false;
    const auto binding = reference_callback_struct_->snapshot();
    if (!currentReferenceValid(binding) ||
        binding.request_identity != request_identity ||
        binding.reference_provider_name != server->second->action_name()) return false;
    const bool tracked_object_goal =
        (maneuver_type == MANEUVER_TYPE_FLY_TO_OBJECT &&
         std::static_pointer_cast<FlyToObjectManeuverServer>(server->second)
             ->RetainsTrackedSource(binding)) ||
        (maneuver_type == MANEUVER_TYPE_HOVER_BY_OBJECT &&
         std::static_pointer_cast<HoverByObjectManeuverServer>(server->second)
             ->RetainsTrackedSource(binding));

    const auto now = std::chrono::steady_clock::now();
    const auto max_ack_age = std::chrono::milliseconds(configuration_->GetParameter(
        "/control/maneuver_controller/reference_stream_timeout_ms").as_int());
    std::lock_guard<std::mutex> lock(reference_stream_mutex_);
    const auto & stream = reference_stream_state_;
    if (!stream.valid || stream.paused || stream.prepared ||
        stream.committed_waiting_for_applied ||
        stream.abort_waiting_for_consumer_ready || stream.claim_ack_pending ||
        !stream.claimed_consumer_identity.empty() ||
        stream.stream_id.empty() || stream.provider != server->second->action_name() ||
        stream.request_identity != request_identity ||
        stream.execution_id == 0 || stream.execution_id != binding.execution_id ||
        stream.execution_id != current_reference_execution_id_.Load() ||
        !stream.ack_seen || stream.sequence == 0 || stream.last_ack_sequence == 0 ||
        stream.last_ack_sequence > stream.sequence ||
        stream.last_consumer_status !=
            iii_drone_interfaces::msg::ManeuverReferenceAck::STATUS_APPLIED ||
        !stream.last_ack_reference_valid ||
        stream.last_ack < stream.generation_started || now < stream.last_ack ||
        now - stream.last_ack > max_ack_age) return false;
    const bool known_published_sequence = std::any_of(
        stream.recent_references.begin(), stream.recent_references.end(),
        [&stream](const auto & entry) {
            return entry.first == stream.last_ack_sequence;
        });
    // The consumer has already applied this exact published command. Its
    // channel shape can legitimately be position-only during initialization.
    return known_published_sequence && (!tracked_object_goal ||
        std::find(stream.object_tracking_sequences.begin(),
            stream.object_tracking_sequences.end(), stream.last_ack_sequence) !=
                stream.object_tracking_sequences.end());
}

bool ManeuverScheduler::appliedFiniteRestReference(
    const std::string & request_identity, const Reference & command
) {
    if (request_identity.empty() || !command.position().allFinite() ||
        !command.velocity().allFinite() || !command.acceleration().allFinite() ||
        !std::isfinite(command.yaw()) || !std::isfinite(command.yaw_rate()) ||
        !std::isfinite(command.yaw_acceleration()) ||
        command.velocity().norm() > 1.0e-5 ||
        command.acceleration().norm() > 1.0e-5 ||
        std::abs(command.yaw_rate()) > 1.0e-5 ||
        std::abs(command.yaw_acceleration()) > 1.0e-5) return false;
    const auto binding = reference_callback_struct_->snapshot();
    if (!currentReferenceValid(binding) || binding.request_identity != request_identity) return false;
    const auto now = std::chrono::steady_clock::now();
    const auto max_ack_age = std::chrono::milliseconds(configuration_->GetParameter(
        "/control/maneuver_controller/reference_stream_timeout_ms").as_int());
    std::lock_guard<std::mutex> lock(reference_stream_mutex_);
    const auto & stream = reference_stream_state_;
    if (!stream.valid || stream.paused || stream.prepared ||
        stream.committed_waiting_for_applied || !stream.ack_seen ||
        stream.request_identity != request_identity ||
        stream.execution_id != binding.execution_id ||
        stream.last_consumer_status !=
            iii_drone_interfaces::msg::ManeuverReferenceAck::STATUS_APPLIED ||
        stream.last_ack_sequence == 0 || !stream.last_ack_reference_valid ||
        now - stream.last_ack > max_ack_age) return false;
    const auto & applied = stream.last_ack_reference;
    return applied.position().allFinite() && applied.velocity().allFinite() &&
        applied.acceleration().allFinite() && std::isfinite(applied.yaw()) &&
        std::isfinite(applied.yaw_rate()) &&
        std::isfinite(applied.yaw_acceleration()) &&
        (applied.position() - command.position()).norm() <= 1.0e-5 &&
        (applied.velocity() - command.velocity()).norm() <= 1.0e-5 &&
        (applied.acceleration() - command.acceleration()).norm() <= 1.0e-5 &&
        std::abs(std::atan2(std::sin(applied.yaw() - command.yaw()),
                            std::cos(applied.yaw() - command.yaw()))) <= 1.0e-5 &&
        std::abs(applied.yaw_rate() - command.yaw_rate()) <= 1.0e-5 &&
        std::abs(applied.yaw_acceleration() - command.yaw_acceleration()) <= 1.0e-5;
}

std::shared_ptr<TerminalTrackingHold> ManeuverScheduler::retainedTerminalHold() const {
    const auto entry = registered_maneuvers_.find(MANEUVER_TYPE_HOVER);
    if (entry == registered_maneuvers_.end()) return nullptr;
    return std::static_pointer_cast<HoverManeuverServer>(entry->second)->terminalHold();
}

bool ManeuverScheduler::retireCompletedTerminalHoldAfterNativeHold() {
    const auto hover_entry = registered_maneuvers_.find(MANEUVER_TYPE_HOVER);
    if (hover_entry == registered_maneuvers_.end()) return false;
    const auto hover = std::static_pointer_cast<HoverManeuverServer>(hover_entry->second);

    // stream -> Hover -> callback matches the established ACK/publish order.
    // The lock also excludes a concurrent QUERY/CLAIM while the offer is
    // withdrawn. No measured fallback or replacement command is emitted.
    // Any fresh PX4-native navigation state (Hold, Land, ...) ends external
    // command authority; an active maneuver is never retired here.
    std::lock_guard<std::mutex> lock(reference_stream_mutex_);
    const auto navigation = combined_drone_awareness_handler_->GetVehicleNavigationEvidence();
    const auto now = std::chrono::steady_clock::now();
    if (!freshNativeNavigation(navigation, now)) return false;
    const Maneuver maneuver = current_maneuver_;
    if (maneuver.started() && !maneuver.terminated()) return false;
    const auto & epoch = retained_native_hold_epoch_;
    const auto & stream = reference_stream_state_;
    const auto owner = hover->terminalHoldBinding();
    const auto binding = reference_callback_struct_->snapshot();
    const auto object_entry = registered_maneuvers_.find(MANEUVER_TYPE_HOVER_BY_OBJECT);
    const auto fly_entry = registered_maneuvers_.find(MANEUVER_TYPE_FLY_TO_OBJECT);
    const bool terminal_owner = owner.hold &&
        owner.request_identity == epoch.request_identity;
    const bool object_owner = !terminal_owner &&
        object_entry != registered_maneuvers_.end() &&
        ((binding.reference_provider_name == object_entry->second->action_name()) ||
         (fly_entry != registered_maneuvers_.end() &&
          binding.reference_provider_name == fly_entry->second->action_name())) &&
        std::static_pointer_cast<HoverByObjectManeuverServer>(object_entry->second)
            ->RetainsTrackedSource(binding) &&
        stream.provider == binding.reference_provider_name;
    // A completed non-sustained hover callback has no successor transfer; a
    // pending successor keeps it until that successor begins.
    const bool idle_owner = !terminal_owner && !object_owner &&
        epoch.idle_callback && maneuver.maneuver_type() == MANEUVER_TYPE_NONE &&
        stream.provider == binding.reference_provider_name;
    if (!epoch.completed || (!terminal_owner && !object_owner && !idle_owner) ||
        !isValidManeuverRequestIdentity(epoch.request_identity) ||
        binding.request_identity != epoch.request_identity || !binding.callback ||
        binding.execution_id == 0 || binding.execution_id != epoch.execution_id ||
        current_reference_execution_id_.Load() != epoch.execution_id ||
        !maneuver_server_get_reference_callback_still_registered_.Load() ||
        !stream.valid || stream.stream_id.empty() ||
        (!idle_owner && stream.stream_id != epoch.stream_id) ||
        stream.execution_id != epoch.execution_id ||
        stream.request_identity != epoch.request_identity ||
        stream.claim_ack_pending ||
        stream.claimed_consumer_identity != epoch.claimed_consumer_identity ||
        !epoch.claimed_applied ||
        navigation.source_epoch != epoch.status_source_epoch ||
        navigation.latest->receipt <= epoch.owner_started ||
        (epoch.external_status_timestamp_us == 0 &&
            epoch.claimed_consumer_identity.empty())) return false;

    uint64_t external_stamp = epoch.external_status_timestamp_us;
    uint64_t external_transition = epoch.external_nav_transition_us;
    if (!epoch.claimed_consumer_identity.empty()) {
        // A consumer CLAIM is a new owner epoch. Old external status cannot
        // arm it. A positive PX4 external nav transition newer than every
        // earlier owner's is required: either observed after the claim, or
        // the claimant's own transition already observed fresh at CLAIM time.
        if (navigation.last_external &&
            navigation.last_external->nav_state_timestamp_us >
                epoch.minimum_external_transition_us) {
            external_stamp = navigation.last_external->source_timestamp_us;
            external_transition = navigation.last_external->nav_state_timestamp_us;
        } else if (!navigation.last_external ||
            navigation.last_external->nav_state_timestamp_us !=
                epoch.external_nav_transition_us) {
            return false;
        }
    } else if (navigation.last_external &&
        navigation.last_external->nav_state_timestamp_us > external_transition) {
        // A newer external mode took control before native Hold. The old
        // source epoch is not entitled to interpret that later transition.
        return false;
    }
    if (external_stamp == 0 || external_transition == 0 ||
        navigation.latest->nav_state_timestamp_us <= external_stamp ||
        navigation.latest->source_timestamp_us <
            navigation.latest->nav_state_timestamp_us) return false;

    const auto phase = terminal_owner ? owner.hold->phase() :
        TerminalTrackingHold::Phase::Tracking;
    const std::string failure_reason = terminal_owner ? owner.hold->failureReason() : "";
    const std::string request_identity = epoch.request_identity;
    const std::string stream_id = stream.stream_id;
    const uint64_t execution_id = epoch.execution_id;
    const bool succeeded = epoch.succeeded;
    const std::string consumer_identity = epoch.claimed_consumer_identity;
    const char * owner_label = terminal_owner ? "terminal hold" :
        (object_owner ? "object session" : "idle hover callback");
    if (terminal_owner) {
        hover->ClearTerminalHold();
    } else if (object_owner) {
        std::static_pointer_cast<HoverByObjectManeuverServer>(object_entry->second)
            ->RetireTrackedSource(binding);
    }
    clearRetainedOwnerLocked("native_hold_retired", execution_id, request_identity);

    RCLCPP_INFO(node_->get_logger(),
        "Completed %s owner retired after fresh PX4 native navigation state %u "
        "(request=%s execution=%lu stream=%s consumer=%s prior_phase=%u "
        "prior_success=%d prior_reason=%s source_us=%lu nav_transition_us=%lu status_epoch=%lu)",
        owner_label, static_cast<unsigned>(navigation.latest->nav_state),
        request_identity.c_str(), static_cast<unsigned long>(execution_id),
        stream_id.c_str(), consumer_identity.c_str(), static_cast<unsigned>(phase),
        succeeded, failure_reason.c_str(),
        static_cast<unsigned long>(navigation.latest->source_timestamp_us),
        static_cast<unsigned long>(navigation.latest->nav_state_timestamp_us),
        static_cast<unsigned long>(navigation.source_epoch));
    auto event = iii_drone::diagnostics::HilTrace::event("terminal_hold_retired_native_hold");
    event.text("request_identity", request_identity);
    event.text("owner_type", terminal_owner ? "terminal_hold" :
        (object_owner ? "object_session" : "idle_hover_callback"));
    event.number("nav_state", navigation.latest->nav_state);
    event.text("stream_id", stream_id);
    event.text("consumer_identity", consumer_identity);
    event.number("execution_id", execution_id);
    event.number("prior_phase", static_cast<uint64_t>(phase));
    event.boolean("prior_success", succeeded);
    event.text("prior_reason", failure_reason);
    event.number("source_timestamp_us", navigation.latest->source_timestamp_us);
    event.number("nav_state_timestamp_us", navigation.latest->nav_state_timestamp_us);
    event.commit();
    return true;
}

void ManeuverScheduler::clearRetainedOwnerLocked(
    const char * provider_label, uint64_t execution_id,
    const std::string & request_identity
) {
    reference_callback_struct_->set(nullptr, provider_label,
        execution_id, request_identity);
    maneuver_server_get_reference_callback_still_registered_ = false;
    reference_stream_state_.valid = false;
    reference_stream_state_.offer_consumer_identity.clear();
    reference_stream_state_.offer_ack_sequence = 0;
    reference_stream_state_.claim_ack_pending = false;
    reference_stream_state_.claimed_consumer_identity.clear();
    reference_stream_state_.ack_seen = false;
    reference_stream_state_.last_ack_reference_valid = false;
    reference_stream_state_.recent_references.clear();
    retained_native_hold_epoch_ = RetainedNativeHoldEpoch{};
}

bool ManeuverScheduler::retireReleasedConsumerOwner() {
    const auto hover_entry = registered_maneuvers_.find(MANEUVER_TYPE_HOVER);
    if (hover_entry == registered_maneuvers_.end()) return false;
    const auto hover = std::static_pointer_cast<HoverManeuverServer>(hover_entry->second);

    // Same stream -> Hover -> callback order as native-Hold retirement. No
    // navigation evidence is needed: the consumer explicitly released.
    std::lock_guard<std::mutex> lock(reference_stream_mutex_);
    if (!released_consumer_scope_) return false;
    const ManeuverRequestScope scope = *released_consumer_scope_;
    const Maneuver maneuver = current_maneuver_;
    // An executing goal ends through its own server (CONSUMER_RELEASED); its
    // retained successor state, if any, is retired on a later tick.
    if (maneuver.started() && !maneuver.terminated()) return false;
    const auto binding = reference_callback_struct_->snapshot();
    if (!binding.callback || binding.execution_id == 0 ||
        !scope.contains(binding.request_identity) ||
        binding.execution_id != current_reference_execution_id_.Load() ||
        !reference_callback_token_.master_has_token()) return false;

    const auto owner = hover->terminalHoldBinding();
    const auto object_entry = registered_maneuvers_.find(MANEUVER_TYPE_HOVER_BY_OBJECT);
    const auto fly_entry = registered_maneuvers_.find(MANEUVER_TYPE_FLY_TO_OBJECT);
    const bool terminal_owner = owner.hold &&
        owner.request_identity == binding.request_identity;
    const bool object_owner = !terminal_owner &&
        object_entry != registered_maneuvers_.end() &&
        ((binding.reference_provider_name == object_entry->second->action_name()) ||
         (fly_entry != registered_maneuvers_.end() &&
          binding.reference_provider_name == fly_entry->second->action_name())) &&
        std::static_pointer_cast<HoverByObjectManeuverServer>(object_entry->second)
            ->RetainsTrackedSource(binding);
    const char * owner_label = terminal_owner ? "terminal hold" :
        (object_owner ? "object session" : "retained callback");
    if (terminal_owner) {
        hover->ClearTerminalHold();
    } else if (object_owner) {
        std::static_pointer_cast<HoverByObjectManeuverServer>(object_entry->second)
            ->RetireTrackedSource(binding);
    }
    const std::string stream_id = reference_stream_state_.stream_id;
    clearRetainedOwnerLocked("consumer_released", binding.execution_id,
        binding.request_identity);

    RCLCPP_INFO(node_->get_logger(),
        "Mission Exit: retired released %s owner (request=%s execution=%lu stream=%s provider=%s)",
        owner_label, binding.request_identity.c_str(),
        static_cast<unsigned long>(binding.execution_id), stream_id.c_str(),
        binding.reference_provider_name.c_str());
    auto event = iii_drone::diagnostics::HilTrace::event("retained_owner_retired_consumer_release");
    event.text("request_identity", binding.request_identity);
    event.text("owner_type", terminal_owner ? "terminal_hold" :
        (object_owner ? "object_session" : "retained_callback"));
    event.text("provider", binding.reference_provider_name);
    event.text("stream_id", stream_id);
    event.number("execution_id", binding.execution_id);
    event.commit();
    return true;
}

void ManeuverScheduler::releaseConsumerControl(
    const std::shared_ptr<iii_drone_interfaces::srv::ReleaseConsumerControl::Request> request,
    std::shared_ptr<iii_drone_interfaces::srv::ReleaseConsumerControl::Response> response
) {
    using Release = iii_drone_interfaces::srv::ReleaseConsumerControl;
    ManeuverRequestScope scope;
    scope.epoch = request->producer_epoch;
    scope.last_counter = request->last_request_counter;
    if (!scope.valid()) {
        response->accepted = false;
        response->reason = "malformed consumer scope";
        RCLCPP_WARN(node_->get_logger(),
            "ManeuverScheduler::releaseConsumerControl(): Refusing malformed consumer scope");
        return;
    }
    std::unique_lock<std::shared_mutex> lck(maneuver_mutex_);
    {
        std::lock_guard<std::mutex> lock(reference_stream_mutex_);
        released_consumer_scope_ = scope;
    }
    for (auto & entry : registered_maneuvers_) {
        entry.second->ReleaseConsumerScope(scope);
    }
    response->cleared_queued_count =
        maneuver_queue_ ? maneuver_queue_->ClearRequestScope(scope) : 0U;
    const Maneuver current = current_maneuver_;
    response->released_active_count =
        current.started() && !current.terminated() &&
        scope.contains(current.requestIdentity()) ? 1U : 0U;
    response->retired_owner_count = retireReleasedConsumerOwner() ? 1U : 0U;
    response->accepted = true;
    response->reason = "consumer released";

    const char * reason_label =
        request->reason == Release::Request::REASON_OPERATOR_MODE_CHANGE ? "operator mode change" :
        request->reason == Release::Request::REASON_OPERATOR_STICK_OVERRIDE ? "operator stick override" :
        request->reason == Release::Request::REASON_FAILSAFE ? "failsafe" : "other";
    RCLCPP_INFO(node_->get_logger(),
        "Mission Exit: consumer %s released control up to request %lu (%s, PX4 nav_state %u): "
        "%u queued cleared, %u executing released, %u retained owner retired",
        scope.epoch.c_str(), static_cast<unsigned long>(scope.last_counter), reason_label,
        static_cast<unsigned>(request->px4_nav_state),
        response->cleared_queued_count, response->released_active_count,
        response->retired_owner_count);
    auto event = iii_drone::diagnostics::HilTrace::event("consumer_control_released");
    event.text("producer_epoch", scope.epoch);
    event.number("last_request_counter", scope.last_counter);
    event.text("reason", reason_label);
    event.number("px4_nav_state", request->px4_nav_state);
    event.number("cleared_queued_count", response->cleared_queued_count);
    event.number("released_active_count", response->released_active_count);
    event.number("retired_owner_count", response->retired_owner_count);
    event.commit();
}

void ManeuverScheduler::terminalHoldTransfer(
    const std::shared_ptr<iii_drone_interfaces::srv::TerminalHoldTransfer::Request> request,
    std::shared_ptr<iii_drone_interfaces::srv::TerminalHoldTransfer::Response> response
) {
    using Transfer = iii_drone_interfaces::srv::TerminalHoldTransfer;
    (void)retireCompletedTerminalHoldAfterNativeHold();
    const auto hover = std::static_pointer_cast<HoverManeuverServer>(
        registered_maneuvers_.at(MANEUVER_TYPE_HOVER));
    // Token return, successor begin, and native-Hold retirement publish their
    // owner/binding/completion generations under this lock. A transfer must
    // classify and mutate one generation, including the completion transient.
    std::lock_guard<std::mutex> lock(reference_stream_mutex_);
    // A halt's QUERY can precede the scheduler tick that sees its successor end.
    installUnexecutedSuccessorHoldLocked();
    const auto owner = hover->terminalHoldBinding();
    const auto hold = owner.hold;
    const Maneuver source_maneuver = current_maneuver_;
    const auto binding = reference_callback_struct_->snapshot();
    const uint64_t current_execution_id = current_reference_execution_id_.Load();
    const bool completion_valid =
        maneuver_server_get_reference_callback_still_registered_.Load();
    const bool master_has_token = reference_callback_token_.master_has_token();
    const auto source_entry = registered_maneuvers_.find(source_maneuver.maneuver_type());
    const bool active_source = source_maneuver.maneuver_type() != MANEUVER_TYPE_NONE &&
        source_maneuver.started() && !source_maneuver.terminated() &&
        binding.request_identity == source_maneuver.requestIdentity();
    const bool current_reference_valid = binding.callback &&
        isValidManeuverRequestIdentity(binding.request_identity) &&
        binding.execution_id == current_execution_id &&
        (active_source
            ? source_entry != registered_maneuvers_.end() &&
                binding.reference_provider_name == source_entry->second->action_name()
            : completion_valid);
    const auto reject = [&](const std::string & reason) {
        response->reason = reason;
        auto event = iii_drone::diagnostics::HilTrace::event("terminal_hold_transfer_rejected");
        event.text("reason", reason);
        event.number("operation", request->operation);
        event.text("owner_request_identity", owner.request_identity);
        event.text("binding_request_identity", binding.request_identity);
        event.number("binding_execution_id", binding.execution_id);
        event.number("current_execution_id", current_execution_id);
        event.text("binding_provider", binding.reference_provider_name);
        event.boolean("binding_callback", static_cast<bool>(binding.callback));
        event.text("maneuver_request_identity", source_maneuver.requestIdentity());
        event.number("maneuver_type", static_cast<int>(source_maneuver.maneuver_type()));
        event.boolean("maneuver_started", source_maneuver.started());
        event.boolean("maneuver_terminated", source_maneuver.terminated());
        event.boolean("master_has_token", master_has_token);
        event.boolean("post_action_valid", completion_valid);
        event.boolean("current_reference_valid", current_reference_valid);
        const auto & rejected_stream = reference_stream_state_;
        const auto rejected_now = std::chrono::steady_clock::now();
        const auto ack_age_ms = rejected_stream.ack_seen
            ? std::chrono::duration_cast<std::chrono::milliseconds>(
                rejected_now - rejected_stream.last_ack).count() : -1;
        const auto generation_age_ms =
            rejected_stream.generation_started != std::chrono::steady_clock::time_point{}
                ? std::chrono::duration_cast<std::chrono::milliseconds>(
                    rejected_now - rejected_stream.generation_started).count() : -1;
        event.text("stream_id", rejected_stream.stream_id);
        event.text("stream_request_identity", rejected_stream.request_identity);
        event.number("stream_execution_id", rejected_stream.execution_id);
        event.boolean("stream_valid", rejected_stream.valid);
        event.number("stream_sequence", rejected_stream.sequence);
        event.boolean("ack_seen", rejected_stream.ack_seen);
        event.number("ack_status", rejected_stream.last_consumer_status);
        event.number("ack_sequence", rejected_stream.last_ack_sequence);
        event.boolean("ack_reference_valid", rejected_stream.last_ack_reference_valid);
        event.signed_number("ack_age_ms", ack_age_ms);
        event.signed_number("generation_age_ms", generation_age_ms);
        event.boolean("stream_paused", rejected_stream.paused);
        event.boolean("stream_prepared", rejected_stream.prepared);
        event.boolean("stream_committed_waiting", rejected_stream.committed_waiting_for_applied);
        event.boolean("stream_abort_waiting", rejected_stream.abort_waiting_for_consumer_ready);
        event.boolean("claim_ack_pending", rejected_stream.claim_ack_pending);
        event.text("claimed_consumer_identity", rejected_stream.claimed_consumer_identity);
        event.commit();
        if (reason == "no retained terminal hold" ||
            reason == "terminal action has not completed" ||
            reason == "terminal callback finalizing" ||
            reason == "terminal generation awaiting first applied acknowledgement") {
            RCLCPP_DEBUG(node_->get_logger(),
                "Terminal hold transfer pending: %s (operation=%u owner=%s binding=%s "
                "execution=%lu current=%lu provider=%s maneuver=%d started=%d terminated=%d "
                "master_token=%d post_action_valid=%d current_valid=%d)",
                reason.c_str(), static_cast<unsigned>(request->operation),
                owner.request_identity.c_str(), binding.request_identity.c_str(),
                static_cast<unsigned long>(binding.execution_id),
                static_cast<unsigned long>(current_execution_id),
                binding.reference_provider_name.c_str(),
                static_cast<int>(source_maneuver.maneuver_type()),
                source_maneuver.started(), source_maneuver.terminated(),
                master_has_token, completion_valid, current_reference_valid);
        } else {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                "Terminal hold transfer rejected: %s (operation=%u owner=%s binding=%s "
                "execution=%lu current=%lu provider=%s maneuver=%d started=%d terminated=%d "
                "master_token=%d post_action_valid=%d current_valid=%d "
                "stream=%s stream_valid=%d stream_execution=%lu stream_seq=%lu "
                "ack_seen=%d ack_status=%u ack_seq=%lu ack_ref_valid=%d ack_age_ms=%lld "
                "generation_age_ms=%lld paused=%d prepared=%d commit_wait=%d abort_wait=%d "
                "claim_wait=%d claimed=%s)",
                reason.c_str(), static_cast<unsigned>(request->operation),
                owner.request_identity.c_str(), binding.request_identity.c_str(),
                static_cast<unsigned long>(binding.execution_id),
                static_cast<unsigned long>(current_execution_id),
                binding.reference_provider_name.c_str(),
                static_cast<int>(source_maneuver.maneuver_type()),
                source_maneuver.started(), source_maneuver.terminated(),
                master_has_token, completion_valid, current_reference_valid,
                rejected_stream.stream_id.c_str(), rejected_stream.valid,
                static_cast<unsigned long>(rejected_stream.execution_id),
                static_cast<unsigned long>(rejected_stream.sequence),
                rejected_stream.ack_seen,
                static_cast<unsigned>(rejected_stream.last_consumer_status),
                static_cast<unsigned long>(rejected_stream.last_ack_sequence),
                rejected_stream.last_ack_reference_valid,
                static_cast<long long>(ack_age_ms),
                static_cast<long long>(generation_age_ms),
                rejected_stream.paused, rejected_stream.prepared,
                rejected_stream.committed_waiting_for_applied,
                rejected_stream.abort_waiting_for_consumer_ready,
                rejected_stream.claim_ack_pending,
                rejected_stream.claimed_consumer_identity.c_str());
        }
    };
    if (!hold) {
        reject("no retained terminal hold");
        return;
    }
    if (hold->phase() != TerminalTrackingHold::Phase::Tracking) {
        reject("retained terminal hold degraded");
        return;
    }
    if (source_maneuver.started() && !source_maneuver.terminated()) {
        reject("terminal action has not completed");
        return;
    }
    if (terminal_hold_transfer_after_validity_hook_) {
        terminal_hold_transfer_after_validity_hook_();
    }
    if (!current_reference_valid || owner.request_identity != binding.request_identity) {
        const bool exact_completion_pending = source_maneuver.started() &&
            source_maneuver.terminated() &&
            binding.callback &&
            isValidManeuverRequestIdentity(owner.request_identity) &&
            owner.request_identity == source_maneuver.requestIdentity() &&
            binding.request_identity == owner.request_identity &&
            binding.execution_id != 0 &&
            binding.execution_id == current_execution_id &&
            source_entry != registered_maneuvers_.end() &&
            (binding.reference_provider_name == source_entry->second->action_name() ||
             (!source_maneuver.success() &&
              binding.reference_provider_name == hover->action_name())) &&
            !completion_valid;
        reject(exact_completion_pending
            ? "terminal callback finalizing" : "retained terminal callback is not current");
        return;
    }
    const auto now = std::chrono::steady_clock::now();
    const auto ack_timeout = std::chrono::milliseconds(configuration_->GetParameter(
        "/control/maneuver_controller/reference_stream_timeout_ms").as_int());
    auto & stream = reference_stream_state_;
    const bool exact_stream = stream.valid &&
        stream.execution_id == binding.execution_id &&
        stream.request_identity == binding.request_identity &&
        !stream.stream_id.empty();
    const bool completed_owner =
        (source_maneuver.started() && source_maneuver.terminated() &&
            source_maneuver.requestIdentity() == owner.request_identity) ||
        (retained_native_hold_epoch_.completed &&
            retained_native_hold_epoch_.request_identity == owner.request_identity &&
            retained_native_hold_epoch_.execution_id == binding.execution_id &&
            retained_native_hold_epoch_.stream_id == stream.stream_id);
    const bool published_first_generation = stream.sequence > 0 &&
        !stream.recent_references.empty() &&
        stream.recent_references.back().first == stream.sequence &&
        stream.recent_references.back().second.position().allFinite() &&
        stream.recent_references.back().second.velocity().allFinite() &&
        stream.recent_references.back().second.acceleration().allFinite() &&
        stream.generation_started != std::chrono::steady_clock::time_point{} &&
        now >= stream.generation_started &&
        now - stream.generation_started <= ack_timeout;
    if (request->operation == Transfer::Request::OP_QUERY &&
        completed_owner && exact_stream && published_first_generation &&
        !stream.ack_seen && stream.last_ack_sequence == 0 &&
        !stream.last_ack_reference_valid &&
        !stream.paused && !stream.prepared &&
        !stream.committed_waiting_for_applied &&
        !stream.abort_waiting_for_consumer_ready &&
        !stream.claim_ack_pending && stream.claimed_consumer_identity.empty() &&
        stream.offer_consumer_identity.empty()) {
        reject("terminal generation awaiting first applied acknowledgement");
        return;
    }
    if (!exact_stream || stream.paused || !stream.ack_seen ||
        stream.last_consumer_status !=
            iii_drone_interfaces::msg::ManeuverReferenceAck::STATUS_APPLIED ||
        now - stream.last_ack > ack_timeout || stream.last_ack_sequence == 0 ||
        !stream.last_ack_reference_valid ||
        !stream.last_ack_reference.position().allFinite() ||
        !stream.last_ack_reference.velocity().allFinite() ||
        !stream.last_ack_reference.acceleration().allFinite()) {
        reject("retained hold lacks a fresh applied generation");
        return;
    }
    if (request->operation == Transfer::Request::OP_QUERY) {
        if (stream.claim_ack_pending) {
            reject("terminal hold transfer awaiting successor acknowledgement");
            return;
        }
        if (!request->consumer_identity.empty()) {
            if (!isValidManeuverRequestIdentity(request->consumer_identity)) {
                reject("invalid terminal hold offer consumer");
                return;
            }
            stream.offer_consumer_identity = request->consumer_identity;
            stream.offer_ack_sequence = stream.last_ack_sequence;
            stream.offer_execution_id = stream.execution_id;
            // The consumer CLAIMs after a full QUERY round trip (HIL: the
            // QUERY reply alone took 253 ms); allow the stream's ACK deadline.
            stream.offer_deadline = now + std::chrono::milliseconds(
                configuration_->GetParameter(
                    "/control/maneuver_controller/reference_stream_timeout_ms").as_int());
        }
    } else if (request->operation == Transfer::Request::OP_CLAIM) {
        if (stream.claim_ack_pending ||
            !isValidManeuverRequestIdentity(request->consumer_identity) ||
            request->consumer_identity != stream.offer_consumer_identity ||
            now >= stream.offer_deadline ||
            stream.offer_execution_id != stream.execution_id ||
            request->source_request_identity != stream.request_identity ||
            request->source_stream_id != stream.stream_id ||
            request->source_ack_sequence == 0 ||
            request->source_ack_sequence != stream.offer_ack_sequence) {
            reject("stale or unowned terminal hold claim");
            return;
        }
        stream.claimed_consumer_identity = request->consumer_identity;
        stream.offer_consumer_identity.clear();
        stream.claim_ack_pending = true;
        stream.claim_source_ack_sequence = stream.last_ack_sequence;
        stream.claim_deadline = now + std::chrono::seconds(2);
        stream.ack_seen = false;
        auto & native_epoch = retained_native_hold_epoch_;
        if (native_epoch.completed && native_epoch.stream_id == stream.stream_id &&
            native_epoch.request_identity == stream.request_identity &&
            native_epoch.execution_id == stream.execution_id) {
            const auto navigation =
                combined_drone_awareness_handler_->GetVehicleNavigationEvidence();
            const uint64_t prior_watermark = std::max(
                native_epoch.minimum_external_transition_us,
                native_epoch.external_nav_transition_us);
            uint64_t watermark = prior_watermark;
            if (navigation.source_epoch == native_epoch.status_source_epoch &&
                navigation.last_external) {
                // A previously observed successor transition remains a
                // negative fence even after its receipt is too old to be
                // positive takeover proof for a new claim.
                watermark = std::max(watermark,
                    navigation.last_external->nav_state_timestamp_us);
            }
            // PX4 activates the claimant before it can claim. A fresh latest
            // external status whose transition is newer than every earlier
            // owner's is the claimant's epoch, armed as at execution begin.
            // Predecessor-era or stale external status never arms a claim.
            const bool claimant_external = navigation.source_epoch ==
                    native_epoch.status_source_epoch &&
                freshNavigationSample(navigation.latest, now) &&
                navigation.last_external &&
                navigation.latest->source_timestamp_us ==
                    navigation.last_external->source_timestamp_us &&
                navigation.last_external->nav_state_timestamp_us > prior_watermark;
            native_epoch.minimum_external_transition_us = watermark;
            native_epoch.external_status_timestamp_us = claimant_external
                ? navigation.last_external->source_timestamp_us : 0;
            native_epoch.external_nav_transition_us = claimant_external
                ? navigation.last_external->nav_state_timestamp_us : 0;
            native_epoch.owner_started = now;
            native_epoch.claimed_consumer_identity = request->consumer_identity;
            native_epoch.claimed_applied = false;
        }
        auto event = iii_drone::diagnostics::HilTrace::event("terminal_hold_claimed");
        event.text("source_request_identity", stream.request_identity);
        event.text("stream_id", stream.stream_id);
        event.text("consumer_identity", stream.claimed_consumer_identity);
        event.number("source_ack_sequence", stream.claim_source_ack_sequence);
        event.commit();
    } else {
        reject("unsupported terminal hold transfer operation");
        return;
    }
    response->accepted = true;
    response->source_request_identity = stream.request_identity;
    response->source_stream_id = stream.stream_id;
    response->source_ack_sequence = stream.last_ack_sequence;
    response->reference = ReferenceAdapter(stream.last_ack_reference).ToMsg();
}

void ManeuverScheduler::pauseReferenceStream(
    const std::shared_ptr<iii_drone_interfaces::srv::PauseReferenceStream::Request> request,
    std::shared_ptr<iii_drone_interfaces::srv::PauseReferenceStream::Response> response
) {
    auto event = iii_drone::diagnostics::HilTrace::event("reference_pause_service_received");
    event.text("stream_id", request->stream_id);
    event.number("last_applied_sequence", request->last_applied_sequence);
    event.commit();
    ManeuverServer::SharedPtr server;
    {
        std::lock_guard<std::mutex> lock(reference_stream_mutex_);
        if (
            !reference_stream_state_.valid || request->stream_id != reference_stream_state_.stream_id ||
            request->last_applied_sequence > reference_stream_state_.sequence
        ) {
            response->accepted = false;
            response->reason = "unknown stream or invalid applied sequence";
            return;
        }
        reference_stream_state_.paused = true;
        server = activeManeuverServer();
    }
    if (server) {
        server->PauseReferenceStream();
    }
    response->accepted = true;
    response->reason = "producer paused";
    auto result = iii_drone::diagnostics::HilTrace::event("reference_pause_service_result");
    result.text("stream_id", request->stream_id);
    result.boolean("accepted", response->accepted);
    result.commit();
}

void ManeuverScheduler::rebaseReferenceStream(
    const std::shared_ptr<iii_drone_interfaces::srv::RebaseReferenceStream::Request> request,
    std::shared_ptr<iii_drone_interfaces::srv::RebaseReferenceStream::Response> response
) {
    auto event = iii_drone::diagnostics::HilTrace::event("reference_rebase_service_received");
    event.text("stream_id", request->stream_id);
    event.number("last_applied_sequence", request->last_applied_sequence);
    event.commit();
    ManeuverServer::SharedPtr server;
    uint64_t execution_id = 0;
    {
        std::lock_guard<std::mutex> lock(reference_stream_mutex_);
        if (
            !reference_stream_state_.valid || !reference_stream_state_.paused ||
            request->stream_id != reference_stream_state_.stream_id ||
            request->last_applied_sequence > reference_stream_state_.sequence
        ) {
            response->accepted = false;
            response->reason = "stream is not the active paused generation";
            return;
        }
        server = activeManeuverServer();
        execution_id = reference_stream_state_.execution_id;
    }
    if (!server) {
        response->accepted = false;
        response->reason = "no active maneuver server";
        return;
    }

    const State stopped_state = StateAdapter(request->stopped_state).state();
    std::string reason;
    const auto disposition = server->PrepareReferenceStreamRecovery(stopped_state, reason);
    if (disposition == ReferenceStreamRecoveryDisposition::REJECT) {
        response->accepted = false;
        response->abort_action = false;
        response->reason = reason;
        return;
    }

    if (disposition == ReferenceStreamRecoveryDisposition::ABORT_ACTION) {
        {
            std::lock_guard<std::mutex> lock(reference_stream_mutex_);
            if (
                !reference_stream_state_.valid ||
                request->stream_id != reference_stream_state_.stream_id ||
                execution_id != reference_stream_state_.execution_id
            ) {
                response->accepted = false;
                response->abort_action = false;
                response->reason = "stream changed during rebase";
                return;
            }
            reference_stream_state_.abort_waiting_for_consumer_ready = true;
        }
        response->accepted = true;
        response->abort_action = true;
        response->reason = reason;
        return;
    }

    {
        std::lock_guard<std::mutex> lock(reference_stream_mutex_);
        if (
            !reference_stream_state_.valid ||
            request->stream_id != reference_stream_state_.stream_id ||
            execution_id != reference_stream_state_.execution_id
        ) {
            response->accepted = false;
            response->reason = "stream changed during rebase";
            return;
        }
        reference_stream_state_.stream_id = nextReferenceStreamId(reference_stream_state_.provider);
        reference_stream_state_.sequence = 0;
        reference_stream_state_.last_ack_sequence = 0;
        reference_stream_state_.ack_seen = false;
        reference_stream_state_.paused = true;
        reference_stream_state_.prepared = true;
        reference_stream_state_.committed_waiting_for_applied = false;
        reference_stream_state_.prepared_reference = Reference(stopped_state);
        reference_stream_state_.latest_reference = reference_stream_state_.prepared_reference;
        reference_stream_state_.generation_started = std::chrono::steady_clock::now();
        response->accepted = true;
        response->abort_action = false;
        response->prepared_stream_id = reference_stream_state_.stream_id;
        response->reason = reason;
    }

    auto result = iii_drone::diagnostics::HilTrace::event("reference_rebase_service_result");
    result.text("stream_id", request->stream_id);
    result.text("prepared_stream_id", response->prepared_stream_id);
    result.boolean("accepted", response->accepted);
    result.boolean("abort_action", response->abort_action);
    result.commit();
}

void ManeuverScheduler::commitReferenceStream(
    const std::shared_ptr<iii_drone_interfaces::srv::CommitReferenceStream::Request> request,
    std::shared_ptr<iii_drone_interfaces::srv::CommitReferenceStream::Response> response
) {
    auto event = iii_drone::diagnostics::HilTrace::event("reference_commit_service_received");
    event.text("stream_id", request->stream_id);
    event.number("prepared_sequence", request->prepared_sequence);
    event.commit();
    {
        std::lock_guard<std::mutex> lock(reference_stream_mutex_);
        if (
            !reference_stream_state_.valid || !reference_stream_state_.prepared ||
            request->stream_id != reference_stream_state_.stream_id
        ) {
            response->accepted = false;
            response->reason = "stream is not a prepared generation";
            return;
        }
        if (
            request->prepared_sequence == 0 ||
            request->prepared_sequence > reference_stream_state_.sequence
        ) {
            response->accepted = false;
            response->reason = "commit does not identify a published prepared reference";
            return;
        }
        if (!activeManeuverServer()) {
            response->accepted = false;
            response->reason = "no active maneuver server";
            return;
        }
        reference_stream_state_.prepared = false;
        reference_stream_state_.paused = true;
        reference_stream_state_.committed_waiting_for_applied = true;
        reference_stream_state_.ack_seen = false;
        reference_stream_state_.generation_started = std::chrono::steady_clock::now();
    }
    response->accepted = true;
    response->reason = "prepared generation committed; awaiting first applied acknowledgement";
    auto result = iii_drone::diagnostics::HilTrace::event("reference_commit_service_result");
    result.text("stream_id", request->stream_id);
    result.boolean("accepted", response->accepted);
    result.commit();
}

void ManeuverScheduler::progressScheduler() {

    static bool waiting_for_maneuver_to_start = false;
    static std::string waiting_for_maneuver_to_start_action_name;

    std::unique_lock<std::shared_mutex> lck(maneuver_mutex_);

    (void)retireCompletedTerminalHoldAfterNativeHold();
    (void)retireReleasedConsumerOwner();

    // Verified on-ground disarm retires the airborne owner before a later
    // takeoff or mode can discover this hold as a transfer offer.
    if (retainedTerminalHold()) {
        const auto awareness = combined_drone_awareness_handler_->adapter();
        if (!awareness.armed() &&
            awareness.drone_location() == iii_drone::adapters::DRONE_LOCATION_ON_GROUND) {
            auto hover = std::static_pointer_cast<HoverManeuverServer>(
                registered_maneuvers_.at(MANEUVER_TYPE_HOVER));
            hover->Update(Reference(combined_drone_awareness_handler_->GetState()));
            RCLCPP_INFO(node_->get_logger(),
                "Terminal hold retired after verified on-ground disarm");
        }
    }

    auto on_no_maneuver = [this](Maneuver previous_maneuver) {

        // A successful terminal correction remains a live Core-owned command
        // source after the action result. It is retired only by an explicit
        // successor execution or safety failure, never by the idle timer.
        if (retainedTerminalHold()) {
            maneuver_server_get_reference_callback_still_registered_ = true;
            return;
        }

        // The completed maneuver is replaced by NONE after its first idle
        // tick. Preserve an object continuation on later idle ticks only if
        // the same successful, completed owner still has its original live
        // stream. Otherwise the NONE case below would immediately replace
        // that callback with passthrough (the idle count remains -1), leaving
        // a subsequent same-target Hover unable to adopt its applied source.
        if (previous_maneuver.maneuver_type() == MANEUVER_TYPE_NONE) {
            const auto binding = reference_callback_struct_->snapshot();
            const auto object_entry = registered_maneuvers_.find(
                MANEUVER_TYPE_HOVER_BY_OBJECT);
            const auto fly_entry = registered_maneuvers_.find(
                MANEUVER_TYPE_FLY_TO_OBJECT);
            const bool object_provider = object_entry != registered_maneuvers_.end() &&
                (binding.reference_provider_name == object_entry->second->action_name() ||
                 (fly_entry != registered_maneuvers_.end() &&
                  binding.reference_provider_name == fly_entry->second->action_name()));
            if (binding.lease && !binding.lease->drained() &&
                object_provider && binding.execution_id ==
                    current_reference_execution_id_.Load() &&
                std::static_pointer_cast<HoverByObjectManeuverServer>(object_entry->second)
                    ->HasTrackedSourceIdentity(binding)) {
                // An entered old-owner callback may be inside the session
                // planner. Do not block the scheduler on that session mutex
                // while the callback is draining after successor rejection.
                maneuver_server_get_reference_callback_still_registered_ = true;
                return;
            }
            if (binding.callback && object_provider &&
                isValidManeuverRequestIdentity(binding.request_identity) &&
                binding.execution_id != 0 &&
                binding.execution_id == current_reference_execution_id_.Load() &&
                std::static_pointer_cast<HoverByObjectManeuverServer>(object_entry->second)
                    ->RetainsTrackedSource(binding)) {
                std::lock_guard<std::mutex> stream_lock(reference_stream_mutex_);
                const auto & stream = reference_stream_state_;
                const auto & epoch = retained_native_hold_epoch_;
                if (epoch.completed && epoch.succeeded &&
                    epoch.request_identity == binding.request_identity &&
                    epoch.execution_id == binding.execution_id &&
                    epoch.stream_id == stream.stream_id &&
                    stream.valid && !stream.stream_id.empty() &&
                    stream.provider == binding.reference_provider_name &&
                    stream.request_identity == binding.request_identity &&
                    stream.execution_id == binding.execution_id) {
                    maneuver_server_get_reference_callback_still_registered_ = true;
                    return;
                }
            }
        }

        if (!previous_maneuver.started() &&
            isValidManeuverRequestIdentity(previous_maneuver.requestIdentity())) {
            const auto binding = reference_callback_struct_->snapshot();
            const auto object_entry = registered_maneuvers_.find(MANEUVER_TYPE_HOVER_BY_OBJECT);
            const auto fly_entry = registered_maneuvers_.find(MANEUVER_TYPE_FLY_TO_OBJECT);
            if (object_entry != registered_maneuvers_.end() && binding.callback &&
                binding.request_identity != previous_maneuver.requestIdentity() &&
                binding.execution_id == current_reference_execution_id_.Load() &&
                (binding.reference_provider_name == object_entry->second->action_name() ||
                 (fly_entry != registered_maneuvers_.end() &&
                  binding.reference_provider_name == fly_entry->second->action_name()))) {
                const auto object = std::static_pointer_cast<HoverByObjectManeuverServer>(
                    object_entry->second);
                if (binding.lease && !binding.lease->drained() &&
                    object->HasTrackedSourceIdentity(binding)) {
                    // The rejected goal never owned this callback. Keep its
                    // exact predecessor binding while the entered call exits.
                    maneuver_server_get_reference_callback_still_registered_ = true;
                    return;
                }
                auto rest = object->TrackedTransitionRest(binding);
                if (!rest) rest = object->TrackedFailureRest(binding);
                if (rest && appliedFiniteRestReference(binding.request_identity, *rest)) {
                    std::lock_guard<std::mutex> stream_lock(reference_stream_mutex_);
                    const auto & stream = reference_stream_state_;
                    if (stream.valid && !stream.paused && !stream.prepared &&
                        stream.provider == binding.reference_provider_name &&
                        stream.request_identity == binding.request_identity &&
                        stream.execution_id == binding.execution_id) {
                        // A rejected unstarted successor never displaced the
                        // exact applied object rest. The existing failure
                        // path rejects its action; preserve only G1's stream.
                        maneuver_server_get_reference_callback_still_registered_ = true;
                        return;
                    }
                }
            }
        }

        if (previous_maneuver.success() &&
            (previous_maneuver.maneuver_type() == MANEUVER_TYPE_FLY_TO_OBJECT ||
             previous_maneuver.maneuver_type() == MANEUVER_TYPE_HOVER_BY_OBJECT)) {
            const auto binding = reference_callback_struct_->snapshot();
            const auto source = registered_maneuvers_.find(previous_maneuver.maneuver_type());
            const auto object = registered_maneuvers_.find(MANEUVER_TYPE_HOVER_BY_OBJECT);
            if (source != registered_maneuvers_.end() &&
                object != registered_maneuvers_.end() && binding.callback &&
                binding.reference_provider_name == source->second->action_name() &&
                binding.request_identity == previous_maneuver.requestIdentity() &&
                binding.execution_id != 0 &&
                binding.execution_id == current_reference_execution_id_.Load() &&
                std::static_pointer_cast<HoverByObjectManeuverServer>(object->second)
                    ->RetainsTrackedSource(binding)) {
                maneuver_server_get_reference_callback_still_registered_ = true;
                return;
            }
        }

        auto set_default_no_maneuver_idle_cnt = [this]() {

            no_maneuver_idle_cnt_ = configuration_->GetParameter("/control/maneuver_controller/no_maneuver_idle_cnt_s").as_double() * 1000 / configuration_->GetParameter("/control/maneuver_controller/maneuver_execution_period_ms").as_int();

            maneuver_server_get_reference_callback_still_registered_ = false;


        };

        // The idle count keeps a completed non-sustained hover callback live
        // for its duration. Record that exact generation so evidenced PX4
        // native control can retire it before intentional consumer silence
        // (e.g. a native Land handoff) reaches the ACK guard.
        // A goal can succeed before its first sample is published (e.g. a
        // 68 ms HoverOnCable), so the stream of this execution may only be
        // created after this tick; retirement matches it by execution.
        auto mark_idle_callback_owner = [this]() {
            const auto binding = reference_callback_struct_->snapshot();
            std::lock_guard<std::mutex> stream_lock(reference_stream_mutex_);
            auto & epoch = retained_native_hold_epoch_;
            if (!binding.callback ||
                !isValidManeuverRequestIdentity(binding.request_identity) ||
                binding.execution_id == 0 ||
                binding.execution_id != current_reference_execution_id_.Load() ||
                epoch.request_identity != binding.request_identity ||
                epoch.execution_id != binding.execution_id) return;
            epoch.completed = true;
            epoch.succeeded = true;
            epoch.idle_callback = true;
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

                    no_maneuver_idle_cnt_ = maneuver_params.duration_s * 1000 / configuration_->GetParameter("/control/maneuver_controller/maneuver_execution_period_ms").as_int();

                    maneuver_server_get_reference_callback_still_registered_ = true;

                    mark_idle_callback_owner();

                    break;
                }
                case MANEUVER_TYPE_HOVER_BY_OBJECT: {
                    hover_by_object_maneuver_params_t maneuver_params(previous_maneuver.maneuver_params());

                    if (maneuver_params.sustain_action) {

                        set_default_no_maneuver_idle_cnt();

                        break;

                    }

                    no_maneuver_idle_cnt_ = maneuver_params.duration_s * 1000 / configuration_->GetParameter("/control/maneuver_controller/maneuver_execution_period_ms").as_int();

                    maneuver_server_get_reference_callback_still_registered_ = true;

                    mark_idle_callback_owner();

                    break;
                }
                case MANEUVER_TYPE_HOVER_ON_CABLE: {
                    hover_on_cable_maneuver_params_t maneuver_params(previous_maneuver.maneuver_params());

                    // A cable release push is sustained only until it is
                    // established; it must then keep holding the cable for
                    // duration_s, until its successor (CableTakeoff) takes over.
                    if (maneuver_params.sustain_action && !(maneuver_params.push_upwards_acceleration > 0.0)) {

                        set_default_no_maneuver_idle_cnt();

                        break;

                    }

                    no_maneuver_idle_cnt_ = maneuver_params.duration_s * 1000 / configuration_->GetParameter("/control/maneuver_controller/maneuver_execution_period_ms").as_int();

                    maneuver_server_get_reference_callback_still_registered_ = true;

                    mark_idle_callback_owner();

                    break;
                }
                case MANEUVER_TYPE_NONE:
                    if (*no_maneuver_idle_cnt_ >= 0)
                        (*no_maneuver_idle_cnt_)--;

                    break;

                case MANEUVER_TYPE_FLY_TO_POSITION: {
                    fly_to_position_maneuver_params_t maneuver_params(previous_maneuver.maneuver_params());

                    if (maneuver_params.blend_to_next) {
                        constexpr double blend_handoff_grace_s = 0.5;
                        const int period_ms = configuration_->GetParameter("/control/maneuver_controller/maneuver_execution_period_ms").as_int();
                        no_maneuver_idle_cnt_ = std::max(1, static_cast<int>(blend_handoff_grace_s * 1000.0 / period_ms));
                        maneuver_server_get_reference_callback_still_registered_ = true;

                        RCLCPP_DEBUG(
                            node_->get_logger(),
                            "ManeuverScheduler::progressScheduler(): preserving successful blended FTP reference callback for %d scheduler tick(s).",
                            *no_maneuver_idle_cnt_
                        );

                        break;
                    }

                    set_default_no_maneuver_idle_cnt();

                    break;
                }

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

    // Every failure exit names its cause: a pending successor rejected here is
    // otherwise only visible as "could not acquire reference callback token".
    // A maneuver that ended unsuccessfully (cancelled, or failed and reported
    // by its own server) is logged at INFO; rejections decided here warn.
    auto on_failure = [this, &on_no_maneuver](
        Maneuver previous_maneuver, const std::string & reason, bool decided_here = true) {

        if (decided_here) {
            RCLCPP_WARN(
                node_->get_logger(),
                "ManeuverScheduler::progressScheduler(): failing maneuver %d (request %s): %s",
                previous_maneuver.maneuver_type(),
                previous_maneuver.requestIdentity().c_str(),
                reason.c_str()
            );
        } else {
            RCLCPP_INFO(
                node_->get_logger(),
                "ManeuverScheduler::progressScheduler(): maneuver %d (request %s) ended: %s",
                previous_maneuver.maneuver_type(),
                previous_maneuver.requestIdentity().c_str(),
                reason.c_str()
            );
        }

        previous_maneuver.Terminate(false);

        cancelAllPendingManeuvers();

        reference_callback_token_.DenyAll();

        on_no_maneuver(previous_maneuver);

    };

    const auto object_handoff_failure_reason = [](
        bool applied_rest, bool unrecoverable, bool expired, bool resumed) {
        std::string reason = "object hand-off: tracked source failed (";
        reason += applied_rest ? "rest applied" : "rest not applied";
        if (unrecoverable) reason += ", unrecoverable";
        if (expired) reason += ", consumer acknowledgement expired";
        if (!resumed) reason += ", lease not resumed";
        return reason + ")";
    };

    // Check if a maneuver is executing
    if (maneuverIsExecutingOrPending()) {

        // Check if the current maneuver is completed
        if(current_maneuver_->terminated()) {

            if (!reference_callback_token_.master_has_token()) {

                rclcpp::Time now = rclcpp::Clock().now();

                float elapsed_seconds = (now - current_maneuver_->termination_time()).seconds();

                float timeout_seconds = configuration_->GetParameter("/control/maneuver_controller/maneuver_completion_token_acquisition_timeout_s").as_double();

                if (elapsed_seconds > timeout_seconds) {

                    RCLCPP_ERROR_THROTTLE(
                        node_->get_logger(),
                        *node_->get_clock(),
                        1000,
                        "ManeuverScheduler::progressScheduler(): timed out waiting for master token after maneuver %d terminated. Current token holder: %s. Waiting instead of crashing.",
                        current_maneuver_->maneuver_type(),
                        reference_callback_token_.token_holder().c_str()
                    );

                }

                return;

            }

            {
                // No token return follows for a successor whose server never
                // took over; install its retained hold here.
                std::lock_guard<std::mutex> stream_lock(reference_stream_mutex_);
                installUnexecutedSuccessorHoldLocked();
            }

            // Token::Release makes the master holder visible before invoking
            // onReferenceCallbackTokenReacquired(). A scheduler tick in that
            // interval must not retire this exact retained owner or advertise
            // the source callback as a completed hold. The token callback
            // publishes the rebound callback before setting the validity flag.
            if (!maneuver_server_get_reference_callback_still_registered_.Load()) {
                if (current_maneuver_->success() &&
                    (current_maneuver_->maneuver_type() == MANEUVER_TYPE_FLY_TO_OBJECT ||
                     current_maneuver_->maneuver_type() == MANEUVER_TYPE_HOVER_BY_OBJECT)) {
                    const auto object_entry = registered_maneuvers_.find(
                        MANEUVER_TYPE_HOVER_BY_OBJECT);
                    const auto source_entry = registered_maneuvers_.find(
                        current_maneuver_->maneuver_type());
                    const auto binding = reference_callback_struct_->snapshot();
                    if (object_entry != registered_maneuvers_.end() &&
                        source_entry != registered_maneuvers_.end() &&
                        binding.reference_provider_name == source_entry->second->action_name() &&
                        binding.request_identity == current_maneuver_->requestIdentity() &&
                        binding.execution_id == current_reference_execution_id_.Load() &&
                        std::static_pointer_cast<HoverByObjectManeuverServer>(object_entry->second)
                            ->RetainsTrackedSource(binding)) return;
                }
                const auto hover_entry = registered_maneuvers_.find(MANEUVER_TYPE_HOVER);
                const auto source_entry = registered_maneuvers_.find(current_maneuver_->maneuver_type());
                if (hover_entry != registered_maneuvers_.end() &&
                    source_entry != registered_maneuvers_.end()) {
                    const auto owner = std::static_pointer_cast<HoverManeuverServer>(
                        hover_entry->second)->terminalHoldBinding();
                    const auto binding = reference_callback_struct_->snapshot();
                    if (owner.hold && binding.callback &&
                        isValidManeuverRequestIdentity(owner.request_identity) &&
                        owner.request_identity == current_maneuver_->requestIdentity() &&
                        binding.request_identity == owner.request_identity &&
                        binding.execution_id != 0 &&
                        binding.execution_id == current_reference_execution_id_.Load() &&
                        (binding.reference_provider_name == source_entry->second->action_name() ||
                         (!current_maneuver_->success() &&
                          binding.reference_provider_name == hover_entry->second->action_name()))) {
                        return;
                    }
                }
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

                on_failure(current_maneuver_, "terminated unsuccessfully (cancelled or failed; see its server's log)", false);

            }

        } else {
            // Current maneuver has not terminated.

            // Check that the maneuver has a goal handle:
            if (current_maneuver_->goal_handle() == nullptr) {

                // Check that the update timeout has elapsed:
                rclcpp::Time maneuver_creation_time = current_maneuver_->creation_time();
                rclcpp::Time current_time = rclcpp::Clock().now();

                if ((current_time - maneuver_creation_time).seconds() > configuration_->GetParameter("/control/maneuver_controller/maneuver_register_update_timeout_s").as_double()) {

                    on_failure(current_maneuver_, "maneuver goal was not registered within maneuver_register_update_timeout_s");

                }

                return;

            }

            if (!current_maneuver_->started()) {

                std::optional<Reference> terminal_seed;
                std::optional<Reference> object_seed;
                std::optional<ReferenceCallbackBinding> object_exit_binding;
                if (const auto terminal_hold = retainedTerminalHold()) {
                    const auto terminal_binding = std::static_pointer_cast<HoverManeuverServer>(
                        registered_maneuvers_.at(MANEUVER_TYPE_HOVER))->terminalHoldBinding();
                    if (terminal_binding.hold != terminal_hold ||
                        terminal_binding.request_identity.empty()) {
                        on_failure(current_maneuver_, "pending successor does not match the retained terminal hold binding");
                        return;
                    }
                    const bool same_target_hover =
                        current_maneuver_->maneuver_type() == MANEUVER_TYPE_HOVER;
                    const auto request_identity = current_maneuver_->requestIdentity();
                    const auto now = std::chrono::steady_clock::now();
                    if (terminal_quiesce_request_identity_ != request_identity) {
                        terminal_quiesce_request_identity_ = request_identity;
                        terminal_quiesce_started_ = now;
                        RCLCPP_INFO(node_->get_logger(),
                            "Terminal hold: preparing %s request %s",
                            same_target_hover ? "same-target hover" : "quiescent successor",
                            request_identity.c_str());
                    }
                    const auto limits = TerminalPositionTrackingController::Limits{};
                    const double maximum_segment_distance = 2.0 * limits.max_offset_m;
                    const double maximum_segment_s = std::max({
                        (35.0 / 16.0) * maximum_segment_distance / limits.max_offset_speed_m_s,
                        std::sqrt((84.0 / (5.0 * 2.2360679774997896964)) *
                            maximum_segment_distance / limits.max_offset_acceleration_m_s2),
                        std::cbrt(52.5 * maximum_segment_distance /
                            limits.max_offset_jerk_m_s3)
                    });
                    if (terminal_hold->phase() == TerminalTrackingHold::Phase::Tracking &&
                        std::chrono::duration<double>(now - terminal_quiesce_started_).count() >
                            maximum_segment_s + 2.5) {
                        terminal_hold->Fail("terminal handoff quiescence deadline exceeded");
                    }
                    if (terminal_hold->phase() == TerminalTrackingHold::Phase::Degraded ||
                        terminal_hold->phase() == TerminalTrackingHold::Phase::Unrecoverable) {
                        RCLCPP_ERROR(node_->get_logger(),
                            "Terminal hold: successor handoff failed: %s",
                            terminal_hold->failureReason().c_str());
                        on_failure(current_maneuver_, "terminal hold failed during the successor hand-off: " + terminal_hold->failureReason());
                        return;
                    }
                    if (terminal_hold->phase() != TerminalTrackingHold::Phase::Tracking) return;
                    if (!same_target_hover && !terminal_hold->isQuiescent()) {
                        if (!terminal_hold->RequestQuiescence()) {
                            terminal_hold->Fail("terminal handoff could not request quiescence");
                        }
                        return;
                    }
                    Reference command = terminal_hold->lastCommand();
                    bool command_applied = false;
                    const auto source_execution_id = reference_callback_struct_->snapshot().execution_id;
                    {
                        std::lock_guard<std::mutex> stream_lock(reference_stream_mutex_);
                        const auto & stream = reference_stream_state_;
                        command_applied = stream.valid && stream.ack_seen &&
                            stream.request_identity == terminal_binding.request_identity &&
                            stream.execution_id == source_execution_id &&
                            stream.last_consumer_status ==
                                iii_drone_interfaces::msg::ManeuverReferenceAck::STATUS_APPLIED &&
                            stream.last_ack_reference_valid &&
                            now - stream.last_ack < std::chrono::milliseconds(
                                configuration_->GetParameter(
                                    "/control/maneuver_controller/reference_stream_timeout_ms").as_int()) &&
                            (same_target_hover ||
                             ((stream.last_ack_reference.position() - command.position()).norm() < 1.0e-3 &&
                              (stream.last_ack_reference.velocity() - command.velocity()).norm() < 1.0e-3 &&
                              (stream.last_ack_reference.acceleration() - command.acceleration()).norm() < 1.0e-3));
                        if (command_applied && same_target_hover) {
                            command = stream.last_ack_reference;
                        }
                    }
                    if (!command_applied) return;
                    terminal_seed = command;
                }

                if (!terminal_seed && current_maneuver_->maneuver_type() ==
                        MANEUVER_TYPE_HOVER_BY_OBJECT) {
                    const auto binding = reference_callback_struct_->snapshot();
                    const auto object = std::static_pointer_cast<HoverByObjectManeuverServer>(
                        registered_maneuvers_.at(MANEUVER_TYPE_HOVER_BY_OBJECT));
                    if (object->HasMatchingTrackedSession(current_maneuver_)) {
                        // FTO success installs a retained HBO callable under
                        // the predecessor generation. Exclude new entries and
                        // wait across scheduler ticks for any copied call to
                        // finish before the shared session changes owner. An
                        // entered call may hold the session mutex throughout
                        // its planner RPC, so only inspect identity fields
                        // before the lease drains.
                        const auto fly_entry =
                            registered_maneuvers_.find(MANEUVER_TYPE_FLY_TO_OBJECT);
                        const bool source_provider =
                            binding.reference_provider_name == object->action_name() ||
                            (fly_entry != registered_maneuvers_.end() &&
                             binding.reference_provider_name ==
                                 fly_entry->second->action_name());
                        if (!binding.lease || !source_provider ||
                            binding.execution_id !=
                                current_reference_execution_id_.Load() ||
                            !object->HasTrackedSourceIdentity(binding)) {
                            on_failure(current_maneuver_, "object hand-off: callback binding, lease, provider, execution or tracked-source identity mismatch");
                            return;
                        }
                        // HIL soak run 19: a hard-coded 250 ms deadline against
                        // acknowledgements arriving every 200 ms (/control/dt)
                        // failed the hand-off; use the stream's own deadline.
                        const auto ack_deadline = std::chrono::milliseconds(
                            configuration_->GetParameter(
                                "/control/maneuver_controller/reference_stream_timeout_ms"
                            ).as_int());
                        const auto acknowledgement_expired = [this, ack_deadline]() {
                            const auto now = std::chrono::steady_clock::now();
                            std::lock_guard<std::mutex> stream_lock(
                                reference_stream_mutex_);
                            const auto & stream = reference_stream_state_;
                            const auto age = stream.ack_seen
                                ? now - stream.last_ack
                                : now - stream.generation_started;
                            return detail::ObjectHandoffAcknowledgementExpired(
                                age, stream.ack_seen && now < stream.last_ack, ack_deadline);
                        };
                        binding.lease->requestQuiescence();
                        if (!binding.lease->drained()) {
                            // The pending HBO has not started, so the normal
                            // active-maneuver watchdog cannot reject it. Do
                            // not wait on the provider/session mutex to apply
                            // the existing ACK freshness deadline.
                            if (acknowledgement_expired()) {
                                (void)binding.lease->resume();
                                on_failure(current_maneuver_, "object hand-off: consumer acknowledgement expired while the predecessor callback drained");
                            }
                            return;
                        }
                        if (!object->CanAdoptTrackedSession(
                                current_maneuver_, binding)) {
                            on_failure(current_maneuver_, "object hand-off: tracked object session cannot be adopted by this request");
                            return;
                        }
                        // An entered fallback may latch a real tracking fault
                        // while draining. Keep the same owner publishing its
                        // certified failure stop; it cannot be adopted.
                        if (object->TrackedSourceFailed(binding)) {
                            const auto rest = object->TrackedFailureRest(binding);
                            const bool applied_rest = rest &&
                                appliedFiniteRestReference(
                                    binding.request_identity, *rest);
                            const bool unrecoverable =
                                object->TrackedSourceUnrecoverable(binding);
                            const bool expired = acknowledgement_expired();
                            // Even after an APPLIED rest rejects the successor,
                            // the same old owner must keep publishing that
                            // finite rest until control is explicitly retired.
                            const bool resumed = binding.lease->resume();
                            if (applied_rest || unrecoverable || expired || !resumed) {
                                on_failure(current_maneuver_, object_handoff_failure_reason(applied_rest, unrecoverable, expired, resumed));
                            }
                            return;
                        }
                        const auto now = std::chrono::steady_clock::now();
                        bool expired = false;
                        {
                            std::lock_guard<std::mutex> stream_lock(reference_stream_mutex_);
                            const auto & stream = reference_stream_state_;
                            const bool known_ack = std::any_of(
                                stream.recent_references.begin(), stream.recent_references.end(),
                                [&stream](const auto & entry) {
                                    return entry.first == stream.last_ack_sequence;
                                });
                            const auto current_binding = reference_callback_struct_->snapshot();
                            const bool exact = current_binding.revision == binding.revision &&
                                stream.valid && !stream.paused && !stream.prepared &&
                                !stream.committed_waiting_for_applied &&
                                !stream.abort_waiting_for_consumer_ready &&
                                !stream.claim_ack_pending &&
                                stream.request_identity == binding.request_identity &&
                                stream.execution_id == binding.execution_id &&
                                stream.execution_id == current_reference_execution_id_.Load() &&
                                stream.provider == binding.reference_provider_name;
                            const auto age = stream.ack_seen
                                ? now - stream.last_ack : now - stream.generation_started;
                            expired = detail::ObjectHandoffAcknowledgementExpired(
                                age, stream.ack_seen && now < stream.last_ack, ack_deadline);
                            if (exact && stream.ack_seen && stream.last_ack_reference_valid &&
                                known_ack && stream.last_consumer_status ==
                                    iii_drone_interfaces::msg::ManeuverReferenceAck::STATUS_APPLIED &&
                                !expired) {
                                const Reference applied = stream.last_ack_reference;
                                if (applied.position().allFinite() &&
                                    applied.velocity().allFinite() &&
                                    applied.acceleration().allFinite() &&
                                    std::isfinite(applied.yaw()) &&
                                    std::isfinite(applied.yaw_rate()) &&
                                    std::isfinite(applied.yaw_acceleration())) {
                                    object_seed = applied;
                                }
                            }
                        }
                        if (!object_seed) {
                            if (expired) {
                                binding.lease->resume();
                                on_failure(current_maneuver_, "object hand-off: no applied finite rest seed before the acknowledgement deadline");
                            }
                            return;
                        }
                    }
                }

                const auto successor_type = current_maneuver_->maneuver_type();
                const bool object_stop_successor =
                    successor_type == MANEUVER_TYPE_CABLE_LANDING ||
                    successor_type == MANEUVER_TYPE_FLY_TO_POSITION ||
                    successor_type == MANEUVER_TYPE_FOLLOW_WAYPOINT_PATH ||
                    successor_type == MANEUVER_TYPE_FLY_TO_OBJECT ||
                    successor_type == MANEUVER_TYPE_HOVER ||
                    successor_type == MANEUVER_TYPE_HOVER_BY_OBJECT;
                if (!terminal_seed && !object_seed && object_stop_successor) {
                    const auto binding = reference_callback_struct_->snapshot();
                    const auto object = std::static_pointer_cast<HoverByObjectManeuverServer>(
                        registered_maneuvers_.at(MANEUVER_TYPE_HOVER_BY_OBJECT));
                    if (object->HasTrackedSession() &&
                        !object->RetainsTrackedSource(binding)) return;
                    if (object->RetainsTrackedSource(binding)) {
                        if (successor_type == MANEUVER_TYPE_HOVER_BY_OBJECT) {
                            // A different target has no bounded direct-Hover
                            // path from this certified rest. Keep the old
                            // owner instead of seeding measured/new-target pose.
                            RCLCPP_ERROR(node_->get_logger(),
                                "Different-target object hover cannot consume an owned object stop; approach the new target with FlyToObject first");
                            on_failure(current_maneuver_, "different-target object hover cannot consume an owned object stop");
                            return;
                        }
                        if (!object->RequestTrackedTransitionStop(binding)) {
                            on_failure(current_maneuver_, "object hand-off: tracked transition stop request was rejected");
                            return;
                        }
                        const auto rest = object->TrackedTransitionRest(binding);
                        if (!rest || !appliedFiniteRestReference(
                                binding.request_identity, *rest)) return;
                        object_seed = *rest;
                        object_exit_binding = binding;
                    }
                }

                // A generation belongs to an action execution, not just a
                // provider name. Retire the predecessor before this goal can
                // acquire the callback token, including when both goals use
                // the same maneuver server.
                auto registered_maneuver = registered_maneuvers_.find(current_maneuver_->maneuver_type());
                const auto finite_seed = terminal_seed ? terminal_seed : object_seed;
                const bool terminal_successor =
                    successor_type == MANEUVER_TYPE_FLY_TO_POSITION ||
                    successor_type == MANEUVER_TYPE_FOLLOW_WAYPOINT_PATH ||
                    successor_type == MANEUVER_TYPE_FLY_TO_OBJECT ||
                    successor_type == MANEUVER_TYPE_HOVER;
                const bool object_successor =
                    successor_type == MANEUVER_TYPE_HOVER_BY_OBJECT ||
                    object_stop_successor;
                if (finite_seed &&
                    ((terminal_seed && !terminal_successor) ||
                     (object_seed && !object_successor) ||
                     !registered_maneuver->second->StageTerminalStartReference(
                         current_maneuver_->requestIdentity(), *finite_seed))) {
                    if (const auto hold = retainedTerminalHold()) {
                        hold->Fail("terminal successor cannot accept a finite start command");
                    } else {
                        on_failure(current_maneuver_, "terminal successor cannot accept a finite start command");
                    }
                    return;
                }
                if (terminal_seed) {
                    beginSeededSuccessorExecution(
                        registered_maneuver->second->action_name(),
                        current_maneuver_->requestIdentity(),
                        *terminal_seed
                    );
                } else {
                    beginReferenceExecution(
                        registered_maneuver->second->action_name(),
                        current_maneuver_->requestIdentity(),
                        finite_seed
                    );
                }
                if (object_exit_binding) {
                    std::static_pointer_cast<HoverByObjectManeuverServer>(
                        registered_maneuvers_.at(MANEUVER_TYPE_HOVER_BY_OBJECT))
                            ->RetireTrackedSource(*object_exit_binding);
                }
                terminal_quiesce_request_identity_.clear();

                // Start the maneuver execution:
                current_maneuver_->Start();

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

                    if ((current_time - maneuver_start_time).seconds() > configuration_->GetParameter("/control/maneuver_controller/maneuver_start_timeout_s").as_double()) {

                        on_failure(current_maneuver_, "maneuver did not start within maneuver_start_timeout_s");

                    }
                }
            }
        }

    } else {

        if (current_maneuver_->maneuver_type() != MANEUVER_TYPE_NONE) {

            RCLCPP_ERROR(
                node_->get_logger(),
                "ManeuverScheduler::progressScheduler(): maneuver is not executing or pending, but current maneuver is %d. Resetting scheduler current maneuver.",
                current_maneuver_->maneuver_type()
            );

            current_maneuver_ = Maneuver();

        }

        on_no_maneuver(current_maneuver_);

    }
}

std::optional<iii_drone_interfaces::msg::Reference>
ManeuverScheduler::fetchNextReferenceAndPublish(
    const ReferenceCallbackBinding & binding
) {

    if (!binding.callback) return std::nullopt;
    Reference ref;
    try {
        ref = binding.callback(combined_drone_awareness_handler_->GetState());
    } catch (const RetiredReferenceCallback &) {
        return std::nullopt;
    }
    std::lock_guard<std::mutex> stream_lock(reference_stream_mutex_);
    std::lock_guard<std::mutex> binding_lock(
        reference_callback_struct_->publication_mutex_);
    const auto current = reference_callback_struct_->snapshot();
    if (current.revision != binding.revision ||
        current.execution_id != binding.execution_id ||
        current.request_identity != binding.request_identity ||
        (binding.lease && binding.lease->retired()) ||
        !currentReferenceValid(binding)) {
        return std::nullopt;
    }
    auto result = ReferenceAdapter(ref).ToMsg();
    reference_publisher_->publish(result);
    return result;

}

void ManeuverScheduler::cancelAllPendingManeuvers() {

    maneuver_queue_->Clear();

    current_maneuver_ = Maneuver();

}
