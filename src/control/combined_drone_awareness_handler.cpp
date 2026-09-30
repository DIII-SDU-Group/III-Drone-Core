/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/combined_drone_awareness_handler.hpp>

#include <cstdio>
#include <algorithm>
#include <cmath>
#include <utility>

namespace {
int64_t steadyNowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

template<class Function> class ScopeExit {
public:
    explicit ScopeExit(Function function) : function_(std::move(function)) {}
    ~ScopeExit() { function_(); }
    ScopeExit(const ScopeExit &) = delete;
    ScopeExit & operator=(const ScopeExit &) = delete;
private:
    Function function_;
};

template<class Function> ScopeExit<Function> onScopeExit(Function function) {
    return ScopeExit<Function>(std::move(function));
}
}  // namespace

using namespace iii_drone::control;
using namespace iii_drone::utils;
using namespace iii_drone::types;
using namespace iii_drone::math;
using namespace iii_drone::adapters;

using VehicleStatusAdapterHistory = iii_drone::utils::History<iii_drone::adapters::px4::VehicleStatusAdapter>;
using VehicleOdometryAdapterHistory = iii_drone::utils::History<iii_drone::adapters::px4::VehicleOdometryAdapter>;
using PowerlineAdapterHistory = iii_drone::utils::History<iii_drone::adapters::PowerlineAdapter>;
using GripperStatusAdapterHistory = iii_drone::utils::History<iii_drone::adapters::GripperStatusAdapter>;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

CombinedDroneAwarenessHandler::CombinedDroneAwarenessHandler(
    iii_drone::configuration::Configuration::SharedPtr params,
    tf2_ros::Buffer::SharedPtr tf_buffer,
    rclcpp_lifecycle::LifecycleNode * node,
    bool debug
) : configuration_(params),
    tf_buffer_(tf_buffer),
    node_(node) {

    debug_ = debug;

    // PX4 external modes are setpoint-driven modes just like the standard
    // OFFBOARD state.  Keep them offboard-equivalent from startup so maneuvers
    // can execute immediately after an external mode is selected.  The
    // registration service remains available for additional application modes.
    offboard_nav_state_ids_.Store({
        px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD,
        px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_EXTERNAL1,
        px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_EXTERNAL2,
        px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_EXTERNAL3,
        px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_EXTERNAL4,
        px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_EXTERNAL5,
        px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_EXTERNAL6,
        px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_EXTERNAL7,
        px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_EXTERNAL8
    });

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::CombinedDroneAwarenessHandler(): Creating publishers");

    combined_drone_awareness_pub_ = node_->create_publisher<iii_drone_interfaces::msg::CombinedDroneAwareness>(
        "combined_drone_awareness",
        10
    );

	rclcpp::QoS px4_sub_qos(rclcpp::KeepLast(1));
	px4_sub_qos.transient_local();
	px4_sub_qos.best_effort();

    target_pub_ = node_->create_publisher<iii_drone_interfaces::msg::Target>(
        "target",
        10
    );

    target_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        "target_pose",
        10
    );

    target_drone_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        "target_drone_pose",
        10
    );


}

CombinedDroneAwarenessHandler::~CombinedDroneAwarenessHandler() {

    stopOdometryIngress();
    callback_lifetime_.Close();

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::~CombinedDroneAwarenessHandler(): Destroying CombinedDroneAwarenessHandler");

    if (is_started_) {
        Stop();
    }

}

void CombinedDroneAwarenessHandler::Start() {

    if (is_started_) {

        RCLCPP_ERROR(node_->get_logger(), "CombinedDroneAwarenessHandler::Start(): CombinedDroneAwarenessHandler is already started");

        return;

    }

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::Start(): Starting CombinedDroneAwarenessHandler");

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::Start(): Creating tf_broadcaster_");

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::Start(): Creating history and atomic member objects");
    vehicle_status_adapter_history_ = std::make_shared<VehicleStatusAdapterHistory>(1);
    vehicle_odometry_adapter_history_ = std::make_shared<VehicleOdometryAdapterHistory>(2);
    measured_odometry_.Store(std::nullopt);
    vehicle_global_position_adapter_history_ = std::make_shared<VehicleGlobalPositionAdapterHistory>(1);
    powerline_adapter_history_ = std::make_shared<PowerlineAdapterHistory>(1);
    gripper_status_adapter_history_ = std::make_shared<GripperStatusAdapterHistory>(1);
    {
        std::lock_guard<std::mutex> lock(odometry_ingest_mutex_);
        latest_local_reset_.reset();
        verified_local_reset_.reset();
        pending_odometry_.reset();
        local_provenance_invalid_ = false;
        ++odometry_source_epoch_;
        position_epoch_ = 0;
        odometry_ingress_next_ = 0;
        odometry_ingress_count_ = 0;
        odometry_ingress_total_callbacks_ = 0;
        accepted_odometry_samples_ = 0;
        latest_accepted_odometry_available_ = false;
    }
    ground_altitude_estimate_ = std::make_shared<iii_drone::utils::Atomic<double>>(0.0);
    ground_altitude_estimate_amsl_ = std::make_shared<iii_drone::utils::Atomic<double>>(0.0);
    target_adapter_ = std::make_shared<iii_drone::utils::Atomic<TargetAdapter>>();
    
    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::Start(): Creating Register Offboard Mode service");

    register_offboard_mode_srv_ = node_->create_service<iii_drone_interfaces::srv::RegisterOffboardMode>(
        "register_offboard_mode",
        [this](
            const std::shared_ptr<iii_drone_interfaces::srv::RegisterOffboardMode::Request> request, 
            std::shared_ptr<iii_drone_interfaces::srv::RegisterOffboardMode::Response>
        ) {
            if (request->deregister) {
                deregisterOffboardMode(request->mode_id);
            } else {
                registerOffboardMode(request->mode_id);
            }
        }
    );

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::Start(): Creating ground altitude estimate");
    int ground_estimate_window_size = configuration_->GetParameter("/control/maneuver_controller/ground_estimate_window_size").as_int();
    ground_altitudes_history_ = std::make_shared<History<double>>(0, ground_estimate_window_size);

    ground_altitude_update_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(configuration_->GetParameter("/control/maneuver_controller/ground_estimate_update_period_ms").as_int()),
        [this]() {
            if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::ground_altitude_update_timer_: Updating ground altitude estimate");
            ground_altitude_update_timer_->cancel();
        }
    );

    ground_altitude_update_timer_->cancel();

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::Start(): Creating combined_drone_awareness_");
    combined_drone_awareness_adapter_ = std::make_shared<Atomic<CombinedDroneAwarenessAdapter>>();

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::Start(): Updating combined drone awareness");
    updateCombinedDroneAwareness();

    if (debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::Start(): Creating combined drone awareness publish timer");
    combined_drone_awareness_pub_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(configuration_->GetParameter("/control/maneuver_controller/combined_drone_awareness_pub_period_ms").as_int()),
        [this]() {
            if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::combined_drone_awareness_pub_timer_: Publishing combined drone awareness");

            // CombinedDroneAwarenessAdapter adapter = combined_drone_awareness();
            
            // iii_drone_interfaces::msg::CombinedDroneAwareness msg;

            // msg.state = StateAdapter(cda.state).ToMsg();
            // msg.armed = cda.armed;
            // msg.offboard = cda.offboard;
            // msg.has_target = cda.has_target();
            // msg.target = cda.target_adapter.ToMsg();
            // msg.target_position_known = cda.target_position_known;
            // msg.drone_location = (uint8_t)cda.drone_location;
            // msg.on_cable_id = cda.on_cable_id;
            // msg.ground_altitude_estimate = cda.ground_altitude_estimate;
            // msg.ground_altitude_estimate_amsl = cda.ground_altitude_estimate_amsl;
            // msg.gripper_open = cda.gripper_open;

            combined_drone_awareness_pub_->publish((*combined_drone_awareness_adapter_)->ToMsg());

        }
    );

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::Start(): Creating subscribers");
	rclcpp::QoS px4_sub_qos(rclcpp::KeepLast(1));
	px4_sub_qos.transient_local();
	px4_sub_qos.best_effort();

    vehicle_status_sub_ = node_->create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status_v1",
        px4_sub_qos,
        [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
            if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::vehicle_status_sub_: Vehicle status received");
            const auto external_modes = offboard_nav_state_ids_.Load();
            const bool external_mode = std::find(
                external_modes.begin(), external_modes.end(), msg->nav_state) !=
                external_modes.end();
            vehicle_navigation_evidence_.Store(AdvanceVehicleNavigation(
                vehicle_navigation_evidence_.Load(), *msg,
                std::chrono::steady_clock::now(), external_mode));
            iii_drone::adapters::px4::VehicleStatusAdapter adapter(*msg);
            vehicle_status_adapter_history_->Store(adapter);
            updateCombinedDroneAwarenessFromVehicleStatus();
        }
    );

    vehicle_land_detected_sub_ = node_->create_subscription<px4_msgs::msg::VehicleLandDetected>(
        "/fmu/out/vehicle_land_detected",
        px4_sub_qos,
        [this, lifetime = callback_lifetime_.token()](const px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
            const auto alive = lifetime.Enter();
            if (!alive.owns_lock()) return;
            px4_land_state_.Store(Px4LandState{
                msg->landed, msg->maybe_landed, msg->ground_contact,
                std::chrono::steady_clock::now()});
        }
    );

    vehicle_local_position_setpoint_sub_ = node_->create_subscription<px4_msgs::msg::VehicleLocalPositionSetpoint>(
        "/fmu/out/vehicle_local_position_setpoint",
        px4_sub_qos,
        [this, lifetime = callback_lifetime_.token()](const px4_msgs::msg::VehicleLocalPositionSetpoint::SharedPtr msg) {
            const auto alive = lifetime.Enter();
            if (!alive.owns_lock()) return;
            const auto now = std::chrono::steady_clock::now();
            const double thrust_up = -static_cast<double>(msg->thrust[2]);
            px4_thrust_setpoint_.Store(Px4ThrustSetpoint{
                thrust_up, -static_cast<double>(msg->acceleration[2]), now});
            // Free flight only: on the cable the vehicle is held and the
            // thrust says nothing about its weight.
            double speed = NAN;
            double vertical_speed = NAN;
            if (state_available()) {
                const auto velocity = GetState().velocity();
                speed = velocity.norm();
                vertical_speed = velocity(2);
            }
            const bool free_flight = px4_airborne(now) && !on_cable();
            std::lock_guard<std::mutex> lock(hover_thrust_meter_mutex_);
            hover_thrust_meter_.Add(now, thrust_up, speed, vertical_speed, free_flight);
        }
    );

    if (!odometry_callback_group_) {
        // Not added to the node's executor: startOdometryIngress() spins it.
        odometry_callback_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive, false);
    }
    rclcpp::SubscriptionOptions odometry_options;
    odometry_options.callback_group = odometry_callback_group_;
    rclcpp::QoS odometry_qos{rclcpp::KeepLast{odometry_queue_depth_}};
    odometry_qos.transient_local();
    odometry_qos.best_effort();

    vehicle_odometry_sub_ = node_->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry",
        odometry_qos,
        [this, lifetime = callback_lifetime_.token()](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
            const auto alive = lifetime.Enter();
            if (!alive.owns_lock()) return;
            if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::vehicle_odometry_sub_: Vehicle odometry received");
            ingestVehicleOdometry(*msg, node_->now());
            updateCombinedDroneAwarenessFromVehicleOdometry();
        },
        odometry_options
    );

    // The other half of the measured-odometry transaction: same group and
    // queue depth as the odometry ingress.
    vehicle_local_position_sub_ = node_->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", odometry_qos,
        [this, lifetime = callback_lifetime_.token()](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
            const auto alive = lifetime.Enter();
            if (!alive.owns_lock()) return;
            ingestVehicleLocalPosition(*msg, node_->now());
            if (vehicle_odometry_adapter_history_ &&
                !vehicle_odometry_adapter_history_->empty())
                updateCombinedDroneAwarenessFromVehicleOdometry();
        },
        odometry_options);
    startOdometryIngress();

    vehicle_global_position_sub_ = node_->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
        "/fmu/out/vehicle_global_position",
        px4_sub_qos,
        [this](const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) {
            if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::vehicle_global_position_sub_: Vehicle global position received");
            iii_drone::adapters::px4::VehicleGlobalPositionAdapter adapter(*msg);
            vehicle_global_position_adapter_history_->Store(adapter);
        }
    );

    powerline_sub_ = node_->create_subscription<iii_drone_interfaces::msg::Powerline>(
        "/perception/pl_mapper/powerline",
        10,
        [this](const iii_drone_interfaces::msg::Powerline::SharedPtr msg) {
            if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::powerline_sub_: Powerline received");
            iii_drone::adapters::PowerlineAdapter adapter(*msg);
            powerline_adapter_history_->Store(adapter);
            updateCombinedDroneAwarenessFromPowerline();
        }
    );

	rclcpp::QoS gripper_sub_qos(rclcpp::KeepLast(1));
	gripper_sub_qos.best_effort();

    gripper_status_sub_ = node_->create_subscription<iii_drone_interfaces::msg::GripperStatus>(
        "/payload/charger_gripper/gripper_status",
        gripper_sub_qos,
        [this](const iii_drone_interfaces::msg::GripperStatus::SharedPtr msg) {
            if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::gripper_status_sub_: Gripper status received");
            iii_drone::adapters::GripperStatusAdapter adapter(*msg);
            gripper_status_adapter_history_->Store(adapter);
            updateCombinedDroneAwarenessFromGripperStatus();
        }
    );

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::Start(): Creating publish timer");
    publish_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(
            &CombinedDroneAwarenessHandler::publishMembers,
            this
        )
    );

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::Start(): Started CombinedDroneAwarenessHandler");

    is_started_ = true;

}

void CombinedDroneAwarenessHandler::startOdometryIngress() {
    odometry_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    odometry_executor_->add_callback_group(
        odometry_callback_group_, node_->get_node_base_interface());
    odometry_thread_ = std::thread([executor = odometry_executor_]() {
        executor->spin();
    });
}

void CombinedDroneAwarenessHandler::stopOdometryIngress() {
    if (!odometry_executor_) return;
    odometry_executor_->cancel();
    if (odometry_thread_.joinable()) odometry_thread_.join();
    odometry_executor_->remove_callback_group(odometry_callback_group_);
    odometry_executor_.reset();
}

void CombinedDroneAwarenessHandler::Stop() {

    if (!is_started_) {

        RCLCPP_ERROR(node_->get_logger(), "CombinedDroneAwarenessHandler::Stop(): CombinedDroneAwarenessHandler is already stopped");

        return;

    }

    is_started_ = false;

    // The ingress thread uses the awareness state reset below.
    stopOdometryIngress();

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::Stop(): Stopping CombinedDroneAwarenessHandler");

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::Stop(): Stopping publish timer");
    publish_timer_->cancel();
    publish_timer_.reset();
    publish_timer_ = nullptr;

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::Stop(): Stopping subscribers");
    vehicle_status_sub_->clear_on_new_message_callback();
    vehicle_status_sub_.reset();
    vehicle_status_sub_ = nullptr;

    vehicle_odometry_sub_->clear_on_new_message_callback();
    vehicle_odometry_sub_.reset();
    vehicle_odometry_sub_ = nullptr;
    vehicle_local_position_sub_->clear_on_new_message_callback();
    vehicle_local_position_sub_.reset();
    vehicle_local_position_sub_ = nullptr;

    powerline_sub_->clear_on_new_message_callback();
    powerline_sub_.reset();
    powerline_sub_ = nullptr;

    gripper_status_sub_->clear_on_new_message_callback();
    gripper_status_sub_.reset();
    gripper_status_sub_ = nullptr;

    vehicle_land_detected_sub_.reset();
    px4_land_state_.Store(std::nullopt);
    vehicle_local_position_setpoint_sub_.reset();
    px4_thrust_setpoint_.Store(std::nullopt);

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::Stop(): Stopping combined_drone_awareness_pub_timer_");
    combined_drone_awareness_pub_timer_->cancel();
    combined_drone_awareness_pub_timer_.reset();
    combined_drone_awareness_pub_timer_ = nullptr;

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::Stop(): Resetting combined_drone_awareness_adapter_");
    combined_drone_awareness_adapter_.reset();
    combined_drone_awareness_adapter_ = nullptr;

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::Stop(): Stopping ground altitude estimate");
    ground_altitude_update_timer_->cancel();
    ground_altitude_update_timer_.reset();
    ground_altitude_update_timer_ = nullptr;

    ground_altitudes_history_->clear();
    ground_altitudes_history_.reset();
    ground_altitudes_history_ = nullptr;

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::Stop(): Stopping Register Offboard Mode service");
    register_offboard_mode_srv_->clear_on_new_request_callback();
    register_offboard_mode_srv_.reset();
    register_offboard_mode_srv_ = nullptr;

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::Stop(): Clearing histories and atomics");
    {
        std::lock_guard<std::mutex> lock(odometry_ingest_mutex_);
        latest_local_reset_.reset();
        verified_local_reset_.reset();
        pending_odometry_.reset();
        local_provenance_invalid_ = false;
        ++odometry_source_epoch_;
        position_epoch_ = 0;
    }
    vehicle_status_adapter_history_->clear();
    vehicle_status_adapter_history_.reset();
    vehicle_status_adapter_history_ = nullptr;

    vehicle_odometry_adapter_history_->clear();
    vehicle_odometry_adapter_history_.reset();
    vehicle_odometry_adapter_history_ = nullptr;

    powerline_adapter_history_->clear();
    powerline_adapter_history_.reset();
    powerline_adapter_history_ = nullptr;

    gripper_status_adapter_history_->clear();
    gripper_status_adapter_history_.reset();
    gripper_status_adapter_history_ = nullptr;

    ground_altitude_estimate_.reset();
    ground_altitude_estimate_ = nullptr;

    ground_altitude_estimate_amsl_.reset();
    ground_altitude_estimate_amsl_ = nullptr;

    target_adapter_.reset();
    target_adapter_ = nullptr;

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::Stop(): Destroying tf_broadcaster_");
    tf_broadcaster_.reset();
    tf_broadcaster_ = nullptr;

    if (debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::Stop(): Stopped CombinedDroneAwarenessHandler");

}

iii_drone::control::State CombinedDroneAwarenessHandler::GetState() const {

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::GetState(): Getting state");

    if (!state_available()) {
        return iii_drone::control::State();
    }

    return (*vehicle_odometry_adapter_history_)[0].ToState();

}

std::optional<MeasuredOdometrySnapshot>
CombinedDroneAwarenessHandler::GetMeasuredOdometry() const {
    return measured_odometry_.Load();
}

OdometryIngressDiagnostics
CombinedDroneAwarenessHandler::TryGetOdometryIngressDiagnostics() const {
    OdometryIngressDiagnostics result;
    std::unique_lock<std::mutex> lock(odometry_ingest_mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {
        result.busy = true;
        return result;
    }
    result.available = true;
    result.latest_available = latest_accepted_odometry_available_;
    result.latest_source_sample_timestamp_us = latest_accepted_source_sample_us_;
    result.latest_reset_counter = latest_accepted_reset_counter_;
    result.latest_receipt_ros_ns = latest_accepted_receipt_ros_ns_;
    result.latest_accepted_steady_ns = latest_accepted_steady_ns_;
    result.total_callbacks = odometry_ingress_total_callbacks_;
    result.history_count = odometry_ingress_count_;
    const size_t start = (odometry_ingress_next_ +
        OdometryIngressDiagnostics::history_capacity - odometry_ingress_count_) %
        OdometryIngressDiagnostics::history_capacity;
    for (size_t i = 0; i < odometry_ingress_count_; ++i) {
        result.history[i] = odometry_ingress_history_[
            (start + i) % OdometryIngressDiagnostics::history_capacity];
    }
    return result;
}

VehicleNavigationEvidence
CombinedDroneAwarenessHandler::GetVehicleNavigationEvidence() const {
    return vehicle_navigation_evidence_.Load();
}

bool iii_drone::control::IsOperatorNativeControl(
    const VehicleNavigationEvidence & navigation,
    std::chrono::steady_clock::time_point now
) {
    if (!navigation.latest) return false;
    const auto & latest = *navigation.latest;
    if (latest.source_timestamp_us == 0 || latest.nav_state_timestamp_us == 0 ||
        latest.receipt > now || now - latest.receipt > kVehicleNavigationFreshness ||
        latest.failsafe) return false;
    if (navigation.last_external &&
        navigation.last_external->source_timestamp_us == latest.source_timestamp_us) return false;
    const auto nav_state = latest.nav_state;
    return nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD &&
        (nav_state < px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_EXTERNAL1 ||
         nav_state > px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_EXTERNAL8);
}

bool CombinedDroneAwarenessHandler::OperatorNativeControl() const {
    return IsOperatorNativeControl(
        vehicle_navigation_evidence_.Load(), std::chrono::steady_clock::now());
}

VehicleNavigationEvidence CombinedDroneAwarenessHandler::AdvanceVehicleNavigation(
    VehicleNavigationEvidence previous,
    const px4_msgs::msg::VehicleStatus & status,
    std::chrono::steady_clock::time_point receipt,
    bool external_mode
) {
    const auto reset = [&previous]() {
        previous.latest.reset();
        previous.last_external.reset();
        ++previous.source_epoch;
        return previous;
    };
    if (status.timestamp == 0 || status.nav_state_timestamp > status.timestamp ||
        (status.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER &&
         status.nav_state_timestamp == 0)) return reset();
    if (previous.latest) {
        if (status.timestamp < previous.latest->source_timestamp_us ||
            status.nav_state_timestamp < previous.latest->nav_state_timestamp_us) {
            return reset();
        }
        if (status.timestamp == previous.latest->source_timestamp_us) {
            // Identical source samples never refresh receipt. A changed nav
            // state at the same source stamp is not valid causal evidence.
            if (status.nav_state != previous.latest->nav_state ||
                status.nav_state_timestamp != previous.latest->nav_state_timestamp_us) {
                return reset();
            }
            return previous;
        }
    }
    VehicleNavigationSample sample{
        status.timestamp, status.nav_state_timestamp, status.nav_state,
        status.failsafe, receipt};
    previous.latest = sample;
    if (external_mode) previous.last_external = sample;
    return previous;
}

std::optional<MeasuredOdometrySnapshot>
CombinedDroneAwarenessHandler::AdvanceMeasuredOdometry(
    std::optional<MeasuredOdometrySnapshot> previous,
    const iii_drone::adapters::px4::VehicleOdometryAdapter & adapter,
    uint64_t source_sample_timestamp_us,
    const rclcpp::Time & receipt_stamp
) {
    if (previous && previous->reset_counter == adapter.reset_counter() &&
        source_sample_timestamp_us <= previous->source_sample_timestamp_us) {
        return previous;
    }
    return MeasuredOdometrySnapshot{
        adapter.ToState(), receipt_stamp, source_sample_timestamp_us,
        adapter.reset_counter(),
        PositionContinuityIdentity{0, 0, adapter.reset_counter(), false}
    };
}

bool CombinedDroneAwarenessHandler::state_available() const {

    return vehicle_status_adapter_history_ &&
        vehicle_odometry_adapter_history_ &&
        !vehicle_status_adapter_history_->empty() &&
        !vehicle_odometry_adapter_history_->empty();

}

bool CombinedDroneAwarenessHandler::samePositionBasis(
    const LocalResetMetadata & before, const LocalResetMetadata & after) {
    return after.xy == before.xy && after.z == before.z &&
        after.vxy == before.vxy && after.vz == before.vz &&
        after.xy_global == before.xy_global &&
        after.z_global == before.z_global &&
        after.origin_timestamp_us == before.origin_timestamp_us &&
        after.origin_lat == before.origin_lat &&
        after.origin_lon == before.origin_lon &&
        after.origin_alt == before.origin_alt;
}

bool CombinedDroneAwarenessHandler::headingOnly(
    const LocalResetMetadata & before, const LocalResetMetadata & after) {
    return after.heading == static_cast<uint8_t>(before.heading + 1) &&
        after.source_sample_us > before.source_sample_us &&
        after.source_sample_us - before.source_sample_us <= 250'000 &&
        after.receipt.get_clock_type() == before.receipt.get_clock_type() &&
        (after.receipt - before.receipt).seconds() >= 0.0 &&
        (after.receipt - before.receipt).seconds() <= 0.25 &&
        samePositionBasis(before, after);
}

bool CombinedDroneAwarenessHandler::metadataMatches(
    const LocalResetMetadata & metadata,
    const px4_msgs::msg::VehicleOdometry & message,
    const rclcpp::Time & receipt) const {
    if (message.pose_frame != px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED ||
        message.velocity_frame != px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED ||
        metadata.aggregate() != message.reset_counter ||
        message.timestamp_sample == 0 || metadata.source_sample_us == 0 ||
        receipt.get_clock_type() != metadata.receipt.get_clock_type()) return false;
    constexpr uint64_t max_source_gap_us = 250'000;
    const uint64_t gap = metadata.source_sample_us > message.timestamp_sample
        ? metadata.source_sample_us - message.timestamp_sample
        : message.timestamp_sample - metadata.source_sample_us;
    return gap <= max_source_gap_us &&
        std::abs((receipt - metadata.receipt).seconds()) <= 0.25;
}

void CombinedDroneAwarenessHandler::logResetClassification(
    bool heading_only, const char * context, uint8_t from_counter, uint8_t to_counter,
    uint64_t odometry_source_us, uint64_t local_source_us,
    uint64_t prior_local_source_us) const {
    // A qualified heading-only reset is a normal PX4 event that Core handles
    // continuously; only a position-continuity fault is a warning.
    char text[256];
    std::snprintf(text, sizeof(text),
        "PX4 odometry reset %u->%u classified %s%s (odometry_source_us=%llu local_source_us=%llu prior_local_source_us=%llu)",
        static_cast<unsigned>(from_counter), static_cast<unsigned>(to_counter),
        heading_only ? "heading-only position-continuous" : "position-continuity fault",
        context, static_cast<unsigned long long>(odometry_source_us),
        static_cast<unsigned long long>(local_source_us),
        static_cast<unsigned long long>(prior_local_source_us));
    // PX4 re-initialises its position estimate on touchdown/disarm; with the
    // vehicle disarmed no command depends on the fenced epoch, so that is a
    // normal event too. Airborne position faults stay warnings.
    const bool disarmed = vehicle_status_adapter_history_ &&
        !vehicle_status_adapter_history_->empty() &&
        (*vehicle_status_adapter_history_)[0].arming_state() !=
            iii_drone::adapters::px4::ARMING_STATE_ARMED;
    if (heading_only || disarmed) {
        RCLCPP_INFO(node_->get_logger(), "%s%s", text, disarmed && !heading_only ? " while disarmed" : "");
    } else {
        RCLCPP_WARN(node_->get_logger(), "%s", text);
    }
}

bool CombinedDroneAwarenessHandler::isolatedOdometryStampRegression(
    const MeasuredOdometrySnapshot & previous,
    const px4_msgs::msg::VehicleOdometry & message,
    const rclcpp::Time & receipt) const {
    if (message.reset_counter != previous.reset_counter ||
        !previous.position_continuity.source_qualified ||
        pending_odometry_ || local_provenance_invalid_ ||
        !latest_local_reset_ || !verified_local_reset_ ||
        latest_local_reset_->aggregate() != previous.reset_counter ||
        !samePositionBasis(*verified_local_reset_, *latest_local_reset_) ||
        receipt.get_clock_type() != previous.receipt_stamp.get_clock_type() ||
        receipt.get_clock_type() != latest_local_reset_->receipt.get_clock_type()) return false;
    const double since_accepted = (receipt - previous.receipt_stamp).seconds();
    const double since_metadata = (receipt - latest_local_reset_->receipt).seconds();
    return since_accepted >= 0.0 && since_accepted <= 0.25 &&
        since_metadata >= 0.0 && since_metadata <= 0.25;
}

bool CombinedDroneAwarenessHandler::isolatedLocalStampRegression(
    const LocalResetMetadata & metadata) const {
    const auto current = measured_odometry_.Load();
    if (!current || !latest_local_reset_ || pending_odometry_ ||
        !current->position_continuity.source_qualified ||
        current->reset_counter != metadata.aggregate() ||
        metadata.aggregate() != latest_local_reset_->aggregate() ||
        metadata.heading != latest_local_reset_->heading ||
        !samePositionBasis(*latest_local_reset_, metadata) ||
        metadata.receipt.get_clock_type() != latest_local_reset_->receipt.get_clock_type()) return false;
    const double since_metadata = (metadata.receipt - latest_local_reset_->receipt).seconds();
    return since_metadata >= 0.0 && since_metadata <= 0.25;
}

void CombinedDroneAwarenessHandler::acceptMeasuredOdometry(
    const px4_msgs::msg::VehicleOdometry & message,
    const rclcpp::Time & receipt, bool qualified, bool new_position_epoch,
    bool force_source_fault) {
    if (new_position_epoch) ++position_epoch_;
    iii_drone::adapters::px4::VehicleOdometryAdapter adapter(message);
    auto snapshot = force_source_fault
        ? std::optional<MeasuredOdometrySnapshot>(MeasuredOdometrySnapshot{
            adapter.ToState(), receipt, message.timestamp_sample,
            message.reset_counter,
            PositionContinuityIdentity{odometry_source_epoch_, position_epoch_,
                                       message.reset_counter, false}})
        : AdvanceMeasuredOdometry(measured_odometry_.Load(), adapter,
            message.timestamp_sample, receipt);
    if (!snapshot || snapshot->source_sample_timestamp_us != message.timestamp_sample) return;
    snapshot->position_continuity = PositionContinuityIdentity{
        odometry_source_epoch_, position_epoch_, message.reset_counter, qualified};
    vehicle_odometry_adapter_history_->Store(adapter);
    measured_odometry_.Store(snapshot);
    ++accepted_odometry_samples_;
    latest_accepted_odometry_available_ = true;
    latest_accepted_source_sample_us_ = message.timestamp_sample;
    latest_accepted_reset_counter_ = message.reset_counter;
    latest_accepted_receipt_ros_ns_ = receipt.nanoseconds();
    latest_accepted_steady_ns_ = steadyNowNs();
}

void CombinedDroneAwarenessHandler::ingestVehicleOdometry(
    const px4_msgs::msg::VehicleOdometry & message, const rclcpp::Time & receipt) {
    const int64_t callback_entry_ns = steadyNowNs();
    std::lock_guard<std::mutex> lock(odometry_ingest_mutex_);
    const int64_t lock_acquired_ns = steadyNowNs();
    const uint64_t accepted_before = accepted_odometry_samples_;
    const bool pending_before = pending_odometry_.has_value();
    const auto record_ingress = onScopeExit([&] {
        OdometryIngressEvent event;
        event.source_sample_timestamp_us = message.timestamp_sample;
        event.callback_receipt_ros_ns = receipt.nanoseconds();
        event.callback_entry_steady_ns = callback_entry_ns;
        event.lock_acquired_steady_ns = lock_acquired_ns;
        event.completed_steady_ns = steadyNowNs();
        event.accepted = accepted_odometry_samples_ != accepted_before;
        event.accepted_steady_ns = event.accepted ? latest_accepted_steady_ns_ : 0;
        event.reset_counter = message.reset_counter;
        event.pending_before = pending_before;
        event.pending_after = pending_odometry_.has_value();
        odometry_ingress_history_[odometry_ingress_next_] = event;
        odometry_ingress_next_ = (odometry_ingress_next_ + 1) %
            OdometryIngressDiagnostics::history_capacity;
        odometry_ingress_count_ = std::min(odometry_ingress_count_ + 1,
            OdometryIngressDiagnostics::history_capacity);
        ++odometry_ingress_total_callbacks_;
    });
    const auto previous = measured_odometry_.Load();
    if (message.timestamp_sample == 0) return;
    if (message.pose_frame != px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED ||
        message.velocity_frame != px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED) {
        ++odometry_source_epoch_;
        ++position_epoch_;
        latest_local_reset_.reset();
        verified_local_reset_.reset();
        pending_odometry_.reset();
        acceptMeasuredOdometry(message, receipt, false, false, true);
        return;
    }
    if (previous && message.timestamp_sample < previous->source_sample_timestamp_us &&
        isolatedOdometryStampRegression(*previous, message, receipt)) {
        ++discarded_stamp_regressions_;
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
            "Discarded isolated PX4 odometry timestamp regression (raw_reset=%u odometry_source_us=%llu retained_source_us=%llu discarded_total=%llu)",
            static_cast<unsigned>(message.reset_counter),
            static_cast<unsigned long long>(message.timestamp_sample),
            static_cast<unsigned long long>(previous->source_sample_timestamp_us),
            static_cast<unsigned long long>(discarded_stamp_regressions_));
        return;
    }
    if (previous && message.timestamp_sample < previous->source_sample_timestamp_us) {
        ++odometry_source_epoch_;
        ++position_epoch_;
        latest_local_reset_.reset();
        verified_local_reset_.reset();
        pending_odometry_.reset();
        acceptMeasuredOdometry(message, receipt, false, false, true);
        return;
    }
    if (previous && message.timestamp_sample == previous->source_sample_timestamp_us) {
        if (message.reset_counter != previous->reset_counter) {
            ++odometry_source_epoch_;
            ++position_epoch_;
            latest_local_reset_.reset();
            verified_local_reset_.reset();
            acceptMeasuredOdometry(message, receipt, false, false, true);
        }
        return;
    }
    if (pending_odometry_) {
        if (message.timestamp_sample <= pending_odometry_->message.timestamp_sample &&
            message.reset_counter == pending_odometry_->message.reset_counter) return;
        if (message.reset_counter == previous->reset_counter) return;
        if (message.reset_counter != pending_odometry_->message.reset_counter) {
            ++position_epoch_;
            pending_odometry_.reset();
            verified_local_reset_.reset();
            acceptMeasuredOdometry(message, receipt, false, false);
            return;
        }
    }
    if (!previous || message.reset_counter == previous->reset_counter) {
        const bool qualified = latest_local_reset_ &&
            metadataMatches(*latest_local_reset_, message, receipt) &&
            (!verified_local_reset_ ||
             latest_local_reset_->source_sample_us >= verified_local_reset_->source_sample_us);
        if (qualified && verified_local_reset_ &&
            !samePositionBasis(*verified_local_reset_, *latest_local_reset_))
            ++position_epoch_;
        if (qualified) verified_local_reset_ = latest_local_reset_;
        acceptMeasuredOdometry(message, receipt,
            qualified || (previous && previous->position_continuity.source_qualified), false);
        return;
    }
    if (latest_local_reset_ && verified_local_reset_ &&
        previous->position_continuity.source_qualified &&
        metadataMatches(*latest_local_reset_, message, receipt)) {
        const bool heading_only = headingOnly(*verified_local_reset_, *latest_local_reset_) &&
            message.reset_counter == static_cast<uint8_t>(previous->reset_counter + 1);
        logResetClassification(heading_only, "",
            previous->reset_counter, message.reset_counter, message.timestamp_sample,
            latest_local_reset_->source_sample_us, verified_local_reset_->source_sample_us);
        acceptMeasuredOdometry(message, receipt, heading_only, !heading_only);
        verified_local_reset_ = latest_local_reset_;
        pending_odometry_.reset();
        return;
    }
    // Wait for reordered metadata without refreshing the accepted sample.
    // If none arrives, its original receipt expires under the existing guard.
    if (!pending_odometry_) {
        // Transient: the matching local-position record normally follows
        // within one DDS reorder window; expiry is fenced by freshness.
        RCLCPP_INFO(node_->get_logger(),
            "PX4 odometry reset %u->%u awaiting source-qualified local-position metadata (odometry_source_us=%llu prior_source_us=%llu)",
            static_cast<unsigned>(previous->reset_counter),
            static_cast<unsigned>(message.reset_counter),
            static_cast<unsigned long long>(message.timestamp_sample),
            static_cast<unsigned long long>(previous->source_sample_timestamp_us));
    }
    pending_odometry_ = PendingOdometry{message, receipt};
}

void CombinedDroneAwarenessHandler::ingestVehicleLocalPosition(
    const px4_msgs::msg::VehicleLocalPosition & message,
    const rclcpp::Time & receipt) {
    std::lock_guard<std::mutex> lock(odometry_ingest_mutex_);
    if (message.timestamp_sample == 0 || message.ref_timestamp == 0 ||
        !message.xy_global || !message.z_global ||
        !message.xy_valid || !message.z_valid ||
        !message.v_xy_valid || !message.v_z_valid ||
        (message.xy_global && (!std::isfinite(message.ref_lat) ||
                               !std::isfinite(message.ref_lon))) ||
        (message.z_global && !std::isfinite(message.ref_alt))) {
        const auto current = measured_odometry_.Load();
        if (current && !local_provenance_invalid_) {
            ++odometry_source_epoch_;
            ++position_epoch_;
            auto invalidated = *current;
            invalidated.position_continuity = PositionContinuityIdentity{
                odometry_source_epoch_, position_epoch_, current->reset_counter, false};
            measured_odometry_.Store(invalidated); // original receipt/sample remain authoritative
            RCLCPP_WARN(node_->get_logger(),
                "PX4 local-position reset provenance became invalid (raw_reset=%u odometry_source_us=%llu local_source_us=%llu)",
                static_cast<unsigned>(current->reset_counter),
                static_cast<unsigned long long>(current->source_sample_timestamp_us),
                static_cast<unsigned long long>(message.timestamp_sample));
        }
        latest_local_reset_.reset();
        verified_local_reset_.reset();
        pending_odometry_.reset();
        local_provenance_invalid_ = true;
        return;
    }
    local_provenance_invalid_ = false;
    LocalResetMetadata metadata;
    metadata.source_sample_us = message.timestamp_sample;
    metadata.receipt = receipt;
    metadata.xy = message.xy_reset_counter;
    metadata.z = message.z_reset_counter;
    metadata.vxy = message.vxy_reset_counter;
    metadata.vz = message.vz_reset_counter;
    metadata.heading = message.heading_reset_counter;
    metadata.xy_global = message.xy_global;
    metadata.z_global = message.z_global;
    metadata.origin_timestamp_us = message.ref_timestamp;
    metadata.origin_lat = message.ref_lat;
    metadata.origin_lon = message.ref_lon;
    metadata.origin_alt = message.ref_alt;
    if (latest_local_reset_ &&
        metadata.source_sample_us <= latest_local_reset_->source_sample_us) {
        if (metadata.source_sample_us < latest_local_reset_->source_sample_us) {
            if (latest_local_reset_->source_sample_us - metadata.source_sample_us <= 250'000)
                return; // DDS reorder, never refresh provenance.
            if (isolatedLocalStampRegression(metadata)) {
                ++discarded_stamp_regressions_;
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                    "Discarded isolated PX4 local-position timestamp regression (aggregate_reset=%u local_source_us=%llu retained_source_us=%llu discarded_total=%llu)",
                    static_cast<unsigned>(metadata.aggregate()),
                    static_cast<unsigned long long>(metadata.source_sample_us),
                    static_cast<unsigned long long>(latest_local_reset_->source_sample_us),
                    static_cast<unsigned long long>(discarded_stamp_regressions_));
                return; // Trusted metadata and its original receipt stay authoritative.
            }
            ++odometry_source_epoch_;
            ++position_epoch_;
            verified_local_reset_.reset();
            latest_local_reset_.reset();
            auto current = measured_odometry_.Load();
            if (current) {
                current->position_continuity.source_epoch = odometry_source_epoch_;
                current->position_continuity.position_epoch = position_epoch_;
                current->position_continuity.source_qualified = false;
                measured_odometry_.Store(current);
            }
            return;
        }
        if (metadata.aggregate() != latest_local_reset_->aggregate() ||
            metadata.heading != latest_local_reset_->heading ||
            !samePositionBasis(metadata, *latest_local_reset_)) {
            ++odometry_source_epoch_;
            ++position_epoch_;
            verified_local_reset_.reset();
            latest_local_reset_.reset();
            auto current = measured_odometry_.Load();
            if (current) {
                current->position_continuity.source_epoch = odometry_source_epoch_;
                current->position_continuity.position_epoch = position_epoch_;
                current->position_continuity.source_qualified = false;
                measured_odometry_.Store(current);
            }
        }
        return;
    }
    latest_local_reset_ = metadata;
    if (pending_odometry_ &&
        metadataMatches(metadata, pending_odometry_->message,
            pending_odometry_->receipt)) {
        const auto previous = measured_odometry_.Load();
        const bool heading_only = previous && verified_local_reset_ &&
            previous->position_continuity.source_qualified &&
            headingOnly(*verified_local_reset_, metadata) &&
            pending_odometry_->message.reset_counter ==
                static_cast<uint8_t>(previous->reset_counter + 1);
        logResetClassification(heading_only, " after DDS reorder",
            previous ? previous->reset_counter : 0U,
            pending_odometry_->message.reset_counter,
            pending_odometry_->message.timestamp_sample, metadata.source_sample_us,
            verified_local_reset_ ? verified_local_reset_->source_sample_us : 0);
        acceptMeasuredOdometry(pending_odometry_->message,
            pending_odometry_->receipt, heading_only, !heading_only);
        verified_local_reset_ = metadata;
        pending_odometry_.reset();
        return;
    }
    const auto current = measured_odometry_.Load();
    const uint64_t source_gap = current
        ? (metadata.source_sample_us > current->source_sample_timestamp_us
            ? metadata.source_sample_us - current->source_sample_timestamp_us
            : current->source_sample_timestamp_us - metadata.source_sample_us)
        : 0;
    if (current && metadata.aggregate() == current->reset_counter &&
        source_gap <= 250'000 &&
        std::abs((receipt - current->receipt_stamp).seconds()) <= 0.25) {
        auto qualified = *current;
        if (verified_local_reset_ &&
            !samePositionBasis(*verified_local_reset_, metadata)) {
            ++position_epoch_;
        }
        qualified.position_continuity = PositionContinuityIdentity{
            odometry_source_epoch_, position_epoch_, current->reset_counter, true};
        measured_odometry_.Store(qualified); // receipt and sample identity are unchanged.
        verified_local_reset_ = metadata;
    }
}

iii_drone::control::State CombinedDroneAwarenessHandler::ComputeTargetState(const iii_drone::adapters::TargetAdapter & target_adapter) const {

    transform_matrix_t target_transform = ComputeTargetTransform(target_adapter);

    // RCLCPP_DEBUG(
    //     node_->get_logger(),
    //     "Target adapter id: %d",
    //     target_adapter.target_id()
    // );

    // RCLCPP_DEBUG(
    //     node_->get_logger(),
    //     "Target adapter type: %d",
    //     target_adapter.target_type()
    // );

    // RCLCPP_DEBUG(
    //     node_->get_logger(),
    //     "Target adapter transform:\n%f, %f, %f, %f\n%f, %f, %f, %f\n%f, %f, %f, %f\n%f, %f, %f, %f",
    //     target_adapter.target_transform()(0, 0),
    //     target_adapter.target_transform()(0, 1),
    //     target_adapter.target_transform()(0, 2),
    //     target_adapter.target_transform()(0, 3),
    //     target_adapter.target_transform()(1, 0),
    //     target_adapter.target_transform()(1, 1),
    //     target_adapter.target_transform()(1, 2),
    //     target_adapter.target_transform()(1, 3),
    //     target_adapter.target_transform()(2, 0),
    //     target_adapter.target_transform()(2, 1),
    //     target_adapter.target_transform()(2, 2),
    //     target_adapter.target_transform()(2, 3),
    //     target_adapter.target_transform()(3, 0),
    //     target_adapter.target_transform()(3, 1),
    //     target_adapter.target_transform()(3, 2),
    //     target_adapter.target_transform()(3, 3)
    // );

    // RCLCPP_DEBUG(
    //     node_->get_logger(),
    //     "Target transform:\n%f, %f, %f, %f\n%f, %f, %f, %f\n%f, %f, %f, %f\n%f, %f, %f, %f",
    //     target_transform(0, 0),
    //     target_transform(0, 1),
    //     target_transform(0, 2),
    //     target_transform(0, 3),
    //     target_transform(1, 0),
    //     target_transform(1, 1),
    //     target_transform(1, 2),
    //     target_transform(1, 3),
    //     target_transform(2, 0),
    //     target_transform(2, 1),
    //     target_transform(2, 2),
    //     target_transform(2, 3),
    //     target_transform(3, 0),
    //     target_transform(3, 1),
    //     target_transform(3, 2),
    //     target_transform(3, 3)
    // );

    return State(
        target_transform.block<3, 1>(0, 3),
        vector_t::Zero(),
        matToQuat(target_transform.block<3, 3>(0, 0)),
        vector_t::Zero()
    );

}

iii_drone::types::transform_matrix_t CombinedDroneAwarenessHandler::ComputeTargetTransform(const iii_drone::adapters::TargetAdapter & target_adapter) const {

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::ComputeTargetTransform(): Computing target transform");

    iii_drone::control::Reference reference;

    if (target_adapter.target_type() == iii_drone::adapters::TARGET_TYPE_CABLE) {

        iii_drone::adapters::PowerlineAdapter powerline_adapter = (*powerline_adapter_history_)[0];

        iii_drone::adapters::SingleLineAdapter target_line;

        try {

            target_line = powerline_adapter.GetLine(target_adapter.target_id());

        } catch (std::runtime_error & e) {

            if (debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::ComputeTargetTransform: target line with id %d not found", target_adapter.target_id());

            throw std::runtime_error("CombinedDroneAwarenessHandler::ComputeTargetTransform(): target line with id " + std::to_string(target_adapter.target_id()) + " not found");
        }

        geometry_msgs::msg::PoseStamped target_object_pose_stamped = target_line.ToPoseStampedMsg();

        geometry_msgs::msg::PoseStamped target_object_pose_stamped_world;

        try {

            // Powerline detections can be stamped before the matching drone TF has
            // reached this buffer. Use the latest available transform here; this
            // function runs in the setpoint reference path and must not block.
            target_object_pose_stamped.header.stamp = builtin_interfaces::msg::Time();
            target_object_pose_stamped_world = tf_buffer_->transform(
                target_object_pose_stamped,
                configuration_->GetParameter("/tf/world_frame_id").as_string()
            );

        } catch (tf2::TransformException & e) {

            if (debug_) RCLCPP_DEBUG(
                node_->get_logger(),
                "CombinedDroneAwarenessHandler::ComputeTargetTransform: could not transform target object pose to world frame: %s",
                e.what()
            );

            throw std::runtime_error("CombinedDroneAwarenessHandler::ComputeTargetTransform(): could not transform target object pose to world frame: " + std::string(e.what()));

        }

        // RCLCPP_DEBUG(
        //     node_->get_logger(),
        //     "CombinedDroneAwarenessHandler::ComputeTargetTransform: target object pose in world frame: %f, %f, %f, %f, %f, %f, %f",
        //     target_object_pose_stamped_world.pose.position.x,
        //     target_object_pose_stamped_world.pose.position.y,
        //     target_object_pose_stamped_world.pose.position.z,
        //     target_object_pose_stamped_world.pose.orientation.w,
        //     target_object_pose_stamped_world.pose.orientation.x,
        //     target_object_pose_stamped_world.pose.orientation.y,
        //     target_object_pose_stamped_world.pose.orientation.z
        // );

        iii_drone::types::transform_matrix_t ref_T_c = target_adapter.target_transform();
        iii_drone::types::pose_t w_T_c_pose = poseFromPoseMsg(target_object_pose_stamped_world.pose);
        // RCLCPP_DEBUG()
        iii_drone::types::transform_matrix_t w_T_c = createTransformMatrix(
            w_T_c_pose.position,
            w_T_c_pose.orientation
        );
        iii_drone::types::transform_matrix_t w_T_ref = w_T_c * ref_T_c.inverse();
        std::string target_reference_frame_id = target_adapter.reference_frame_id();
        geometry_msgs::msg::TransformStamped ref_T_drone_msg = tf_buffer_->lookupTransform(
            target_reference_frame_id,
            configuration_->GetParameter("/tf/drone_frame_id").as_string(),
            tf2::TimePointZero
        );
        iii_drone::types::transform_matrix_t ref_T_drone = transformMatrixFromTransformMsg(ref_T_drone_msg.transform);
        iii_drone::types::transform_matrix_t w_T_d = w_T_ref * ref_T_drone;

        if (debug_) {
            RCLCPP_DEBUG_THROTTLE(
                node_->get_logger(),
                *node_->get_clock(),
                1000,
                "CombinedDroneAwarenessHandler::ComputeTargetTransform(): target_id=%d line_world=[%.3f, %.3f, %.3f] reference_frame=%s target_transform_translation=[%.3f, %.3f, %.3f] drone_target_world=[%.3f, %.3f, %.3f]",
                target_adapter.target_id(),
                w_T_c(0, 3),
                w_T_c(1, 3),
                w_T_c(2, 3),
                target_reference_frame_id.c_str(),
                ref_T_c(0, 3),
                ref_T_c(1, 3),
                ref_T_c(2, 3),
                w_T_d(0, 3),
                w_T_d(1, 3),
                w_T_d(2, 3)
            );
        }

        return w_T_d;

    } else {

        std::string msg = "CombinedDroneAwarenessHandler::ComputeTargetTransform(): target type " + std::to_string(target_adapter.target_type()) + " not implemented";

        throw std::runtime_error(msg);

    }
}

iii_drone::types::pose_t CombinedDroneAwarenessHandler::GetPoseOfTarget(const iii_drone::adapters::TargetAdapter & target_adapter) const {

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::GetPoseOfTarget(): Getting the pose of the target");

    iii_drone::types::pose_t pose;

    if (target_adapter.target_type() == iii_drone::adapters::TARGET_TYPE_CABLE) {

        iii_drone::adapters::PowerlineAdapter powerline_adapter = (*powerline_adapter_history_)[0];

        iii_drone::adapters::SingleLineAdapter target_line;

        try {

            target_line = powerline_adapter.GetLine(target_adapter.target_id());

        } catch (std::runtime_error & e) {

            throw std::runtime_error("CombinedDroneAwarenessHandler::GetPoseOfTarget(): target line with id " + std::to_string(target_adapter.target_id()) + " not found");
        }

        geometry_msgs::msg::PoseStamped target_object_pose_stamped = target_line.ToPoseStampedMsg();

        geometry_msgs::msg::PoseStamped target_object_pose_stamped_world;
        try {
            target_object_pose_stamped_world = tf_buffer_->transform(
                target_object_pose_stamped,
                configuration_->GetParameter("/tf/world_frame_id").as_string()
            );
        } catch (tf2::TransformException & e) {
            target_object_pose_stamped.header.stamp = builtin_interfaces::msg::Time();
            target_object_pose_stamped_world = tf_buffer_->transform(
                target_object_pose_stamped,
                configuration_->GetParameter("/tf/world_frame_id").as_string()
            );
        }

        pose = poseFromPoseMsg(target_object_pose_stamped_world.pose);

    } else {

        throw std::runtime_error("CombinedDroneAwarenessHandler::GetPoseOfTarget(): target type not implemented");

    }

    return pose;

}

iii_drone::adapters::PowerlineAdapter CombinedDroneAwarenessHandler::GetPowerlineAdapter() const {

    if (powerline_adapter_history_->empty()) {
        throw std::runtime_error("CombinedDroneAwarenessHandler::GetPowerlineAdapter(): no powerline available");
    }

    return (*powerline_adapter_history_)[0];

}

void CombinedDroneAwarenessHandler::SetTarget(iii_drone::adapters::TargetAdapter target_adapter) {

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::SetTarget(): Setting target");
    
    if (target_adapter.target_type() != iii_drone::adapters::TARGET_TYPE_CABLE) {
        
        throw std::runtime_error("CombinedDroneAwarenessHandler::SetTarget: target type not implemented");

    }

    target_adapter_->Store(target_adapter);

    updateCombinedDroneAwarenessFromTarget();

}

void CombinedDroneAwarenessHandler::ClearTarget() {

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::ClearTarget(): Clearing target");

    target_adapter_->Store(TargetAdapter());

    updateCombinedDroneAwarenessFromTarget();

}

const CombinedDroneAwarenessAdapter CombinedDroneAwarenessHandler::adapter() const {
    return *combined_drone_awareness_adapter_;
}

bool CombinedDroneAwarenessHandler::armed() const {
    return (*combined_drone_awareness_adapter_)->armed();
}

bool CombinedDroneAwarenessHandler::offboard() const {
    return (*combined_drone_awareness_adapter_)->offboard();
}

bool CombinedDroneAwarenessHandler::has_target() const {
    return (*combined_drone_awareness_adapter_)->has_target();
}

bool CombinedDroneAwarenessHandler::target_position_known() const {
    return (*combined_drone_awareness_adapter_)->target_position_known();
}

TargetAdapter CombinedDroneAwarenessHandler::target_adapter() const {
    return target_adapter_->Load();
}

bool CombinedDroneAwarenessHandler::on_ground() const {
    return (*combined_drone_awareness_adapter_)->on_ground();
}

bool CombinedDroneAwarenessHandler::on_cable() const {
    return (*combined_drone_awareness_adapter_)->on_cable();
}

bool CombinedDroneAwarenessHandler::in_flight() const {
    return (*combined_drone_awareness_adapter_)->in_flight();
}

int CombinedDroneAwarenessHandler::on_cable_id() const {
    return (*combined_drone_awareness_adapter_)->on_cable_id();
}

double CombinedDroneAwarenessHandler::ground_altitude_estimate() const {
    return (*combined_drone_awareness_adapter_)->ground_altitude_estimate();
}

drone_location_t CombinedDroneAwarenessHandler::drone_location(int &on_cable_id) const {
    on_cable_id = this->on_cable_id();
    return (*combined_drone_awareness_adapter_)->drone_location();
}

bool CombinedDroneAwarenessHandler::gripper_open() const {
    return (*combined_drone_awareness_adapter_)->gripper_open();
}

std::optional<CombinedDroneAwarenessHandler::Px4LandState> CombinedDroneAwarenessHandler::px4_land_state() const {
    return px4_land_state_.Load();
}

bool CombinedDroneAwarenessHandler::px4_airborne(std::chrono::steady_clock::time_point now) const {
    const auto state = px4_land_state_.Load();
    return state &&
        now - state->received_at <= kPx4LandStateMaxAge &&
        !state->landed && !state->maybe_landed && !state->ground_contact;
}

std::optional<HoverThrustMeter::Estimate> CombinedDroneAwarenessHandler::measured_hover_thrust() const {
    std::lock_guard<std::mutex> lock(hover_thrust_meter_mutex_);
    return hover_thrust_meter_.estimate();
}

std::optional<CombinedDroneAwarenessHandler::Px4ThrustSetpoint> CombinedDroneAwarenessHandler::px4_thrust_setpoint(
    std::chrono::steady_clock::time_point now
) const {
    const auto setpoint = px4_thrust_setpoint_.Load();
    if (!setpoint || now - setpoint->received_at > kPx4ThrustSetpointMaxAge ||
        !std::isfinite(setpoint->thrust_up)) {
        return std::nullopt;
    }
    return setpoint;
}

drone_location_t CombinedDroneAwarenessHandler::drone_location() const {
    return (*combined_drone_awareness_adapter_)->drone_location();
}

tf2_ros::Buffer::SharedPtr CombinedDroneAwarenessHandler::tf_buffer() const {
    return tf_buffer_;
}

void CombinedDroneAwarenessHandler::registerOffboardMode(int navigation_state_id) {

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::RegisterOffboardMode(): Registering offboard mode with id %d", navigation_state_id);

    std::vector<int> offboard_states = offboard_nav_state_ids_.Load();

    if (std::find(offboard_states.begin(), offboard_states.end(), navigation_state_id) == offboard_states.end()) {
        offboard_states.push_back(navigation_state_id);
        offboard_nav_state_ids_.Store(offboard_states);
    }

}

void CombinedDroneAwarenessHandler::deregisterOffboardMode(int navigation_state_id) {

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::DeregisterOffboardMode(): Deregistering offboard mode with id %d", navigation_state_id);

    std::vector<int> offboard_states = offboard_nav_state_ids_.Load();

    offboard_states.erase(std::remove(offboard_states.begin(), offboard_states.end(), navigation_state_id), offboard_states.end());

    offboard_nav_state_ids_.Store(offboard_states);

}

void CombinedDroneAwarenessHandler::updateCombinedDroneAwareness() {

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::updateCombinedDroneAwareness(): Updating combined drone awareness");

    // Update the combined drone awareness
    std::lock_guard<std::mutex> awareness_lock(awareness_update_mutex_);
    CombinedDroneAwarenessAdapter adapter = *combined_drone_awareness_adapter_;

    updateCombinedDroneAwarenessFromVehicleStatus(adapter);
    updateCombinedDroneAwarenessFromVehicleOdometry(adapter);
    updateCombinedDroneAwarenessFromPowerline(adapter);
    updateCombinedDroneAwarenessFromTarget(adapter);
    updateCombinedDroneAwarenessFromGripperStatus(adapter);

    // Update the combined drone awareness
    *combined_drone_awareness_adapter_ = adapter;

}

void CombinedDroneAwarenessHandler::updateCombinedDroneAwarenessFromVehicleStatus() {

    std::lock_guard<std::mutex> awareness_lock(awareness_update_mutex_);
    CombinedDroneAwarenessAdapter adapter = *combined_drone_awareness_adapter_;

    updateCombinedDroneAwarenessFromVehicleStatus(adapter);
    updateCombinedDroneAwarenessFromTarget(adapter);

    *combined_drone_awareness_adapter_ = adapter;

}

void CombinedDroneAwarenessHandler::updateCombinedDroneAwarenessFromVehicleStatus(CombinedDroneAwarenessAdapter & adapter) {

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::updateCombinedDroneAwarenessFromVehicleStatus(): Updating combined drone awareness from vehicle status");

    // Update the armed and offboard states
    if (vehicle_status_adapter_history_->empty()) {
        adapter.armed() = false;
        adapter.offboard() = false;
    } else {
        adapter.armed() = (*vehicle_status_adapter_history_)[0].arming_state() == iii_drone::adapters::px4::ARMING_STATE_ARMED;

        int navigation_state_id = (*vehicle_status_adapter_history_)[0].nav_state();

        std::vector<int> offboard_states = offboard_nav_state_ids_.Load();

        adapter.offboard() = (std::find(offboard_states.begin(), offboard_states.end(), navigation_state_id) != offboard_states.end());

    }

    // Update the drone location:
    updateDroneLocation(adapter);

}

void CombinedDroneAwarenessHandler::updateCombinedDroneAwarenessFromVehicleOdometry() {

    std::lock_guard<std::mutex> awareness_lock(awareness_update_mutex_);
    CombinedDroneAwarenessAdapter adapter = *combined_drone_awareness_adapter_;

    updateCombinedDroneAwarenessFromVehicleOdometry(adapter);
    updateCombinedDroneAwarenessFromTarget(adapter);

    *combined_drone_awareness_adapter_ = adapter;

}

void CombinedDroneAwarenessHandler::updateCombinedDroneAwarenessFromVehicleOdometry(CombinedDroneAwarenessAdapter & adapter) {

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::updateCombinedDroneAwarenessFromVehicleOdometry(): Updating combined drone awareness from vehicle odometry");

    // Update the ground altitude estimate
    updateGroundAltitudeEstimate(
        adapter.armed(),
        adapter.has_target()
    );

    adapter.ground_altitude_estimate() = ground_altitude_estimate_->Load();
    adapter.ground_altitude_estimate_amsl() = ground_altitude_estimate_amsl_->Load();

    // Update the drone location
    updateDroneLocation(adapter);

    // Update state
    adapter.state() = GetState();

}

void CombinedDroneAwarenessHandler::updateCombinedDroneAwarenessFromPowerline() {

    std::lock_guard<std::mutex> awareness_lock(awareness_update_mutex_);
    CombinedDroneAwarenessAdapter adapter = *combined_drone_awareness_adapter_;

    updateCombinedDroneAwarenessFromPowerline(adapter);
    updateCombinedDroneAwarenessFromTarget(adapter);

    *combined_drone_awareness_adapter_ = adapter;

}

void CombinedDroneAwarenessHandler::updateCombinedDroneAwarenessFromPowerline(CombinedDroneAwarenessAdapter & adapter) {

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::updateCombinedDroneAwarenessFromPowerline(): Updating combined drone awareness from powerline");

    // Update the target cable position known
    if (powerline_adapter_history_->empty() || !adapter.has_target()) {
        return;
    } else if (adapter.target_adapter().target_type() == TARGET_TYPE_CABLE) {
        int target_cable_id = adapter.target_adapter().target_id();
        adapter.target_position_known() = (*powerline_adapter_history_)[0].HasLine(target_cable_id);
    }

    // Update the drone location
    updateDroneLocation(adapter);

}

void CombinedDroneAwarenessHandler::updateCombinedDroneAwarenessFromGripperStatus() {

    std::lock_guard<std::mutex> awareness_lock(awareness_update_mutex_);
    CombinedDroneAwarenessAdapter adapter = *combined_drone_awareness_adapter_;

    updateCombinedDroneAwarenessFromTarget(adapter);
    updateCombinedDroneAwarenessFromGripperStatus(adapter);

    *combined_drone_awareness_adapter_ = adapter;

}

void CombinedDroneAwarenessHandler::updateCombinedDroneAwarenessFromGripperStatus(CombinedDroneAwarenessAdapter & adapter) {

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::updateCombinedDroneAwarenessFromGripperStatus(): Updating combined drone awareness from gripper status");

    // Update the gripper open flag
    if (gripper_status_adapter_history_->empty()) {
        adapter.gripper_open() = true;
    } else {
        adapter.gripper_open() = (*gripper_status_adapter_history_)[0].open();
    }

    updateDroneLocation(adapter);

}

void CombinedDroneAwarenessHandler::updateCombinedDroneAwarenessFromTarget() {

    std::lock_guard<std::mutex> awareness_lock(awareness_update_mutex_);
    CombinedDroneAwarenessAdapter adapter = *combined_drone_awareness_adapter_;

    updateCombinedDroneAwarenessFromTarget(adapter);

    *combined_drone_awareness_adapter_ = adapter;

}

void CombinedDroneAwarenessHandler::updateCombinedDroneAwarenessFromTarget(CombinedDroneAwarenessAdapter & adapter) {

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::updateCombinedDroneAwarenessFromTarget(): Updating combined drone awareness from target");

    // Update the target cable id
    adapter.target_adapter() = target_adapter_->Load();

    // Update the target cable position known
    if (adapter.target_adapter().target_type() == TARGET_TYPE_CABLE) {
        if (powerline_adapter_history_->empty()) {
            adapter.target_position_known() = false;
        } else {
            int target_cable_id = adapter.target_adapter().target_id();
            adapter.target_position_known() = (*powerline_adapter_history_)[0].HasLine(target_cable_id);
        }
    } else {
        adapter.target_position_known() = false;
    }

}

void CombinedDroneAwarenessHandler::updateGroundAltitudeEstimate(
    bool armed,
    bool has_target_cable
) {

    if (!has_found_initial_location_) {
        return;
    }

    bool is_on_ground = !armed && !has_target_cable;

    if (!is_on_ground) {
        ground_altitude_update_timer_->cancel();
        return;
    }

    if (vehicle_odometry_adapter_history_->empty()) {
        return;
    }

    if (!ground_altitude_update_timer_->is_canceled()) {
        return;
    }

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::updateGroundAltitudeEstimate(): Updating ground altitude estimate");

    double ground_altitude = (*vehicle_odometry_adapter_history_)[0].position()[2];

    ground_altitudes_history_->Store(ground_altitude);

    std::vector<double> ground_altitudes = ground_altitudes_history_->vector();

    double ground_altitude_estimate = std::accumulate(ground_altitudes.begin(), ground_altitudes.end(), 0.0) / ground_altitudes.size();

    ground_altitude_estimate_->Store(ground_altitude_estimate);

    if (!vehicle_global_position_adapter_history_->empty()) {

        float altitude_amsl = (*vehicle_global_position_adapter_history_)[0].altitude();

        if (altitude_amsl != 0 && altitude_amsl != NAN) {

            float altitude_local = (*vehicle_odometry_adapter_history_)[0].position()[2];

            float diff = altitude_amsl - altitude_local;

            ground_altitude_estimate_amsl_->Store(ground_altitude_estimate + diff);

        } else {

            ground_altitude_estimate_amsl_->Store(NAN);

        }

    } else {

        ground_altitude_estimate_amsl_->Store(NAN);

    }

    ground_altitude_update_timer_->reset();

    geometry_msgs::msg::TransformStamped ground_tf;

    ground_tf.header.stamp = node_->now();
    ground_tf.header.frame_id = configuration_->GetParameter("/tf/world_frame_id").as_string();
    ground_tf.child_frame_id = configuration_->GetParameter("/tf/ground_frame_id").as_string();

    ground_tf.transform = transformMsgFromTransform(
        vector_t(0, 0, ground_altitude_estimate),
        quaternion_t(1, 0, 0, 0)
    );

    tf_broadcaster_->sendTransform(ground_tf);

}

void CombinedDroneAwarenessHandler::updateDroneLocation(CombinedDroneAwarenessAdapter & adapter) {

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::updateDroneLocation(): Updating drone location");

    if (vehicle_odometry_adapter_history_->empty() || vehicle_status_adapter_history_->empty()) {
        adapter.drone_location() = DRONE_LOCATION_UNKNOWN;
        adapter.on_cable_id() = -1;
        return;
    }

    const bool use_gripper_status_condition =
        configuration_->GetParameter("/control/maneuver_controller/use_gripper_status_condition").as_bool();
    const bool has_gripper_status =
        gripper_status_adapter_history_ && !gripper_status_adapter_history_->empty();
    const bool gripper_closed =
        use_gripper_status_condition && has_gripper_status && !adapter.gripper_open();
    const auto mark_on_cable = [this, &adapter](int cable_id) {
        adapter.drone_location() = DRONE_LOCATION_ON_CABLE;
        adapter.on_cable_id() = cable_id;
        has_found_initial_location_ = true;
    };
    const auto active_target_cable_id = [&adapter]() {
        if (
            adapter.has_target()
            && adapter.target_adapter().target_type() == TARGET_TYPE_CABLE
        ) {
            return adapter.target_adapter().target_id();
        }
        return -1;
    };

    iii_drone::adapters::px4::VehicleOdometryAdapter vehicle_odometry_adapter = (*vehicle_odometry_adapter_history_)[0];

    point_t drone_position = vehicle_odometry_adapter.position();

    // Check if on ground. Armed vehicles must remain flight-capable from the
    // maneuver layer even when operating close to the ground.
    if (
        !adapter.armed()
        && drone_position[2] - ground_altitude_estimate_->Load() < configuration_->GetParameter("/control/maneuver_controller/landed_altitude_threshold").as_double()
    ) {
        adapter.drone_location() = DRONE_LOCATION_ON_GROUND;
        adapter.on_cable_id() = -1;
        has_found_initial_location_ = true;
        return;
    }

    if (gripper_closed) {
        mark_on_cable(active_target_cable_id());
        return;
    }

    // Check if on cable:
    if (!powerline_adapter_history_->empty()) {

        bool could_transform = false;

        geometry_msgs::msg::TransformStamped transform_stamped;
        try{
            transform_stamped = tf_buffer_->lookupTransform(
                configuration_->GetParameter("/tf/drone_frame_id").as_string(),
                configuration_->GetParameter("/tf/cable_gripper_frame_id").as_string(),
                tf2::TimePointZero
            );

            could_transform = true;

        } catch (tf2::TransformException & e) {
            RCLCPP_WARN(node_->get_logger(), "CombinedDroneAwarenessHandler::updateDroneLocation(): Could not transform drone to gripper frame: %s", e.what());
            could_transform = false;
        }

        if(could_transform) {

            vector_t v_drone_to_gripper = vectorFromTransformMsg(transform_stamped.transform);

            point_t gripper_position = drone_position + v_drone_to_gripper;

            iii_drone::adapters::PowerlineAdapter powerline_adapter = (*powerline_adapter_history_)[0];

            iii_drone::adapters::SingleLineAdapter closest_line;

            bool found_closest_line = false;

            try {
                closest_line = powerline_adapter.GetClosestLine(gripper_position);
                found_closest_line = true;
            } catch (std::exception & e) {
                found_closest_line = false;
            }

            if (found_closest_line) {

                point_t closest_line_position = closest_line.position();

                geometry_msgs::msg::PointStamped closest_line_position_stamped;
                closest_line_position_stamped.header.frame_id = closest_line.frame_id();
                closest_line_position_stamped.point = pointMsgFromPoint(closest_line_position);

                could_transform = false;

                try {
                    closest_line_position_stamped = tf_buffer_->transform(
                        closest_line_position_stamped,
                        configuration_->GetParameter("/tf/drone_frame_id").as_string()
                    );
                    could_transform = true;
                } catch (tf2::TransformException & e) {
                    RCLCPP_WARN(node_->get_logger(), "CombinedDroneAwarenessHandler::updateDroneLocation(): Could not transform closest line position to drone frame: %s", e.what());
                    could_transform = false;
                }

                if (could_transform) {

                    closest_line_position = pointFromPointMsg(closest_line_position_stamped.point);

                    float closest_line_distance = (closest_line_position - v_drone_to_gripper).norm();

                    if (closest_line_distance <= configuration_->GetParameter("/control/maneuver_controller/on_cable_max_euc_distance").as_double()) {
                        
                        mark_on_cable(closest_line.id());

                        return;

                    }
                }
            }
        }
    }

    // Check if in flight:
    if (adapter.armed()) {
        adapter.drone_location() = DRONE_LOCATION_IN_FLIGHT;
        has_found_initial_location_ = true;
        return;

    } else if (!has_found_initial_location_) {
        // Drone is unarmed and initial location has not been found and drone is not on cable,
        // assume drone is on ground

        adapter.drone_location() = DRONE_LOCATION_ON_GROUND;
        adapter.on_cable_id() = -1;
        has_found_initial_location_ = true;
        return;

    }

    // Assume drone is on ground:
    adapter.drone_location() = DRONE_LOCATION_ON_GROUND;
    adapter.on_cable_id() = -1;
    has_found_initial_location_ = false;

    // // Throw error if no location found
    // std::string error_message = "CombinedDroneAwarenessHandler::updateDroneLocation(): Could not determine drone location.";

    // if (configuration_->GetParameter("/control/maneuver_controller/fail_on_unable_to_locate").as_bool()) {
    //     RCLCPP_FATAL(node_->get_logger(), error_message.c_str());
    //     throw std::runtime_error(error_message);
    // } else {
    //     RCLCPP_ERROR(node_->get_logger(), error_message.c_str());
    //     adapter.drone_location() = DRONE_LOCATION_UNKNOWN;
    //     adapter.on_cable_id() = -1;
    // }

}

void CombinedDroneAwarenessHandler::publishMembers() {

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::publishMembers(): Publishing members");

    iii_drone::adapters::TargetAdapter target_adapter = target_adapter_->Load();

    if (target_adapter.target_type() != iii_drone::adapters::TARGET_TYPE_CABLE) {
        return;
    }

    iii_drone_interfaces::msg::Target target_msg = target_adapter.ToMsg();

    target_pub_->publish(target_msg);

    try {
        pose_t target_pose = GetPoseOfTarget(target_adapter);

        geometry_msgs::msg::PoseStamped target_pose_stamped;
        target_pose_stamped.header.stamp = node_->now();
        target_pose_stamped.header.frame_id = configuration_->GetParameter("/tf/world_frame_id").as_string();
        target_pose_stamped.pose = poseMsgFromPose(target_pose);

        target_pose_pub_->publish(target_pose_stamped);

    } catch (std::runtime_error & e) {
        
    }

    try {
        transform_matrix_t target_world_to_drone = ComputeTargetTransform(target_adapter);

        pose_t target_pose_world_to_drone = poseFromTransformMatrix(target_world_to_drone);

        geometry_msgs::msg::PoseStamped target_pose_world_to_drone_stamped;
        target_pose_world_to_drone_stamped.header.stamp = node_->now();
        target_pose_world_to_drone_stamped.header.frame_id = configuration_->GetParameter("/tf/world_frame_id").as_string();
        target_pose_world_to_drone_stamped.pose = poseMsgFromPose(target_pose_world_to_drone);

        target_drone_pose_pub_->publish(target_pose_world_to_drone_stamped);

    } catch (std::runtime_error & e) {
        
    }

}
