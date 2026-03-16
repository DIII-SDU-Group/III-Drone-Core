/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "iii_drone_core/control/maneuver_controller_node/maneuver_controller_node.hpp"

using namespace iii_drone::control::maneuver_controller_node;
using namespace iii_drone::control::maneuver;
using namespace iii_drone::control;
using namespace iii_drone::types;
using namespace iii_drone::math;

namespace {

using LifecycleConfigurator = iii_drone::configuration::Configurator<rclcpp_lifecycle::LifecycleNode>;
using ParameterType = rclcpp::ParameterType;
using ConfigurationEntry = iii_drone::configuration::configuration_entry_t;

void DeclareManagedParameters(LifecycleConfigurator & configurator)
{
    const auto bool_t = ParameterType::PARAMETER_BOOL;
    const auto int_t = ParameterType::PARAMETER_INTEGER;
    const auto double_t = ParameterType::PARAMETER_DOUBLE;
    const auto string_t = ParameterType::PARAMETER_STRING;

    configurator.DeclareParameter("/control/maneuver_controller/ground_estimate_window_size", int_t);
    configurator.DeclareParameter("/control/maneuver_controller/ground_estimate_update_period_ms", int_t);
    configurator.DeclareParameter("/control/maneuver_controller/ground_estimate_initial_delay_s", double_t);
    configurator.DeclareParameter("/control/maneuver_controller/landed_altitude_threshold", double_t);
    configurator.DeclareParameter("/control/maneuver_controller/landed_altitude_threshold_on_start", double_t);
    configurator.DeclareParameter("/control/maneuver_controller/on_cable_max_euc_distance", double_t);
    configurator.DeclareParameter("/control/maneuver_controller/fail_on_unable_to_locate", bool_t);
    configurator.DeclareParameter("/control/maneuver_controller/combined_drone_awareness_pub_period_ms", int_t);
    configurator.DeclareParameter("/control/maneuver_controller/maneuver_publish_period_ms", int_t);
    configurator.DeclareParameter("/control/maneuver_controller/maneuver_register_update_timeout_s", double_t);
    configurator.DeclareParameter("/control/maneuver_controller/maneuver_start_timeout_s", double_t);
    configurator.DeclareParameter("/control/maneuver_controller/maneuver_queue_size", int_t);
    configurator.DeclareParameter("/control/maneuver_controller/maneuver_execution_period_ms", int_t);
    configurator.DeclareParameter("/control/maneuver_controller/reference_callback_provider_publish_period_ms", int_t);
    configurator.DeclareParameter("/control/maneuver_controller/maneuver_completion_token_acquisition_timeout_s", double_t);
    configurator.DeclareParameter("/control/maneuver_controller/use_nans_when_hovering", bool_t);
    configurator.DeclareParameter("/control/maneuver_controller/no_maneuver_idle_cnt_s", double_t);
    configurator.DeclareParameter("/control/maneuver_controller/hover_by_object_max_euc_dist", double_t);
    configurator.DeclareParameter("/control/maneuver_controller/hover_on_cable_default_z_velocity", double_t);
    configurator.DeclareParameter("/control/maneuver_controller/hover_on_cable_default_yaw_rate", double_t);
    configurator.DeclareParameter("/control/maneuver_controller/maneuver_wait_for_execute_poll_ms", int_t);
    configurator.DeclareParameter("/control/maneuver_controller/maneuver_evaluate_done_poll_ms", int_t);
    configurator.DeclareParameter("/control/maneuver_controller/reached_position_euclidean_distance_threshold", double_t);
    configurator.DeclareParameter("/control/maneuver_controller/minimum_target_altitude", double_t);
    configurator.DeclareParameter("/control/maneuver_controller/cable_landing_min_z_distance", double_t);
    configurator.DeclareParameter("/control/maneuver_controller/cable_landing_max_z_distance", double_t);
    configurator.DeclareParameter("/control/maneuver_controller/cable_landing_max_initial_distance_error", double_t);
    configurator.DeclareParameter("/control/maneuver_controller/cable_landing_max_initial_yaw_error", double_t);
    configurator.DeclareParameter("/control/maneuver_controller/cable_landing_safety_zone_radius", double_t);
    configurator.DeclareParameter("/control/maneuver_controller/cable_landing_safety_margin_max_xy_position_error", double_t);
    configurator.DeclareParameter("/control/maneuver_controller/cable_landing_safety_margin_max_xy_velocity", double_t);
    configurator.DeclareParameter("/control/maneuver_controller/cable_landing_safety_margin_max_yaw_error", double_t);
    configurator.DeclareParameter("/control/maneuver_controller/cable_landing_safety_margin_max_yaw_rate", double_t);
    configurator.DeclareParameter("/control/maneuver_controller/cable_landing_safety_margin_max_negative_vertical_distance", double_t);
    configurator.DeclareParameter("/control/maneuver_controller/cable_landing_safety_margin_cone_slope", double_t);
    configurator.DeclareParameter("/control/maneuver_controller/cable_landing_safety_margin_cone_tolerance", double_t);
    configurator.DeclareParameter("/control/maneuver_controller/cable_landing_reference_truncate_radius", double_t);
    configurator.DeclareParameter("/control/maneuver_controller/cable_landing_target_upwards_velocity", double_t);
    configurator.DeclareParameter("/control/maneuver_controller/cable_landing_reached_position_euclidean_distance_threshold", double_t);
    configurator.DeclareParameter("/control/maneuver_controller/use_gripper_status_condition", bool_t);
    configurator.DeclareParameter("/control/maneuver_controller/cable_takeoff_min_target_cable_distance", double_t);
    configurator.DeclareParameter("/control/maneuver_controller/cable_takeoff_max_target_cable_distance", double_t);
    configurator.DeclareParameter("/control/maneuver_controller/generate_trajectories_asynchronously_with_delay", bool_t);
    configurator.DeclareParameter("/control/maneuver_controller/generate_trajectories_poll_period_ms", int_t);
    configurator.DeclareParameter("/control/maneuver_controller/generate_trajectories_timeout_ms", int_t);
    configurator.DeclareParameter("/control/maneuver_controller/fly_to_position_use_mpc", bool_t);
    configurator.DeclareParameter("/control/maneuver_controller/fly_to_object_use_mpc", bool_t);
    configurator.DeclareParameter("/control/maneuver_controller/cable_landing_use_mpc", bool_t);
    configurator.DeclareParameter("/control/maneuver_controller/cable_takeoff_use_mpc", bool_t);
    configurator.DeclareParameter("/tf/cable_gripper_frame_id", string_t);
    configurator.DeclareParameter("/tf/drone_frame_id", string_t);
    configurator.DeclareParameter("/tf/world_frame_id", string_t);
    configurator.DeclareParameter("/tf/ground_frame_id", string_t);

    configurator.CreateConfiguration("combined_drone_awareness_handler", {
        ConfigurationEntry("/control/maneuver_controller/ground_estimate_window_size", int_t),
        ConfigurationEntry("/control/maneuver_controller/ground_estimate_update_period_ms", int_t),
        ConfigurationEntry("/control/maneuver_controller/ground_estimate_initial_delay_s", double_t),
        ConfigurationEntry("/control/maneuver_controller/landed_altitude_threshold", double_t),
        ConfigurationEntry("/control/maneuver_controller/landed_altitude_threshold_on_start", double_t),
        ConfigurationEntry("/control/maneuver_controller/on_cable_max_euc_distance", double_t),
        ConfigurationEntry("/control/maneuver_controller/fail_on_unable_to_locate", bool_t),
        ConfigurationEntry("/control/maneuver_controller/combined_drone_awareness_pub_period_ms", int_t),
        ConfigurationEntry("/control/maneuver_controller/use_gripper_status_condition", bool_t),
        ConfigurationEntry("/tf/cable_gripper_frame_id", string_t),
        ConfigurationEntry("/tf/drone_frame_id", string_t),
        ConfigurationEntry("/tf/world_frame_id", string_t),
        ConfigurationEntry("/tf/ground_frame_id", string_t),
    });
    configurator.CreateConfiguration("maneuver_scheduler", {
        ConfigurationEntry("/control/maneuver_controller/maneuver_publish_period_ms", int_t),
        ConfigurationEntry("/control/maneuver_controller/maneuver_register_update_timeout_s", double_t),
        ConfigurationEntry("/control/maneuver_controller/maneuver_start_timeout_s", double_t),
        ConfigurationEntry("/control/maneuver_controller/maneuver_queue_size", int_t),
        ConfigurationEntry("/control/maneuver_controller/maneuver_execution_period_ms", int_t),
        ConfigurationEntry("/control/maneuver_controller/reference_callback_provider_publish_period_ms", int_t),
        ConfigurationEntry("/control/maneuver_controller/maneuver_completion_token_acquisition_timeout_s", double_t),
        ConfigurationEntry("/control/maneuver_controller/use_nans_when_hovering", bool_t),
        ConfigurationEntry("/control/maneuver_controller/no_maneuver_idle_cnt_s", double_t),
        ConfigurationEntry("/control/maneuver_controller/hover_by_object_max_euc_dist", double_t),
        ConfigurationEntry("/control/maneuver_controller/hover_on_cable_default_z_velocity", double_t),
        ConfigurationEntry("/control/maneuver_controller/hover_on_cable_default_yaw_rate", double_t),
    });
    configurator.CreateConfiguration("trajectory_generator_client", {
        ConfigurationEntry("/control/maneuver_controller/generate_trajectories_asynchronously_with_delay", bool_t),
        ConfigurationEntry("/control/maneuver_controller/generate_trajectories_poll_period_ms", int_t),
        ConfigurationEntry("/control/maneuver_controller/generate_trajectories_timeout_ms", int_t),
    });
    configurator.CreateConfiguration("hover_on_cable_maneuver_server", {
        ConfigurationEntry("/tf/cable_gripper_frame_id", string_t),
    });
    configurator.CreateConfiguration("fly_to_position_maneuver_server", {
        ConfigurationEntry("/control/maneuver_controller/reached_position_euclidean_distance_threshold", double_t),
        ConfigurationEntry("/control/maneuver_controller/minimum_target_altitude", double_t),
        ConfigurationEntry("/control/maneuver_controller/fly_to_position_use_mpc", bool_t),
        ConfigurationEntry("/tf/world_frame_id", string_t),
    });
    configurator.CreateConfiguration("fly_to_object_maneuver_server", {
        ConfigurationEntry("/control/maneuver_controller/reached_position_euclidean_distance_threshold", double_t),
        ConfigurationEntry("/control/maneuver_controller/minimum_target_altitude", double_t),
        ConfigurationEntry("/control/maneuver_controller/fly_to_object_use_mpc", bool_t),
        ConfigurationEntry("/tf/world_frame_id", string_t),
    });
    configurator.CreateConfiguration("cable_landing_maneuver_server", {
        ConfigurationEntry("/control/maneuver_controller/cable_landing_target_upwards_velocity", double_t),
        ConfigurationEntry("/control/maneuver_controller/cable_landing_min_z_distance", double_t),
        ConfigurationEntry("/control/maneuver_controller/cable_landing_max_z_distance", double_t),
        ConfigurationEntry("/control/maneuver_controller/cable_landing_max_initial_distance_error", double_t),
        ConfigurationEntry("/control/maneuver_controller/cable_landing_max_initial_yaw_error", double_t),
        ConfigurationEntry("/tf/cable_gripper_frame_id", string_t),
        ConfigurationEntry("/tf/drone_frame_id", string_t),
        ConfigurationEntry("/tf/world_frame_id", string_t),
        ConfigurationEntry("/control/maneuver_controller/hover_on_cable_default_z_velocity", double_t),
        ConfigurationEntry("/control/maneuver_controller/hover_on_cable_default_yaw_rate", double_t),
        ConfigurationEntry("/control/maneuver_controller/cable_landing_safety_zone_radius", double_t),
        ConfigurationEntry("/control/maneuver_controller/cable_landing_safety_margin_max_xy_position_error", double_t),
        ConfigurationEntry("/control/maneuver_controller/cable_landing_safety_margin_max_xy_velocity", double_t),
        ConfigurationEntry("/control/maneuver_controller/cable_landing_safety_margin_max_yaw_error", double_t),
        ConfigurationEntry("/control/maneuver_controller/cable_landing_safety_margin_max_yaw_rate", double_t),
        ConfigurationEntry("/control/maneuver_controller/cable_landing_safety_margin_max_negative_vertical_distance", double_t),
        ConfigurationEntry("/control/maneuver_controller/cable_landing_safety_margin_cone_slope", double_t),
        ConfigurationEntry("/control/maneuver_controller/cable_landing_safety_margin_cone_tolerance", double_t),
        ConfigurationEntry("/control/maneuver_controller/cable_landing_reference_truncate_radius", double_t),
        ConfigurationEntry("/control/maneuver_controller/cable_landing_reached_position_euclidean_distance_threshold", double_t),
        ConfigurationEntry("/control/maneuver_controller/use_gripper_status_condition", bool_t),
        ConfigurationEntry("/control/maneuver_controller/cable_landing_use_mpc", bool_t),
    });
    configurator.CreateConfiguration("cable_takeoff_maneuver_server", {
        ConfigurationEntry("/control/maneuver_controller/cable_takeoff_min_target_cable_distance", double_t),
        ConfigurationEntry("/control/maneuver_controller/cable_takeoff_max_target_cable_distance", double_t),
        ConfigurationEntry("/control/maneuver_controller/reached_position_euclidean_distance_threshold", double_t),
        ConfigurationEntry("/tf/drone_frame_id", string_t),
        ConfigurationEntry("/tf/world_frame_id", string_t),
        ConfigurationEntry("/tf/cable_gripper_frame_id", string_t),
        ConfigurationEntry("/control/maneuver_controller/cable_takeoff_use_mpc", bool_t),
    });
}

}  // namespace

/*****************************************************************************/
// Implementation
/*****************************************************************************/

ManeuverControllerNode::ManeuverControllerNode(
    const std::string & node_name,
    const std::string & node_namespace,
    const rclcpp::NodeOptions & options
) : rclcpp_lifecycle::LifecycleNode(
    node_name, 
    node_namespace, 
    options
) {
    auto set_logger_level = [this](int severity) {
        const rcutils_ret_t ret = rcutils_logging_set_logger_level(this->get_logger().get_name(), severity);
        if (ret != RCUTILS_RET_OK) {
            RCLCPP_WARN(this->get_logger(), "Failed to set logger level, rcutils_ret_t=%d", static_cast<int>(ret));
        }
    };

	std::string log_level = std::getenv("MANEUVER_CONTROLLER_LOG_LEVEL");

	if (log_level != "") {

		// Convert to upper case:
		std::transform(
			log_level.begin(), 
			log_level.end(), 
			log_level.begin(), 
			[](unsigned char c){ return std::toupper(c); }
		);

		if (log_level == "DEBUG") {
			set_logger_level(RCUTILS_LOG_SEVERITY_DEBUG);
		} else if (log_level == "INFO") {
			set_logger_level(RCUTILS_LOG_SEVERITY_INFO);
		} else if (log_level == "WARN") {
			set_logger_level(RCUTILS_LOG_SEVERITY_WARN);
		} else if (log_level == "ERROR") {
			set_logger_level(RCUTILS_LOG_SEVERITY_ERROR);
		} else if (log_level == "FATAL") {
			set_logger_level(RCUTILS_LOG_SEVERITY_FATAL);
		}

	}

    trajectory_generator_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive
    );

    maneuver_execution_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive
    );

    RCLCPP_INFO(get_logger(), "ManeuverControllerNode::ManeuverControllerNode(): Maneuver controller ready");
    
}

ManeuverControllerNode::~ManeuverControllerNode() {

    RCLCPP_INFO(get_logger(), "Shutting down maneuver controller node");

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
ManeuverControllerNode::on_configure(const rclcpp_lifecycle::State & state) {

    RCLCPP_INFO(get_logger(), "ManeuverControllerNode::on_configure()");

    CallbackReturn parent_return = rclcpp_lifecycle::LifecycleNode::on_configure(state);

    if (parent_return != CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(
            this->get_logger(), 
            "ManeuverControllerNode::on_configure(): Failed to configure parent class."
        );
        return parent_return;
    }

    RCLCPP_DEBUG(
        get_logger(),
        "ManeuverControllerNode::on_configure(): Initializing configurator"
    );

    configurator_ = std::make_shared<iii_drone::configuration::Configurator<rclcpp_lifecycle::LifecycleNode>>(
        this,
        "maneuver_controller"
    );
    DeclareManagedParameters(*configurator_);
    configurator_->validate();

    // tf
    RCLCPP_DEBUG(
        get_logger(),
        "ManeuverControllerNode::on_configure(): Initializing tf"
    );

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_DEBUG(get_logger(), "ManeuverControllerNode::on_configure(): Initializing combined drone awareness handler");

    combined_drone_awareness_handler_ = std::make_shared<CombinedDroneAwarenessHandler>(
        configurator_->GetConfiguration("combined_drone_awareness_handler"),
        tf_buffer_,
        this
    );

    RCLCPP_DEBUG(get_logger(), "ManeuverControllerNode::on_configure(): Initializing trajectory generator client");

    trajectory_generator_client_ = std::make_shared<TrajectoryGeneratorClient>(
        this,
        configurator_->GetConfiguration("trajectory_generator_client"),
        trajectory_generator_cb_group_
    );

    RCLCPP_DEBUG(get_logger(), "ManeuverControllerNode::on_configure(): Initializing maneuver scheduler");

    maneuver_scheduler_ = std::make_unique<ManeuverScheduler>(
        this,
        combined_drone_awareness_handler_,
        configurator_->GetConfiguration("maneuver_scheduler"),
        maneuver_execution_cb_group_
    );


    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ManeuverControllerNode::on_cleanup(const rclcpp_lifecycle::State & state) {

    RCLCPP_INFO(get_logger(), "ManeuverControllerNode::on_cleanup()");

    CallbackReturn parent_return = rclcpp_lifecycle::LifecycleNode::on_cleanup(state);

    if (parent_return != CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(
            this->get_logger(), 
            "ManeuverControllerNode::on_cleanup(): Failed to cleanup parent class."
        );
        return parent_return;
    }

    RCLCPP_DEBUG(
        get_logger(),
        "ManeuverControllerNode::on_cleanup(): Cleaning up maneuver scheduler"
    );

    maneuver_scheduler_.reset();
    maneuver_scheduler_ = nullptr;

    RCLCPP_DEBUG(
        get_logger(),
        "ManeuverControllerNode::on_cleanup(): Cleaning up trajectory generator client"
    );

    trajectory_generator_client_.reset();
    trajectory_generator_client_ = nullptr;

    RCLCPP_DEBUG(
        get_logger(),
        "ManeuverControllerNode::on_cleanup(): Cleaning up combined drone awareness handler"
    );

    combined_drone_awareness_handler_.reset();
    combined_drone_awareness_handler_ = nullptr;

    RCLCPP_DEBUG(
        get_logger(),
        "ManeuverControllerNode::on_cleanup(): Cleaning up tf"
    );

    tf_listener_.reset();
    tf_listener_ = nullptr;

    tf_buffer_.reset();
    tf_buffer_ = nullptr;

    RCLCPP_DEBUG(
        get_logger(),
        "ManeuverControllerNode::on_cleanup(): Cleaning up configurator"
    );

    configurator_.reset();
    configurator_ = nullptr;

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ManeuverControllerNode::on_activate(const rclcpp_lifecycle::State & state) {

    RCLCPP_INFO(get_logger(), "ManeuverControllerNode::on_activate()");

    CallbackReturn parent_return = rclcpp_lifecycle::LifecycleNode::on_activate(state);

    if (parent_return != CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(
            this->get_logger(), 
            "ManeuverControllerNode::on_activate(): Failed to activate parent class."
        );
        return parent_return;
    }

    RCLCPP_DEBUG(
        get_logger(),
        "ManeuverControllerNode::on_activate(): Starting combined drone awareness handler"
    );

    combined_drone_awareness_handler_->Start();

    RCLCPP_DEBUG(
        get_logger(),
        "ManeuverControllerNode::on_activate(): Starting maneuver scheduler"
    );

    maneuver_scheduler_->Start();

    RCLCPP_DEBUG(get_logger(), "ManeuverControllerNode::on_activate(): Registering maneuver servers");

    registerManeuverServers();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ManeuverControllerNode::on_deactivate(const rclcpp_lifecycle::State & state) {

    RCLCPP_INFO(get_logger(), "ManeuverControllerNode::on_deactivate()");

    CallbackReturn parent_return = rclcpp_lifecycle::LifecycleNode::on_deactivate(state);

    if (parent_return != CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(
            this->get_logger(), 
            "ManeuverControllerNode::on_deactivate(): Failed to deactivate parent class."
        );
        return parent_return;
    }


    RCLCPP_DEBUG(get_logger(), "ManeuverControllerNode::on_deactivate(): Unregistering maneuver servers");

    unregisterManeuverServers();

    RCLCPP_DEBUG(
        get_logger(),
        "ManeuverControllerNode::on_deactivate(): Stopping maneuver scheduler"
    );

    maneuver_scheduler_->Stop();

    RCLCPP_DEBUG(
        get_logger(),
        "ManeuverControllerNode::on_deactivate(): Stopping combined drone awareness handler"
    );

    combined_drone_awareness_handler_->Stop();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ManeuverControllerNode::on_shutdown(const rclcpp_lifecycle::State & state) {

    RCLCPP_INFO(get_logger(), "ManeuverControllerNode::on_shutdown()");

    CallbackReturn parent_return = rclcpp_lifecycle::LifecycleNode::on_shutdown(state);

    if (parent_return != CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(
            this->get_logger(), 
            "ManeuverControllerNode::on_shutdown(): Failed to shutdown parent class."
        );
        return parent_return;
    }

    // Create and start thread detached which sleeps for 1 second, then shuts down rclcpp
    std::thread shutdown_thread([this](){
        std::this_thread::sleep_for(std::chrono::seconds(1));
        rclcpp::shutdown();
    });
    shutdown_thread.detach();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ManeuverControllerNode::on_error(const rclcpp_lifecycle::State & state) {
    (void)state;

    RCLCPP_FATAL(this->get_logger(), "ManeuverControllerNode::on_error(): An error occured");

    throw std::runtime_error("An error occured");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;

}

void ManeuverControllerNode::registerManeuverServers() {

    RCLCPP_DEBUG(get_logger(), "ManeuverControllerNode::registerManeuverServers(): Creating hover maneuver server");
    
    hover_maneuver_server_ = std::make_shared<HoverManeuverServer>(
        this,
        combined_drone_awareness_handler_,
        "hover",
        configurator_->GetParameter("/control/maneuver_controller/maneuver_wait_for_execute_poll_ms").as_int(),
        configurator_->GetParameter("/control/maneuver_controller/maneuver_evaluate_done_poll_ms").as_int(),
        configurator_->GetParameter("/control/maneuver_controller/use_nans_when_hovering").as_bool()
    );

    RCLCPP_DEBUG(get_logger(), "ManeuverControllerNode::registerManeuverServers(): Registering hover maneuver server");

    maneuver_scheduler_->RegisterManeuverServer(
        MANEUVER_TYPE_HOVER,
        std::dynamic_pointer_cast<ManeuverServer>(hover_maneuver_server_)
    );

    RCLCPP_DEBUG(get_logger(), "ManeuverControllerNode::registerManeuverServers(): Creating hover by object maneuver server");

    hover_by_object_maneuver_server_ = std::make_shared<HoverByObjectManeuverServer>(
        this,
        combined_drone_awareness_handler_,
        "hover_by_object",
        configurator_->GetParameter("/control/maneuver_controller/maneuver_wait_for_execute_poll_ms").as_int(),
        configurator_->GetParameter("/control/maneuver_controller/maneuver_evaluate_done_poll_ms").as_int(),
        configurator_->GetParameter("/control/maneuver_controller/use_nans_when_hovering").as_bool(),
        configurator_->GetParameter("/control/maneuver_controller/hover_by_object_max_euc_dist").as_double()
    );

    RCLCPP_DEBUG(get_logger(), "ManeuverControllerNode::registerManeuverServers(): Registering hover by object maneuver server");

    maneuver_scheduler_->RegisterManeuverServer(
        MANEUVER_TYPE_HOVER_BY_OBJECT,
        std::dynamic_pointer_cast<ManeuverServer>(hover_by_object_maneuver_server_)
    );

    RCLCPP_DEBUG(get_logger(), "ManeuverControllerNode::registerManeuverServers(): Creating hover on cable maneuver server");

    hover_on_cable_maneuver_server_ = std::make_shared<HoverOnCableManeuverServer>(
        this,
        combined_drone_awareness_handler_,
        "hover_on_cable",
        configurator_->GetParameter("/control/maneuver_controller/maneuver_wait_for_execute_poll_ms").as_int(),
        configurator_->GetParameter("/control/maneuver_controller/maneuver_evaluate_done_poll_ms").as_int(),
        configurator_->GetConfiguration("hover_on_cable_maneuver_server")
    );

    RCLCPP_DEBUG(get_logger(), "ManeuverControllerNode::registerManeuverServers(): Registering hover on cable maneuver server");

    maneuver_scheduler_->RegisterManeuverServer(
        MANEUVER_TYPE_HOVER_ON_CABLE,
        std::dynamic_pointer_cast<ManeuverServer>(hover_on_cable_maneuver_server_)
    );

    RCLCPP_DEBUG(get_logger(), "ManeuverControllerNode::registerManeuverServers(): Creating fly to position maneuver server");

    fly_to_position_maneuver_server_ = std::make_shared<FlyToPositionManeuverServer>(
        this,
        combined_drone_awareness_handler_,
        "fly_to_position",
        configurator_->GetParameter("/control/maneuver_controller/maneuver_wait_for_execute_poll_ms").as_int(),
        configurator_->GetParameter("/control/maneuver_controller/maneuver_evaluate_done_poll_ms").as_int(),
        configurator_->GetConfiguration("fly_to_position_maneuver_server"),
        trajectory_generator_client_
    );

    RCLCPP_DEBUG(get_logger(), "ManeuverControllerNode::registerManeuverServers(): Registering fly to position maneuver server");

    maneuver_scheduler_->RegisterManeuverServer(
        MANEUVER_TYPE_FLY_TO_POSITION,
        std::dynamic_pointer_cast<ManeuverServer>(fly_to_position_maneuver_server_)
    );

    RCLCPP_DEBUG(get_logger(), "ManeuverControllerNode::registerManeuverServers(): Creating fly to object maneuver server");

    fly_to_object_maneuver_server_ = std::make_shared<FlyToObjectManeuverServer>(
        this,
        combined_drone_awareness_handler_,
        "fly_to_object",
        configurator_->GetParameter("/control/maneuver_controller/maneuver_wait_for_execute_poll_ms").as_int(),
        configurator_->GetParameter("/control/maneuver_controller/maneuver_evaluate_done_poll_ms").as_int(),
        configurator_->GetConfiguration("fly_to_object_maneuver_server"),
        trajectory_generator_client_
    );

    RCLCPP_DEBUG(get_logger(), "ManeuverControllerNode::registerManeuverServers(): Registering fly to object maneuver server");

    maneuver_scheduler_->RegisterManeuverServer(
        MANEUVER_TYPE_FLY_TO_OBJECT,
        std::dynamic_pointer_cast<ManeuverServer>(fly_to_object_maneuver_server_)
    );

    RCLCPP_DEBUG(get_logger(), "ManeuverControllerNode::registerManeuverServers(): Creating cable landing maneuver server");

    cable_landing_maneuver_server_ = std::make_shared<CableLandingManeuverServer>(
        this,
        combined_drone_awareness_handler_,
        "cable_landing",
        configurator_->GetParameter("/control/maneuver_controller/maneuver_wait_for_execute_poll_ms").as_int(),
        configurator_->GetParameter("/control/maneuver_controller/maneuver_evaluate_done_poll_ms").as_int(),
        configurator_->GetConfiguration("cable_landing_maneuver_server"),
        trajectory_generator_client_
    );

    RCLCPP_DEBUG(get_logger(), "ManeuverControllerNode::registerManeuverServers(): Registering cable landing maneuver server");

    maneuver_scheduler_->RegisterManeuverServer(
        MANEUVER_TYPE_CABLE_LANDING,
        std::dynamic_pointer_cast<ManeuverServer>(cable_landing_maneuver_server_)
    );

    RCLCPP_DEBUG(get_logger(), "ManeuverControllerNode::registerManeuverServers(): Creating cable takeoff maneuver server");

    cable_takeoff_maneuver_server_ = std::make_shared<CableTakeoffManeuverServer>(
        this,
        combined_drone_awareness_handler_,
        "cable_takeoff",
        configurator_->GetParameter("/control/maneuver_controller/maneuver_wait_for_execute_poll_ms").as_int(),
        configurator_->GetParameter("/control/maneuver_controller/maneuver_evaluate_done_poll_ms").as_int(),
        configurator_->GetConfiguration("cable_takeoff_maneuver_server"),
        trajectory_generator_client_
    );

    RCLCPP_DEBUG(get_logger(), "ManeuverControllerNode::registerManeuverServers(): Registering cable takeoff maneuver server");

    maneuver_scheduler_->RegisterManeuverServer(
        MANEUVER_TYPE_CABLE_TAKEOFF,
        std::dynamic_pointer_cast<ManeuverServer>(cable_takeoff_maneuver_server_)
    );

    RCLCPP_INFO(get_logger(), "ManeuverControllerNode::registerManeuverServers(): Finished registering maneuver servers");

}

void ManeuverControllerNode::unregisterManeuverServers() {

    RCLCPP_DEBUG(get_logger(), "ManeuverControllerNode::unregisterManeuverServers(): Unregistering maneuver servers");

    maneuver_scheduler_->UnregisterManeuverServer(MANEUVER_TYPE_HOVER);
    maneuver_scheduler_->UnregisterManeuverServer(MANEUVER_TYPE_HOVER_BY_OBJECT);
    maneuver_scheduler_->UnregisterManeuverServer(MANEUVER_TYPE_HOVER_ON_CABLE);
    maneuver_scheduler_->UnregisterManeuverServer(MANEUVER_TYPE_FLY_TO_POSITION);
    maneuver_scheduler_->UnregisterManeuverServer(MANEUVER_TYPE_FLY_TO_OBJECT);
    maneuver_scheduler_->UnregisterManeuverServer(MANEUVER_TYPE_CABLE_LANDING);
    maneuver_scheduler_->UnregisterManeuverServer(MANEUVER_TYPE_CABLE_TAKEOFF);

    hover_maneuver_server_.reset();
    hover_maneuver_server_ = nullptr;

    hover_by_object_maneuver_server_.reset();
    hover_by_object_maneuver_server_ = nullptr;

    hover_on_cable_maneuver_server_.reset();
    hover_on_cable_maneuver_server_ = nullptr;

    fly_to_position_maneuver_server_.reset();
    fly_to_position_maneuver_server_ = nullptr;

    fly_to_object_maneuver_server_.reset();
    fly_to_object_maneuver_server_ = nullptr;

    cable_landing_maneuver_server_.reset();
    cable_landing_maneuver_server_ = nullptr;

    cable_takeoff_maneuver_server_.reset();
    cable_takeoff_maneuver_server_ = nullptr;

    RCLCPP_DEBUG(get_logger(), "ManeuverControllerNode::unregisterManeuverServers(): Finished unregistering maneuver servers");

}

int main(int argc, char * argv[]) {
    
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;

    auto node = std::make_shared<ManeuverControllerNode>();
    // auto trajectory_generator_node = std::make_shared<iii_drone::control::trajectory_generator_node::TrajectoryGeneratorNode>();

    executor.add_node(node->get_node_base_interface());
    // executor.add_node(trajectory_generator_node->get_node_base_interface());

    try {
        
        executor.spin();

    } catch(const std::exception& e) {
        
        node.reset();

    }
    
	if (rclcpp::ok()) {
		node.reset();
		rclcpp::shutdown();
	}

    return 0;


}
