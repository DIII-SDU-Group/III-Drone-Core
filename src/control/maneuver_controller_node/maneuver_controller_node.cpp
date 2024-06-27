/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "iii_drone_core/control/maneuver_controller_node/maneuver_controller_node.hpp"

using namespace iii_drone::control::maneuver_controller_node;
using namespace iii_drone::control::maneuver;
using namespace iii_drone::control;
using namespace iii_drone::types;
using namespace iii_drone::math;

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
			rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
		} else if (log_level == "INFO") {
			rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);
		} else if (log_level == "WARN") {
			rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_WARN);
		} else if (log_level == "ERROR") {
			rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_ERROR);
		} else if (log_level == "FATAL") {
			rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_FATAL);
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
        this
    );

    // tf
    RCLCPP_DEBUG(
        get_logger(),
        "ManeuverControllerNode::on_configure(): Initializing tf"
    );

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_DEBUG(get_logger(), "ManeuverControllerNode::on_configure(): Initializing combined drone awareness handler");

    combined_drone_awareness_handler_ = std::make_shared<CombinedDroneAwarenessHandler>(
        configurator_->GetParameterBundle("combined_drone_awareness_handler"),
        tf_buffer_,
        this
    );

    RCLCPP_DEBUG(get_logger(), "ManeuverControllerNode::on_configure(): Initializing trajectory generator client");

    trajectory_generator_client_ = std::make_shared<TrajectoryGeneratorClient>(
        this,
        configurator_->GetParameterBundle("trajectory_generator_client"),
        trajectory_generator_cb_group_
    );

    RCLCPP_DEBUG(get_logger(), "ManeuverControllerNode::on_configure(): Initializing maneuver scheduler");

    maneuver_scheduler_ = std::make_unique<ManeuverScheduler>(
        this,
        combined_drone_awareness_handler_,
        configurator_->GetParameterBundle("maneuver_scheduler"),
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
        configurator_->GetParameter("maneuver_wait_for_execute_poll_ms").as_int(),
        configurator_->GetParameter("maneuver_evaluate_done_poll_ms").as_int(),
        configurator_->GetParameter("use_nans_when_hovering").as_bool()
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
        configurator_->GetParameter("maneuver_wait_for_execute_poll_ms").as_int(),
        configurator_->GetParameter("maneuver_evaluate_done_poll_ms").as_int(),
        configurator_->GetParameter("use_nans_when_hovering").as_bool(),
        configurator_->GetParameter("hover_by_object_max_euc_dist").as_double()
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
        configurator_->GetParameter("maneuver_wait_for_execute_poll_ms").as_int(),
        configurator_->GetParameter("maneuver_evaluate_done_poll_ms").as_int(),
        configurator_->GetParameterBundle("hover_on_cable_maneuver_server")
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
        configurator_->GetParameter("maneuver_wait_for_execute_poll_ms").as_int(),
        configurator_->GetParameter("maneuver_evaluate_done_poll_ms").as_int(),
        configurator_->GetParameterBundle("fly_to_position_maneuver_server"),
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
        configurator_->GetParameter("maneuver_wait_for_execute_poll_ms").as_int(),
        configurator_->GetParameter("maneuver_evaluate_done_poll_ms").as_int(),
        configurator_->GetParameterBundle("fly_to_object_maneuver_server"),
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
        configurator_->GetParameter("maneuver_wait_for_execute_poll_ms").as_int(),
        configurator_->GetParameter("maneuver_evaluate_done_poll_ms").as_int(),
        configurator_->GetParameterBundle("cable_landing_maneuver_server"),
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
        configurator_->GetParameter("maneuver_wait_for_execute_poll_ms").as_int(),
        configurator_->GetParameter("maneuver_evaluate_done_poll_ms").as_int(),
        configurator_->GetParameterBundle("cable_takeoff_maneuver_server"),
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