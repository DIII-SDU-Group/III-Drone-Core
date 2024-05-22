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
) : Node(
    node_name, 
    node_namespace, 
    options
),  configurator_(this) {

    RCLCPP_INFO(get_logger(), "ManeuverControllerNode::ManeuverControllerNode()");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(get_logger(), "ManeuverControllerNode::ManeuverControllerNode(): Initializing combined drone awareness handler");

    combined_drone_awareness_handler_ = std::make_shared<CombinedDroneAwarenessHandler>(
        configurator_.GetParameterBundle("combined_drone_awareness_handler"),
        tf_buffer_,
        this
    );

    RCLCPP_INFO(get_logger(), "ManeuverControllerNode::ManeuverControllerNode(): Initializing trajectory generator client");

    trajectory_generator_client_ = std::make_shared<TrajectoryGeneratorClient>(
        this,
        configurator_.GetParameterBundle("trajectory_generator_client")
    );

    RCLCPP_INFO(get_logger(), "ManeuverControllerNode::ManeuverControllerNode(): Initializing maneuver scheduler");

    maneuver_execution_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive
    );

    maneuver_scheduler_ = std::make_unique<ManeuverScheduler>(
        this,
        combined_drone_awareness_handler_,
        configurator_.GetParameterBundle("maneuver_scheduler"),
        maneuver_execution_cb_group_
    );

    RCLCPP_INFO(get_logger(), "ManeuverControllerNode::ManeuverControllerNode(): Registering maneuver servers");

    registerManeuverServers();

    RCLCPP_INFO(get_logger(), "ManeuverControllerNode::ManeuverControllerNode(): Finished initializing maneuver controller node");
    
}

ManeuverControllerNode::~ManeuverControllerNode() {
    RCLCPP_INFO(get_logger(), "Shutting down maneuver controller node");
}

void ManeuverControllerNode::registerManeuverServers() {

    RCLCPP_DEBUG(get_logger(), "ManeuverControllerNode::registerManeuverServers(): Creating hover maneuver server");
    
    hover_maneuver_server_ = std::make_shared<HoverManeuverServer>(
        this,
        combined_drone_awareness_handler_,
        "hover",
        configurator_.GetParameter("maneuver_wait_for_execute_poll_ms").as_int(),
        configurator_.GetParameter("maneuver_evaluate_done_poll_ms").as_int(),
        configurator_.GetParameter("use_nans_when_hovering").as_bool()
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
        configurator_.GetParameter("maneuver_wait_for_execute_poll_ms").as_int(),
        configurator_.GetParameter("maneuver_evaluate_done_poll_ms").as_int(),
        configurator_.GetParameter("use_nans_when_hovering").as_bool(),
        configurator_.GetParameter("hover_by_object_max_euc_dist").as_double()
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
        configurator_.GetParameter("maneuver_wait_for_execute_poll_ms").as_int(),
        configurator_.GetParameter("maneuver_evaluate_done_poll_ms").as_int(),
        configurator_.GetParameterBundle("hover_on_cable_maneuver_server")
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
        configurator_.GetParameter("maneuver_wait_for_execute_poll_ms").as_int(),
        configurator_.GetParameter("maneuver_evaluate_done_poll_ms").as_int(),
        configurator_.GetParameterBundle("fly_to_position_maneuver_server"),
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
        configurator_.GetParameter("maneuver_wait_for_execute_poll_ms").as_int(),
        configurator_.GetParameter("maneuver_evaluate_done_poll_ms").as_int(),
        configurator_.GetParameterBundle("fly_to_object_maneuver_server"),
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
        configurator_.GetParameter("maneuver_wait_for_execute_poll_ms").as_int(),
        configurator_.GetParameter("maneuver_evaluate_done_poll_ms").as_int(),
        configurator_.GetParameterBundle("cable_landing_maneuver_server"),
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
        configurator_.GetParameter("maneuver_wait_for_execute_poll_ms").as_int(),
        configurator_.GetParameter("maneuver_evaluate_done_poll_ms").as_int(),
        configurator_.GetParameterBundle("cable_takeoff_maneuver_server"),
        trajectory_generator_client_
    );

    RCLCPP_DEBUG(get_logger(), "ManeuverControllerNode::registerManeuverServers(): Registering cable takeoff maneuver server");

    maneuver_scheduler_->RegisterManeuverServer(
        MANEUVER_TYPE_CABLE_TAKEOFF,
        std::dynamic_pointer_cast<ManeuverServer>(cable_takeoff_maneuver_server_)
    );

    RCLCPP_INFO(get_logger(), "ManeuverControllerNode::registerManeuverServers(): Finished registering maneuver servers");

}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ManeuverControllerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}