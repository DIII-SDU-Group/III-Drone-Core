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

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    combined_drone_awareness_handler_ = std::make_shared<CombinedDroneAwarenessHandler>(
        configurator_.combined_drone_awareness_handler_parameters(),
        tf_buffer_,
        this
    );

    std::vector<int64_t> px4_offboard_mode_ids = configurator_.px4_offboard_mode_ids();

    for (const auto & id : px4_offboard_mode_ids) {
        combined_drone_awareness_handler_->RegisterOffboardMode(id);
    }

    trajectory_generator_client_ = std::make_shared<TrajectoryGeneratorClient>(this);

    maneuver_execution_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive
    );

    maneuver_scheduler_ = std::make_unique<ManeuverScheduler>(
        this,
        combined_drone_awareness_handler_,
        configurator_.maneuver_scheduler_parameters(),
        maneuver_execution_cb_group_
    );

    registerManeuverServers();
    
}

ManeuverControllerNode::~ManeuverControllerNode() {
    RCLCPP_INFO(get_logger(), "Shutting down maneuver controller node");
}

void ManeuverControllerNode::registerManeuverServers() {
    
    hover_maneuver_server_ = std::make_shared<HoverManeuverServer>(
        this,
        combined_drone_awareness_handler_,
        "hover",
        configurator_.maneuver_wait_for_execute_poll_ms(),
        configurator_.maneuver_evaluate_done_poll_ms(),
        configurator_.use_nans_when_hovering()
    );

    maneuver_scheduler_->RegisterManeuverServer(
        MANEUVER_TYPE_HOVER,
        std::static_pointer_cast<ManeuverServer>(hover_maneuver_server_)
    );

    hover_by_object_maneuver_server_ = std::make_shared<HoverByObjectManeuverServer>(
        this,
        combined_drone_awareness_handler_,
        "hover_by_object",
        configurator_.maneuver_wait_for_execute_poll_ms(),
        configurator_.maneuver_evaluate_done_poll_ms(),
        configurator_.use_nans_when_hovering(),
        configurator_.hover_by_object_max_euc_dist()
    );

    maneuver_scheduler_->RegisterManeuverServer(
        MANEUVER_TYPE_HOVER_BY_OBJECT,
        std::static_pointer_cast<ManeuverServer>(hover_by_object_maneuver_server_)
    );

    hover_on_cable_maneuver_server_ = std::make_shared<HoverOnCableManeuverServer>(
        this,
        combined_drone_awareness_handler_,
        "hover_on_cable",
        configurator_.maneuver_wait_for_execute_poll_ms(),
        configurator_.maneuver_evaluate_done_poll_ms(),
        configurator_.hover_on_cable_default_z_velocity(),
        configurator_.hover_on_cable_default_yaw_rate()
    );

    maneuver_scheduler_->RegisterManeuverServer(
        MANEUVER_TYPE_HOVER_ON_CABLE,
        std::static_pointer_cast<ManeuverServer>(hover_on_cable_maneuver_server_)
    );

    fly_to_position_maneuver_server_ = std::make_shared<FlyToPositionManeuverServer>(
        this,
        combined_drone_awareness_handler_,
        "fly_to_position",
        configurator_.maneuver_wait_for_execute_poll_ms(),
        configurator_.maneuver_evaluate_done_poll_ms(),
        configurator_.fly_to_position_maneuver_server_parameters(),
        trajectory_generator_client_
    );

    maneuver_scheduler_->RegisterManeuverServer(
        MANEUVER_TYPE_FLY_TO_POSITION,
        std::static_pointer_cast<ManeuverServer>(fly_to_position_maneuver_server_)
    );

    fly_to_object_maneuver_server_ = std::make_shared<FlyToObjectManeuverServer>(
        this,
        combined_drone_awareness_handler_,
        "fly_to_object",
        configurator_.maneuver_wait_for_execute_poll_ms(),
        configurator_.maneuver_evaluate_done_poll_ms(),
        configurator_.fly_to_object_maneuver_server_parameters(),
        trajectory_generator_client_
    );

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