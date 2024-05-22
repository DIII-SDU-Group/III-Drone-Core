/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/trajectory_generator_node/trajectory_generator_node.hpp>

using namespace iii_drone::control::trajectory_generator_node;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

TrajectoryGeneratorNode::TrajectoryGeneratorNode(
    const std::string node_name,
    const std::string node_namespace,
    const rclcpp::NodeOptions & options
) : Node(node_name, node_namespace, options),
    configurator_(
        this
    ), 
    trajectory_generator_(
        configurator_.GetParameterBundle("position_MPC"),
        configurator_.GetParameterBundle("cable_landing_MPC"),
        configurator_.GetParameterBundle("cable_takeoff_MPC")
    )
{

    RCLCPP_INFO(this->get_logger(), "TrajectoryGeneratorNode::TrajectoryGeneratorNode(): Initializing.");

    compute_reference_trajectory_service_ = this->create_service<iii_drone_interfaces::srv::ComputeReferenceTrajectory>(
        "compute_reference_trajectory",
        std::bind(
            &TrajectoryGeneratorNode::computeReferenceTrajectoryCallback, 
            this, 
            std::placeholders::_1, 
            std::placeholders::_2
        )
    );

}

TrajectoryGeneratorNode::~TrajectoryGeneratorNode() {}

void TrajectoryGeneratorNode::computeReferenceTrajectoryCallback(
    const std::shared_ptr<iii_drone_interfaces::srv::ComputeReferenceTrajectory::Request> request,
    std::shared_ptr<iii_drone_interfaces::srv::ComputeReferenceTrajectory::Response> response
) {

    // RCLCPP_DEBUG(this->get_logger(), "TrajectoryGeneratorNode::computeReferenceTrajectoryCallback(): Computing reference trajectory.");

    adapters::StateAdapter state_adapter(request->state);
    adapters::ReferenceAdapter reference_adapter(request->reference);

    // RCLCPP_DEBUG(this->get_logger(), "TrajectoryGeneratorNode::computeReferenceTrajectoryCallback(): State: [%f, %f, %f, %f, %f, %f, %f]", state_adapter.state().position().x(), state_adapter.state().position().y(), state_adapter.state().position().z(), state_adapter.state().velocity().x(), state_adapter.state().velocity().y(), state_adapter.state().velocity().z(), state_adapter.state().yaw());
    // RCLCPP_DEBUG(this->get_logger(), "TrajectoryGeneratorNode::computeReferenceTrajectoryCallback(): Reference: [%f, %f, %f, %f, %f, %f, %f]", reference_adapter.reference().position().x(), reference_adapter.reference().position().y(), reference_adapter.reference().position().z(), reference_adapter.reference().velocity().x(), reference_adapter.reference().velocity().y(), reference_adapter.reference().velocity().z(), reference_adapter.reference().yaw());

    // Compute the reference trajectory
    ReferenceTrajectory ref_traj = trajectory_generator_.ComputeReferenceTrajectory(
        state_adapter.state(),
        reference_adapter.reference(),
        request->set_reference,
        request->reset,
        (MPC_mode_t)request->mpc_mode.mode
    );

    // RCLCPP_DEBUG(this->get_logger(), "TrajectoryGeneratorNode::computeReferenceTrajectoryCallback(): Reference trajectory computed:");

    // for (size_t i = 0; i < ref_traj.references().size(); i++) {
    //     // RCLCPP_DEBUG(this->get_logger(), "TrajectoryGeneratorNode::computeReferenceTrajectoryCallback(): Reference %d: [%f, %f, %f, %f, %f, %f, %f]", i, ref_traj.references()[i].position().x(), ref_traj.references()[i].position().y(), ref_traj.references()[i].position().z(), ref_traj.references()[i].velocity().x(), ref_traj.references()[i].velocity().y(), ref_traj.references()[i].velocity().z(), ref_traj.references()[i].yaw());
    // }

    // Set the response
    iii_drone_interfaces::msg::ReferenceTrajectory msg = adapters::ReferenceTrajectoryAdapter(ref_traj).ToMsg();

    // RCLCPP_DEBUG(this->get_logger(), "TrajectoryGeneratorNode::computeReferenceTrajectoryCallback(): Reference trajectory msg:");

    // for (size_t i = 0; i < msg.references.size(); i++) {
    //     // RCLCPP_DEBUG(this->get_logger(), "TrajectoryGeneratorNode::computeReferenceTrajectoryCallback(): Reference %d: [%f, %f, %f, %f, %f, %f, %f]", i, msg.references[i].position.x, msg.references[i].position.y, msg.references[i].position.z, msg.references[i].velocity.x, msg.references[i].velocity.y, msg.references[i].velocity.z, msg.references[i].yaw);
    // }

    response->reference_trajectory = msg;

    // RCLCPP_DEBUG(this->get_logger(), "TrajectoryGeneratorNode::computeReferenceTrajectoryCallback(): Reference trajectory computed.");

}

int main(int argc, char * argv[]) {

    rclcpp::init(argc, argv);

    auto node = std::make_shared<TrajectoryGeneratorNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;

}