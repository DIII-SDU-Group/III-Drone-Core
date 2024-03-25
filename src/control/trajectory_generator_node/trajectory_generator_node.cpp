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

    adapters::StateAdapter state_adapter(request->state);
    adapters::ReferenceAdapter reference_adapter(request->reference);

    // Compute the reference trajectory
    ReferenceTrajectory ref_traj = trajectory_generator_.ComputeReferenceTrajectory(
        state_adapter.state(),
        reference_adapter.reference(),
        request->set_reference,
        request->reset,
        (MPC_mode_t)request->mpc_mode.mode
    );

    // Set the response
    response->reference_trajectory = adapters::ReferenceTrajectoryAdapter(ref_traj).ToMsg();

}

int main(int argc, char * argv[]) {

    rclcpp::init(argc, argv);

    auto node = std::make_shared<TrajectoryGeneratorNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;

}