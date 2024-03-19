#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/msg/state.hpp>
#include <iii_drone_interfaces/msg/reference.hpp>
#include <iii_drone_interfaces/msg/reference_trajectory.hpp>
              
#include <iii_drone_interfaces/srv/compute_reference_trajectory.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/configuration/configurator.hpp>

#include <iii_drone_core/control/trajectory_generator.hpp>

#include <iii_drone_core/adapters/state_adapter.hpp>
#include <iii_drone_core/adapters/reference_adapter.hpp>
#include <iii_drone_core/adapters/reference_trajectory_adapter.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace control {
namespace trajectory_generator_node {

    class TrajectoryGeneratorNode : public rclcpp::Node {

    public:

        /**
         * Constructor.
         * 
         * @param node_name The name of the node.
         * @param node_namespace The namespace of the node.
         * @param options The options for the node.
         */
        TrajectoryGeneratorNode(
            const std::string node_name="trajectory_generator",
            const std::string node_namespace="/control/trajectory_generator",
            const rclcpp::NodeOptions & options = rclcpp::NodeOptions()
        );

        /**
         * Destructor.
         */
        ~TrajectoryGeneratorNode();

    private:
		/**
		 * @brief The configurator
		 */
		iii_drone::configuration::Configurator configurator_;

        /**
         * @brief The trajectory generator object
         */
        TrajectoryGenerator trajectory_generator_;

		/**
		 * @brief The compute reference trajectory service
         */
		rclcpp::Service<iii_drone_interfaces::srv::ComputeReferenceTrajectory>::SharedPtr compute_reference_trajectory_service_;

		/**
		 * @brief The compute reference trajectory service callback
		 * 
		 * @param request The request
		 * @param response The response
		 * 
		 * @return void
		 */
		void computeReferenceTrajectoryCallback(
			const std::shared_ptr<iii_drone_interfaces::srv::ComputeReferenceTrajectory::Request> request,
			std::shared_ptr<iii_drone_interfaces::srv::ComputeReferenceTrajectory::Response> response
		);

    };

} // namespace trajectory_generator_node
} // namespace control
} // namespace iii_drone

/*****************************************************************************/
// Main
/*****************************************************************************/

int main(int argc, char * argv[]);