#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <vector>
#include <string>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/configuration/configurator.hpp>

#include <iii_drone_core/control/combined_drone_awareness_handler.hpp>

#include <iii_drone_core/control/maneuver/maneuver_scheduler.hpp>
#include <iii_drone_core/control/maneuver/maneuver_types.hpp>

#include <iii_drone_core/control/maneuver/maneuver_server.hpp>
#include <iii_drone_core/control/maneuver/hover_maneuver_server.hpp>
#include <iii_drone_core/control/maneuver/hover_by_object_maneuver_server.hpp>
#include <iii_drone_core/control/maneuver/hover_on_cable_maneuver_server.hpp>
#include <iii_drone_core/control/maneuver/fly_to_position_maneuver_server.hpp>
#include <iii_drone_core/control/maneuver/fly_to_object_maneuver_server.hpp>
#include <iii_drone_core/control/maneuver/cable_landing_maneuver_server.hpp>
#include <iii_drone_core/control/maneuver/cable_takeoff_maneuver_server.hpp>

#include <iii_drone_core/control/trajectory_generator_client.hpp>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {

namespace control {

namespace maneuver_controller_node {

    /**
     * @brief The ManeuverControllerNode class. Responsible for execution of maneuvers.
     * Uses the CombinedDroneAwarenessHandler to get the state of the drone and the environment.
     * Uses the ManeuverScheduler to schedule and execute maneuvers.
     * Creates all maneuver servers and registers them with the ManeuverScheduler.
     * Exposes action servers contained in the registered maneuver servers.
     * Exposes a topic for publishing the reference, contained in the ManeuverScheduler.
     * 
     * When creating new maneuvers, take the following steps:
     * 1. Create a new action type in the iii_drone_interfaces package.
     * 2. Create a new maneuver type in the maneuver_types.hpp file.
     * 3. If necessary, create a maneuver parameters struct in the maneuver_types.hpp file.
     * 4. Create a new maneuver server class in the maneuver directory, inheriting from ManeuverServer,
     * implement the virtual methods.
     * 5. Create a new maneuver server in the registerManeuverServers method of this class
     * and register it with the ManeuverScheduler.
     */
    class ManeuverControllerNode : public rclcpp::Node {

    public:
        /**
         * @brief Constructor. Creates the node, initializes the configurator, the transform buffer and listener,
         * the combined drone awareness handler, the maneuver scheduler.
         * Creates and registers all maneuver servers with the maneuver scheduler.
         * 
         * @param node_name The name of the node.
         * @param node_namespace The namespace of the node.
         * @param options The node options.
         */
        ManeuverControllerNode(
            const std::string & node_name="maneuver_controller",
            const std::string & node_namespace="/control/maneuver_controller",
            const rclcpp::NodeOptions & options=rclcpp::NodeOptions()
        );

        /**
         * @brief Destructor.
         */
        ~ManeuverControllerNode();

    private:
        /**
         * @brief Configurator for handling configuration parameters of the node.
         * Exposes parameters for the maneuver scheduler and the maneuver servers.
         */
        iii_drone::configuration::Configurator configurator_;

        /**
         * @brief The transform buffer. Shared with the CombinedDroneAwarenessHandler.
         */
        tf2_ros::Buffer::SharedPtr tf_buffer_;

        /**
         * @brief The transform listener for listening to transforms and updating the transform buffer.
         */
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        /**
         * @brief The combined drone awareness handler shared pointer. 
         * Shared with the maneuver servers and the maneuver scheduler.
         */
        iii_drone::control::CombinedDroneAwarenessHandler::SharedPtr combined_drone_awareness_handler_;

        /**
         * @brief The trajectory generator client shared pointer.
         */
        iii_drone::control::TrajectoryGeneratorClient::SharedPtr trajectory_generator_client_;

        /**
         * @brief The maneuver execution callback group. 
         * Enables multi processing of the execution of maneuvers (scheduling and computing references).
         */
        rclcpp::CallbackGroup::SharedPtr maneuver_execution_cb_group_;

        /**
         * @brief The maneuver scheduler shared pointer. 
         * Responsible for scheduling and executing maneuvers.
         */
        iii_drone::control::maneuver::ManeuverScheduler::UniquePtr maneuver_scheduler_;

        // Maneuver servers:
        /**
         * @brief The hover maneuver server shared pointer. 
         * Will serve hover action requests as well as generate hover references in between maneuvers 
         * for the maneuver scheduler to use.
         */
        std::shared_ptr<iii_drone::control::maneuver::HoverManeuverServer> hover_maneuver_server_;

        /**
         * @brief The hover by object maneuver server shared pointer. 
         * Will serve hover by object action requests as well as generate hover by object references in between maneuvers 
         * for the maneuver scheduler to use.
         */
        std::shared_ptr<iii_drone::control::maneuver::HoverByObjectManeuverServer> hover_by_object_maneuver_server_;

        /**
         * @brief The hover on cable maneuver server shared pointer. 
         * Will serve hover on cable action requests as well as generate hover on cable references in between maneuvers 
         * for the maneuver scheduler to use.
         */
        std::shared_ptr<iii_drone::control::maneuver::HoverOnCableManeuverServer> hover_on_cable_maneuver_server_;

        /**
         * @brief The fly to position maneuver server shared pointer. 
         * Will serve fly to position action requests.
         */
        std::shared_ptr<iii_drone::control::maneuver::FlyToPositionManeuverServer> fly_to_position_maneuver_server_;

        /**
         * @brief The fly to object maneuver server shared pointer. 
         * Will serve fly to object action requests.
         */
        std::shared_ptr<iii_drone::control::maneuver::FlyToObjectManeuverServer> fly_to_object_maneuver_server_;

        /**
         * @brief The cable landing maneuver server shared pointer.
         * Will serve cable landing action requests.
         */
        std::shared_ptr<iii_drone::control::maneuver::CableLandingManeuverServer> cable_landing_maneuver_server_;

        /**
         * @brief The cable takeoff maneuver server shared pointer.
         * Will serve cable takeoff action requests.
         */
        std::shared_ptr<iii_drone::control::maneuver::CableTakeoffManeuverServer> cable_takeoff_maneuver_server_;

        /**
         * @brief Creates and registers all maneuver servers with the maneuver scheduler.
         */
        void registerManeuverServers();

    };


} // namespace maneuver_controller_node
} // namespace control
} // namespace iii_drone

/*****************************************************************************/
// Main:
/*****************************************************************************/

int main(int argc, char * argv[]);