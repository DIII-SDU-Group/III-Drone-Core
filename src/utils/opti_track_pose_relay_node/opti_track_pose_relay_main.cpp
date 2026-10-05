/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "iii_drone_core/utils/opti_track_pose_relay_node/opti_track_pose_relay_node.hpp"

#include <cstdio>
#include <cstdlib>
#include <exception>

/*****************************************************************************/
// Main
/*****************************************************************************/

int main(int argc, char * argv[]) {

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    int exit_code = EXIT_SUCCESS;

    try {

        auto node = std::make_shared<iii_drone::utils::opti_track_pose_relay_node::OptiTrackPoseRelayNode>();

        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();

        if (node->lab_side_failed()) {
            exit_code = EXIT_FAILURE;
        }

    } catch (const std::exception & error) {

        RCLCPP_FATAL(rclcpp::get_logger("opti_track_pose_relay"), "Pose relay failed: %s", error.what());
        exit_code = EXIT_FAILURE;

    }

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }

    return exit_code;

}
