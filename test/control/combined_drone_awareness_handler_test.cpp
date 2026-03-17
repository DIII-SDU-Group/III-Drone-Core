/**
 * @file combined_drone_awareness_handler_test.cpp
 * @brief CombinedDroneAwarenessHandler test. This tests relies on the simulation environment to be running.
 * The test assume the drone is in flight and in position mode.
*/

#include <iii_drone_configuration/configurator.hpp>

#include <iii_drone_core/control/combined_drone_awareness_handler.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

#include <iostream>
#include <memory>
#include <thread>

using namespace iii_drone::control;
using namespace iii_drone::configuration;

rclcpp::Node::SharedPtr node;
CombinedDroneAwarenessHandler::SharedPtr cdah;
rclcpp::TimerBase::SharedPtr timer;

bool test_offboard_modes = false;
bool test_target = false;
bool test_awareness = false;
bool test_ground_altitude_estimate = false;


bool testOffboardModes(bool &is_done) {

    is_done = false;
    bool success = true;

    static int cnt = 0;

    switch(cnt) {
        case 0:

            RCLCPP_INFO(node->get_logger(), "Testing offboard modes");
            break;

        case 1:

            RCLCPP_INFO(node->get_logger(), "CombinedDroneAwarenessHandler.armed(): %s", cdah->armed() ? "true" : "false");
            RCLCPP_INFO(node->get_logger(), "CombinedDroneAwarenessHandler.offboard(): %s", cdah->offboard() ? "true" : "false");

            RCLCPP_INFO(node->get_logger(), "Registering id 2 (position) as offboard");

            // cdah->RegisterOffboardMode(2);
            break;

        case 2: {

            RCLCPP_INFO(node->get_logger(), "Getting offboard");
            bool offboard = cdah->offboard();

            RCLCPP_INFO(node->get_logger(), "CombinedDroneAwarenessHandler.offboard(): %s", offboard ? "true" : "false");
            RCLCPP_INFO(node->get_logger(), "CombinedDroneAwarenessHandler.armed(): %s", cdah->armed() ? "true" : "false");

            if (!offboard) {
                RCLCPP_ERROR(node->get_logger(), "Offboard mode not detected");
                success = false;
            }
            break;
        }

        case 3:
            RCLCPP_INFO(node->get_logger(), "Offboard mode test passed");
            break;

        default:
            is_done = true;
            break;
    }

    cnt++;

    return success;

}

bool testTarget(bool &is_done) {

    static int cnt = 0;

    is_done = false;
    bool success = true;

    switch(cnt) {
        case 0:
            RCLCPP_INFO(node->get_logger(), "Testing target");
            break;
        case 1: {

            bool has_target = cdah->has_target();
            bool target_position_known = cdah->target_position_known();
            iii_drone::adapters::TargetAdapter target_adapter = cdah->target_adapter();

            RCLCPP_INFO(node->get_logger(), "CombinedDroneAwarenessHandler.has_target(): %s", has_target ? "true" : "false");
            RCLCPP_INFO(node->get_logger(), "CombinedDroneAwarenessHandler.target_position_known(): %s", target_position_known ? "true" : "false");
            RCLCPP_INFO(node->get_logger(), "CombinedDroneAwarenessHandler.target_adapter().target_type(): %d", target_adapter.target_type());
            break;
        }
        case 2: {

            iii_drone::types::transform_matrix_t tf_mat;
            tf_mat << 1, 0, 0, 0,
                    0, 1, 0, 0,
                    0, 0, 1, 0,
                    0, 0, 0, 1;

            iii_drone::adapters::TargetAdapter new_target_adapter(
                iii_drone::adapters::TARGET_TYPE_CABLE,
                2,
                "frame_id",
                tf_mat
            );
            
            RCLCPP_INFO(node->get_logger(), "Setting target");

            cdah->SetTarget(new_target_adapter);
            break;
        }
        case 3: {

            bool has_target = cdah->has_target();
            bool target_position_known = cdah->target_position_known();
            iii_drone::adapters::TargetAdapter target_adapter = cdah->target_adapter();

            success = has_target && target_position_known && target_adapter.target_type() == iii_drone::adapters::TARGET_TYPE_CABLE;

            RCLCPP_INFO(node->get_logger(), "CombinedDroneAwarenessHandler.has_target(): %s", has_target ? "true" : "false");
            RCLCPP_INFO(node->get_logger(), "CombinedDroneAwarenessHandler.target_position_known(): %s", target_position_known ? "true" : "false");
            RCLCPP_INFO(node->get_logger(), "CombinedDroneAwarenessHandler.target_adapter().target_type(): %d", target_adapter.target_type());
            RCLCPP_INFO(node->get_logger(), "CombinedDroneAwarenessHandler.target_adapter().target_id(): %d", target_adapter.target_id());
            RCLCPP_INFO(node->get_logger(), "CombinedDroneAwarenessHandler.target_adapter().reference_frame_id(): %s", target_adapter.reference_frame_id().c_str());

            if(!success) {
                RCLCPP_ERROR(node->get_logger(), "Target not set correctly");
            }

            break;

        }

        case 4: {

            RCLCPP_INFO(node->get_logger(), "Clearing target");

            cdah->ClearTarget();
            break;
        }

        case 5: {

            bool has_target = cdah->has_target();
            bool target_position_known = cdah->target_position_known();
            iii_drone::adapters::TargetAdapter target_adapter = cdah->target_adapter();

            RCLCPP_INFO(node->get_logger(), "CombinedDroneAwarenessHandler.has_target(): %s", has_target ? "true" : "false");
            RCLCPP_INFO(node->get_logger(), "CombinedDroneAwarenessHandler.target_position_known(): %s", target_position_known ? "true" : "false");
            RCLCPP_INFO(node->get_logger(), "CombinedDroneAwarenessHandler.target_adapter().target_type(): %d", target_adapter.target_type());

            success = success && !has_target && !target_position_known && target_adapter.target_type() == iii_drone::adapters::TARGET_TYPE_NONE;

            if (!success) {
                RCLCPP_ERROR(node->get_logger(), "Target not set or cleared correctly");
            }

            break;
        }

        case 6:
            RCLCPP_INFO(node->get_logger(), "Target test passed");
            break;
        default:
            is_done = true;
            break;

    }

    cnt++;

    return success;

}

bool testAwareness(bool &is_done) {

    static int cnt = 0;

    is_done = false;
    bool success = true;

    switch(cnt) {

        case 0:
            RCLCPP_INFO(node->get_logger(), "Testing awareness");
            break;

        case 1: {
            RCLCPP_INFO(node->get_logger(), "Getting combined_drone_awareness");
            combined_drone_awareness_t awareness = cdah->combined_drone_awareness();

            RCLCPP_INFO(node->get_logger(), "CombinedDroneAwarenessHandler.combined_drone_awareness().state.position(): %f, %f, %f",
                awareness.state.position()[0],
                awareness.state.position()[1],
                awareness.state.position()[2]
            );
            RCLCPP_INFO(node->get_logger(), "CombinedDroneAwarenessHandler.combined_drone_awareness().armed: %s", awareness.armed ? "true" : "false");
            RCLCPP_INFO(node->get_logger(), "CombinedDroneAwarenessHandler.combined_drone_awareness().offboard: %s", awareness.offboard ? "true" : "false");
            RCLCPP_INFO(node->get_logger(), "CombinedDroneAwarenessHandler.combined_drone_awareness().target_position_known: %s", awareness.target_position_known ? "true" : "false");
            RCLCPP_INFO(node->get_logger(), "CombinedDroneAwarenessHandler.combined_drone_awareness().drone_location: %d", awareness.drone_location);
            RCLCPP_INFO(node->get_logger(), "CombinedDroneAwarenessHandler.combined_drone_awareness().gripper_open: %s", awareness.gripper_open ? "true" : "false");
            RCLCPP_INFO(node->get_logger(), "CombinedDroneAwarenessHandler.combined_drone_awareness().has_target(): %s", awareness.has_target() ? "true" : "false");
            RCLCPP_INFO(node->get_logger(), "CombinedDroneAwarenessHandler.combined_drone_awareness().on_ground(): %s", awareness.on_ground() ? "true" : "false");
            RCLCPP_INFO(node->get_logger(), "CombinedDroneAwarenessHandler.combined_drone_awareness().on_cable(): %s", awareness.on_cable() ? "true" : "false");
            RCLCPP_INFO(node->get_logger(), "CombinedDroneAwarenessHandler.combined_drone_awareness().in_flight(): %s", awareness.in_flight() ? "true" : "false");

            break;

        }

    default:
        RCLCPP_INFO(node->get_logger(), "Awareness test passed");
        is_done = true;
        break;

    }

    cnt++;

    return success;
}

bool testGroundAltitudeEstimate(bool &is_done) {

    static int cnt = 0;

    is_done = false;
    bool success = true;

    double previous_ground_altitude_estimate = 0;
    static bool has_changed = false;

    switch(cnt) {

        case 0:
            RCLCPP_INFO(node->get_logger(), "Testing ground altitude estimate");
            RCLCPP_INFO(node->get_logger(), "Outputting ground altitude estimate 10 times. Fly the ground to the ground.");
            break;

        case 1: 
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
        case 9:
        case 10: {
            double ground_altitude_estimate = cdah->ground_altitude_estimate();
            RCLCPP_INFO(node->get_logger(), "CombinedDroneAwarenessHandler.ground_altitude_estimate(): %f", ground_altitude_estimate);

            if (cnt > 1) {
                if (ground_altitude_estimate != previous_ground_altitude_estimate) {
                    has_changed = true;
                }
            }

            break;

        }

    case 11:
        if (!has_changed) {
            RCLCPP_ERROR(node->get_logger(), "Ground altitude estimate did not change");
            success = false;
        }
        break;

    default:
        RCLCPP_INFO(node->get_logger(), "Ground altitude estimate test passed");
        is_done = true;
        break;

    }

    cnt++;

    return success;

}

void test() {
    static bool offboard_done = false;
    static bool target_done = false;
    static bool awareness_done = false;
    static bool ground_altitude_estimate_done = false;

    if (test_offboard_modes && !offboard_done) {
        bool success = testOffboardModes(offboard_done);
        if (!success) {
            RCLCPP_ERROR(node->get_logger(), "Offboard mode test failed");
            timer->cancel();
        }
    } else if (test_target && !target_done) {
        bool success = testTarget(target_done);
        if (!success) {
            RCLCPP_ERROR(node->get_logger(), "Target test failed");
            timer->cancel();
        }
    } else if (test_awareness && !awareness_done) {
        bool success = testAwareness(awareness_done);
        if (!success) {
            RCLCPP_ERROR(node->get_logger(), "Awareness test failed");
            timer->cancel();
        }
    } else if (test_ground_altitude_estimate && !ground_altitude_estimate_done) {
        bool success = testGroundAltitudeEstimate(ground_altitude_estimate_done);
        if (!success) {
            RCLCPP_ERROR(node->get_logger(), "Ground altitude estimate test failed");
            timer->cancel();
        }
    } else {
        timer->cancel();
    }

}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "offboard_modes") {
            test_offboard_modes = true;
        } else if (arg == "target") {
            test_target = true;
        } else if (arg == "awareness") {
            test_awareness = true;
        } else if (arg == "ground_altitude_estimate") {
            test_ground_altitude_estimate = true;
        } else if (arg == "--ros-args") {
            break;
        } else if (arg == "all") {
            test_offboard_modes = true;
            test_target = true;
            test_awareness = true;
            test_ground_altitude_estimate = true;
        }

    }

    node = std::make_shared<rclcpp::Node>("maneuver_controller");

    rclcpp::Node *nodePtr = node.get();

    std::shared_ptr<Configurator> configurator = std::make_shared<Configurator>(nodePtr,"maneuver_controller");
    configurator->DeclareParameter("/control/maneuver_controller/ground_estimate_window_size", rclcpp::ParameterType::PARAMETER_INTEGER);
    configurator->DeclareParameter("/control/maneuver_controller/ground_estimate_update_period_ms", rclcpp::ParameterType::PARAMETER_INTEGER);
    configurator->DeclareParameter("/control/maneuver_controller/ground_estimate_initial_delay_s", rclcpp::ParameterType::PARAMETER_DOUBLE);
    configurator->DeclareParameter("/control/maneuver_controller/landed_altitude_threshold", rclcpp::ParameterType::PARAMETER_DOUBLE);
    configurator->DeclareParameter("/control/maneuver_controller/landed_altitude_threshold_on_start", rclcpp::ParameterType::PARAMETER_DOUBLE);
    configurator->DeclareParameter("/control/maneuver_controller/on_cable_max_euc_distance", rclcpp::ParameterType::PARAMETER_DOUBLE);
    configurator->DeclareParameter("/control/maneuver_controller/fail_on_unable_to_locate", rclcpp::ParameterType::PARAMETER_BOOL);
    configurator->DeclareParameter("/control/maneuver_controller/combined_drone_awareness_pub_period_ms", rclcpp::ParameterType::PARAMETER_INTEGER);
    configurator->DeclareParameter("/control/maneuver_controller/use_gripper_status_condition", rclcpp::ParameterType::PARAMETER_BOOL);
    configurator->DeclareParameter("/tf/cable_gripper_frame_id", rclcpp::ParameterType::PARAMETER_STRING);
    configurator->DeclareParameter("/tf/drone_frame_id", rclcpp::ParameterType::PARAMETER_STRING);
    configurator->DeclareParameter("/tf/world_frame_id", rclcpp::ParameterType::PARAMETER_STRING);
    configurator->DeclareParameter("/tf/ground_frame_id", rclcpp::ParameterType::PARAMETER_STRING);
    configurator->CreateConfiguration("combined_drone_awareness_handler", {
        iii_drone::configuration::configuration_entry_t("/control/maneuver_controller/ground_estimate_window_size", rclcpp::ParameterType::PARAMETER_INTEGER),
        iii_drone::configuration::configuration_entry_t("/control/maneuver_controller/ground_estimate_update_period_ms", rclcpp::ParameterType::PARAMETER_INTEGER),
        iii_drone::configuration::configuration_entry_t("/control/maneuver_controller/ground_estimate_initial_delay_s", rclcpp::ParameterType::PARAMETER_DOUBLE),
        iii_drone::configuration::configuration_entry_t("/control/maneuver_controller/landed_altitude_threshold", rclcpp::ParameterType::PARAMETER_DOUBLE),
        iii_drone::configuration::configuration_entry_t("/control/maneuver_controller/landed_altitude_threshold_on_start", rclcpp::ParameterType::PARAMETER_DOUBLE),
        iii_drone::configuration::configuration_entry_t("/control/maneuver_controller/on_cable_max_euc_distance", rclcpp::ParameterType::PARAMETER_DOUBLE),
        iii_drone::configuration::configuration_entry_t("/control/maneuver_controller/fail_on_unable_to_locate", rclcpp::ParameterType::PARAMETER_BOOL),
        iii_drone::configuration::configuration_entry_t("/control/maneuver_controller/combined_drone_awareness_pub_period_ms", rclcpp::ParameterType::PARAMETER_INTEGER),
        iii_drone::configuration::configuration_entry_t("/control/maneuver_controller/use_gripper_status_condition", rclcpp::ParameterType::PARAMETER_BOOL),
        iii_drone::configuration::configuration_entry_t("/tf/cable_gripper_frame_id", rclcpp::ParameterType::PARAMETER_STRING),
        iii_drone::configuration::configuration_entry_t("/tf/drone_frame_id", rclcpp::ParameterType::PARAMETER_STRING),
        iii_drone::configuration::configuration_entry_t("/tf/world_frame_id", rclcpp::ParameterType::PARAMETER_STRING),
        iii_drone::configuration::configuration_entry_t("/tf/ground_frame_id", rclcpp::ParameterType::PARAMETER_STRING),
    });
    configurator->validate();

    auto tfBuffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

    cdah = std::make_shared<CombinedDroneAwarenessHandler>(
        configurator->GetConfiguration("combined_drone_awareness_handler"),
        tfBuffer,
        nodePtr
    );

    // if (test_offboard_modes) testOffboardModes(combinedDroneAwarenessHandler, node);
    // if (test_target) testTarget(combinedDroneAwarenessHandler, node);

    // std::thread test_thread(test, node, std::ref(combinedDroneAwarenessHandler));

    timer = node->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(test)
    );

    while(true) {
        rclcpp::spin_some(node);
        if (timer->is_canceled()) {
            RCLCPP_INFO(node->get_logger(), "Test done, shutting down");
            break;
        }
    }

    delete configurator.get();
    delete tfBuffer.get();
    delete tfListener.get();
    delete cdah.get();
    delete node.get();

    rclcpp::shutdown();
    return 0;
}
