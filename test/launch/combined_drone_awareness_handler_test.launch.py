from launch import LaunchDescription
from launch_ros.actions import Node
import os
import yaml

from iii_drone_configuration.schema_utils import resolve_active_parameter_file, seed_runtime_configuration


def _resolve_ros_params_file() -> str:
    seed_runtime_configuration("sim")
    return str(resolve_active_parameter_file("sim"))


def _parameter_sources() -> list[object]:
    return [_resolve_ros_params_file(), {"use_sim_time": True}]


def generate_launch_description():
    ros_params = _resolve_ros_params_file()
    with open(ros_params, "r") as file:
        ros_params_dict = yaml.safe_load(file) or {}
    params = ros_params_dict["/**"]["ros__parameters"]

    simulation = os.getenv("SIMULATION", "false").lower() == "true"

    if not simulation:
        raise Exception("This launch file is only for simulation")
    
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        arguments=["udp4", "--port", "8888", "--ros-args", "--log-level", "error"],
    )

    mmwave = Node(
        package='iii_drone_simulation',
        executable='depth_cam_to_mmwave',
        name='depth_cam_to_mmwave',
        arguments=["--ros-args", "--log-level", "error"]
    )
    
    drone_frame_id = params["/tf/drone_frame_id"]
    cable_gripper_frame_id = params["/tf/cable_gripper_frame_id"]
    mmwave_frame_id = params["/tf/mmwave_frame_id"]
    depth_cam_frame_id = params["/tf/sim/depth_cam_frame_id"]

    args = [str(val) for val in params["/tf/sim/drone_to_cable_gripper"]] + [drone_frame_id, cable_gripper_frame_id]
    tf_drone_to_cable_gripper = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=args + ["--ros-args", "--log-level", "error"]
    )

    args = [str(val) for val in params["/tf/sim/drone_to_mmwave"]] + [drone_frame_id, mmwave_frame_id]
    tf_drone_to_iwr = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=args + ["--ros-args", "--log-level", "error"]
    )

    args = [str(val) for val in params["/tf/sim/drone_to_depth_cam"]] + [drone_frame_id, depth_cam_frame_id]
    tf_drone_to_depth_cam = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=args + ["--ros-args", "--log-level", "error"]
    )

    world_to_drone = Node(
        package="iii_drone_core",
        executable="drone_frame_broadcaster",
        arguments=["--ros-args", "--log-level", "error"]
    )
        
    hough = Node(
        package="iii_drone_core",
        executable="hough_transformer",
        namespace="/perception/hough_transformer",
        arguments=["--ros-args", "--log-level", "error"]
    )

    pl_dir_computer = Node(
        package="iii_drone_core",
        executable="pl_dir_computer",
        namespace="/perception/pl_dir_computer",
        arguments=["--ros-args", "--log-level", "error"]
    )

    pl_mapper = Node(
        package="iii_drone_core",
        executable="pl_mapper",
        namespace="/perception/pl_mapper",
        arguments=["--ros-args", "--log-level", "error"]
    )
    
    combined_drone_awareness_handler_test = Node(
        package="iii_drone_core",
        executable="combined_drone_awareness_handler_test",
        arguments=["--ros-args", "--log-level", "combined_drone_awareness_handler_test:=debug"]
    )
    
    launch_list = [micro_ros_agent, mmwave, tf_drone_to_cable_gripper, tf_drone_to_depth_cam, tf_drone_to_iwr, world_to_drone, hough, pl_dir_computer, pl_mapper]#, combined_drone_awareness_handler_test]

    cmd = "ros2 node list"
    running_nodes = os.popen(cmd).read().split("\n")
    
    configuration_server_is_running = False

    for node in running_nodes:
        if "configuration_server" in node:
            configuration_server_is_running = True
            break

    if not configuration_server_is_running:
        configuration_server = Node(
            package='iii_drone_core',
            executable='configuration_server_node.py',
            parameters=_parameter_sources(),
            arguments=["--ros-args", "--log-level", "error"]
        )
        
        launch_list = [configuration_server] + launch_list

        
    return LaunchDescription(launch_list)
