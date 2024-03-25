from struct import pack
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    ros_params = "/home/" + os.getenv("USER") + "/.config/iii_drone/ros_params.yaml"
    ros_params_dict = yaml.safe_load(open(ros_params,"r").read())
    parameters_dir = ros_params_dict["/**"]["ros__parameters"]["parameters_dir"]
    default_parameter_file = ros_params_dict["/**"]["ros__parameters"]["default_parameter_file"]
    parameters_file = os.path.join(parameters_dir, default_parameter_file)

    # Replace "~" with "/home/<user>" in the path
    if parameters_file[0] == "~":
        parameters_file = "/home/" + os.getenv("USER") + parameters_file[1:]
    
    params_dict = yaml.safe_load(open(parameters_file,"r").read())
    
    simulation = params_dict["global"]["simulation"]["value"]

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
    
    drone_frame_id = params_dict["tf"]["drone_frame_id"]["value"]
    cable_gripper_frame_id = params_dict["tf"]["cable_gripper_frame_id"]["value"]
    mmwave_frame_id = params_dict["tf"]["mmwave_frame_id"]["value"]
    depth_cam_frame_id = params_dict["tf"]["sim"]["depth_cam_frame_id"]["value"]

    args = [str(val) for val in params_dict["tf"]["sim"]["drone_to_cable_gripper"]["value"]] + [drone_frame_id, cable_gripper_frame_id]
    tf_drone_to_cable_gripper = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=args + ["--ros-args", "--log-level", "error"]
    )

    args = [str(val) for val in params_dict["tf"]["sim"]["drone_to_mmwave"]["value"]] + [drone_frame_id, mmwave_frame_id]
    tf_drone_to_iwr = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=args + ["--ros-args", "--log-level", "error"]
    )

    args = [str(val) for val in params_dict["tf"]["sim"]["drone_to_depth_cam"]["value"]] + [drone_frame_id, depth_cam_frame_id]
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
            parameters=[ros_params],
            arguments=["--ros-args", "--log-level", "error"]
        )
        
        launch_list = [configuration_server] + launch_list

        
    return LaunchDescription(launch_list)
