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
    parameters_file = ros_params_dict["/**"]["ros__parameters"]["parameters_file_path"]

    # Replace "~" with "/home/<user>" in the path
    if parameters_file[0] == "~":
        parameters_file = "/home/" + os.getenv("USER") + parameters_file[1:]
    
    params_dict = yaml.safe_load(open(parameters_file,"r").read())
    
    simulation = params_dict["global"]["simulation"]["value"]
    
    if simulation:
        micro_ros_agent = Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            arguments=["udp4", "--port", "8888"],
        )
    
        sensors = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('iii_drone_simulation'), 'launch', 'sensors_sim.launch.py'])
            )
        )
        
        tf = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('iii_drone_simulation'), 'launch', 'tf_sim.launch.py'])
            )
        )
        
    else:
        micro_ros_agent = Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            arguments=["udp4", "--port", "8888"], # Fix for real drone
        )

        sensors = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('iii_drone_core'), 'launch', 'sensors.launch.py'])
            )
        )
        
        tf = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('iii_drone_core'), 'launch', 'tf.launch.py'])
            )
        )
        
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('iii_drone_core'), 'launch', 'perception.launch.py'])
        )
    )
    
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('iii_drone_core'), 'launch', 'control.launch.py'])
        )
    )
    
    launch_list = [micro_ros_agent, sensors, tf, perception, control]

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
        )
        
        launch_list = [configuration_server] + launch_list

        
    return LaunchDescription(launch_list)
