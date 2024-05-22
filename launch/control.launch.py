from struct import pack
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ros_params = "/home/" + os.getenv("USER") + "/.config/iii_drone/ros_params.yaml"
    
    trajectory_generator_log_level = LaunchConfiguration("trajectory_generator_log_level")

    trajectory_generator_log_level_arg = DeclareLaunchArgument(
        "trajectory_generator_log_level",
        default_value=["info"],
        description="The logging level for the trajectory_generator node, default is INFO",
    )
    
    maneuver_controller_log_level = LaunchConfiguration("maneuver_controller_log_level")
    
    maneuver_controller_log_level_arg = DeclareLaunchArgument(
        "maneuver_controller_log_level",
        default_value=["info"],
        description="The logging level for the maneuver_controller node, default is INFO",
    )
    
    trajectory_generator = Node(
        package="iii_drone_core",
        executable="trajectory_generator",
        namespace="/control/trajectory_generator",
        arguments=["--ros-args", "--log-level", trajectory_generator_log_level],
        parameters=[ros_params],
    )
    
    maneuver_controller = Node(
        package="iii_drone_core",
        executable="maneuver_controller",
        namespace="/control/maneuver_controller",
        arguments=["--ros-args", "--log-level", maneuver_controller_log_level],
        parameters=[ros_params],
    )

    return LaunchDescription([
        trajectory_generator_log_level_arg,
        trajectory_generator,
        maneuver_controller_log_level_arg,
        maneuver_controller
    ])
