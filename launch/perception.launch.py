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
    hough_transformer_log_level = LaunchConfiguration("hough_transformer_log_level")

    hough_transformer_log_level_arg = DeclareLaunchArgument(
        "hough_transformer_log_level",
        default_value=["info"],
        description="The logging level for the hough transformer node, default is INFO",
    )
    
    pl_dir_computer_log_level = LaunchConfiguration("pl_dir_computer_log_level")
    
    pl_dir_computer_log_level_arg = DeclareLaunchArgument(
        "pl_dir_computer_log_level",
        default_value=["info"],
        description="The logging level for the pl_dir_computer node, default is INFO",
    )
    
    pl_mapper_log_level = LaunchConfiguration("pl_mapper_log_level")
    
    pl_mapper_log_level_arg = DeclareLaunchArgument(
        "pl_mapper_log_level",
        default_value=["info"],
        description="The logging level for the pl_mapper node, default is INFO",
    )
    
    hough = Node(
        package="iii_drone_core",
        executable="hough_transformer",
        namespace="/perception/hough_transformer",
        arguments=["--ros-args", "--log-level", hough_transformer_log_level]
    )

    pl_dir_computer = Node(
        package="iii_drone_core",
        executable="pl_dir_computer",
        namespace="/perception/pl_dir_computer",
        arguments=["--ros-args", "--log-level", pl_dir_computer_log_level]
    )

    pl_mapper = Node(
        package="iii_drone_core",
        executable="pl_mapper",
        namespace="/perception/pl_mapper",
        arguments=["--ros-args", "--log-level", pl_mapper_log_level]
    )

    return LaunchDescription([
        hough,
        pl_dir_computer,
        pl_mapper,
        hough_transformer_log_level_arg,
        pl_dir_computer_log_level_arg,
        pl_mapper_log_level_arg
    ])
