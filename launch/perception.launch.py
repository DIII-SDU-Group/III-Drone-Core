from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import os

from iii_drone_configuration.schema_utils import resolve_active_parameter_file, seed_runtime_configuration


def _resolve_ros_params_file(profile_name: str) -> str:
    seed_runtime_configuration(profile_name)
    return str(resolve_active_parameter_file(profile_name))


def _parameter_sources(profile_name: str) -> list[object]:
    return [_resolve_ros_params_file(profile_name), {"use_sim_time": profile_name == "sim"}]


def generate_launch_description():
    simulation = os.getenv("SIMULATION", "false").lower() == "true"
    profile_name = "sim" if simulation else "real"

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
        arguments=["--ros-args", "--log-level", hough_transformer_log_level],
        parameters=_parameter_sources(profile_name),
    )

    pl_dir_computer = Node(
        package="iii_drone_core",
        executable="pl_dir_computer",
        namespace="/perception/pl_dir_computer",
        arguments=["--ros-args", "--log-level", pl_dir_computer_log_level],
        parameters=_parameter_sources(profile_name),
    )

    pl_mapper = Node(
        package="iii_drone_core",
        executable="pl_mapper",
        namespace="/perception/pl_mapper",
        arguments=["--ros-args", "--log-level", pl_mapper_log_level],
        parameters=_parameter_sources(profile_name),
    )

    return LaunchDescription([
        hough,
        pl_dir_computer,
        pl_mapper,
        hough_transformer_log_level_arg,
        pl_dir_computer_log_level_arg,
        pl_mapper_log_level_arg
    ])
