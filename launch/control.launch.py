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
        parameters=_parameter_sources(profile_name),
    )
    
    maneuver_controller = Node(
        package="iii_drone_core",
        executable="maneuver_controller",
        namespace="/control/maneuver_controller",
        arguments=["--ros-args", "--log-level", maneuver_controller_log_level],
        parameters=_parameter_sources(profile_name),
    )

    return LaunchDescription([
        trajectory_generator_log_level_arg,
        trajectory_generator,
        maneuver_controller_log_level_arg,
        maneuver_controller
    ])
