from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import os

def generate_launch_description():
    simulation = os.getenv("SIMULATION", "false").lower() == "true"
    ros_params = os.path.join(
        os.path.expanduser(os.getenv("CONFIG_BASE_DIR", default="~/.config")),
        "iii_drone",
        "ros_params_sim.yaml" if simulation else "ros_params_real.yaml"
    )

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
        parameters=[ros_params],
    )

    pl_dir_computer = Node(
        package="iii_drone_core",
        executable="pl_dir_computer",
        namespace="/perception/pl_dir_computer",
        arguments=["--ros-args", "--log-level", pl_dir_computer_log_level],
        parameters=[ros_params],
    )

    pl_mapper = Node(
        package="iii_drone_core",
        executable="pl_mapper",
        namespace="/perception/pl_mapper",
        arguments=["--ros-args", "--log-level", pl_mapper_log_level],
        parameters=[ros_params],
    )

    return LaunchDescription([
        hough,
        pl_dir_computer,
        pl_mapper,
        hough_transformer_log_level_arg,
        pl_dir_computer_log_level_arg,
        pl_mapper_log_level_arg
    ])
