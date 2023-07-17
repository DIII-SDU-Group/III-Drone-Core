from struct import pack
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    user = os.getenv("USER")
    config = "/home/" + user + "/config.yaml"

    cable_drum_controller = Node(
        package="iii_drone",
        executable="cable_drum_controller",
        parameters=[config],
        arguments=["--ros-args", "--log-level", "trajectory_controller.trajectory_controller:=info"]
    )

    double_cable_lander = Node(
        package="iii_drone",
        executable="double_cable_lander",
        parameters=[config],
        arguments=["--ros-args", "--log-level", "trajectory_controller.trajectory_controller:=info"]
    )

    return LaunchDescription([
        cable_drum_controller,
        double_cable_lander
    ])
