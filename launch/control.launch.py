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
    config = "/home/" + os.getenv("USER") + "/.config/iii_drone/params.yaml"

    trajectory_controller = Node(
        package="iii_drone_core",
        executable="trajectory_controller",
        namespace="/control/trajectory_controller",
        parameters=[config],
        arguments=["--ros-args", "--log-level", "control.trajectory_controller:=debug"]
    )

    return LaunchDescription([
        trajectory_controller
    ])
