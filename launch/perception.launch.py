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

    hough = Node(
        package="iii_drone_core",
        executable="hough_transformer",
        namespace="perception",
        parameters=[config]
    )

    pl_dir_computer = Node(
        package="iii_drone_core",
        executable="pl_dir_computer",
        namespace="perception",
        parameters=[config]
    )

    pl_mapper = Node(
        package="iii_drone_core",
        executable="pl_mapper",
        namespace="perception",
        parameters=[config]
    )

    return LaunchDescription([
        hough,
        pl_dir_computer,
        pl_mapper
    ])
