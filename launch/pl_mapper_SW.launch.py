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
    config = os.path.join(
        get_package_share_directory('iii_drone'),
        'config',
        'params.yaml'
    )

    hough = Node(
        package="iii_drone",
        executable="hough_transformer_SW",
        parameters=[config]
    )

    pl_dir_computer = Node(
        package="iii_drone",
        executable="pl_dir_computer",
        parameters=[config]
    )

    pl_mapper = Node(
        package="iii_drone",
        executable="pl_mapper",
        parameters=[config]
    )

    return LaunchDescription([
        hough,
        pl_dir_computer,
        pl_mapper
    ])
