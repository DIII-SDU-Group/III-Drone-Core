from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os
import yaml

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('iii_drone'),
        'config',
        'params.yaml'
    )

    ctrl_rc_launcher = Node(
        package="iii_drone",
        executable="rc_node_launcher.py",
        namespace="ctrl_rc_node_launcher",
        parameters=[config]
    )


    return LaunchDescription([
        ctrl_rc_launcher
    ])
