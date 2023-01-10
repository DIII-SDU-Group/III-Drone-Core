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
    share_dir = get_package_share_directory('iii_drone')

    sensors = IncludeLaunchDescription(PythonLaunchDescriptionSource(share_dir + "/launch/sensors_real.launch.py"))
    pl_mapper = IncludeLaunchDescription(PythonLaunchDescriptionSource(share_dir + "/launch/pl_mapper_SW.launch.py"))

    return LaunchDescription([
        sensors,
        pl_mapper
    ])
