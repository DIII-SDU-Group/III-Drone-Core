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
    camera_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam",
        namespace="/sensor/cable_camera",
    )

    mmwave_node = Node(
        package="iwr6843aop_pub",
        executable="pcl_pub",
        namespace="/sensor/mmwave",
        remappings=[("/sensor/iwr6843_pcl", "/sensor/mmwave/pcl")],
    )


    return LaunchDescription([
        camera_node,
        mmwave_node
    ])
