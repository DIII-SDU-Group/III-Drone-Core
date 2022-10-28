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

    camera_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam",
        namespace="cable_camera",
        parameters=[config]
    )

    mmwave_node = Node(
        package="iwr6843aop_pub",
        executable="pcl_pub",
        namespace="mmwave",
        remappings=[("/mmwave/iwr6843_pcl", "/mmwave/pcl")],
        parameters=[config]
    )


    return LaunchDescription([
        camera_node,
        mmwave_node
    ])
