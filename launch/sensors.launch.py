from struct import pack
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ros_params = os.path.join(os.getenv("CONFIG_BASE_DIR", default="~/.config"), "iii_drone", "ros_params.yaml")

    mmwave_log_level = LaunchConfiguration("mmwave_log_level")

    mmwave_log_level_arg = DeclareLaunchArgument(
        "mmwave_log_level",
        default_value=["info"],
        description="The logging level for the mmwave node, default is INFO",
    )
    camera_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam",
        namespace="/sensor/cable_camera",
        parameters=[ros_params],
    )

    mmwave_node = Node(
        package="iwr6843aop_pub",
        executable="pcl_pub",
        namespace="/sensor/mmwave",
        remappings=[("/sensor/iwr6843_pcl", "/sensor/mmwave/pcl")],
        arguments=["--ros-args", "--log-level", mmwave_log_level],
        parameters=[ros_params],
    )


    return LaunchDescription([
        camera_node,
        mmwave_log_level_arg,
        mmwave_node,
    ])
