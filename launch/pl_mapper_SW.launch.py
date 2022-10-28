from struct import pack
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    hough = Node(
        package="iii_drone",
        executable="hough_pub"
    )

    pl_dir_computer = Node(
        package="iii_drone",
        executable="pl_dir_computer"
    )

    pl_mapper = Node(
        package="iii_drone",
        executable="pl_mapper"
    )

    return LaunchDescription([
        hough,
        pl_dir_computer,
        pl_mapper
    ])
