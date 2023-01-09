from struct import pack
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('iii_drone'),
        'config',
        'params.yaml'
    )

    config_dict = yaml.safe_load(open(config,"r").read())

    world_frame_id = config_dict["/**"]["ros__parameters"]["world_frame_id"]
    drone_frame_id = config_dict["/**"]["ros__parameters"]["drone_frame_id"]
    t265_world_frame_id = config_dict["/**"]["ros__parameters"]["t265_world_frame_id"]
    t265_camera_frame_id = config_dict["/**"]["ros__parameters"]["t265_camera_frame_id"]

    args = [str(val) for val in config_dict["tf"]["real"]["ros__parameters"]["drone_to_t265"]] + [drone_frame_id, t265_camera_frame_id]
    tf_drone_to_t265 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=args
    )

    args = [str(val) for val in config_dict["tf"]["real"]["ros__parameters"]["world_to_t265_world"]] + [world_frame_id, t265_world_frame_id]
    tf_world_to_t265_world = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=args
    )

    tf_t265_world_to_odom_frame = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", t265_world_frame_id, "odom_frame"]
    )

    realsense_share_dir = get_package_share_directory('realsense2_camera')

    t265 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([realsense_share_dir, 'launch', 'rs_launch.py'])),
    )
    

    t265_vio_republisher = Node(
        package="iii_drone",
        executable="t265_vio_republisher",
        parameters=[config]
    )

    return LaunchDescription([
        tf_drone_to_t265,
        tf_world_to_t265_world,
        tf_t265_world_to_odom_frame,
        t265,
        t265_vio_republisher
    ])
