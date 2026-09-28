from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import os
import yaml

from iii_drone_configuration.schema_utils import resolve_active_parameter_file, seed_runtime_configuration



def _static_transform_arguments(values, frame_id, child_frame_id):
    """Named static_transform_publisher arguments for [x, y, z, yaw, pitch, roll].

    The positional form is deprecated in Jazzy and logs a warning per start.
    """
    if len(values) != 6:
        raise ValueError(
            f"static transform {frame_id}->{child_frame_id} needs [x, y, z, yaw, pitch, roll], got {values!r}"
        )
    names = ("--x", "--y", "--z", "--yaw", "--pitch", "--roll")
    arguments = []
    for name, value in zip(names, values):
        arguments += [name, str(value)]
    return arguments + ["--frame-id", frame_id, "--child-frame-id", child_frame_id]

def _runtime_profile() -> str:
    """Return the configuration identity for the aircraft-side TF graph.

    HIL deliberately runs the PX4-driven world-to-drone broadcaster from this
    launch file, but its payloads live in the Gazebo model.  Preserve the HIL
    identity so its own simulation-backed parameter set and selector are used.
    """
    profile = os.environ.get("III_SYSTEM_PROFILE", "real").strip()
    return profile if profile in {"real", "opti_track", "hil"} else "real"


def _static_transform_prefix(profile: str) -> str:
    """Select payload extrinsics without changing the dynamic pose source."""
    return "/tf/sim" if profile == "hil" else "/tf"


def _resolve_ros_params_file() -> str:
    profile = _runtime_profile()
    seed_runtime_configuration(profile)
    return str(resolve_active_parameter_file(profile))


def _parameter_sources() -> list[object]:
    return [_resolve_ros_params_file(), {"use_sim_time": False}]


def generate_launch_description():
    drone_frame_broadcaster_log_level = LaunchConfiguration("drone_frame_broadcaster_log_level")

    drone_frame_broadcaster_log_level_arg = DeclareLaunchArgument(
        "drone_frame_broadcaster_log_level",
        default_value=["info"],
        description="The logging level for the drone frame broadcaster node, default is INFO",
    )
    
    profile = _runtime_profile()
    transform_prefix = _static_transform_prefix(profile)
    ros_params = _resolve_ros_params_file()
    with open(ros_params, "r") as file:
        ros_params_dict = yaml.safe_load(file) or {}
    params = ros_params_dict["/**"]["ros__parameters"]

    drone_frame_id = params["/tf/drone_frame_id"]
    cable_gripper_frame_id = params["/tf/cable_gripper_frame_id"]
    mmwave_frame_id = params["/tf/mmwave_frame_id"]

    args = _static_transform_arguments(params[f"{transform_prefix}/drone_to_cable_gripper"], drone_frame_id, cable_gripper_frame_id)
    tf_drone_to_cable_gripper = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=args,
        parameters=_parameter_sources(),
    )

    args = _static_transform_arguments(params[f"{transform_prefix}/drone_to_mmwave"], drone_frame_id, mmwave_frame_id)
    tf_drone_to_iwr = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=args,
        parameters=_parameter_sources(),
    )

    world_to_drone = Node(
        package="iii_drone_core",
        executable="drone_frame_broadcaster",
        arguments=["--ros-args", "--log-level", drone_frame_broadcaster_log_level],
        parameters=_parameter_sources(),
    )

    # HIL gets static payload extrinsics from the workstation simulation
    # adapter. Keep only the PX4-driven dynamic broadcaster on the aircraft to
    # avoid a second publisher for the same static frames.
    transform_nodes = [world_to_drone] if profile == "hil" else [
        tf_drone_to_cable_gripper,
        tf_drone_to_iwr,
        world_to_drone,
    ]

    return LaunchDescription([*transform_nodes, drone_frame_broadcaster_log_level_arg])
