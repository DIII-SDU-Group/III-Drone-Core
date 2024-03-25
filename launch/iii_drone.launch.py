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
import yaml

def generate_launch_description():
    mmwave_log_level = LaunchConfiguration("mmwave_log_level")

    mmwave_log_level_arg = DeclareLaunchArgument(
        "mmwave_log_level",
        default_value=["warn"],
        description="The logging level for the mmwave node, default is warn",
    )

    hough_transformer_log_level = LaunchConfiguration("hough_transformer_log_level")

    hough_transformer_log_level_arg = DeclareLaunchArgument(
        "hough_transformer_log_level",
        default_value=["warn"],
        description="The logging level for the hough transformer node, default is warn",
    )
    
    pl_dir_computer_log_level = LaunchConfiguration("pl_dir_computer_log_level")
    
    pl_dir_computer_log_level_arg = DeclareLaunchArgument(
        "pl_dir_computer_log_level",
        default_value=["warn"],
        description="The logging level for the pl_dir_computer node, default is warn",
    )
    
    pl_mapper_log_level = LaunchConfiguration("pl_mapper_log_level")
    
    pl_mapper_log_level_arg = DeclareLaunchArgument(
        "pl_mapper_log_level",
        default_value=["warn"],
        description="The logging level for the pl_mapper node, default is warn",
    )
    
    trajectory_generator_log_level = LaunchConfiguration("trajectory_generator_log_level")

    trajectory_generator_log_level_arg = DeclareLaunchArgument(
        "trajectory_generator_log_level",
        default_value=["warn"],
        description="The logging level for the trajectory_generator node, default is warn",
    )

    drone_frame_broadcaster_log_level = LaunchConfiguration("drone_frame_broadcaster_log_level")

    drone_frame_broadcaster_log_level_arg = DeclareLaunchArgument(
        "drone_frame_broadcaster_log_level",
        default_value=["warn"],
        description="The logging level for the drone frame broadcaster node, default is warn",
    )

    configuration_server_log_level = LaunchConfiguration("configuration_server_log_level")
    
    configuration_server_log_level_arg = DeclareLaunchArgument(
        "configuration_server_log_level",
        default_value=["warn"],
        description="The logging level for the configuration server node, default is warn",
    )

    ros_params = "/home/" + os.getenv("USER") + "/.config/iii_drone/ros_params.yaml"
    ros_params_dict = yaml.safe_load(open(ros_params,"r").read())
    parameters_dir = ros_params_dict["/**"]["ros__parameters"]["parameters_dir"]
    default_parameter_file = ros_params_dict["/**"]["ros__parameters"]["default_parameter_file"]
    parameters_file = os.path.join(parameters_dir, default_parameter_file)

    # Replace "~" with "/home/<user>" in the path
    if parameters_file[0] == "~":
        parameters_file = "/home/" + os.getenv("USER") + parameters_file[1:]
    
    params_dict = yaml.safe_load(open(parameters_file,"r").read())
    
    simulation = params_dict["global"]["simulation"]["value"]
    
    if simulation:
        micro_ros_agent = Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            arguments=["udp4", "--port", "8888"],
        )
    
        sensors = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('iii_drone_simulation'), 'launch', 'sensors_sim.launch.py'])
            ),
            launch_arguments={"mmwave_log_level": mmwave_log_level}.items()
        )
        
        tf = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('iii_drone_simulation'), 'launch', 'tf_sim.launch.py'])
            ),
            launch_arguments={"drone_frame_broadcaster_log_level": drone_frame_broadcaster_log_level}.items()
        )
        
    else:
        micro_ros_agent = Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            arguments=["udp4", "--port", "8888"], # Fix for real drone
        )

        sensors = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('iii_drone_core'), 'launch', 'sensors.launch.py'])
            ),
            launch_arguments={"mmwave_log_level": mmwave_log_level}.items()
        )
        
        tf = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('iii_drone_core'), 'launch', 'tf.launch.py'])
            ),
            launch_arguments={"drone_frame_broadcaster_log_level": drone_frame_broadcaster_log_level}.items()
        )
        
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('iii_drone_core'), 'launch', 'perception.launch.py'])
        ),
        launch_arguments={"hough_transformer_log_level": hough_transformer_log_level, "pl_dir_computer_log_level": pl_dir_computer_log_level, "pl_mapper_log_level": pl_mapper_log_level}.items()
    )
    
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('iii_drone_core'), 'launch', 'control.launch.py'])
        ),
        launch_arguments={"trajectory_generator_log_level": trajectory_generator_log_level}.items()
    )
    
    launch_list = [micro_ros_agent, sensors, tf, perception, control]

    cmd = "ros2 node list"
    running_nodes = os.popen(cmd).read().split("\n")
    
    configuration_server_is_running = False

    for node in running_nodes:
        if "configuration_server" in node:
            configuration_server_is_running = True
            break

    if not configuration_server_is_running:
        configuration_server = Node(
            package='iii_drone_core',
            executable='configuration_server_node.py',
            parameters=[ros_params],
            arguments=["--ros-args", "--log-level", configuration_server_log_level],
        )
        
        launch_list = [configuration_server] + launch_list
        
    args_list = [mmwave_log_level_arg, hough_transformer_log_level_arg, pl_dir_computer_log_level_arg, pl_mapper_log_level_arg, trajectory_generator_log_level_arg, drone_frame_broadcaster_log_level_arg, configuration_server_log_level_arg]

        
    return LaunchDescription(args_list + launch_list)
