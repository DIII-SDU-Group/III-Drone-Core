#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from launch.event_handlers import OnProcessIO
import launch
from ros2launch.api import get_share_file_path_from_package
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

from px4_msgs.msg import RcChannels

from multiprocessing import Process
from threading import Thread
from time import sleep
import psutil

from threading import Lock
import os, signal
import asyncio


class RcNodeLauncher(Node):
    def __init__(self):
        super().__init__("rc_node_launcher")

        self.get_logger().info("Initializing rc_node_launcher node")

        self.declare_parameter("rc_channel_indexes",[7,8])
        self.declare_parameter("rc_channel_thresholds", [0.75,0.75])
        self.declare_parameter("rc_channel_launch_files", ["sensors_real.launch.py", "pl_mapper_SW.launch.py"])
        self.declare_parameter("rc_channel_launch_packages", ["iii_drone", "iii_drone"])
        self.declare_parameter("rc_channel_terminate_all_idx", 11)
        self.declare_parameter("rc_channel_terminate_all_threshold", 0.75)

        self.n_rc_channel_commands = None

        rc_channel_indexes, rc_channel_thresholds, rc_channel_launch_files, rc_channel_launch_packages = self.load_rc_params()

        self.rc_channels = [-1 for i in range(18)]
        self.rc_channels_lock = Lock()

        self.processes = [None for i in range(len(rc_channel_indexes))]
        self.processes_are_running = [False for i in range(len(rc_channel_indexes))]

        # ROS2 timer
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.sub = self.create_subscription(
            RcChannels,
            "/fmu/rc_channels/out",
            self.callback,
            10
        )

    def load_rc_params(self):
        rc_channel_indexes = self.get_parameter("rc_channel_indexes").value
        rc_channel_thresholds = self.get_parameter("rc_channel_thresholds").value
        rc_channel_launch_files = self.get_parameter("rc_channel_launch_files").value
        rc_channel_launch_packages = self.get_parameter("rc_channel_launch_packages").value

        if self.n_rc_channel_commands is None:
            self.n_rc_channel_commands = len(rc_channel_indexes)

            # if length of arrays are not equal, log error and exit
            if len(rc_channel_indexes) != len(rc_channel_thresholds) or len(rc_channel_indexes) != len(rc_channel_launch_files) or len(rc_channel_indexes) != len(rc_channel_launch_packages):
                self.get_logger().fatal("rc_channel_indexes, rc_channel_thresholds, rc_channel_launch_files, rc_channel_launch_packages must have the same length")
                exit(1)

        else:
            # If length of arrays are not equal to previous length, log error and exit
            if len(rc_channel_indexes) != self.n_rc_channel_commands or len(rc_channel_thresholds) != self.n_rc_channel_commands or len(rc_channel_launch_files) != self.n_rc_channel_commands or len(rc_channel_launch_packages) != self.n_rc_channel_commands:
                self.get_logger().fatal("rc_channel_indexes, rc_channel_thresholds, rc_channel_launch_files, rc_channel_launch_packages must have the same length")
                exit(1)

        return rc_channel_indexes, rc_channel_thresholds, rc_channel_launch_files, rc_channel_launch_packages

    def callback(self, msg: RcChannels):
        self.get_logger().debug("Received RC message")

        self.rc_channels_lock.acquire()
        self.rc_channels = msg.channels
        self.rc_channels_lock.release()

    def timer_callback(self):
        self.get_logger().debug("Timer callback")

        rc_channel_indexes, rc_channel_thresholds, rc_channel_launch_files, rc_channel_launch_packages = self.load_rc_params()

        rc_terminate_all_idx = self.get_parameter("rc_channel_terminate_all_idx").value
        rc_terminate_all_threshold = self.get_parameter("rc_channel_terminate_all_threshold").value

        self.rc_channels_lock.acquire()
        rc_channel_vals = self.rc_channels
        self.rc_channels_lock.release()

        if rc_channel_vals[rc_terminate_all_idx] > rc_terminate_all_threshold:
            for i in range(len(self.processes)):
                if self.processes_are_running[i]:
                    self.get_logger().debug("Attempting to terminate launch file: " + rc_channel_launch_files[i])
                    child_processes = psutil.Process(self.processes[i].pid).children(recursive=True)
                    for child in child_processes:
                        self.get_logger().debug("Terminating child process: " + str(child.pid))
                        # child.kill()
                        os.kill(child.pid, signal.SIGTERM)

                    for child in child_processes:
                        self.get_logger().debug("Waiting for child process to terminate: " + str(child.pid))
                        child.wait()
                    os.kill(self.processes[i].pid, signal.SIGTERM)
                    while(self.processes[i].is_alive()):
                        self.get_logger().debug("Waiting for launch file to terminate: " + rc_channel_launch_files[i])
                    self.processes_are_running[i] = False
                    self.get_logger().debug("Terminated launch file: " + rc_channel_launch_files[i])

        else:
            for (i, idx) in enumerate(rc_channel_indexes):
                if rc_channel_vals[idx] > rc_channel_thresholds[i]:
                    if not self.processes_are_running[i]:
                        launch_service = self.launch_a_launch_file(rc_channel_launch_packages[i], rc_channel_launch_files[i], debug=False)
                        self.processes[i] = Process(target=self.launch_process_target, args=(launch_service,))
                        self.processes[i].start()
                        self.processes_are_running[i] = True
                        self.get_logger().debug("Started launch file: " + rc_channel_launch_files[i])
                else:
                    if self.processes_are_running[i]:
                        self.get_logger().debug("Attempting to terminate launch file: " + rc_channel_launch_files[i])
                        child_processes = psutil.Process(self.processes[i].pid).children(recursive=True)
                        for child in child_processes:
                            self.get_logger().debug("Terminating child process: " + str(child.pid))
                            # child.kill()
                            os.kill(child.pid, signal.SIGTERM)

                        for child in child_processes:
                            self.get_logger().debug("Waiting for child process to terminate: " + str(child.pid))
                            child.wait()
                        os.kill(self.processes[i].pid, signal.SIGTERM)
                        while(self.processes[i].is_alive()):
                            self.get_logger().debug("Waiting for launch file to terminate: " + rc_channel_launch_files[i])
                        self.processes_are_running[i] = False
                        self.get_logger().debug("Terminated launch file: " + rc_channel_launch_files[i])

    def launch_process_target(self, launch_service):
        launch_service.run()

    def launch_a_launch_file(self, package, launch_file, debug=False):
        """Launch a given launch file (by path) and pass it the given launch file arguments."""
        launch_service = launch.LaunchService(argv=None, debug=debug)
        # Include the user provided launch file using IncludeLaunchDescription so that the
        # location of the current launch file is set.
        launch_file_path = get_package_share_directory(package) + '/launch/' + launch_file
        launch_description = launch.LaunchDescription([
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.AnyLaunchDescriptionSource(
                    launch_file_path
                )
            ),
            # launch.actions.RegisterEventHandler(
            #     OnProcessIO(
            #         on_stdout=lambda info: print('>>>'+str(info.text)+'<<<'),
            #         on_stderr=lambda info: print('***'+str(info.text)+'***')
            #     )
            # )
        ])
        launch_service.include_launch_description(launch_description)
        return launch_service

def main(args=None):
    rclpy.init(args=args)

    publisher = RcNodeLauncher()

    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

