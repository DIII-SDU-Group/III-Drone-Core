#!/usr/bin/python3

###############################################################################
# Includes
###############################################################################

import struct
from unittest import result
from xml.sax.handler import property_xml_string
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Int32
from sensor_msgs.msg import Image, PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from iii_interfaces.msg import Powerline, PowerlineDirection, ControlState
from iii_interfaces.action import Takeoff, Landing, FlyToPosition, FlyUnderCable, CableLanding, CableTakeoff
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import cv2 as cv
from cv_bridge import CvBridge

import numpy as np

import time
import copy
import sys
import os
import math 
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.lines as mlines

from threading import Thread, Lock

import tkinter # note that module name has changed from Tkinter in Python 2 to tkinter in Python 3
from tkinter import Toplevel, ttk
from PIL import ImageTk, Image

from time import sleep
import yaml
import subprocess

###############################################################################
# Methods
###############################################################################

def eulToQuat(eul):
    roll, pitch, yaw = eul[0], eul[1], eul[2]
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return np.array([qw, qx, qy, qz])

def quatToEul(quat):
    x = atan2(2*(quat[0]*quat[1] + quat[2]*quat[3]), 1-2*(quat[1]*quat[1] + quat[2]*quat[2])),
    y = asin(2*(quat[0]*quat[2] - quat[3]*quat[1])),
    z = atan2(2*(quat[0]*quat[3] + quat[1]*quat[2]), 1-2*(quat[2]*quat[2]+quat[3]*quat[3]))

    return np.array([x, y, z])

def quatInv(quat):
    return np.array([quat[0], -quat[1], -quat[2], -quat[3]])

def quatMultiply(quat1, quat2):
    w = quat1[0]*quat2[0] - quat1[1]*quat2[1] - quat1[2]*quat2[2] - quat1[3]*quat2[3],
    x = quat1[0]*quat2[1] + quat1[1]*quat2[0] + quat1[2]*quat2[3] - quat1[3]*quat2[2],
    y = quat1[0]*quat2[2] - quat1[1]*quat2[3] + quat1[2]*quat2[0] + quat1[3]*quat2[1],
    z = quat1[0]*quat2[3] + quat1[1]*quat2[2] - quat1[2]*quat2[1] + quat1[3]*quat2[0]

    return np.array([w, x, y, z])

def quatToMat(quat):
    mat = np.ndarray((3,3))

    q0 = quat[0]
    q1 = quat[1]
    q2 = quat[2]
    q3 = quat[3]
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3) 
    r02 = 2 * (q1 * q3 + q0 * q2) 
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    mat[0,0] = r00
    mat[0,1] = r01
    mat[0,2] = r02
    mat[1,0] = r10
    mat[1,1] = r11
    mat[1,2] = r12
    mat[2,0] = r20
    mat[2,1] = r21
    mat[2,2] = r22

    return mat

def eulToR(eul):
    cos_yaw = cos(eul[2])
    cos_pitch = cos(eul[1])
    cos_roll = cos(eul[0])
    sin_yaw = sin(eul[2])
    sin_pitch = sin(eul[1])
    sin_roll = sin(eul[0])

    mat = np.ndarray((3,3))

    mat[0,0] = cos_pitch*cos_yaw
    mat[0,1] = sin_roll*sin_pitch*cos_yaw-cos_roll*sin_yaw
    mat[0,2] = cos_roll*sin_pitch*cos_yaw+sin_roll*sin_yaw
    mat[1,0] = cos_pitch*sin_yaw
    mat[1,1] = sin_roll*sin_pitch*sin_yaw+cos_roll*cos_yaw
    mat[1,2] = cos_roll*sin_pitch*sin_yaw-sin_roll*cos_pitch
    mat[2,0] = -sin_pitch
    mat[2,1] = sin_roll*cos_pitch
    mat[2,2] = cos_roll*cos_pitch

    return mat

def matToQuat(R):
    tr = R[0,0] + R[1,1] + R[2,2]

    qw, qx, qy, qz = 0,0,0,0

    if (tr > 0):

        S = sqrt(tr+1.0) * 2
        qw = 0.25 * S
        qx = (R[2,1] - R[1,2]) / S
        qy = (R[0,2] - R[2,0]) / S
        qz = (R[1,0] - R[0,1]) / S

    elif ((R[0,0] > R[1,1]) and (R[0,0] > R[2,2])):
        S = sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
        qw = (R[2,1] - R[1,2]) / S
        qx = 0.25 * S
        qy = (R[0,1] + R[1,0]) / S 
        qz = (R[0,2] + R[2,0]) / S 

    elif (R(1,1) > R(2,2)):
        S = sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
        qw = (R[0,2] - R[2,0]) / S
        qx = (R[0,1] + R[1,0]) / S
        qy = 0.25 * S
        qz = (R[1,2] + R[2,1]) / S 
    else:
        S = sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
        qw = (R[1,0] - R[0,1]) / S
        qx = (R[0,2] + R[2,0]) / S
        qy = (R[1,2] + R[2,1]) / S
        qz = 0.25 * S

    quat = [qw, qx, qy, qz]

    return quat

###############################################################################
# Class
###############################################################################

class IIIGuiNode(Node):
    def __init__(self):
        super().__init__("iii_gui")

        self.declare_parameter("world_frame_id", "world")
        self.declare_parameter("drone_frame_id", "drone")

        self.declare_parameter("/pl_dir_computer/pl_dir_computer/kf_r", 0.1)

        self.declare_parameter("config_file_path", "III-Drone-ROS2-pkg/config/params.yaml")
        config_file_path = self.get_parameter("config_file_path").value

        self.config_file_path = os.path.dirname(os.path.realpath(__file__)).replace("install/iii_drone/lib/iii_drone", "src/"+config_file_path) 

        qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
        )
        
        self.powerline_tuples_ = [] # (id, point)
        self.powerline_quat_ = None
        
        self.pl_lock_ = Lock()
        self.img_lock_ = Lock()
        self.control_state_lock_ = Lock()
        self.action_status_lock_ = Lock()
        self.target_lock_ = Lock()
        self.traj_lock_ = Lock()

        self.future = None
        self.goal_handle = None
        self.action_client = None

        self.img_ = None
        self.control_state_ = "unknown"
        self.target = None
        self.traj = None

        self.takeoff_client = ActionClient(self, Takeoff, "/trajectory_controller/takeoff",feedback_sub_qos_profile=qos)
        self.landing_client = ActionClient(self, Landing, "/trajectory_controller/landing",feedback_sub_qos_profile=qos)
        self.fly_to_position_client = ActionClient(self, FlyToPosition, "/trajectory_controller/fly_to_position",feedback_sub_qos_profile=qos)
        self.fly_under_cable_client = ActionClient(self, FlyUnderCable, "/trajectory_controller/fly_under_cable",feedback_sub_qos_profile=qos)
        self.cable_landing_client = ActionClient(self, CableLanding, "/trajectory_controller/cable_landing",feedback_sub_qos_profile=qos)
        self.cable_takeoff_client = ActionClient(self, CableTakeoff, "/trajectory_controller/cable_takeoff",feedback_sub_qos_profile=qos)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        
        self.pl_sub_ = self.create_subscription(
            Powerline,
            "/pl_mapper/powerline",
            self.on_pl_msg,
            qos_profile=qos
        )

        self.control_state_sub_ = self.create_subscription(
            ControlState,
            "/trajectory_controller/control_state",
            self.on_state_msg,
            qos_profile=qos
        )

        self.planned_target_sub_ = self.create_subscription(
            PoseStamped,
            "/trajectory_controller/planned_target",
            self.on_planned_target_msg,
            qos_profile=qos
        )

        self.planned_trajectory_sub_ = self.create_subscription(
            Path,
            "/trajectory_controller/planned_trajectory",
            self.on_planned_trajectory_msg,
            qos_profile=qos
        )

        self.current_action = "None"
        self.action_status = "Idle"

    def on_pl_msg(self, msg: Powerline):
        if self.pl_lock_.acquire(blocking=True):
            self.powerline_tuples_ = []
            self.powerline_quat_ = None

            for i in range(msg.count):
                pose = msg.poses[i]
                id = msg.ids[i]
                point = [
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z
                ]
                self.powerline_tuples_.append((id, point))

                if (self.powerline_quat_ is None):
                    self.powerline_quat_ = [
                        pose.pose.orientation.w,
                        pose.pose.orientation.x,
                        pose.pose.orientation.y,
                        pose.pose.orientation.z
                    ]

            self.pl_lock_.release()

    def on_state_msg(self, msg: ControlState):
        if self.control_state_lock_.acquire(blocking=True):
            if msg.state == msg.CONTROL_STATE_INIT:
                self.control_state_ = "init"
            elif msg.state == msg.CONTROL_STATE_ON_GROUND_NON_OFFBOARD:
                self.control_state_ = "on ground non offboard"
            elif msg.state == msg.CONTROL_STATE_IN_FLIGHT_NON_OFFBOARD:
                self.control_state_ = "in flight non offboard"
            elif msg.state == msg.CONTROL_STATE_ARMING:
                self.control_state_ = "arming"
            elif msg.state == msg.CONTROL_STATE_SETTING_OFFBOARD:
                self.control_state_ = "setting offboard"
            elif msg.state == msg.CONTROL_STATE_TAKING_OFF:
                self.control_state_ = "taking off"
            elif msg.state == msg.CONTROL_STATE_HOVERING:
                self.control_state_ = "hovering"
            elif msg.state == msg.CONTROL_STATE_LANDING:
                self.control_state_ = "landing"
            elif msg.state == msg.CONTROL_STATE_IN_POSITIONAL_FLIGHT:
                self.control_state_ = "in positional flight"
            elif msg.state == msg.CONTROL_STATE_DURING_CABLE_LANDING:
                self.control_state_ = "during cable landing"
            elif msg.state == msg.CONTROL_STATE_ON_CABLE_ARMED:
                self.control_state_ = "on cable armed"
            elif msg.state == msg.CONTROL_STATE_DURING_CABLE_TAKEOFF:
                self.control_state_ = "during cable takeoff"
            elif msg.state == msg.CONTROL_STATE_HOVERING_UNDER_CABLE:
                self.control_state_ = "hovering under cable"
            else:
                self.control_state_ = "unknown"

            self.control_state_lock_.release()

    def on_planned_target_msg(self, msg: PoseStamped):
        if self.target_lock_.acquire(blocking=True):
            self.target = msg
            self.target_lock_.release()

    def on_planned_trajectory_msg(self, msg: Path):
        if self.traj_lock_.acquire(blocking=True):
            self.traj = msg
            self.traj_lock_.release()

    def get_img(self):
        img = None
        
        if self.img_lock_.acquire(blocking=True):
            img = self.img_
            self.img_lock_.release()
            
        return img

    def get_control_state(self):
        state = "unknown"
        
        if (self.control_state_lock_.acquire(blocking=True)):
            state = self.control_state_
            self.control_state_lock_.release()
            
        return state

    def get_target(self):
        target = None
        
        if (self.target_lock_.acquire(blocking=True)):
            drone_frame_id = self.get_parameter("drone_frame_id").value
            world_frame_id = self.get_parameter("world_frame_id").value
            tf = self.tf_buffer.lookup_transform(drone_frame_id, world_frame_id, rclpy.time.Time())

            quat = np.array([tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z])
            trans = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
            rotm = quatToMat(quat)

            target = np.array([self.target.pose.position.x, self.target.pose.position.y, self.target.pose.position.z])

            target = np.matmul(rotm, target) + trans

            self.target_lock_.release()
            
        return target

    def get_trajectory(self):
        traj = []
        
        if (self.traj_lock_.acquire(blocking=True)):
            drone_frame_id = self.get_parameter("drone_frame_id").value
            world_frame_id = self.get_parameter("world_frame_id").value
            tf = self.tf_buffer.lookup_transform(drone_frame_id, world_frame_id, rclpy.time.Time())

            quat = np.array([tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z])
            trans = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
            rotm = quatToMat(quat)

            for i in range(len(self.traj.poses)):
                p = np.array([self.traj.poses[i].pose.position.x, self.traj.poses[i].pose.position.y, self.traj.poses[i].pose.position.z])
                p = np.matmul(rotm,p) + trans
                traj.append(p)

            self.traj_lock_.release()
            
        return traj

    def get_action_status(self):
        current_action, action_status = "None", "Idle"

        if self.action_status_lock_.acquire(blocking=True):
            current_action = self.current_action
            action_status = self.action_status
            self.action_status_lock_.release()

        return current_action, action_status

    def get_cable_ids(self):       
        self.pl_lock_.acquire(blocking=True)
        ids = [self.powerline_tuples_[i][0] for i in range(len(self.powerline_tuples_))]
        self.pl_lock_.release()

        return ids

    def send_takeoff_action_request(self, takeoff_height):
        print("Sending takeoff action request with height: "+str(takeoff_height))

        if self.action_status_lock_.acquire(blocking=True):
            self.current_action = "Takeoff"
            self.action_status = "Waiting for reply"
            self.action_status_lock_.release()

        goal_msg = Takeoff.Goal()
        goal_msg.target_altitude = takeoff_height
        
        self.takeoff_client.wait_for_server()
        
        self.future = self.takeoff_client.send_goal_async(goal_msg)
        self.future.add_done_callback(self.goal_response_callback)

    def send_landing_action_request(self):
        print("Sending landing action request")

        if self.action_status_lock_.acquire(blocking=True):
            self.current_action = "Landing"
            self.action_status = "Waiting for reply"
            self.action_status_lock_.release()

        goal_msg = Landing.Goal()
        
        self.landing_client.wait_for_server()
        
        self.future = self.landing_client.send_goal_async(goal_msg)
        self.future.add_done_callback(self.goal_response_callback)
        self.action_client = self.landing_client

    def send_fly_to_position_action_request(self, target_pose):
        print("Sending fly-to-position action request with target pose:", target_pose)

        if self.action_status_lock_.acquire(blocking=True):
            self.current_action = "FlyToPosition"
            self.action_status = "Waiting for reply"
            self.action_status_lock_.release()

        goal_msg = FlyToPosition.Goal()
        goal_msg.target_pose = target_pose
        
        self.fly_to_position_client.wait_for_server()
        
        self.future = self.fly_to_position_client.send_goal_async(goal_msg)
        self.future.add_done_callback(self.goal_response_callback)
        self.action_client = self.fly_to_position_client

    def send_fly_under_cable_action_request(self, cable_id, target_distance):
        print("Sending fly-under-cable action request with cable id", cable_id, "and target distance", target_distance)

        if self.action_status_lock_.acquire(blocking=True):
            self.current_action = "FlyUnderCable"
            self.action_status = "Waiting for reply"
            self.action_status_lock_.release()

        goal_msg = FlyUnderCable.Goal()
        goal_msg.target_cable_id = cable_id
        goal_msg.target_cable_distance = target_distance
        
        self.fly_under_cable_client.wait_for_server()
        
        self.future = self.fly_under_cable_client.send_goal_async(goal_msg)
        self.future.add_done_callback(self.goal_response_callback)
        self.action_client = self.fly_under_cable_client

    def send_cable_landing_action_request(self, target_cable_id):
        print("Sending cable landing action request with target cable id:", target_cable_id)

        if self.action_status_lock_.acquire(blocking=True):
            self.current_action = "CableLanding"
            self.action_status = "Waiting for reply"
            self.action_status_lock_.release()

        goal_msg = CableLanding.Goal()
        goal_msg.target_cable_id = target_cable_id
        
        self.cable_landing_client.wait_for_server()
        
        self.future = self.cable_landing_client.send_goal_async(goal_msg)
        self.future.add_done_callback(self.goal_response_callback)
        self.action_client = self.cable_landing_client

    def send_cable_takeoff_action_request(self, target_cable_distance):
        print("Sending cable takeoff action request with target cable distance:", target_cable_distance)

        if self.action_status_lock_.acquire(blocking=True):
            self.current_action = "CableTakeoff"
            self.action_status = "Waiting for reply"
            self.action_status_lock_.release()

        goal_msg = CableTakeoff.Goal()
        goal_msg.target_cable_distance = target_cable_distance
        
        self.cable_takeoff_client.wait_for_server()
        
        self.future = self.cable_takeoff_client.send_goal_async(goal_msg)
        self.future.add_done_callback(self.goal_response_callback)
        self.action_client = self.cable_takeoff_client

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        self.action_status_lock_.acquire(blocking=True)
        if not self.goal_handle.accepted:
            self.action_status = "Cancelled"
            self.action_status_lock_.release()
            return

        self.action_status = "Executing"
        self.action_status_lock_.release()

        self.future = self.goal_handle.get_result_async()
        self.future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        res = future.result()
        
        self.action_status_lock_.acquire(blocking=True)
        if res is None or not res.result.success:
            self.action_status = "Cancelled"
            self.action_status_lock_.release()
            return

        self.action_status = "Success"
        self.action_status_lock_.release()

    def cancel_action(self):
        self.action_client._cancel_goal(self.goal_handle)

class IIIGui():
    def __init__(self):
        self.node = IIIGuiNode()

        self.end = False

        self.node_thread = Thread(target=self.spin_node)
        self.node_thread.start()

        sleep(1)

        self.config = yaml.safe_load(open(self.node.config_file_path,"r").read())

        self.config_node_keys = []
        for key in self.config.keys():
            key = str(key)
            if (key == "/**" or key == "tf" or key == "iii_gui"):
                continue

            self.config_node_keys.append(key)

        self.takeoff_height = 0
        self.target_pose = PoseStamped()
        self.target_cable_id = 0
        self.target_cable_distance = 1.

        self.current_action = "None"
        self.action_status = "Idle"

        self.root = tkinter.Tk()
        self.root.title("III-Drone GUI")
        # self.root.geometry("200x200")

        self.action_options_window = None
        self.params_options_window = None

        self.pl_viz_frame = tkinter.Frame(self.root, bg="white")
        self.action_ctrl_frame = tkinter.Frame(self.root, bg="white")
        self.status_monitor_frame = tkinter.Frame(self.root, bg="white")
        self.action_options_frame = None
        self.params_frame = tkinter.Frame(self.root, bg="white")
        self.params_options_frame = None

        self.label_viz = tkinter.Label(self.pl_viz_frame)

        self.actions = [
            "Takeoff",
            "Landing",
            "FlyToPosition",
            "FlyUnderCable",
            "CableLanding",
            "CableTakeoff"
        ]

        self.action_stringvar = tkinter.StringVar(self.root)
        self.action_stringvar.set(self.actions[0])
        self.action_optionmenu = tkinter.OptionMenu(
            self.action_ctrl_frame,
            self.action_stringvar,
            *self.actions
        )

        self.execute_action_button = tkinter.Button(
            self.action_ctrl_frame,
            text="Execute action",
            command=self.execute_action
        )
        self.cancel_action_button = tkinter.Button(
            self.action_ctrl_frame,
            text="Cancel action",
            command=self.cancel_action
        )

        self.control_state_label = tkinter.Label(
            self.status_monitor_frame, 
            text="Trajectory controller state: "+self.node.get_control_state()
        )
        self.current_action_label = tkinter.Label(
            self.status_monitor_frame, 
            text=""
        )
        self.action_status_label = tkinter.Label(
            self.status_monitor_frame, 
            text=""
        )

        self.param_label = tkinter.Label(
            self.params_frame, 
            text="Param options:"
        )

        self.param_stringvar = tkinter.StringVar(self.root)
        self.param_stringvar.set(self.config_node_keys[0])
        self.param_optionmenu = tkinter.OptionMenu(
            self.params_frame,
            self.param_stringvar,
            *self.config_node_keys
        )

        self.set_params_button = tkinter.Button(
            self.params_frame,
            text="Set params",
            command=self.set_params
        )

        self.pl_viz_frame.grid()
        self.action_ctrl_frame.grid()
        self.status_monitor_frame.grid()
        self.params_frame.grid()

        self.label_viz.grid()

        self.action_optionmenu.grid()
        self.execute_action_button.grid()
        self.cancel_action_button.grid()

        self.control_state_label.grid()
        self.current_action_label.grid()
        self.action_status_label.grid()

        self.param_label.grid()
        self.param_optionmenu.grid()
        self.set_params_button.grid()

        self.set_execute_button_state()
        self.set_cancel_button_state()
        self.put_img()
        self.put_control_state()
        self.put_action_status()
        self.root.mainloop()
        self.end = True
        self.node_thread.join()


        # self.node.destroy_node()

    def set_params(self):
        self.params_options_window = Toplevel(self.root)
        self.params_options_window.title("Params options")
        self.params_options_frame = tkinter.Frame(self.params_options_window, bg="white")

        self.params_options_frame.grid()

        node_key = self.param_stringvar.get()

        params = []
        for key in self.config[node_key][node_key]["ros__parameters"].keys():
            params.append(str(key))

        param_stringvar = tkinter.StringVar(self.root)
        param_stringvar.set(params[0])
        param_optionmenu = tkinter.OptionMenu(
            self.params_options_frame,
            param_stringvar,
            *params
        )

        param_optionmenu.grid()

        value_label = tkinter.Label(
            self.params_options_frame,
            text="Parameter value:"
        )
        value_entry = tkinter.Entry(
            self.params_options_frame
        )

        value_label.grid()
        value_entry.grid()

        def on_cancel_btn_click():
            self.params_options_window.destroy()
            self.params_options_window = None
            self.params_options_frame = None

        def on_ok_btn_click():
            value = value_entry.get()
            param = param_stringvar.get()

            parameter_type = type(self.config[node_key][node_key]["ros__parameters"][param])

            success = True

            try:
                value = parameter_type(value)
            except ValueError:
                success = False

            if success:
                node_name = "/" + node_key + "/" + node_key

                bashCommand = str("ros2 param set " + str(node_name) + " " + str(param) + " " + str(value))
                process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
                output, error = process.communicate()

                self.node.get_logger().info(str(output))
                self.node.get_logger().info(str(error))

            self.params_options_window.destroy()
            self.params_options_window = None
            self.params_options_frame = None

        ok_btn = tkinter.Button(
            self.params_options_frame,
            text="OK",
            command=on_ok_btn_click
        )

        cancel_btn = tkinter.Button(
            self.params_options_frame,
            text="Cancel",
            command=on_cancel_btn_click
        )

        ok_btn.grid()
        cancel_btn.grid()

    def execute_action(self):
        self.action_options_window = Toplevel(self.root)
        self.action_options_window.title("Action options")
        self.action_options_frame = tkinter.Frame(self.action_options_window, bg="white")

        self.action_options_frame.grid()

        action = self.action_stringvar.get()

        vcmd_numeric = (self.root.register(self.validate_numeric_and_empty),
                    '%d', '%i', '%P', '%s', '%S', '%v', '%V', '%W')

        def on_cancel_btn_click():
            self.action_options_window.destroy()
            self.action_options_window = None
            self.action_options_frame = None


        if (action == "Takeoff"):
            takeoff_label = tkinter.Label(
                self.action_options_frame,
                text="Takeoff options"
            )
            takeoff_height_label = tkinter.Label(
                self.action_options_frame,
                text="Takeoff height:"
            )
            takeoff_height_entry = tkinter.Entry(
                self.action_options_frame,
                validate="key",
                validatecommand=vcmd_numeric
            )

            def on_ok_btn_click():
                try:
                    self.takeoff_height = float(takeoff_height_entry.get())
                except ValueError:
                    takeoff_height_entry.configure(
                        bg="red"
                    )

                    return

                self.action_options_window.destroy()
                self.action_options_window = None
                self.action_options_frame = None

                self.node.send_takeoff_action_request(self.takeoff_height)

            ok_btn = tkinter.Button(
                self.action_options_frame,
                text="OK",
                command=on_ok_btn_click
            )

            cancel_btn = tkinter.Button(
                self.action_options_frame,
                text="Cancel",
                command=on_cancel_btn_click
            )

            takeoff_label.grid()
            takeoff_height_label.grid()
            takeoff_height_entry.grid()
            ok_btn.grid()
            cancel_btn.grid()

        if action == "Landing":
            landing_label = tkinter.Label(
                self.action_options_frame,
                text="Landing options"
            )

            def on_ok_btn_click():
                self.action_options_window.destroy()
                self.action_options_window = None
                self.action_options_frame = None

                self.node.send_landing_action_request()

            ok_btn = tkinter.Button(
                self.action_options_frame,
                text="OK",
                command=on_ok_btn_click
            )

            cancel_btn = tkinter.Button(
                self.action_options_frame,
                text="Cancel",
                command=on_cancel_btn_click
            )

            landing_label.grid()
            ok_btn.grid()
            cancel_btn.grid()

        elif action == "FlyToPosition":
            ftp_label = tkinter.Label(
                self.action_options_frame,
                text="FlyToPosition options"
            )
            x_label = tkinter.Label(
                self.action_options_frame,
                text="Position x:"
            )
            x_entry = tkinter.Entry(
                self.action_options_frame,
                validate="key",
                validatecommand=vcmd_numeric
            )
            y_label = tkinter.Label(
                self.action_options_frame,
                text="Position y:"
            )
            y_entry = tkinter.Entry(
                self.action_options_frame,
                validate="key",
                validatecommand=vcmd_numeric
            )
            z_label = tkinter.Label(
                self.action_options_frame,
                text="Position z:"
            )
            z_entry = tkinter.Entry(
                self.action_options_frame,
                validate="key",
                validatecommand=vcmd_numeric
            )
            yaw_label = tkinter.Label(
                self.action_options_frame,
                text="Yaw:"
            )
            yaw_entry = tkinter.Entry(
                self.action_options_frame,
                validate="key",
                validatecommand=vcmd_numeric
            )
            frame_label = tkinter.Label(
                self.action_options_frame,
                text="Target position frame:"
            )
            frames = [
                self.node.get_parameter("world_frame_id").value,
                self.node.get_parameter("drone_frame_id").value
            ]
            frame_stringvar = tkinter.StringVar(self.action_options_window)
            frame_stringvar.set(frames[0])
            frame_optionmenu = tkinter.OptionMenu(
                self.action_options_frame,
                frame_stringvar,
                *frames
            )

            def on_ok_btn_click():
                fail = False

                self.target_pose = PoseStamped()
                self.target_pose.header.frame_id = frame_stringvar.get()
                try:
                    self.target_pose.pose.position.x = float(x_entry.get())
                except ValueError:
                    x_entry.configure(
                        bg="red"
                    )

                    fail = True

                try:
                    self.target_pose.pose.position.y = float(y_entry.get())
                except ValueError:
                    y_entry.configure(
                        bg="red"
                    )

                    fail = True

                try:
                    self.target_pose.pose.position.z = float(z_entry.get())
                except ValueError:
                    z_entry.configure(
                        bg="red"
                    )

                    fail = True

                try:
                    eul = [0, 0, float(yaw_entry.get())]
                    quat = eulToQuat(eul)
                except ValueError:
                    yaw_entry.configure(
                        bg="red"
                    )

                    fail = True

                if fail:
                    return

                self.target_pose.pose.orientation.w = quat[0]
                self.target_pose.pose.orientation.x = quat[1]
                self.target_pose.pose.orientation.y = quat[2]
                self.target_pose.pose.orientation.z = quat[3]

                self.action_options_window.destroy()
                self.action_options_window = None
                self.action_options_frame = None

                self.node.send_fly_to_position_action_request(self.target_pose)

            ok_btn = tkinter.Button(
                self.action_options_frame,
                text="OK",
                command=on_ok_btn_click
            )

            cancel_btn = tkinter.Button(
                self.action_options_frame,
                text="Cancel",
                command=on_cancel_btn_click
            )

            ftp_label.grid()
            x_label.grid()
            x_entry.grid()
            y_label.grid()
            y_entry.grid()
            z_label.grid()
            z_entry.grid()
            yaw_label.grid()
            yaw_entry.grid()
            frame_label.grid()
            frame_optionmenu.grid()
            ok_btn.grid()
            cancel_btn.grid()

        elif action == "FlyUnderCable":
            fuc_label = tkinter.Label(
                self.action_options_frame,
                text="FlyUnderCable options"
            )
            id_label = tkinter.Label(
                self.action_options_frame,
                text="Cable ID:"
            )
            cable_ids = self.node.get_cable_ids()
            cable_id_stringvar = tkinter.StringVar(self.action_options_window)
            cable_id_stringvar.set(cable_ids[0])
            cable_id_optionmenu = tkinter.OptionMenu(
                self.action_options_frame,
                cable_id_stringvar,
                *cable_ids
            )
            distance_label = tkinter.Label(
                self.action_options_frame,
                text="Target cable distance:"
            )
            distance_entry = tkinter.Entry(
                self.action_options_frame,
                validate="key",
                validatecommand=vcmd_numeric
            )

            def on_ok_btn_click():
                fail = False

                try:
                    self.target_cable_distance = float(distance_entry.get())
                except ValueError:
                    distance_entry.configure(
                        bg="red"
                    )

                    fail = True

                try:
                    self.target_cable_id = int(cable_id_stringvar.get())
                except ValueError:
                    cable_id_optionmenu.configure(
                        bg="red"
                    )

                    fail = True

                if fail:
                    return

                self.action_options_window.destroy()
                self.action_options_window = None
                self.action_options_frame = None

                self.node.send_fly_under_cable_action_request(self.target_cable_id, self.target_cable_distance)

            ok_btn = tkinter.Button(
                self.action_options_frame,
                text="OK",
                command=on_ok_btn_click
            )

            cancel_btn = tkinter.Button(
                self.action_options_frame,
                text="Cancel",
                command=on_cancel_btn_click
            )

            fuc_label.grid()
            id_label.grid()
            cable_id_optionmenu.grid()
            distance_label.grid()
            distance_entry.grid()
            ok_btn.grid()
            cancel_btn.grid()

        elif action == "CableLanding":
            cl_label = tkinter.Label(
                self.action_options_frame,
                text="CableLanding options"
            )
            id_label = tkinter.Label(
                self.action_options_frame,
                text="Cable ID:"
            )
            cable_ids = self.node.get_cable_ids()
            cable_id_stringvar = tkinter.StringVar(self.action_options_window)
            cable_id_stringvar.set(cable_ids[0])
            cable_id_optionmenu = tkinter.OptionMenu(
                self.action_options_frame,
                cable_id_stringvar,
                *cable_ids
            )

            def on_ok_btn_click():
                fail = False

                try:
                    self.target_cable_id = int(cable_id_stringvar.get())
                except ValueError:
                    cable_id_optionmenu.configure(
                        bg="red"
                    )

                    fail = True

                if fail:
                    return

                self.action_options_window.destroy()
                self.action_options_window = None
                self.action_options_frame = None

                self.node.send_cable_landing_action_request(self.target_cable_id)

            ok_btn = tkinter.Button(
                self.action_options_frame,
                text="OK",
                command=on_ok_btn_click
            )

            cancel_btn = tkinter.Button(
                self.action_options_frame,
                text="Cancel",
                command=on_cancel_btn_click
            )

            cl_label.grid()
            id_label.grid()
            cable_id_optionmenu.grid()
            ok_btn.grid()
            cancel_btn.grid()

        elif action == "CableTakeoff":
            ct_label = tkinter.Label(
                self.action_options_frame,
                text="CableTakeoff options"
            )
            distance_label = tkinter.Label(
                self.action_options_frame,
                text="Target cable distance:"
            )
            distance_entry = tkinter.Entry(
                self.action_options_frame,
                validate="key",
                validatecommand=vcmd_numeric
            )

            def on_ok_btn_click():
                fail = False

                try:
                    self.target_cable_distance = float(distance_entry.get())
                except ValueError:
                    distance_entry.configure(
                        bg="red"
                    )

                    fail = True

                if fail:
                    return

                self.action_options_window.destroy()
                self.action_options_window = None
                self.action_options_frame = None

                self.node.send_cable_takeoff_action_request(self.target_cable_distance)

            ok_btn = tkinter.Button(
                self.action_options_frame,
                text="OK",
                command=on_ok_btn_click
            )

            cancel_btn = tkinter.Button(
                self.action_options_frame,
                text="Cancel",
                command=on_cancel_btn_click
            )

            ct_label.grid()
            distance_label.grid()
            distance_entry.grid()
            ok_btn.grid()
            cancel_btn.grid()
            
    def cancel_action(self):
        self.node.cancel_action()

    def validate_numeric_and_empty(self, action, index, value_if_allowed,
                       prior_value, text, validation_type, trigger_type, widget_name):
        if value_if_allowed == "\b" or value_if_allowed == "" or value_if_allowed == "-":
            return True
        try:
            float(value_if_allowed)
            return True
        except ValueError:
            return False

    def spin_node(self):
        while True:
            if self.end:
                break

            rclpy.spin_once(self.node)

    def set_execute_button_state(self):
        if self.action_status == "Idle" or self.action_status == "Cancelled" or self.action_status == "Success":
            self.execute_action_button["state"] = "normal"
        else:
            self.execute_action_button["state"] = "disabled"

        self.execute_action_button.after(25, self.set_execute_button_state)

    def set_cancel_button_state(self):
        if self.action_status == "Idle" or self.action_status == "Cancelled" or self.action_status == "Success":
            self.cancel_action_button["state"] = "disabled"
        else:
            self.cancel_action_button["state"] = "normal"

        self.cancel_action_button.after(25, self.set_cancel_button_state)

    def put_control_state(self):
        state = self.node.get_control_state()

        self.control_state_label.configure(text="Trajectory controller state: "+state)
        
        self.control_state_label.after(100, self.put_control_state)

    def put_action_status(self):
        self.current_action, self.action_status = self.node.get_action_status()

        self.current_action_label.configure(text="Current action: "+self.current_action)
        self.action_status_label.configure(text="Action status: "+self.action_status)
        
        self.action_status_label.after(100, self.put_action_status)

    def put_img(self):
        pl_tuples = []
        pl_quat = None
        self.node.pl_lock_.acquire(blocking=True)
        for tuple in self.node.powerline_tuples_:
            pl_tuples.append((tuple[0], tuple[1]))
        pl_quat = self.node.powerline_quat_
        self.node.pl_lock_.release()

        rotated_tuples = []

        rotm = quatToMat(pl_quat).transpose()

        for tuple in pl_tuples:
            point = tuple[1]
            point = np.matmul(rotm, point)
            new_tuple = (tuple[0], point)
            rotated_tuples.append(new_tuple)

        points_y = [rotated_tuples[i][1][1] for i in range(len(rotated_tuples))]
        points_z = [rotated_tuples[i][1][2] for i in range(len(rotated_tuples))]
        points_id = [rotated_tuples[i][0] for i in range(len(rotated_tuples))]

        fig, ax = plt.subplots()
        ax.scatter(points_y, points_z, linewidth=0.000001, color='green', label='Powerlines (#ID)')
        ax.scatter(0, 0, linewidth=0.000001, color='red', label='Ego', marker='X')

        for i, txt in enumerate(points_id):
            ax.annotate(txt, (points_y[i], points_z[i]))



        target = self.node.get_target()
        if target is not None and (target[0]**2 + target[1]**2 + target[2]**2)**0.5 > 0.1:
            target = np.matmul(rotm, target)
            ax.scatter(target[1], target[2], linewidth=0.000001, color='blue', label='Target', marker='X')
            ax.annotate("Target", (target[1], target[2]))


        traj = self.node.get_trajectory()
        if len(traj) > 0:
            traj_y = []
            traj_z = []
            for point in traj:
                point = np.matmul(rotm, point)
                traj_y.append(point[1])
                traj_z.append(point[2])
            ax.plot(traj_y, traj_z, color='yellow', label='Trajectory')


        plt.axis('square')

        if (len(points_y)>0):
            plt.xlim([min([min(points_y)-2,-2]), max([max(points_y)+2, 2])])


        fig.canvas.draw()

        img = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
        # img = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
        img  = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))

        # img is rgb, convert to opencv's default bgr
        img = cv.cvtColor(img,cv.COLOR_RGB2BGR)

        plt.cla()
        plt.clf()
        plt.close('all')

        if img is not None:
            img = Image.fromarray(img)
            imgtk = ImageTk.PhotoImage(image=img)
            self.label_viz.imgtk = imgtk
            self.label_viz.configure(image=imgtk)

        self.label_viz.after(100, self.put_img)



###############################################################################
# Main
###############################################################################


if __name__ == "__main__":
    rclpy.init()

    print("Starting IIIGui")
    gui = IIIGui()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rclpy.shutdown()

    
