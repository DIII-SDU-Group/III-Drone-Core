#!/usr/bin/python3

import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from iii_interfaces.msg import Powerline, PowerlineDirection, ControlState

from threading import Lock


from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import numpy as np

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

filename = "/home/ffn/logged_data.txt"

with open(filename, "w") as f:
    f.write("x,y,z,p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z,p4x,p4y,p4z\n\r")

class Publisher(Node):
    def __init__(self):
        super().__init__("logger")

        sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.points = None
        self.points_lock = Lock()

        self.sub = self.create_subscription(
            Powerline,
            "/pl_mapper/powerline",
            self.callback,
            qos_profile=sub_qos
        )

    def callback(self, msg):
        try:
            tf = self.tf_buffer.lookup_transform("world", "drone", rclpy.time.Time())
        except TransformException as e:
            self.get_logger().error("Transform error: {}".format(e))
            return

        q = [tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z]

        R = quatToMat(q)


        points = []

        for pose in msg.poses:
            p = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
            p = np.matmul(R, p)
            
            point = [p[0]+tf.transform.translation.x, p[1]+tf.transform.translation.y, p[2]+tf.transform.translation.z]

            points.append(point)

        with open(filename, "a") as f:
            f.write("{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n\r".format(
                tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z,
                points[0][0] if len(points) > 0 else 0, points[0][1] if len(points) > 0 else 0, points[0][2] if len(points) > 0 else 0,
                points[1][0] if len(points) > 1 else 0, points[1][1] if len(points) > 1 else 0, points[1][2] if len(points) > 1 else 0,
                points[2][0] if len(points) > 2 else 0, points[2][1] if len(points) > 2 else 0, points[2][2] if len(points) > 2 else 0,
                points[3][0] if len(points) > 3 else 0, points[3][1] if len(points) > 3 else 0, points[3][2] if len(points) > 3 else 0
            ))




def main(args=None):
    rclpy.init(args=args)

    publisher = Publisher()

    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

