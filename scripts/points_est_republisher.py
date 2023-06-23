#!/usr/bin/python3

import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import PointCloud2

class Publisher(Node):
    def __init__(self):
        super().__init__("Republisher")

        sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.sub = self.create_subscription(
            PointCloud2,
            "/pl_mapper/points_est",
            self.callback,
            qos_profile=sub_qos
        )

        pub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.pub = self.create_publisher(
            PointCloud2,
            "/republish/points_est",
            qos_profile=pub_qos
        )

    def callback(self, msg: PointCloud2):
        msg.header.stamp = self.get_clock().now().to_msg()

        self.pub.publish(msg)


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

