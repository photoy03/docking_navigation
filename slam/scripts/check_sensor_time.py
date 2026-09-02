#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class SensorTimeChecker(Node):

    def __init__(self):
        super().__init__('sensor_time_checker')

        self.scan_time = None
        self.odom_time = None

        self.create_subscription(
            LaserScan,
            '/scan_filtered',
            self.scan_callback,
            10
        )

        self.create_subscription(
            Odometry,
            '/robot1/odom',
            self.odom_callback,
            10
        )

        self.create_timer(
            1.0,
            self.print_status
        )

    def scan_callback(self, msg):

        self.scan_time = (
            msg.header.stamp.sec
            + msg.header.stamp.nanosec * 1e-9
        )

    def odom_callback(self, msg):

        self.odom_time = (
            msg.header.stamp.sec
            + msg.header.stamp.nanosec * 1e-9
        )

    def print_status(self):

        now = self.get_clock().now().nanoseconds * 1e-9

        print("\n==========================")

        if self.scan_time is not None:
            print(
                f"SCAN stamp : {self.scan_time:.6f}"
            )
            print(
                f"SCAN age   : "
                f"{now - self.scan_time:.3f} s"
            )

        if self.odom_time is not None:
            print(
                f"ODOM stamp : {self.odom_time:.6f}"
            )
            print(
                f"ODOM age   : "
                f"{now - self.odom_time:.3f} s"
            )

        if (
            self.scan_time is not None
            and self.odom_time is not None
        ):
            print(
                f"ODOM-SCAN  : "
                f"{self.odom_time - self.scan_time:.3f} s"
            )


def main():

    rclpy.init()

    node = SensorTimeChecker()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()