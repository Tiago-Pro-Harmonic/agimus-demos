#!/usr/bin/env python3
"""Save the URDF from /robot_description topic to a file."""

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import String


class UrdfSaver(Node):
    def __init__(self, output_path):
        super().__init__("urdf_saver")
        self.output_path = output_path

        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.sub = self.create_subscription(
            String, "/robot_description", self.callback, qos
        )
        self.get_logger().info(f"Waiting for /robot_description...")

    def callback(self, msg):
        with open(self.output_path, "w") as f:
            f.write(msg.data)
        self.get_logger().info(f"URDF saved to {self.output_path}")
        raise SystemExit


def main():
    output_path = sys.argv[1] if len(sys.argv) > 1 else "robot.urdf"
    rclpy.init()
    node = UrdfSaver(output_path)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
