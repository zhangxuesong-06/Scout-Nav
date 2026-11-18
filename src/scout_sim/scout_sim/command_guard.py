"""Simulation-only command watchdog. Never publishes on the real base topic."""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CommandGuard(Node):
    def __init__(self):
        super().__init__('command_guard', namespace='scout_sim')
        self.last_received = None
        self.command = Twist()
        self.output = self.create_publisher(Twist, '/scout_sim/drive_cmd', 10)
        self.create_subscription(Twist, '/scout_sim/cmd_vel', self.receive, 10)
        # Wall time is intentional: pause /clock or stop publishing => stop on resume.
        self.create_timer(0.05, self.tick, clock=rclpy.clock.Clock())

    def receive(self, msg):
        if not all(math.isfinite(v) for v in (msg.linear.x, msg.angular.z)):
            self.last_received = None
            return
        self.command = Twist()
        self.command.linear.x = max(-0.3, min(0.3, msg.linear.x))
        self.command.angular.z = max(-0.6, min(0.6, msg.angular.z))
        self.last_received = time.monotonic()

    def tick(self):
        fresh = self.last_received is not None and time.monotonic() - self.last_received < 0.5
        self.output.publish(self.command if fresh else Twist())


def main():
    rclpy.init()
    node = CommandGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.output.publish(Twist())
            node.destroy_node()
            rclpy.shutdown()
