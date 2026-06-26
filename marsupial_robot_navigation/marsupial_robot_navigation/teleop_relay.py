#!/usr/bin/env python3
"""
teleop_relay: repassa /ui/manual_cmd → /cmd_vel
com watchdog de segurança (zera se parar de receber)
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TeleopRelay(Node):
    def __init__(self):
        super().__init__("teleop_relay")
        self.declare_parameter("watchdog_timeout", 0.5)
        self._timeout = self.get_parameter("watchdog_timeout").value
        self._last_ts = 0.0

        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Twist, "/ui/manual_cmd", self._on_cmd, 10)
        self.create_timer(0.1, self._watchdog)

        self.get_logger().info("[teleop_relay] up")

    def _on_cmd(self, msg: Twist):
        self._last_ts = time.monotonic()
        self._pub.publish(msg)

    def _watchdog(self):
        if self._last_ts == 0.0:
            return
        if time.monotonic() - self._last_ts > self._timeout:
            self._pub.publish(Twist())  # zera
            self._last_ts = 0.0  # não fica publicando zero infinitamente

def main():
    rclpy.init()
    node = TeleopRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()