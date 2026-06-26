#!/usr/bin/env python3

import json
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from nav2_msgs.msg import SpeedLimit


class LocalizationSpeedSupervisor(Node):
    def __init__(self):
        super().__init__("localization_speed_supervisor")

        self.declare_parameter("robot_health_topic", "/robot_health")
        self.declare_parameter("speed_limit_topic", "/speed_limit")

        self.declare_parameter("loc_max_var_x", 0.4)
        self.declare_parameter("loc_max_var_y", 0.4)
        self.declare_parameter("loc_max_var_yaw", 0.15)

        self.declare_parameter("speed_ok", 100.0)
        self.declare_parameter("speed_warn", 70.0)
        self.declare_parameter("speed_degraded", 40.0)
        self.declare_parameter("speed_critical", 20.0)
        self.declare_parameter("speed_lost", 0.0)

        self.robot_health_topic = self.get_parameter("robot_health_topic").value
        self.speed_limit_topic = self.get_parameter("speed_limit_topic").value

        self.loc_max_var_x = self.get_parameter("loc_max_var_x").value
        self.loc_max_var_y = self.get_parameter("loc_max_var_y").value
        self.loc_max_var_yaw = self.get_parameter("loc_max_var_yaw").value

        self.speed_by_level = {
            "OK": self.get_parameter("speed_ok").value,
            "WARN": self.get_parameter("speed_warn").value,
            "DEGRADED": self.get_parameter("speed_degraded").value,
            "CRITICAL": self.get_parameter("speed_critical").value,
            "LOST": self.get_parameter("speed_lost").value,
            "INITIALIZING": self.get_parameter("speed_degraded").value,
        }

        self.pub_speed = self.create_publisher(
            SpeedLimit,
            self.speed_limit_topic,
            10,
        )

        self.sub_health = self.create_subscription(
            String,
            self.robot_health_topic,
            self._on_robot_health,
            10,
        )

        self.last_level = None
        self.last_speed = None

        self.get_logger().info(
            f"[localization_speed_supervisor] up | "
            f"sub={self.robot_health_topic} | pub={self.speed_limit_topic}"
        )

    def _classify_ratio(self, ratio: float) -> str:
        if ratio <= 0.50:
            return "OK"
        if ratio <= 0.75:
            return "WARN"
        if ratio <= 0.90:
            return "DEGRADED"
        if ratio <= 1.00:
            return "CRITICAL"
        return "LOST"

    def _worst_level(self, levels):
        priority = {
            "OK": 0,
            "WARN": 1,
            "DEGRADED": 2,
            "CRITICAL": 3,
            "LOST": 4,
            "INITIALIZING": 2,
        }
        return max(levels, key=lambda level: priority[level])

    def _classify_localization(self, loc: dict) -> str:
        status = loc.get("status", "INITIALIZING")

        if status == "LOST":
            return "LOST"

        var_x = loc.get("var_x")
        var_y = loc.get("var_y")
        var_yaw = loc.get("var_yaw")

        if var_x is None or var_y is None or var_yaw is None:
            return "INITIALIZING"

        level_x = self._classify_ratio(var_x / self.loc_max_var_x)
        level_y = self._classify_ratio(var_y / self.loc_max_var_y)
        level_yaw = self._classify_ratio(var_yaw / self.loc_max_var_yaw)

        return self._worst_level([level_x, level_y, level_yaw])

    def _publish_speed_limit(self, level: str):
        speed = float(self.speed_by_level.get(level, 40.0))

        msg = SpeedLimit()
        msg.percentage = True
        msg.speed_limit = speed

        self.pub_speed.publish(msg)

        if level != self.last_level or speed != self.last_speed:
            self.get_logger().warn(
                f"[localization_speed_supervisor] "
                f"localization_level={level} | speed_limit={speed:.1f}%"
            )
            self.last_level = level
            self.last_speed = speed

    def _on_robot_health(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"Failed to parse /robot_health: {e}")
            return

        loc = data.get("localization", {})
        level = self._classify_localization(loc)

        self._publish_speed_limit(level)


def main():
    rclpy.init()
    node = LocalizationSpeedSupervisor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()