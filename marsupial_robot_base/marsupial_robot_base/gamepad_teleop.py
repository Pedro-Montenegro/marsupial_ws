#!/usr/bin/env python3
"""Publish safe, smoothed 4WD velocity commands from sensor_msgs/Joy."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy


def apply_deadzone(value: float, deadzone: float) -> float:
    magnitude = abs(value)
    if magnitude <= deadzone:
        return 0.0
    scaled = (magnitude - deadzone) / max(1e-6, 1.0 - deadzone)
    return math.copysign(min(1.0, scaled), value)


def slew(current: float, target: float, max_delta: float) -> float:
    delta = max(-max_delta, min(max_delta, target - current))
    return current + delta


class GamepadTeleop(Node):
    def __init__(self):
        super().__init__("gamepad_teleop")
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("axis_linear", 1)
        self.declare_parameter("axis_angular", 0)
        self.declare_parameter("invert_linear", False)
        self.declare_parameter("invert_angular", False)
        self.declare_parameter("deadman_button", 4)
        self.declare_parameter("turbo_button", 5)
        self.declare_parameter("linear_speed", 0.20)
        self.declare_parameter("angular_speed", 0.30)
        self.declare_parameter("turbo_linear_speed", 0.30)
        self.declare_parameter("turbo_angular_speed", 0.45)
        self.declare_parameter("deadzone", 0.12)
        self.declare_parameter("max_linear_accel", 0.40)
        self.declare_parameter("max_angular_accel", 0.80)
        self.declare_parameter("joy_timeout", 0.50)
        self.declare_parameter("publish_hz", 30.0)

        self.axis_linear = int(self.get_parameter("axis_linear").value)
        self.axis_angular = int(self.get_parameter("axis_angular").value)
        self.invert_linear = bool(self.get_parameter("invert_linear").value)
        self.invert_angular = bool(self.get_parameter("invert_angular").value)
        self.deadman_button = int(
            self.get_parameter("deadman_button").value
        )
        self.turbo_button = int(self.get_parameter("turbo_button").value)
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.turbo_linear_speed = float(
            self.get_parameter("turbo_linear_speed").value
        )
        self.turbo_angular_speed = float(
            self.get_parameter("turbo_angular_speed").value
        )
        self.deadzone = float(self.get_parameter("deadzone").value)
        self.max_linear_accel = float(
            self.get_parameter("max_linear_accel").value
        )
        self.max_angular_accel = float(
            self.get_parameter("max_angular_accel").value
        )
        self.joy_timeout = float(self.get_parameter("joy_timeout").value)
        publish_hz = float(self.get_parameter("publish_hz").value)

        joy_topic = str(self.get_parameter("joy_topic").value)
        cmd_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.publisher = self.create_publisher(Twist, cmd_topic, 10)
        self.subscription = self.create_subscription(
            Joy,
            joy_topic,
            self._on_joy,
            20,
        )

        self.latest_joy = None
        self.latest_joy_monotonic = 0.0
        self.output_v = 0.0
        self.output_w = 0.0
        self.last_tick = time.monotonic()
        self.received_first_message = False
        self.reported_mapping_error = False
        self.timer = self.create_timer(
            1.0 / max(1.0, publish_hz),
            self._tick,
        )
        self.get_logger().info(
            f"Gamepad pronto: {joy_topic} → {cmd_topic}; "
            f"axis_linear={self.axis_linear}, "
            f"axis_angular={self.axis_angular}, "
            f"deadman_button={self.deadman_button}."
        )

    def _on_joy(self, msg: Joy):
        self.latest_joy = msg
        self.latest_joy_monotonic = time.monotonic()
        if not self.received_first_message:
            self.received_first_message = True
            self.get_logger().info(
                f"Controle detectado: {len(msg.axes)} eixos, "
                f"{len(msg.buttons)} botões."
            )

    @staticmethod
    def _button(msg: Joy, index: int) -> bool:
        return 0 <= index < len(msg.buttons) and bool(msg.buttons[index])

    def _mapping_valid(self, msg: Joy) -> bool:
        valid = (
            0 <= self.axis_linear < len(msg.axes)
            and 0 <= self.axis_angular < len(msg.axes)
            and (
                self.deadman_button < 0
                or self.deadman_button < len(msg.buttons)
            )
        )
        if not valid and not self.reported_mapping_error:
            self.reported_mapping_error = True
            self.get_logger().error(
                "Mapeamento incompatível com o controle: "
                f"{len(msg.axes)} eixos/{len(msg.buttons)} botões. "
                "Ajuste gamepad_teleop.yaml."
            )
        return valid

    def _publish(self, linear_x: float, angular_z: float):
        command = Twist()
        command.linear.x = float(linear_x)
        command.angular.z = float(angular_z)
        self.publisher.publish(command)

    def _tick(self):
        now = time.monotonic()
        dt = max(1e-3, min(0.2, now - self.last_tick))
        self.last_tick = now
        joy = self.latest_joy
        fresh = joy is not None and (
            now - self.latest_joy_monotonic <= self.joy_timeout
        )

        if not fresh or not self._mapping_valid(joy):
            self.output_v = 0.0
            self.output_w = 0.0
            self._publish(0.0, 0.0)
            return

        deadman_pressed = (
            self.deadman_button < 0
            or self._button(joy, self.deadman_button)
        )
        if not deadman_pressed:
            self.output_v = 0.0
            self.output_w = 0.0
            self._publish(0.0, 0.0)
            return

        turbo = self._button(joy, self.turbo_button)
        linear_limit = (
            self.turbo_linear_speed if turbo else self.linear_speed
        )
        angular_limit = (
            self.turbo_angular_speed if turbo else self.angular_speed
        )
        linear_axis = apply_deadzone(
            float(joy.axes[self.axis_linear]),
            self.deadzone,
        )
        angular_axis = apply_deadzone(
            float(joy.axes[self.axis_angular]),
            self.deadzone,
        )
        if self.invert_linear:
            linear_axis *= -1.0
        if self.invert_angular:
            angular_axis *= -1.0

        target_v = linear_axis * linear_limit
        target_w = angular_axis * angular_limit
        self.output_v = slew(
            self.output_v,
            target_v,
            self.max_linear_accel * dt,
        )
        self.output_w = slew(
            self.output_w,
            target_w,
            self.max_angular_accel * dt,
        )
        self._publish(self.output_v, self.output_w)

    def destroy_node(self):
        if rclpy.ok():
            for _ in range(3):
                self._publish(0.0, 0.0)
                time.sleep(0.02)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GamepadTeleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
