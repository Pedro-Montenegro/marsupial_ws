#!/usr/bin/env python3
"""Execute a conservative, repeatable 4WD motion test profile."""

import json
import time
from dataclasses import dataclass
from typing import List

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


@dataclass
class Segment:
    name: str
    duration_s: float
    linear_x: float
    angular_z: float


class PerformanceTestRunner(Node):
    def __init__(self):
        super().__init__("performance_test_runner")
        self.declare_parameter("armed", False)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("event_topic", "/performance_test/event")
        self.declare_parameter("publish_hz", 30.0)
        self.declare_parameter("countdown_s", 5.0)
        self.declare_parameter("stop_hold_s", 1.0)
        self.declare_parameter(
            "segments",
            [
                "settle,2.0,0.0,0.0",
                "straight_slow,5.0,0.15,0.0",
                "stop_1,2.0,0.0,0.0",
                "straight_medium,5.0,0.25,0.0",
                "stop_2,2.0,0.0,0.0",
                "arc_left,6.0,0.18,0.25",
                "stop_3,2.0,0.0,0.0",
                "arc_right,6.0,0.18,-0.25",
                "stop_4,2.0,0.0,0.0",
                "pivot_left,4.0,0.0,0.20",
                "stop_5,2.0,0.0,0.0",
                "pivot_right,4.0,0.0,-0.20",
                "final_stop,2.0,0.0,0.0",
            ],
        )

        self.armed = bool(self.get_parameter("armed").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.countdown_s = float(self.get_parameter("countdown_s").value)
        self.stop_hold_s = float(self.get_parameter("stop_hold_s").value)
        self.segments = self._parse_segments(
            list(self.get_parameter("segments").value)
        )

        cmd_topic = str(self.get_parameter("cmd_vel_topic").value)
        event_topic = str(self.get_parameter("event_topic").value)
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.event_pub = self.create_publisher(String, event_topic, 10)

        self.segment_index = -1
        self.segment_started = 0.0
        self.started = time.monotonic()
        self.finished_at = None
        self.done = False
        self.timer = self.create_timer(
            1.0 / max(1.0, self.publish_hz),
            self._tick,
        )

        total = sum(segment.duration_s for segment in self.segments)
        if self.armed:
            self.get_logger().warn(
                f"TESTE ARMADO: movimento começa em {self.countdown_s:.1f}s; "
                f"duração do perfil={total:.1f}s."
            )
            self._event("countdown", name="countdown")
        else:
            self.get_logger().error(
                "Teste NÃO armado. Use armed:=true somente com área livre, "
                "operador próximo ao E-stop e sensores gravando."
            )

    @staticmethod
    def _parse_segments(raw_segments: List[str]) -> List[Segment]:
        segments = []
        for raw in raw_segments:
            parts = [part.strip() for part in str(raw).split(",")]
            if len(parts) != 4:
                raise ValueError(
                    f"Segmento inválido '{raw}'. Use nome,duração,v,w."
                )
            segment = Segment(
                name=parts[0],
                duration_s=float(parts[1]),
                linear_x=float(parts[2]),
                angular_z=float(parts[3]),
            )
            if segment.duration_s <= 0.0:
                raise ValueError(f"Duração inválida em '{raw}'.")
            segments.append(segment)
        if not segments:
            raise ValueError("O perfil precisa conter pelo menos um segmento.")
        return segments

    def _publish_cmd(self, linear_x: float, angular_z: float):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def _event(self, event: str, name: str = "", segment=None):
        payload = {
            "event": event,
            "name": name,
            "segment_index": self.segment_index,
            "monotonic_ns": time.monotonic_ns(),
            "ros_time_ns": self.get_clock().now().nanoseconds,
        }
        if segment is not None:
            payload.update(
                {
                    "duration_s": segment.duration_s,
                    "linear_x": segment.linear_x,
                    "angular_z": segment.angular_z,
                }
            )
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.event_pub.publish(msg)

    def _start_next_segment(self, now: float):
        if self.segment_index >= 0:
            previous = self.segments[self.segment_index]
            self._event("segment_end", previous.name, previous)

        self.segment_index += 1
        if self.segment_index >= len(self.segments):
            self.finished_at = now
            self._publish_cmd(0.0, 0.0)
            self._event("profile_complete", "profile_complete")
            self.get_logger().info("Perfil concluído; mantendo comando zero.")
            return

        segment = self.segments[self.segment_index]
        self.segment_started = now
        self._event("segment_start", segment.name, segment)
        self.get_logger().info(
            f"[{self.segment_index + 1}/{len(self.segments)}] "
            f"{segment.name}: {segment.duration_s:.1f}s, "
            f"v={segment.linear_x:.3f}m/s, w={segment.angular_z:.3f}rad/s"
        )

    def _tick(self):
        if self.done:
            return

        now = time.monotonic()
        if not self.armed:
            self._publish_cmd(0.0, 0.0)
            self.done = True
            return

        if self.segment_index < 0:
            self._publish_cmd(0.0, 0.0)
            if now - self.started >= self.countdown_s:
                self._start_next_segment(now)
            return

        if self.finished_at is not None:
            self._publish_cmd(0.0, 0.0)
            if now - self.finished_at >= self.stop_hold_s:
                self._event("test_complete", "test_complete")
                self.done = True
            return

        segment = self.segments[self.segment_index]
        if now - self.segment_started >= segment.duration_s:
            self._start_next_segment(now)
            if self.finished_at is not None:
                return
            segment = self.segments[self.segment_index]

        self._publish_cmd(segment.linear_x, segment.angular_z)

    def destroy_node(self):
        if rclpy.ok():
            for _ in range(3):
                self._publish_cmd(0.0, 0.0)
                time.sleep(0.02)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PerformanceTestRunner()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
