#!/usr/bin/env python3
"""Record synchronized 4WD performance data to a durable run directory."""

import csv
import json
import math
import os
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Imu
from std_msgs.msg import String


WHEELS = ("FL", "RL", "FR", "RR")


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PerformanceLogger(Node):
    def __init__(self):
        super().__init__("performance_logger")
        self.declare_parameter("run_name", "terrain_4wd")
        self.declare_parameter(
            "output_root",
            str(Path.home() / "marsupial_test_results"),
        )
        self.declare_parameter("sample_hz", 50.0)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("wheel_odom_topic", "/wheel/odom")
        self.declare_parameter("reference_odom_topic", "/zed/zed_node/odom")
        self.declare_parameter("imu_topic", "/camera/camera/imu")
        self.declare_parameter("telemetry_topic", "/drive/telemetry")
        self.declare_parameter("drive_status_topic", "/ui/drive_status")
        self.declare_parameter("event_topic", "/performance_test/event")

        run_name = str(self.get_parameter("run_name").value)
        output_root = Path(
            os.path.expanduser(str(self.get_parameter("output_root").value))
        )
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        safe_name = "".join(
            char if char.isalnum() or char in "-_" else "_"
            for char in run_name
        )
        self.run_dir = output_root / f"{timestamp}_{safe_name}"
        self.run_dir.mkdir(parents=True, exist_ok=False)

        self.start_monotonic = time.monotonic()
        self.start_wall = datetime.now(timezone.utc)
        self.rows_written = 0
        self.latest = self._initial_state()
        self.previous_telemetry = None
        self.reference_received = False
        self.imu_received = False

        self.fields = self._fieldnames()
        self.samples_handle = (self.run_dir / "samples.csv").open(
            "w",
            newline="",
            encoding="utf-8",
        )
        self.samples_writer = csv.DictWriter(
            self.samples_handle,
            fieldnames=self.fields,
        )
        self.samples_writer.writeheader()

        self.events_handle = (self.run_dir / "events.csv").open(
            "w",
            newline="",
            encoding="utf-8",
        )
        self.events_fields = [
            "t_s",
            "event",
            "name",
            "segment_index",
            "duration_s",
            "linear_x",
            "angular_z",
            "payload_json",
        ]
        self.events_writer = csv.DictWriter(
            self.events_handle,
            fieldnames=self.events_fields,
        )
        self.events_writer.writeheader()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            durability=DurabilityPolicy.VOLATILE,
        )
        reliable_qos = QoSProfile(depth=20)

        self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            self._on_cmd,
            qos,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("wheel_odom_topic").value),
            self._on_wheel_odom,
            qos,
        )

        reference_topic = str(
            self.get_parameter("reference_odom_topic").value
        )
        if reference_topic:
            self.create_subscription(
                Odometry,
                reference_topic,
                self._on_reference_odom,
                qos,
            )

        imu_topic = str(self.get_parameter("imu_topic").value)
        if imu_topic:
            self.create_subscription(
                Imu,
                imu_topic,
                self._on_imu,
                qos,
            )

        self.create_subscription(
            String,
            str(self.get_parameter("telemetry_topic").value),
            self._on_telemetry,
            qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("drive_status_topic").value),
            self._on_drive_status,
            reliable_qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("event_topic").value),
            self._on_event,
            reliable_qos,
        )

        sample_hz = float(self.get_parameter("sample_hz").value)
        self.timer = self.create_timer(
            1.0 / max(1.0, sample_hz),
            self._sample,
        )
        self._write_metadata(complete=False)
        self.get_logger().info(f"Gravando ensaio em: {self.run_dir}")

    @staticmethod
    def _initial_state():
        state = {
            "segment": "",
            "cmd_v": 0.0,
            "cmd_w": 0.0,
            "wheel_x": math.nan,
            "wheel_y": math.nan,
            "wheel_yaw": math.nan,
            "wheel_v": math.nan,
            "wheel_w": math.nan,
            "ref_x": math.nan,
            "ref_y": math.nan,
            "ref_yaw": math.nan,
            "ref_v": math.nan,
            "ref_w": math.nan,
            "imu_wz": math.nan,
            "imu_ax": math.nan,
            "imu_ay": math.nan,
            "telemetry_sample_s": math.nan,
            "drive_status_json": "",
        }
        for wheel in WHEELS:
            state[f"{wheel}_pos_turns"] = math.nan
            state[f"{wheel}_vel_turns_s"] = math.nan
            state[f"{wheel}_cmd_turns_s"] = 0.0
        return state

    @staticmethod
    def _fieldnames():
        fields = [
            "t_s",
            "wall_time_utc",
            "segment",
            "cmd_v",
            "cmd_w",
            "wheel_x",
            "wheel_y",
            "wheel_yaw",
            "wheel_v",
            "wheel_w",
            "ref_available",
            "ref_x",
            "ref_y",
            "ref_yaw",
            "ref_v",
            "ref_w",
            "imu_available",
            "imu_wz",
            "imu_ax",
            "imu_ay",
            "telemetry_sample_s",
            "telemetry_age_s",
        ]
        for wheel in WHEELS:
            fields.extend(
                [
                    f"{wheel}_pos_turns",
                    f"{wheel}_vel_turns_s",
                    f"{wheel}_cmd_turns_s",
                ]
            )
        fields.append("drive_status_json")
        return fields

    def _now_s(self):
        return time.monotonic() - self.start_monotonic

    def _on_cmd(self, msg: Twist):
        self.latest["cmd_v"] = float(msg.linear.x)
        self.latest["cmd_w"] = float(msg.angular.z)

    def _store_odom(self, prefix: str, msg: Odometry):
        self.latest[f"{prefix}_x"] = float(msg.pose.pose.position.x)
        self.latest[f"{prefix}_y"] = float(msg.pose.pose.position.y)
        self.latest[f"{prefix}_yaw"] = yaw_from_quaternion(
            msg.pose.pose.orientation
        )
        self.latest[f"{prefix}_v"] = float(msg.twist.twist.linear.x)
        self.latest[f"{prefix}_w"] = float(msg.twist.twist.angular.z)

    def _on_wheel_odom(self, msg: Odometry):
        self._store_odom("wheel", msg)

    def _on_reference_odom(self, msg: Odometry):
        self.reference_received = True
        self._store_odom("ref", msg)

    def _on_imu(self, msg: Imu):
        self.imu_received = True
        self.latest["imu_wz"] = float(msg.angular_velocity.z)
        self.latest["imu_ax"] = float(msg.linear_acceleration.x)
        self.latest["imu_ay"] = float(msg.linear_acceleration.y)

    def _on_telemetry(self, msg: String):
        received_s = self._now_s()
        try:
            payload = json.loads(msg.data)
            sample_ns = int(payload.get("sample_monotonic_ns", 0))
            sample_s = sample_ns * 1e-9 - self.start_monotonic
            positions = payload.get("positions_turns", {})
            commands = payload.get("commands_turns_s", {})

            previous = self.previous_telemetry
            for wheel in WHEELS:
                value = positions.get(wheel)
                position = math.nan if value is None else float(value)
                self.latest[f"{wheel}_pos_turns"] = position
                self.latest[f"{wheel}_cmd_turns_s"] = float(
                    commands.get(wheel, 0.0)
                )
                velocity = math.nan
                if previous is not None and math.isfinite(position):
                    previous_position = previous["positions"].get(wheel)
                    dt = sample_s - previous["sample_s"]
                    if (
                        previous_position is not None
                        and math.isfinite(previous_position)
                        and dt > 1e-4
                    ):
                        velocity = (position - previous_position) / dt
                self.latest[f"{wheel}_vel_turns_s"] = velocity

            self.latest["telemetry_sample_s"] = sample_s
            self.previous_telemetry = {
                "sample_s": sample_s,
                "received_s": received_s,
                "positions": {
                    wheel: self.latest[f"{wheel}_pos_turns"]
                    for wheel in WHEELS
                },
            }
        except Exception as exc:
            self.get_logger().warn(f"Telemetria inválida: {exc}")

    def _on_drive_status(self, msg: String):
        self.latest["drive_status_json"] = msg.data

    def _on_event(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            payload = {"event": "invalid", "name": "", "raw": msg.data}

        event = str(payload.get("event", ""))
        name = str(payload.get("name", ""))
        if event == "segment_start":
            self.latest["segment"] = name
        elif event in ("profile_complete", "test_complete"):
            self.latest["segment"] = event

        row = {
            "t_s": self._now_s(),
            "event": event,
            "name": name,
            "segment_index": payload.get("segment_index", ""),
            "duration_s": payload.get("duration_s", ""),
            "linear_x": payload.get("linear_x", ""),
            "angular_z": payload.get("angular_z", ""),
            "payload_json": json.dumps(payload, separators=(",", ":")),
        }
        self.events_writer.writerow(row)
        self.events_handle.flush()

    def _sample(self):
        now_s = self._now_s()
        telemetry_sample_s = self.latest["telemetry_sample_s"]
        telemetry_age = (
            now_s - telemetry_sample_s
            if math.isfinite(telemetry_sample_s)
            else math.nan
        )
        row = {
            "t_s": now_s,
            "wall_time_utc": datetime.now(timezone.utc).isoformat(),
            "ref_available": int(self.reference_received),
            "imu_available": int(self.imu_received),
            "telemetry_age_s": telemetry_age,
        }
        row.update(self.latest)
        self.samples_writer.writerow(row)
        self.rows_written += 1
        if self.rows_written % 100 == 0:
            self.samples_handle.flush()

    def _write_metadata(self, complete: bool):
        metadata = {
            "schema_version": 1,
            "run_dir": str(self.run_dir),
            "started_utc": self.start_wall.isoformat(),
            "ended_utc": (
                datetime.now(timezone.utc).isoformat() if complete else None
            ),
            "complete": complete,
            "rows_written": self.rows_written,
            "reference_received": self.reference_received,
            "imu_received": self.imu_received,
            "parameters": {
                name: self.get_parameter(name).value
                for name in (
                    "run_name",
                    "sample_hz",
                    "cmd_vel_topic",
                    "wheel_odom_topic",
                    "reference_odom_topic",
                    "imu_topic",
                    "telemetry_topic",
                    "drive_status_topic",
                    "event_topic",
                )
            },
        }
        path = self.run_dir / "metadata.json"
        path.write_text(
            json.dumps(metadata, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )

    def destroy_node(self):
        self.samples_handle.flush()
        self.events_handle.flush()
        self._write_metadata(complete=True)
        self.samples_handle.close()
        self.events_handle.close()
        if rclpy.ok():
            self.get_logger().info(
                f"Ensaio salvo: {self.rows_written} amostras em {self.run_dir}"
            )
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PerformanceLogger()
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
