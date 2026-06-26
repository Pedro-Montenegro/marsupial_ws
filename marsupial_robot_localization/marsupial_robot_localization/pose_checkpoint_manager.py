#!/usr/bin/env python3
import os
import math
import time
import shutil
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry


def yaw_from_quat(q: Quaternion) -> float:
    return math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))


def ensure_dir(path: str):
    d = os.path.dirname(path)
    if d and not os.path.exists(d):
        os.makedirs(d, exist_ok=True)


def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def write_simple_yaml_pose(
    path: str,
    x: float,
    y: float,
    yaw: float,
    var_x: float,
    var_y: float,
    var_yaw: float,
    stamp_unix: float,
):
    path = os.path.expanduser(path)
    ensure_dir(path)
    with open(path, "w", encoding="utf-8") as f:
        f.write("# Marsupial last known good pose (map frame)\n")
        f.write(f"stamp_unix: {stamp_unix:.3f}\n")
        f.write(f"x: {x:.6f}\n")
        f.write(f"y: {y:.6f}\n")
        f.write(f"yaw: {yaw:.6f}\n")
        f.write(f"var_x: {var_x:.6f}\n")
        f.write(f"var_y: {var_y:.6f}\n")
        f.write(f"var_yaw: {var_yaw:.6f}\n")


@dataclass
class Params:
    amcl_pose_topic: str = "amcl_pose"
    odom_topic: str = "odometry/filtered"
    last_pose_path: str = str(Path.home() / ".marsupial" / "last_pose.yaml")

    timer_period_s: float = 1.0
    history_window_s: float = 3.0

    max_var_x: float = 0.30
    max_var_y: float = 0.30
    max_var_yaw: float = 0.15

    max_linear_speed: float = 0.03
    max_angular_speed: float = 0.05

    min_save_interval_s: float = 30.0
    min_translation_delta_m: float = 0.40
    min_yaw_delta_rad: float = 0.21  # ~12 graus

    require_robot_stopped: bool = True
    save_prev_backup: bool = True
    log_every_save: bool = True


class PoseCheckpointManager(Node):
    def __init__(self):
        super().__init__("pose_checkpoint_manager")

        self.declare_parameter("amcl_pose_topic", Params.amcl_pose_topic)
        self.declare_parameter("odom_topic", Params.odom_topic)
        self.declare_parameter("last_pose_path", Params.last_pose_path)

        self.declare_parameter("timer_period_s", Params.timer_period_s)
        self.declare_parameter("history_window_s", Params.history_window_s)

        self.declare_parameter("max_var_x", Params.max_var_x)
        self.declare_parameter("max_var_y", Params.max_var_y)
        self.declare_parameter("max_var_yaw", Params.max_var_yaw)

        self.declare_parameter("max_linear_speed", Params.max_linear_speed)
        self.declare_parameter("max_angular_speed", Params.max_angular_speed)

        self.declare_parameter("min_save_interval_s", Params.min_save_interval_s)
        self.declare_parameter("min_translation_delta_m", Params.min_translation_delta_m)
        self.declare_parameter("min_yaw_delta_rad", Params.min_yaw_delta_rad)

        self.declare_parameter("require_robot_stopped", Params.require_robot_stopped)
        self.declare_parameter("save_prev_backup", Params.save_prev_backup)
        self.declare_parameter("log_every_save", Params.log_every_save)

        self.p = self._load_params()

        qos_amcl = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            self.p.amcl_pose_topic,
            self._on_amcl,
            qos_amcl,
        )

        self.create_subscription(
            Odometry,
            self.p.odom_topic,
            self._on_odom,
            10,
        )

        self.hist = deque()
        self.last_odom_t: Optional[float] = None
        self.linear_speed: float = 0.0
        self.angular_speed: float = 0.0

        self.last_saved_pose: Optional[Tuple[float, float, float]] = None
        self.last_save_t: float = 0.0

        self.timer = self.create_timer(self.p.timer_period_s, self._on_timer)

        self.get_logger().info(
            f"pose_checkpoint_manager iniciado | amcl={self.p.amcl_pose_topic} | "
            f"odom={self.p.odom_topic} | file={self.p.last_pose_path}"
        )

    def _load_params(self) -> Params:
        p = Params()
        p.amcl_pose_topic = str(self.get_parameter("amcl_pose_topic").value)
        p.odom_topic = str(self.get_parameter("odom_topic").value)
        p.last_pose_path = str(self.get_parameter("last_pose_path").value)

        p.timer_period_s = float(self.get_parameter("timer_period_s").value)
        p.history_window_s = float(self.get_parameter("history_window_s").value)

        p.max_var_x = float(self.get_parameter("max_var_x").value)
        p.max_var_y = float(self.get_parameter("max_var_y").value)
        p.max_var_yaw = float(self.get_parameter("max_var_yaw").value)

        p.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        p.max_angular_speed = float(self.get_parameter("max_angular_speed").value)

        p.min_save_interval_s = float(self.get_parameter("min_save_interval_s").value)
        p.min_translation_delta_m = float(self.get_parameter("min_translation_delta_m").value)
        p.min_yaw_delta_rad = float(self.get_parameter("min_yaw_delta_rad").value)

        p.require_robot_stopped = bool(self.get_parameter("require_robot_stopped").value)
        p.save_prev_backup = bool(self.get_parameter("save_prev_backup").value)
        p.log_every_save = bool(self.get_parameter("log_every_save").value)
        return p

    def _on_amcl(self, msg: PoseWithCovarianceStamped):
        t = self.get_clock().now().nanoseconds / 1e9

        cov = msg.pose.covariance
        var_x = float(cov[0])
        var_y = float(cov[7])
        var_yaw = float(cov[35])

        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        yaw = float(yaw_from_quat(msg.pose.pose.orientation))

        self.hist.append((t, x, y, yaw, var_x, var_y, var_yaw))
        while self.hist and (t - self.hist[0][0]) > self.p.history_window_s:
            self.hist.popleft()

    def _on_odom(self, msg: Odometry):
        self.last_odom_t = self.get_clock().now().nanoseconds / 1e9
        self.linear_speed = abs(float(msg.twist.twist.linear.x))
        self.angular_speed = abs(float(msg.twist.twist.angular.z))

    def _robot_is_stopped(self) -> bool:
        if not self.p.require_robot_stopped:
            return True
        if self.last_odom_t is None:
            return False
        return (
            self.linear_speed <= self.p.max_linear_speed
            and self.angular_speed <= self.p.max_angular_speed
        )

    def _stable_pose_now(self) -> Optional[Tuple[float, float, float, float, float, float]]:
        if len(self.hist) < 3:
            return None

        xs = [v[1] for v in self.hist]
        ys = [v[2] for v in self.hist]
        yaws = [v[3] for v in self.hist]
        vars_x = [v[4] for v in self.hist]
        vars_y = [v[5] for v in self.hist]
        vars_yaw = [v[6] for v in self.hist]

        mx = sorted(vars_x)[len(vars_x) // 2]
        my = sorted(vars_y)[len(vars_y) // 2]
        myaw = sorted(vars_yaw)[len(vars_yaw) // 2]

        ok = (
            mx <= self.p.max_var_x
            and my <= self.p.max_var_y
            and myaw <= self.p.max_var_yaw
        )
        if not ok:
            return None

        x = xs[-1]
        y = ys[-1]
        yaw = yaws[-1]
        return (x, y, yaw, mx, my, myaw)

    def _pose_changed_enough(self, x: float, y: float, yaw: float) -> bool:
        if self.last_saved_pose is None:
            return True

        lx, ly, lyaw = self.last_saved_pose
        d = math.hypot(x - lx, y - ly)
        dyaw = abs(normalize_angle(yaw - lyaw))

        return (
            d >= self.p.min_translation_delta_m
            or dyaw >= self.p.min_yaw_delta_rad
        )

    def _save_pose(self, x: float, y: float, yaw: float, var_x: float, var_y: float, var_yaw: float):
        path = os.path.expanduser(self.p.last_pose_path)

        if self.p.save_prev_backup and os.path.exists(path):
            prev_path = path + ".prev"
            try:
                shutil.copy2(path, prev_path)
            except Exception as e:
                self.get_logger().warn(f"Falha ao criar backup {prev_path}: {e}")

        stamp_unix = time.time()
        write_simple_yaml_pose(
            path=path,
            x=x,
            y=y,
            yaw=yaw,
            var_x=var_x,
            var_y=var_y,
            var_yaw=var_yaw,
            stamp_unix=stamp_unix,
        )

        self.last_saved_pose = (x, y, yaw)
        self.last_save_t = stamp_unix

        if self.p.log_every_save:
            self.get_logger().info(
                f"Checkpoint atualizado em {path}: "
                f"x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} | "
                f"var=({var_x:.3f}, {var_y:.3f}, {var_yaw:.3f})"
            )

    def _on_timer(self):
        now = time.time()

        stable = self._stable_pose_now()
        if stable is None:
            return

        if not self._robot_is_stopped():
            return

        if (now - self.last_save_t) < self.p.min_save_interval_s:
            return

        x, y, yaw, var_x, var_y, var_yaw = stable

        if not self._pose_changed_enough(x, y, yaw):
            return

        self._save_pose(x, y, yaw, var_x, var_y, var_yaw)


def main():
    rclpy.init()
    node = PoseCheckpointManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
