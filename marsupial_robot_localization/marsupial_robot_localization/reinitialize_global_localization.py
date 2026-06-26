#!/usr/bin/env python3
import math
import time
from dataclasses import dataclass
from collections import deque
from typing import Optional, Tuple, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty


def yaw_from_quat(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z),
        1.0 - 2.0 * (q.z * q.z)
    )


@dataclass
class Params:
    cmd_vel_topic: str = "cmd_vel"
    amcl_pose_topic: str = "amcl_pose"
    scan_topic: str = "scan"
    reinit_service: str = "reinitialize_global_localization"

    start_delay_s: float = 1.0

    max_var_x: float = 0.4
    max_var_y: float = 0.4
    max_var_yaw: float = 0.15
    stable_s: float = 1.5
    timeout_total_s: float = 45.0

    # segurança local
    robot_radius_m: float = 0.35
    safety_margin_m: float = 0.20
    front_clearance_m: float = 0.65
    rear_clearance_m: float = 0.65
    rotation_clearance_m: float = 0.55

    # setores do scan
    front_angle_deg: float = 35.0
    rear_angle_deg: float = 35.0

    # micro movimentos
    spin_wz: float = 0.5
    spin_step_deg: float = 60.0
    max_spin_steps: int = 18

    do_linear_motion: bool = True
    linear_vx: float = 0.2
    linear_step_s: float = 2.0
    max_linear_cycles: int = 4

    pause_after_motion_s: float = 0.5
    debug_log_period_s: float = 1.0


class RelocalizationRecovery(Node):
    def __init__(self):
        super().__init__("relocalization_recovery")

        self.cb_group = ReentrantCallbackGroup()

        self.declare_parameter("cmd_vel_topic", Params.cmd_vel_topic)
        self.declare_parameter("amcl_pose_topic", Params.amcl_pose_topic)
        self.declare_parameter("scan_topic", Params.scan_topic)
        self.declare_parameter("reinit_service", Params.reinit_service)

        self.declare_parameter("start_delay_s", Params.start_delay_s)

        self.declare_parameter("max_var_x", Params.max_var_x)
        self.declare_parameter("max_var_y", Params.max_var_y)
        self.declare_parameter("max_var_yaw", Params.max_var_yaw)
        self.declare_parameter("stable_s", Params.stable_s)
        self.declare_parameter("timeout_total_s", Params.timeout_total_s)

        self.declare_parameter("robot_radius_m", Params.robot_radius_m)
        self.declare_parameter("safety_margin_m", Params.safety_margin_m)
        self.declare_parameter("front_clearance_m", Params.front_clearance_m)
        self.declare_parameter("rear_clearance_m", Params.rear_clearance_m)
        self.declare_parameter("rotation_clearance_m", Params.rotation_clearance_m)

        self.declare_parameter("front_angle_deg", Params.front_angle_deg)
        self.declare_parameter("rear_angle_deg", Params.rear_angle_deg)

        self.declare_parameter("spin_wz", Params.spin_wz)
        self.declare_parameter("spin_step_deg", Params.spin_step_deg)
        self.declare_parameter("max_spin_steps", Params.max_spin_steps)

        self.declare_parameter("do_linear_motion", Params.do_linear_motion)
        self.declare_parameter("linear_vx", Params.linear_vx)
        self.declare_parameter("linear_step_s", Params.linear_step_s)
        self.declare_parameter("max_linear_cycles", Params.max_linear_cycles)

        self.declare_parameter("pause_after_motion_s", Params.pause_after_motion_s)
        self.declare_parameter("debug_log_period_s", Params.debug_log_period_s)

        self.p = self._load_params()

        self.cmd_pub = self.create_publisher(
            Twist,
            self.p.cmd_vel_topic,
            10,
            callback_group=self.cb_group,
        )

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
            callback_group=self.cb_group,
        )

        self.create_subscription(
            LaserScan,
            self.p.scan_topic,
            self._on_scan,
            10,
            callback_group=self.cb_group,
        )

        self.reinit_client = self.create_client(
            Empty,
            self.p.reinit_service,
            callback_group=self.cb_group,
        )

        self.amcl_hist = deque()
        self.last_scan: Optional[LaserScan] = None
        self.last_scan_t: Optional[float] = None
        self.last_amcl_t: Optional[float] = None

        self._started = False
        self._finished = False
        self._node_start_t = time.time()
        self._last_debug_log_t = 0.0

        self.timer = self.create_timer(0.2, self._maybe_start)

        self.get_logger().info(
            f"[relocalization_recovery] up | cmd={self.p.cmd_vel_topic} | "
            f"scan={self.p.scan_topic} | amcl={self.p.amcl_pose_topic}"
        )

    def _load_params(self) -> Params:
        p = Params()

        for field in p.__dataclass_fields__:
            setattr(p, field, self.get_parameter(field).value)

        return p

    def _on_amcl(self, msg: PoseWithCovarianceStamped):
        t = time.time()
        self.last_amcl_t = t

        cov = msg.pose.covariance
        var_x = float(cov[0])
        var_y = float(cov[7])
        var_yaw = float(cov[35])

        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        yaw = yaw_from_quat(msg.pose.pose.orientation)

        self.amcl_hist.append((t, var_x, var_y, var_yaw, x, y, yaw))

        while self.amcl_hist and (t - self.amcl_hist[0][0]) > 10.0:
            self.amcl_hist.popleft()

    def _on_scan(self, msg: LaserScan):
        self.last_scan = msg
        self.last_scan_t = time.time()

    def _maybe_start(self):
        if self._started or self._finished:
            return

        if time.time() - self._node_start_t < self.p.start_delay_s:
            return

        self._started = True
        self.destroy_timer(self.timer)

        ok = self._run_recovery()

        self._stop(0.5)

        if ok:
            self.get_logger().info("✅ Relocalização concluída com sucesso.")
        else:
            self.get_logger().warn("⚠️ Relocalização não estabilizou.")

        self._finished = True

        try:
            self.destroy_node()
        except Exception:
            pass

    def _run_recovery(self) -> bool:
        self.get_logger().warn("Iniciando recovery de relocalização.")
        self._stop(0.5)

        if not self._wait_for_scan(5.0):
            self.get_logger().error("Sem /scan. Não é seguro executar recovery automático.")
            return False

        if not self._call_global_localization():
            self.get_logger().error("Falha ao chamar global localization.")
            return False

        self._stop(0.8)

        start = time.time()

        if self._wait_stable_nonblocking():
            return True

        spin_direction = 1.0

        for step in range(int(self.p.max_spin_steps)):
            if time.time() - start > self.p.timeout_total_s:
                return False

            if self._wait_stable_nonblocking():
                return True

            if self._can_rotate():
                self.get_logger().info(
                    f"Micro giro {step + 1}/{self.p.max_spin_steps} "
                    f"dir={spin_direction:+.0f}"
                )
                self._rotate_step(direction=spin_direction)
                self._stop(self.p.pause_after_motion_s)
            else:
                self.get_logger().warn(
                    "Sem espaço seguro para girar. Tentando movimento linear mínimo, se permitido."
                )

            if self._wait_stable_nonblocking():
                return True

            if self.p.do_linear_motion and step % 3 == 2:
                self._try_small_linear_cycle()

            spin_direction *= -1.0

        return self._wait_stable_blocking(3.0)

    def _call_global_localization(self) -> bool:
        self.get_logger().info("Chamando /reinitialize_global_localization...")

        if not self.reinit_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("Serviço de global localization indisponível.")
            return False

        future = self.reinit_client.call_async(Empty.Request())
        t0 = time.time()

        while rclpy.ok() and time.time() - t0 < 10.0:
            if future.done():
                if future.exception() is not None:
                    self.get_logger().error(f"Erro no serviço: {future.exception()}")
                    return False
                self.get_logger().info("Global localization chamado.")
                return True
            time.sleep(0.1)

        self.get_logger().error("Timeout chamando global localization.")
        return False

    def _rotate_step(self, direction: float):
        angle_rad = math.radians(float(self.p.spin_step_deg))
        wz = float(self.p.spin_wz) * float(direction)
        duration = angle_rad / max(abs(wz), 1e-3)
        self._twist(vx=0.0, wz=wz, duration_s=duration)

    def _try_small_linear_cycle(self):
        if self._can_move_front():
            self.get_logger().info("Micro movimento para frente.")
            self._twist(vx=abs(self.p.linear_vx), wz=0.0, duration_s=self.p.linear_step_s)
            self._stop(self.p.pause_after_motion_s)
        else:
            self.get_logger().warn("Frente bloqueada. Não vou avançar.")

        if self._wait_stable_nonblocking():
            return

        if self._can_move_rear():
            self.get_logger().info("Micro movimento para trás.")
            self._twist(vx=-abs(self.p.linear_vx), wz=0.0, duration_s=self.p.linear_step_s)
            self._stop(self.p.pause_after_motion_s)
        else:
            self.get_logger().warn("Traseira bloqueada ou sem leitura. Não vou dar ré.")

    def _wait_for_scan(self, timeout_s: float) -> bool:
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout_s:
            if self.last_scan is not None and self.last_scan_t is not None:
                if time.time() - self.last_scan_t < 1.0:
                    return True
            time.sleep(0.1)
        return False

    def _valid_ranges_in_sector(self, center_angle: float, half_width: float) -> List[float]:
        if self.last_scan is None:
            return []

        scan = self.last_scan
        values = []

        angle = scan.angle_min

        for r in scan.ranges:
            if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                a = self._normalize_angle(angle)
                delta = abs(self._normalize_angle(a - center_angle))
                if delta <= half_width:
                    values.append(float(r))
            angle += scan.angle_increment

        return values

    def _min_range_sector(self, center_deg: float, width_deg: float) -> Optional[float]:
        center = math.radians(center_deg)
        half = math.radians(width_deg)

        vals = self._valid_ranges_in_sector(center, half)
        if not vals:
            return None
        return min(vals)

    def _min_range_all(self) -> Optional[float]:
        if self.last_scan is None:
            return None

        vals = [
            float(r)
            for r in self.last_scan.ranges
            if math.isfinite(r)
            and self.last_scan.range_min <= r <= self.last_scan.range_max
        ]

        if not vals:
            return None
        return min(vals)

    def _can_rotate(self) -> bool:
        min_r = self._min_range_all()
        if min_r is None:
            return False

        required = max(
            float(self.p.rotation_clearance_m),
            float(self.p.robot_radius_m) + float(self.p.safety_margin_m),
        )

        ok = min_r >= required

        if not ok:
            self.get_logger().warn(
                f"Rotação bloqueada: min_scan={min_r:.2f} m < required={required:.2f} m"
            )

        return ok

    def _can_move_front(self) -> bool:
        min_front = self._min_range_sector(0.0, self.p.front_angle_deg)
        if min_front is None:
            return False

        ok = min_front >= float(self.p.front_clearance_m)

        if not ok:
            self.get_logger().warn(
                f"Frente bloqueada: min_front={min_front:.2f} m "
                f"< {self.p.front_clearance_m:.2f} m"
            )

        return ok

    def _can_move_rear(self) -> bool:
        min_rear = self._min_range_sector(180.0, self.p.rear_angle_deg)
        if min_rear is None:
            return False

        ok = min_rear >= float(self.p.rear_clearance_m)

        if not ok:
            self.get_logger().warn(
                f"Traseira bloqueada: min_rear={min_rear:.2f} m "
                f"< {self.p.rear_clearance_m:.2f} m"
            )

        return ok

    def _is_stable_now(self) -> Tuple[bool, Optional[Tuple[float, float, float]]]:
        if len(self.amcl_hist) < 3:
            return False, None

        now = time.time()
        recent = [v for v in self.amcl_hist if now - v[0] <= self.p.stable_s + 0.5]

        if len(recent) < 2:
            return False, None

        vars_x = sorted([v[1] for v in recent])
        vars_y = sorted([v[2] for v in recent])
        vars_yaw = sorted([v[3] for v in recent])

        mx = vars_x[len(vars_x) // 2]
        my = vars_y[len(vars_y) // 2]
        myaw = vars_yaw[len(vars_yaw) // 2]

        ok = (
            mx <= self.p.max_var_x and
            my <= self.p.max_var_y and
            myaw <= self.p.max_var_yaw
        )

        return ok, (mx, my, myaw)

    def _wait_stable_nonblocking(self) -> bool:
        ok, last = self._is_stable_now()

        now = time.time()
        if last is not None and now - self._last_debug_log_t >= self.p.debug_log_period_s:
            self._last_debug_log_t = now
            vx, vy, vyaw = last
            self.get_logger().info(
                f"Cov mediana: x={vx:.3f}, y={vy:.3f}, yaw={vyaw:.3f} | "
                f"lim=({self.p.max_var_x:.3f}, {self.p.max_var_y:.3f}, {self.p.max_var_yaw:.3f})"
            )

        return ok

    def _wait_stable_blocking(self, timeout_s: float) -> bool:
        t0 = time.time()
        stable_start = None

        while rclpy.ok() and time.time() - t0 < timeout_s:
            ok, _ = self._is_stable_now()

            if ok:
                if stable_start is None:
                    stable_start = time.time()
                if time.time() - stable_start >= self.p.stable_s:
                    return True
            else:
                stable_start = None

            time.sleep(0.1)

        return False

    def _twist(self, vx: float, wz: float, duration_s: float):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)

        end = time.time() + float(duration_s)

        while rclpy.ok() and time.time() < end:
            self.cmd_pub.publish(msg)
            time.sleep(0.05)

    def _stop(self, duration_s: float = 0.2):
        msg = Twist()
        end = time.time() + float(duration_s)

        while rclpy.ok() and time.time() < end:
            self.cmd_pub.publish(msg)
            time.sleep(0.05)

    @staticmethod
    def _normalize_angle(a: float) -> float:
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a


def main():
    rclpy.init()
    node = RelocalizationRecovery()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            executor.remove_node(node)
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()