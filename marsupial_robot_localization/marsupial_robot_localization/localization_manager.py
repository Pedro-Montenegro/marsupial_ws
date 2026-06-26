#!/usr/bin/env python3
import os
import math
import time
from dataclasses import dataclass
from collections import deque
from pathlib import Path
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Quaternion
from std_srvs.srv import Empty
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


def quat_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def yaw_from_quat(q: Quaternion) -> float:
    return math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))


def ensure_dir(path: str):
    d = os.path.dirname(path)
    if d and not os.path.exists(d):
        os.makedirs(d, exist_ok=True)


def read_simple_yaml_pose(path: str) -> Optional[dict]:
    path = os.path.expanduser(path)
    if not os.path.exists(path):
        return None

    data = {}
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            if "#" in line:
                line = line.split("#", 1)[0].strip()
            if ":" not in line:
                continue
            k, v = line.split(":", 1)
            k = k.strip()
            v = v.strip()
            try:
                data[k] = float(v)
            except ValueError:
                data[k] = v

    if "x" in data and "y" in data and "yaw" in data:
        return data
    return None


def write_simple_yaml_pose(path: str, x: float, y: float, yaw: float,
                           var_x: float, var_y: float, var_yaw: float,
                           stamp_unix: float):
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
    mode: str = "boot"

    cmd_vel_topic: str = "cmd_vel"
    initialpose_topic: str = "initialpose"
    amcl_pose_topic: str = "amcl_pose"
    reinit_service: str = "reinitialize_global_localization"

    map_frame: str = "map"

    last_pose_path: str = str(Path.home() / ".marsupial" / "last_pose.yaml")

    start_delay_s: float = 2.5
    after_initialpose_pause_s: float = 0.3
    after_reinit_pause_s: float = 0.4

    try_checkpoint_first: bool = True

    checkpoint_var_x: float = 0.50
    checkpoint_var_y: float = 0.50
    checkpoint_var_yaw: float = 0.30

    do_spin: bool = True
    spin_wz: float = 0.45
    spins: int = 2
    spin_pause_s: float = 0.4

    do_fb: bool = True
    forward_vx: float = 0.2
    backward_vx: float = -0.2
    move_time_s: float = 2.5
    move_pause_s: float = 0.3
    cycles_fb: int = 2

    window_s: float = 10.0
    stable_s: float = 1.5
    max_var_x: float = 0.4
    max_var_y: float = 0.4
    max_var_yaw: float = 0.15
    timeout_stable_s: float = 10.0

    timeout_wait_amcl_pose_s: float = 10.0

    save_checkpoint: bool = True
    debug_log_period_s: float = 1.0


class LocalizationManager(Node):
    def __init__(self):
        super().__init__("localization_manager")

        self.callback_group = ReentrantCallbackGroup()

        self.declare_parameter("mode", Params.mode)

        self.declare_parameter("cmd_vel_topic", Params.cmd_vel_topic)
        self.declare_parameter("initialpose_topic", Params.initialpose_topic)
        self.declare_parameter("amcl_pose_topic", Params.amcl_pose_topic)
        self.declare_parameter("reinit_service", Params.reinit_service)
        self.declare_parameter("map_frame", Params.map_frame)

        self.declare_parameter("last_pose_path", Params.last_pose_path)

        self.declare_parameter("start_delay_s", Params.start_delay_s)
        self.declare_parameter("after_initialpose_pause_s", Params.after_initialpose_pause_s)
        self.declare_parameter("after_reinit_pause_s", Params.after_reinit_pause_s)

        self.declare_parameter("try_checkpoint_first", Params.try_checkpoint_first)

        self.declare_parameter("checkpoint_var_x", Params.checkpoint_var_x)
        self.declare_parameter("checkpoint_var_y", Params.checkpoint_var_y)
        self.declare_parameter("checkpoint_var_yaw", Params.checkpoint_var_yaw)

        self.declare_parameter("do_spin", Params.do_spin)
        self.declare_parameter("spin_wz", Params.spin_wz)
        self.declare_parameter("spins", Params.spins)
        self.declare_parameter("spin_pause_s", Params.spin_pause_s)

        self.declare_parameter("do_fb", Params.do_fb)
        self.declare_parameter("forward_vx", Params.forward_vx)
        self.declare_parameter("backward_vx", Params.backward_vx)
        self.declare_parameter("move_time_s", Params.move_time_s)
        self.declare_parameter("move_pause_s", Params.move_pause_s)
        self.declare_parameter("cycles_fb", Params.cycles_fb)

        self.declare_parameter("window_s", Params.window_s)
        self.declare_parameter("stable_s", Params.stable_s)
        self.declare_parameter("max_var_x", Params.max_var_x)
        self.declare_parameter("max_var_y", Params.max_var_y)
        self.declare_parameter("max_var_yaw", Params.max_var_yaw)
        self.declare_parameter("timeout_stable_s", Params.timeout_stable_s)

        self.declare_parameter("timeout_wait_amcl_pose_s", Params.timeout_wait_amcl_pose_s)

        self.declare_parameter("save_checkpoint", Params.save_checkpoint)
        self.declare_parameter("debug_log_period_s", Params.debug_log_period_s)

        self.p = self._load_params()

        self.cmd_pub = self.create_publisher(
            Twist, self.p.cmd_vel_topic, 10, callback_group=self.callback_group
        )
        self.init_pub = self.create_publisher(
            PoseWithCovarianceStamped, self.p.initialpose_topic, 10, callback_group=self.callback_group
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
            callback_group=self.callback_group
        )

        self.reinit_client = self.create_client(
            Empty, self.p.reinit_service, callback_group=self.callback_group
        )

        self.amcl_hist = deque()
        self._last_amcl_msg_time = None
        self._last_debug_log_t = 0.0
        self._started = False
        self._finished = False

        self._node_start_time = self.get_clock().now().nanoseconds / 1e9

        self.get_logger().info(
            f"localization_manager iniciado. mode={self.p.mode} | start_delay_s={self.p.start_delay_s:.1f}s"
        )

        self.start_timer = self.create_timer(0.2, self._maybe_start)

    def _load_params(self) -> Params:
        p = Params()
        p.mode = str(self.get_parameter("mode").value)

        p.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        p.initialpose_topic = str(self.get_parameter("initialpose_topic").value)
        p.amcl_pose_topic = str(self.get_parameter("amcl_pose_topic").value)
        p.reinit_service = str(self.get_parameter("reinit_service").value)
        p.map_frame = str(self.get_parameter("map_frame").value)

        p.last_pose_path = str(self.get_parameter("last_pose_path").value)

        p.start_delay_s = float(self.get_parameter("start_delay_s").value)
        p.after_initialpose_pause_s = float(self.get_parameter("after_initialpose_pause_s").value)
        p.after_reinit_pause_s = float(self.get_parameter("after_reinit_pause_s").value)

        p.try_checkpoint_first = bool(self.get_parameter("try_checkpoint_first").value)

        p.checkpoint_var_x = float(self.get_parameter("checkpoint_var_x").value)
        p.checkpoint_var_y = float(self.get_parameter("checkpoint_var_y").value)
        p.checkpoint_var_yaw = float(self.get_parameter("checkpoint_var_yaw").value)

        p.do_spin = bool(self.get_parameter("do_spin").value)
        p.spin_wz = float(self.get_parameter("spin_wz").value)
        p.spins = int(self.get_parameter("spins").value)
        p.spin_pause_s = float(self.get_parameter("spin_pause_s").value)

        p.do_fb = bool(self.get_parameter("do_fb").value)
        p.forward_vx = float(self.get_parameter("forward_vx").value)
        p.backward_vx = float(self.get_parameter("backward_vx").value)
        p.move_time_s = float(self.get_parameter("move_time_s").value)
        p.move_pause_s = float(self.get_parameter("move_pause_s").value)
        p.cycles_fb = int(self.get_parameter("cycles_fb").value)

        p.window_s = float(self.get_parameter("window_s").value)
        p.stable_s = float(self.get_parameter("stable_s").value)
        p.max_var_x = float(self.get_parameter("max_var_x").value)
        p.max_var_y = float(self.get_parameter("max_var_y").value)
        p.max_var_yaw = float(self.get_parameter("max_var_yaw").value)
        p.timeout_stable_s = float(self.get_parameter("timeout_stable_s").value)

        p.timeout_wait_amcl_pose_s = float(self.get_parameter("timeout_wait_amcl_pose_s").value)

        p.save_checkpoint = bool(self.get_parameter("save_checkpoint").value)
        p.debug_log_period_s = float(self.get_parameter("debug_log_period_s").value)

        return p

    def _on_amcl(self, msg: PoseWithCovarianceStamped):
        t = self.get_clock().now().nanoseconds / 1e9
        self._last_amcl_msg_time = t

        cov = msg.pose.covariance
        var_x = float(cov[0])
        var_y = float(cov[7])
        var_yaw = float(cov[35])

        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        yaw = float(yaw_from_quat(msg.pose.pose.orientation))

        self.amcl_hist.append((t, var_x, var_y, var_yaw, x, y, yaw))

        while self.amcl_hist and (t - self.amcl_hist[0][0]) > self.p.window_s:
            self.amcl_hist.popleft()

    def _maybe_start(self):
        if self._started or self._finished:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if (now - self._node_start_time) < self.p.start_delay_s:
            return

        self._started = True
        self.destroy_timer(self.start_timer)

        self.get_logger().info("Começando rotina de localização...")
        ok = self._run_localization_flow()

        self._stop(0.3)

        if ok:
            self.get_logger().info("✅ Localização estabilizada.")
        else:
            self.get_logger().warn("⚠️ Não estabilizou dentro do timeout.")

        self._finished = True

        try:
            self.destroy_node()
        except Exception:
            pass

    def _run_localization_flow(self) -> bool:
        attempted_checkpoint = False

        if self.p.try_checkpoint_first:
            pose = read_simple_yaml_pose(self.p.last_pose_path)
            if pose is not None:
                attempted_checkpoint = True
                self.get_logger().info(f"Checkpoint encontrado: {self.p.last_pose_path}. Publicando /initialpose...")
                self._publish_initialpose(
                    x=float(pose["x"]),
                    y=float(pose["y"]),
                    yaw=float(pose["yaw"]),
                    var_x=float(pose.get("var_x", self.p.checkpoint_var_x)),
                    var_y=float(pose.get("var_y", self.p.checkpoint_var_y)),
                    var_yaw=float(pose.get("var_yaw", self.p.checkpoint_var_yaw)),
                )
                self._stop(self.p.after_initialpose_pause_s)

                if not self._wait_for_amcl_pose(self.p.timeout_wait_amcl_pose_s):
                    self.get_logger().warn("Não recebi /amcl_pose após checkpoint. Vou escalonar para global localization...")
                else:
                    self._do_motion_for_convergence()
                    ok = self._wait_stable(self.p.timeout_stable_s)
                    if ok:
                        self._maybe_save_checkpoint_from_amcl()
                        return True
                    self.get_logger().warn("Checkpoint não estabilizou. Escalonando para global localization...")

        if not attempted_checkpoint:
            self.get_logger().warn(f"Sem checkpoint ({self.p.last_pose_path}) ou try_checkpoint_first=false.")

        self.get_logger().info("Chamando global localization (reinitialize_global_localization)...")
        self._call_global_localization(timeout_s=25.0)
        self._stop(self.p.after_reinit_pause_s)

        if not self._wait_for_amcl_pose(self.p.timeout_wait_amcl_pose_s):
            self.get_logger().warn("Timeout esperando /amcl_pose aparecer.")
            return False

        self._do_motion_for_convergence()
        ok = self._wait_stable(self.p.timeout_stable_s)
        if ok:
            self._maybe_save_checkpoint_from_amcl()
            return True

        return False

    def _wait_for_amcl_pose(self, timeout_s: float) -> bool:
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < float(timeout_s):
            time.sleep(0.1)
            if self._last_amcl_msg_time is not None:
                return True
        self.get_logger().warn(f"Timeout esperando /amcl_pose aparecer ({timeout_s:.1f}s).")
        return False

    def _publish_initialpose(self, x: float, y: float, yaw: float,
                             var_x: float, var_y: float, var_yaw: float):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.p.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = quat_from_yaw(float(yaw))

        cov = [0.0] * 36
        cov[0] = float(var_x)
        cov[7] = float(var_y)
        cov[35] = float(var_yaw)
        msg.pose.covariance = cov

        self.init_pub.publish(msg)
        self.get_logger().info(
            f"/initialpose publicado: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} | "
            f"var=({var_x:.3f},{var_y:.3f},{var_yaw:.3f})"
        )

    def _call_global_localization(self, timeout_s: float = 25.0) -> bool:
        if not self.reinit_client.wait_for_service(timeout_sec=20.0):
            self.get_logger().warn("Serviço de global localization não disponível (20s).")
            return False

        future = self.reinit_client.call_async(Empty.Request())
        t0 = time.time()
        while rclpy.ok():
            time.sleep(0.2)
            if future.done():
                if future.exception() is not None:
                    self.get_logger().warn(f"Erro no serviço global localization: {future.exception()}")
                    return False
                self.get_logger().info("global localization chamado ✅")
                return True
            if time.time() - t0 > timeout_s:
                self.get_logger().warn("Timeout esperando resposta do serviço.")
                return False

    def _do_motion_for_convergence(self):
        if self.p.do_spin:
            self._do_spins()
        if self.p.do_fb:
            self._do_forward_backward()

    def _do_spins(self):
        wz = float(self.p.spin_wz)
        spins = max(0, int(self.p.spins))
        if spins <= 0 or abs(wz) < 1e-3:
            return

        self.get_logger().info(f"Fazendo {spins} giro(s) 360° (wz={wz:.2f})...")
        for i in range(spins):
            duration = (2.0 * math.pi) / max(1e-3, abs(wz))
            self.get_logger().info(f"  Spin {i+1}/{spins}: {duration:.1f}s")
            self._twist(0.0, wz, duration)
            self._stop(self.p.spin_pause_s)

    def _do_forward_backward(self):
        cycles = max(0, int(self.p.cycles_fb))
        if cycles <= 0:
            return

        self.get_logger().info(f"Fazendo vai-e-volta {cycles}x...")
        for i in range(cycles):
            self.get_logger().info(f"  Ciclo {i+1}/{cycles}: frente")
            self._twist(self.p.forward_vx, 0.0, self.p.move_time_s)
            self._stop(self.p.move_pause_s)

            self.get_logger().info(f"  Ciclo {i+1}/{cycles}: ré")
            self._twist(self.p.backward_vx, 0.0, self.p.move_time_s)
            self._stop(self.p.move_pause_s)

    def _wait_stable(self, timeout_s: float) -> bool:
        start = self.get_clock().now().nanoseconds / 1e9
        stable_start = None

        while rclpy.ok():
            now = self.get_clock().now().nanoseconds / 1e9
            if (now - start) > timeout_s:
                return False

            is_ok, last = self._is_stable_now()
            if (not is_ok) and (last is not None):
                if (now - self._last_debug_log_t) >= self.p.debug_log_period_s:
                    self._last_debug_log_t = now
                    vx, vy, vyaw = last
                    self.get_logger().info(
                        f"Cov mediana: var_x={vx:.3f} var_y={vy:.3f} var_yaw={vyaw:.3f} | "
                        f"limites=({self.p.max_var_x},{self.p.max_var_y},{self.p.max_var_yaw})"
                    )

            if is_ok:
                if stable_start is None:
                    stable_start = now
                if (now - stable_start) >= self.p.stable_s:
                    return True
            else:
                stable_start = None

            time.sleep(0.1)

    def _is_stable_now(self) -> Tuple[bool, Optional[Tuple[float, float, float]]]:
        if len(self.amcl_hist) < 5:
            return False, None

        now = self.get_clock().now().nanoseconds / 1e9
        recent = [v for v in self.amcl_hist if (now - v[0]) <= (self.p.stable_s + 1.0)]

        if len(recent) < 2:
            return False, None

        vars_x = [v[1] for v in recent]
        vars_y = [v[2] for v in recent]
        vars_yaw = [v[3] for v in recent]

        mx = sorted(vars_x)[len(vars_x)//2]
        my = sorted(vars_y)[len(vars_y)//2]
        myaw = sorted(vars_yaw)[len(vars_yaw)//2]

        ok = (mx <= self.p.max_var_x) and (my <= self.p.max_var_y) and (myaw <= self.p.max_var_yaw)
        return ok, (mx, my, myaw)

    def _maybe_save_checkpoint_from_amcl(self):
        if not self.p.save_checkpoint:
            return
        if not self.amcl_hist:
            self.get_logger().warn("Sem histórico AMCL para salvar checkpoint.")
            return

        _, var_x, var_y, var_yaw, x, y, yaw = self.amcl_hist[-1]
        stamp_unix = time.time()
        write_simple_yaml_pose(
            self.p.last_pose_path,
            x=x, y=y, yaw=yaw,
            var_x=var_x, var_y=var_y, var_yaw=var_yaw,
            stamp_unix=stamp_unix
        )
        self.get_logger().info(
            f"Checkpoint salvo em {self.p.last_pose_path}: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
        )

    def _twist(self, vx: float, wz: float, duration_s: float):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)

        end = time.time() + float(duration_s)
        while time.time() < end and rclpy.ok():
            self.cmd_pub.publish(msg)
            time.sleep(0.05)

    def _stop(self, duration_s: float = 0.2):
        msg = Twist()
        end = time.time() + float(duration_s)
        while time.time() < end and rclpy.ok():
            self.cmd_pub.publish(msg)
            time.sleep(0.05)


def main():
    rclpy.init()
    node = LocalizationManager()
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
            if rclpy.ok():
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
