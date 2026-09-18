#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
follow_waypoints_odom.py

Segue uma rota de waypoints (x, y) usando exclusivamente odometria de roda
(dead reckoning) como feedback de posição — sem Nav2, sem AMCL, sem mapa.

Como a odometria de roda começa zerada (0,0,0) a cada bringup do
odom_node_4wd/odom_node, os waypoints aqui são relativos à pose inicial do
robô no momento em que este script é ligado (e não a um frame 'map' fixo).

Uso (rota em "8", curvas amplas — padrão):
  python3 follow_waypoints_odom.py --ros-args \
      -p route_shape:=figure8 \
      -p figure8_width:=2.0 \
      -p figure8_height:=1.2 \
      -p laps:=2

Uso (lista explícita de waypoints, com parada e giro em cada ponto):
  python3 follow_waypoints_odom.py --ros-args \
      -p route_shape:=waypoints \
      -p waypoints_str:="1.5 0.0; 1.5 1.5; 0.0 1.5; 0.0 0.0" \
      -p laps:=1
"""

import math
import time
import threading
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time as RosTime
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


# -----------------------------
# Funções auxiliares
# -----------------------------

def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def ang_diff(a: float, b: float) -> float:
    """Diferença angular (a - b) normalizada em (-pi, pi]."""
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


def stamp_to_seconds(stamp) -> float:
    try:
        return stamp.sec + stamp.nanosec / 1e9
    except Exception:
        return RosTime.from_msg(stamp).nanoseconds / 1e9


def parse_waypoints(waypoints_str: str) -> List[Tuple[float, float]]:
    pts = []
    for tok in waypoints_str.split(";"):
        tok = tok.strip()
        if not tok:
            continue
        xs, ys = tok.split()
        pts.append((float(xs), float(ys)))
    return pts


def generate_figure8(width: float, height: float, points_per_lap: int) -> List[Tuple[float, float]]:
    """
    Gera uma lemniscata de Gerono (curva em "8") centrada na pose inicial do robô:
        x(t) = width  * sin(t)
        y(t) = height * sin(t) * cos(t)
    para t em [0, 2π). O robô começa e cruza sempre por (0,0) — sua pose inicial.
    Curva contínua (sem cantos vivos); 'width'/'height' controlam o quão "aberto"
    fica o 8 — height menor que width dá curvas mais amplas e suaves.
    """
    n = max(8, int(points_per_lap))
    pts = []
    for i in range(n):
        t = 2.0 * math.pi * i / n
        x = width * math.sin(t)
        y = height * math.sin(t) * math.cos(t)
        pts.append((x, y))
    return pts


# -----------------------------
# Classes auxiliares
# -----------------------------

class OdomWatch:
    """Assina um tópico nav_msgs/Odometry e mantém o último estado (x, y, yaw, v, w)."""

    def __init__(self, node: Node, topic: str, qos: Optional[QoSProfile] = None):
        self.node = node
        self.has_odom = False
        self.t = self.x = self.y = self.yaw = self.vx = self.wz = 0.0
        if qos is None:
            qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                              history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self._sub = self.node.create_subscription(Odometry, topic, self._cb, qos)

    def _cb(self, msg: Odometry):
        self.t = stamp_to_seconds(msg.header.stamp)
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.vx = msg.twist.twist.linear.x
        self.wz = msg.twist.twist.angular.z
        if not self.has_odom:
            self.node.get_logger().info(f"OdomWatch[{self._sub.topic_name}]: primeiro pacote recebido.")
        self.has_odom = True


class RateLimiter:
    """Limita aceleração linear (m/s²) e angular (rad/s²) entre comandos sucessivos."""

    def __init__(self, lin_acc=0.5, ang_acc=1.5, hz=30):
        self.lin_acc = float(lin_acc)
        self.ang_acc = float(ang_acc)
        self.dt = 1.0 / float(hz)
        self.v_prev = 0.0
        self.w_prev = 0.0

    def step(self, v_des, w_des):
        dv_max = self.lin_acc * self.dt
        dw_max = self.ang_acc * self.dt
        v_cmd = self._clamp(v_des, self.v_prev - dv_max, self.v_prev + dv_max)
        w_cmd = self._clamp(w_des, self.w_prev - dw_max, self.w_prev + dw_max)
        self.v_prev, self.w_prev = v_cmd, w_cmd
        return v_cmd, w_cmd

    @staticmethod
    def _clamp(x, lo, hi):
        return min(max(x, lo), hi)


class PIController:
    """Controlador PI com anti-windup (clamp no termo integral)."""

    def __init__(self, kp, ki, dt, min_output, max_output, integral_clamp):
        self.kp = kp
        self.ki = ki
        self.dt = dt
        self.min_output = min_output
        self.max_output = max_output
        self.integral_clamp = integral_clamp
        self.integral_error = 0.0

    def step(self, error):
        self.integral_error += error * self.dt
        self.integral_error = max(-self.integral_clamp, min(self.integral_clamp, self.integral_error))
        output = self.kp * error + self.ki * self.integral_error
        return max(self.min_output, min(self.max_output, output))

    def reset(self):
        self.integral_error = 0.0


# -----------------------------
# Nó principal
# -----------------------------

class WaypointOdomFollowerNode(Node):
    def __init__(self):
        super().__init__("waypoint_odom_follower_node")

        # --- Tópicos ---
        self.declare_parameter("wheel_odom_topic", "/wheel/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("hz", 30)

        # --- Rota ---
        self.declare_parameter("route_shape", "figure8")  # "figure8" | "waypoints"
        self.declare_parameter("waypoints_str", "")
        self.declare_parameter("laps", 1)

        # --- Rota em "8" (curvas amplas, seguimento contínuo) ---
        self.declare_parameter("figure8_width", 2.0)   # meia-largura do 8 (m)
        self.declare_parameter("figure8_height", 1.2)  # meia-altura de cada laço (m) — menor que width = curvas mais amplas
        self.declare_parameter("figure8_points_per_lap", 72)
        self.declare_parameter("wp_switch_tol", 0.35)  # raio p/ trocar de waypoint sem parar (modo contínuo)

        # --- Controle linear/angular ---
        self.declare_parameter("lin_speed", 0.6)
        self.declare_parameter("ang_speed", 1.0)
        self.declare_parameter("yaw_kp", 1.6)
        self.declare_parameter("yaw_ki", 0.15)
        self.declare_parameter("yaw_integral_clamp", 0.6)
        self.declare_parameter("yaw_w_limit", 0.8)
        self.declare_parameter("lin_acc", 0.6)
        self.declare_parameter("ang_acc", 2.0)
        self.declare_parameter("min_linear", 0.05)
        self.declare_parameter("ang_stop_deg", 35.0)
        self.declare_parameter("slow_radius", 0.6)
        self.declare_parameter("pos_tol", 0.07)
        self.declare_parameter("ang_tol_deg", 8.0)
        self.declare_parameter("arrived_pause", 0.25)

        # --- Giros explícitos antes de cada segmento ---
        self.declare_parameter("explicit_turns", True)
        self.declare_parameter("turn_pre_sleep", 0.5)
        self.declare_parameter("turn_post_sleep", 0.5)

        # --- Timeout de odometria ---
        self.declare_parameter("odom_timeout", 5.0)

        # Leitura
        self.wheel_topic = self.get_parameter("wheel_odom_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.hz = int(self.get_parameter("hz").value)

        self.route_shape = str(self.get_parameter("route_shape").value).strip().lower()
        self.waypoints_str = str(self.get_parameter("waypoints_str").value).strip()
        self.laps = int(self.get_parameter("laps").value)

        self.figure8_width = float(self.get_parameter("figure8_width").value)
        self.figure8_height = float(self.get_parameter("figure8_height").value)
        self.figure8_points_per_lap = int(self.get_parameter("figure8_points_per_lap").value)
        self.wp_switch_tol = float(self.get_parameter("wp_switch_tol").value)

        self.lin_speed = float(self.get_parameter("lin_speed").value)
        self.ang_speed = float(self.get_parameter("ang_speed").value)
        self.yaw_kp = float(self.get_parameter("yaw_kp").value)
        self.yaw_ki = float(self.get_parameter("yaw_ki").value)
        self.yaw_integral_clamp = float(self.get_parameter("yaw_integral_clamp").value)
        self.yaw_w_limit = float(self.get_parameter("yaw_w_limit").value)
        self.lin_acc = float(self.get_parameter("lin_acc").value)
        self.ang_acc = float(self.get_parameter("ang_acc").value)
        self.min_linear = float(self.get_parameter("min_linear").value)
        self.ang_stop_deg = float(self.get_parameter("ang_stop_deg").value)
        self.slow_radius = float(self.get_parameter("slow_radius").value)
        self.pos_tol = float(self.get_parameter("pos_tol").value)
        self.ang_tol_deg = float(self.get_parameter("ang_tol_deg").value)
        self.arrived_pause = float(self.get_parameter("arrived_pause").value)

        self.explicit_turns = bool(self.get_parameter("explicit_turns").value)
        self.turn_pre_sleep = float(self.get_parameter("turn_pre_sleep").value)
        self.turn_post_sleep = float(self.get_parameter("turn_post_sleep").value)

        self.odom_timeout = float(self.get_parameter("odom_timeout").value)

        if self.route_shape == "figure8":
            base_route = generate_figure8(self.figure8_width, self.figure8_height,
                                           self.figure8_points_per_lap)
        elif self.route_shape == "waypoints":
            if not self.waypoints_str:
                raise ValueError(
                    "route_shape='waypoints' mas 'waypoints_str' está vazio. Informe algo como "
                    "\"1.5 0.0; 1.5 1.5; 0.0 1.5\" (relativo à pose inicial do robô)."
                )
            base_route = parse_waypoints(self.waypoints_str)
            if not base_route:
                raise ValueError("Nenhum waypoint válido em 'waypoints_str'.")
        else:
            raise ValueError(f"route_shape inválido: '{self.route_shape}' (use 'figure8' ou 'waypoints').")

        self.route: List[Tuple[float, float]] = base_route * max(1, self.laps)

        odom_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                               history=QoSHistoryPolicy.KEEP_LAST, depth=10)

        self.pub_cmd = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.odom = OdomWatch(self, self.wheel_topic, odom_qos)

        self.get_logger().info(
            f"Rota '{self.route_shape}' carregada com {len(self.route)} pontos (laps={self.laps}) | "
            f"odom='{self.wheel_topic}' cmd_vel='{self.cmd_vel_topic}'"
        )
        if self.route_shape != "figure8":
            for i, (gx, gy) in enumerate(self.route, 1):
                self.get_logger().info(f"  WP{i}: ({gx:.3f}, {gy:.3f})")

        self._stop_flag = False
        self.control_thread = threading.Thread(target=self._run_logic, daemon=True)
        self.control_thread.start()

    # -------------------------
    # Utilidades
    # -------------------------
    def _pub_twist(self, v: float, w: float):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub_cmd.publish(msg)

    def _wait_for_odom(self):
        t0 = time.time()
        while rclpy.ok() and not self.odom.has_odom:
            if time.time() - t0 > self.odom_timeout:
                self.get_logger().warn(f"Sem odometria em '{self.wheel_topic}' após {self.odom_timeout:.1f}s. Continuando…")
                break
            time.sleep(0.02)

    def _stop(self, limiter: RateLimiter, hz: int, t: float = 0.25):
        r = self.create_rate(hz)
        steps = max(1, int(t * hz))
        for _ in range(steps):
            if not rclpy.ok() or self._stop_flag:
                break
            v_cmd, w_cmd = limiter.step(0.0, 0.0)
            self._pub_twist(v_cmd, w_cmd)
            r.sleep()

    # -------------------------
    # Controle: ir até (gx, gy)
    # -------------------------
    def _go_to_xy(self, gx: float, gy: float, hz: int, seg_name: str):
        self._wait_for_odom()

        r = self.create_rate(hz)
        limiter = RateLimiter(lin_acc=self.lin_acc, ang_acc=self.ang_acc, hz=hz)
        yaw_ctrl = PIController(kp=self.yaw_kp, ki=self.yaw_ki, dt=1.0 / hz,
                                 min_output=-self.yaw_w_limit, max_output=self.yaw_w_limit,
                                 integral_clamp=self.yaw_integral_clamp)
        ang_stop = math.radians(self.ang_stop_deg)

        while rclpy.ok() and not self._stop_flag:
            dx = gx - self.odom.x
            dy = gy - self.odom.y
            dist = math.hypot(dx, dy)
            if dist <= self.pos_tol:
                break

            heading_des = math.atan2(dy, dx)
            e_yaw = ang_diff(heading_des, self.odom.yaw)
            w_des = yaw_ctrl.step(e_yaw)

            v_scale_dist = min(1.0, dist / self.slow_radius)
            if abs(e_yaw) > ang_stop:
                v_des = 0.0
            else:
                v_des = self.lin_speed * v_scale_dist * max(0.2, math.cos(abs(e_yaw)))
                v_des = max(self.min_linear, v_des)

            v_cmd, w_cmd = limiter.step(v_des, w_des)
            self._pub_twist(v_cmd, w_cmd)
            self.get_logger().debug(
                f"[{seg_name}] pos=({self.odom.x:.2f},{self.odom.y:.2f}) yaw={math.degrees(self.odom.yaw):.1f}° "
                f"dist={dist:.2f} e_yaw={math.degrees(e_yaw):.1f}° v={v_cmd:.2f} w={w_cmd:.2f}"
            )
            r.sleep()

        self._stop(limiter, hz=hz, t=self.arrived_pause)

    # -------------------------
    # Controle: girar até yaw alvo
    # -------------------------
    def _rotate_to_yaw(self, yaw_target: float, hz: int, seg_name: str,
                        pre_sleep: float = 0.0, post_sleep: float = 0.0):
        self._wait_for_odom()

        if pre_sleep > 0.0:
            time.sleep(pre_sleep)

        limiter = RateLimiter(lin_acc=999.0, ang_acc=self.ang_acc, hz=hz)
        yaw_ctrl = PIController(kp=self.yaw_kp, ki=self.yaw_ki, dt=1.0 / hz,
                                 min_output=-self.ang_speed, max_output=self.ang_speed,
                                 integral_clamp=self.yaw_integral_clamp)
        r = self.create_rate(hz)

        while rclpy.ok() and not self._stop_flag:
            e_yaw = ang_diff(yaw_target, self.odom.yaw)
            if abs(e_yaw) <= math.radians(self.ang_tol_deg):
                break
            w_des = yaw_ctrl.step(e_yaw)
            v_cmd, w_cmd = limiter.step(0.0, w_des)
            self._pub_twist(v_cmd, w_cmd)
            r.sleep()

        self._stop(limiter, hz=hz)
        if post_sleep > 0.0:
            time.sleep(post_sleep)

    # -------------------------
    # Controle contínuo: percorre uma lista densa de pontos sem parar/girar
    # em cada um (curvas amplas, tipo o "8") — só freia de verdade no último.
    # -------------------------
    def _follow_smooth_path(self, points: List[Tuple[float, float]]):
        self._wait_for_odom()

        r = self.create_rate(self.hz)
        limiter = RateLimiter(lin_acc=self.lin_acc, ang_acc=self.ang_acc, hz=self.hz)
        yaw_ctrl = PIController(kp=self.yaw_kp, ki=self.yaw_ki, dt=1.0 / self.hz,
                                 min_output=-self.yaw_w_limit, max_output=self.yaw_w_limit,
                                 integral_clamp=self.yaw_integral_clamp)
        ang_stop = math.radians(max(self.ang_stop_deg, 60.0))  # mais tolerante: pontos são próximos entre si

        n = len(points)
        idx = 0
        # pula waypoints já "alcançados" no instante zero (ex.: o primeiro ponto do 8 é a própria origem)
        while idx < n and math.hypot(points[idx][0] - self.odom.x, points[idx][1] - self.odom.y) <= self.wp_switch_tol:
            idx += 1

        while rclpy.ok() and not self._stop_flag and idx < n:
            gx, gy = points[idx]
            is_last = (idx == n - 1)
            tol = self.pos_tol if is_last else self.wp_switch_tol

            dx = gx - self.odom.x
            dy = gy - self.odom.y
            dist = math.hypot(dx, dy)
            if dist <= tol:
                idx += 1
                continue

            heading_des = math.atan2(dy, dx)
            e_yaw = ang_diff(heading_des, self.odom.yaw)
            w_des = yaw_ctrl.step(e_yaw)

            if abs(e_yaw) > ang_stop:
                v_des = 0.0
            else:
                v_des = self.lin_speed * max(0.3, math.cos(abs(e_yaw)))
                v_des = max(self.min_linear, v_des)

            v_cmd, w_cmd = limiter.step(v_des, w_des)
            self._pub_twist(v_cmd, w_cmd)
            self.get_logger().debug(
                f"[smooth wp{idx+1}/{n}] pos=({self.odom.x:.2f},{self.odom.y:.2f}) "
                f"dist={dist:.2f} e_yaw={math.degrees(e_yaw):.1f}° v={v_cmd:.2f} w={w_cmd:.2f}"
            )
            r.sleep()

        self._stop(limiter, hz=self.hz, t=self.arrived_pause)

    # -------------------------
    # Loop principal da rota
    # -------------------------
    def _follow_route(self):
        if self.route_shape == "figure8":
            self.get_logger().info(f"Seguindo o 8 ({len(self.route)} pontos) em modo contínuo…")
            self._follow_smooth_path(self.route)
            return

        self.get_logger().info(f"Seguindo {len(self.route)} waypoints por odometria de roda…")
        prev = None
        for i, (gx, gy) in enumerate(self.route, 1):
            if not rclpy.ok() or self._stop_flag:
                return

            if self.explicit_turns and prev is not None:
                heading_des = math.atan2(gy - prev[1], gx - prev[0])
                self.get_logger().info(f"Girando para WP{i}: heading {math.degrees(heading_des):.1f}°")
                self._rotate_to_yaw(heading_des, hz=max(self.hz, 40), seg_name=f"turn_wp{i}",
                                     pre_sleep=self.turn_pre_sleep, post_sleep=self.turn_post_sleep)

            self.get_logger().info(f"Indo para WP{i}/{len(self.route)}: ({gx:.3f}, {gy:.3f})")
            self._go_to_xy(gx, gy, hz=self.hz, seg_name=f"wp{i}")
            prev = (gx, gy)

    def _run_logic(self):
        try:
            self._follow_route()
            self.get_logger().info("✔ Rota concluída.")
        except Exception as e:
            self.get_logger().error(f"Erro seguindo a rota: {e}")
        finally:
            self._stop_flag = True
            if rclpy.ok():
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointOdomFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
