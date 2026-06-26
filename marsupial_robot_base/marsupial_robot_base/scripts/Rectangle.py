#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math, csv, os, time, threading
from typing import Optional

# ROS 2
import rclpy
from rclpy.node import Node
from rclpy.time import Time as RosTime
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# pip install tf-transformations


# -----------------------------
# Funções Auxiliares
# -----------------------------

def euler_from_quaternion_xyzw(x, y, z, w):
    """Converte (x,y,z,w) → (roll, pitch, yaw), em rad."""
    # Fórmulas padrão XYZ (roll-pitch-yaw)
    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2.0, sinp)  # uso de 90° se fora do range
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def yaw_from_quat(q):
    """Extrai yaw de geometry_msgs/Quaternion."""
    return euler_from_quaternion_xyzw(q.x, q.y, q.z, q.w)[2]
    



def ang_diff(a, b):
    """Diferença angular (a - b) em (-pi, pi]."""
    return (a - b + math.pi) % (2*math.pi) - math.pi

def stamp_to_seconds(stamp):
    """Converte builtin_interfaces/Time -> float (segundos)."""
    try:
        return stamp.sec + stamp.nanosec / 1e9
    except Exception:
        # Fallback robusto
        return RosTime.from_msg(stamp).nanoseconds / 1e9

# -----------------------------
# Classes Auxiliares
# -----------------------------

class OdomWatch:
    """Inscreve em um tópico de Odometry e mantém o último estado."""
    def __init__(self, node: Node, topic: str, qos: Optional[QoSProfile] = None):
        self.node = node
        self.has_odom = False
        self.t = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        if qos is None:
            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10
            )

        self._sub = self.node.create_subscription(Odometry, topic, self.cb, qos)

    def cb(self, msg: Odometry):
        self.t   = stamp_to_seconds(msg.header.stamp)
        self.x   = msg.pose.pose.position.x
        self.y   = msg.pose.pose.position.y
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.vx  = msg.twist.twist.linear.x
        self.vy  = msg.twist.twist.linear.y
        self.wz  = msg.twist.twist.angular.z
        if not self.has_odom:
            self.node.get_logger().info("OdomWatch: primeiro pacote recebido.")
        self.has_odom = True

class RateLimiter:
    """Limita aceleração (m/s²) e aceleração angular (rad/s²) de v e w."""
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

class DataLogger:
    """Registra amostras em CSV."""
    def __init__(self, node: Node, out_csv="rect_route_log.csv"):
        self.node = node
        self.rows = []
        self.out_csv = out_csv

    def add(self, t, seg_name, pose_source: OdomWatch, wheel: Optional[OdomWatch], v_cmd, w_cmd):
        row = {
            "t": t,
            "segment": seg_name,
            "v_cmd": v_cmd,
            "w_cmd": w_cmd,
            "pose_x": pose_source.x,
            "pose_y": pose_source.y,
            "pose_yaw": pose_source.yaw,
            "pose_vx": pose_source.vx,
            "pose_wz": pose_source.wz
        }
        if wheel and wheel.has_odom:
            row.update({
                "wheel_x": wheel.x,
                "wheel_y": wheel.y,
                "wheel_yaw": wheel.yaw,
                "wheel_vx": wheel.vx,
                "wheel_wz": wheel.wz
            })
        self.rows.append(row)

    def save(self):
        if not self.rows:
            self.node.get_logger().warn("DataLogger: nada para salvar.")
            return
        os.makedirs(os.path.dirname(self.out_csv) or ".", exist_ok=True)
        try:
            with open(self.out_csv, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=list(self.rows[0].keys()))
                w.writeheader()
                w.writerows(self.rows)
            self.node.get_logger().info(f"✔ Log salvo em: {self.out_csv}")
        except Exception as e:
            self.node.get_logger().error(f"Falha ao salvar CSV '{self.out_csv}': {e}")

# -----------------------------
# Nó principal
# -----------------------------

class RectangleDriverNode(Node):
    def __init__(self):
        super().__init__('rectangle_driver_node')

        # Parâmetros
        self.declare_parameter("pose_topic", "zed/zed_node/odom")
        self.declare_parameter("wheel_odom_topic", "/wheel/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        self.declare_parameter("width", 1.5)
        self.declare_parameter("height", 4.0)
        self.declare_parameter("laps", 1)
        self.declare_parameter("clockwise", False)

        self.declare_parameter("lin_speed", 0.2)     # m/s
        self.declare_parameter("ang_speed", 0.5)     # rad/s
        self.declare_parameter("pos_tol", 0.1)      # m
        self.declare_parameter("ang_tol_deg", 10.0)   # graus

        self.declare_parameter("yaw_kp", 1.0)
        self.declare_parameter("yaw_w_limit", 0.2)   # rad/s

        self.declare_parameter("hz", 30)
        self.declare_parameter("out_csv", "rect_route_log.csv")

        # Leitura
        self.pose_topic   = self.get_parameter("pose_topic").value
        self.wheel_topic  = self.get_parameter("wheel_odom_topic").value
        self.cmd_vel_topic= self.get_parameter("cmd_vel_topic").value

        self.width        = float(self.get_parameter("width").value)
        self.height       = float(self.get_parameter("height").value)
        self.laps         = int(self.get_parameter("laps").value)
        self.clockwise    = bool(self.get_parameter("clockwise").value)

        self.lin_speed    = float(self.get_parameter("lin_speed").value)
        self.ang_speed    = float(self.get_parameter("ang_speed").value)
        self.pos_tol      = float(self.get_parameter("pos_tol").value)
        self.ang_tol_deg  = float(self.get_parameter("ang_tol_deg").value)

        self.yaw_kp       = float(self.get_parameter("yaw_kp").value)
        self.yaw_w_limit  = float(self.get_parameter("yaw_w_limit").value)

        self.hz           = int(self.get_parameter("hz").value)
        self.out_csv      = self.get_parameter("out_csv").value

        # QoS “best effort” para odometria (muito comum)
        odom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Pub/Sub
        self.pub_cmd = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.pose_feedback  = OdomWatch(self, self.pose_topic, odom_qos)
        self.wheel_feedback = OdomWatch(self, self.wheel_topic, odom_qos) if self.wheel_topic else None

        # Logger
        self.logger_instance = DataLogger(self, self.out_csv)

        self.get_logger().info(
            f"Nó iniciado. Pose: '{self.pose_topic}', Wheel: '{self.wheel_topic}', "
            f"cmd_vel: '{self.cmd_vel_topic}'."
        )

        # Dá um tempinho para começar a receber odom
        time.sleep(0.5)

        # Thread do controle
        self.control_thread = threading.Thread(target=self.run_rectangle_logic, daemon=True)
        self.control_thread.start()

    # -------------------------
    # Loop de controle
    # -------------------------
    def run_rectangle_logic(self):
        try:
            self.drive_rectangle()
            self.logger_instance.save()
            self.get_logger().info("✔ Trajeto retangular concluído. Encerrando nó.")
        except Exception as e:
            self.get_logger().error(f"Erro na lógica do retângulo: {e}")
        finally:
            rclpy.shutdown()

    # -------------------------
    # Utilidades de controle
    # -------------------------
    def pub_twist(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub_cmd.publish(msg)

    # Substitua a função _rate pelo uso do create_rate do Node
    def _rate(self, hz):
        return self.create_rate(hz)

    def stop(self, limiter: RateLimiter, hz=30, t=0.4, seg_name="stop"):
        r = self._rate(hz)
        steps = max(1, int(t * hz))
        for _ in range(steps):
            if not rclpy.ok():
                break
            v_cmd, w_cmd = limiter.step(0.0, 0.0)
            self.pub_twist(v_cmd, w_cmd)
            self.logger_instance.add(self.get_clock().now().nanoseconds / 1e9,
                                     seg_name, self.pose_feedback, self.wheel_feedback, v_cmd, w_cmd)
            r.sleep()

    def drive_distance(self, dist_m, speed, yaw_hold, yaw_kp, yaw_w_limit, pos_tol, hz, seg_name):
        # Espera primeira odom
        t0 = time.time()
        while rclpy.ok() and not self.pose_feedback.has_odom:
            if time.time() - t0 > 5.0:
                self.get_logger().warn("Sem odom ainda (5s). Continuando mesmo assim…")
                break
            time.sleep(0.01)

        x0, y0 = self.pose_feedback.x, self.pose_feedback.y
        yaw_ref = self.pose_feedback.yaw
        r = self._rate(hz)
        limiter = RateLimiter(hz=hz)

        v_nom = abs(speed) * (1 if dist_m >= 0 else -1)
        total = abs(dist_m)

        while rclpy.ok():
            d = math.hypot(self.pose_feedback.x - x0, self.pose_feedback.y - y0)
            if d >= total - pos_tol:
                break

            remain = max(0.0, total - d)
            v_des = v_nom * max(0.2, min(1.0, remain / 0.5))

            w_des = 0.0
            if yaw_hold:
                e_yaw = ang_diff(yaw_ref, self.pose_feedback.yaw)
                w_des = max(-yaw_w_limit, min(yaw_w_limit, yaw_kp * e_yaw))

            v_cmd, w_cmd = limiter.step(v_des, w_des)
            self.pub_twist(v_cmd, w_cmd)
            self.logger_instance.add(self.get_clock().now().nanoseconds / 1e9,
                                     seg_name, self.pose_feedback, self.wheel_feedback, v_cmd, w_cmd)
            r.sleep()

        self.stop(limiter, hz=hz, seg_name=f"{seg_name}_stop")

    def rotate_in_place(self, delta_deg, ang_speed, ang_acc, ang_tol_deg, hz, seg_name):
        t0 = time.time()
        while rclpy.ok() and not self.pose_feedback.has_odom:
            if time.time() - t0 > 5.0:
                self.get_logger().warn("Sem odom (5s) para rotação. Continuando…")
                break
            time.sleep(0.01)

        yaw0 = self.pose_feedback.yaw
        target = math.radians(delta_deg)
        w_nom = abs(ang_speed) * (1.0 if delta_deg >= 0 else -1.0)

        limiter = RateLimiter(lin_acc=999.0, ang_acc=ang_acc, hz=hz)
        r = self._rate(hz)

        while rclpy.ok():
            d_yaw = ang_diff(self.pose_feedback.yaw, yaw0)
            remain = abs(target) - abs(d_yaw)
            if remain <= math.radians(ang_tol_deg):
                break

            scale = max(0.25, min(1.0, remain / math.radians(10.0)))
            w_des = w_nom * scale
            v_cmd, w_cmd = limiter.step(0.0, w_des)
            self.pub_twist(v_cmd, w_cmd)
            self.logger_instance.add(self.get_clock().now().nanoseconds / 1e9,
                                     seg_name, self.pose_feedback, self.wheel_feedback, v_cmd, w_cmd)
            r.sleep()

        self.stop(limiter, hz=hz, seg_name=f"{seg_name}_stop")

    # -------------------------
    # Plano do retângulo
    # -------------------------
    def drive_rectangle(self):
        # sentido do giro
        turn = -90.0 if self.clockwise else 90.0

        # 1 volta → 4 retas + 4 giros
        plan = [
            ("straight", self.width),  ("turn", turn),
            ("straight", self.height), ("turn", turn),
            ("straight", self.width),  ("turn", turn),
            ("straight", self.height), ("turn", turn),
        ]

        self.get_logger().info(
            f"Trajeto alvo: {self.width:.2f} x {self.height:.2f} m, "
            f"voltas={self.laps}, clockwise={self.clockwise}"
        )

        for lap in range(1, self.laps + 1):
            self.get_logger().info(f"▶ Iniciando volta {lap}/{self.laps}")
            for i, (kind, val) in enumerate(plan, start=1):
                if not rclpy.ok():
                    return
                if kind == "straight":
                    self.drive_distance(
                        dist_m=val,
                        speed=self.lin_speed,
                        yaw_hold=True,
                        yaw_kp=self.yaw_kp,
                        yaw_w_limit=self.yaw_w_limit,
                        pos_tol=self.pos_tol,
                        hz=self.hz,
                        seg_name=f"lap{lap}_straight_{i}"
                    )
                else:
                    self.rotate_in_place(
                        delta_deg=val,
                        ang_speed=self.ang_speed,
                        ang_acc=2.0,
                        ang_tol_deg=self.ang_tol_deg,
                        hz=max(self.hz, 40),
                        seg_name=f"lap{lap}_turn_{i}"
                    )
            self.get_logger().info(f"✔ Volta {lap} concluída.")

# -----------------------------
# main
# -----------------------------

def main(args=None):
    rclpy.init(args=args)
    node = RectangleDriverNode()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()

if __name__ == "__main__":
    main()

