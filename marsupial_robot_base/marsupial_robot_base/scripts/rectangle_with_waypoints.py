#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math, csv, os, time, threading
from typing import Optional, List, Tuple

# ROS 2
import rclpy
from rclpy.node import Node
from rclpy.time import Time as RosTime
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# -----------------------------
# Funções Auxiliares
# -----------------------------

def euler_from_quaternion_xyzw(x, y, z, w):
    """Converte (x,y,z,w) → (roll, pitch, yaw), em rad (convenção RPY)."""
    # roll (x)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # pitch (y)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    # yaw (z)
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

    def add(self, t, seg_name, pose_source: OdomWatch, wheel: Optional[OdomWatch], v_cmd, w_cmd, goal_xy: Optional[Tuple[float,float]] = None):
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
        if goal_xy is not None:
            row.update({"goal_x": goal_xy[0], "goal_y": goal_xy[1]})
            row.update({"err_xy": math.hypot(goal_xy[0]-pose_source.x, goal_xy[1]-pose_source.y)})
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
        with open(self.out_csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(self.rows[0].keys()))
            w.writeheader()
            w.writerows(self.rows)
        self.node.get_logger().info(f"✔ Log salvo em: {self.out_csv}")

# -----------------------------
# Nó principal
# -----------------------------

class RectangleDriverNode(Node):
    def __init__(self):
        super().__init__('rectangle_driver_node')

        # -------- Parâmetros gerais --------
        self.declare_parameter("pose_topic", "zed/zed_node/odom")
        self.declare_parameter("wheel_odom_topic", "/wheel/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        self.declare_parameter("hz", 30)
        self.declare_parameter("out_csv", "rect_route_log.csv")

        # Controle
        self.declare_parameter("lin_speed", 0.25)         # m/s nominal
        self.declare_parameter("ang_speed", 0.6)          # rad/s nominal (usado como teto)
        self.declare_parameter("yaw_kp", 1.6)             # ganho proporcional para heading-to-goal
        self.declare_parameter("yaw_w_limit", 0.8)        # limite |w| (rad/s)
        self.declare_parameter("lin_acc", 0.6)            # m/s²
        self.declare_parameter("ang_acc", 2.0)            # rad/s²
        self.declare_parameter("min_linear", 0.05)        # m/s mínimo quando “virado” para o alvo
        self.declare_parameter("ang_stop_deg", 35.0)      # acima disso, zera v para girar parado
        self.declare_parameter("slow_radius", 0.6)        # desaceleração quando dist < slow_radius
        self.declare_parameter("pos_tol", 0.07)           # tolerância de posição (m)
        self.declare_parameter("ang_tol_deg", 8.0)        # tolerância angular final (graus)
        self.declare_parameter("arrived_pause", 0.25)     # pausa ao chegar (s)

        # -------- Como definir o retângulo/waypoints --------
        self.declare_parameter("laps", 1)
        self.declare_parameter("clockwise", False)

        # Opção A: gerar retângulo por parâmetros (âncora + W/H + yaw)
        self.declare_parameter("use_rectangle_params", True)
        self.declare_parameter("start_x", 0.0)
        self.declare_parameter("start_y", 0.0)
        self.declare_parameter("width", 2.0)
        self.declare_parameter("height", 4.0)
        self.declare_parameter("yaw_deg", 0.0)  # orientação da primeira aresta (graus)

        # Giros explícitos (com pausa antes/depois)
        self.declare_parameter("explicit_turns", True)
        self.declare_parameter("turn_pre_sleep", 2.0)   # s antes de iniciar o giro
        self.declare_parameter("turn_post_sleep", 2.0)  # s após terminar o giro


        # Opção B: definir metas manualmente (string "x1 y1; x2 y2; ...")
        self.declare_parameter("goals_str", "")

        # Leitura dos parâmetros
        self.pose_topic    = self.get_parameter("pose_topic").value
        self.wheel_topic   = self.get_parameter("wheel_odom_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        self.hz            = int(self.get_parameter("hz").value)
        self.out_csv       = self.get_parameter("out_csv").value

        self.lin_speed     = float(self.get_parameter("lin_speed").value)
        self.ang_speed     = float(self.get_parameter("ang_speed").value)
        self.yaw_kp        = float(self.get_parameter("yaw_kp").value)
        self.yaw_w_limit   = float(self.get_parameter("yaw_w_limit").value)
        self.lin_acc       = float(self.get_parameter("lin_acc").value)
        self.ang_acc       = float(self.get_parameter("ang_acc").value)
        self.min_linear    = float(self.get_parameter("min_linear").value)
        self.ang_stop_deg  = float(self.get_parameter("ang_stop_deg").value)
        self.slow_radius   = float(self.get_parameter("slow_radius").value)
        self.pos_tol       = float(self.get_parameter("pos_tol").value)
        self.ang_tol_deg   = float(self.get_parameter("ang_tol_deg").value)
        self.arrived_pause = float(self.get_parameter("arrived_pause").value)

        self.laps          = int(self.get_parameter("laps").value)
        self.clockwise     = bool(self.get_parameter("clockwise").value)

        self.use_rect      = bool(self.get_parameter("use_rectangle_params").value)
        self.start_x       = float(self.get_parameter("start_x").value)
        self.start_y       = float(self.get_parameter("start_y").value)
        self.width         = float(self.get_parameter("width").value)
        self.height        = float(self.get_parameter("height").value)
        self.yaw_deg       = float(self.get_parameter("yaw_deg").value)
        self.goals_str     = str(self.get_parameter("goals_str").value).strip()

        self.explicit_turns = bool(self.get_parameter("explicit_turns").value)
        self.turn_pre_sleep = float(self.get_parameter("turn_pre_sleep").value)
        self.turn_post_sleep = float(self.get_parameter("turn_post_sleep").value)


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

        # Waypoints (lista de (x,y))
        self.goals: List[Tuple[float,float]] = self.build_waypoints()

        self.get_logger().info(
            f"Nó iniciado. Pose: '{self.pose_topic}', Wheel: '{self.wheel_topic}', "
            f"cmd_vel: '{self.cmd_vel_topic}'. {len(self.goals)} metas."
        )
        for i, (gx, gy) in enumerate(self.goals, 1):
            self.get_logger().info(f"  WP{i}: ({gx:.3f}, {gy:.3f})")

        # Dá um tempinho para começar a receber odom
        time.sleep(0.5)

        # Thread do controle
        self.control_thread = threading.Thread(target=self.run_logic, daemon=True)
        self.control_thread.start()

    # -------------------------
    # Construção dos waypoints
    # -------------------------
    def build_waypoints(self) -> List[Tuple[float,float]]:
        # Se goals_str fornecido, usa isso: formato "x1 y1; x2 y2; x3 y3"
        if self.goals_str:
            pts = []
            for tok in self.goals_str.split(";"):
                tok = tok.strip()
                if not tok:
                    continue
                xs, ys = tok.split()
                pts.append((float(xs), float(ys)))
            # Repete pelas voltas
            return pts * max(1, self.laps)

        # Caso contrário, gera retângulo a partir dos parâmetros
        yaw = math.radians(self.yaw_deg)
        dir_w = (math.cos(yaw), math.sin(yaw))  # direção da largura
        if self.clockwise:
            # Rotação -90° (CW): (x, y) -> ( y, -x)
            dir_h = ( dir_w[1], -dir_w[0])
        else:
            # Rotação +90° (CCW): (x, y) -> (-y, x)
            dir_h = (-dir_w[1],  dir_w[0])

        c0 = (self.start_x, self.start_y)
        c1 = (c0[0] + self.width  * dir_w[0], c0[1] + self.width  * dir_w[1])
        c2 = (c1[0] + self.height * dir_h[0], c1[1] + self.height * dir_h[1])
        c3 = (c2[0] - self.width  * dir_w[0], c2[1] - self.width  * dir_w[1])
        # Fechamos no c0 para completar o retângulo
        base_loop = [c0, c1, c2, c3, c0]

        goals = []
        for _ in range(max(1, self.laps)):
            goals.extend(base_loop)
        return goals

    # -------------------------
    # Loop de controle
    # -------------------------
    def run_logic(self):
        try:
            self.follow_waypoints(self.goals)
            self.logger_instance.save()
            self.get_logger().info("✔ Percurso concluído. Encerrando nó.")
        except Exception as e:
            self.get_logger().error(f"Erro no percurso: {e}")
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

    def _rate(self, hz):
        return self.create_rate(hz)

    def stop(self, limiter: RateLimiter, hz=30, t=0.25, seg_name="stop"):
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

    # Controller “vá-para-alvo (x,y)”
    def go_to_xy(self, gx: float, gy: float, hz: int, seg_name: str):
        # Espera primeira odom
        t0 = time.time()
        while rclpy.ok() and not self.pose_feedback.has_odom:
            if time.time() - t0 > 5.0:
                self.get_logger().warn("Sem odom (5s). Continuando…")
                break
            time.sleep(0.01)

        r = self._rate(hz)
        limiter = RateLimiter(lin_acc=self.lin_acc, ang_acc=self.ang_acc, hz=hz)

        ang_stop = math.radians(self.ang_stop_deg)
        ang_tol  = math.radians(self.ang_tol_deg)

        while rclpy.ok():
            # Erros
            dx = gx - self.pose_feedback.x
            dy = gy - self.pose_feedback.y
            dist = math.hypot(dx, dy)

            if dist <= self.pos_tol:
                break

            # Heading desejado = direção até o goal
            heading_des = math.atan2(dy, dx)
            e_yaw = ang_diff(heading_des, self.pose_feedback.yaw)

            # Controle angular P saturado
            w_des = max(-self.yaw_w_limit, min(self.yaw_w_limit, self.yaw_kp * e_yaw))
            # Limite nominal adicional
            #w_des = max(-self.ang_speed, min(self.ang_speed, w_des))

            # Controle linear: desacelera perto do alvo e quando erro angular é grande
            # - desaceleração por distância
            v_scale_dist = min(1.0, dist / self.slow_radius)  # <= 1
            # - reduz quando desalinhado; se muito desalinhado, zera v e gira no lugar
            if abs(e_yaw) > ang_stop:
                v_des = 0.0
            else:
                v_des = self.lin_speed * v_scale_dist * max(0.2, math.cos(abs(e_yaw)))
                v_des = max(self.min_linear, v_des)

            v_cmd, w_cmd = limiter.step(v_des, w_des)
            self.pub_twist(v_cmd, w_cmd)
            self.logger_instance.add(self.get_clock().now().nanoseconds / 1e9,
                                     seg_name, self.pose_feedback, self.wheel_feedback, v_cmd, w_cmd,
                                     goal_xy=(gx, gy))
            r.sleep()

        # freia suave e dá uma pequena pausa para estabilizar
        self.stop(limiter, hz=hz, t=self.arrived_pause, seg_name=f"{seg_name}_arrived")
    
    def rotate_to_yaw(self, yaw_target, hz, seg_name, pre_sleep=0.0, post_sleep=0.0):
        # espera odom
        t0 = time.time()
        while rclpy.ok() and not self.pose_feedback.has_odom:
            if time.time() - t0 > 5.0:
                self.get_logger().warn("Sem odom (5s) para rotate_to_yaw. Continuando…")
                break
            time.sleep(0.01)

        if pre_sleep > 0.0:
            time.sleep(pre_sleep)

        limiter = RateLimiter(lin_acc=999.0, ang_acc=self.ang_acc, hz=hz)
        r = self.create_rate(hz)

        while rclpy.ok():
            e_yaw = ang_diff(yaw_target, self.pose_feedback.yaw)
            if abs(e_yaw) <= math.radians(self.ang_tol_deg):
                break
            # controle P com saturação
            w_des = max(-self.yaw_w_limit, min(self.yaw_w_limit, self.yaw_kp * e_yaw))
            w_des = max(-self.ang_speed, min(self.ang_speed, w_des))
            v_cmd, w_cmd = limiter.step(0.0, w_des)  # gira PARADO
            self.pub_twist(v_cmd, w_cmd)
            self.logger_instance.add(self.get_clock().now().nanoseconds / 1e9,
                                    seg_name, self.pose_feedback, self.wheel_feedback, v_cmd, w_cmd)
            r.sleep()

        self.stop(limiter, hz=hz, seg_name=f"{seg_name}_stop")

        if post_sleep > 0.0:
            time.sleep(post_sleep)


    def follow_waypoints(self, goals):
        self.get_logger().info(f"Seguindo {len(goals)} waypoints; laps={self.laps}, clockwise={self.clockwise}")
        prev_goal = None
        for i, (gx, gy) in enumerate(goals, 1):
            if not rclpy.ok():
                return

            # Se já passamos pelo primeiro WP, fazemos a rotação explícita para a direção do próximo segmento
            if self.explicit_turns and prev_goal is not None:
                # heading desejado = direção de prev_goal -> (gx, gy)
                heading_des = math.atan2(gy - prev_goal[1], gx - prev_goal[0])
                self.get_logger().info(f"↻ Giro para WP{i}: heading {math.degrees(heading_des):.1f}°")
                self.rotate_to_yaw(heading_des, hz=max(self.hz, 40),
                                seg_name=f"turn_to_wp{i}",
                                pre_sleep=self.turn_pre_sleep,
                                post_sleep=self.turn_post_sleep)

            self.get_logger().info(f"▶ Indo para WP{i}: ({gx:.3f}, {gy:.3f})")
            self.go_to_xy(gx, gy, hz=self.hz, seg_name=f"wp{i}")
            prev_goal = (gx, gy)

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

