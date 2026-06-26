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
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

def yaw_from_quat(q):
    return euler_from_quaternion_xyzw(q.x, q.y, q.z, q.w)[2]

def ang_diff(a, b):
    return (a - b + math.pi) % (2*math.pi) - math.pi

def stamp_to_seconds(stamp):
    try:
        return stamp.sec + stamp.nanosec / 1e9
    except Exception:
        return RosTime.from_msg(stamp).nanoseconds / 1e9

# -----------------------------
# Classes Auxiliares
# -----------------------------

class OdomWatch:
    # ... (esta classe permanece inalterada)
    def __init__(self, node: Node, topic: str, qos: Optional[QoSProfile] = None):
        self.node = node; self.has_odom = False
        self.t = self.x = self.y = self.yaw = self.vx = self.vy = self.wz = 0.0
        if qos is None:
            qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self._sub = self.node.create_subscription(Odometry, topic, self.cb, qos)
    def cb(self, msg: Odometry):
        self.t = stamp_to_seconds(msg.header.stamp); self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y; self.yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.vx = msg.twist.twist.linear.x; self.vy = msg.twist.twist.linear.y
        self.wz = msg.twist.twist.angular.z
        if not self.has_odom: self.node.get_logger().info("OdomWatch: primeiro pacote recebido.")
        self.has_odom = True

class RateLimiter:
    # ... (esta classe permanece inalterada)
    def __init__(self, lin_acc=0.5, ang_acc=1.5, hz=30):
        self.lin_acc, self.ang_acc, self.dt = float(lin_acc), float(ang_acc), 1.0 / float(hz)
        self.v_prev, self.w_prev = 0.0, 0.0
    def step(self, v_des, w_des):
        dv_max, dw_max = self.lin_acc * self.dt, self.ang_acc * self.dt
        v_cmd = self._clamp(v_des, self.v_prev - dv_max, self.v_prev + dv_max)
        w_cmd = self._clamp(w_des, self.w_prev - dw_max, self.w_prev + dw_max)
        self.v_prev, self.w_prev = v_cmd, w_cmd
        return v_cmd, w_cmd
    @staticmethod
    def _clamp(x, lo, hi): return min(max(x, lo), hi)

class DataLogger:
    # ... (esta classe permanece inalterada)
    def __init__(self, node: Node, out_csv="rect_route_log.csv"):
        self.node, self.rows, self.out_csv = node, [], out_csv
    def add(self, t, seg_name, pose_source: OdomWatch, wheel: Optional[OdomWatch], v_cmd, w_cmd, goal_xy: Optional[Tuple[float,float]] = None):
        row = {"t": t, "segment": seg_name, "v_cmd": v_cmd, "w_cmd": w_cmd,
               "pose_x": pose_source.x, "pose_y": pose_source.y, "pose_yaw": pose_source.yaw,
               "pose_vx": pose_source.vx, "pose_wz": pose_source.wz}
        if goal_xy is not None:
            row.update({"goal_x": goal_xy[0], "goal_y": goal_xy[1]})
            row.update({"err_xy": math.hypot(goal_xy[0]-pose_source.x, goal_xy[1]-pose_source.y)})
        if wheel and wheel.has_odom:
            row.update({"wheel_x": wheel.x, "wheel_y": wheel.y, "wheel_yaw": wheel.yaw,
                        "wheel_vx": wheel.vx, "wheel_wz": wheel.wz})
        self.rows.append(row)
    def save(self):
        if not self.rows: self.node.get_logger().warn("DataLogger: nada para salvar."); return
        os.makedirs(os.path.dirname(self.out_csv) or ".", exist_ok=True)
        with open(self.out_csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(self.rows[0].keys())); w.writeheader(); w.writerows(self.rows)
        self.node.get_logger().info(f"✔ Log salvo em: {self.out_csv}")

# <<< NOVA CLASSE: CONTROLADOR PI >>>
class PIController:
    """Controlador Proporcional-Integral com anti-windup."""
    def __init__(self, Kp, Ki, dt, min_output, max_output, integral_clamp):
        self.Kp = Kp
        self.Ki = Ki
        self.dt = dt
        self.min_output = min_output
        self.max_output = max_output
        self.integral_clamp = integral_clamp
        self.integral_error = 0.0

    def step(self, error):
        # Termo Proporcional
        p_term = self.Kp * error
        
        # Termo Integral com anti-windup (clamp)
        self.integral_error += error * self.dt
        self.integral_error = max(-self.integral_clamp, min(self.integral_clamp, self.integral_error))
        i_term = self.Ki * self.integral_error
        
        # Saída total do controlador
        output = p_term + i_term
        
        # Saturação da saída
        return max(self.min_output, min(self.max_output, output))

    def reset(self):
        """Reseta o estado do controlador (erro integral)."""
        self.integral_error = 0.0

# -----------------------------
# Nó principal
# -----------------------------

class RectangleDriverNode(Node):
    def __init__(self):
        super().__init__('rectangle_driver_node')

        # -------- Parâmetros gerais --------
        # ... (parâmetros de tópicos, etc. permanecem os mesmos)
        self.declare_parameter("pose_topic", "zed/zed_node/odom")
        self.declare_parameter("wheel_odom_topic", "/wheel/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("hz", 30)
        self.declare_parameter("out_csv", "rect_route_log.csv")

        # Controle
        self.declare_parameter("lin_speed", 0.25)
        self.declare_parameter("ang_speed", 0.6)
        self.declare_parameter("yaw_kp", 1.6)
        # <<< NOVOS PARÂMETROS PARA O CONTROLADOR PI >>>
        self.declare_parameter("yaw_ki", 0.15) # Ganho integral
        self.declare_parameter("yaw_integral_clamp", 0.6) # Limite do termo integral (anti-windup)
        # <<< FIM DOS NOVOS PARÂMETROS >>>
        self.declare_parameter("yaw_w_limit", 0.8)
        self.declare_parameter("lin_acc", 0.6)
        self.declare_parameter("ang_acc", 2.0)
        self.declare_parameter("min_linear", 0.05)
        self.declare_parameter("ang_stop_deg", 35.0)
        self.declare_parameter("slow_radius", 0.6)
        self.declare_parameter("pos_tol", 0.07)
        self.declare_parameter("ang_tol_deg", 8.0)
        self.declare_parameter("arrived_pause", 0.25)

        # ... (parâmetros de waypoints permanecem os mesmos)
        self.declare_parameter("laps", 1); self.declare_parameter("clockwise", False)
        self.declare_parameter("use_rectangle_params", True)
        self.declare_parameter("start_x", 0.0); self.declare_parameter("start_y", 0.0)
        self.declare_parameter("width", 2.0); self.declare_parameter("height", 4.0)
        self.declare_parameter("yaw_deg", 0.0)
        self.declare_parameter("explicit_turns", True)
        self.declare_parameter("turn_pre_sleep", 2.0)
        self.declare_parameter("turn_post_sleep", 2.0)
        self.declare_parameter("goals_str", "")

        # Leitura dos parâmetros
        # ... (leitura dos parâmetros existentes)
        self.pose_topic = self.get_parameter("pose_topic").value; self.wheel_topic = self.get_parameter("wheel_odom_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value; self.hz = int(self.get_parameter("hz").value)
        self.out_csv = self.get_parameter("out_csv").value; self.lin_speed = float(self.get_parameter("lin_speed").value)
        self.ang_speed = float(self.get_parameter("ang_speed").value); self.yaw_kp = float(self.get_parameter("yaw_kp").value)
        # <<< LEITURA DOS NOVOS PARÂMETROS >>>
        self.yaw_ki = float(self.get_parameter("yaw_ki").value)
        self.yaw_integral_clamp = float(self.get_parameter("yaw_integral_clamp").value)
        # <<< FIM DA LEITURA >>>
        self.yaw_w_limit = float(self.get_parameter("yaw_w_limit").value); self.lin_acc = float(self.get_parameter("lin_acc").value)
        self.ang_acc = float(self.get_parameter("ang_acc").value); self.min_linear = float(self.get_parameter("min_linear").value)
        self.ang_stop_deg = float(self.get_parameter("ang_stop_deg").value); self.slow_radius = float(self.get_parameter("slow_radius").value)
        self.pos_tol = float(self.get_parameter("pos_tol").value); self.ang_tol_deg = float(self.get_parameter("ang_tol_deg").value)
        self.arrived_pause = float(self.get_parameter("arrived_pause").value); self.laps = int(self.get_parameter("laps").value)
        self.clockwise = bool(self.get_parameter("clockwise").value); self.use_rect = bool(self.get_parameter("use_rectangle_params").value)
        self.start_x = float(self.get_parameter("start_x").value); self.start_y = float(self.get_parameter("start_y").value)
        self.width = float(self.get_parameter("width").value); self.height = float(self.get_parameter("height").value)
        self.yaw_deg = float(self.get_parameter("yaw_deg").value); self.goals_str = str(self.get_parameter("goals_str").value).strip()
        self.explicit_turns = bool(self.get_parameter("explicit_turns").value); self.turn_pre_sleep = float(self.get_parameter("turn_pre_sleep").value)
        self.turn_post_sleep = float(self.get_parameter("turn_post_sleep").value)

        # QoS
        odom_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        
        # Pub/Sub e Helpers
        self.pub_cmd = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.pose_feedback = OdomWatch(self, self.pose_topic, odom_qos)
        self.wheel_feedback = OdomWatch(self, self.wheel_topic, odom_qos) if self.wheel_topic else None
        self.logger_instance = DataLogger(self, self.out_csv)
        self.goals: List[Tuple[float,float]] = self.build_waypoints()

        self.get_logger().info(f"Nó iniciado. {len(self.goals)} metas. Kp={self.yaw_kp}, Ki={self.yaw_ki}")
        time.sleep(0.5)
        self.control_thread = threading.Thread(target=self.run_logic, daemon=True)
        self.control_thread.start()

    # ... (build_waypoints, run_logic, pub_twist, _rate, stop permanecem os mesmos)
    def build_waypoints(self) -> List[Tuple[float,float]]:
        if self.goals_str:
            pts = [tuple(map(float, tok.split())) for tok in self.goals_str.split(";") if tok.strip()]
            return pts * max(1, self.laps)
        yaw = math.radians(self.yaw_deg)
        dir_w, dir_h = (math.cos(yaw), math.sin(yaw)), ((-math.sin(yaw), math.cos(yaw)) if not self.clockwise else (math.sin(yaw), -math.cos(yaw)))
        c0 = (self.start_x, self.start_y)
        c1 = (c0[0] + self.width * dir_w[0], c0[1] + self.width * dir_w[1])
        c2 = (c1[0] + self.height * dir_h[0], c1[1] + self.height * dir_h[1])
        c3 = (c2[0] - self.width * dir_w[0], c2[1] - self.width * dir_w[1])
        base_loop = [c0, c1, c2, c3, c0]
        goals = []
        for _ in range(max(1, self.laps)): goals.extend(base_loop)
        return goals
    def run_logic(self):
        try: self.follow_waypoints(self.goals); self.logger_instance.save(); self.get_logger().info("✔ Percurso concluído. Encerrando nó.")
        except Exception as e: self.get_logger().error(f"Erro no percurso: {e}")
        finally: rclpy.shutdown()
    def pub_twist(self, v, w): msg = Twist(); msg.linear.x=float(v); msg.angular.z=float(w); self.pub_cmd.publish(msg)
    def _rate(self, hz): return self.create_rate(hz)
    def stop(self, limiter: RateLimiter, hz=30, t=0.25, seg_name="stop"):
        r, steps = self._rate(hz), max(1, int(t * hz))
        for _ in range(steps):
            if not rclpy.ok(): break
            v_cmd, w_cmd = limiter.step(0.0, 0.0)
            self.pub_twist(v_cmd, w_cmd)
            self.logger_instance.add(self.get_clock().now().nanoseconds/1e9, seg_name, self.pose_feedback, self.wheel_feedback, v_cmd, w_cmd)
            r.sleep()


    # <<< FUNÇÃO MODIFICADA: go_to_xy >>>
    def go_to_xy(self, gx: float, gy: float, hz: int, seg_name: str):
        t0 = time.time()
        while rclpy.ok() and not self.pose_feedback.has_odom:
            if time.time() - t0 > 5.0: self.get_logger().warn("Sem odom (5s). Continuando…"); break
            time.sleep(0.01)

        r = self._rate(hz)
        limiter = RateLimiter(lin_acc=self.lin_acc, ang_acc=self.ang_acc, hz=hz)
        
        # Cria e reseta o controlador PI para este alvo
        yaw_controller = PIController(Kp=self.yaw_kp, Ki=self.yaw_ki, dt=1.0/hz,
                                      min_output=-self.yaw_w_limit, max_output=self.yaw_w_limit,
                                      integral_clamp=self.yaw_integral_clamp)

        ang_stop = math.radians(self.ang_stop_deg)

        while rclpy.ok():
            dx, dy = gx - self.pose_feedback.x, gy - self.pose_feedback.y
            dist = math.hypot(dx, dy)
            if dist <= self.pos_tol: break

            heading_des = math.atan2(dy, dx)
            e_yaw = ang_diff(heading_des, self.pose_feedback.yaw)

            # Usa o controlador PI para obter a velocidade angular desejada
            w_des = yaw_controller.step(e_yaw)
            
            # Controle linear
            v_scale_dist = min(1.0, dist / self.slow_radius)
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
        self.stop(limiter, hz=hz, t=self.arrived_pause, seg_name=f"{seg_name}_arrived")
    
    # <<< FUNÇÃO MODIFICADA: rotate_to_yaw >>>
    def rotate_to_yaw(self, yaw_target, hz, seg_name, pre_sleep=0.0, post_sleep=0.0):
        t0 = time.time()
        while rclpy.ok() and not self.pose_feedback.has_odom:
            if time.time() - t0 > 5.0: self.get_logger().warn("Sem odom (5s) para rotate_to_yaw. Continuando…"); break
            time.sleep(0.01)

        if pre_sleep > 0.0: time.sleep(pre_sleep)

        limiter = RateLimiter(lin_acc=999.0, ang_acc=self.ang_acc, hz=hz)
        r = self.create_rate(hz)

        # Cria e reseta o controlador PI para esta rotação
        yaw_controller = PIController(Kp=self.yaw_kp, Ki=self.yaw_ki, dt=1.0/hz,
                                      min_output=-self.ang_speed, max_output=self.ang_speed,
                                      integral_clamp=self.yaw_integral_clamp)
        
        while rclpy.ok():
            e_yaw = ang_diff(yaw_target, self.pose_feedback.yaw)
            if abs(e_yaw) <= math.radians(self.ang_tol_deg): break
            
            # Usa o controlador PI
            w_des = yaw_controller.step(e_yaw)
            
            v_cmd, w_cmd = limiter.step(0.0, w_des)
            self.pub_twist(v_cmd, w_cmd)
            self.logger_instance.add(self.get_clock().now().nanoseconds / 1e9,
                                     seg_name, self.pose_feedback, self.wheel_feedback, v_cmd, w_cmd)
            r.sleep()
        self.stop(limiter, hz=hz, seg_name=f"{seg_name}_stop")
        if post_sleep > 0.0: time.sleep(post_sleep)

    # ... (follow_waypoints permanece o mesmo)
    def follow_waypoints(self, goals):
        self.get_logger().info(f"Seguindo {len(goals)} waypoints; laps={self.laps}, clockwise={self.clockwise}")
        prev_goal = None
        for i, (gx, gy) in enumerate(goals, 1):
            if not rclpy.ok(): return
            if self.explicit_turns and prev_goal is not None:
                heading_des = math.atan2(gy - prev_goal[1], gx - prev_goal[0])
                self.get_logger().info(f"↻ Giro para WP{i}: heading {math.degrees(heading_des):.1f}°")
                self.rotate_to_yaw(heading_des, hz=max(self.hz, 40), seg_name=f"turn_to_wp{i}",
                                   pre_sleep=self.turn_pre_sleep, post_sleep=self.turn_post_sleep)
            self.get_logger().info(f"▶ Indo para WP{i}: ({gx:.3f}, {gy:.3f})")
            self.go_to_xy(gx, gy, hz=self.hz, seg_name=f"wp{i}")
            prev_goal = (gx, gy)


# -----------------------------
# main
# -----------------------------

def main(args=None):
    rclpy.init(args=args)
    node = RectangleDriverNode()
    try: rclpy.spin(node)
    finally:
        if rclpy.ok(): rclpy.shutdown()
        node.destroy_node()

if __name__ == "__main__":
    main()
