#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import csv
import threading
import time
import math
from typing import Optional

# Importações do ROS 2
import rclpy
from rclpy.node import Node
from rclpy.time import Time as RosTime
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# --- Funções Auxiliares ---

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
    """Extrai yaw de geometry_msgs/Quaternion."""
    return euler_from_quaternion_xyzw(q.x, q.y, q.z, q.w)[2]

def stamp_to_seconds(stamp):
    """Converte a mensagem de timestamp do ROS 2 para segundos."""
    try:
        return stamp.sec + stamp.nanosec / 1e9
    except Exception:
        return RosTime.from_msg(stamp).nanoseconds / 1e9

def quat_inverse(q):
    x, y, z, w = q
    return [-x, -y, -z, w]

def quat_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return [x, y, z, w]

# --- Classes Auxiliares ---

class OdomWatch:
    def __init__(self, node: Node, topic: str):
        self.node = node
        self.wz = 0.0
        self.yaw = 0.0 # <<< ALTERAÇÃO: Adicionado para armazenar o yaw
        
        self.last_orientation = None
        self.last_time = None

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self._sub = self.node.create_subscription(Odometry, topic, self.cb, qos)

    def cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        current_orientation = [q.x, q.y, q.z, q.w]
        current_time = stamp_to_seconds(msg.header.stamp)
        
        # <<< ALTERAÇÃO: Armazena o yaw atual a cada callback >>>
        self.yaw = yaw_from_quat(q)

        if self.last_orientation is not None and self.last_time is not None:
            delta_t = current_time - self.last_time
            if delta_t > 1e-6:
                q_prev_inv = quat_inverse(self.last_orientation)
                q_diff = quat_multiply(current_orientation, q_prev_inv)
                
                dx, dy, dz, dw = q_diff
                if dw > 1.0: dw = 1.0
                
                angle = 2.0 * math.acos(dw)
                norm = math.sqrt(dx**2 + dy**2 + dz**2)

                if norm < 1e-6:
                    angular_velocity_z = 0.0
                else:
                    angular_velocity_z = (angle * dz / norm) / delta_t
                
                self.wz = angular_velocity_z
        
        self.last_orientation = current_orientation
        self.last_time = current_time

class DataLogger:
    def __init__(self, node: Node, out_csv: str):
        self.node = node
        self.rows = []
        self.out_csv = out_csv
        # <<< ALTERAÇÃO: Adicionada nova coluna ao cabeçalho >>>
        self.header = ["timestamp", "command_wz", "measured_wz", "measured_yaw"]

    def add(self, t: float, cmd_wz: float, odom: OdomWatch):
        # <<< ALTERAÇÃO: Adicionado o `measured_yaw` à linha do log >>>
        row = {
            "timestamp": t,
            "command_wz": cmd_wz,
            "measured_wz": odom.wz,
            "measured_yaw": odom.yaw 
        }
        self.rows.append(row)

    def save(self):
        # ... (sem alterações aqui)
        if not self.rows: self.node.get_logger().warn("DataLogger: nada para salvar."); return
        try:
            os.makedirs(os.path.dirname(self.out_csv) or ".", exist_ok=True)
            with open(self.out_csv, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=self.header); writer.writeheader(); writer.writerows(self.rows)
            self.node.get_logger().info(f"✔ Log de rotação salvo em: {self.out_csv}")
        except Exception as e: self.node.get_logger().error(f"Falha ao salvar o log: {e}")

# --- Nó Principal ---
# Nenhuma alteração necessária no YawTesterNode ou na função main.
class YawTesterNode(Node):
    def __init__(self):
        super().__init__('yaw_tester_node')
        self.declare_parameter("target_wz", 1.0); self.declare_parameter("duration", 10.0)
        self.declare_parameter("hz", 30); self.declare_parameter("odom_topic", "/zed/zed_node/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel"); self.declare_parameter("log_file", "yaw_test_log.csv")
        self.target_wz = self.get_parameter("target_wz").value; self.duration = self.get_parameter("duration").value
        self.hz = self.get_parameter("hz").value; odom_topic = self.get_parameter("odom_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value; log_file = self.get_parameter("log_file").value
        self.pub_cmd = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.odom_feedback = OdomWatch(self, odom_topic)
        self.logger = DataLogger(self, log_file)
        self.get_logger().info("Nó de teste de rotação iniciado."); self.get_logger().info(f"  - Velocidade Alvo: {self.target_wz:.2f} rad/s")
        self.get_logger().info(f"  - Duração: {self.duration:.1f} s"); self.get_logger().info(f"  - Tópico Odom: {odom_topic}")
        self.get_logger().info(f"  - Arquivo de Log: {log_file}")
        self.control_thread = threading.Thread(target=self.run_test_logic, daemon=True)
        self.control_thread.start()
    def run_test_logic(self):
        time.sleep(1.0); self.get_logger().info("Iniciando rotação...")
        rate = self.create_rate(self.hz); start_time = self.get_clock().now()
        cmd_msg = Twist(); cmd_msg.linear.x = 0.0; cmd_msg.angular.z = float(self.target_wz)
        while rclpy.ok():
            current_time = self.get_clock().now(); elapsed_time = (current_time - start_time).nanoseconds / 1e9
            if elapsed_time >= self.duration: break
            self.pub_cmd.publish(cmd_msg)
            self.logger.add(elapsed_time, self.target_wz, self.odom_feedback)
            rate.sleep()
        self.get_logger().info("Duração do teste concluída. Parando o robô...")
        self.pub_cmd.publish(Twist()); time.sleep(0.5)
        self.logger.save(); self.get_logger().info("Teste finalizado.")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = YawTesterNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == "__main__":
    main()
