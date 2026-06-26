import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import threading
import time

# Importações necessárias para a matemática manual
import math
import numpy as np

# --- Parâmetros de Configuração ---
WHEEL_TOPIC = '/wheel/odom'
ZED_TOPIC = '/zed/zed_node/odom'
MAX_POINTS = 100 # Número máximo de pontos no gráfico
# --- Fim da Configuração ---

# --- Funções Auxiliares para Matemática de Quaternions ---

def quat_inverse(q):
    """Calcula o inverso de um quaternion (conjugado para unitários)."""
    # q é uma lista ou array no formato [x, y, z, w]
    x, y, z, w = q
    return [-x, -y, -z, w]

def quat_multiply(q1, q2):
    """Multiplica dois quaternions q1 * q2."""
    # q1 e q2 são listas ou arrays no formato [x, y, z, w]
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    
    return [x, y, z, w]

def stamp_to_seconds(stamp):
    """Converte a mensagem de timestamp do ROS para segundos."""
    return stamp.sec + stamp.nanosec / 1e9

class OdomComparePlotterNode(Node):
    def __init__(self, wheel_topic, zed_topic, max_points):
        super().__init__('odom_compare_plotter_node')
        
        self.lock = threading.Lock()
        self.start_time = time.time()

        self.last_zed_orientation = None
        self.last_zed_time = None
        self.calculated_zed_wz = 0.0

        self.timestamps = deque(maxlen=max_points)
        self.wheel_wz_data = deque(maxlen=max_points)
        self.zed_calculated_wz_data = deque(maxlen=max_points)
        self.latest_wheel_wz = 0.0

        self.wheel_subscription = self.create_subscription(
            Odometry, wheel_topic, self.wheel_odom_callback, 10)
        self.zed_subscription = self.create_subscription(
            Odometry, zed_topic, self.zed_odom_callback, 10)
            
        self.get_logger().info(f'Escutando os tópicos "{wheel_topic}" e "{zed_topic}"...')
        self.create_timer(0.1, self.update_plot_data)

    def wheel_odom_callback(self, msg):
        with self.lock:
            self.latest_wheel_wz = msg.twist.twist.angular.z

    def zed_odom_callback(self, msg):
        q = msg.pose.pose.orientation
        current_orientation = [q.x, q.y, q.z, q.w]
        current_time = stamp_to_seconds(msg.header.stamp)

        with self.lock:
            if self.last_zed_orientation is not None and self.last_zed_time is not None:
                delta_t = current_time - self.last_zed_time
                if delta_t > 1e-6:
                    # Passo 1: Calcular a rotação diferencial: q_diff = q_curr * q_prev_inverse
                    q_prev_inv = quat_inverse(self.last_zed_orientation)
                    q_diff = quat_multiply(current_orientation, q_prev_inv)
                    
                    # Passo 2: Converter o quaternion diferencial para vetor de velocidade angular
                    dx, dy, dz, dw = q_diff
                    
                    # Prevenir erros de domínio no acos por imprecisão de float
                    if dw > 1.0: dw = 1.0
                    if dw < -1.0: dw = -1.0
                    
                    angle = 2.0 * math.acos(dw)
                    
                    # Pequena aproximação para ângulos muito pequenos para evitar divisão por zero
                    if abs(angle) < 1e-6:
                        angular_velocity_vec = [0.0, 0.0, 0.0]
                    else:
                        norm = math.sqrt(dx**2 + dy**2 + dz**2)
                        # O vetor de velocidade angular é (ângulo * eixo) / delta_t
                        angular_velocity_vec = [(angle * dx / norm) / delta_t,
                                                (angle * dy / norm) / delta_t,
                                                (angle * dz / norm) / delta_t]
                    
                    self.calculated_zed_wz = angular_velocity_vec[2]
            
            self.last_zed_orientation = current_orientation
            self.last_zed_time = current_time
            
    def update_plot_data(self):
        with self.lock:
            plot_time = time.time() - self.start_time
            self.timestamps.append(plot_time)
            self.wheel_wz_data.append(self.latest_wheel_wz)
            self.zed_calculated_wz_data.append(self.calculated_zed_wz)
            
    def get_data_for_plot(self):
        with self.lock:
            return list(self.timestamps), list(self.wheel_wz_data), list(self.zed_calculated_wz_data)

class RealTimePlotter:
    def __init__(self, node):
        self.node = node
        self.fig, self.ax = plt.subplots()
        
        self.line_wheel, = self.ax.plot([], [], 'r-', label='Wheel Odom Wz (Direto)')
        self.line_zed, = self.ax.plot([], [], 'c--', label='ZED Wz (Calculado da Orientação)')
        
        self.ax.set_ylabel('Velocidade Angular Z (rad/s)')
        self.ax.set_xlabel('Tempo (s)')
        self.ax.legend()
        self.ax.grid(True)
        self.fig.suptitle('Comparativo de Velocidade Angular (Wz)', fontsize=14)

        self.ax.set_ylim(-1, 1)

    def update_plot(self, frame):
        timestamps, wheel_data, zed_data = self.node.get_data_for_plot()
        if not timestamps:
            return self.line_wheel, self.line_zed
        self.line_wheel.set_data(timestamps, wheel_data)
        self.line_zed.set_data(timestamps, zed_data)
        if timestamps:
            self.ax.set_xlim(timestamps[0], timestamps[-1] if len(timestamps) > 1 else timestamps[0] + 1)
        return self.line_wheel, self.line_zed

    def run(self):
        ani = FuncAnimation(self.fig, self.update_plot, blit=True, interval=100)
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    plotter_node = OdomComparePlotterNode(WHEEL_TOPIC, ZED_TOPIC, MAX_POINTS)
    spin_thread = threading.Thread(target=rclpy.spin, args=(plotter_node,), daemon=True)
    spin_thread.start()
    plotter = RealTimePlotter(plotter_node)
    try:
        plotter.run()
    except KeyboardInterrupt:
        pass
    finally:
        plotter_node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()
