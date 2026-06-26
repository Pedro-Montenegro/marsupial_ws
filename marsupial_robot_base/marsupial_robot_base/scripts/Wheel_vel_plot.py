import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import threading
import time

# --- Parâmetros de Configuração ---
WHEEL_TOPIC = '/wheel/odom'
ZED_TOPIC = '/zed/zed_node/odom'
MAX_POINTS = 100 # Número máximo de pontos no gráfico
# --- Fim da Configuração ---

def stamp_to_seconds(stamp):
    """Converte a mensagem de timestamp do ROS para segundos."""
    return stamp.sec + stamp.nanosec / 1e9

class OdomComparePlotterNode(Node):
    """
    Nó que se inscreve em dois tópicos de odometria, calcula a velocidade
    a partir da posição de um deles e plota a comparação.
    """
    def __init__(self, wheel_topic, zed_topic, max_points):
        super().__init__('odom_compare_plotter_node')
        
        self.lock = threading.Lock()
        self.start_time = time.time()

        # Variáveis para o cálculo da velocidade da ZED
        self.last_zed_x = None
        self.last_zed_time = None
        self.calculated_zed_vx = 0.0

        # Deques para armazenar os dados de plotagem
        self.timestamps = deque(maxlen=max_points)
        self.wheel_vx_data = deque(maxlen=max_points)
        self.zed_calculated_vx_data = deque(maxlen=max_points)
        
        # Variável para guardar o valor mais recente da odometria da roda
        self.latest_wheel_vx = 0.0

        # Inscrições nos tópicos
        self.wheel_subscription = self.create_subscription(
            Odometry, wheel_topic, self.wheel_odom_callback, 10)
        self.zed_subscription = self.create_subscription(
            Odometry, zed_topic, self.zed_odom_callback, 10)
            
        self.get_logger().info(f'Escutando os tópicos "{wheel_topic}" e "{zed_topic}"...')
        
        # Timer para adicionar dados ao gráfico de forma sincronizada
        self.create_timer(0.1, self.update_plot_data) # Roda a 10Hz

    def wheel_odom_callback(self, msg):
        """Callback para a odometria das rodas (lê a velocidade diretamente)."""
        with self.lock:
            self.latest_wheel_vx = msg.twist.twist.linear.x

    def zed_odom_callback(self, msg):
        """Callback para a odometria da ZED (lê a posição e calcula a velocidade)."""
        current_x = msg.pose.pose.position.x
        current_time = stamp_to_seconds(msg.header.stamp)

        with self.lock:
            if self.last_zed_x is not None and self.last_zed_time is not None:
                delta_t = current_time - self.last_zed_time
                if delta_t > 1e-6: # Evita divisão por zero
                    delta_x = current_x - self.last_zed_x
                    self.calculated_zed_vx = delta_x / delta_t
            
            # Atualiza os valores para o próximo cálculo
            self.last_zed_x = current_x
            self.last_zed_time = current_time
            
    def update_plot_data(self):
        """Adiciona os valores mais recentes às listas de dados do gráfico."""
        with self.lock:
            plot_time = time.time() - self.start_time
            self.timestamps.append(plot_time)
            self.wheel_vx_data.append(self.latest_wheel_vx)
            self.zed_calculated_vx_data.append(self.calculated_zed_vx)
            
    def get_data_for_plot(self):
        """Retorna uma cópia dos dados atuais de forma segura para threads."""
        with self.lock:
            return list(self.timestamps), list(self.wheel_vx_data), list(self.zed_calculated_vx_data)

class RealTimePlotter:
    """
    Classe que gerencia a figura e a animação do Matplotlib.
    AGORA COM ESCALA Y FIXA.
    """
    def __init__(self, node):
        self.node = node
        self.fig, self.ax = plt.subplots()
        
        self.line_wheel, = self.ax.plot([], [], 'r-', label='Wheel Odom Vx (Direto)')
        self.line_zed, = self.ax.plot([], [], 'g--', label='ZED Vx (Calculado da Posição)')
        
        self.ax.set_ylabel('Velocidade Linear X (m/s)')
        self.ax.set_xlabel('Tempo (s)')
        self.ax.legend()
        self.ax.grid(True)
        self.fig.suptitle('Comparativo de Velocidade (Vx)', fontsize=14)

        # <<< MUDANÇA AQUI >>>
        # Define a escala do eixo Y como fixa entre -1 e 1
        self.ax.set_ylim(-1, 1)

    def update_plot(self, frame):
        """Função chamada periodicamente para atualizar o gráfico."""
        timestamps, wheel_data, zed_data = self.node.get_data_for_plot()
        
        if not timestamps:
            return self.line_wheel, self.line_zed

        self.line_wheel.set_data(timestamps, wheel_data)
        self.line_zed.set_data(timestamps, zed_data)

        # <<< MUDANÇA AQUI >>>
        # Remove o auto-ajuste do eixo Y, mas ajusta o eixo X para
        # criar um efeito de "janela deslizante".
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
