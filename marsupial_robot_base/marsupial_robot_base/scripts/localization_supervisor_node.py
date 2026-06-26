#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class LocalizationSupervisor(Node):
    """
    Nó supervisor de localização.
    Monitora /amcl_pose, /odom e /particlecloud e publica um status em /localization_status:
      - "OK"
      - "UNCERTAIN"
      - "LOST"
    """

    # Limiares (ajuste fino depois de testar)
    MAX_COV_X_OK = 0.20
    MAX_COV_Y_OK = 0.20
    MAX_COV_YAW_OK = (5.0 * math.pi / 180.0)   # 5 graus

    MAX_COV_X_LOST = 0.50
    MAX_COV_Y_LOST = 0.50
    MAX_COV_YAW_LOST = (10.0 * math.pi / 180.0)  # 10 graus

    MAX_DIST_ODOM_AMCL_OK = 0.30    # m
    MAX_DIST_ODOM_AMCL_LOST = 1.00  # m

    MAX_STD_PARTICLE_OK = 0.20      # m
    MAX_STD_PARTICLE_LOST = 0.80    # m

    def __init__(self):
        super().__init__('localization_supervisor')

        # Subs
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.particle_sub = self.create_subscription(
            PoseArray,
            '/particlecloud',
            self.particlecloud_callback,
            10
        )

        # Pub
        self.status_pub = self.create_publisher(String, '/localization_status', 10)

        # Últimos dados recebidos
        self.last_amcl: Optional[PoseWithCovarianceStamped] = None
        self.last_odom: Optional[Odometry] = None
        self.last_particles: Optional[PoseArray] = None

        # Timer de avaliação (10 Hz, por exemplo)
        self.timer = self.create_timer(0.1, self.evaluate_localization)

        self.get_logger().info('LocalizationSupervisor iniciado.')

    # ---------- Callbacks ----------

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        self.last_amcl = msg

    def odom_callback(self, msg: Odometry):
        self.last_odom = msg

    def particlecloud_callback(self, msg: PoseArray):
        self.last_particles = msg

    # ---------- Funções auxiliares ----------

    @staticmethod
    def _extract_covariances(pose_cov: PoseWithCovarianceStamped):
        """
        Extrai variâncias de x, y, yaw da matriz 6x6 flatten (36 elementos).
        Indices: 0 -> var(x), 7 -> var(y), 35 -> var(yaw)
        """
        cov = pose_cov.pose.covariance
        var_x = cov[0]
        var_y = cov[7]
        var_yaw = cov[35]
        return var_x, var_y, var_yaw

    @staticmethod
    def _distance_2d(x1, y1, x2, y2):
        return math.hypot(x2 - x1, y2 - y1)

    @staticmethod
    def _compute_particle_spread(particles: PoseArray):
        """
        Calcula desvio padrão de x e y dos particles.
        Se não houver partículas suficientes, retorna (None, None).
        """
        n = len(particles.poses)
        if n < 2:
            return None, None

        xs = [p.position.x for p in particles.poses]
        ys = [p.position.y for p in particles.poses]

        mean_x = sum(xs) / n
        mean_y = sum(ys) / n

        var_x = sum((x - mean_x) ** 2 for x in xs) / (n - 1)
        var_y = sum((y - mean_y) ** 2 for y in ys) / (n - 1)

        std_x = math.sqrt(var_x)
        std_y = math.sqrt(var_y)
        return std_x, std_y

    # ---------- Núcleo da lógica ----------

    def evaluate_localization(self):
        """
        Avalia se a localização está OK, UNCERTAIN ou LOST e publica o resultado.
        """
        if self.last_amcl is None or self.last_odom is None:
            # Ainda sem dados suficientes
            status = 'UNKNOWN'
            self._publish_status(status)
            return

        # 1) Covariâncias do AMCL
        cov_x, cov_y, cov_yaw = self._extract_covariances(self.last_amcl)

        # 2) Distância entre pose do odom e pose do AMCL
        ax = self.last_amcl.pose.pose.position.x
        ay = self.last_amcl.pose.pose.position.y

        ox = self.last_odom.pose.pose.position.x
        oy = self.last_odom.pose.pose.position.y

        dist_odom_amcl = self._distance_2d(ax, ay, ox, oy)

        # 3) Espalhamento dos particles (se disponível)
        std_particles = None
        if self.last_particles is not None:
            std_x, std_y = self._compute_particle_spread(self.last_particles)
            if std_x is not None and std_y is not None:
                std_particles = max(std_x, std_y)

        # --------- Decisão de estado ---------
        status = 'OK'

        # Critérios de LOST
        if (cov_x > self.MAX_COV_X_LOST or
            cov_y > self.MAX_COV_Y_LOST or
            cov_yaw > self.MAX_COV_YAW_LOST or
            dist_odom_amcl > self.MAX_DIST_ODOM_AMCL_LOST or
            (std_particles is not None and std_particles > self.MAX_STD_PARTICLE_LOST)):
            status = 'LOST'

        # Critérios de UNCERTAIN (se ainda não foi considerado LOST)
        elif (cov_x > self.MAX_COV_X_OK or
              cov_y > self.MAX_COV_Y_OK or
              cov_yaw > self.MAX_COV_YAW_OK or
              dist_odom_amcl > self.MAX_DIST_ODOM_AMCL_OK or
              (std_particles is not None and std_particles > self.MAX_STD_PARTICLE_OK)):
            status = 'UNCERTAIN'

        # (Opcional) log de debug em mudança de estado ou de tempos em tempos
        # self.get_logger().debug(
        #     f"Status={status} cov=({cov_x:.3f},{cov_y:.3f},{cov_yaw:.3f}) "
        #     f"dist_odom_amcl={dist_odom_amcl:.3f} "
        #     f"std_particles={std_particles if std_particles is not None else 'NA'}"
        # )

        self._publish_status(status)

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
