#!/usr/bin/env python3
"""
localization_sanity_monitor

Monitora se a localização está se comportando corretamente — não apenas se
o AMCL está vivo (isso é responsabilidade do robot_health_monitor).

Duas camadas de detecção:

  Camada A — AMCL jump detector:
    Guarda sempre a última pose conhecida do AMCL. A cada nova pose, calcula
    o delta entre atual e anterior e compara com o deslocamento real da odom
    no mesmo intervalo. Se o AMCL pulou muito mais que a odom registrou,
    é sinal de que as partículas convergiram para um local errado.

  Camada B — Covariância:
    Monitora a covariância da pose do AMCL. Zona de alerta entre 50% e 100%
    dos thresholds → SUSPECT. Acima dos thresholds → LOST.

Status resultante:
  OK      → sem jump, covariância ok
  SUSPECT → jump detectado OU covariância na zona de alerta
  LOST    → covariância acima dos thresholds OU jump muito grande (> 2x jump_min_m)

Payload publicado em /localization_sanity a 1Hz:
{
  "t": float,
  "status": "OK" | "SUSPECT" | "LOST",
  "ok": bool,
  "jump_detected": bool,
  "jump_delta_amcl_m": float | null,
  "jump_delta_odom_m": float | null,
  "covariance_ok": bool,
  "var_x":   float | null,
  "var_y":   float | null,
  "var_yaw": float | null,
  "reasons": [str]
}

Parâmetros ROS:
  pub_topic          (padrão: /localization_sanity)
  odom_topic         (padrão: /wheel/odom)

  # Camada A — jump detector
  jump_min_m         (padrão: 0.30)  — pulo mínimo do AMCL para ser considerado jump
  jump_ratio         (padrão: 5.0)   — pulo do AMCL deve ser N× maior que delta odom
  jump_big_m         (padrão: 0.60)  — pulo acima disto é LOST direto (2× jump_min_m)

  # Camada B — covariância
  loc_max_var_x      (padrão: 0.40)
  loc_max_var_y      (padrão: 0.40)
  loc_max_var_yaw    (padrão: 0.15)
  loc_warn_ratio     (padrão: 0.50)  — zona de alerta: entre warn_ratio e threshold

  # Odom — acumulação de deslocamento entre poses AMCL
  odom_max_age_s     (padrão: 2.0)   — janela de odom para calcular delta entre duas poses AMCL
"""

import time
import json
import math
from collections import deque
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped


def _yaw_from_quat(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _dist(x0, y0, x1, y1) -> float:
    return math.hypot(x1 - x0, y1 - y0)


class LocalizationSanityMonitor(Node):

    def __init__(self):
        super().__init__("localization_sanity_monitor")

        # ── parâmetros ────────────────────────────────────────────────
        self.declare_parameter("pub_topic",   "/localization_sanity")
        self.declare_parameter("odom_topic",  "/odometry/filtered")

        # Camada A
        self.declare_parameter("jump_min_m",  0.30)
        self.declare_parameter("jump_ratio",  5.0)
        self.declare_parameter("jump_big_m",  0.60)

        # Camada B
        self.declare_parameter("loc_max_var_x",   0.40)
        self.declare_parameter("loc_max_var_y",   0.40)
        self.declare_parameter("loc_max_var_yaw", 0.15)
        self.declare_parameter("loc_warn_ratio",  0.50)

        # Odom
        self.declare_parameter("odom_max_age_s", 2.0)

        self.pub_topic       = self.get_parameter("pub_topic").value
        self.odom_topic      = self.get_parameter("odom_topic").value
        self.jump_min_m      = float(self.get_parameter("jump_min_m").value)
        self.jump_ratio      = float(self.get_parameter("jump_ratio").value)
        self.jump_big_m      = float(self.get_parameter("jump_big_m").value)
        self.max_var_x       = float(self.get_parameter("loc_max_var_x").value)
        self.max_var_y       = float(self.get_parameter("loc_max_var_y").value)
        self.max_var_yaw     = float(self.get_parameter("loc_max_var_yaw").value)
        self.warn_ratio      = float(self.get_parameter("loc_warn_ratio").value)
        self.odom_max_age_s  = float(self.get_parameter("odom_max_age_s").value)

        # ── publisher ─────────────────────────────────────────────────
        self.pub = self.create_publisher(String, self.pub_topic, 10)

        # ── estado interno ────────────────────────────────────────────

        # Última pose AMCL conhecida: (timestamp, x, y, var_x, var_y, var_yaw)
        self._last_amcl: Optional[Tuple] = None

        # Última covariância conhecida (mantida mesmo sem nova pose)
        self._last_var_x:   Optional[float] = None
        self._last_var_y:   Optional[float] = None
        self._last_var_yaw: Optional[float] = None

        # Histórico de odom para calcular deslocamento entre duas poses AMCL
        # Entradas: (timestamp, x, y)
        self._odom_hist: deque = deque()

        # ── subscriptions ─────────────────────────────────────────────
        qos_amcl = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, qos_amcl
        )
        self.create_subscription(
            Odometry, self.odom_topic, self._on_odom, 10
        )

        self.create_timer(1.0, self._publish_sanity)

        self.get_logger().info(
            f"[localization_sanity_monitor] up | "
            f"pub={self.pub_topic} | odom={self.odom_topic} | "
            f"jump_min={self.jump_min_m}m jump_ratio={self.jump_ratio}x "
            f"jump_big={self.jump_big_m}m | "
            f"var_thresh=({self.max_var_x},{self.max_var_y},{self.max_var_yaw})"
        )

    # ─────────────────────────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────────────────────────

    def _on_odom(self, msg: Odometry):
        now = time.time()
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        self._odom_hist.append((now, x, y))
        # mantém só os últimos odom_max_age_s segundos
        while self._odom_hist and (now - self._odom_hist[0][0]) > self.odom_max_age_s:
            self._odom_hist.popleft()

    def _on_amcl(self, msg: PoseWithCovarianceStamped):
        """
        Atualiza a pose AMCL e avalia o jump detector.
        Guarda sempre a última pose conhecida para comparar com a próxima.
        """
        now = time.time()
        x   = float(msg.pose.pose.position.x)
        y   = float(msg.pose.pose.position.y)
        cov = msg.pose.covariance

        var_x   = float(cov[0])
        var_y   = float(cov[7])
        var_yaw = float(cov[35])

        # Atualiza covariância (usada na publicação periódica)
        self._last_var_x   = var_x
        self._last_var_y   = var_y
        self._last_var_yaw = var_yaw

        # Primeira pose — apenas inicializa, sem avaliar jump
        if self._last_amcl is None:
            self._last_amcl = (now, x, y)
            self.get_logger().info(
                f"[sanity] Pose inicial AMCL registrada: ({x:.3f}, {y:.3f})"
            )
            return

        # Calcula delta AMCL entre pose anterior e atual
        last_t, last_x, last_y = self._last_amcl
        delta_amcl = _dist(last_x, last_y, x, y)

        # Calcula deslocamento odom no mesmo intervalo de tempo
        delta_odom = self._odom_delta_since(last_t)

        # Avalia jump
        if delta_amcl >= self.jump_min_m:
            ratio = delta_amcl / max(delta_odom, 1e-4)
            if ratio >= self.jump_ratio:
                # Jump detectado — loga mas não toma ação aqui
                # A publicação periódica vai reportar o status
                self.get_logger().warn(
                    f"[sanity] AMCL jump detectado: "
                    f"delta_amcl={delta_amcl:.3f}m "
                    f"delta_odom={delta_odom:.3f}m "
                    f"ratio={ratio:.1f}x"
                )
                # Guarda o jump para a próxima publicação
                self._pending_jump = {
                    "delta_amcl_m": round(delta_amcl, 4),
                    "delta_odom_m": round(delta_odom, 4),
                    "ratio":        round(ratio, 2),
                    "big":          delta_amcl >= self.jump_big_m,
                }
            else:
                self._pending_jump = None
        else:
            self._pending_jump = None

        # Atualiza última pose conhecida
        self._last_amcl = (now, x, y)

    # ─────────────────────────────────────────────────────────────────
    # Helpers
    # ─────────────────────────────────────────────────────────────────

    def _odom_delta_since(self, since_t: float) -> float:
        """
        Calcula o deslocamento acumulado da odom desde since_t.
        Usa o histórico de odom mantido em _odom_hist.
        """
        # Filtra entradas após since_t
        relevant = [(t, x, y) for t, x, y in self._odom_hist if t >= since_t]

        if len(relevant) < 2:
            return 0.0

        total = 0.0
        for i in range(1, len(relevant)):
            _, x0, y0 = relevant[i - 1]
            _, x1, y1 = relevant[i]
            total += _dist(x0, y0, x1, y1)

        return total

    # ─────────────────────────────────────────────────────────────────
    # Publicação periódica
    # ─────────────────────────────────────────────────────────────────

    def _publish_sanity(self):
        now     = time.time()
        reasons = []

        # ── Camada A — jump ───────────────────────────────────────────
        jump     = getattr(self, "_pending_jump", None)
        jump_det = jump is not None
        jump_big = jump.get("big", False) if jump else False

        delta_amcl_m = jump["delta_amcl_m"] if jump else None
        delta_odom_m = jump["delta_odom_m"] if jump else None

        if jump_det:
            reasons.append(
                f"amcl_jump delta_amcl={delta_amcl_m}m "
                f"delta_odom={delta_odom_m}m "
                f"ratio={jump['ratio']}x"
            )

        # ── Camada B — covariância ────────────────────────────────────
        var_x   = self._last_var_x
        var_y   = self._last_var_y
        var_yaw = self._last_var_yaw

        cov_available = var_x is not None

        # Zona de alerta: entre warn_ratio e threshold
        warn_x   = self.max_var_x   * self.warn_ratio
        warn_y   = self.max_var_y   * self.warn_ratio
        warn_yaw = self.max_var_yaw * self.warn_ratio

        cov_lost = False
        cov_warn = False

        if cov_available:
            cov_lost = (
                var_x   > self.max_var_x   or
                var_y   > self.max_var_y   or
                var_yaw > self.max_var_yaw
            )
            if not cov_lost:
                cov_warn = (
                    var_x   > warn_x   or
                    var_y   > warn_y   or
                    var_yaw > warn_yaw
                )

            if cov_lost:
                reasons.append(
                    f"covariance_above_threshold "
                    f"var_x={var_x:.4f}(>{self.max_var_x}) "
                    f"var_y={var_y:.4f}(>{self.max_var_y}) "
                    f"var_yaw={var_yaw:.4f}(>{self.max_var_yaw})"
                )
            elif cov_warn:
                reasons.append(
                    f"covariance_warn_zone "
                    f"var_x={var_x:.4f} var_y={var_y:.4f} var_yaw={var_yaw:.4f}"
                )

        cov_ok = cov_available and not cov_lost and not cov_warn

        # ── Status final ──────────────────────────────────────────────
        #
        # LOST:    covariância acima dos thresholds
        #          OU jump muito grande (>= jump_big_m)
        #
        # SUSPECT: jump detectado (mas não grande)
        #          OU covariância na zona de alerta
        #
        # OK:      sem jump, covariância ok

        if cov_lost or jump_big:
            status = "LOST"
        elif jump_det or cov_warn:
            status = "SUSPECT"
        else:
            status = "OK"

        payload = {
            "t":      now,
            "status": status,
            "ok":     status == "OK",

            # Camada A
            "jump_detected":      jump_det,
            "jump_delta_amcl_m":  delta_amcl_m,
            "jump_delta_odom_m":  delta_odom_m,

            # Camada B
            "covariance_ok": cov_ok,
            "var_x":         round(var_x,   4) if var_x   is not None else None,
            "var_y":         round(var_y,   4) if var_y   is not None else None,
            "var_yaw":       round(var_yaw, 4) if var_yaw is not None else None,

            "reasons": reasons,
        }

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub.publish(msg)

        if status != "OK":
            self.get_logger().warn(
                f"[sanity] status={status} | reasons={reasons}"
            )

        # Limpa o jump após publicar — ele é detectado por evento,
        # não deve persistir na próxima publicação se não houve novo jump
        self._pending_jump = None


def main():
    rclpy.init()
    node = LocalizationSanityMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()