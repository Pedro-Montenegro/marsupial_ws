#!/usr/bin/env python3
"""
RobotHealthMonitor — com máquina de estados + detecção de AMCL via TF

Baseado na versão corrigida pelo GPT (machine_state). Única diferença:

  PROBLEMA CORRIGIDO: O AMCL não publica /amcl_pose quando o robô está parado
  — ele apenas atualiza o TF map→base_link continuamente. Usar TTL no
  /amcl_pose causava falsos positivos de localização perdida com robô parado.

  SOLUÇÃO: _localization_status() agora verifica o TF map→base_link para saber
  se o AMCL está vivo. O /amcl_pose continua sendo usado para covariância,
  mas SEM TTL agressivo — usa a última covariância conhecida enquanto o TF
  estiver fresco.

  Lógica resultante:
    - TF fresco + covariância ok          → LOCALIZED  (ok pleno)
    - TF fresco + sem covariância ainda   → LOCALIZED  (AMCL vivo, assume ok)
    - TF fresco + covariância zona alerta → DEGRADED
    - TF fresco + covariância acima limit → LOST → critical_fail
    - TF ausente + booting_grace          → pending
    - TF ausente fora do boot             → INITIALIZING → degraded

Todos os outros comportamentos (máquina de estados, pending, STOPPED, etc.)
são idênticos à versão GPT.

Novos parâmetros ROS:
  tf_global_frame   (padrão: "map")
  tf_robot_frame    (padrão: "base_link")
  tf_max_age_s      (padrão: 1.0) — idade máxima do TF para AMCL vivo
"""

import time
import json
from collections import deque
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from bond.msg import Status as BondStatus
from zed_msgs.msg import HealthStatusStamped, Heartbeat

# NOVO: TF
from tf2_ros import Buffer, TransformListener


NAV2_CRITICAL = {
    "controller_server",
    "planner_server",
    "bt_navigator",
    "map_server",
    "amcl",
    "velocity_smoother",
}

NAV2_NON_CRITICAL = {
    "behavior_server",
    "smoother_server",
    "waypoint_follower",
}

NAV2_COMPONENTS = {
    **{k: True  for k in NAV2_CRITICAL},
    **{k: False for k in NAV2_NON_CRITICAL},
}

BOND_TTL_MARGIN = 1.5


class MonitorState(str, Enum):
    BOOTING  = "BOOTING"
    READY    = "READY"
    DEGRADED = "DEGRADED"
    FAILED   = "FAILED"
    STOPPED  = "STOPPED"


def to_int(v, default=0):
    try:
        return int(v)
    except Exception:
        return default


def _median(values):
    s = sorted(values)
    n = len(s)
    mid = n // 2
    return s[mid] if n % 2 else (s[mid - 1] + s[mid]) / 2.0


class RobotHealthMonitor(Node):

    def __init__(self):
        super().__init__("robot_health_monitor")

        # ── parâmetros — idênticos ao original ────────────────────────
        # self.declare_parameter("scan_topic",      "/scan")
        self.declare_parameter("pub_topic",       "/robot_health")
        self.declare_parameter("zed_ttl_sec",     3.0)
        self.declare_parameter("lidar_ttl_sec",   2.0)
        self.declare_parameter("ekf_ttl_sec",     3.0)
        self.declare_parameter("drive_ttl_sec",   3.0)
        self.declare_parameter("amcl_ttl_sec",    3.0)   # mantido para compatibilidade
        self.declare_parameter("loc_max_var_x",   0.4)
        self.declare_parameter("loc_max_var_y",   0.4)
        self.declare_parameter("loc_max_var_yaw", 0.15)
        self.declare_parameter("loc_window_s",    10.0)
        self.declare_parameter("loc_min_samples", 5)
        self.declare_parameter("booting_timeout_s",                60.0)
        self.declare_parameter("stop_detection_window_s",           2.0)
        self.declare_parameter("stopped_recovery_min_ok",           3)
        self.declare_parameter("degraded_zed_timeout_s",            5.0)
        self.declare_parameter("degraded_nav2_nc_timeout_s",       10.0)
        self.declare_parameter("degraded_loc_warn_ratio",           0.5)
        self.declare_parameter("degraded_loc_timeout_s",            5.0)
        self.declare_parameter("localization_degraded_escalates",   False)
        self.declare_parameter("zed_degraded_escalates",            False)
        self.declare_parameter("nav2_noncritical_degraded_escalates", False)

        # ── parâmetros NOVOS — TF ─────────────────────────────────────
        self.declare_parameter("tf_global_frame", "map")
        self.declare_parameter("tf_robot_frame",  "base_link")
        self.declare_parameter("tf_max_age_s",    1.0)

        # Lê tudo
        # self.scan_topic      = self.get_parameter("scan_topic").value
        self.pub_topic       = self.get_parameter("pub_topic").value
        self.zed_ttl         = float(self.get_parameter("zed_ttl_sec").value)
        self.lidar_ttl       = float(self.get_parameter("lidar_ttl_sec").value)
        self.ekf_ttl         = float(self.get_parameter("ekf_ttl_sec").value)
        self.drive_ttl       = float(self.get_parameter("drive_ttl_sec").value)
        self.loc_max_var_x   = float(self.get_parameter("loc_max_var_x").value)
        self.loc_max_var_y   = float(self.get_parameter("loc_max_var_y").value)
        self.loc_max_var_yaw = float(self.get_parameter("loc_max_var_yaw").value)
        self.loc_window_s    = float(self.get_parameter("loc_window_s").value)
        self.loc_min_samples = int(self.get_parameter("loc_min_samples").value)
        self.booting_timeout_s              = float(self.get_parameter("booting_timeout_s").value)
        self.stop_window_s                  = float(self.get_parameter("stop_detection_window_s").value)
        self.stopped_recovery_min_ok        = int(self.get_parameter("stopped_recovery_min_ok").value)
        self.deg_zed_timeout                = float(self.get_parameter("degraded_zed_timeout_s").value)
        self.deg_nav2_nc_timeout            = float(self.get_parameter("degraded_nav2_nc_timeout_s").value)
        self.deg_loc_warn_ratio             = float(self.get_parameter("degraded_loc_warn_ratio").value)
        self.deg_loc_timeout                = float(self.get_parameter("degraded_loc_timeout_s").value)
        self.localization_degraded_escalates        = bool(self.get_parameter("localization_degraded_escalates").value)
        self.zed_degraded_escalates                 = bool(self.get_parameter("zed_degraded_escalates").value)
        self.nav2_noncritical_degraded_escalates    = bool(self.get_parameter("nav2_noncritical_degraded_escalates").value)
        self.tf_global_frame = self.get_parameter("tf_global_frame").value
        self.tf_robot_frame  = self.get_parameter("tf_robot_frame").value
        self.tf_max_age_s    = float(self.get_parameter("tf_max_age_s").value)

        # ── publisher ─────────────────────────────────────────────────
        self.pub = self.create_publisher(String, self.pub_topic, 10)

        # ── TF buffer — NOVO ──────────────────────────────────────────
        self._tf_buffer   = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── estado interno — idêntico ao original ─────────────────────
        self.zed = {
            "last_health_t": None, "last_heartbeat_t": None,
            "low_image_quality": False, "low_lighting": False,
            "low_depth_reliability": False, "low_motion_sensors_reliability": False,
        }
        self.lidar  = {"last_scan_t": None, "hz": None, "_prev_t": None, "_ema_hz": None}
        self.ekf    = {"last_ok_t": None, "hz": None}
        self.drive  = {"last_msg_t": None, "connected": False, "axis0": {}, "axis1": {}}
        self.nav2_bonds = {name: {"last_t": None, "timeout": 4.0} for name in NAV2_COMPONENTS}

        # Cache de covariância do AMCL.
        # Nota: /amcl_pose pode NÃO ser publicado com robô parado — tudo bem.
        # A detecção de AMCL vivo usa o TF, não o timestamp desta mensagem.
        self.amcl_hist: deque   = deque()
        self.amcl_last_pose_t   = None   # última vez que /amcl_pose chegou (sem TTL)

        # ── máquina de estados — idêntica ao original ─────────────────
        self._node_start_t  = time.time()
        self._state         = MonitorState.BOOTING
        self._state_since   = time.time()
        self._last_ok_t     = {}
        self._degraded_since = {}

        # ── subscriptions — idênticos ao original ─────────────────────
        self.create_subscription(HealthStatusStamped, "/zed/zed_node/status/health",    self._on_zed_health,    10)
        self.create_subscription(Heartbeat,           "/zed/zed_node/status/heartbeat", self._on_zed_heartbeat, 10)
        self.create_subscription(LaserScan,           "/scan",                          self._on_scan,          10)
        self.create_subscription(DiagnosticArray,     "/diagnostics",                   self._on_diag,          10)
        self.create_subscription(String,              "/ui/drive_status",               self._on_drive_status,  10)
        self.create_subscription(BondStatus,          "/bond",                          self._on_bond,          10)

        qos_amcl = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl_pose, qos_amcl)

        self.create_timer(1.0, self._publish_health)

        self.get_logger().info(
            f"[robot_health_monitor] up | pub={self.pub_topic} | "
            f"booting_timeout={self.booting_timeout_s:.1f}s | "
            f"tf={self.tf_global_frame}→{self.tf_robot_frame} max_age={self.tf_max_age_s}s"
        )

    # ─────────────────────────────────────────────────────────────────
    # Callbacks — idênticos ao original
    # ─────────────────────────────────────────────────────────────────

    def _on_zed_health(self, msg: HealthStatusStamped):
        now = time.time()
        self.zed["last_health_t"]                  = now
        self.zed["low_image_quality"]              = bool(msg.low_image_quality)
        self.zed["low_lighting"]                   = bool(msg.low_lighting)
        self.zed["low_depth_reliability"]          = bool(msg.low_depth_reliability)
        self.zed["low_motion_sensors_reliability"] = bool(msg.low_motion_sensors_reliability)

    def _on_zed_heartbeat(self, msg: Heartbeat):
        self.zed["last_heartbeat_t"] = time.time()

    def _on_scan(self, msg: LaserScan):
        now = time.time()
        self.lidar["last_scan_t"] = now
        prev = self.lidar["_prev_t"]
        if prev is not None:
            dt = now - prev
            if dt > 1e-3:
                hz_inst = 1.0 / dt
                ema     = self.lidar["_ema_hz"]
                self.lidar["_ema_hz"] = hz_inst if ema is None else 0.2 * hz_inst + 0.8 * ema
                self.lidar["hz"] = round(self.lidar["_ema_hz"], 2)
        self.lidar["_prev_t"] = now

    def _on_drive_status(self, msg: String):
        now = time.time()
        try:
            data = json.loads(msg.data)
            self.drive["last_msg_t"] = now
            self.drive["connected"]  = bool(data.get("connected", False))
            self.drive["axis0"]      = data.get("axis0", {})
            self.drive["axis1"]      = data.get("axis1", {})
        except Exception as e:
            self.get_logger().warn(f"[drive_status] parse error: {e}")

    def _on_diag(self, msg: DiagnosticArray):
        now = time.time()
        for st in msg.status:
            name    = (st.name    or "").lower()
            message = (st.message or "").lower()
            ok      = to_int(st.level, 0) == 0
            if "ekf_odom" in name:
                if "functioning properly" in message and ok:
                    self.ekf["last_ok_t"] = now
                for kv in st.values:
                    if (kv.key or "").lower() == "actual frequency (hz)":
                        try:
                            self.ekf["hz"] = round(float(kv.value), 2)
                        except Exception:
                            pass

    def _on_bond(self, msg: BondStatus):
        component = msg.id
        if component not in self.nav2_bonds:
            return
        if not msg.active:
            self.nav2_bonds[component]["last_t"] = None
            return
        now = time.time()
        self.nav2_bonds[component]["last_t"]  = now
        self.nav2_bonds[component]["timeout"] = float(msg.heartbeat_timeout)

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        """
        Atualiza o cache de covariância.
        NÃO usar o timestamp desta mensagem para detectar AMCL vivo —
        o AMCL pode parar de publicar /amcl_pose com o robô parado.
        A detecção de AMCL vivo é feita via TF em _amcl_alive_via_tf().
        """
        now = time.time()
        self.amcl_last_pose_t = now
        cov = msg.pose.covariance
        self.amcl_hist.append((now, float(cov[0]), float(cov[7]), float(cov[35])))
        while self.amcl_hist and (now - self.amcl_hist[0][0]) > self.loc_window_s:
            self.amcl_hist.popleft()

    # ─────────────────────────────────────────────────────────────────
    # Detecção de AMCL via TF — NOVO
    # ─────────────────────────────────────────────────────────────────

    def _amcl_alive_via_tf(self) -> tuple:
        """
        Verifica se o AMCL está publicando o TF map→base_link.
        O AMCL mantém este TF atualizado mesmo com o robô completamente parado.

        Retorna (alive: bool, tf_age_s: float|None)
        """
        try:
            tr = self._tf_buffer.lookup_transform(
                self.tf_global_frame,
                self.tf_robot_frame,
                rclpy.time.Time(),  # tempo zero = mais recente disponível
            )
            tf_stamp_ns = (
                tr.header.stamp.sec * 1_000_000_000 +
                tr.header.stamp.nanosec
            )
            now_ns = int(time.time() * 1_000_000_000)
            age_s  = (now_ns - tf_stamp_ns) / 1_000_000_000.0
            return age_s <= self.tf_max_age_s, round(age_s, 3)
        except Exception:
            return False, None

    # ─────────────────────────────────────────────────────────────────
    # _localization_status — CORRIGIDA para usar TF
    # ─────────────────────────────────────────────────────────────────

    def _localization_status(self, now: float):
        """
        Retorna (loc_ok_raw, loc_status, var_x, var_y, var_yaw, amcl_alive, tf_age_s).

        Diferença em relação ao original:
          - AMCL vivo = TF map→base_link fresco (não TTL do /amcl_pose)
          - Covariância usa histórico acumulado sem TTL — a última covariância
            conhecida é válida enquanto o AMCL estiver vivo via TF
        """
        amcl_alive, tf_age_s = self._amcl_alive_via_tf()

        if not amcl_alive:
            return False, "INITIALIZING", None, None, None, False, tf_age_s

        # AMCL vivo — avalia covariância se temos amostras suficientes
        if len(self.amcl_hist) < self.loc_min_samples:
            # Ainda não acumulou amostras — AMCL vivo mas sem covariância
            # Considera LOCALIZED: o TF garante que o AMCL está funcionando
            return True, "LOCALIZED", None, None, None, True, tf_age_s

        mx   = round(_median([e[1] for e in self.amcl_hist]), 4)
        my   = round(_median([e[2] for e in self.amcl_hist]), 4)
        myaw = round(_median([e[3] for e in self.amcl_hist]), 4)

        localized = (
            mx   <= self.loc_max_var_x   and
            my   <= self.loc_max_var_y   and
            myaw <= self.loc_max_var_yaw
        )
        status = "LOCALIZED" if localized else "LOST"
        return localized, status, mx, my, myaw, True, tf_age_s

    # ─────────────────────────────────────────────────────────────────
    # Helpers — idênticos ao original
    # ─────────────────────────────────────────────────────────────────

    def _ttl_ok(self, last_t, ttl, now):
        return last_t is not None and (now - last_t) <= ttl

    def _booting_grace_active(self, now):
        return self._state == MonitorState.BOOTING and \
               (now - self._node_start_t) < self.booting_timeout_s

    def _make_result(self, *, ok, degraded=False, critical_fail=False, pending=False,
                     degraded_reason=None, degraded_timeout=None, escalates=False, payload=None):
        return {
            "ok":                 bool(ok),
            "degraded":           bool(degraded),
            "critical_fail":      bool(critical_fail),
            "pending":            bool(pending),
            "degraded_reason":    degraded_reason,
            "degraded_timeout":   degraded_timeout,
            "degraded_escalates": bool(escalates),
            "payload":            payload or {},
        }

    # ─────────────────────────────────────────────────────────────────
    # _eval_subsystems — idêntico ao original exceto localização
    # ─────────────────────────────────────────────────────────────────

    def _eval_subsystems(self, now: float) -> dict:
        results = {}
        booting_grace = self._booting_grace_active(now)

        # ── ZED — idêntico ───────────────────────────────────────────
        zed_has_health    = self.zed["last_health_t"] is not None
        zed_has_heartbeat = self.zed["last_heartbeat_t"] is not None
        zed_health_ok     = self._ttl_ok(self.zed["last_health_t"],    self.zed_ttl, now)
        zed_heartbeat_ok  = self._ttl_ok(self.zed["last_heartbeat_t"], self.zed_ttl, now)
        zed_missing       = not zed_has_health or not zed_has_heartbeat
        zed_pending       = booting_grace and zed_missing
        zed_img_ok        = not self.zed["low_image_quality"]
        zed_motion_ok     = not self.zed["low_motion_sensors_reliability"]
        zed_offline       = not (zed_health_ok and zed_heartbeat_ok)
        zed_critical      = (not zed_pending) and (zed_offline or not zed_img_ok or not zed_motion_ok)
        zed_degraded      = (not zed_pending) and (not zed_critical) and (
            self.zed["low_lighting"] or self.zed["low_depth_reliability"]
        )
        zed_ok = not zed_pending and not zed_critical and not zed_degraded

        results["zed"] = self._make_result(
            ok=zed_ok, degraded=zed_degraded, critical_fail=zed_critical, pending=zed_pending,
            degraded_reason="zed_quality" if zed_degraded else None,
            degraded_timeout=self.deg_zed_timeout, escalates=self.zed_degraded_escalates,
            payload={
                "ok": zed_ok, "degraded": zed_degraded, "critical_fail": zed_critical, "pending": zed_pending,
                "heartbeat_ok": zed_heartbeat_ok, "health_ok": zed_health_ok,
                "low_image_quality":              self.zed["low_image_quality"],
                "low_lighting":                   self.zed["low_lighting"],
                "low_depth_reliability":          self.zed["low_depth_reliability"],
                "low_motion_sensors_reliability": self.zed["low_motion_sensors_reliability"],
            },
        )

        # ── LiDAR — idêntico ─────────────────────────────────────────
        lidar_has_msg  = self.lidar["last_scan_t"] is not None
        lidar_pending  = booting_grace and not lidar_has_msg
        lidar_alive    = self._ttl_ok(self.lidar["last_scan_t"], self.lidar_ttl, now)
        lidar_critical = (not lidar_pending) and (not lidar_alive)
        lidar_ok       = not lidar_pending and not lidar_critical

        results["lidar"] = self._make_result(
            ok=lidar_ok, critical_fail=lidar_critical, pending=lidar_pending,
            payload={
                "ok": lidar_ok, "degraded": False, "critical_fail": lidar_critical,
                "pending": lidar_pending, "hz": self.lidar["hz"],
            },
        )

        # ── EKF — idêntico ───────────────────────────────────────────
        ekf_has_msg  = self.ekf["last_ok_t"] is not None
        ekf_pending  = booting_grace and not ekf_has_msg
        ekf_alive    = self._ttl_ok(self.ekf["last_ok_t"], self.ekf_ttl, now)
        ekf_critical = (not ekf_pending) and (not ekf_alive)
        ekf_ok       = not ekf_pending and not ekf_critical

        results["ekf"] = self._make_result(
            ok=ekf_ok, critical_fail=ekf_critical, pending=ekf_pending,
            payload={
                "ok": ekf_ok, "degraded": False, "critical_fail": ekf_critical,
                "pending": ekf_pending, "hz": self.ekf["hz"],
            },
        )

        # ── Nav2 — idêntico ──────────────────────────────────────────
        components              = {}
        component_pending       = {}
        nav2_critical_fail      = False
        nav2_nc_degraded        = False
        nav2_critical_pending   = False
        nav2_noncritical_pending = False

        for name, is_critical in NAV2_COMPONENTS.items():
            bond          = self.nav2_bonds[name]
            last_t        = bond["last_t"]
            timeout       = float(bond["timeout"]) + BOND_TTL_MARGIN
            has_ever_seen = last_t is not None
            comp_pending  = booting_grace and not has_ever_seen
            comp_ok       = has_ever_seen and (now - last_t) <= timeout
            components[name]        = bool(comp_ok)
            component_pending[name] = bool(comp_pending)

            if comp_pending:
                if is_critical: nav2_critical_pending    = True
                else:           nav2_noncritical_pending = True
                continue
            if not comp_ok:
                if is_critical: nav2_critical_fail = True
                else:           nav2_nc_degraded   = True

        nav2_pending  = booting_grace and nav2_critical_pending
        nav2_degraded = (not nav2_pending) and (not nav2_critical_fail) and (
            nav2_nc_degraded or nav2_noncritical_pending
        )
        nav2_ok = not nav2_pending and not nav2_critical_fail and not nav2_degraded

        results["nav2"] = self._make_result(
            ok=nav2_ok, degraded=nav2_degraded, critical_fail=nav2_critical_fail, pending=nav2_pending,
            degraded_reason="nav2_non_critical" if nav2_degraded else None,
            degraded_timeout=self.deg_nav2_nc_timeout, escalates=self.nav2_noncritical_degraded_escalates,
            payload={
                "ok": nav2_ok, "degraded": nav2_degraded, "critical_fail": nav2_critical_fail,
                "pending": nav2_pending, "components": components, "component_pending": component_pending,
            },
        )

        # ── Drive — idêntico ─────────────────────────────────────────
        drive_has_msg  = self.drive["last_msg_t"] is not None
        drive_pending  = booting_grace and not drive_has_msg
        drive_msg_ok   = self._ttl_ok(self.drive["last_msg_t"], self.drive_ttl, now)
        a0, a1         = self.drive["axis0"], self.drive["axis1"]
        axis0_ok       = a0.get("connected", False) and not a0.get("has_error", False)
        axis1_ok       = a1.get("connected", False) and not a1.get("has_error", False)
        drive_alive    = drive_msg_ok and axis0_ok and axis1_ok
        drive_critical = (not drive_pending) and (not drive_alive)
        drive_ok       = not drive_pending and not drive_critical

        results["drive"] = self._make_result(
            ok=drive_ok, critical_fail=drive_critical, pending=drive_pending,
            payload={
                "ok": drive_ok, "degraded": False, "critical_fail": drive_critical,
                "pending": drive_pending, "axis0": a0, "axis1": a1,
            },
        )

        # ── Localização — CORRIGIDA para usar TF ─────────────────────
        loc_ok_raw, loc_status, var_x, var_y, var_yaw, amcl_alive, tf_age_s = \
            self._localization_status(now)

        loc_pending  = (not amcl_alive) and booting_grace
        loc_warn     = False

        if loc_status == "LOCALIZED" and var_x is not None:
            warn_x   = self.loc_max_var_x   * self.deg_loc_warn_ratio
            warn_y   = self.loc_max_var_y   * self.deg_loc_warn_ratio
            warn_yaw = self.loc_max_var_yaw * self.deg_loc_warn_ratio
            loc_warn = var_x > warn_x or var_y > warn_y or var_yaw > warn_yaw

        loc_critical = (not loc_pending) and (loc_status == "LOST")
        loc_degraded = (not loc_pending) and (not loc_critical) and (
            loc_warn or (loc_status == "INITIALIZING" and not loc_pending)
        )
        loc_ok = loc_ok_raw and not loc_warn and not loc_pending and not loc_degraded and not loc_critical

        results["localization"] = self._make_result(
            ok=loc_ok, degraded=loc_degraded, critical_fail=loc_critical, pending=loc_pending,
            degraded_reason="localization_uncertain" if loc_degraded else None,
            degraded_timeout=self.deg_loc_timeout, escalates=self.localization_degraded_escalates,
            payload={
                "ok": loc_ok, "degraded": loc_degraded, "critical_fail": loc_critical,
                "pending": loc_pending, "status": loc_status,
                "amcl_alive": amcl_alive, "tf_age_s": tf_age_s,
                "var_x": var_x, "var_y": var_y, "var_yaw": var_yaw,
            },
        )

        return results

    # ─────────────────────────────────────────────────────────────────
    # Máquina de estados e publicação — idênticos ao original
    # ─────────────────────────────────────────────────────────────────

    def _detect_stop(self, eval_results: dict, now: float) -> bool:
        if self._state not in (MonitorState.READY, MonitorState.DEGRADED):
            return False
        just_failed = 0
        for name, res in eval_results.items():
            if res["critical_fail"]:
                last_ok = self._last_ok_t.get(name)
                if last_ok is not None and (now - last_ok) <= self.stop_window_s:
                    just_failed += 1
        return just_failed >= 3

    def _update_last_ok(self, eval_results: dict, now: float):
        for name, res in eval_results.items():
            if res["ok"]:
                self._last_ok_t[name] = now

    def _update_degraded_timers(self, eval_results: dict, now: float) -> dict:
        active = {}
        for name, res in eval_results.items():
            reason = res.get("degraded_reason")
            if not reason:
                continue
            if reason not in active:
                active[reason] = {
                    "timeout": res.get("degraded_timeout") or 999.0,
                    "escalates": bool(res.get("degraded_escalates", False)),
                    "subsystems": [],
                }
            active[reason]["subsystems"].append(name)
            active[reason]["escalates"] = active[reason]["escalates"] or \
                                          bool(res.get("degraded_escalates", False))

        for reason in list(self._degraded_since.keys()):
            if reason not in active:
                del self._degraded_since[reason]

        out = {}
        for reason, meta in active.items():
            if reason not in self._degraded_since:
                self._degraded_since[reason] = now
            out[reason] = {
                "elapsed":    now - self._degraded_since[reason],
                "timeout":    float(meta["timeout"]),
                "escalates":  bool(meta["escalates"]),
                "subsystems": list(meta["subsystems"]),
            }
        return out

    def _transition(self, new_state: MonitorState):
        if new_state != self._state:
            self.get_logger().info(
                f"[health_monitor] {self._state.value} → {new_state.value}"
            )
            self._state       = new_state
            self._state_since = time.time()

    def _run_state_machine(self, eval_results: dict, overall_ok: bool, now: float) -> dict:
        uptime_s      = now - self._node_start_t
        failed_subs   = [n for n, r in eval_results.items() if r["critical_fail"]]
        degraded_subs = [n for n, r in eval_results.items() if r.get("degraded_reason")]
        pending_subs  = [n for n, r in eval_results.items() if r.get("pending")]

        self._update_last_ok(eval_results, now)
        degraded_timers = self._update_degraded_timers(eval_results, now)

        degraded_expired_reasons = [
            reason for reason, meta in degraded_timers.items()
            if meta["escalates"] and meta["elapsed"] >= meta["timeout"]
        ]
        degraded_expired_subs = []
        for reason in degraded_expired_reasons:
            degraded_expired_subs.extend(degraded_timers[reason]["subsystems"])

        stop_detected = self._detect_stop(eval_results, now)

        if self._state == MonitorState.BOOTING:
            if overall_ok:
                self._degraded_since.clear()
                self._transition(MonitorState.READY)
            elif uptime_s >= self.booting_timeout_s:
                if failed_subs or pending_subs:
                    self._transition(MonitorState.FAILED)
                elif degraded_subs:
                    self._transition(MonitorState.DEGRADED)

        elif self._state == MonitorState.READY:
            if stop_detected:
                self._degraded_since.clear()
                self._transition(MonitorState.STOPPED)
            elif failed_subs:
                self._degraded_since.clear()
                self._transition(MonitorState.FAILED)
            elif degraded_subs:
                self._transition(MonitorState.DEGRADED)

        elif self._state == MonitorState.DEGRADED:
            if stop_detected:
                self._degraded_since.clear()
                self._transition(MonitorState.STOPPED)
            elif failed_subs:
                self._degraded_since.clear()
                self._transition(MonitorState.FAILED)
            elif degraded_expired_reasons:
                self._degraded_since.clear()
                self._transition(MonitorState.FAILED)
            elif not degraded_subs:
                self._degraded_since.clear()
                if pending_subs:
                    self._transition(MonitorState.BOOTING)
                else:
                    self._transition(MonitorState.READY)

        elif self._state == MonitorState.FAILED:
            if not failed_subs:
                self._degraded_since.clear()
                self._last_ok_t.clear()
                self._transition(MonitorState.BOOTING)

        elif self._state == MonitorState.STOPPED:
            recovered_ok = sum(1 for r in eval_results.values() if r["ok"])
            if recovered_ok >= self.stopped_recovery_min_ok:
                self._degraded_since.clear()
                self._last_ok_t.clear()
                self._transition(MonitorState.BOOTING)

        return {
            "state":                       self._state.value,
            "state_since_s":               round(now - self._state_since, 1),
            "uptime_s":                    round(uptime_s, 1),
            "failed_subsystems":           failed_subs,
            "degraded_subsystems":         degraded_subs,
            "pending_subsystems":          pending_subs,
            "degraded_expired_subsystems": sorted(set(degraded_expired_subs)),
            "stop_detected":               stop_detected,
        }

    def _publish_health(self):
        now          = time.time()
        eval_results = self._eval_subsystems(now)
        overall_ok   = all(r["ok"] for r in eval_results.values())

        monitor_payload = self._run_state_machine(eval_results, overall_ok, now)

        payload = {"t": now, "overall_ok": overall_ok, "monitor": monitor_payload}
        for name, res in eval_results.items():
            payload[name] = res["payload"]

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub.publish(msg)

        if not overall_ok or self._state != MonitorState.READY:
            self.get_logger().info(
                f"[health] state={self._state.value} | "
                f"failed={monitor_payload['failed_subsystems']} | "
                f"degraded={monitor_payload['degraded_subsystems']} | "
                f"pending={monitor_payload['pending_subsystems']} | "
                f"expired={monitor_payload['degraded_expired_subsystems']}"
            )


def main():
    rclpy.init()
    node = RobotHealthMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()