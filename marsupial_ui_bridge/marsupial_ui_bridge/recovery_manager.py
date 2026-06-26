#!/usr/bin/env python3
"""
RecoveryManager

Ramos principais:
- localization.status = LOST persistente:
    cancela/pausa missão, para robô e executa relocalization_recovery.py

- nav2.ok = false persistente:
    cancela missão, para robô e reinicia marsupial-stack.target

- ekf.ok = false persistente:
    cancela missão, para robô e reinicia marsupial-stack.target

- drive.ok = false persistente:
    cancela missão, para robô e alerta operador

- zed/lidar:
    por enquanto apenas alerta
"""

import json
import shlex
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist


NAV2_FAIL_THRESHOLD = 5
LOC_FAIL_THRESHOLD = 1
EKF_FAIL_THRESHOLD = 5
DRIVE_FAIL_THRESHOLD = 3

RECOVERY_COOLDOWN_S = 60.0

ACTIVE_MISSION_STATES = {
    "RUNNING", "PAUSED", "RETURNING",
    "AT_STATION", "WAITING",
}

RESTART_CMD = ["systemctl", "restart", "marsupial-stack.target"]


class RecoveryManager(Node):

    def __init__(self):
        super().__init__("recovery_manager")

        self.declare_parameter("nav2_fail_threshold", NAV2_FAIL_THRESHOLD)
        self.declare_parameter("loc_fail_threshold", LOC_FAIL_THRESHOLD)
        self.declare_parameter("ekf_fail_threshold", EKF_FAIL_THRESHOLD)
        self.declare_parameter("drive_fail_threshold", DRIVE_FAIL_THRESHOLD)
        self.declare_parameter("lidar_fail_threshold", 3)
        self.declare_parameter("lidar_wait_s", 15.0)
        self.declare_parameter("recovery_cooldown_s", RECOVERY_COOLDOWN_S)
        self.declare_parameter("dry_run", False)

        # Comando para chamar seu recovery de relocalização
        self.declare_parameter(
            "relocalization_cmd",
            "ros2 run marsupial_robot_localization reinitialize_global_localization"
        )

        self.declare_parameter("loc_recovery_timeout_s", 60.0)
        self.declare_parameter("loc_recovery_verify_s", 10.0)

        self.nav2_thresh = int(self.get_parameter("nav2_fail_threshold").value)
        self.loc_thresh = int(self.get_parameter("loc_fail_threshold").value)
        self.ekf_thresh = int(self.get_parameter("ekf_fail_threshold").value)
        self.drive_thresh = int(self.get_parameter("drive_fail_threshold").value)
        self.lidar_thresh  = int(self.get_parameter("lidar_fail_threshold").value)
        self.lidar_wait_s  = float(self.get_parameter("lidar_wait_s").value)
        self.cooldown_s = float(self.get_parameter("recovery_cooldown_s").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)

        self.relocalization_cmd = str(self.get_parameter("relocalization_cmd").value)
        self.loc_recovery_timeout_s = float(self.get_parameter("loc_recovery_timeout_s").value)
        self.loc_recovery_verify_s = float(self.get_parameter("loc_recovery_verify_s").value)

        self.pub_ctrl = self.create_publisher(String, "/mission_ctrl", 10)
        self.pub_cmdvel = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_recovery = self.create_publisher(String, "/ui/recovery_event", 10)

        self.create_subscription(String, "/robot_health", self._on_health, 10)
        self.create_subscription(String, "/ui/progress", self._on_progress, 10)

        self._mission_state = "IDLE"

        self._fail_counts = {
            "nav2": 0,
            "loc": 0,
            "ekf": 0,
            "drive": 0,
            "lidar": 0,
        }

        self._last_recovery_t = 0.0
        self._recovery_in_progress = False
        self._latest_health = None

        # Sanity monitor — substitui localização do robot_health
        self._sanity_status = "OK"
        self.create_subscription(String, "/localization_sanity", self._on_sanity, 10)

        if self.dry_run:
            self.get_logger().warn("[recovery_manager] DRY RUN ativo.")

        self.get_logger().info(
            f"[recovery_manager] up | nav2={self.nav2_thresh}s "
            f"loc={self.loc_thresh}s ekf={self.ekf_thresh}s "
            f"drive={self.drive_thresh}s"
        )

    def _on_progress(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        if data.get("event") == "state":
            self._mission_state = data.get("status", "IDLE")

    def _on_sanity(self, msg: String):
        try:
            data = json.loads(msg.data)
            self._sanity_status = data.get("status", "OK")
        except Exception:
            pass

    def _on_health(self, msg: String):
        if self._recovery_in_progress:
            return

        try:
            health = json.loads(msg.data)
        except Exception:
            self.get_logger().warn("[recovery_manager] Falha ao parsear /robot_health")
            return

        self._latest_health = health

        mission_active = self._mission_state in ACTIVE_MISSION_STATES

        nav2_ok = health.get("nav2", {}).get("ok", True)
        ekf_ok = health.get("ekf", {}).get("ok", True)
        drive_ok = health.get("drive", {}).get("ok", True)

        lidar_ok = health.get("lidar", {}).get("ok", True)
        self._fail_counts["lidar"] = 0 if lidar_ok else self._fail_counts["lidar"] + 1

        lidar_critical = self._fail_counts["lidar"] >= self.lidar_thresh

        if lidar_critical:
            self._fail_counts["lidar"] = 0
            self._start_recovery_thread(
                self._execute_lidar_recovery,
                mission_active,
            )
            return

        #loc_status = health.get("localization", {}).get("status", "LOCALIZED")
        #loc_lost = loc_status == "LOST"

        # Localização avaliada pelo sanity monitor, não pelo robot_health
        loc_lost = self._sanity_status == "LOST"
        self._fail_counts["loc"] = 0 if not loc_lost else self._fail_counts["loc"] + 1

        zed_ok = health.get("zed", {}).get("ok", True)
        lidar_ok = health.get("lidar", {}).get("ok", True)

        self._fail_counts["nav2"] = 0 if nav2_ok else self._fail_counts["nav2"] + 1
        self._fail_counts["ekf"] = 0 if ekf_ok else self._fail_counts["ekf"] + 1
        self._fail_counts["drive"] = 0 if drive_ok else self._fail_counts["drive"] + 1
        #self._fail_counts["loc"] = 0 if not loc_lost else self._fail_counts["loc"] + 1

        for name, count in self._fail_counts.items():
            if count > 0:
                thresh = {
                    "nav2": self.nav2_thresh,
                    "loc": self.loc_thresh,
                    "ekf": self.ekf_thresh,
                    "drive": self.drive_thresh,
                    "lidar": self.lidar_thresh,
                }[name]
                self.get_logger().warn(
                    f"[recovery_manager] {name} falha há {count}/{thresh} ciclos"
                )

        if not zed_ok:
            self._publish_alert("zed_degraded", "ZED com falha — monitorando.")

        if not lidar_ok:
            self._publish_alert("lidar_offline", "LiDAR offline — monitorando.")

        if self._fail_counts["drive"] >= self.drive_thresh:
            self._fail_counts["drive"] = 0
            self._start_recovery_thread(
                self._execute_drive_failure,
                mission_active,
            )
            return

        loc_critical = self._fail_counts["loc"] >= self.loc_thresh
        nav2_critical = self._fail_counts["nav2"] >= self.nav2_thresh
        ekf_critical = self._fail_counts["ekf"] >= self.ekf_thresh

        if loc_critical:
            self._fail_counts["loc"] = 0
            self._start_recovery_thread(
                self._execute_localization_recovery,
                mission_active,
            )
            return

        if nav2_critical or ekf_critical:
            reason = "nav2 offline" if nav2_critical else "EKF offline"
            subsystem = "nav2" if nav2_critical else "ekf"
            self._fail_counts[subsystem] = 0

            self._start_recovery_thread(
                self._execute_stack_restart_recovery,
                reason,
                mission_active,
            )
    
    def _execute_lidar_recovery(self, mission_active: bool):
        self.get_logger().error("[recovery_manager] LiDAR offline persistente.")

        self._publish_alert(
            "lidar_offline",
            "LiDAR offline. Pausando missão e aguardando retorno.",
        )

        if mission_active:
            self._pause_mission()
            time.sleep(0.5)

        self._stop_robot(duration_s=1.0)

        # Aguarda o LiDAR voltar sozinho
        self.get_logger().info(
            f"[recovery_manager] Aguardando LiDAR voltar ({self.lidar_wait_s}s)..."
        )
        recovered = self._wait_lidar_recovered(self.lidar_wait_s)

        if recovered:
            self.get_logger().info("[recovery_manager] LiDAR voltou.")
            if mission_active:
                self._resume_mission()
            self._publish_alert(
                "lidar_recovered",
                "LiDAR voltou. Missão retomada.",
            )
            return

        # Não voltou — restart do stack
        self.get_logger().error(
            "[recovery_manager] LiDAR não voltou. Reiniciando stack."
        )
        if mission_active:
            self._cancel_mission()

        self._publish_alert(
            "lidar_restart",
            "LiDAR não voltou. Reiniciando stack.",
        )

        if not self.dry_run:
            try:
                subprocess.run(RESTART_CMD, timeout=10.0, capture_output=True, text=True)
            except Exception as e:
                self._publish_alert("lidar_restart_error", f"Erro ao reiniciar stack: {e}")
        else:
            self.get_logger().warn(
                f"[recovery_manager] DRY RUN — comando seria: {' '.join(RESTART_CMD)}"
            )

    def _wait_lidar_recovered(self, timeout_s: float) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            health = self._latest_health or {}
            if health.get("lidar", {}).get("ok", False):
                return True
            time.sleep(0.5)
        return False

    def _start_recovery_thread(self, target, *args):
        if self._recovery_in_progress:
            return

        now = time.time()
        if (now - self._last_recovery_t) < self.cooldown_s:
            remaining = self.cooldown_s - (now - self._last_recovery_t)
            self.get_logger().warn(
                f"[recovery_manager] Recovery em cooldown ({remaining:.0f}s restantes)."
            )
            return

        self._recovery_in_progress = True
        self._last_recovery_t = now

        th = threading.Thread(target=self._recovery_wrapper, args=(target, *args), daemon=True)
        th.start()

    def _recovery_wrapper(self, target, *args):
        try:
            target(*args)
        except Exception as e:
            self.get_logger().error(f"[recovery_manager] Erro no recovery: {e}")
        finally:
            self._recovery_in_progress = False

    def _pause_mission(self):
        msg = String()
        msg.data = "PAUSE"
        self.pub_ctrl.publish(msg)
        self.get_logger().info("[recovery_manager] PAUSE publicado em /mission_ctrl.")

    def _resume_mission(self):
        msg = String()
        msg.data = "RESUME"
        self.pub_ctrl.publish(msg)
        self.get_logger().info("[recovery_manager] RESUME publicado em /mission_ctrl.")

    def _execute_localization_recovery(self, mission_active: bool):
        self.get_logger().error(
            "[recovery_manager] Localização LOST persistente. "
            "Iniciando recovery de relocalização."
        )

        self._publish_alert(
            "localization_lost",
            "Localização perdida. Tentando relocalização automática.",
        )

        # Fase 1: pausa a missão sem cancelar — guarda o goal para retomar depois
        if mission_active:
            self._pause_mission()
            time.sleep(0.5)

        self._stop_robot(duration_s=1.0)

        if self.dry_run:
            self.get_logger().warn(
                f"[recovery_manager] DRY RUN — executaria: {self.relocalization_cmd}"
            )
            return

        try:
            cmd = shlex.split(self.relocalization_cmd)

            result = subprocess.run(
                cmd,
                timeout=self.loc_recovery_timeout_s,
                capture_output=True,
                text=True,
            )

            if result.stdout.strip():
                self.get_logger().info(result.stdout.strip())

            if result.stderr.strip():
                self.get_logger().warn(result.stderr.strip())

            if result.returncode != 0:
                # Relocalização falhou — agora sim cancela a missão
                if mission_active:
                    self._cancel_mission()
                self._publish_alert(
                    "localization_recovery_failed",
                    "Recovery de relocalização terminou com erro. Missão cancelada. Intervenção necessária.",
                )
                return

        except subprocess.TimeoutExpired:
            self.get_logger().error("[recovery_manager] Timeout no recovery de relocalização.")
            if mission_active:
                self._cancel_mission()
            self._publish_alert(
                "localization_recovery_timeout",
                "Timeout na relocalização automática. Missão cancelada. Intervenção necessária.",
            )
            return

        except Exception as e:
            self.get_logger().error(f"[recovery_manager] Erro chamando relocalização: {e}")
            if mission_active:
                self._cancel_mission()
            self._publish_alert(
                "localization_recovery_error",
                "Erro ao executar relocalização automática. Missão cancelada.",
            )
            return

        recovered = self._wait_localization_recovered(self.loc_recovery_verify_s)

        if recovered:
            # Fase 2a: relocalizado — retoma a missão de onde parou
            if mission_active:
                self._resume_mission()
                self._publish_alert(
                    "localization_recovered",
                    "Robô relocalizado com sucesso. Missão retomada.",
                )
            else:
                self._publish_alert(
                    "localization_recovered",
                    "Robô relocalizado com sucesso.",
                )
            self.get_logger().info("[recovery_manager] Localização recuperada.")
        else:
            # Fase 2b: não estabilizou — cancela a missão agora
            if mission_active:
                self._cancel_mission()
            self._publish_alert(
                "localization_recovery_uncertain",
                "Relocalização executada, mas localização não estabilizou. Missão cancelada. Intervenção necessária.",
            )
            self.get_logger().warn("[recovery_manager] Relocalização ainda incerta.")

    def _execute_stack_restart_recovery(self, reason: str, mission_active: bool):
        self.get_logger().error(
            f"[recovery_manager] Falha crítica persistente: {reason}. "
            "Reiniciando stack."
        )

        self._publish_alert(
            "stack_recovery_started",
            f"Falha crítica: {reason}. Reiniciando stack.",
        )

        if mission_active:
            self._cancel_mission()
            time.sleep(0.5)

        self._stop_robot(duration_s=1.0)

        if self.dry_run:
            self.get_logger().warn(
                f"[recovery_manager] DRY RUN — comando seria: {' '.join(RESTART_CMD)}"
            )
            return

        try:
            result = subprocess.run(
                RESTART_CMD,
                timeout=10.0,
                capture_output=True,
                text=True,
            )

            if result.returncode == 0:
                self._publish_alert(
                    "stack_restarted",
                    f"Stack reiniciado devido a: {reason}.",
                )
            else:
                self._publish_alert(
                    "stack_restart_failed",
                    f"Falha ao reiniciar stack: {result.stderr.strip()}",
                )

        except Exception as e:
            self._publish_alert(
                "stack_restart_error",
                f"Erro ao reiniciar stack: {e}",
            )

    def _execute_drive_failure(self, mission_active: bool):
        self.get_logger().error("[recovery_manager] Drive com falha persistente.")

        if mission_active:
            self._cancel_mission()
            time.sleep(0.5)

        self._stop_robot(duration_s=1.0)

        self._publish_alert(
            "drive_failure",
            "Falha persistente no drive. Missão cancelada. Verifique o ODrive.",
        )

    def _wait_localization_recovered(self, timeout_s: float) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            if self._sanity_status == "OK":
                return True
            time.sleep(0.2)
        return False

    def _cancel_mission(self):
        msg = String()
        msg.data = "CANCEL"
        self.pub_ctrl.publish(msg)
        self.get_logger().info("[recovery_manager] CANCEL publicado em /mission_ctrl.")

    def _stop_robot(self, duration_s: float = 1.0):
        twist = Twist()
        end = time.time() + duration_s

        while time.time() < end:
            self.pub_cmdvel.publish(twist)
            time.sleep(0.05)

    def _publish_alert(self, event: str, message: str):
        payload = {
            "event": event,
            "message": message,
            "t": time.time(),
        }

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub_recovery.publish(msg)

        self.get_logger().warn(f"[recovery_event] {event}: {message}")


def main(args=None):
    rclpy.init(args=args)
    node = RecoveryManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
