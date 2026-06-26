#!/usr/bin/env python3
import time
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class RobotStateManager(Node):
    def __init__(self):
        super().__init__("robot_state_manager")

        # ---------------- parâmetros ----------------
        self.declare_parameter("health_topic", "/robot_health")
        self.declare_parameter("service_topic", "/ui/service_status")
        self.declare_parameter("progress_topic", "/ui/progress")
        self.declare_parameter("control_mode_topic", "/ui/control_mode")
        self.declare_parameter("manual_cmd_topic", "/ui/manual_cmd")
        self.declare_parameter("pub_topic", "/robot_operational_state")

        self.declare_parameter("health_ttl_sec", 3.0)
        self.declare_parameter("service_ttl_sec", 5.0)
        self.declare_parameter("control_ttl_sec", 5.0)
        self.declare_parameter("manual_cmd_ttl_sec", 0.7)

        self.declare_parameter("mission_active_ttl_sec", 2.0)
        self.declare_parameter("stack_start_grace_sec", 40.0)
        self.declare_parameter("abort_display_sec", 5.0)

        self.health_topic = self.get_parameter("health_topic").value
        self.service_topic = self.get_parameter("service_topic").value
        self.progress_topic = self.get_parameter("progress_topic").value
        self.control_mode_topic = self.get_parameter("control_mode_topic").value
        self.manual_cmd_topic = self.get_parameter("manual_cmd_topic").value
        self.pub_topic = self.get_parameter("pub_topic").value

        self.health_ttl = float(self.get_parameter("health_ttl_sec").value)
        self.service_ttl = float(self.get_parameter("service_ttl_sec").value)
        self.control_ttl = float(self.get_parameter("control_ttl_sec").value)
        self.manual_cmd_ttl = float(self.get_parameter("manual_cmd_ttl_sec").value)

        self.mission_active_ttl = float(self.get_parameter("mission_active_ttl_sec").value)
        self.stack_start_grace_sec = float(self.get_parameter("stack_start_grace_sec").value)
        self.abort_display_sec = float(self.get_parameter("abort_display_sec").value)

        # ---------------- estado interno ----------------
        self.last_health_t = None
        self.last_service_t = None
        self.last_control_t = None
        self.last_manual_cmd_t = None

        self.health = {}
        self.services_status_map = {}
        self.control = {}

        # stack lifecycle
        self.stack_became_active_t = None
        self._last_stack_active = False

        # missão via /ui/progress
        self.last_progress_t = None
        self.last_progress_event = None
        self.last_mission_active_t = None
        self.last_mission_status = "PRONTO"
        self.last_abort_t = None

        # ---------------- publisher ----------------
        self.pub = self.create_publisher(String, self.pub_topic, 10)

        # ---------------- subscriptions ----------------
        self.create_subscription(String, self.health_topic, self.on_health, 10)
        self.create_subscription(String, self.service_topic, self.on_service_status, 10)
        self.create_subscription(String, self.progress_topic, self.on_progress, 10)
        self.create_subscription(String, self.control_mode_topic, self.on_control_mode, 10)
        self.create_subscription(Twist, self.manual_cmd_topic, self.on_manual_cmd, 10)

        # ---------------- timer ----------------
        self.create_timer(1.0, self.publish_state)

        self.get_logger().info("robot_state_manager iniciado")

    # ---------------------------------------------------
    # callbacks
    # ---------------------------------------------------

    def on_health(self, msg: String):
        try:
            self.health = json.loads(msg.data)
            self.last_health_t = time.time()
        except Exception as e:
            self.get_logger().warn(f"Erro parse robot_health: {e}")

    def on_service_status(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.services_status_map = data.get("services", data)
            self.last_service_t = time.time()

            stack_active_now = False
            entry = self.services_status_map.get("marsupial-stack.target", {})
            if isinstance(entry, dict):
                stack_active_now = bool(entry.get("active", False))
            else:
                stack_active_now = bool(entry)

            if stack_active_now and not self._last_stack_active:
                self.stack_became_active_t = time.time()

            if not stack_active_now:
                self.stack_became_active_t = None

            self._last_stack_active = stack_active_now

        except Exception as e:
            self.get_logger().warn(f"Erro parse service_status: {e}")

    def on_control_mode(self, msg: String):
        try:
            self.control = json.loads(msg.data)
            self.last_control_t = time.time()
        except Exception as e:
            self.get_logger().warn(f"Erro parse control_mode: {e}")

    def on_manual_cmd(self, msg: Twist):
        self.last_manual_cmd_t = time.time()

    def on_progress(self, msg: String):
        now = time.time()

        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"Erro parse /ui/progress: {e}")
            return

        self.last_progress_t = now
        self.last_progress_event = data

        event = str(data.get("event", "")).strip().lower()
        status = str(data.get("status", "")).strip().upper()

        # evidências diretas de missão ativa
        if event in {"progress", "step", "arrived"}:
            self.last_mission_status = "EXECUTANDO"
            self.last_mission_active_t = now
            return

        if event == "state":
            if status in {"RUNNING", "RETURNING", "AT_STATION"}:
                self.last_mission_status = "EXECUTANDO"
                self.last_mission_active_t = now
                return

            if status == "PAUSED":
                self.last_mission_status = "PAUSADA"
                self.last_mission_active_t = None
                return

            if status in {"ABORTED", "FAILED", "ERROR"}:
                self.last_mission_status = "ABORTADA"
                self.last_mission_active_t = None
                self.last_abort_t = now
                return

            if status == "READY":
                self.last_mission_status = "PRONTO"
                self.last_mission_active_t = None
                self.last_abort_t = None
                return

    # ---------------------------------------------------
    # util
    # ---------------------------------------------------

    def is_fresh(self, last_t: float, ttl: float) -> bool:
        return last_t is not None and (time.time() - last_t) <= ttl

    def get_service_active(self, name: str) -> bool:
        entry = self.services_status_map.get(name, {})
        if isinstance(entry, dict):
            return bool(entry.get("active", False))
        return bool(entry)

    def normalize_control_mode(self) -> str:
        raw = str(self.control.get("mode", "")).strip().upper()

        mapping = {
            "": "AUTO",
            "AUTO": "AUTO",
            "AUTONOMOUS": "AUTO",
            "AUTONOMO": "AUTO",
            "AUTÔNOMO": "AUTO",
            "MANUAL": "MANUAL",
            "TELEOP": "MANUAL",
            "JOYSTICK": "MANUAL",
        }

        return mapping.get(raw, "AUTO")

    def derive_mission_state(self) -> str:
        now = time.time()

        if self.last_mission_status == "EXECUTANDO":
            if self.last_mission_active_t is not None:
                if (now - self.last_mission_active_t) <= self.mission_active_ttl:
                    return "EXECUTANDO"
            return "PRONTO"

        if self.last_mission_status == "PAUSADA":
            return "PAUSADA"

        if self.last_mission_status == "ABORTADA":
            if self.last_abort_t is not None and (now - self.last_abort_t) <= self.abort_display_sec:
                return "ABORTADA"
            return "PRONTO"

        return "PRONTO"

    # ---------------------------------------------------
    # decisão
    # ---------------------------------------------------

    def derive_state(self):
        now = time.time()

        health_fresh = self.is_fresh(self.last_health_t, self.health_ttl)
        services_fresh = self.is_fresh(self.last_service_t, self.service_ttl)
        control_fresh = self.is_fresh(self.last_control_t, self.control_ttl)
        manual_cmd_fresh = self.is_fresh(self.last_manual_cmd_t, self.manual_cmd_ttl)

        mission_state = self.derive_mission_state()
        control_mode_topic_mode = self.normalize_control_mode()
        control_mode = "MANUAL" if manual_cmd_fresh else control_mode_topic_mode

        health_ok = bool(self.health.get("overall_ok", False)) if health_fresh else False

        drive_ok = bool(self.health.get("drive", {}).get("ok", False)) if health_fresh else False
        ekf_ok = bool(self.health.get("ekf", {}).get("ok", False)) if health_fresh else False
        nav2_ok = bool(self.health.get("nav2", {}).get("ok", False)) if health_fresh else False
        lidar_ok = bool(self.health.get("lidar", {}).get("ok", False)) if health_fresh else False

        stack_active = self.get_service_active("marsupial-stack.target") if services_fresh else False
        localization_active = self.get_service_active("marsupial-localization.service") if services_fresh else False
        nav2_active = self.get_service_active("marsupial-nav2.service") if services_fresh else False
        sensors_active = self.get_service_active("marsupial-sensors.service") if services_fresh else False

        autonomy_ready = drive_ok and ekf_ok and nav2_ok and lidar_ok

        in_startup_grace = (
            stack_active
            and self.stack_became_active_t is not None
            and (now - self.stack_became_active_t) <= self.stack_start_grace_sec
        )

        critical_drive_error = not drive_ok
        critical_localization_error = localization_active and not ekf_ok
        critical_sensor_error = sensors_active and not lidar_ok
        autonomy_error = nav2_active and not nav2_ok

        sources = {
            "health_fresh": health_fresh,
            "services_fresh": services_fresh,
            "control_fresh": control_fresh,
            "manual_cmd_fresh": manual_cmd_fresh,
            "in_startup_grace": in_startup_grace,
        }

        # 1) sem health recente => desconectado
        if not health_fresh:
            return {
                "t": now,
                "operational_state": "DESCONECTADO",
                "mission_state": "PRONTO",
                "control_mode": control_mode,
                "manual_active": manual_cmd_fresh,
                "health_ok": False,
                "stack_active": stack_active,
                "autonomy_ready": False,
                "has_error": False,
                "reason": "robot_health stale ou ausente",
                "sources": sources,
            }

        # 2) stack parado => desconectado/parado
        if services_fresh and not stack_active:
            return {
                "t": now,
                "operational_state": "DESCONECTADO",
                "mission_state": "PRONTO",
                "control_mode": control_mode,
                "manual_active": manual_cmd_fresh,
                "health_ok": False,
                "stack_active": False,
                "autonomy_ready": False,
                "has_error": False,
                "reason": "stack principal inativo",
                "sources": sources,
            }

        # 3) janela de boot/calibração
        if in_startup_grace:
            return {
                "t": now,
                "operational_state": "INICIALIZANDO",
                "mission_state": "PRONTO",
                "control_mode": control_mode,
                "manual_active": manual_cmd_fresh,
                "health_ok": health_ok,
                "stack_active": stack_active,
                "autonomy_ready": autonomy_ready,
                "has_error": False,
                "reason": f"janela de inicialização ativa ({self.stack_start_grace_sec:.0f}s)",
                "sources": sources,
            }

        # 4) modo manual
        if control_mode == "MANUAL":
            if critical_drive_error or critical_localization_error or critical_sensor_error:
                return {
                    "t": now,
                    "operational_state": "ERRO",
                    "mission_state": mission_state,
                    "control_mode": control_mode,
                    "manual_active": manual_cmd_fresh,
                    "health_ok": health_ok,
                    "stack_active": stack_active,
                    "autonomy_ready": False,
                    "has_error": True,
                    "reason": "falha crítica durante modo manual",
                    "sources": sources,
                }

            return {
                "t": now,
                "operational_state": "MANUAL",
                "mission_state": mission_state,
                "control_mode": control_mode,
                "manual_active": manual_cmd_fresh,
                "health_ok": health_ok,
                "stack_active": stack_active,
                "autonomy_ready": False,
                "has_error": False,
                "reason": "atividade recente em /ui/manual_cmd",
                "sources": sources,
            }

        # 5) erros reais em AUTO
        if critical_drive_error:
            return {
                "t": now,
                "operational_state": "ERRO",
                "mission_state": mission_state,
                "control_mode": control_mode,
                "manual_active": manual_cmd_fresh,
                "health_ok": health_ok,
                "stack_active": stack_active,
                "autonomy_ready": autonomy_ready,
                "has_error": True,
                "reason": "falha no drive",
                "sources": sources,
            }

        if critical_localization_error:
            return {
                "t": now,
                "operational_state": "ERRO",
                "mission_state": mission_state,
                "control_mode": control_mode,
                "manual_active": manual_cmd_fresh,
                "health_ok": health_ok,
                "stack_active": stack_active,
                "autonomy_ready": autonomy_ready,
                "has_error": True,
                "reason": "falha no EKF/localização",
                "sources": sources,
            }

        if critical_sensor_error:
            return {
                "t": now,
                "operational_state": "ERRO",
                "mission_state": mission_state,
                "control_mode": control_mode,
                "manual_active": manual_cmd_fresh,
                "health_ok": health_ok,
                "stack_active": stack_active,
                "autonomy_ready": autonomy_ready,
                "has_error": True,
                "reason": "falha no LiDAR",
                "sources": sources,
            }

        if autonomy_error and mission_state == "EXECUTANDO":
            return {
                "t": now,
                "operational_state": "ERRO",
                "mission_state": mission_state,
                "control_mode": control_mode,
                "manual_active": manual_cmd_fresh,
                "health_ok": health_ok,
                "stack_active": stack_active,
                "autonomy_ready": autonomy_ready,
                "has_error": True,
                "reason": "falha no Nav2 durante missão",
                "sources": sources,
            }

        # 6) missão
        if mission_state == "EXECUTANDO":
            return {
                "t": now,
                "operational_state": "EM_MISSAO",
                "mission_state": mission_state,
                "control_mode": control_mode,
                "manual_active": manual_cmd_fresh,
                "health_ok": health_ok,
                "stack_active": stack_active,
                "autonomy_ready": autonomy_ready,
                "has_error": False,
                "reason": "missão em execução",
                "sources": sources,
            }

        if mission_state == "PAUSADA":
            return {
                "t": now,
                "operational_state": "PAUSADO",
                "mission_state": mission_state,
                "control_mode": control_mode,
                "manual_active": manual_cmd_fresh,
                "health_ok": health_ok,
                "stack_active": stack_active,
                "autonomy_ready": autonomy_ready,
                "has_error": False,
                "reason": "missão pausada",
                "sources": sources,
            }

        # 7) saudável e disponível
        return {
            "t": now,
            "operational_state": "PRONTO",
            "mission_state": mission_state,
            "control_mode": control_mode,
            "manual_active": manual_cmd_fresh,
            "health_ok": health_ok,
            "stack_active": stack_active,
            "autonomy_ready": autonomy_ready,
            "has_error": False,
            "reason": "robô saudável e disponível",
            "sources": sources,
        }

    # ---------------------------------------------------
    # publicação
    # ---------------------------------------------------

    def publish_state(self):
        try:
            payload = self.derive_state()

            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False)
            self.pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Erro em publish_state: {e}")

def main():
    rclpy.init()
    node = RobotStateManager()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()