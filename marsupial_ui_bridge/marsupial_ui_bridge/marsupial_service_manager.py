#!/usr/bin/env python3
import json
import shlex
import subprocess
import time
from typing import Dict, Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


ALLOWED_ACTIONS = {"start", "stop", "restart", "status"}

ALLOWED_SYSTEM_ACTIONS = {
    "shutdown_robot",
    "reboot_robot",
    "stop_robot_stack",
}

# Lista branca (NUNCA execute unit fora disso)
ALLOWED_UNITS = {
    "marsupial-sensors.service",
    "marsupial-localization.service",
    "marsupial-nav2.service",
    "marsupial-service-manager.service",
    "marsupial-stack.target",
    "marsupial-video.target",
    "marsupial-core.service",
    "marsupial-relocalization.service",
}


def run_cmd(cmd: str, timeout: float = 10.0) -> Dict[str, Any]:
    """Executa comando via subprocess (sem shell), retorna stdout/stderr/returncode."""
    args = shlex.split(cmd)
    try:
        p = subprocess.run(
            args,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        return {
            "returncode": p.returncode,
            "stdout": (p.stdout or "").strip(),
            "stderr": (p.stderr or "").strip(),
        }
    except subprocess.TimeoutExpired as e:
        return {
            "returncode": 124,
            "stdout": (e.stdout or "").strip() if getattr(e, "stdout", None) else "",
            "stderr": "timeout",
        }
    except Exception as e:
        return {
            "returncode": 1,
            "stdout": "",
            "stderr": f"exception: {e}",
        }


def normalize_unit(name: str) -> str:
    """
    Aceita:
      - "marsupial-sensors" -> "marsupial-sensors.service"
      - "marsupial-stack"   -> "marsupial-stack.target"
      - já com sufixo -> mantém
    """
    n = (name or "").strip()
    if not n:
        return ""

    if n.endswith(".service") or n.endswith(".target"):
        return n

    if n in ("marsupial-stack", "marsupial-video"):
        return n + ".target"

    return n + ".service"


class ServiceManagerNode(Node):
    def __init__(self):
        super().__init__("marsupial_service_manager")

        # tópicos relativos -> respeitam namespace do robô
        self.declare_parameter("service_cmd_topic", "ui/service_cmd")
        self.declare_parameter("system_cmd_topic", "ui/system_cmd")
        self.declare_parameter("service_status_topic", "ui/service_status")
        self.declare_parameter("poll_period_s", 1.0)

        self.service_cmd_topic = str(self.get_parameter("service_cmd_topic").value)
        self.system_cmd_topic = str(self.get_parameter("system_cmd_topic").value)
        self.service_status_topic = str(self.get_parameter("service_status_topic").value)
        self.poll_s = float(self.get_parameter("poll_period_s").value)

        self.cmd_sub = self.create_subscription(
            String,
            self.service_cmd_topic,
            self.on_cmd,
            10
        )

        self.system_cmd_sub = self.create_subscription(
            String,
            self.system_cmd_topic,
            self.on_system_cmd,
            10
        )

        self.status_pub = self.create_publisher(
            String,
            self.service_status_topic,
            10
        )

        self.create_timer(self.poll_s, self.publish_all_status)

        self.get_logger().info(
            f"marsupial_service_manager up. "
            f"cmd={self.service_cmd_topic} "
            f"system={self.system_cmd_topic} "
            f"status={self.service_status_topic}"
        )

    def reply(self, payload: Dict[str, Any]):
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)

    def systemctl(self, action: str, unit: str) -> Dict[str, Any]:
        """
        Usa sudo porque o node roda como usuário (marsupial).
        IMPORTANTE: sudoers deve permitir APENAS os commands/unidades necessárias.
        """
        if action == "status":
            cmd = f"sudo /bin/systemctl is-active {unit}"
            res = run_cmd(cmd, timeout=5.0)
            active = (res["returncode"] == 0 and res["stdout"].strip() == "active")
            return {
                "ok": True,  # consultar status em si rodou
                "active": active,
                "state": res["stdout"].strip() or "unknown",
                "raw": res,
            }

        # target: restart = stop + start
        if action == "restart" and unit.endswith(".target"):
            r_stop = run_cmd(f"sudo /bin/systemctl stop {unit}", timeout=15.0)
            r_start = run_cmd(f"sudo /bin/systemctl start {unit}", timeout=15.0)
            ok = (r_start["returncode"] == 0)
            return {
                "ok": ok,
                "raw": {"stop": r_stop, "start": r_start},
            }

        cmd = f"sudo /bin/systemctl {action} {unit}"
        res = run_cmd(cmd, timeout=15.0)
        ok = (res["returncode"] == 0)
        return {"ok": ok, "raw": res}

    def on_cmd(self, msg: String):
        now = time.time()

        try:
            data = json.loads(msg.data)
        except Exception:
            self.reply({
                "t": now,
                "type": "cmd",
                "ok": False,
                "error": "invalid_json",
                "example": {"action": "restart", "service": "marsupial-sensors"},
            })
            return

        action = str(data.get("action", "")).strip()
        service_in = str(data.get("service", "")).strip()
        unit = normalize_unit(service_in)

        if action not in ALLOWED_ACTIONS:
            self.reply({
                "t": now,
                "type": "cmd",
                "ok": False,
                "error": "action_not_allowed",
                "allowed_actions": sorted(ALLOWED_ACTIONS),
            })
            return

        if unit not in ALLOWED_UNITS:
            self.reply({
                "t": now,
                "type": "cmd",
                "ok": False,
                "error": "unit_not_allowed",
                "service_received": service_in,
                "unit_normalized": unit,
                "allowed_units": sorted(ALLOWED_UNITS),
            })
            return

        self.get_logger().info(f"cmd: {action} {unit}")

        out = self.systemctl(action, unit)
        self.reply({
            "t": now,
            "type": "cmd",
            "service": service_in,
            "unit": unit,
            "action": action,
            **out,
        })

    def on_system_cmd(self, msg: String):
        now = time.time()

        try:
            data = json.loads(msg.data)
        except Exception:
            self.reply({
                "t": now,
                "type": "system_cmd",
                "ok": False,
                "error": "invalid_json",
                "example": {"action": "shutdown_robot"},
            })
            return

        action = str(data.get("action", "")).strip()

        if action not in ALLOWED_SYSTEM_ACTIONS:
            self.reply({
                "t": now,
                "type": "system_cmd",
                "ok": False,
                "error": "system_action_not_allowed",
                "allowed_actions": sorted(ALLOWED_SYSTEM_ACTIONS),
            })
            return

        self.get_logger().info(f"system cmd: {action}")

        out = self.run_system_action(action)

        self.reply({
            "t": now,
            "type": "system_cmd",
            "action": action,
            **out,
        })

    def run_system_action(self, action: str) -> Dict[str, Any]:
        """
        Executa ações de nível de sistema.
        """
        if action == "shutdown_robot":
            raw = run_cmd("sudo /usr/local/bin/robot_shutdown.sh", timeout=20.0)
            return {"ok": raw["returncode"] == 0, "raw": raw}

        if action == "reboot_robot":
            raw = run_cmd("sudo /sbin/shutdown -r now", timeout=10.0)
            return {"ok": raw["returncode"] == 0, "raw": raw}

        if action == "stop_robot_stack":
            raw = run_cmd("sudo /bin/systemctl stop marsupial-stack.target", timeout=15.0)
            return {"ok": raw["returncode"] == 0, "raw": raw}

        return {
            "ok": False,
            "raw": {"stderr": f"unknown system action: {action}", "returncode": 1},
        }

    def publish_all_status(self):
        st = {}
        for unit in sorted(ALLOWED_UNITS):
            res = self.systemctl("status", unit)
            st[unit] = {
                "active": bool(res.get("active", False)),
                "state": res.get("state", "unknown"),
            }

        self.reply({
            "t": time.time(),
            "type": "poll",
            "services": st,
        })


def main():
    rclpy.init()
    node = ServiceManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()