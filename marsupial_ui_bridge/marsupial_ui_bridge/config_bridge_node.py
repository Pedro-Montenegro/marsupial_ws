#!/usr/bin/env python3
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import String
from std_srvs.srv import Trigger


import os

DEFAULT_CONFIG = {
    "robotIp": os.getenv("ROBOT_IP", "192.168.10.1"),
    "wsPort": int(os.getenv("ROSBRIDGE_PORT", "9090")),
    "mqttHost": "mqtt.robotics.local",
    "maxSpeed": 1.5,
    "safetyDist": 0.5,
    "silentMode": False,
    "avoidDynamic": True,
}

class ConfigBridgeNode(Node):
    def __init__(self):
        super().__init__("config_bridge_node")

        # Tópicos/serviços relativos -> respeitam namespace do robô
        self.declare_parameter("config_topic", "ui/config")
        self.declare_parameter("config_set_topic", "ui/config_set")
        self.declare_parameter("recalibrate_service", "ui/recalibrate_horizon")
        self.declare_parameter("zero_odom_service", "ui/zero_odometry")
        self.declare_parameter("restart_camera_service", "ui/restart_camera")
        self.declare_parameter(
            "config_path",
            str(Path.home() / ".marsupial" / "config.json")
        )

        self.config_topic = str(self.get_parameter("config_topic").value)
        self.config_set_topic = str(self.get_parameter("config_set_topic").value)
        self.recalibrate_service = str(self.get_parameter("recalibrate_service").value)
        self.zero_odom_service = str(self.get_parameter("zero_odom_service").value)
        self.restart_camera_service = str(self.get_parameter("restart_camera_service").value)
        self.config_path = Path(str(self.get_parameter("config_path").value)).expanduser()

        qos_latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.config = self._load_config()

        self.pub_cfg = self.create_publisher(String, self.config_topic, qos_latched)
        self.sub_set = self.create_subscription(
            String,
            self.config_set_topic,
            self._on_set,
            10
        )

        self.srv_recal = self.create_service(
            Trigger,
            self.recalibrate_service,
            self._srv_recalibrate
        )
        self.srv_zero = self.create_service(
            Trigger,
            self.zero_odom_service,
            self._srv_zero_odom
        )
        self.srv_cam = self.create_service(
            Trigger,
            self.restart_camera_service,
            self._srv_restart_camera
        )

        self._publish_config()

        self.get_logger().info(
            "ConfigBridgeNode pronto. "
            f"config={self.config_topic}, "
            f"config_set={self.config_set_topic}, "
            f"config_path={self.config_path}"
        )

    def _load_config(self):
        try:
            self.config_path.parent.mkdir(parents=True, exist_ok=True)
            if self.config_path.exists():
                data = json.loads(self.config_path.read_text())
                merged = {**DEFAULT_CONFIG, **data}
                return merged
        except Exception as e:
            self.get_logger().warn(f"Falha ao ler config: {e}")

        return dict(DEFAULT_CONFIG)

    def _save_config(self):
        try:
            self.config_path.parent.mkdir(parents=True, exist_ok=True)
            self.config_path.write_text(
                json.dumps(self.config, indent=2, ensure_ascii=False)
            )
            return True
        except Exception as e:
            self.get_logger().error(f"Falha ao salvar config: {e}")
            return False

    def _publish_config(self):
        msg = String()
        msg.data = json.dumps(self.config, ensure_ascii=False)
        self.pub_cfg.publish(msg)

    def _on_set(self, msg: String):
        try:
            incoming = json.loads(msg.data)
            if not isinstance(incoming, dict):
                raise ValueError("config_set deve ser JSON object")

            for k in DEFAULT_CONFIG.keys():
                if k in incoming:
                    self.config[k] = incoming[k]

            ok = self._save_config()
            self._publish_config()

            self.get_logger().info(f"Config atualizada (ok={ok}): {incoming}")
        except Exception as e:
            self.get_logger().error(f"config_set inválido: {e}")

    def _srv_recalibrate(self, req, res):
        # TODO: chamar serviço real da ZED se existir
        res.success = True
        res.message = "Recalibrar horizonte: OK (stub)"
        return res

    def _srv_zero_odom(self, req, res):
        # TODO: implementar reset real de odometria/localização
        res.success = True
        res.message = "Zerar odometria: OK (stub)"
        return res

    def _srv_restart_camera(self, req, res):
        # TODO: integrar com restart seguro da câmera
        res.success = True
        res.message = "Restart câmera: OK (stub)"
        return res


def main():
    rclpy.init()
    node = ConfigBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()