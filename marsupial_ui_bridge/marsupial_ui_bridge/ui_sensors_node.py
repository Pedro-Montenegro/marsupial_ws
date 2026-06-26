#!/usr/bin/env python3
import time
import re
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import LaserScan, Image


def level_to_int(level):
    if isinstance(level, (bytes, bytearray)):
        return int(level[0]) if len(level) else 0
    try:
        return int(level)
    except Exception:
        return 0


def extract_mean_freq(s):
    m = re.search(r"Mean Frequency:\s*([\d\.]+)\s*Hz", str(s))
    return float(m.group(1)) if m else None


def extract_drop_pct(s):
    m = re.search(r"\(([\d\.]+)%\)", str(s))
    return float(m.group(1)) if m else None


class UiSensorsBridge(Node):
    def __init__(self):
        super().__init__("ui_sensors_node")

        # ---- parâmetros de tópicos ----
        self.declare_parameter("status_topic", "ui/sensors_status")
        self.declare_parameter("diagnostics_topic", "/diagnostics")  # manter global por compatibilidade
        self.declare_parameter("scan_topic", "scan")
        self.declare_parameter("zed_image_topic", "/zed/zed_node/rgb/color/raw/image")

        # ---- parâmetros de TTL ----
        self.declare_parameter("ttl_zed", 3.0)
        self.declare_parameter("ttl_nav2", 3.0)
        self.declare_parameter("ttl_ekf", 3.0)
        self.declare_parameter("ttl_lidar", 2.0)

        status_topic = str(self.get_parameter("status_topic").value)
        diagnostics_topic = str(self.get_parameter("diagnostics_topic").value)
        scan_topic = str(self.get_parameter("scan_topic").value)
        zed_image_topic = str(self.get_parameter("zed_image_topic").value)

        self.TTL_ZED = float(self.get_parameter("ttl_zed").value)
        self.TTL_NAV2 = float(self.get_parameter("ttl_nav2").value)
        self.TTL_EKF = float(self.get_parameter("ttl_ekf").value)
        self.TTL_LIDAR = float(self.get_parameter("ttl_lidar").value)

        self.get_logger().info(f"status_topic: {status_topic}")
        self.get_logger().info(f"diagnostics_topic: {diagnostics_topic}")
        self.get_logger().info(f"scan_topic: {scan_topic}")
        self.get_logger().info(f"zed_image_topic: {zed_image_topic}")

        self.pub = self.create_publisher(String, status_topic, 10)

        self.create_subscription(DiagnosticArray, diagnostics_topic, self.on_diag, 10)
        self.create_subscription(LaserScan, scan_topic, self.on_scan, 10)
        self.create_subscription(Image, zed_image_topic, self.on_zed_image, 10)

        self.state = {
            "zed": {
                "last_diag_ok_t": None,
                "last_frame_t": None,
                "hz": None,
                "drop_pct": None,
                "temp_left": None,
                "temp_right": None,
                "_last_img_t": None,
                "_ema_img_hz": None,
            },
            "nav2": {"last_ok_t": None},
            "ekf": {"last_ok_t": None, "hz": None},
            "lidar": {
                "last_ok_t": None,
                "hz": None,
                "src": None,
                "_last_scan_t": None,
                "_ema_hz": None,
            },
        }

        self.create_timer(1.0, self.publish_status)

    def on_scan(self, msg: LaserScan):
        now = time.time()
        self.state["lidar"]["last_ok_t"] = now
        self.state["lidar"]["src"] = "scan"

        last_t = self.state["lidar"]["_last_scan_t"]
        if last_t is not None:
            dt = now - last_t
            if dt > 1e-3:
                hz = 1.0 / dt
                ema = self.state["lidar"]["_ema_hz"]
                alpha = 0.2
                self.state["lidar"]["_ema_hz"] = hz if ema is None else (alpha * hz + (1 - alpha) * ema)
                self.state["lidar"]["hz"] = self.state["lidar"]["_ema_hz"]

        self.state["lidar"]["_last_scan_t"] = now

    def on_zed_image(self, msg: Image):
        now = time.time()
        self.state["zed"]["last_frame_t"] = now

        last_t = self.state["zed"]["_last_img_t"]
        if last_t is not None:
            dt = now - last_t
            if dt > 1e-3:
                hz = 1.0 / dt
                ema = self.state["zed"]["_ema_img_hz"]
                alpha = 0.2
                self.state["zed"]["_ema_img_hz"] = hz if ema is None else (alpha * hz + (1 - alpha) * ema)

        self.state["zed"]["_last_img_t"] = now

    def on_diag(self, msg: DiagnosticArray):
        now = time.time()

        for st in msg.status:
            name = (st.name or "").lower()
            message = (st.message or "").lower()
            lvl = level_to_int(st.level)
            ok = (lvl == 0)

            if "zed_node" in name or ("zed" in name and "zed 2i" in name):
                grabbing = "camera grabbing" in message
                if ok and grabbing:
                    self.state["zed"]["last_diag_ok_t"] = now

                for kv in st.values:
                    k = (kv.key or "").lower()
                    v = kv.value

                    if "data capture" in k:
                        hz = extract_mean_freq(v)
                        if hz is not None:
                            self.state["zed"]["hz"] = hz

                    if "frame drop rate" in k:
                        dp = extract_drop_pct(v)
                        if dp is not None:
                            self.state["zed"]["drop_pct"] = dp

                    if "left cmos temp" in k:
                        try:
                            self.state["zed"]["temp_left"] = float(str(v).split()[0])
                        except Exception:
                            pass

                    if "right cmos temp" in k:
                        try:
                            self.state["zed"]["temp_right"] = float(str(v).split()[0])
                        except Exception:
                            pass

            if "ekf_odom" in name and "functioning properly" in message:
                if ok:
                    self.state["ekf"]["last_ok_t"] = now

            if "ekf_odom" in name:
                for kv in st.values:
                    if (kv.key or "").lower() == "actual frequency (hz)":
                        try:
                            self.state["ekf"]["hz"] = float(kv.value)
                        except Exception:
                            pass

            if "nav2 health" in name:
                active = "active" in message
                if ok and active:
                    self.state["nav2"]["last_ok_t"] = now

            is_lidar_diag = any(k in name for k in [
                "rplidar", "sllidar", "ydlidar", "urg", "hokuyo", "laser", "lidar"
            ])
            if is_lidar_diag:
                running_hint = any(k in message for k in ["ok", "running", "publishing", "spinning"])
                if ok or running_hint:
                    self.state["lidar"]["last_ok_t"] = now
                    self.state["lidar"]["src"] = "diagnostics"

                for kv in st.values:
                    k = (kv.key or "").lower()
                    v = str(kv.value)
                    if "frequency" in k or "rate" in k:
                        m = re.search(r"([\d\.]+)", v)
                        if m:
                            try:
                                self.state["lidar"]["hz"] = float(m.group(1))
                            except Exception:
                                pass

    def publish_status(self):
        now = time.time()

        def ttl_ok(last_t, ttl):
            return (last_t is not None) and ((now - last_t) <= ttl)

        zed_diag_ok = ttl_ok(self.state["zed"]["last_diag_ok_t"], self.TTL_ZED)
        zed_stream_ok = ttl_ok(self.state["zed"]["last_frame_t"], self.TTL_ZED)

        zed_ok = zed_diag_ok and zed_stream_ok
        nav2_ok = ttl_ok(self.state["nav2"]["last_ok_t"], self.TTL_NAV2)
        ekf_ok = ttl_ok(self.state["ekf"]["last_ok_t"], self.TTL_EKF)
        lidar_ok = ttl_ok(self.state["lidar"]["last_ok_t"], self.TTL_LIDAR)

        payload = {
            "zed": {
                "ok": zed_ok,
                "diag_ok": zed_diag_ok,
                "stream_ok": zed_stream_ok,
                "hz": self.state["zed"]["_ema_img_hz"] or self.state["zed"]["hz"],
                "drop_pct": self.state["zed"]["drop_pct"],
                "temp_left": self.state["zed"]["temp_left"],
                "temp_right": self.state["zed"]["temp_right"],
                "age_ms": None if self.state["zed"]["last_frame_t"] is None else int((now - self.state["zed"]["last_frame_t"]) * 1000),
            },
            "lidar": {
                "ok": lidar_ok,
                "hz": self.state["lidar"]["hz"],
                "src": self.state["lidar"]["src"],
                "age_ms": None if self.state["lidar"]["last_ok_t"] is None else int((now - self.state["lidar"]["last_ok_t"]) * 1000),
            },
            "nav2": {"ok": nav2_ok},
            "ekf": {"ok": ekf_ok, "hz": self.state["ekf"]["hz"]},
        }

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = UiSensorsBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()