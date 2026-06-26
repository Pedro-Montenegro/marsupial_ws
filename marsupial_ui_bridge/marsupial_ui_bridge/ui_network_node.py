#!/usr/bin/env python3
import json
import re
import shlex
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def run_cmd(cmd: str, timeout: float = 1.5) -> str:
    try:
        out = subprocess.check_output(
            shlex.split(cmd),
            stderr=subprocess.STDOUT,
            timeout=timeout,
            text=True,
        )
        return out.strip()
    except Exception:
        return ""


def parse_iw_link(iw_out: str):
    ssid = None
    rssi = None
    tx_mbps = None

    m = re.search(r"SSID:\s*(.+)", iw_out)
    if m:
        ssid = m.group(1).strip()

    m = re.search(r"signal:\s*(-?\d+)\s*dBm", iw_out)
    if m:
        rssi = int(m.group(1))

    m = re.search(r"tx bitrate:\s*([\d\.]+)\s*MBit/s", iw_out)
    if m:
        try:
            tx_mbps = float(m.group(1))
        except Exception:
            tx_mbps = None

    return ssid, rssi, tx_mbps


def parse_iw_info(iw_info: str):
    mode = None
    ssid = None
    channel = None
    txpower_dbm = None

    m = re.search(r"type\s+(\S+)", iw_info)
    if m:
        mode = m.group(1).strip()

    m = re.search(r"ssid\s+(.+)", iw_info)
    if m:
        ssid = m.group(1).strip()

    m = re.search(r"channel\s+(\d+)", iw_info)
    if m:
        try:
            channel = int(m.group(1))
        except Exception:
            channel = None

    m = re.search(r"txpower\s+([\d\.]+)\s+dBm", iw_info)
    if m:
        try:
            txpower_dbm = float(m.group(1))
        except Exception:
            txpower_dbm = None

    return mode, ssid, channel, txpower_dbm


def get_ipv4_addr(iface: str) -> str | None:
    out = run_cmd(f"ip -4 addr show dev {iface}")
    m = re.search(r"inet\s+([0-9\.]+)/", out)
    if m:
        return m.group(1)
    return None


def get_default_gateway() -> str | None:
    out = run_cmd("ip route show default")
    m = re.search(r"default via ([0-9\.]+)", out)
    if m:
        return m.group(1)
    return None


def ping_latency_ms(host: str, timeout: float = 1.5) -> float | None:
    if not host:
        return None

    out = run_cmd(f"ping -c 1 -W 1 {host}", timeout=timeout)

    m = re.search(r"rtt .* = [\d\.]+/([\d\.]+)/", out)
    if m:
        try:
            return float(m.group(1))
        except Exception:
            return None

    return None


def get_wifi_status(iface: str):
    iw_info = run_cmd(f"iw dev {iface} info")
    iw_link = run_cmd(f"iw dev {iface} link")

    mode_raw, ap_ssid, channel, txpower_dbm = parse_iw_info(iw_info)
    ip_addr = get_ipv4_addr(iface)

    if mode_raw in ("AP", "__ap"):
        return {
            "mode": "ap",
            "connected": True,
            "ssid": ap_ssid,
            "rssi_dbm": None,
            "tx_mbps": None,
            "channel": channel,
            "txpower_dbm": txpower_dbm,
            "ip_addr": ip_addr,
            "local_network": ip_addr is not None,
        }

    ssid, rssi_dbm, tx_mbps = parse_iw_link(iw_link)
    client_connected = "Connected to" in iw_link

    return {
        "mode": "client",
        "connected": client_connected,
        "ssid": ssid,
        "rssi_dbm": rssi_dbm,
        "tx_mbps": tx_mbps,
        "channel": channel,
        "txpower_dbm": txpower_dbm,
        "ip_addr": ip_addr,
        "local_network": client_connected and ip_addr is not None,
    }


class NetworkStatusNode(Node):
    def __init__(self):
        super().__init__("marsupial_network_status")

        self.declare_parameter("iface", "wlP1p1s0")
        self.declare_parameter("topic", "ui/network_status")
        self.declare_parameter("hz", 1.0)

        # vazio => tenta pingar o gateway padrão
        self.declare_parameter("ping_host", "")

        self._iface = str(self.get_parameter("iface").value)
        self._topic = str(self.get_parameter("topic").value)
        hz = float(self.get_parameter("hz").value)

        self._period = 1.0 / max(hz, 0.2)

        self.pub = self.create_publisher(String, self._topic, 10)
        self.timer = self.create_timer(self._period, self.tick)

        self.get_logger().info(
            f"Publicando {self._topic} @ {hz:.2f} Hz | iface={self._iface}"
        )

    def tick(self):
        iface = self._iface

        wifi = get_wifi_status(iface)

        ping_host_param = str(self.get_parameter("ping_host").value).strip()
        ping_host = ping_host_param or get_default_gateway() or ""

        lat_ms = ping_latency_ms(ping_host) if ping_host else None

        payload = {
            "ts": time.time(),
            "iface": iface,
            **wifi,
            "ping_host": ping_host if ping_host else None,
            "lat_ms": lat_ms,
            "internet": lat_ms is not None,
        }

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = NetworkStatusNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()