#!/usr/bin/env python3
"""
odom_node_4wd_can.py

Controle skid-steer 4WD via ODrive CANSimple (um node_id por roda) + odometria.

Baseado em odom_node.py (transporte CAN, 2 rodas) e odom_node_4wd.py
(topologia/odometria skid-steer, 4 rodas via USB). Este node junta os dois:
mesma topologia e interface ROS do 4WD, mas falando CAN em vez de USB.

Topologia (node_id por roda, ver robot_4wd_can.yaml):
  FL (dianteira esq) node_id=2   RL (traseira esq) node_id=3
  FR (dianteira dir) node_id=1   RR (traseira dir) node_id=0

Odometria:
  avg_left  = (pos_FL + pos_RL) / 2
  avg_right = (pos_FR + pos_RR) / 2
  → DiffOdom.update_from_turns(avg_left, avg_right)

Interface ROS 2:
  Sub: /cmd_vel          (geometry_msgs/Twist)
  Pub: /wheel/odom       (nav_msgs/Odometry)
  Pub: /ui/drive_status  (std_msgs/String — JSON com estado dos 4 eixos)
  Pub: /drive/telemetry  (std_msgs/String — JSON com posições/comandos)
"""

import json
import math
import time
import threading
from typing import Callable, Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.executors import ExternalShutdownException

import can
import cantools

from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from .diff_odom import DiffOdom

AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP_CONTROL = 8
CONTROL_MODE_VELOCITY_CONTROL = 2
INPUT_MODE_VEL_RAMP = 1

WHEELS = ("FL", "RL", "FR", "RR")


def quat_from_yaw(yaw: float) -> Quaternion:
    half = yaw * 0.5
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


def to_int(v, default=0) -> int:
    """Converte valores do cantools (inclui NamedSignalValue) para int, sem quebrar."""
    if v is None:
        return default
    if hasattr(v, "value"):
        try:
            return int(v.value)
        except Exception:
            pass
    if isinstance(v, (bytes, bytearray)):
        return int(v[0]) if len(v) else default
    try:
        return int(v)
    except Exception:
        return default


class ODriveCAN4WD:
    """
    Wrapper para N eixos ODrive via CANSimple (DBC), um node_id por roda.

    O .dbc já traz mensagens genéricas AxisN_* para N=0..7, então qualquer
    node_id usado nas ODrives (aqui: 0,1,2,3) já tem mensagens prontas —
    não precisa estender o arquivo.

    Implementa:
      - envio protegido (não crasha se TX buffer encher, ex: Errno 105)
      - RTR async (send_enc_rtr) + cache via thread RX, round-robin entre eixos
    """

    def __init__(
        self,
        channel: str,
        dbc_path: str,
        node_ids: Dict[str, int],
        log: Optional[Callable[[str], None]] = None,
    ):
        self.bus = can.interface.Bus(channel=channel, interface="socketcan")
        self.db = cantools.database.load_file(dbc_path)
        self.node_ids = dict(node_ids)  # label (FL/RL/FR/RR) -> node_id
        self._log = log or print

        active_ids = sorted(set(self.node_ids.values()))
        self.msg = {}
        for nid in active_ids:
            for key, suffix in (
                ("set_axis_state", "Set_Axis_State"),
                ("set_ctrl_mode", "Set_Controller_Mode"),
                ("set_input_vel", "Set_Input_Vel"),
                ("get_enc_est", "Get_Encoder_Estimates"),
                ("heartbeat", "Heartbeat"),
                ("clear_errors", "Clear_Errors"),
            ):
                self.msg[(nid, key)] = self.db.get_message_by_name(f"Axis{nid}_{suffix}")

        filters = []
        for nid in active_ids:
            filters.append({"can_id": self.msg[(nid, "get_enc_est")].frame_id, "can_mask": 0x7FF, "extended": False})
            filters.append({"can_id": self.msg[(nid, "heartbeat")].frame_id, "can_mask": 0x7FF, "extended": False})
        try:
            self.bus.set_filters(filters)
        except Exception:
            pass

        self._enc_cache = {nid: (None, None, 0.0) for nid in active_ids}
        self._hb_cache = {nid: (None, 0.0) for nid in active_ids}
        self._active_ids = active_ids

        self._stop_rx = False
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    def _rx_loop(self):
        while not self._stop_rx:
            try:
                resp = self.bus.recv(timeout=0.01)
            except Exception:
                continue
            if resp is None:
                continue
            ts = time.time()
            for nid in self._active_ids:
                m = self.msg[(nid, "get_enc_est")]
                if resp.arbitration_id == m.frame_id and len(resp.data) == m.length:
                    try:
                        vals = m.decode(resp.data)
                    except Exception:
                        continue
                    self._enc_cache[nid] = (vals.get("Pos_Estimate"), vals.get("Vel_Estimate"), ts)
                    continue
                hb = self.msg[(nid, "heartbeat")]
                if resp.arbitration_id == hb.frame_id and len(resp.data) == hb.length:
                    try:
                        vals = hb.decode(resp.data)
                    except Exception:
                        continue
                    self._hb_cache[nid] = (vals, ts)

    def _safe_log(self, s: str):
        try:
            self._log(s)
        except Exception:
            pass

    def _send(self, m, data_dict) -> bool:
        data = m.encode(data_dict)
        msg = can.Message(arbitration_id=m.frame_id, data=data, is_extended_id=False)
        try:
            self.bus.send(msg)
            return True
        except can.CanOperationError as e:
            self._safe_log(f"[CAN] send falhou: {e}")
            return False
        except OSError as e:
            self._safe_log(f"[CAN] OSError no send: {e}")
            return False

    def clear_errors(self, node_id: int) -> bool:
        m = self.msg[(node_id, "clear_errors")]
        msg = can.Message(arbitration_id=m.frame_id, data=b"", is_extended_id=False)
        try:
            self.bus.send(msg)
            return True
        except (can.CanOperationError, OSError) as e:
            self._safe_log(f"[CAN] clear_errors falhou: {e}")
            return False

    def set_axis_state(self, node_id: int, state: int) -> bool:
        m = self.msg[(node_id, "set_axis_state")]
        return self._send(m, {"Axis_Requested_State": int(state)})

    def set_controller_mode_velocity(self, node_id: int) -> bool:
        m = self.msg[(node_id, "set_ctrl_mode")]
        return self._send(m, {"Input_Mode": INPUT_MODE_VEL_RAMP, "Control_Mode": CONTROL_MODE_VELOCITY_CONTROL})

    def set_input_vel(self, node_id: int, vel_rev_s: float, torque_ff: float = 0.0) -> bool:
        m = self.msg[(node_id, "set_input_vel")]
        return self._send(m, {"Input_Vel": float(vel_rev_s), "Input_Torque_FF": float(torque_ff)})

    def send_enc_rtr(self, node_id: int) -> bool:
        req = self.msg[(node_id, "get_enc_est")]
        msg = can.Message(arbitration_id=req.frame_id, is_extended_id=False, is_remote_frame=True, dlc=0)
        try:
            self.bus.send(msg)
            return True
        except (can.CanOperationError, OSError) as e:
            self._safe_log(f"[CAN] RTR falhou: {e}")
            return False

    def read_cached(self, node_id: int, max_age: float = 0.2) -> Tuple[Optional[float], Optional[float]]:
        pos, vel, ts = self._enc_cache.get(node_id, (None, None, 0.0))
        if pos is None or (time.time() - ts) > max_age:
            return None, None
        return pos, vel

    def read_heartbeat_cached(self, node_id: int, max_age: float = 0.8):
        d, ts = self._hb_cache.get(node_id, (None, 0.0))
        if d is None or (time.time() - ts) > max_age:
            return None
        return d

    def safe_idle(self, tries: int = 3):
        for _ in range(tries):
            try:
                for nid in self._active_ids:
                    self.set_input_vel(nid, 0.0, 0.0)
                time.sleep(0.02)
                for nid in self._active_ids:
                    self.set_axis_state(nid, AXIS_STATE_IDLE)
                return
            except Exception:
                time.sleep(0.05)

    def stop(self):
        self._stop_rx = True
        try:
            self._rx_thread.join(timeout=0.2)
        except Exception:
            pass
        try:
            self.bus.shutdown()
        except Exception:
            pass


class WheelOdom4WDCANNode(Node):

    def __init__(self):
        super().__init__("odom_node_4wd_can")

        # ── Parâmetros ────────────────────────────────────────────────
        self.declare_parameter("wheel_radius", 0.082)
        self.declare_parameter("wheel_base", 0.50)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_topic", "wheel/odom")
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("drive_status_topic", "ui/drive_status")
        self.declare_parameter("telemetry_topic", "drive/telemetry")
        self.declare_parameter("hz", 30.0)
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("cmd_timeout", 0.5)
        self.declare_parameter("sign_left", -1.0)
        self.declare_parameter("sign_right", 1.0)
        self.declare_parameter("right_disabled", False)
        self.declare_parameter("right_front_disabled", False)

        # CAN
        self.declare_parameter("can_channel", "can0")
        self.declare_parameter("can_dbc", "odrive-cansimple.dbc")
        self.declare_parameter("fl_node_id", 2)
        self.declare_parameter("rl_node_id", 3)
        self.declare_parameter("fr_node_id", 1)
        self.declare_parameter("rr_node_id", 0)
        self.declare_parameter("rtr_period", 0.025)  # ~10 Hz por eixo com 4 eixos ativos

        wheel_radius = float(self.get_parameter("wheel_radius").value)
        wheel_base = float(self.get_parameter("wheel_base").value)
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        odom_topic = self.get_parameter("odom_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        drive_status_topic = self.get_parameter("drive_status_topic").value
        telemetry_topic = self.get_parameter("telemetry_topic").value
        hz = float(self.get_parameter("hz").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self.sign_l = float(self.get_parameter("sign_left").value)
        self.sign_r = float(self.get_parameter("sign_right").value)
        self.right_disabled = bool(self.get_parameter("right_disabled").value)
        self.right_front_disabled = bool(self.get_parameter("right_front_disabled").value)

        can_channel = self.get_parameter("can_channel").value
        can_dbc = self.get_parameter("can_dbc").value
        rtr_period = float(self.get_parameter("rtr_period").value)

        self.node_ids = {
            "FL": int(self.get_parameter("fl_node_id").value),
            "RL": int(self.get_parameter("rl_node_id").value),
            "FR": int(self.get_parameter("fr_node_id").value),
            "RR": int(self.get_parameter("rr_node_id").value),
        }
        if self.right_disabled:
            self.active_wheels = ["FL", "RL"]
        elif self.right_front_disabled:
            self.active_wheels = ["FL", "RL", "RR"]
        else:
            self.active_wheels = ["FL", "RL", "FR", "RR"]
        active_node_ids = {w: self.node_ids[w] for w in self.active_wheels}

        if hz <= 0.0:
            raise ValueError("hz deve ser maior que zero.")
        if self.cmd_timeout <= 0.0:
            raise ValueError("cmd_timeout deve ser maior que zero.")

        # ── Publishers ────────────────────────────────────────────────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(Odometry, odom_topic, qos)
        self.pub_drive = self.create_publisher(String, drive_status_topic, 10)
        self.pub_telemetry = self.create_publisher(String, telemetry_topic, qos)
        self.br = TransformBroadcaster(self) if self.publish_tf else None

        # ── ODrive/CAN ────────────────────────────────────────────────
        self.get_logger().info(f"Conectando ODrive via CAN: {can_channel} (eixos ativos: {self.active_wheels})")
        try:
            self.can4wd = ODriveCAN4WD(
                channel=can_channel,
                dbc_path=can_dbc,
                node_ids=active_node_ids,
                log=lambda s: self.get_logger().warn(s),
            )
        except Exception as e:
            self.get_logger().error(f"Falha ao iniciar CAN/DBC: {e}")
            raise

        # Sequência com pausas entre passos e entre eixos — testada manualmente e
        # confirmada como necessária: mandar Set_Controller_Mode e
        # Set_Axis_State(CLOSED_LOOP) em sequência apertada (sem pausa), ou pra
        # vários eixos de uma vez sem intervalo, pode deixar algum eixo com o
        # controller mode ainda não aplicado quando o closed-loop é solicitado —
        # o eixo entra em CLOSED_LOOP_CONTROL normalmente (sem erro) mas o
        # vel_setpoint nunca acompanha o input_vel (torque zero, roda não gira).
        for w in self.active_wheels:
            nid = self.node_ids[w]
            self.can4wd.clear_errors(nid)
            time.sleep(0.05)
        for w in self.active_wheels:
            nid = self.node_ids[w]
            self.can4wd.set_controller_mode_velocity(nid)
            time.sleep(0.2)
            self.can4wd.set_axis_state(nid, AXIS_STATE_CLOSED_LOOP_CONTROL)
            time.sleep(0.3)

        # ── Odometria ─────────────────────────────────────────────────
        circ = 2.0 * math.pi * wheel_radius
        self.odom_solver = DiffOdom(L=wheel_base, CIRC=circ, sign_l=self.sign_l, sign_r=self.sign_r)
        self.odom_solver.reset()

        # ── Estado cmd_vel ────────────────────────────────────────────
        self.last_cmd_monotonic = time.monotonic()
        self._stopped_due_timeout = False
        self._last_sent = (None, None)  # (wr, wl)
        self._last_sent_t = 0.0
        self._min_send_period = 0.02  # máx 50 Hz
        self._eps = 1e-4

        # ── Subscribers / Timers ──────────────────────────────────────
        self.sub_cmd = self.create_subscription(Twist, cmd_vel_topic, self.cmd_cb, qos)

        period = 1.0 / max(1.0, hz)
        self.timer = self.create_timer(period, self.loop_cb)
        self.timer_drv = self.create_timer(1.0, self.publish_drive_status)

        self._rtr_toggle = 0
        self.timer_rtr = self.create_timer(rtr_period, self._enc_rtr_cb)

        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = self.odom_frame
        self.odom_msg.child_frame_id = self.base_frame

        self.get_logger().info("WheelOdom4WDCANNode pronto.")

    # ─────────────────────────────────────────────────────────────────
    # RTR round-robin entre eixos ativos
    # ─────────────────────────────────────────────────────────────────

    def _enc_rtr_cb(self):
        w = self.active_wheels[self._rtr_toggle % len(self.active_wheels)]
        self._rtr_toggle += 1
        ok = self.can4wd.send_enc_rtr(self.node_ids[w])
        if not ok:
            time.sleep(0.005)

    # ─────────────────────────────────────────────────────────────────
    # cmd_vel callback
    # ─────────────────────────────────────────────────────────────────

    def _set_left_vel(self, vel_rev_s: float):
        self.can4wd.set_input_vel(self.node_ids["FL"], vel_rev_s, 0.0)
        self.can4wd.set_input_vel(self.node_ids["RL"], vel_rev_s, 0.0)

    def _set_right_vel(self, vel_rev_s: float):
        if self.right_disabled:
            return
        self.can4wd.set_input_vel(self.node_ids["RR"], vel_rev_s, 0.0)
        if not self.right_front_disabled:
            self.can4wd.set_input_vel(self.node_ids["FR"], vel_rev_s, 0.0)

    def cmd_cb(self, msg: Twist):
        v = float(msg.linear.x)
        w = float(msg.angular.z)

        CIRC = self.odom_solver.CIRC
        L = self.odom_solver.L

        wr = (v + 0.5 * L * w) / CIRC
        wl = (v - 0.5 * L * w) / CIRC

        now_s = time.time()
        last_wr, last_wl = self._last_sent
        if last_wr is not None:
            if (abs(wr - last_wr) < self._eps and abs(wl - last_wl) < self._eps and
                    (now_s - self._last_sent_t) < self._min_send_period):
                self.last_cmd_monotonic = time.monotonic()
                return

        self._set_right_vel(self.sign_r * wr)
        self._set_left_vel(self.sign_l * wl)

        self._last_sent = (wr, wl)
        self._last_sent_t = now_s
        self.last_cmd_monotonic = time.monotonic()
        self._stopped_due_timeout = False

    # ─────────────────────────────────────────────────────────────────
    # Loop principal
    # ─────────────────────────────────────────────────────────────────

    def loop_cb(self):
        now = self.get_clock().now()
        elapsed = time.monotonic() - self.last_cmd_monotonic

        if elapsed > self.cmd_timeout:
            if not self._stopped_due_timeout:
                self._set_right_vel(0.0)
                self._set_left_vel(0.0)
                self._stopped_due_timeout = True
        else:
            self._stopped_due_timeout = False

        l_turns_fl, _ = self.can4wd.read_cached(self.node_ids["FL"], max_age=0.5)
        l_turns_rl, _ = self.can4wd.read_cached(self.node_ids["RL"], max_age=0.5)
        avg_l = None
        if l_turns_fl is not None and l_turns_rl is not None:
            avg_l = (l_turns_fl + l_turns_rl) * 0.5

        r_turns_fr = r_turns_rr = None
        if self.right_disabled:
            avg_r = avg_l
        elif self.right_front_disabled:
            r_turns_rr, _ = self.can4wd.read_cached(self.node_ids["RR"], max_age=0.5)
            avg_r = r_turns_rr
        else:
            r_turns_fr, _ = self.can4wd.read_cached(self.node_ids["FR"], max_age=0.5)
            r_turns_rr, _ = self.can4wd.read_cached(self.node_ids["RR"], max_age=0.5)
            avg_r = None
            if r_turns_fr is not None and r_turns_rr is not None:
                avg_r = (r_turns_fr + r_turns_rr) * 0.5

        if avg_l is not None and avg_r is not None:
            x, y, yaw = self.odom_solver.update_from_turns(pos_l_raw_turns=avg_l, pos_r_raw_turns=avg_r)
        else:
            x, y, yaw = self.odom_solver.x, self.odom_solver.y, self.odom_solver.theta

        telemetry_msg = String()
        telemetry_msg.data = json.dumps(
            {
                "backend": "can_4wd",
                "positions_turns": {
                    "FL": l_turns_fl, "RL": l_turns_rl,
                    "FR": r_turns_fr, "RR": r_turns_rr,
                },
                "node_ids": self.node_ids,
            },
            separators=(",", ":"),
        )
        self.pub_telemetry.publish(telemetry_msg)

        od = self.odom_msg
        od.header.stamp = now.to_msg()
        od.pose.pose.position.x = float(x)
        od.pose.pose.position.y = float(y)
        od.pose.pose.position.z = 0.0
        od.pose.pose.orientation = quat_from_yaw(yaw)
        od.twist.twist.linear.x = float(self.odom_solver.v)
        od.twist.twist.linear.y = 0.0
        od.twist.twist.angular.z = float(self.odom_solver.w)
        self.pub.publish(od)

        if self.br is not None:
            tfm = TransformStamped()
            tfm.header.stamp = now.to_msg()
            tfm.header.frame_id = self.odom_frame
            tfm.child_frame_id = self.base_frame
            tfm.transform.translation.x = float(x)
            tfm.transform.translation.y = float(y)
            tfm.transform.translation.z = 0.0
            tfm.transform.rotation = quat_from_yaw(yaw)
            self.br.sendTransform(tfm)

    # ─────────────────────────────────────────────────────────────────
    # Drive status (1 Hz)
    # ─────────────────────────────────────────────────────────────────

    def _axis_status(self, label: str) -> dict:
        if label not in self.active_wheels:
            return {"label": label, "connected": False}
        nid = self.node_ids[label]
        hb = self.can4wd.read_heartbeat_cached(nid, max_age=1.0)
        if hb is None:
            return {"label": label, "connected": False}
        state = to_int(hb.get("Axis_State"), 0)
        axis_error = to_int(hb.get("Axis_Error"), 0)
        return {
            "label": label,
            "connected": True,
            "axis_state": state,
            "closed_loop": state == AXIS_STATE_CLOSED_LOOP_CONTROL,
            "axis_error": axis_error,
            "has_error": axis_error != 0,
        }

    def publish_drive_status(self):
        try:
            payload = {
                "connected": True,
                "backend": "can_4wd",
                "axes": {w: self._axis_status(w) for w in WHEELS},
            }
            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False)
            self.pub_drive.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"publish_drive_status falhou: {e}")

    # ─────────────────────────────────────────────────────────────────
    # Shutdown
    # ─────────────────────────────────────────────────────────────────

    def destroy_node(self):
        try:
            self.can4wd.safe_idle()
        except Exception:
            self.get_logger().warn("Falha ao enviar safe_idle para ODrive.")
        try:
            self.can4wd.stop()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdom4WDCANNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
