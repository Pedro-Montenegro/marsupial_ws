#!/usr/bin/env python3
"""
odom_node.py — versão corrigida para NÃO estourar buffer (Errno 105) e para usar CAN nativo (can0) por padrão.

Principais mudanças:
- Default de CAN: can0 (mttcan), não slcan0
- Watchdog: manda 0/0 UMA vez quando entra em timeout (não spamma)
- cmd_vel: rate-limit + "send only on change"
- RTR encoder: taxa reduzida (20 Hz total) e proteção contra overflow
- CAN send: try/except em todos os sends (não derruba o node por CanOperationError)
- (opcional) txqueuelen deve ser configurado no SO (você já fez)
"""

import math
import time
import json
import threading
from typing import Optional, Tuple, Callable

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from tf2_ros import TransformBroadcaster

# --- CAN / ODrive ---
import can
import cantools

from .diff_odom import DiffOdom


def quat_from_yaw(yaw: float) -> Quaternion:
    half = yaw * 0.5
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))

def to_int(v, default=0) -> int:
    """Converte valores do cantools (inclui NamedSignalValue) para int, sem quebrar."""
    if v is None:
        return default
    # cantools.database.namedsignalvalue.NamedSignalValue
    if hasattr(v, "value"):
        try:
            return int(v.value)
        except Exception:
            pass
    # bytes / bytearray
    if isinstance(v, (bytes, bytearray)):
        return int(v[0]) if len(v) else default
    # bool / int / float / str numérica
    try:
        return int(v)
    except Exception:
        return default

class ODriveCAN:
    """
    Wrapper para ODrive CANSimple via DBC.
    Implementa:
      - envio protegido (não crasha se TX buffer encher)
      - RTR async (send_enc_rtr) + cache via thread RX
    """

    def __init__(
        self,
        channel: str = "can0",
        dbc_path: str = "odrive-cansimple.dbc",
        right_axis: int = 0,
        left_axis: int = 1,
        timeout: float = 0.02,
        log: Optional[Callable[[str], None]] = None,   # logger simples opcional
    ):
        self.bus = can.interface.Bus(channel=channel, interface="socketcan")
        self.db = cantools.database.load_file(dbc_path)
        self.right_axis = int(right_axis)
        self.left_axis = int(left_axis)
        self.timeout = float(timeout)
        self._log = log

        

        self.msg = {}
        for ax in (0, 1):
            self.msg[(ax, "set_axis_state")] = self.db.get_message_by_name(
                f"Axis{ax}_Set_Axis_State"
            )
            self.msg[(ax, "set_ctrl_mode")] = self.db.get_message_by_name(
                f"Axis{ax}_Set_Controller_Mode"
            )
            self.msg[(ax, "set_input_vel")] = self.db.get_message_by_name(
                f"Axis{ax}_Set_Input_Vel"
            )
            self.msg[(ax, "get_enc_est")] = self.db.get_message_by_name(
                f"Axis{ax}_Get_Encoder_Estimates"
            )
            self.msg[(ax, "heartbeat")] = self.db.get_message_by_name(
                f"Axis{ax}_Heartbeat"
            )

        # Filtro RX: só o que interessa pra cache de encoder
        try:
            self.bus.set_filters([
                # encoder estimates
                {"can_id": self.msg[(0, "get_enc_est")].frame_id, "can_mask": 0x7FF, "extended": False},
                {"can_id": self.msg[(1, "get_enc_est")].frame_id, "can_mask": 0x7FF, "extended": False},
                # heartbeat
                {"can_id": self.msg[(0, "heartbeat")].frame_id, "can_mask": 0x7FF, "extended": False},
                {"can_id": self.msg[(1, "heartbeat")].frame_id, "can_mask": 0x7FF, "extended": False},
            ])
        except Exception:
            pass

        # cache: axis -> (pos, vel, timestamp)
        self._enc_cache = {0: (None, None, 0.0), 1: (None, None, 0.0)}

        # cache heartbeat: axis -> dict + timestamp
        self._hb_cache = {
            0: (None, 0.0),  # (dict, ts)
            1: (None, 0.0),
        }

        self._stop_rx = False

        def _rx_loop():
            while not self._stop_rx:
                try:
                    resp = self.bus.recv(timeout=0.01)
                except Exception:
                    continue
                if resp is None:
                    continue

                ts = time.time()
                for ax in (0, 1):
                    m = self.msg[(ax, "get_enc_est")]
                    if resp.arbitration_id == m.frame_id and len(resp.data) == m.length:
                        try:
                            vals = m.decode(resp.data)
                        except Exception:
                            continue
                        pos = vals.get("Pos_Estimate", None)
                        vel = vals.get("Vel_Estimate", None)
                        self._enc_cache[ax] = (pos, vel, ts)
                    hb = self.msg[(ax, "heartbeat")]
                    if resp.arbitration_id == hb.frame_id and len(resp.data) == hb.length:
                        try:
                            vals = hb.decode(resp.data)
                        except Exception:
                            continue
                        self._hb_cache[ax] = (vals, ts)

        self._rx_thread = threading.Thread(target=_rx_loop, daemon=True)
        self._rx_thread.start()

    def _safe_log(self, s: str):
        if self._log:
            try:
                self._log(s)
            except Exception:
                pass

    def _send(self, m, data_dict=None, raw=b"") -> bool:
        """
        Retorna True se enviou; False se falhou (ex: Errno 105).
        NÃO lança exceção para o caller.
        """
        data = m.encode(data_dict) if data_dict is not None else raw
        msg = can.Message(arbitration_id=m.frame_id, data=data, is_extended_id=False)
        try:
            self.bus.send(msg)
            return True
        except can.CanOperationError as e:
            # Errno 105 (No buffer space) cai aqui
            self._safe_log(f"[CAN] send falhou: {e}")
            return False
        except OSError as e:
            self._safe_log(f"[CAN] OSError no send: {e}")
            return False

    def set_axis_state(self, axis: int, state: int) -> bool:
        m = self.msg[(axis, "set_axis_state")]
        return self._send(m, {"Axis_Requested_State": int(state)})

    def set_controller_mode_velocity(
        self, axis: int, input_mode: int = 1, control_mode: int = 2
    ) -> bool:
        m = self.msg[(axis, "set_ctrl_mode")]
        return self._send(m, {"Input_Mode": int(input_mode), "Control_Mode": int(control_mode)})

    def set_input_vel(self, axis: int, vel_rev_s: float, torque_ff: float = 0.0) -> bool:
        m = self.msg[(axis, "set_input_vel")]
        return self._send(m, {"Input_Vel": float(vel_rev_s), "Input_Torque_FF": float(torque_ff)})

    def send_enc_rtr(self, axis: int) -> bool:
        """Envia RTR para pedir encoder estimates (não bloqueante)."""
        req = self.msg[(axis, "get_enc_est")]
        msg = can.Message(
            arbitration_id=req.frame_id,
            is_extended_id=False,
            is_remote_frame=True,
            dlc=0,
        )
        try:
            self.bus.send(msg)
            return True
        except (can.CanOperationError, OSError) as e:
            self._safe_log(f"[CAN] RTR falhou: {e}")
            return False

    def read_cached(self, axis: int, max_age: float = 0.2) -> Tuple[Optional[float], Optional[float]]:
        pos, vel, ts = self._enc_cache.get(axis, (None, None, 0.0))
        if pos is None:
            return None, None
        if (time.time() - ts) > max_age:
            return None, None
        return pos, vel

    def read_heartbeat_cached(self, axis: int, max_age: float = 0.8):
        d, ts = self._hb_cache.get(axis, (None, 0.0))
        if d is None:
            return None
        if (time.time() - ts) > max_age:
            return None
        return d

    def safe_idle(self, right_axis: int, left_axis: int, tries: int = 3):
        AXIS_STATE_IDLE = 1
        for _ in range(tries):
            try:
                self.set_input_vel(right_axis, 0.0, 0.0)
                self.set_input_vel(left_axis, 0.0, 0.0)
                time.sleep(0.02)
                self.set_axis_state(right_axis, AXIS_STATE_IDLE)
                self.set_axis_state(left_axis, AXIS_STATE_IDLE)
                return
            except Exception:
                time.sleep(0.05)

    def stop(self):
        self._stop_rx = True
        try:
            self._rx_thread.join(timeout=0.2)
        except Exception:
            pass


class WheelOdomNode(Node):
    """
    Nó ROS2 para controle diferencial via ODrive CAN e publicação de odometria.
    """

    def __init__(self):
        super().__init__("odom_node")

        # ---- Parâmetros ----
        self.declare_parameter("wheel_radius", 0.082)
        self.declare_parameter("wheel_base", 0.31)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_topic", "wheel/odom")
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("drive_status_topic", "ui/drive_status")
        self.declare_parameter("hz", 20.0)
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("cmd_timeout", 0.5)

        # <<<<<< MUDANÇA: default can0
        self.declare_parameter("can_channel", "can0")
        self.declare_parameter("can_dbc", "odrive-cansimple.dbc")
        self.declare_parameter("right_id", 0)
        self.declare_parameter("left_id", 1)

        wheel_radius = float(self.get_parameter("wheel_radius").value)
        wheel_base = float(self.get_parameter("wheel_base").value)
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        odom_topic = self.get_parameter("odom_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        drive_status_topic = self.get_parameter("drive_status_topic").value
        hz = float(self.get_parameter("hz").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        can_channel = self.get_parameter("can_channel").value
        can_dbc = self.get_parameter("can_dbc").value
        self.right_axis = int(self.get_parameter("right_id").value)
        self.left_axis = int(self.get_parameter("left_id").value)

        self.pub_drive = self.create_publisher(String, drive_status_topic, 10)
        self.create_timer(1.0, self.publish_drive_status)  # 1 Hz

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pub = self.create_publisher(Odometry, odom_topic, qos)
        self.br = TransformBroadcaster(self) if self.publish_tf else None

        # ---- ODrive/CAN ----
        self.get_logger().info(f"Conectando ODrive via CAN: {can_channel}")
        try:
            self.canodrv = ODriveCAN(
                channel=can_channel,
                dbc_path=can_dbc,
                right_axis=self.right_axis,
                left_axis=self.left_axis,
                timeout=0.05,
                log=lambda s: self.get_logger().warn(s),
            )
        except Exception as e:
            self.get_logger().error(f"Falha ao iniciar CAN/DBC: {e}")
            raise

        AXIS_STATE_CLOSED_LOOP_CONTROL = 8
        for ax in (self.right_axis, self.left_axis):
            self.canodrv.set_controller_mode_velocity(ax, input_mode=1, control_mode=2)
            self.canodrv.set_axis_state(ax, AXIS_STATE_CLOSED_LOOP_CONTROL)

        # ---- Odometria ----
        CIRC = 2.0 * math.pi * wheel_radius
        self.odom_solver = DiffOdom(L=wheel_base, CIRC=CIRC, sign_l=-1.0, sign_r=+1.0)
        self.odom_solver.reset()

        # ---- cmd_vel ----
        self.last_cmd_stamp = self.get_clock().now()
        self.sub_cmd = self.create_subscription(Twist, cmd_vel_topic, self.cmd_cb, qos)

        # ---- Anti-spam / rate limit ----
        self._stopped_due_timeout = False

        # rate-limit do envio de set_input_vel no cmd_cb
        self._last_sent = (None, None)     # (wr, wl)
        self._last_sent_t = 0.0
        self._min_send_period = 0.02       # 50 Hz max
        self._eps = 1e-4                   # tolerância para "mudou"

        # ---- Timer Principal ----
        period = 1.0 / max(1.0, hz)
        self.timer = self.create_timer(period, self.loop_cb)
        self.get_logger().info("wheel_odom_node_can (ROS2) pronto.")

        # ---- Mensagem Odom (reuso) ----
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = self.odom_frame
        self.odom_msg.child_frame_id = self.base_frame

        # ---- RTR timer (reduzido) ----
        # 0.05s = 20 Hz total -> 10 Hz por eixo alternando
        self._rtr_axis_toggle = 0
        self.timer_rtr = self.create_timer(0.05, self._enc_rtr_cb)

    def _enc_rtr_cb(self):
        # Alterna eixos para não inundar o barramento
        if self._rtr_axis_toggle == 0:
            ok = self.canodrv.send_enc_rtr(self.left_axis)
            self._rtr_axis_toggle = 1
        else:
            ok = self.canodrv.send_enc_rtr(self.right_axis)
            self._rtr_axis_toggle = 0

        # Se falhou, não tente "compensar" mandando mais — apenas deixa quieto.
        if not ok:
            # micro backoff evita loop apertado em caso de falha persistente
            time.sleep(0.005)

    def cmd_cb(self, msg: Twist):
        v = float(msg.linear.x)
        w = float(msg.angular.z)

        CIRC = self.odom_solver.CIRC
        L = self.odom_solver.L

        wr = (v + 0.5 * L * w) / CIRC
        wl = (v - 0.5 * L * w) / CIRC

        now_s = time.time()
        last_wr, last_wl = self._last_sent

        # send only on change + rate limit
        if last_wr is not None:
            if (abs(wr - last_wr) < self._eps and abs(wl - last_wl) < self._eps and
                    (now_s - self._last_sent_t) < self._min_send_period):
                self.last_cmd_stamp = self.get_clock().now()
                return

        # Envia (2 frames) — protegido no ODriveCAN (não crasha)
        self.canodrv.set_input_vel(self.right_axis, +wr, 0.0)
        self.canodrv.set_input_vel(self.left_axis, -wl, 0.0)

        self._last_sent = (wr, wl)
        self._last_sent_t = now_s
        self.last_cmd_stamp = self.get_clock().now()

        # saiu do estado "timeout stop"
        self._stopped_due_timeout = False

    def loop_cb(self):
        now = self.get_clock().now()

        # Watchdog de cmd_vel: manda 0 UMA vez ao entrar em timeout
        if (now - self.last_cmd_stamp).nanoseconds * 1e-9 > self.cmd_timeout:
            if not self._stopped_due_timeout:
                self.canodrv.set_input_vel(self.right_axis, 0.0, 0.0)
                self.canodrv.set_input_vel(self.left_axis, 0.0, 0.0)
                self._stopped_due_timeout = True
        else:
            self._stopped_due_timeout = False

        # Leitura não bloqueante do cache
        l_turns, _ = self.canodrv.read_cached(self.left_axis, max_age=0.5)
        r_turns, _ = self.canodrv.read_cached(self.right_axis, max_age=0.5)

        if (l_turns is not None) and (r_turns is not None):
            x, y, yaw = self.odom_solver.update_from_turns(
                pos_l_raw_turns=l_turns,
                pos_r_raw_turns=r_turns,
            )
        else:
            x, y, yaw = self.odom_solver.x, self.odom_solver.y, self.odom_solver.theta

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

    def publish_drive_status(self):
        now = time.time()

        def axis_payload(ax_id: int):
            hb = self.canodrv.read_heartbeat_cached(ax_id, max_age=1.0)
            if hb is None:
                return {
                    "connected": False,
                    "axis_state": None,
                    "axis_state_name": None,
                    "closed_loop": False,
                    "axis_error": None,
                    "has_error": False,
                    "flags": {},
                }

            # pega raw (pode ser NamedSignalValue) e também o int
            state_raw = hb.get("Axis_State")
            state = to_int(state_raw, 0)
            axis_error = to_int(hb.get("Axis_Error"), 0)

            flags = {
                "motor": bool(to_int(hb.get("Motor_Error_Flag"), 0)),
                "encoder": bool(to_int(hb.get("Encoder_Error_Flag"), 0)),
                "controller": bool(to_int(hb.get("Controller_Error_Flag"), 0)),
            }

            has_error = (axis_error != 0) or any(flags.values())
            closed_loop = (state == 8)  # CLOSED_LOOP_CONTROL

            return {
                "connected": True,
                "axis_state": state,
                "axis_state_name": str(state_raw) if state_raw is not None else None,
                "closed_loop": closed_loop,
                "axis_error": axis_error,
                "has_error": has_error,
                "flags": flags,
            }

        try:
            payload = {
                "connected": True,
                "axis0": axis_payload(0),
                "axis1": axis_payload(1),
            }
            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False)
            self.pub_drive.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"publish_drive_status falhou: {e}")


    def destroy_node(self):
        try:
            self.canodrv.safe_idle(self.right_axis, self.left_axis)
        except Exception:
            self.get_logger().warn("Falha ao enviar safe_idle para ODrive.")
        try:
            self.canodrv.stop()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()