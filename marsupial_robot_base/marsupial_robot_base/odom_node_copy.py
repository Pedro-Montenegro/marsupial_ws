#!/usr/bin/env python3
"""
odom_node.py

Controle diferencial via ODrive CANSimple + publicação de odometria.

Mudanças em relação à versão anterior:
  1. clear_errors() chamado no boot para limpar WATCHDOG_TIMER_EXPIRED acumulado
  2. loop_cb envia Set_Input_Vel(0,0) CONTINUAMENTE quando cmd_vel está em timeout
     (não só uma vez) — serve como keepalive para o watchdog do ODrive
  3. ODriveCAN.clear_errors() adicionado
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

import can
import cantools

from .diff_odom import DiffOdom


def quat_from_yaw(yaw: float) -> Quaternion:
    half = yaw * 0.5
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


def to_int(v, default=0) -> int:
    """Converte valores do cantools (inclui NamedSignalValue) para int."""
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


class ODriveCAN:
    """
    Wrapper para ODrive CANSimple via DBC.
    Versão 0.5.6.

    Funcionalidades:
      - Envio protegido (não crasha se TX buffer encher)
      - RTR async (send_enc_rtr) + cache via thread RX
      - clear_errors() para limpar erros no boot (ex: WATCHDOG_TIMER_EXPIRED)
    """

    def __init__(
        self,
        channel: str = "can0",
        dbc_path: str = "odrive-cansimple.dbc",
        right_axis: int = 0,
        left_axis: int = 1,
        timeout: float = 0.02,
        log: Optional[Callable[[str], None]] = None,
    ):
        self.bus = can.interface.Bus(channel=channel, interface="socketcan")
        self.db = cantools.database.load_file(dbc_path)
        self.right_axis = int(right_axis)
        self.left_axis = int(left_axis)
        self.timeout = float(timeout)
        self._log = log

        # Pré-carrega mensagens do DBC para cada eixo
        self.msg = {}
        for ax in (0, 1):
            self.msg[(ax, "set_axis_state")]  = self.db.get_message_by_name(f"Axis{ax}_Set_Axis_State")
            self.msg[(ax, "set_ctrl_mode")]   = self.db.get_message_by_name(f"Axis{ax}_Set_Controller_Mode")
            self.msg[(ax, "set_input_vel")]   = self.db.get_message_by_name(f"Axis{ax}_Set_Input_Vel")
            self.msg[(ax, "get_enc_est")]     = self.db.get_message_by_name(f"Axis{ax}_Get_Encoder_Estimates")
            self.msg[(ax, "heartbeat")]       = self.db.get_message_by_name(f"Axis{ax}_Heartbeat")
            self.msg[(ax, "clear_errors")]    = self.db.get_message_by_name(f"Axis{ax}_Clear_Errors")

        # Filtro RX: só encoder estimates e heartbeat
        try:
            self.bus.set_filters([
                {"can_id": self.msg[(0, "get_enc_est")].frame_id, "can_mask": 0x7FF, "extended": False},
                {"can_id": self.msg[(1, "get_enc_est")].frame_id, "can_mask": 0x7FF, "extended": False},
                {"can_id": self.msg[(0, "heartbeat")].frame_id,   "can_mask": 0x7FF, "extended": False},
                {"can_id": self.msg[(1, "heartbeat")].frame_id,   "can_mask": 0x7FF, "extended": False},
            ])
        except Exception:
            pass

        # Cache encoder: axis -> (pos, vel, timestamp)
        self._enc_cache = {0: (None, None, 0.0), 1: (None, None, 0.0)}

        # Cache heartbeat: axis -> (dict, timestamp)
        self._hb_cache = {0: (None, 0.0), 1: (None, 0.0)}

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
                    # Encoder estimates
                    m = self.msg[(ax, "get_enc_est")]
                    if resp.arbitration_id == m.frame_id and len(resp.data) == m.length:
                        try:
                            vals = m.decode(resp.data)
                            pos = vals.get("Pos_Estimate")
                            vel = vals.get("Vel_Estimate")
                            self._enc_cache[ax] = (pos, vel, ts)
                        except Exception:
                            pass

                    # Heartbeat
                    hb = self.msg[(ax, "heartbeat")]
                    if resp.arbitration_id == hb.frame_id and len(resp.data) == hb.length:
                        try:
                            vals = hb.decode(resp.data)
                            self._hb_cache[ax] = (vals, ts)
                        except Exception:
                            pass

        self._rx_thread = threading.Thread(target=_rx_loop, daemon=True)
        self._rx_thread.start()

    def _safe_log(self, s: str):
        if self._log:
            try:
                self._log(s)
            except Exception:
                pass

    def _send(self, m, data_dict=None, raw: bytes = b"") -> bool:
        """
        Envia mensagem CAN de forma protegida.
        Retorna True se enviou, False se falhou (ex: Errno 105).
        Nunca lança exceção para o caller.
        """
        data = m.encode(data_dict) if data_dict is not None else raw
        msg = can.Message(arbitration_id=m.frame_id, data=data, is_extended_id=False)
        try:
            self.bus.send(msg)
            return True
        except can.CanOperationError as e:
            self._safe_log(f"[CAN] send falhou (CanOperationError): {e}")
            return False
        except OSError as e:
            self._safe_log(f"[CAN] send falhou (OSError): {e}")
            return False

    def _send_empty(self, m) -> bool:
        """
        Envia mensagem CAN com payload vazio (ex: Clear_Errors, Reboot).
        """
        msg = can.Message(
            arbitration_id=m.frame_id,
            data=b"",
            is_extended_id=False,
        )
        try:
            self.bus.send(msg)
            return True
        except (can.CanOperationError, OSError) as e:
            self._safe_log(f"[CAN] send_empty falhou: {e}")
            return False

    # ── Comandos ─────────────────────────────────────────────────────

    def clear_errors(self, axis: int) -> bool:
        """
        Envia Clear_Errors para o eixo especificado.
        Limpa erros acumulados no boot, incluindo WATCHDOG_TIMER_EXPIRED.
        Deve ser chamado antes de colocar o eixo em CLOSED_LOOP_CONTROL.
        """
        return self._send_empty(self.msg[(axis, "clear_errors")])

    def set_axis_state(self, axis: int, state: int) -> bool:
        return self._send(self.msg[(axis, "set_axis_state")], {"Axis_Requested_State": int(state)})

    def set_controller_mode_velocity(self, axis: int, input_mode: int = 1, control_mode: int = 2) -> bool:
        return self._send(
            self.msg[(axis, "set_ctrl_mode")],
            {"Input_Mode": int(input_mode), "Control_Mode": int(control_mode)},
        )

    def set_input_vel(self, axis: int, vel_rev_s: float, torque_ff: float = 0.0) -> bool:
        return self._send(
            self.msg[(axis, "set_input_vel")],
            {"Input_Vel": float(vel_rev_s), "Input_Torque_FF": float(torque_ff)},
        )

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

    # ── Leitura de cache ─────────────────────────────────────────────

    def read_cached(self, axis: int, max_age: float = 0.2) -> Tuple[Optional[float], Optional[float]]:
        pos, vel, ts = self._enc_cache.get(axis, (None, None, 0.0))
        if pos is None or (time.time() - ts) > max_age:
            return None, None
        return pos, vel

    def read_heartbeat_cached(self, axis: int, max_age: float = 0.8):
        d, ts = self._hb_cache.get(axis, (None, 0.0))
        if d is None or (time.time() - ts) > max_age:
            return None
        return d

    # ── Shutdown seguro ──────────────────────────────────────────────

    def safe_idle(self, right_axis: int, left_axis: int, tries: int = 3):
        """Para os motores e coloca em IDLE. Chamado no destroy_node."""
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

    def __init__(self):
        super().__init__("odom_node")

        # ── Parâmetros ────────────────────────────────────────────────
        self.declare_parameter("wheel_radius",       0.082)
        self.declare_parameter("wheel_base",         0.31)
        self.declare_parameter("odom_frame",         "odom")
        self.declare_parameter("base_frame",         "base_link")
        self.declare_parameter("odom_topic",         "wheel/odom")
        self.declare_parameter("cmd_vel_topic",      "cmd_vel")
        self.declare_parameter("drive_status_topic", "ui/drive_status")
        self.declare_parameter("hz",                 20.0)
        self.declare_parameter("publish_tf",         False)
        self.declare_parameter("cmd_timeout",        0.5)
        self.declare_parameter("can_channel",        "can0")
        self.declare_parameter("can_dbc",            "odrive-cansimple.dbc")
        self.declare_parameter("right_id",           0)
        self.declare_parameter("left_id",            1)
        self.declare_parameter("ff_kv",              0.05)
        self.declare_parameter("ff_ks",              0.03)
        self.declare_parameter("ff_ks_spin",  0.28)   # atrito estático rotação pura
        # self.declare_parameter("ff_ks_spin",  0.15)   # atrito estático rotação pura
        self.declare_parameter("spin_v_thresh", 0.05) # abaixo disso considera rotação pura
        self.declare_parameter("enable_pure_spin_ff", False)
        self.declare_parameter("ff_ka",              0.00)

        wheel_radius        = float(self.get_parameter("wheel_radius").value)
        wheel_base          = float(self.get_parameter("wheel_base").value)
        self.odom_frame     = self.get_parameter("odom_frame").value
        self.base_frame     = self.get_parameter("base_frame").value
        odom_topic          = self.get_parameter("odom_topic").value
        cmd_vel_topic       = self.get_parameter("cmd_vel_topic").value
        drive_status_topic  = self.get_parameter("drive_status_topic").value
        hz                  = float(self.get_parameter("hz").value)
        self.publish_tf     = bool(self.get_parameter("publish_tf").value)
        self.cmd_timeout    = float(self.get_parameter("cmd_timeout").value)
        can_channel         = self.get_parameter("can_channel").value
        can_dbc             = self.get_parameter("can_dbc").value
        self.right_axis     = int(self.get_parameter("right_id").value)
        self.left_axis      = int(self.get_parameter("left_id").value)
        self.ff_kv          = float(self.get_parameter("ff_kv").value)
        self.ff_ks          = float(self.get_parameter("ff_ks").value)
        self.ff_ka          = float(self.get_parameter("ff_ka").value)
        self.ff_ks_spin    = float(self.get_parameter("ff_ks_spin").value)
        self.spin_v_thresh = float(self.get_parameter("spin_v_thresh").value)
        self.enable_pure_spin_ff = bool(
        self.get_parameter("enable_pure_spin_ff").value
)

        # ── Publishers ────────────────────────────────────────────────
        self.pub_drive = self.create_publisher(String, drive_status_topic, 10)
        self.create_timer(1.0, self.publish_drive_status)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(Odometry, odom_topic, qos)
        self.br  = TransformBroadcaster(self) if self.publish_tf else None

        # ── ODrive/CAN ────────────────────────────────────────────────
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

        # ── Inicialização dos eixos ───────────────────────────────────
        # Passo 1: limpa erros acumulados no boot
        #   O ODrive pode ter WATCHDOG_TIMER_EXPIRED se o watchdog estiver
        #   habilitado e o stack ROS ainda não estava rodando.
        self.get_logger().info("Limpando erros do ODrive...")
        for ax in (self.right_axis, self.left_axis):
            self.canodrv.clear_errors(ax)
            time.sleep(0.05)

        # Passo 2: configura modo de controle e coloca em closed loop
        AXIS_STATE_CLOSED_LOOP_CONTROL = 8
        for ax in (self.right_axis, self.left_axis):
            self.canodrv.set_controller_mode_velocity(ax, input_mode=1, control_mode=2)
            self.canodrv.set_axis_state(ax, AXIS_STATE_CLOSED_LOOP_CONTROL)

        self.get_logger().info("ODrive inicializado em CLOSED_LOOP_CONTROL.")

        # ── Odometria ─────────────────────────────────────────────────
        CIRC = 2.0 * math.pi * wheel_radius
        self.odom_solver = DiffOdom(L=wheel_base, CIRC=CIRC, sign_l=-1.0, sign_r=+1.0)
        self.odom_solver.reset()

        # ── Estado de cmd_vel ─────────────────────────────────────────
        self.last_cmd_stamp     = self.get_clock().now()
        self._stopped_due_timeout = False

        # Rate-limit do envio de set_input_vel no cmd_cb
        self._last_sent         = (None, None)  # (wr, wl)
        self._last_sent_t       = 0.0
        self._min_send_period   = 0.02           # 50 Hz máx
        self._eps               = 1e-4

        # Feedforward
        self._prev_wr  = 0.0
        self._prev_wl  = 0.0
        self._prev_cmd_t = time.time()

        # ── Subscribers ───────────────────────────────────────────────
        self.sub_cmd = self.create_subscription(Twist, cmd_vel_topic, self.cmd_cb, qos)

        # ── Timers ────────────────────────────────────────────────────
        period = 1.0 / max(1.0, hz)
        self.timer     = self.create_timer(period, self.loop_cb)

        # RTR encoder: 20Hz total, alternando entre eixos (10Hz por eixo)
        self._rtr_axis_toggle = 0
        self.timer_rtr = self.create_timer(0.05, self._enc_rtr_cb)

        # ── Mensagem Odom (reuso) ─────────────────────────────────────
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = self.odom_frame
        self.odom_msg.child_frame_id  = self.base_frame

        self.get_logger().info("WheelOdomNode pronto.")

    # ─────────────────────────────────────────────────────────────────
    # Feedforward
    # ─────────────────────────────────────────────────────────────────

    def _compute_ff(self, w_target: float, w_prev: float, dt: float, is_pure_spin: bool = False) -> float:
        if dt <= 0 or dt > 0.5:
            accel = 0.0
        else:
            accel = (w_target - w_prev) / dt

        # Usa ff_ks_spin quando é rotação pura (sem vel linear)
        # para vencer o atrito estático maior do carpete
        ks = self.ff_ks_spin if is_pure_spin else self.ff_ks

        ff = 0.0
        ff += self.ff_kv * w_target
        if abs(w_target) > self._eps:
            ff += ks * math.copysign(1.0, w_target)
        ff += self.ff_ka * accel
        return ff

    # ─────────────────────────────────────────────────────────────────
    # RTR encoder
    # ─────────────────────────────────────────────────────────────────

    def _enc_rtr_cb(self):
        if self._rtr_axis_toggle == 0:
            ok = self.canodrv.send_enc_rtr(self.left_axis)
            self._rtr_axis_toggle = 1
        else:
            ok = self.canodrv.send_enc_rtr(self.right_axis)
            self._rtr_axis_toggle = 0

        if not ok:
            time.sleep(0.005)

    # ─────────────────────────────────────────────────────────────────
    # cmd_vel callback
    # ─────────────────────────────────────────────────────────────────

    def cmd_cb(self, msg: Twist):
        v = float(msg.linear.x)
        w = float(msg.angular.z)

        CIRC = self.odom_solver.CIRC
        L    = self.odom_solver.L

        wr = (v + 0.5 * L * w) / CIRC
        wl = (v - 0.5 * L * w) / CIRC

        now_s = time.time()
        dt    = now_s - self._prev_cmd_t

        last_wr, last_wl = self._last_sent
        if last_wr is not None:
            if (abs(wr - last_wr) < self._eps
                    and abs(wl - last_wl) < self._eps
                    and (now_s - self._last_sent_t) < self._min_send_period):
                self.last_cmd_stamp = self.get_clock().now()
                return

        # Rotação pura: sem vel linear e com vel angular
        # is_pure_spin = abs(v) < self.spin_v_thresh and abs(w) > self._eps
        is_pure_spin = (
            self.enable_pure_spin_ff
            and abs(v) < self.spin_v_thresh
            and abs(w) > self._eps
)

        ff_r = self._compute_ff(wr,  self._prev_wr, dt, is_pure_spin)
        ff_l = self._compute_ff(-wl, self._prev_wl, dt, is_pure_spin)

        self.canodrv.set_input_vel(self.right_axis, +wr,  ff_r)
        self.canodrv.set_input_vel(self.left_axis,  -wl,  ff_l)

        self._prev_wr     = wr
        self._prev_wl     = -wl
        self._prev_cmd_t  = now_s
        self._last_sent   = (wr, wl)
        self._last_sent_t = now_s
        self.last_cmd_stamp = self.get_clock().now()
        self._stopped_due_timeout = False

    # ─────────────────────────────────────────────────────────────────
    # Loop principal (20Hz)
    # ─────────────────────────────────────────────────────────────────

    def loop_cb(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_cmd_stamp).nanoseconds * 1e-9

        if elapsed > self.cmd_timeout:
            # Envia zero CONTINUAMENTE enquanto em timeout.
            # Isso serve como keepalive para o watchdog do ODrive —
            # diferente da versão anterior que enviava zero só uma vez.
            # Com watchdog habilitado (timeout=1.0s), o ODrive para os
            # motores automaticamente se a Jetson cair e esse keepalive
            # parar de chegar.
            self.canodrv.set_input_vel(self.right_axis, 0.0, 0.0)
            self.canodrv.set_input_vel(self.left_axis,  0.0, 0.0)
            self._stopped_due_timeout = True
        else:
            self._stopped_due_timeout = False

        # Leitura não bloqueante do cache de encoder
        l_turns, _ = self.canodrv.read_cached(self.left_axis,  max_age=0.5)
        r_turns, _ = self.canodrv.read_cached(self.right_axis, max_age=0.5)

        if l_turns is not None and r_turns is not None:
            x, y, yaw = self.odom_solver.update_from_turns(
                pos_l_raw_turns=l_turns,
                pos_r_raw_turns=r_turns,
            )
        else:
            x, y, yaw = self.odom_solver.x, self.odom_solver.y, self.odom_solver.theta

        od = self.odom_msg
        od.header.stamp = now.to_msg()
        od.pose.pose.position.x    = float(x)
        od.pose.pose.position.y    = float(y)
        od.pose.pose.position.z    = 0.0
        od.pose.pose.orientation   = quat_from_yaw(yaw)
        od.twist.twist.linear.x    = float(self.odom_solver.v)
        od.twist.twist.linear.y    = 0.0
        od.twist.twist.angular.z   = float(self.odom_solver.w)

        self.pub.publish(od)

        if self.br is not None:
            tfm = TransformStamped()
            tfm.header.stamp       = now.to_msg()
            tfm.header.frame_id    = self.odom_frame
            tfm.child_frame_id     = self.base_frame
            tfm.transform.translation.x = float(x)
            tfm.transform.translation.y = float(y)
            tfm.transform.translation.z = 0.0
            tfm.transform.rotation  = quat_from_yaw(yaw)
            self.br.sendTransform(tfm)

    # ─────────────────────────────────────────────────────────────────
    # Drive status (1Hz)
    # ─────────────────────────────────────────────────────────────────

    def publish_drive_status(self):
        def axis_payload(ax_id: int):
            hb = self.canodrv.read_heartbeat_cached(ax_id, max_age=1.0)
            if hb is None:
                return {
                    "connected":       False,
                    "axis_state":      None,
                    "axis_state_name": None,
                    "closed_loop":     False,
                    "axis_error":      None,
                    "has_error":       False,
                    "flags":           {},
                }

            state_raw  = hb.get("Axis_State")
            state      = to_int(state_raw, 0)
            axis_error = to_int(hb.get("Axis_Error"), 0)

            flags = {
                "motor":      bool(to_int(hb.get("Motor_Error_Flag"),      0)),
                "encoder":    bool(to_int(hb.get("Encoder_Error_Flag"),    0)),
                "controller": bool(to_int(hb.get("Controller_Error_Flag"), 0)),
            }

            has_error   = (axis_error != 0) or any(flags.values())
            closed_loop = (state == 8)

            return {
                "connected":       True,
                "axis_state":      state,
                "axis_state_name": str(state_raw) if state_raw is not None else None,
                "closed_loop":     closed_loop,
                "axis_error":      axis_error,
                "has_error":       has_error,
                "flags":           flags,
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

    # ─────────────────────────────────────────────────────────────────
    # Shutdown
    # ─────────────────────────────────────────────────────────────────

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