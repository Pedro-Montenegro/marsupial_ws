#!/usr/bin/env python3
"""
odom_node_4wd.py

Controle skid-steer 4WD via dois ODrives USB + odometria.

Topologia:
  ODrive esquerdo → axis[left_front_axis]=FL, axis[left_rear_axis]=RL (mesma vel)
  ODrive direito  → axis[right_front_axis]=FR, axis[right_rear_axis]=RR (mesma vel)

Odometria:
  avg_left  = (pos_FL + pos_RL) / 2
  avg_right = (pos_FR + pos_RR) / 2
  → DiffOdom.update_from_turns(avg_left, avg_right)   ← inalterado

Interface ROS 2:
  Sub: /cmd_vel          (geometry_msgs/Twist)
  Pub: /wheel/odom       (nav_msgs/Odometry)
  Pub: /ui/drive_status  (std_msgs/String — JSON com estado dos 4 eixos)
"""

import json
import math
import time
from typing import Callable, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

import odrive
from odrive.enums import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
    CONTROL_MODE_VELOCITY_CONTROL,
    INPUT_MODE_VEL_RAMP,
)

from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from .diff_odom import DiffOdom


def quat_from_yaw(yaw: float) -> Quaternion:
    half = yaw * 0.5
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


class ODrive4WD:
    """
    Gerencia dois ODrives USB para robô skid-steer 4WD.

    Rodas do mesmo lado sempre recebem a mesma velocidade.
    Odometria via média das posições dos encoders de cada lado.

    Identificação dos ODrives:
      - Por número de série (recomendado): passe left_serial / right_serial
        como string hex sem "0x" (ex: "366f54483536"), obtida com:
          odrivetool → hex(odrv0.serial_number)
      - String vazia: conecta ao primeiro ODrive encontrado (modo teste,
        não determinístico com dois ODrives conectados simultaneamente).
    """

    def __init__(
        self,
        left_serial: str = "",
        right_serial: str = "",
        left_front_axis: int = 0,
        left_rear_axis: int = 1,
        right_front_axis: int = 0,
        right_rear_axis: int = 1,
        timeout: float = 30.0,
        watchdog_timeout: float = 3.0,
        watchdog_enabled: bool = True,
        right_disabled: bool = False,
        right_front_disabled: bool = False,
        log: Optional[Callable[[str], None]] = None,
    ):
        self._log = log or print
        self.lfa = left_front_axis
        self.lra = left_rear_axis
        self.rfa = right_front_axis
        self.rra = right_rear_axis
        self.right_disabled = right_disabled
        self.right_front_disabled = right_front_disabled
        self.watchdog_timeout = watchdog_timeout
        self.watchdog_enabled = watchdog_enabled
        self.latest_positions = {
            "FL": None,
            "RL": None,
            "FR": None,
            "RR": None,
        }
        self.latest_commands = {
            "FL": 0.0,
            "RL": 0.0,
            "FR": 0.0,
            "RR": 0.0,
        }
        self.latest_sample_monotonic_ns = 0

        self._log("Conectando ODrive esquerdo via USB...")
        if not left_serial:
            self._log("  AVISO: left_odrive_serial vazio — conectando ao 1º ODrive encontrado.")
        kw = {"serial_number": left_serial} if left_serial else {}
        self.odrv_left = odrive.find_any(timeout=timeout, **kw)
        if self.odrv_left is None:
            expected = f" 0x{left_serial.lower()}" if left_serial else ""
            raise RuntimeError(
                f"ODrive esquerdo{expected} não encontrado via USB "
                f"após {timeout:.1f} s. Feche o odrivetool e confira cabo, "
                "alimentação, permissões USB e número de série."
            )
        self.left_serial = f"{int(self.odrv_left.serial_number):X}"
        self._log(f"  ODrive esquerdo: 0x{self.left_serial.lower()}")

        if right_disabled:
            self._log("  ODrive direito DESABILITADO (right_disabled=true) — apenas lado esquerdo ativo.")
            self.odrv_right = self.odrv_left
        else:
            self._log("Conectando ODrive direito via USB...")
            if not right_serial:
                self._log("  AVISO: right_odrive_serial vazio — conectando ao 1º ODrive encontrado.")
            kw = {"serial_number": right_serial} if right_serial else {}
            self.odrv_right = odrive.find_any(timeout=timeout, **kw)
            if self.odrv_right is None:
                expected = f" 0x{right_serial.lower()}" if right_serial else ""
                raise RuntimeError(
                    f"ODrive direito{expected} não encontrado via USB "
                    f"após {timeout:.1f} s. Feche o odrivetool e confira cabo, "
                    "alimentação, permissões USB e número de série."
                )
            self.right_serial = f"{int(self.odrv_right.serial_number):X}"
            self._log(f"  ODrive direito: 0x{self.right_serial.lower()}")

        if right_disabled:
            self.right_serial = None

        if (not right_disabled
                and self.odrv_left.serial_number == self.odrv_right.serial_number):
            raise RuntimeError(
                "Os lados esquerdo e direito apontam para a mesma ODrive. "
                "Confira left_odrive_serial e right_odrive_serial."
            )

        self._init_axes()

    # ── Utilitários internos ─────────────────────────────────────────

    def _ax(self, odrv, ax_id: int):
        return odrv.axis0 if ax_id == 0 else odrv.axis1

    def _init_one(self, odrv, ax_id: int, label: str):
        ax = self._ax(odrv, ax_id)
        try:
            ax.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            ax.controller.config.input_mode = INPUT_MODE_VEL_RAMP
            ax.controller.input_vel = 0.0
            ax.config.watchdog_timeout = self.watchdog_timeout
            ax.config.enable_watchdog = self.watchdog_enabled
            if self.watchdog_enabled:
                ax.watchdog_feed()
        except Exception as e:
            raise RuntimeError(f"[{label}] falha ao configurar modo: {e}") from e
        try:
            ax.clear_errors()
        except Exception:
            pass
        time.sleep(0.05)
        try:
            ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        except Exception as e:
            raise RuntimeError(f"[{label}] falha ao solicitar CLOSED_LOOP_CONTROL: {e}") from e

        deadline = time.time() + 2.0
        while time.time() < deadline:
            if int(ax.current_state) == AXIS_STATE_CLOSED_LOOP_CONTROL:
                return
            if int(ax.error) != 0:
                break
            time.sleep(0.05)

        raise RuntimeError(
            f"[{label}] não entrou em CLOSED_LOOP_CONTROL "
            f"(state={int(ax.current_state)}, axis_error={hex(int(ax.error))}, "
            f"motor_error={hex(int(ax.motor.error))}, "
            f"encoder_error={hex(int(ax.encoder.error))}, "
            f"controller_error={hex(int(ax.controller.error))})."
        )

    def _init_axes(self):
        self._log("Inicializando eixos em CLOSED_LOOP_CONTROL...")
        if self.watchdog_enabled:
            self._log(f"Watchdog dos ODrives habilitado ({self.watchdog_timeout:.1f} s).")
        else:
            self._log("AVISO: watchdog dos ODrives desabilitado por parâmetro.")
        self._init_one(self.odrv_left, self.lfa, "FL")
        self._init_one(self.odrv_left, self.lra, "RL")
        if not self.right_disabled:
            time.sleep(0.1)
            if self.right_front_disabled:
                self._log("  FR DESABILITADA (right_front_disabled=true) — roda solta, sem closed loop.")
            else:
                self._init_one(self.odrv_right, self.rfa, "FR")
            self._init_one(self.odrv_right, self.rra, "RR")
        self._log("ODrive4WD pronto.")

    # ── Comandos de velocidade ────────────────────────────────────────

    def set_left_vel(self, vel_rev_s: float):
        """Envia mesma velocidade para FL e RL."""
        self.latest_commands["FL"] = float(vel_rev_s)
        self.latest_commands["RL"] = float(vel_rev_s)
        try:
            self._ax(self.odrv_left, self.lfa).controller.input_vel = vel_rev_s
            self._ax(self.odrv_left, self.lra).controller.input_vel = vel_rev_s
            if self.watchdog_enabled:
                self._ax(self.odrv_left, self.lfa).watchdog_feed()
                self._ax(self.odrv_left, self.lra).watchdog_feed()
        except Exception as e:
            self._log(f"[4WD] falha vel esq: {e}")

    def set_right_vel(self, vel_rev_s: float):
        """Envia velocidade para RR e, se não desabilitada, para FR (no-op total se right_disabled)."""
        if self.right_disabled:
            return
        self.latest_commands["RR"] = float(vel_rev_s)
        try:
            self._ax(self.odrv_right, self.rra).controller.input_vel = vel_rev_s
            if self.watchdog_enabled:
                self._ax(self.odrv_right, self.rra).watchdog_feed()
        except Exception as e:
            self._log(f"[4WD] falha vel dir (RR): {e}")

        if self.right_front_disabled:
            self.latest_commands["FR"] = 0.0
            return
        self.latest_commands["FR"] = float(vel_rev_s)
        try:
            self._ax(self.odrv_right, self.rfa).controller.input_vel = vel_rev_s
            if self.watchdog_enabled:
                self._ax(self.odrv_right, self.rfa).watchdog_feed()
        except Exception as e:
            self._log(f"[4WD] falha vel dir (FR): {e}")

    # ── Odometria ────────────────────────────────────────────────────

    def read_positions(self) -> Tuple[Optional[float], Optional[float]]:
        """
        Retorna (avg_left_turns, avg_right_turns) em rotações (raw, sem sinal).
        Retorna (None, None) se qualquer leitura falhar.
        """
        try:
            fl = float(self._ax(self.odrv_left, self.lfa).encoder.pos_estimate)
            rl = float(self._ax(self.odrv_left, self.lra).encoder.pos_estimate)
            self.latest_positions["FL"] = fl
            self.latest_positions["RL"] = rl
            avg_l: Optional[float] = (fl + rl) * 0.5
        except Exception as e:
            self._log(f"[4WD] encoder esq: {e}")
            self.latest_positions["FL"] = None
            self.latest_positions["RL"] = None
            avg_l = None

        if self.right_disabled:
            avg_r = avg_l  # espelha esquerda para odometria funcionar em linha reta
        elif self.right_front_disabled:
            try:
                rr = float(self._ax(self.odrv_right, self.rra).encoder.pos_estimate)
                self.latest_positions["FR"] = None
                self.latest_positions["RR"] = rr
                avg_r: Optional[float] = rr
            except Exception as e:
                self._log(f"[4WD] encoder dir (RR): {e}")
                self.latest_positions["RR"] = None
                avg_r = None
        else:
            try:
                fr = float(self._ax(self.odrv_right, self.rfa).encoder.pos_estimate)
                rr = float(self._ax(self.odrv_right, self.rra).encoder.pos_estimate)
                self.latest_positions["FR"] = fr
                self.latest_positions["RR"] = rr
                avg_r: Optional[float] = (fr + rr) * 0.5
            except Exception as e:
                self._log(f"[4WD] encoder dir: {e}")
                self.latest_positions["FR"] = None
                self.latest_positions["RR"] = None
                avg_r = None

        self.latest_sample_monotonic_ns = time.monotonic_ns()
        return avg_l, avg_r

    def get_telemetry(self) -> dict:
        return {
            "sample_monotonic_ns": self.latest_sample_monotonic_ns,
            "positions_turns": dict(self.latest_positions),
            "commands_turns_s": dict(self.latest_commands),
            "left_serial": self.left_serial,
            "right_serial": self.right_serial,
            "watchdog_enabled": self.watchdog_enabled,
        }

    # ── Status ───────────────────────────────────────────────────────

    def _ax_status(self, odrv, ax_id: int, label: str) -> dict:
        try:
            ax = self._ax(odrv, ax_id)
            state = int(ax.current_state)
            err   = int(ax.error)
            return {
                "label":       label,
                "connected":   True,
                "axis_state":  state,
                "closed_loop": state == AXIS_STATE_CLOSED_LOOP_CONTROL,
                "axis_error":  err,
                "has_error":   err != 0,
            }
        except Exception:
            return {"label": label, "connected": False}

    def get_status(self) -> dict:
        return {
            "FL": self._ax_status(self.odrv_left,  self.lfa, "FL"),
            "RL": self._ax_status(self.odrv_left,  self.lra, "RL"),
            "FR": self._ax_status(self.odrv_right, self.rfa, "FR"),
            "RR": self._ax_status(self.odrv_right, self.rra, "RR"),
        }

    # ── Shutdown ─────────────────────────────────────────────────────

    def safe_idle(self):
        sides = [(self.odrv_left, (self.lfa, self.lra))]
        if not self.right_disabled:
            sides.append((self.odrv_right, (self.rfa, self.rra)))
        for odrv, axes in sides:
            for ax_id in axes:
                try:
                    self._ax(odrv, ax_id).controller.input_vel = 0.0
                except Exception:
                    pass
        time.sleep(0.05)
        for odrv, axes in sides:
            for ax_id in axes:
                try:
                    self._ax(odrv, ax_id).requested_state = AXIS_STATE_IDLE
                except Exception:
                    pass
                try:
                    self._ax(odrv, ax_id).config.enable_watchdog = False
                except Exception:
                    pass


# ─────────────────────────────────────────────────────────────────────────────


class WheelOdom4WDNode(Node):

    def __init__(self):
        super().__init__("odom_node_4wd")

        # ── Parâmetros ────────────────────────────────────────────────
        self.declare_parameter("wheel_radius",        0.082)
        self.declare_parameter("wheel_base",          0.31)
        self.declare_parameter("odom_frame",          "odom")
        self.declare_parameter("base_frame",          "base_link")
        self.declare_parameter("odom_topic",          "wheel/odom")
        self.declare_parameter("cmd_vel_topic",       "cmd_vel")
        self.declare_parameter("drive_status_topic",  "ui/drive_status")
        self.declare_parameter("telemetry_topic",     "drive/telemetry")
        self.declare_parameter("hz",                  30.0)
        self.declare_parameter("publish_tf",          False)
        self.declare_parameter("cmd_timeout",         0.5)
        self.declare_parameter("left_odrive_serial",  "")
        self.declare_parameter("right_odrive_serial", "")
        self.declare_parameter("usb_timeout",         30.0)
        self.declare_parameter("odrive_watchdog_timeout", 3.0)
        self.declare_parameter("odrive_watchdog_enabled", True)
        self.declare_parameter("left_front_axis",     0)
        self.declare_parameter("left_rear_axis",      1)
        self.declare_parameter("right_front_axis",    0)
        self.declare_parameter("right_rear_axis",     1)
        self.declare_parameter("sign_left",           -1.0)
        self.declare_parameter("sign_right",           1.0)
        self.declare_parameter("right_disabled",      False)
        self.declare_parameter("right_front_disabled", False)

        wheel_radius       = float(self.get_parameter("wheel_radius").value)
        wheel_base         = float(self.get_parameter("wheel_base").value)
        self.odom_frame    = self.get_parameter("odom_frame").value
        self.base_frame    = self.get_parameter("base_frame").value
        odom_topic         = self.get_parameter("odom_topic").value
        cmd_vel_topic      = self.get_parameter("cmd_vel_topic").value
        drive_status_topic = self.get_parameter("drive_status_topic").value
        telemetry_topic    = self.get_parameter("telemetry_topic").value
        hz                 = float(self.get_parameter("hz").value)
        self.publish_tf    = bool(self.get_parameter("publish_tf").value)
        self.cmd_timeout   = float(self.get_parameter("cmd_timeout").value)
        left_serial        = self.get_parameter("left_odrive_serial").value
        right_serial       = self.get_parameter("right_odrive_serial").value
        usb_timeout        = float(self.get_parameter("usb_timeout").value)
        watchdog_timeout   = float(
            self.get_parameter("odrive_watchdog_timeout").value
        )
        watchdog_enabled   = bool(
            self.get_parameter("odrive_watchdog_enabled").value
        )
        lfa                = int(self.get_parameter("left_front_axis").value)
        lra                = int(self.get_parameter("left_rear_axis").value)
        rfa                = int(self.get_parameter("right_front_axis").value)
        rra                = int(self.get_parameter("right_rear_axis").value)
        self.sign_l        = float(self.get_parameter("sign_left").value)
        self.sign_r        = float(self.get_parameter("sign_right").value)
        right_disabled     = bool(self.get_parameter("right_disabled").value)
        right_front_disabled = bool(self.get_parameter("right_front_disabled").value)

        if hz <= 0.0:
            raise ValueError("hz deve ser maior que zero.")
        if self.cmd_timeout <= 0.0:
            raise ValueError("cmd_timeout deve ser maior que zero.")
        if watchdog_enabled and watchdog_timeout <= self.cmd_timeout:
            raise ValueError(
                "odrive_watchdog_timeout deve ser maior que cmd_timeout para "
                "a parada por perda de cmd_vel ocorrer antes do watchdog de hardware."
            )

        # ── Publishers ────────────────────────────────────────────────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub       = self.create_publisher(Odometry, odom_topic, qos)
        self.pub_drive = self.create_publisher(String, drive_status_topic, 10)
        self.pub_telemetry = self.create_publisher(String, telemetry_topic, qos)
        self.br        = TransformBroadcaster(self) if self.publish_tf else None

        # ── ODrives 4WD ───────────────────────────────────────────────
        self.get_logger().info("Iniciando ODrive4WD via USB...")
        try:
            self.drv = ODrive4WD(
                left_serial=left_serial,
                right_serial=right_serial,
                left_front_axis=lfa,
                left_rear_axis=lra,
                right_front_axis=rfa,
                right_rear_axis=rra,
                timeout=usb_timeout,
                watchdog_timeout=watchdog_timeout,
                watchdog_enabled=watchdog_enabled,
                right_disabled=right_disabled,
                right_front_disabled=right_front_disabled,
                log=lambda s: self.get_logger().info(s),
            )
        except Exception as e:
            self.get_logger().error(f"Falha ao iniciar ODrive4WD: {e}")
            raise

        # ── Odometria ─────────────────────────────────────────────────
        circ = 2.0 * math.pi * wheel_radius
        self.odom_solver = DiffOdom(
            L=wheel_base,
            CIRC=circ,
            sign_l=self.sign_l,
            sign_r=self.sign_r,
        )
        self.odom_solver.reset()

        # ── Estado cmd_vel ────────────────────────────────────────────
        self.last_cmd_monotonic   = time.monotonic()
        self._stopped_due_timeout = False
        self._last_sent           = (None, None)  # (wr, wl) enviados
        self._last_sent_t         = 0.0
        self._min_send_period     = 0.02           # máx 50 Hz
        self._eps                 = 1e-4

        # ── Subscribers / Timers ──────────────────────────────────────
        self.sub_cmd = self.create_subscription(Twist, cmd_vel_topic, self.cmd_cb, qos)

        period = 1.0 / max(1.0, hz)
        self.timer     = self.create_timer(period, self.loop_cb)
        self.timer_drv = self.create_timer(1.0, self.publish_drive_status)

        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = self.odom_frame
        self.odom_msg.child_frame_id  = self.base_frame

        self.get_logger().info("WheelOdom4WDNode pronto.")

    # ─────────────────────────────────────────────────────────────────
    # cmd_vel callback
    # ─────────────────────────────────────────────────────────────────

    def cmd_cb(self, msg: Twist):
        v = float(msg.linear.x)
        w = float(msg.angular.z)

        CIRC = self.odom_solver.CIRC
        L    = self.odom_solver.L

        wr = (v + 0.5 * L * w) / CIRC   # vel necessária roda direita (rev/s)
        wl = (v - 0.5 * L * w) / CIRC   # vel necessária roda esquerda (rev/s)

        now_s = time.time()
        last_wr, last_wl = self._last_sent
        if last_wr is not None:
            if (abs(wr - last_wr) < self._eps
                    and abs(wl - last_wl) < self._eps
                    and (now_s - self._last_sent_t) < self._min_send_period):
                self.last_cmd_monotonic = time.monotonic()
                return

        # sign_l / sign_r corrigem o sentido de montagem dos motores
        self.drv.set_right_vel(self.sign_r * wr)
        self.drv.set_left_vel(self.sign_l * wl)

        self._last_sent           = (wr, wl)
        self._last_sent_t         = now_s
        self.last_cmd_monotonic   = time.monotonic()
        self._stopped_due_timeout = False

    # ─────────────────────────────────────────────────────────────────
    # Loop principal
    # ─────────────────────────────────────────────────────────────────

    def loop_cb(self):
        now = self.get_clock().now()
        # Segurança não deve depender de /clock, que pode pausar ou retroceder.
        elapsed = time.monotonic() - self.last_cmd_monotonic

        if elapsed > self.cmd_timeout:
            # Keepalive contínuo: mantém watchdog de ambos os ODrives
            self.drv.set_right_vel(0.0)
            self.drv.set_left_vel(0.0)
            self._stopped_due_timeout = True
        else:
            self._stopped_due_timeout = False

        avg_l, avg_r = self.drv.read_positions()
        telemetry_msg = String()
        telemetry_msg.data = json.dumps(
            self.drv.get_telemetry(),
            separators=(",", ":"),
        )
        self.pub_telemetry.publish(telemetry_msg)

        if avg_l is not None and avg_r is not None:
            x, y, yaw = self.odom_solver.update_from_turns(
                pos_l_raw_turns=avg_l,
                pos_r_raw_turns=avg_r,
            )
        else:
            x, y, yaw = self.odom_solver.x, self.odom_solver.y, self.odom_solver.theta

        od = self.odom_msg
        od.header.stamp          = now.to_msg()
        od.pose.pose.position.x  = float(x)
        od.pose.pose.position.y  = float(y)
        od.pose.pose.position.z  = 0.0
        od.pose.pose.orientation = quat_from_yaw(yaw)
        od.twist.twist.linear.x  = float(self.odom_solver.v)
        od.twist.twist.linear.y  = 0.0
        od.twist.twist.angular.z = float(self.odom_solver.w)
        self.pub.publish(od)

        if self.br is not None:
            tfm = TransformStamped()
            tfm.header.stamp            = now.to_msg()
            tfm.header.frame_id         = self.odom_frame
            tfm.child_frame_id          = self.base_frame
            tfm.transform.translation.x = float(x)
            tfm.transform.translation.y = float(y)
            tfm.transform.translation.z = 0.0
            tfm.transform.rotation      = quat_from_yaw(yaw)
            self.br.sendTransform(tfm)

    # ─────────────────────────────────────────────────────────────────
    # Drive status (1 Hz)
    # ─────────────────────────────────────────────────────────────────

    def publish_drive_status(self):
        try:
            payload = {
                "connected": True,
                "backend":   "usb_4wd",
                "axes":      self.drv.get_status(),
            }
            msg      = String()
            msg.data = json.dumps(payload, ensure_ascii=False)
            self.pub_drive.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"publish_drive_status falhou: {e}")

    # ─────────────────────────────────────────────────────────────────
    # Shutdown
    # ─────────────────────────────────────────────────────────────────

    def destroy_node(self):
        try:
            self.drv.safe_idle()
        except Exception:
            self.get_logger().warn("Falha ao enviar safe_idle.")
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdom4WDNode()
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
