#!/usr/bin/env python3

import math
import json
import time
from typing import Optional, Tuple

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

import odrive
from odrive.enums import *

from .diff_odom import DiffOdom


def quat_from_yaw(yaw: float) -> Quaternion:
    half = yaw * 0.5
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


class ODriveUSB:
    """
    Wrapper simples para ODrive via USB.
    """

    def __init__(self, right_axis: int = 0, left_axis: int = 1, timeout: float = 10.0):
        print("[ODriveUSB] Procurando ODrive via USB...")
        self.odrv = odrive.find_any(timeout=timeout)
        print("[ODriveUSB] ODrive conectado:", self.odrv)

        self.right_axis = int(right_axis)
        self.left_axis = int(left_axis)

        for ax in (self.right_axis, self.left_axis):
            axis = self._axis(ax)
            axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            axis.controller.config.input_mode = INPUT_MODE_VEL_RAMP
            axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    def _axis(self, axis_id: int):
        if axis_id == 0:
            return self.odrv.axis0
        elif axis_id == 1:
            return self.odrv.axis1
        raise ValueError(f"Eixo inválido para ODriveUSB: {axis_id}")

    def set_input_vel(self, axis_id: int, vel_rev_s: float, torque_ff: float = 0.0):
        axis = self._axis(axis_id)
        axis.controller.input_vel = float(vel_rev_s)
        try:
            axis.controller.input_torque = float(torque_ff)
        except Exception:
            pass

    def read_encoder(self, axis_id: int) -> Tuple[Optional[float], Optional[float]]:
        axis = self._axis(axis_id)
        pos = float(axis.encoder.pos_estimate)
        vel = float(axis.encoder.vel_estimate)
        return pos, vel

    def get_axis_status(self, axis_id: int):
        axis = self._axis(axis_id)
        status = {
            "connected": True,
            "axis_id": axis_id,
            "axis_state": None,
            "axis_state_name": None,
            "closed_loop": False,
            "axis_error": None,
            "has_error": False,
            "motor_error": None,
            "encoder_error": None,
            "controller_error": None,
        }

        try:
            state = int(axis.current_state)
            axis_error = int(axis.error)
            motor_error = int(axis.motor.error)
            encoder_error = int(axis.encoder.error)
            controller_error = int(axis.controller.error)

            status["axis_state"] = state
            status["axis_state_name"] = str(state)
            status["closed_loop"] = (state == AXIS_STATE_CLOSED_LOOP_CONTROL)
            status["axis_error"] = axis_error
            status["motor_error"] = motor_error
            status["encoder_error"] = encoder_error
            status["controller_error"] = controller_error
            status["has_error"] = any([
                axis_error != 0,
                motor_error != 0,
                encoder_error != 0,
                controller_error != 0,
            ])
        except Exception:
            status["connected"] = False

        return status

    def safe_idle(self):
        for ax in (self.right_axis, self.left_axis):
            axis = self._axis(ax)
            try:
                axis.controller.input_vel = 0.0
            except Exception:
                pass

        time.sleep(0.05)

        for ax in (self.right_axis, self.left_axis):
            axis = self._axis(ax)
            try:
                axis.requested_state = AXIS_STATE_IDLE
            except Exception:
                pass


class WheelOdomUSBNode(Node):
    """
    Nó ROS2 para controle diferencial via ODrive USB e publicação de odometria.
    """

    def __init__(self):
        super().__init__("odom_node_usb")

        # ---- Parâmetros ----
        self.declare_parameter("wheel_radius", 0.082)
        self.declare_parameter("wheel_base", 0.31)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        self.declare_parameter("odom_topic", "wheel/odom")
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("drive_status_topic", "ui/drive_status")

        self.declare_parameter("hz", 50.0)
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("cmd_timeout", 0.5)

        self.declare_parameter("right_id", 0)
        self.declare_parameter("left_id", 1)
        self.declare_parameter("usb_timeout", 10.0)

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

        self.right_axis = int(self.get_parameter("right_id").value)
        self.left_axis = int(self.get_parameter("left_id").value)
        usb_timeout = float(self.get_parameter("usb_timeout").value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pub = self.create_publisher(Odometry, odom_topic, qos)
        self.pub_drive = self.create_publisher(String, drive_status_topic, 10)
        self.br = TransformBroadcaster(self) if self.publish_tf else None

        # ---- ODrive via USB ----
        self.get_logger().info("Conectando ao ODrive via USB...")
        try:
            self.odrv = ODriveUSB(
                right_axis=self.right_axis,
                left_axis=self.left_axis,
                timeout=usb_timeout,
            )
        except Exception as e:
            self.get_logger().error(f"Falha ao conectar ODrive via USB: {e}")
            raise

        # ---- Odometria ----
        circ = 2.0 * math.pi * wheel_radius
        self.odom_solver = DiffOdom(
            L=wheel_base,
            CIRC=circ,
            sign_l=-1.0,
            sign_r=+1.0,
        )
        self.odom_solver.reset()

        # ---- cmd_vel ----
        self.last_cmd_stamp = self.get_clock().now()
        self.sub_cmd = self.create_subscription(
            Twist, cmd_vel_topic, self.cmd_cb, qos
        )

        self._stopped_due_timeout = False
        self._last_sent = (None, None)
        self._last_sent_t = 0.0
        self._min_send_period = 0.02   # 50 Hz máx
        self._eps = 1e-4

        # ---- Timer Principal ----
        period = 1.0 / max(1.0, hz)
        self.timer = self.create_timer(period, self.loop_cb)
        self.create_timer(1.0, self.publish_drive_status)

        self.get_logger().info("wheel_odom_node_usb (ROS2) pronto.")

        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = self.odom_frame
        self.odom_msg.child_frame_id = self.base_frame

    def cmd_cb(self, msg: Twist):
        v = float(msg.linear.x)
        w = float(msg.angular.z)

        circ = self.odom_solver.CIRC
        L = self.odom_solver.L

        wr = (v + 0.5 * L * w) / circ
        wl = (v - 0.5 * L * w) / circ

        now_s = time.time()
        last_wr, last_wl = self._last_sent

        if last_wr is not None:
            if (
                abs(wr - last_wr) < self._eps
                and abs(wl - last_wl) < self._eps
                and (now_s - self._last_sent_t) < self._min_send_period
            ):
                self.last_cmd_stamp = self.get_clock().now()
                return

        try:
            self.odrv.set_input_vel(self.right_axis, +wr, 0.0)
            self.odrv.set_input_vel(self.left_axis, -wl, 0.0)
        except Exception as e:
            self.get_logger().warn(f"Falha ao enviar velocidade USB: {e}")
            return

        self._last_sent = (wr, wl)
        self._last_sent_t = now_s
        self.last_cmd_stamp = self.get_clock().now()
        self._stopped_due_timeout = False

    def loop_cb(self):
        now = self.get_clock().now()

        if (now - self.last_cmd_stamp).nanoseconds * 1e-9 > self.cmd_timeout:
            if not self._stopped_due_timeout:
                try:
                    self.odrv.set_input_vel(self.right_axis, 0.0, 0.0)
                    self.odrv.set_input_vel(self.left_axis, 0.0, 0.0)
                except Exception as e:
                    self.get_logger().warn(f"Falha no watchdog USB: {e}")
                self._stopped_due_timeout = True
        else:
            self._stopped_due_timeout = False

        try:
            l_turns, _ = self.odrv.read_encoder(self.left_axis)
            r_turns, _ = self.odrv.read_encoder(self.right_axis)
        except Exception as e:
            self.get_logger().warn(f"Falha ao ler encoders USB: {e}")
            l_turns = r_turns = None

        if (l_turns is not None) and (r_turns is not None):
            x, y, yaw = self.odom_solver.update_from_turns(
                pos_l_raw_turns=l_turns,
                pos_r_raw_turns=r_turns,
            )
        else:
            x, y, yaw = (
                self.odom_solver.x,
                self.odom_solver.y,
                self.odom_solver.theta,
            )

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
        try:
            payload = {
                "connected": True,
                "left_axis": self.odrv.get_axis_status(self.left_axis),
                "right_axis": self.odrv.get_axis_status(self.right_axis),
                "backend": "usb",
            }
            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False)
            self.pub_drive.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"publish_drive_status USB falhou: {e}")

    def destroy_node(self):
        try:
            self.odrv.safe_idle()
        except Exception:
            self.get_logger().warn("Falha ao enviar safe_idle para ODrive USB.")
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdomUSBNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()