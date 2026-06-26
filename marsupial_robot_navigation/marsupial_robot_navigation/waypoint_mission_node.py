#!/usr/bin/env python3
import math
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


@dataclass
class Waypoint:
    x: float
    y: float
    qz: float
    qw: float


def quat_to_yaw(z: float, w: float) -> float:
    return 2.0 * math.atan2(z, w)


def norm_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class WaypointMissionNode(Node):
    def __init__(self):
        super().__init__("waypoint_mission_node")

        # Tolerâncias da missão para dar fluidez
        self.declare_parameter("reach_xy_tol", 0.25)
        self.declare_parameter("reach_yaw_tol", 0.3)
        self.declare_parameter("auto_start", True)
        self.declare_parameter("frame_id", "map")

        self.reach_xy_tol = float(self.get_parameter("reach_xy_tol").value)
        self.reach_yaw_tol = float(self.get_parameter("reach_yaw_tol").value)
        self.auto_start = bool(self.get_parameter("auto_start").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_cb,
            10
        )

        self.timer = self.create_timer(0.2, self.control_loop)

        self.current_pose: Optional[PoseWithCovarianceStamped] = None
        self.goal_handle = None
        self.result_future = None

        self.started = False
        self.current_idx = 0
        self.waiting_nav_response = False
        self.canceling_for_next = False

        self.waypoints: List[Waypoint] = [
            Waypoint(1.7422562074402193, 10.475569754923015, -0.03134478568149905, 0.9995086314837811),
            Waypoint(8.02651378922208, 10.362498434021024, 0.01485929762255683, 0.9998895945423996),
            Waypoint(12.992430972677536, 10.739525260910856, 0.034181771062352685, 0.9994156325208451),
            Waypoint(17.020113254789287, 11.047940667857302, 0.0866537751489349, 0.9962384871367086),
            Waypoint(21.047828224143444, 11.73074488066011, 0.10418254350506446, 0.994558192178223),
            Waypoint(26.772914153665354, 12.744045323459092, 0.105133281261082, 0.9944581404821815),
            Waypoint(27.20767715521237, 11.75509563158547, -0.6357133187312481, 0.7719252401545778),
            Waypoint(26.31820714600221, 7.391240267667984, -0.9915279544889251, 0.12989347738438622),
        ]

        self.get_logger().info(
            f"Missão carregada com {len(self.waypoints)} waypoints | "
            f"tol_xy={self.reach_xy_tol:.2f} m, tol_yaw={self.reach_yaw_tol:.2f} rad"
        )

        if self.auto_start:
            self.started = True

    def amcl_cb(self, msg: PoseWithCovarianceStamped):
        self.current_pose = msg

    def control_loop(self):
        if not self.started:
            return

        if self.current_pose is None:
            return

        if self.current_idx >= len(self.waypoints):
            return

        # Espera o servidor Nav2
        if not self.nav_client.server_is_ready():
            self.get_logger().warn("Aguardando action server navigate_to_pose...")
            return

        # Se ainda não existe goal ativo, envia
        if self.goal_handle is None and not self.waiting_nav_response:
            self.send_current_goal()
            return

        # Se existe um waypoint atual, verifica atingimento com tolerância própria
        wp = self.waypoints[self.current_idx]
        px = self.current_pose.pose.pose.position.x
        py = self.current_pose.pose.pose.position.y
        qz = self.current_pose.pose.pose.orientation.z
        qw = self.current_pose.pose.pose.orientation.w

        dx = wp.x - px
        dy = wp.y - py
        dist = math.hypot(dx, dy)

        yaw_robot = quat_to_yaw(qz, qw)
        yaw_goal = quat_to_yaw(wp.qz, wp.qw)
        yaw_err = norm_angle(yaw_goal - yaw_robot)

        if dist <= self.reach_xy_tol and abs(yaw_err) <= self.reach_yaw_tol:
            self.get_logger().info(
                f"Waypoint {self.current_idx + 1}/{len(self.waypoints)} atingido "
                f"pela tolerância da missão: dist={dist:.3f}, yaw_err={yaw_err:.3f}"
            )
            self.advance_to_next_waypoint()
            return

        # Também observa resultado normal do Nav2
        if self.result_future is not None and self.result_future.done():
            result = self.result_future.result()
            status = result.status

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(
                    f"Waypoint {self.current_idx + 1}/{len(self.waypoints)} concluído pelo Nav2."
                )
                self.goal_handle = None
                self.result_future = None
                self.current_idx += 1

                if self.current_idx < len(self.waypoints):
                    self.send_current_goal()
                else:
                    self.get_logger().info("Missão finalizada.")
            else:
                self.get_logger().warn(
                    f"Goal terminou com status {status}. Tentando reenviar waypoint atual."
                )
                self.goal_handle = None
                self.result_future = None

    def send_current_goal(self):
        if self.current_idx >= len(self.waypoints):
            self.get_logger().info("Todos os waypoints já foram enviados.")
            return

        wp = self.waypoints[self.current_idx]

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self.frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = wp.x
        goal_msg.pose.pose.position.y = wp.y
        goal_msg.pose.pose.position.z = 0.0

        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = wp.qz
        goal_msg.pose.pose.orientation.w = wp.qw

        self.get_logger().info(
            f"Enviando waypoint {self.current_idx + 1}/{len(self.waypoints)}: "
            f"x={wp.x:.3f}, y={wp.y:.3f}"
        )

        self.waiting_nav_response = True
        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        self.waiting_nav_response = False
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.get_logger().warn("Goal rejeitado pelo Nav2.")
            self.goal_handle = None
            return

        self.get_logger().info("Goal aceito pelo Nav2.")
        self.result_future = self.goal_handle.get_result_async()

    def advance_to_next_waypoint(self):
        # Cancela goal atual para já seguir fluido para o próximo
        if self.goal_handle is not None:
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_cb)
            self.canceling_for_next = True
        else:
            self.current_idx += 1
            if self.current_idx < len(self.waypoints):
                self.send_current_goal()
            else:
                self.get_logger().info("Missão finalizada.")

    def cancel_done_cb(self, future):
        self.canceling_for_next = False
        self.goal_handle = None
        self.result_future = None
        self.current_idx += 1

        if self.current_idx < len(self.waypoints):
            self.send_current_goal()
        else:
            self.get_logger().info("Missão finalizada.")


def main(args=None):
    rclpy.init(args=args)
    node = WaypointMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()