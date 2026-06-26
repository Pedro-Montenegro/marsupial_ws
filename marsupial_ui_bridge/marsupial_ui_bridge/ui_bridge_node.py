import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from nav_msgs.msg import Path
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

qos = QoSProfile(depth=5)
qos.reliability = ReliabilityPolicy.BEST_EFFORT
qos.durability = DurabilityPolicy.VOLATILE


class UiBridge(Node):

    def __init__(self):
        super().__init__('marsupial_ui_bridge')

        # subscriber vindo da UI (React → rosbridge)
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/ui/goal',
            self.goal_callback,
            10
        )

        self.plan_pub = self.create_publisher(Path, "/ui/plan", qos)

        self.plan_sub = self.create_subscription(
            Path,
            "/plan",          # <-- se o teu plan for outro nome, troca aqui
            self.plan_callback,
            qos
        )

        # action client Nav2
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.get_logger().info("UI Bridge pronto — aguardando /ui/goal")
    
    def plan_callback(self, msg: Path):
        self.plan_pub.publish(msg)

    def goal_callback(self, msg: PoseStamped):
        self.get_logger().info(
            f"Goal recebido: x={msg.pose.position.x:.2f} y={msg.pose.position.y:.2f}"
        )

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 action server não disponível")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejeitado pelo Nav2")
            return

        self.get_logger().info("Goal aceito pelo Nav2")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Navegação finalizada")


def main(args=None):
    rclpy.init(args=args)
    node = UiBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()