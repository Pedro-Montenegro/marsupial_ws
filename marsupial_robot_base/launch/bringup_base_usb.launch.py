from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")

    default_params = os.path.join(
        get_package_share_directory("marsupial_robot_base"),
        "config",
        "robot_default.yaml"
    )

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("params_file", default_value=default_params),

        Node(
            package="marsupial_robot_base",
            executable="odom_node_usb",
            name="odom_node_usb",
            namespace=namespace,
            output="screen",
            parameters=[params_file],
        ),
    ])