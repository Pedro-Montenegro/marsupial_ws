from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("marsupial_robot_base")

    default_params_file = os.path.join(pkg_share, "config", "robot_4wd.yaml")

    params_file = LaunchConfiguration("params_file")
    namespace   = LaunchConfiguration("namespace")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="",
        ),
        Node(
            package="marsupial_robot_base",
            executable="odom_node_4wd",
            name="odom_node_4wd",
            namespace=namespace,
            output="screen",
            parameters=[params_file],
        ),
    ])
