from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("marsupial_robot_base")
    gamepad_config = os.path.join(pkg_share, "config", "gamepad_teleop.yaml")
    base_launch = os.path.join(pkg_share, "launch", "bringup_base_4wd.launch.py")

    include_base = LaunchConfiguration("include_base")
    device_id = LaunchConfiguration("device_id")

    return LaunchDescription(
        [
            DeclareLaunchArgument("include_base", default_value="true"),
            DeclareLaunchArgument("device_id", default_value="0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(base_launch),
                condition=IfCondition(include_base),
            ),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                parameters=[gamepad_config, {"device_id": device_id}],
            ),
            Node(
                package="marsupial_robot_base",
                executable="gamepad_teleop",
                name="gamepad_teleop",
                output="screen",
                parameters=[gamepad_config],
            ),
        ]
    )
