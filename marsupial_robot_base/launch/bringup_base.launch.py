from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("marsupial_robot_base")

    default_params_file = os.path.join(pkg_share, "config", "robot_default.yaml")
    default_dbc_file = os.path.join(pkg_share, "odrive-cansimple.dbc")

    params_file = LaunchConfiguration("params_file")
    namespace = LaunchConfiguration("namespace")
    can_channel = LaunchConfiguration("can_channel")
    dbc_file = LaunchConfiguration("dbc_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value=""
        ),
        DeclareLaunchArgument(
            "can_channel",
            default_value="can0"
        ),
        DeclareLaunchArgument(
            "dbc_file",
            default_value=default_dbc_file
        ),
        Node(
            package="marsupial_robot_base",
            # executable="odom_node",
            executable="odom_node_copy",
            # name="odom_node",
            #vel_gain = 5.5
            #vel_integrator_gain = 4.0
            name="odom_node_copy",
            namespace=namespace,
            output="screen",
            parameters=[
                params_file,
                {
                    "can_channel": can_channel,
                    "can_dbc": dbc_file,
                }
            ],
        ),
    ])