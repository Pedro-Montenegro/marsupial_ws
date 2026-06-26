from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from pathlib import Path

def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    params = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map")
    default_params = os.path.join(
        get_package_share_directory("marsupial_robot_localization"),
        "config",
        "amcl_nav2.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument(
            "map",
            default_value=str(Path.home() / "maps" / "Lab_30032026.yaml"),
        ),
        Node(package="nav2_map_server", executable="map_server",
             name="map_server", namespace=namespace, output="screen",
             parameters=[params, {"yaml_filename": map_yaml}]),
        Node(package="nav2_amcl", executable="amcl",
             name="amcl", namespace=namespace, output="screen", parameters=[params]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_localization", namespace=namespace, output="screen",
             parameters=[{"autostart": True, "node_names": ["map_server", "amcl"]}]),
    ])
 
