from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from pathlib import Path

def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    ekf_yaml = LaunchConfiguration("ekf_yaml")
    amcl_yaml = LaunchConfiguration("amcl_yaml")
    map_yaml = LaunchConfiguration("map")
    last_pose_path = LaunchConfiguration("last_pose_path") 

    share_dir = get_package_share_directory("marsupial_robot_localization")

    # EKF (odom -> base_link)
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_odom",
        namespace=namespace,
        output="screen",
        parameters=[ekf_yaml],
    )
    ekf_delayed = TimerAction(period=1.0, actions=[ekf_node])

    # AMCL + map_server (map -> odom)
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share_dir, "launch", "amcl_loc.launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "params_file": amcl_yaml,
            "map": map_yaml,
        }.items()
    )
    amcl_delayed = TimerAction(period=5.0, actions=[amcl_launch])

    # localization_manager antigo, mas agora em namespace
    init_loc = Node(
        package="marsupial_robot_localization",
        executable="localization_manager",
        name="localization_manager",
        namespace=namespace,
        parameters=[{
            "mode": "boot",
            "last_pose_path": last_pose_path,
            "spin_wz": 0.35,
            "spins": 1,
            "do_fb": True,
        }],
        output="screen",
    )
    init_loc_delayed = TimerAction(period=10.0, actions=[init_loc])

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument(
            "ekf_yaml",
            default_value=os.path.join(share_dir, "config", "ekf.yaml"),
        ),
        DeclareLaunchArgument(
            "amcl_yaml",
            default_value=os.path.join(share_dir, "config", "amcl_nav2.yaml"),
        ),
        DeclareLaunchArgument(
            "map",
            default_value=str(Path.home() / "maps" / "Lab_30032026.yaml"),
        ),
        DeclareLaunchArgument(
            "last_pose_path",
            default_value=str(Path.home() / ".marsupial" / "last_pose.yaml"),
        ),
        ekf_delayed,
        amcl_delayed,
        init_loc_delayed,
    ])
