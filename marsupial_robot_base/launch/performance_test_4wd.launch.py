from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("marsupial_robot_base")
    test_config = os.path.join(pkg_share, "config", "performance_test.yaml")
    base_launch = os.path.join(pkg_share, "launch", "bringup_base_4wd.launch.py")

    armed = LaunchConfiguration("armed")
    run_name = LaunchConfiguration("run_name")
    output_root = LaunchConfiguration("output_root")
    reference_topic = LaunchConfiguration("reference_odom_topic")
    imu_topic = LaunchConfiguration("imu_topic")

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch),
    )
    logger = Node(
        package="marsupial_robot_base",
        executable="performance_logger",
        name="performance_logger",
        output="screen",
        parameters=[
            test_config,
            {
                "run_name": run_name,
                "output_root": output_root,
                "reference_odom_topic": reference_topic,
                "imu_topic": imu_topic,
            },
        ],
    )
    runner = Node(
        package="marsupial_robot_base",
        executable="performance_test_runner",
        name="performance_test_runner",
        output="screen",
        parameters=[test_config, {"armed": armed}],
    )
    shutdown_when_done = RegisterEventHandler(
        OnProcessExit(
            target_action=runner,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason="Perfil de desempenho finalizado.",
                    )
                )
            ],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("armed", default_value="false"),
            DeclareLaunchArgument("run_name", default_value="terrain_4wd"),
            DeclareLaunchArgument(
                "output_root",
                default_value=os.path.expanduser("~/marsupial_test_results"),
            ),
            DeclareLaunchArgument(
                "reference_odom_topic",
                default_value="/zed/zed_node/odom",
            ),
            DeclareLaunchArgument(
                "imu_topic",
                default_value="/camera/camera/imu",
            ),
            base,
            logger,
            runner,
            shutdown_when_done,
        ]
    )
