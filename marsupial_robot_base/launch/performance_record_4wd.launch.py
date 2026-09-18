from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("marsupial_robot_base")
    test_config = os.path.join(pkg_share, "config", "performance_test.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("run_name", default_value="manual_4wd"),
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
            Node(
                package="marsupial_robot_base",
                executable="performance_logger",
                name="performance_logger",
                output="screen",
                parameters=[
                    test_config,
                    {
                        "run_name": LaunchConfiguration("run_name"),
                        "output_root": LaunchConfiguration("output_root"),
                        "reference_odom_topic": LaunchConfiguration(
                            "reference_odom_topic"
                        ),
                        "imu_topic": LaunchConfiguration("imu_topic"),
                    },
                ],
            ),
        ]
    )
