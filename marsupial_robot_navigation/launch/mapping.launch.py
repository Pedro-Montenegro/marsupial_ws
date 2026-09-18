from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params_file = LaunchConfiguration("slam_params_file")
    ekf_params_file = LaunchConfiguration("ekf_params_file")
    include_base = LaunchConfiguration("include_base")
    include_sensors = LaunchConfiguration("include_sensors")
    include_teleop = LaunchConfiguration("include_teleop")
    device_id = LaunchConfiguration("device_id")

    nav_share = get_package_share_directory("marsupial_robot_navigation")
    base_share = get_package_share_directory("marsupial_robot_base")
    sensors_share = get_package_share_directory("marsupial_robot_sensors")
    loc_share = get_package_share_directory("marsupial_robot_localization")

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(base_share, "launch", "bringup_base_4wd_can.launch.py")
        ),
        condition=IfCondition(include_base),
        launch_arguments={
            "namespace": namespace,
        }.items(),
    )

    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sensors_share, "launch", "sensors.launch.py")
        ),
        condition=IfCondition(include_sensors),
        launch_arguments={
            "namespace": namespace,
        }.items(),
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_odom",
        namespace=namespace,
        output="screen",
        parameters=[ekf_params_file],
    )

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(base_share, "launch", "gamepad_teleop_4wd.launch.py")
        ),
        condition=IfCondition(include_teleop),
        launch_arguments={
            "include_base": "false",
            "device_id": device_id,
        }.items(),
    )

    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_params_file,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("scan", "/scan"),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("include_base", default_value="true"),
        DeclareLaunchArgument("include_sensors", default_value="true"),
        # Evita dois publicadores concorrentes em /cmd_vel. O gamepad publica
        # zero continuamente sem deadman e sobrescreve teleop por teclado/UI.
        DeclareLaunchArgument("include_teleop", default_value="false"),
        DeclareLaunchArgument("device_id", default_value="0"),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=os.path.join(
                nav_share, "config", "slam_toolbox_mapping.yaml"
            ),
        ),
        DeclareLaunchArgument(
            "ekf_params_file",
            default_value=os.path.join(loc_share, "config", "ekf.yaml"),
        ),
        base_launch,
        sensors_launch,
        TimerAction(period=2.0, actions=[ekf_node]),
        teleop_launch,
        TimerAction(period=5.0, actions=[slam_toolbox_node]),
    ])
