from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    enable_realsense = LaunchConfiguration("enable_realsense")

    # ── RealSense D435i — configuracao leve, somente IMU ────────────
    # Todos os streams de imagem e filtros ficam desligados para minimizar CPU,
    # memoria e trafego USB. Gyro + acelerometro alimentam /camera/camera/imu.
    # rs_launch.py inspeciona todas as LaunchConfigurations visiveis. O grupo
    # com escopo impede que argumentos do mapping/base sejam tratados como se
    # fossem parametros da camera.
    realsense_launch = GroupAction(
        scoped=True,
        forwarding=False,
        condition=IfCondition(enable_realsense),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("realsense2_camera"),
                    "launch",
                    "rs_launch.py",
                )
            ),
            launch_arguments={
                "camera_name":           "camera",
                "camera_namespace":      "camera",
                "enable_color":          "false",
                "enable_depth":          "false",
                "enable_infra":          "false",
                "enable_infra1":         "false",
                "enable_infra2":         "false",
                "enable_gyro":           "true",
                "enable_accel":          "true",
                "unite_imu_method":      "1",        # /camera/camera/imu
                "pointcloud.enable":     "false",
                "align_depth.enable":    "false",
                "enable_sync":           "false",
                # Necessario para o EKF transformar os frames de gyro/accel
                # (camera_gyro_optical_frame etc.) ate camera_link/base_link.
                "publish_tf":            "true",
            }.items(),
        )],
    )

    # ── RPLIDAR A2 (sllidar) ────────────────────────────────────────────────
    lidar_node = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        namespace=namespace,
        parameters=[{
            "channel_type": "serial",
            "serial_port": "/dev/rplidar",
            "serial_baudrate": 256000,
            "frame_id": "laser_link",
            "inverted": False,
            "angle_compensate": True,
            "scan_mode": "Sensitivity",
        }],
        output="screen",
    )

    # ── TFs estáticos ─────────────────────────────────────────────────────────
    # Atualizar xyz com medidas reais do robô 4WD
    laser_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="laser_tf",
        namespace=namespace,
        arguments=["0.18", "0.0", "0.24", "0", "0", "0", "1", "base_link", "laser_link"],
        output="screen",
    )

    camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_tf",
        namespace=namespace,
        arguments=["0.18", "0.0", "0.25", "0", "0", "0", "1", "base_link", "camera_link"],
        output="screen",
    )

    tf_static_delayed = TimerAction(
        period=5.0,
        actions=[laser_tf, camera_tf],
    )

    ui_sensors = Node(
        package="marsupial_ui_bridge",
        executable="ui_sensors_node",
        name="ui_sensors_node",
        namespace=namespace,
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument(
            "enable_realsense", default_value="false",
            description="Desligar quando a RealSense ja estiver sendo usada por outro processo (ex: Isaac ROS VSLAM no container Docker).",
        ),
        realsense_launch,
        lidar_node,
        tf_static_delayed,
        ui_sensors,
    ])
