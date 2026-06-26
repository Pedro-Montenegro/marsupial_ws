from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")

    # ── RealSense D435i — config leve para navegação ──────────────────────
    # Color desabilitado (não necessário para nav)
    # Depth: 424x240 @ 15fps (suficiente para detecção de obstáculos)
    # IMU unificado em /camera/camera/imu via unite_imu_method=1
    # Point cloud desabilitado (pesado, substituído pelo depthimage_to_laserscan)
    realsense_launch = IncludeLaunchDescription(
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
            "enable_depth":          "true",
            "depth_module.profile":  "424,240,15",
            "enable_gyro":           "true",
            "enable_accel":          "true",
            "unite_imu_method":      "1",        # /camera/camera/imu
            "pointcloud.enable":     "false",
            "align_depth.enable":    "false",
            "publish_tf":            "false",
        }.items(),
    )

    # ── depth → LaserScan virtual (obstáculos baixos não vistos pelo LiDAR) ─
    depth_scan_node = Node(
        package="depthimage_to_laserscan",
        executable="depthimage_to_laserscan_node",
        name="depth_to_scan",
        namespace=namespace,
        parameters=[{
            "output_frame":  "camera_link",
            "range_min":     0.15,
            "range_max":     3.0,
            "scan_height":   10,    # linhas centrais da imagem usadas
        }],
        remappings=[
            ("depth",             "/camera/camera/depth/image_rect_raw"),
            ("depth_camera_info", "/camera/camera/depth/camera_info"),
            ("scan",              "/depth/scan"),
        ],
        output="screen",
    )

    # ── M1CT_D2 LiDAR ────────────────────────────────────────────────────────
    lidar_node = Node(
        package="m1ct_d2",
        executable="m1ct_d2",
        name="m1ct_d2_node",
        namespace=namespace,
        parameters=[
            os.path.join(get_package_share_directory("m1ct_d2"), "params", "m1ct_d2.yaml")
        ],
        remappings=[("scan_coin", "scan")],
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
        realsense_launch,
        lidar_node,
        depth_scan_node,
        tf_static_delayed,
        ui_sensors,
    ])
