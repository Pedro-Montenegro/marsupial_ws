from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Diretórios de pacotes
    nav2_dir = get_package_share_directory('nav2_bringup')
    marsupial_pkg_dir = get_package_share_directory('marsupial_robot_navigation')

    # Configurações de Launch
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # --- ZED cloud -> LaserScan ---
    """ zed_cloud_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[
            ('cloud_in', '/zed/zed_node/point_cloud/cloud_registered'),
            ('scan', '/zed/scan'),
        ],
        parameters=[params_file],
    ) """

    # --- Nó: Mission Orchestrator ---
    mission_orchestrator_node = Node(
        package='marsupial_robot_navigation',
        executable='mission_orchestrator',
        name='mission_orchestrator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'plan_topic': '/plan',
            'reach_tol': 0.3,
            'progress_hz': 5.0
        }]
    )

    # --- Nó: Teleop Relay ---
    teleop_relay_node = Node(
        package='marsupial_robot_navigation',
        executable='teleop_relay',
        name='teleop_relay',
        output='screen',
        parameters=[{'watchdog_timeout': 0.5}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                get_package_share_directory('marsupial_robot_navigation'),
                'config',
                'nav2_params.yaml'
            )
        ),

        # Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file
            }.items()
        ),

        # Iniciar os nós da camada Marsupial
        mission_orchestrator_node,
        teleop_relay_node,

        # ZED -> LaserScan
        # zed_cloud_to_scan,
    ])