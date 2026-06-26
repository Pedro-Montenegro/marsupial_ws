# marsupial_ws/src/marsupial_ui_bridge/setup.py

from setuptools import setup

package_name = 'marsupial_ui_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marsupial',
    maintainer_email='marsupial@local.local',
    description='Bridge entre UI Web e Nav2 (NavigateToPose / initialpose)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ui_bridge_node = marsupial_ui_bridge.ui_bridge_node:main',
            'ui_network_node = marsupial_ui_bridge.ui_network_node:main',
            'ui_mjpeg_node = marsupial_ui_bridge.ui_mjpeg_node:main',
            'config_bridge_node = marsupial_ui_bridge.config_bridge_node:main',
            'ui_sensors_node = marsupial_ui_bridge.ui_sensors_node:main',
            'marsupial_service_manager = marsupial_ui_bridge.marsupial_service_manager:main',
        ],
    },
)
