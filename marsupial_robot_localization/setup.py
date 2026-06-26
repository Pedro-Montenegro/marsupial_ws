from setuptools import setup
from glob import glob
import os

package_name = 'marsupial_robot_localization'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marsupial',
    maintainer_email='voce@marsupial.com',
    description='Camada de localização da frota Marsupial.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'localization_manager = marsupial_robot_localization.localization_manager:main',
        'pose_checkpoint_manager =  marsupial_robot_localization.pose_checkpoint_manager:main',
        'reinitialize_global_localization = marsupial_robot_localization.reinitialize_global_localization:main',
    ],
},
)
