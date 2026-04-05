from setuptools import setup
from glob import glob
import os

package_name = 'swarm_state_estimation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'camera_info'), glob('config/camera_info/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools', 'numpy', 'PyYAML'],
    zip_safe=True,
    maintainer='Joshua Kevil',
    maintainer_email='josh@example.com',
    description='AprilTag + IMU to UKF localization pipeline for modular swarm robots.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'usb_camera_node = swarm_state_estimation.usb_camera_node:main',
            'tag_router = swarm_state_estimation.tag_router:main',
            'ukf_manager = swarm_state_estimation.ukf_manager:main',
        ],
    },
)
