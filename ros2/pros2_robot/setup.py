from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pros2_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'description'), glob('description/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'models'), glob('yolo/models/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='smoh',
    maintainer_email='dhtjdalsqkqh@gmail.com',
    description='ESP32 micro-ROS RC car: SLAM/Nav2 navigation, LiDAR-based parking, YOLOv8 toy detection and tracking, mission GUI',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robot_data_publisher = pros2_robot.robot_data_publisher:main',
            'robot_telemetry_publisher = pros2_robot.robot_telemetry_publisher:main',
            'lidar_publisher = pros2_robot.lidar_publisher:main',
            'teleop_twist_keyboard = pros2_robot.teleop_twist_keyboard:main',
            'toy_detector = pros2_robot.toy_detector:main',
            'toy_tracker = pros2_robot.toy_tracker:main',
            'mjpeg_camera_publisher = pros2_robot.mjpeg_camera_publisher:main',
            'mission_gui = pros2_robot.mission_gui:main',
            'parking = pros2_robot.parking:main',
            'mission = pros2_robot.mission_runner:main',
        ],
    },
)
