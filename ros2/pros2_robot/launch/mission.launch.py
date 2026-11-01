"""
미션 전체 실행 (bringup + 카메라 + 맵/AMCL/Nav2 + 장난감 감지/따라가기 + 미션 GUI)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = 'pros2_robot'
NAV_START_DELAY = 2.0
GUI_START_DELAY = 6.0


def generate_launch_description():
    share = get_package_share_directory(PACKAGE)
    launch_dir = os.path.join(share, 'launch')

    map_yaml = LaunchConfiguration('map')
    toy = IfCondition(LaunchConfiguration('toy'))

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'robot_bringup.launch.py')),
        launch_arguments={'stream_url': LaunchConfiguration('stream_url')}.items()
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation.launch.py')),
        launch_arguments={
            'map': map_yaml,
            'params_file': LaunchConfiguration('params_file'),
            'rviz': LaunchConfiguration('rviz'),
        }.items()
    )
    
    camera = Node(
        package=PACKAGE,
        executable='mjpeg_camera_publisher',
        name='mjpeg_camera_publisher',
        output='screen',
        parameters=[{'stream_url': LaunchConfiguration('stream_url')}],
        condition=IfCondition(LaunchConfiguration('camera'))
    )

    detector = Node(
        package=PACKAGE,
        executable='toy_detector',
        name='toy_detector',
        output='screen',
        arguments=['--no-view'],
        condition=toy
    )

    tracker = Node(
        package=PACKAGE,
        executable='toy_tracker',
        name='toy_tracker',
        output='screen',
        arguments=['--start-disabled'],
        condition=toy
    )

    gui = Node(
        package=PACKAGE,
        executable='mission_gui',
        name='mission_gui',
        output='screen',
        parameters=[{'map_yaml': map_yaml}],
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.join(share, 'maps', 'map4.yaml'),
                              description='map yaml file'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(share, 'config', 'nav2_params.yaml'),
                              description='AMCL / Nav2 parameter file'),
        DeclareLaunchArgument('rviz', default_value='false', description='also start rviz2'),
        DeclareLaunchArgument('stream_url', default_value='http://192.168.137.100/stream',
                              description='ESP32-CAM MJPEG stream URL'),
        DeclareLaunchArgument('camera', default_value='true', description='start the camera node'),
        DeclareLaunchArgument('toy', default_value='true',
                              description='start toy_detector / toy_tracker'),
        DeclareLaunchArgument('gui', default_value='true', description='start mission_gui'),
        bringup,
        TimerAction(period=NAV_START_DELAY, actions=[navigation]),
        camera,
        detector,
        tracker,
        TimerAction(period=GUI_START_DELAY, actions=[gui]),
    ])
