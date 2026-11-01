import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'pros2_robot'

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    micro_ros = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent_udp',
        output='screen',
        arguments=['udp4', '--port', '8888', '-v4']
    )

    joint_pub = Node(
        package=package_name,
        executable='robot_telemetry_publisher',
        name='robot_telemetry_publisher',
        arguments=['--ros-args', '--log-level', 'INFO']
    )

    lidar_pub = Node(
        package=package_name,
        executable='lidar_publisher',
        name='lidar_publisher',
        arguments=['--ros-args', '--log-level', 'WARN']
    )

    stream_url_arg = DeclareLaunchArgument(
        'stream_url', default_value='http://192.168.137.100/stream',
        description='ESP32-CAM MJPEG stream URL')

    camera = Node(
        package=package_name,
        executable='mjpeg_camera_publisher',
        name='mjpeg_camera_publisher',
        output='screen',
        parameters=[{'stream_url': LaunchConfiguration('stream_url')}]
    )

    return LaunchDescription([
        stream_url_arg,
        micro_ros,
        joint_pub,
        lidar_pub,
        rsp#,
        #camera,
    ])
