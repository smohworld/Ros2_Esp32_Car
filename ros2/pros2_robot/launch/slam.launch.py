"""
SLAM(slam_toolbox async) + Nav2(navigation) 실행 - 맵을 새로 만들 때 사용

  ros2 launch pros2_robot slam.launch.py
  ros2 launch pros2_robot slam.launch.py rviz:=true

인자
  slam_params_file : slam_toolbox 파라미터 (기본: share/pros2_robot/config/slam_params.yaml)
  params_file      : Nav2 파라미터 (기본: share/pros2_robot/config/nav2_params.yaml)
  rviz             : true면 rviz2(config/nav2.rviz)도 실행
  use_sim_time     : 기본 false (slam_toolbox launch의 기본값은 true라서 여기서 false로 넘김)

맵 저장: ros2 run nav2_map_server map_saver_cli -f /root/maps/<이름>
확정된 맵은 maps/ 폴더로 옮겨서 navigation.launch.py map:= 로 사용

robot_bringup.launch.py는 따로 띄워 둔 상태에서 실행
Nav2는 slam_toolbox가 map->odom TF와 첫 /map을 낼 시간을 주려고 3초 뒤에 시작
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
NAV2_START_DELAY = 3.0


def generate_launch_description():
    share = get_package_share_directory(PACKAGE)

    use_sim_time = LaunchConfiguration('use_sim_time')

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': LaunchConfiguration('slam_params_file'),
            'use_sim_time': use_sim_time,
        }.items()
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': use_sim_time,
            'autostart': 'true',
        }.items()
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(share, 'config', 'nav2.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        DeclareLaunchArgument('slam_params_file',
                              default_value=os.path.join(share, 'config', 'slam_params.yaml'),
                              description='slam_toolbox parameter file'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(share, 'config', 'nav2_params.yaml'),
                              description='Nav2 parameter file'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false', description='also start rviz2'),
        slam,
        TimerAction(period=NAV2_START_DELAY, actions=[nav2]),
        rviz,
    ])
