"""Phan 3 + 4 mot lenh: mo phong + dinh vi (AMCL, nap map da luu) + Nav2 + RViz2.

Robot xuat phat dung cho da dung khi ve map (x:=1.0 y:=1.0 yaw:=0.0) nen tu dat vi tri ban dau (0, 0, 0) trong khung map
(goc khung map cua SLAM la cho robot xuat phat). Sau do dat cac diem dich bang cong cu "Nav2 Goal" trong RViz2.
"""
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            TimerAction, UnsetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('tb4_lab04')
    args = [
        DeclareLaunchArgument('world', default_value='small_house'),
        DeclareLaunchArgument('x', default_value='1.0', description='vi tri xuat phat trong world (giong luc ve map)'),
        DeclareLaunchArgument('y', default_value='1.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('map', default_value=PathJoinSubstitution([pkg, 'maps', 'map.yaml'])),
        DeclareLaunchArgument('software_render', default_value='true'),
        DeclareLaunchArgument('loc_delay', default_value='60.0', description='giay cho Gazebo len roi chay dinh vi'),
        DeclareLaunchArgument('nav_delay', default_value='90.0', description='giay truoc khi chay Nav2'),
        DeclareLaunchArgument('pose_delay', default_value='80.0', description='giay truoc khi dat vi tri ban dau'),
        DeclareLaunchArgument('safety_override', default_value='full'),
    ]
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg, 'launch', 'sim.launch.py'])),
        launch_arguments={'world': LaunchConfiguration('world'), 'rviz': 'false',
                          'x': LaunchConfiguration('x'), 'y': LaunchConfiguration('y'), 'yaw': LaunchConfiguration('yaw'),
                          'software_render': LaunchConfiguration('software_render')}.items(),
    )
    # chi Gazebo dung hinh bang phan mem
    unset = UnsetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE')
    safety = TimerAction(period=30.0, actions=[ExecuteProcess(
        cmd=['bash', '-c', 'for i in $(seq 1 40); do ros2 param set /motion_control safety_override '
             + '$0 && break; sleep 5; done', LaunchConfiguration('safety_override')], output='screen')])
    nav_pkg = FindPackageShare('turtlebot4_navigation')
    localization = TimerAction(period=LaunchConfiguration('loc_delay'), actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([nav_pkg, 'launch', 'localization.launch.py'])),
            launch_arguments={'map': LaunchConfiguration('map'), 'use_sim_time': 'true'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('turtlebot4_viz'), 'launch', 'view_robot.launch.py'])),
            launch_arguments={'use_sim_time': 'true'}.items()),
    ])
    nav2 = TimerAction(period=LaunchConfiguration('nav_delay'), actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([nav_pkg, 'launch', 'nav2.launch.py'])),
        launch_arguments={'use_sim_time': 'true'}.items())])
    cov = '[' + ', '.join(['0.25' if i in (0, 7) else '0.07' if i == 35 else '0.0' for i in range(36)]) + ']'
    pose = ('{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, '
            'orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: %s}}' % cov)
    initial = TimerAction(period=LaunchConfiguration('pose_delay'), actions=[ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '-t', '8', '-r', '1', '/initialpose',
             'geometry_msgs/msg/PoseWithCovarianceStamped', pose], output='screen')])
    return LaunchDescription(args + [sim, unset, safety, localization, initial, nav2])
