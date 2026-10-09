"""MOT LENH: robot TU KHAM PHA va ve ban do, tu luu map.yaml + map.pgm khi xong.

Gazebo + SLAM Toolbox + RViz2 + node auto_explore (tim bien gan nhat, lap duong tranh vat can).
    ros2 launch tb4_lab04 auto_mapping.launch.py world:=maze
"""
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction,
                            UnsetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('tb4_lab04')
    args = [
        DeclareLaunchArgument('world', default_value='maze', description='warehouse | depot | maze | small_house'),
        DeclareLaunchArgument('headless', default_value='false', description='true: khong mo cua so Gazebo (nhe hon)'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('map_dir', default_value=os.path.expanduser('~/tb4_ws/src/tb4_lab04/maps')),
        DeclareLaunchArgument('software_render', default_value='true', description='dung hinh bang phan mem (sua LiDAR toan 0.0 tren card Intel)'),
        DeclareLaunchArgument('mapping_delay', default_value='60.0', description='giay cho Gazebo len roi moi chay SLAM + RViz2'),
        DeclareLaunchArgument('explore_delay', default_value='90.0', description='giay truoc khi chay auto_explore'),
        DeclareLaunchArgument('panel_delay', default_value='70.0', description='giay truoc khi mo bang dieu khien'),
        DeclareLaunchArgument('safety_override', default_value='full'),
        DeclareLaunchArgument('vmax', default_value='0.45', description='toc do tien toi da (m/s)'),
        DeclareLaunchArgument('max_duration', default_value='1800.0', description='giay (gio mo phong)'),
    ]
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg, 'launch', 'sim.launch.py'])),
        launch_arguments={'world': LaunchConfiguration('world'), 'rviz': 'false',
                          'headless': LaunchConfiguration('headless'), 'x': LaunchConfiguration('x'), 'y': LaunchConfiguration('y'), 'yaw': LaunchConfiguration('yaw'),
                          'software_render': LaunchConfiguration('software_render')}.items(),
    )
    safety = TimerAction(period=30.0, actions=[ExecuteProcess(
        cmd=['bash', '-c', 'for i in $(seq 1 40); do ros2 param set /motion_control safety_override '
             + '$0 && break; sleep 5; done', LaunchConfiguration('safety_override')],
        output='screen')])
    mapping = TimerAction(period=LaunchConfiguration('mapping_delay'), actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg, 'launch', 'mapping.launch.py'])))])
    explore = TimerAction(period=LaunchConfiguration('explore_delay'), actions=[Node(
        package='tb4_lab04', executable='auto_explore', name='auto_explore', output='screen',
        parameters=[{
            'use_sim_time': True,
            'map_dir': LaunchConfiguration('map_dir'),
            'vmax': LaunchConfiguration('vmax'),
            'max_duration': LaunchConfiguration('max_duration'),
        }])])
    # chi Gazebo dung hinh bang phan mem; cac tien trinh sau (RViz2, SLAM...) dung card do hoa binh thuong
    unset = UnsetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE')
    return LaunchDescription(args + [sim, unset, safety, mapping, explore])
