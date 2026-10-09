"""Mot lenh mo tat ca de TU LAI (bang thanh truot) va ve ban do: Gazebo + SLAM Toolbox + RViz2 + bang dieu khien.

Ban do hien tren RViz2 khi lai; bang dieu khien co nut luu ban do, tu luu dinh ky va tu luu khi dong bang.
    ros2 launch tb4_lab04 drive_mapping.launch.py world:=maze
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('tb4_lab04')
    args = [
        DeclareLaunchArgument('world', default_value='maze', description='warehouse | depot | maze'),
        DeclareLaunchArgument('map_dir', default_value=os.path.expanduser('~/tb4_ws/src/tb4_lab04/maps'),
                              description='thu muc luu map.yaml + map.pgm'),
        DeclareLaunchArgument('autosave_period', default_value='120.0', description='giay giua cac lan tu luu'),
        DeclareLaunchArgument('software_render', default_value='true', description='dung hinh bang phan mem (sua LiDAR toan 0.0 tren card Intel)'),
        DeclareLaunchArgument('safety_override', default_value='full',
                              description='Create 3 motion_control: none | backup_only | full (full = cho lui/khong gioi han, chi dung o mo phong)'),
        DeclareLaunchArgument('max_linear', default_value='0.3'),
        DeclareLaunchArgument('max_angular', default_value='1.0'),
    ]
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg, 'launch', 'sim.launch.py'])),
        launch_arguments={'world': LaunchConfiguration('world'), 'rviz': 'false',
                          'software_render': LaunchConfiguration('software_render')}.items(),
    )
    # Create 3 mac dinh gioi han viec lui ("Reached backup limit"); trong mo phong cho phep lui tu do.
    safety = TimerAction(period=22.0, actions=[ExecuteProcess(
        cmd=['ros2', 'param', 'set', '/motion_control', 'safety_override', LaunchConfiguration('safety_override')],
        output='screen')])
    # SLAM + RViz2 sau khi Gazebo len (~25 s), bang dieu khien sau them vai giay
    mapping = TimerAction(period=25.0, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg, 'launch', 'mapping.launch.py'])))])
    panel = TimerAction(period=32.0, actions=[Node(
        package='tb4_lab04', executable='control_panel', name='control_panel', output='screen',
        parameters=[{
            'map_dir': LaunchConfiguration('map_dir'),
            'autosave_period': LaunchConfiguration('autosave_period'),
            'max_linear': LaunchConfiguration('max_linear'),
            'max_angular': LaunchConfiguration('max_angular'),
        }])])
    return LaunchDescription(args + [sim, safety, mapping, panel])
