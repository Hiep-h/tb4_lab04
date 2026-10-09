"""Mot lenh mo tat ca de TU LAI (bang thanh truot) va ve ban do: Gazebo + SLAM Toolbox + RViz2 + bang dieu khien.

Ban do hien tren RViz2 khi lai; bang dieu khien co nut luu ban do, tu luu dinh ky va tu luu khi dong bang.
    ros2 launch tb4_lab04 drive_mapping.launch.py world:=maze
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
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('map_dir', default_value=os.path.expanduser('~/tb4_ws/src/tb4_lab04/maps'),
                              description='thu muc luu map.yaml + map.pgm'),
        DeclareLaunchArgument('autosave_period', default_value='120.0', description='giay giua cac lan tu luu'),
        DeclareLaunchArgument('software_render', default_value='true', description='dung hinh bang phan mem (sua LiDAR toan 0.0 tren card Intel)'),
        DeclareLaunchArgument('mapping_delay', default_value='60.0', description='giay cho Gazebo len roi moi chay SLAM + RViz2'),
        DeclareLaunchArgument('explore_delay', default_value='90.0', description='giay truoc khi chay auto_explore'),
        DeclareLaunchArgument('panel_delay', default_value='70.0', description='giay truoc khi mo bang dieu khien'),
        DeclareLaunchArgument('safety_override', default_value='full',
                              description='Create 3 motion_control: none | backup_only | full (full = cho lui/khong gioi han, chi dung o mo phong)'),
        DeclareLaunchArgument('max_linear', default_value='0.3'),
        DeclareLaunchArgument('max_angular', default_value='1.0'),
    ]
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg, 'launch', 'sim.launch.py'])),
        launch_arguments={'world': LaunchConfiguration('world'), 'rviz': 'false',
                          'x': LaunchConfiguration('x'), 'y': LaunchConfiguration('y'), 'yaw': LaunchConfiguration('yaw'),
                          'software_render': LaunchConfiguration('software_render')}.items(),
    )
    # Create 3 mac dinh gioi han viec lui ("Reached backup limit"); trong mo phong cho phep lui tu do.
    safety = TimerAction(period=30.0, actions=[ExecuteProcess(
        cmd=['bash', '-c', 'for i in $(seq 1 40); do ros2 param set /motion_control safety_override '
             + '$0 && break; sleep 5; done', LaunchConfiguration('safety_override')],
        output='screen')])
    mapping = TimerAction(period=LaunchConfiguration('mapping_delay'), actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg, 'launch', 'mapping.launch.py'])))])
    panel = TimerAction(period=LaunchConfiguration('panel_delay'), actions=[Node(
        package='tb4_lab04', executable='control_panel', name='control_panel', output='screen',
        parameters=[{
            'map_dir': LaunchConfiguration('map_dir'),
            'autosave_period': LaunchConfiguration('autosave_period'),
            'max_linear': LaunchConfiguration('max_linear'),
            'max_angular': LaunchConfiguration('max_angular'),
        }])])
    # chi Gazebo dung hinh bang phan mem; cac tien trinh sau (RViz2, SLAM...) dung card do hoa binh thuong
    unset = UnsetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE')
    return LaunchDescription(args + [sim, unset, safety, mapping, panel])
