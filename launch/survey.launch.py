"""Tu lai khao sat de SLAM ve ban do (thay cho teleop). Chay cung mapping.launch.py."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('speed', default_value='0.18'),
        DeclareLaunchArgument('target_dist', default_value='0.55'),
        DeclareLaunchArgument('min_path', default_value='15.0'),
        DeclareLaunchArgument('max_duration', default_value='900.0'),
    ]
    node = Node(
        package='tb4_lab04', executable='auto_survey', name='auto_survey', output='screen',
        parameters=[{
            'use_sim_time': True,
            'speed': LaunchConfiguration('speed'),
            'target_dist': LaunchConfiguration('target_dist'),
            'min_path': LaunchConfiguration('min_path'),
            'max_duration': LaunchConfiguration('max_duration'),
        }],
    )
    return LaunchDescription(args + [node])
