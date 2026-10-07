"""Phan 1: TurtleBot 4 trong Gazebo (Ignition) - dung launch co san cua turtlebot4_ignition_bringup."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument('world', default_value='maze', description='warehouse | depot | maze'),
        DeclareLaunchArgument('model', default_value='standard', description='standard | lite'),
        DeclareLaunchArgument('rviz', default_value='false', description='mo RViz2 cung luc'),
    ]
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('turtlebot4_ignition_bringup'), 'launch', 'turtlebot4_ignition.launch.py'])),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'model': LaunchConfiguration('model'),
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
    )
    return LaunchDescription(args + [sim])
