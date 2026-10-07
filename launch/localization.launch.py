"""Phan 3: nap map da luu va chay localization (AMCL) + RViz2."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_map = PathJoinSubstitution([FindPackageShare('tb4_lab04'), 'maps', 'map.yaml'])
    map_arg = DeclareLaunchArgument('map', default_value=default_map, description='duong dan map.yaml')
    loc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('turtlebot4_navigation'), 'launch', 'localization.launch.py'])),
        launch_arguments={'map': LaunchConfiguration('map'), 'use_sim_time': 'true'}.items(),
    )
    viz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('turtlebot4_viz'), 'launch', 'view_robot.launch.py'])),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )
    return LaunchDescription([map_arg, loc, viz])
