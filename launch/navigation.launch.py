"""Phan 4: Nav2 (chay cung localization, dung map da luu)."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('turtlebot4_navigation'), 'launch', 'nav2.launch.py'])),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )
    return LaunchDescription([nav2])
