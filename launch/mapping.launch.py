"""Phan 2: SLAM Toolbox + RViz2 (chay sau khi da mo mo phong bang sim.launch.py)."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('turtlebot4_navigation'), 'launch', 'slam.launch.py'])),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )
    viz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('turtlebot4_viz'), 'launch', 'view_robot.launch.py'])),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )
    return LaunchDescription([slam, viz])
