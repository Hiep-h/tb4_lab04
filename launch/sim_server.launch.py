"""Mo phong TurtleBot 4 CHI chay phan may chu cua Gazebo (khong cua so 3D) -> nhe hon rat nhieu.

Giong turtlebot4_ignition.launch.py nhung them co '-s' (server only) va khong nap GUI. Xem map bang RViz2.
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('world', default_value='maze'),
        DeclareLaunchArgument('model', default_value='standard'),
    ]
    for k in ['x', 'y', 'z', 'yaw']:
        args.append(DeclareLaunchArgument(k, default_value='0.0'))

    bringup = get_package_share_directory('turtlebot4_ignition_bringup')
    create_desc = get_package_share_directory('irobot_create_description')
    create_bringup = get_package_share_directory('irobot_create_ignition_bringup')
    tb4_desc = get_package_share_directory('turtlebot4_description')
    ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')

    resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[os.path.join(bringup, 'worlds'), ':' + os.path.join(create_bringup, 'worlds'), ':' +
               str(Path(tb4_desc).parent.resolve()), ':' + str(Path(create_desc).parent.resolve())])
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'])),
        launch_arguments=[('ign_args', [LaunchConfiguration('world'), '.sdf', ' -r -s -v 2'])])
    clock = Node(package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge', output='screen',
                 arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'])
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([bringup, 'launch', 'turtlebot4_spawn.launch.py'])),
        launch_arguments=[('namespace', LaunchConfiguration('namespace')), ('rviz', LaunchConfiguration('rviz')),
                          ('x', LaunchConfiguration('x')), ('y', LaunchConfiguration('y')),
                          ('z', LaunchConfiguration('z')), ('yaw', LaunchConfiguration('yaw'))])
    return LaunchDescription(args + [resource_path, gz, clock, spawn])
