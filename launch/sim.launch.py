"""Phan 1: TurtleBot 4 trong Gazebo (Ignition) - dung launch co san cua turtlebot4_ignition_bringup."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument('world', default_value='maze', description='warehouse | depot | maze | small_house (xem scripts/install_world.sh)'),
        DeclareLaunchArgument('model', default_value='standard', description='standard | lite'),
        DeclareLaunchArgument('x', default_value='0.0', description='vi tri xuat phat x (m)'),
        DeclareLaunchArgument('y', default_value='0.0', description='vi tri xuat phat y (m)'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='huong xuat phat (rad)'),
        DeclareLaunchArgument('rviz', default_value='false', description='mo RViz2 cung luc'),
        DeclareLaunchArgument('software_render', default_value='true',
                              description='dung hinh bang phan mem (LIBGL_ALWAYS_SOFTWARE=1): sua loi LiDAR toan 0.0 tren card Intel'),
    ]
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('turtlebot4_ignition_bringup'), 'launch', 'turtlebot4_ignition.launch.py'])),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'model': LaunchConfiguration('model'),
            'rviz': LaunchConfiguration('rviz'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'yaw': LaunchConfiguration('yaw'),
        }.items(),
    )
    soft = SetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE', value='1',
                                  condition=IfCondition(LaunchConfiguration('software_render')))
    return LaunchDescription(args + [soft, sim])
