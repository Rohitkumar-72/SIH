"""Spawn one TurtleBot 4 Standard into an already-running warehouse world."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    turtlebot_launch = os.path.join(
        get_package_share_directory('turtlebot4_gz_bringup'),
        'launch', 'turtlebot4_spawn.launch.py')
    arguments = {
        'namespace': LaunchConfiguration('namespace'),
        'model': LaunchConfiguration('model'),
        'x': LaunchConfiguration('x'),
        'y': LaunchConfiguration('y'),
        'z': LaunchConfiguration('z'),
        'yaw': LaunchConfiguration('yaw'),
    }
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='robot_1'),
        DeclareLaunchArgument('model', default_value='standard'),
        DeclareLaunchArgument('x', default_value='2.0'),
        DeclareLaunchArgument('y', default_value='2.0'),
        DeclareLaunchArgument('z', default_value='0.05'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(turtlebot_launch),
            launch_arguments=arguments.items()),
    ])
