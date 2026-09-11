"""Spawn a lightweight kinematic LiDAR carrier AMR in Gazebo.

Replaces wheel physics with a rigid carrier model containing only base_link and
the RPLIDAR sensor. Gazebo acts solely as a ray-tracing LiDAR renderer, bypassing
ros2_control, joint controllers, casters, and wheel contact friction.
"""

from ament_index_python.packages import get_package_share_directory
from irobot_create_common_bringup.namespace import GetNamespacedName
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


ARGUMENTS = [
    DeclareLaunchArgument('namespace', description='Unique robot namespace.'),
    DeclareLaunchArgument('x', description='Robot world x position.'),
    DeclareLaunchArgument('y', description='Robot world y position.'),
    DeclareLaunchArgument('z', default_value='0.03', description='Robot world z position.'),
    DeclareLaunchArgument('yaw', default_value='0.0', description='Robot world yaw in radians.'),
    DeclareLaunchArgument('world', default_value='default',
                          description='Gazebo world name used by the LiDAR bridge.'),
    DeclareLaunchArgument('lidar_update_rate_hz', default_value='10.0',
                          description='Navigation LiDAR update rate in Hz.'),
    DeclareLaunchArgument('description_wait_s', default_value='1.0',
                          description='Delay insertion so robot description is published.'),
    DeclareLaunchArgument('map_file', default_value='',
                          description='Warehouse map file containing dock anchors.'),
]


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    x, y, z, yaw = (LaunchConfiguration(name) for name in ('x', 'y', 'z', 'yaw'))
    world = LaunchConfiguration('world')
    lidar_update_rate = LaunchConfiguration('lidar_update_rate_hz')
    map_file = LaunchConfiguration('map_file')
    robot_name = GetNamespacedName(namespace, 'turtlebot4')

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command([
                'python3', ' ', '-m', ' ',
                'sih_amr_fleet.carrier_robot_description', ' ',
                '--namespace', ' ', namespace, ' ',
                '--lidar-update-rate', ' ', lidar_update_rate]),
        }],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')])

    create_robot = Node(
        package='ros_gz_sim', executable='create', name='create_robot', output='screen',
        arguments=['-name', robot_name, '-x', x, '-y', y, '-z', z,
                   '-Y', yaw, '-topic', 'robot_description'])

    lidar_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='lidar_bridge',
        output='screen', parameters=[{'use_sim_time': True}],
        arguments=[['/world/', world, '/model/', robot_name,
                    '/link/rplidar_link/sensor/rplidar/scan'
                    '@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan']],
        remappings=[
            (['/world/', world, '/model/', robot_name,
              '/link/rplidar_link/sensor/rplidar/scan'], 'scan')])

    interface_readiness = Node(
        package='sih_amr_fleet', executable='interface_readiness_node',
        name='interface_readiness', output='screen',
        parameters=[{'use_sim_time': True, 'carrier_mode': True}])

    localization = Node(
        package='sih_amr_fleet', executable='localization_node',
        name='localization_node', output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_id': namespace,
            'odom_origin_x': x,
            'odom_origin_y': y,
            'odom_origin_yaw': yaw,
            'map_file': map_file,
        }])

    namespaced_odom_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_namespaced_odom_publisher', output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', [namespace, '/odom']],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')])

    namespaced_base_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_namespaced_base_link_publisher', output='screen',
        arguments=['0', '0', '0', '0', '0', '0', [namespace, '/base_link'], 'base_link'],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')])

    group = GroupAction([
        PushRosNamespace(namespace),
        robot_state_publisher,
        TimerAction(period=LaunchConfiguration('description_wait_s'), actions=[create_robot]),
        lidar_bridge,
        localization,
        interface_readiness,
        namespaced_odom_tf,
        namespaced_base_tf,
    ])

    return LaunchDescription(ARGUMENTS + [group])
