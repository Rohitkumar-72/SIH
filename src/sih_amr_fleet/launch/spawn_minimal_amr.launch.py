"""Spawn a lightweight TurtleBot 4 AMR in an existing Gazebo world.

This intentionally does not include the vendor's camera, point-cloud, cliff,
IR, HMI, hazard, UI, or Create 3 auxiliary launch stacks.  It is the stable
base for multi-AMR fleet work in a resource-constrained VM.
"""

from ament_index_python.packages import get_package_share_directory
from irobot_create_common_bringup.namespace import GetNamespacedName
from irobot_create_common_bringup.offset import (
    OffsetParser, RotationalOffsetX, RotationalOffsetY)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
    RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace


ARGUMENTS = [
    DeclareLaunchArgument('namespace', description='Unique robot namespace.'),
    DeclareLaunchArgument('model', default_value='lite', choices=['standard', 'lite'],
                          description='Use lite for four-AMR baseline tests.'),
    DeclareLaunchArgument('x', description='Robot world x position.'),
    DeclareLaunchArgument('y', description='Robot world y position.'),
    DeclareLaunchArgument('z', default_value='0.03', description='Robot world z position.'),
    DeclareLaunchArgument('yaw', default_value='0.0', description='Robot world yaw in radians.'),
    DeclareLaunchArgument('world', default_value='default',
                          description='Gazebo world name used by the LiDAR bridge.'),
    DeclareLaunchArgument('spawn_dock', default_value='false', choices=['true', 'false'],
                          description='Also spawn the vendor dock; disabled for the fleet baseline.'),
    DeclareLaunchArgument('keep_sensors_system', default_value='false', choices=['true', 'false'],
                          description='Keep a model-local Sensors system (normally false: the warehouse loads one world-level system).'),
    DeclareLaunchArgument('sensor_profile', default_value='fleet', choices=['fleet', 'full'],
                          description='Fleet keeps navigation LiDAR/contact only; full preserves every vendor sensor.'),
    DeclareLaunchArgument('lidar_update_rate_hz', default_value='5.0',
                          description='Navigation LiDAR rate for the fleet sensor profile.'),
    DeclareLaunchArgument('description_wait_s', default_value='5.0',
                          description='Delay insertion so the transient robot description is available.'),
    DeclareLaunchArgument('controller_wait_s', default_value='10.0',
                          description='Delay controller spawners until Gazebo has created the model.'),
]


def generate_launch_description():
    turtlebot_description = get_package_share_directory('turtlebot4_description')
    create_bringup = get_package_share_directory('irobot_create_common_bringup')
    fleet_share = get_package_share_directory('sih_amr_fleet')

    namespace = LaunchConfiguration('namespace')
    model = LaunchConfiguration('model')
    x, y, z, yaw = (LaunchConfiguration(name) for name in ('x', 'y', 'z', 'yaw'))
    world = LaunchConfiguration('world')
    keep_sensors_system = LaunchConfiguration('keep_sensors_system')
    sensor_profile = LaunchConfiguration('sensor_profile')
    lidar_update_rate = LaunchConfiguration('lidar_update_rate_hz')
    robot_name = GetNamespacedName(namespace, 'turtlebot4')
    dock_name = GetNamespacedName(namespace, 'standard_dock')

    x_dock = OffsetParser(x, RotationalOffsetX(0.157, yaw))
    y_dock = OffsetParser(y, RotationalOffsetY(0.157, yaw))
    z_robot = OffsetParser(z, -0.0025)
    yaw_dock = OffsetParser(yaw, 3.1416)
    xacro_file = PathJoinSubstitution(
        [turtlebot_description, 'urdf', model, 'turtlebot4.urdf.xacro'])
    # Project-owned limits keep every simulated fleet member on the same
    # 6.0 m/s baseline ceiling, rather than silently using the vendor's
    # 0.46 m/s configuration.
    control_params = PathJoinSubstitution([fleet_share, 'config', 'fleet_fast_control.yaml'])
    dock_description_launch = PathJoinSubstitution(
        [create_bringup, 'launch', 'dock_description.launch.py'])

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command([
                'python3', ' ', '-m', ' ',
                'sih_amr_fleet.fleet_robot_description', ' ', xacro_file, ' ',
                'gazebo:=ignition', ' ', 'namespace:=', namespace, ' ',
                '--keep-sensors-system', ' ', keep_sensors_system, ' ',
                '--sensor-profile', ' ', sensor_profile, ' ',
                '--lidar-update-rate', ' ', lidar_update_rate]),
        }],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')])

    create_robot = Node(
        package='ros_gz_sim', executable='create', name='create_robot', output='screen',
        arguments=['-name', robot_name, '-x', x, '-y', y, '-z', z_robot,
                   '-Y', yaw, '-topic', 'robot_description'])
    create_dock = Node(
        package='ros_gz_sim', executable='create', name='create_dock', output='screen',
        arguments=['-name', dock_name, '-x', x_dock, '-y', y_dock, '-z', z,
                   '-Y', yaw_dock, '-topic', 'standard_dock_description'])

    dock_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([dock_description_launch]),
        launch_arguments={'gazebo': 'ignition'}.items(),
        condition=IfCondition(LaunchConfiguration('spawn_dock')))

    lidar_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='lidar_bridge',
        output='screen', parameters=[{'use_sim_time': True}],
        arguments=[['/world/', world, '/model/', robot_name,
                    '/link/rplidar_link/sensor/rplidar/scan'
                    '@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan']],
        remappings=[
            (['/world/', world, '/model/', robot_name,
              '/link/rplidar_link/sensor/rplidar/scan'], 'scan')])

    # gz_ros2_control is embedded in the model SDF.  These two vendor controller
    # definitions merely activate the already-created controller manager.
    joint_state_spawner = Node(
        package='controller_manager', executable='spawner', name='joint_state_spawner',
        output='screen', parameters=[control_params],
        arguments=['joint_state_broadcaster', '-c', 'controller_manager',
                   '--controller-manager-timeout', '90'])
    diffdrive_spawner = Node(
        package='controller_manager', executable='spawner', name='diffdrive_spawner',
        # OnProcessExit actions are evaluated outside the enclosing GroupAction.
        # Give this spawner its namespace explicitly so it calls this robot's
        # controller manager, never the global /controller_manager service.
        namespace=namespace, output='screen', parameters=[control_params],
        arguments=['diffdrive_controller', '-c', 'controller_manager',
                   '--controller-manager-timeout', '90'])
    start_diffdrive_after_joint_state = RegisterEventHandler(
        OnProcessExit(target_action=joint_state_spawner, on_exit=[diffdrive_spawner]))

    # The fleet's public command contract is Twist; the Gazebo controller expects
    # TwistStamped.  The adapter is deliberately the only control-side helper.
    twist_stamper = Node(
        package='sih_amr_fleet', executable='twist_stamper_node', name='twist_stamper',
        output='screen', parameters=[{'use_sim_time': True}])

    interface_readiness = Node(
        package='sih_amr_fleet', executable='interface_readiness_node',
        name='interface_readiness', output='screen',
        parameters=[{'use_sim_time': True}])

    # Localization is part of robot bring-up, not delayed fleet orchestration.
    # This means it subscribes before the controller starts publishing odometry
    # and retains a valid local state for the later planning stack.
    localization = Node(
        package='sih_amr_fleet', executable='localization_node',
        name='localization_node', output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_id': namespace,
            'odom_origin_x': x,
            'odom_origin_y': y,
            'odom_origin_yaw': yaw,
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
        dock_description,
        TimerAction(period=LaunchConfiguration('description_wait_s'), actions=[create_robot]),
        TimerAction(period=LaunchConfiguration('description_wait_s'), actions=[create_dock],
                    condition=IfCondition(LaunchConfiguration('spawn_dock'))),
        lidar_bridge,
        twist_stamper,
        localization,
        interface_readiness,
        namespaced_odom_tf,
        namespaced_base_tf,
        TimerAction(period=LaunchConfiguration('controller_wait_s'),
                    actions=[joint_state_spawner]),
        start_diffdrive_after_joint_state,
    ])

    return LaunchDescription(ARGUMENTS + [group])
