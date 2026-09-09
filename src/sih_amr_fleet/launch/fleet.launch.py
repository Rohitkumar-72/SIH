from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def robot_group(robot_id, map_file, pad_pose, tracking_speed, reservation_slot):
    # Keep odometry's local origin aligned with each robot's default dock spawn.
    params = {
        'robot_id': robot_id,
        'map_file': map_file,
        'use_sim_time': True,
        # Requested Gazebo baseline speed.  This profile is simulation-only;
        # physical AMRs must use their measured safe limits.
        'max_speed_mps': 6.0,
        'nominal_speed_mps': tracking_speed,
        # Keep the drivetrain's requested simulation ceiling, but track the
        # rolling 0.5 m WHCA* grid at a speed that can stop before a corner or
        # task station.  The previous 6 m/s tracking command advanced an
        # entire 12-cell planning horizon between 1 Hz replans.
        'path_tracking_speed_mps': tracking_speed,
        'time_slot_seconds': reservation_slot,
        'linear_kp': 12.0,
        # Gazebo stress-test braking profile. A real AMR must use its measured
        # braking model, not these deliberately permissive values.
        'braking_deceleration_mps2': 60.0,
        'braking_margin_m': 0.10,
        # Conservative 0.70 m collision envelope, verified against each
        # shelf bounding box in the locked warehouse-layout YAML.
        'robot_radius_m': 0.35,
        # TurtleBot 4 Lite vendor URDF: rplidar_link is +pi/2 about base_link.
        'lidar_x_in_base_m': 0.00393584,
        'lidar_y_in_base_m': 0.0,
        'lidar_yaw_in_base_rad': 1.5707963267948966,
        'pad_x': pad_pose[0], 'pad_y': pad_pose[1], 'pad_yaw': -1.5708,
        'dock_id': f'charging_pad_{robot_id.rsplit("_", 1)[-1]}',
        'odom_origin_x': pad_pose[0], 'odom_origin_y': pad_pose[1],
        'odom_origin_yaw': -1.5708,
    }
    nodes = [
        'local_costmap_node', 'blockage_detector_node', 'peer_tracker_node',
        'health_node', 'charging_pad_node', 'docking_coordinator_node', 'cbba_node', 'whca_planner_node', 'reservation_manager_node',
        'corridor_mutex_node', 'path_follower_node', 'orca_node', 'safety_supervisor_node', 'task_execution_node',
    ]
    return GroupAction([PushRosNamespace(robot_id)] + [Node(package='sih_amr_fleet', executable=name, name=name, parameters=[params], output='screen') for name in nodes])


def generate_launch_description():
    share = get_package_share_directory('sih_amr_fleet')
    default_map = os.path.join(share, 'maps', 'demo_warehouse.yaml')
    default_scenario = os.path.join(share, 'scenarios', 'demo_tasks.yaml')
    map_file = LaunchConfiguration('map_file')
    random_tasks = LaunchConfiguration('random_tasks')
    tracking_speed = LaunchConfiguration('path_tracking_speed_mps')
    reservation_slot = LaunchConfiguration('reservation_time_slot_s')
    tracking_speed_value = ParameterValue(tracking_speed, value_type=float)
    reservation_slot_value = ParameterValue(reservation_slot, value_type=float)
    return LaunchDescription([
        DeclareLaunchArgument('map_file', default_value=default_map),
        DeclareLaunchArgument('scenario_file', default_value=default_scenario),
        DeclareLaunchArgument('random_tasks', default_value='false', description='Generate random shelf-aisle tasks instead of the fixed scenario.'),
        DeclareLaunchArgument('record_data', default_value='false', description='Write JSONL fleet telemetry; never affects control.'),
        DeclareLaunchArgument('data_file', default_value='/tmp/sih_amr_fleet_telemetry.jsonl'),
        DeclareLaunchArgument('path_tracking_speed_mps', default_value='1.0',
                              description='Fleet route-tracking speed; 1.0 is the real-world-like baseline.'),
        DeclareLaunchArgument('reservation_time_slot_s', default_value='0.0',
                              description='WHCA* seconds/cell; 0 derives it from grid resolution and speed.'),
        Node(package='sih_amr_fleet', executable='warehouse_map_node', name='warehouse_map_node', parameters=[{'map_file': map_file, 'use_sim_time': True}], output='screen'),
        robot_group('robot_1', map_file, (-3.6, -29.55), tracking_speed_value, reservation_slot_value),
        robot_group('robot_2', map_file, (-1.2, -29.55), tracking_speed_value, reservation_slot_value),
        robot_group('robot_3', map_file, (1.2, -29.55), tracking_speed_value, reservation_slot_value),
        robot_group('robot_4', map_file, (3.6, -29.55), tracking_speed_value, reservation_slot_value),
        Node(package='sih_amr_fleet', executable='task_scenario_node', name='task_scenario_node', parameters=[{'scenario_file': LaunchConfiguration('scenario_file'), 'use_sim_time': True}], condition=UnlessCondition(LaunchConfiguration('random_tasks')), output='screen'),
        Node(package='sih_amr_fleet', executable='random_task_generator_node', name='random_task_generator_node',
             parameters=[{'use_sim_time': True}], condition=IfCondition(random_tasks), output='screen'),
        Node(package='sih_amr_fleet', executable='data_collection_node', name='data_collection_node', parameters=[{'output_file': LaunchConfiguration('data_file'), 'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('record_data')), output='screen'),
        Node(package='sih_amr_fleet', executable='dashboard_bridge_node', name='dashboard_bridge_node', parameters=[{'use_sim_time': True}], output='screen'),
    ])
