from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os


def robot_group(robot_id, map_file, pad_pose):
    # Keep odometry's local origin aligned with each robot's default dock spawn.
    params = {
        'robot_id': robot_id,
        'map_file': map_file,
        'pad_x': pad_pose[0], 'pad_y': pad_pose[1], 'pad_yaw': -1.5708,
        'odom_origin_x': pad_pose[0], 'odom_origin_y': pad_pose[1],
        'odom_origin_yaw': -1.5708,
    }
    nodes = [
        'localization_node', 'local_costmap_node', 'blockage_detector_node', 'peer_tracker_node',
        'health_node', 'charging_pad_node', 'cbba_node', 'whca_planner_node', 'reservation_manager_node',
        'corridor_mutex_node', 'path_follower_node', 'orca_node', 'safety_supervisor_node', 'task_execution_node',
    ]
    return GroupAction([PushRosNamespace(robot_id)] + [Node(package='sih_amr_fleet', executable=name, name=name, parameters=[params], output='screen') for name in nodes])


def generate_launch_description():
    share = get_package_share_directory('sih_amr_fleet')
    default_map = os.path.join(share, 'maps', 'demo_warehouse.yaml')
    default_scenario = os.path.join(share, 'scenarios', 'demo_tasks.yaml')
    map_file = LaunchConfiguration('map_file')
    return LaunchDescription([
        DeclareLaunchArgument('map_file', default_value=default_map),
        DeclareLaunchArgument('scenario_file', default_value=default_scenario),
        DeclareLaunchArgument('random_tasks', default_value='false', description='Generate random shelf-aisle tasks instead of the fixed scenario.'),
        DeclareLaunchArgument('record_data', default_value='false', description='Write JSONL fleet telemetry; never affects control.'),
        DeclareLaunchArgument('data_file', default_value='/tmp/sih_amr_fleet_telemetry.jsonl'),
        Node(package='sih_amr_fleet', executable='warehouse_map_node', name='warehouse_map_node', parameters=[{'map_file': map_file}], output='screen'),
        robot_group('robot_1', map_file, (-3.6, -29.55)),
        robot_group('robot_2', map_file, (-1.2, -29.55)),
        robot_group('robot_3', map_file, (1.2, -29.55)),
        robot_group('robot_4', map_file, (3.6, -29.55)),
        Node(package='sih_amr_fleet', executable='task_scenario_node', name='task_scenario_node', parameters=[{'scenario_file': LaunchConfiguration('scenario_file')}], condition=UnlessCondition(LaunchConfiguration('random_tasks')), output='screen'),
        Node(package='sih_amr_fleet', executable='random_task_generator_node', name='random_task_generator_node', condition=IfCondition(LaunchConfiguration('random_tasks')), output='screen'),
        Node(package='sih_amr_fleet', executable='data_collection_node', name='data_collection_node', parameters=[{'output_file': LaunchConfiguration('data_file')}], condition=IfCondition(LaunchConfiguration('record_data')), output='screen'),
        Node(package='sih_amr_fleet', executable='dashboard_bridge_node', name='dashboard_bridge_node', output='screen'),
    ])
