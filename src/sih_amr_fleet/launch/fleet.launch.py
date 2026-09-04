from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os


def robot_group(robot_id, map_file):
    params = {'robot_id': robot_id, 'map_file': map_file}
    nodes = [
        'localization_node', 'local_costmap_node', 'blockage_detector_node', 'peer_tracker_node',
        'health_node', 'cbba_node', 'whca_planner_node', 'reservation_manager_node',
        'corridor_mutex_node', 'path_follower_node', 'orca_node', 'safety_supervisor_node',
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
        robot_group('robot_1', map_file), robot_group('robot_2', map_file), robot_group('robot_3', map_file),
        Node(package='sih_amr_fleet', executable='task_scenario_node', name='task_scenario_node', parameters=[{'scenario_file': LaunchConfiguration('scenario_file')}], output='screen'),
        Node(package='sih_amr_fleet', executable='dashboard_bridge_node', name='dashboard_bridge_node', output='screen'),
    ])
