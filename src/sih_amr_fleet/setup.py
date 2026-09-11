from pathlib import Path
from setuptools import find_packages, setup

package_name = 'sih_amr_fleet'
package_root = Path(__file__).resolve().parent


def package_files(directory, pattern):
    return [str(path.relative_to(package_root))
            for path in (package_root / directory).glob(pattern)]


config_files = package_files('config', '*.yaml') + package_files('config', '*.xml')

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', package_files('launch', '*.launch.py')),
        ('share/' + package_name + '/config', config_files),
        ('share/' + package_name + '/maps', package_files('maps', '*')),
        ('share/' + package_name + '/scenarios', package_files('scenarios', '*')),
        ('share/' + package_name + '/models/turtlebot4_carrier', package_files('models/turtlebot4_carrier', '*')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='SIH AMR Team',
    maintainer_email='team@example.invalid',
    description='Decentralized ROS 2 coordination nodes for the SIH AMR fleet.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'localization_node = sih_amr_fleet.localization_node:main',
        'warehouse_map_node = sih_amr_fleet.warehouse_map_node:main',
        'local_costmap_node = sih_amr_fleet.local_costmap_node:main',
        'blockage_detector_node = sih_amr_fleet.blockage_detector_node:main',
        'peer_tracker_node = sih_amr_fleet.peer_tracker_node:main',
        'health_node = sih_amr_fleet.health_node:main',
        'twist_stamper_node = sih_amr_fleet.twist_stamper_node:main',
        'interface_readiness_node = sih_amr_fleet.interface_readiness_node:main',
        'charging_pad_node = sih_amr_fleet.charging_pad_node:main',
        'docking_coordinator_node = sih_amr_fleet.docking_coordinator_node:main',
        'cbba_node = sih_amr_fleet.cbba_node:main',
        'whca_planner_node = sih_amr_fleet.whca_planner_node:main',
        'reservation_manager_node = sih_amr_fleet.reservation_manager_node:main',
        'corridor_mutex_node = sih_amr_fleet.corridor_mutex_node:main',
        'path_follower_node = sih_amr_fleet.path_follower_node:main',
        'orca_node = sih_amr_fleet.orca_node:main',
        'safety_supervisor_node = sih_amr_fleet.safety_supervisor_node:main',
        'task_scenario_node = sih_amr_fleet.task_scenario_node:main',
        'random_task_generator_node = sih_amr_fleet.random_task_generator_node:main',
        'task_execution_node = sih_amr_fleet.task_execution_node:main',
        'data_collection_node = sih_amr_fleet.data_collection_node:main',
        'corridor_sweep_node = sih_amr_fleet.corridor_sweep_node:main',
        'dashboard_bridge_node = sih_amr_fleet.dashboard_bridge_node:main',
        'obstacle_spawner_node = sih_amr_fleet.obstacle_spawner_node:main',
        'fault_injector_node = sih_amr_fleet.fault_injector_node:main',
        'bounded_vision_recorder_node = sih_amr_fleet.bounded_vision_recorder_node:main',
        'robot_agent_process = sih_amr_fleet.robot_agent_process:main',
        'carrier_robot_description = sih_amr_fleet.carrier_robot_description:main',
        'kinematic_carrier_node = sih_amr_fleet.kinematic_carrier_node:main',
    ]},
)

