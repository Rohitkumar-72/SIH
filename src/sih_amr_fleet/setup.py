from glob import glob
from setuptools import find_packages, setup

package_name = 'sih_amr_fleet'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/maps', glob('maps/*')),
        ('share/' + package_name + '/scenarios', glob('scenarios/*')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='SIH AMR Team',
    maintainer_email='team@example.invalid',
    description='Decentralized ROS 2 coordination nodes for the SIH AMR fleet.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'localization_node = sih_amr_fleet.localization_node:main',
        'local_costmap_node = sih_amr_fleet.local_costmap_node:main',
        'blockage_detector_node = sih_amr_fleet.blockage_detector_node:main',
        'peer_tracker_node = sih_amr_fleet.peer_tracker_node:main',
        'health_node = sih_amr_fleet.health_node:main',
        'twist_stamper_node = sih_amr_fleet.twist_stamper_node:main',
        'interface_readiness_node = sih_amr_fleet.interface_readiness_node:main',
        'charging_pad_node = sih_amr_fleet.charging_pad_node:main',
        'cbba_node = sih_amr_fleet.cbba_node:main',
        'whca_planner_node = sih_amr_fleet.whca_planner_node:main',
        'reservation_manager_node = sih_amr_fleet.reservation_manager_node:main',
        'corridor_mutex_node = sih_amr_fleet.corridor_mutex_node:main',
        'path_follower_node = sih_amr_fleet.path_follower_node:main',
        'orca_node = sih_amr_fleet.orca_node:main',
        'safety_supervisor_node = sih_amr_fleet.safety_supervisor_node:main',
        'task_scenario_node = sih_amr_fleet.task_scenario_node:main',
        'dashboard_bridge_node = sih_amr_fleet.dashboard_bridge_node:main',
    ]},
)
