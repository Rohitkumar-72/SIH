"""Render a TurtleBot description safe for a shared Gazebo Sensors system."""

import subprocess
import sys
import xml.etree.ElementTree as ElementTree

from ament_index_python.packages import get_package_share_directory

SENSORS_PLUGIN = 'libgz-sim-sensors-system.so'
CONTROL_PLUGIN = 'gz_ros2_control::GazeboSimROS2ControlPlugin'
FLEET_SENSOR_NAMES = {'rplidar', 'bumper_contact_sensor'}


def main() -> None:
    """Run xacro, removing the per-model Sensors system plugin.

    The TurtleBot 4 xacro adds Gazebo's ``Sensors`` system to every robot.
    That system is world-scoped; another instance tries to recreate the whole
    scene and crashes Harmonic's renderer.  The warehouse owns the single
    Sensors system, so fleet robots omit the duplicate model plugin.
    """
    keep_sensors = False
    sensor_profile = 'fleet'
    lidar_update_rate = 20.0
    arguments = []
    argument_iterator = iter(sys.argv[1:])
    for argument in argument_iterator:
        if argument == '--keep-sensors-system':
            keep_sensors = next(argument_iterator).lower() == 'true'
        elif argument == '--sensor-profile':
            sensor_profile = next(argument_iterator).lower()
        elif argument == '--lidar-update-rate':
            lidar_update_rate = float(next(argument_iterator))
        else:
            arguments.append(argument)
    if sensor_profile not in ('fleet', 'full'):
        raise ValueError(f'Unknown sensor profile: {sensor_profile}')
    if not 1.0 <= lidar_update_rate <= 62.0:
        raise ValueError('LiDAR update rate must be between 1 and 62 Hz')
    result = subprocess.run(
        ['xacro', *arguments], text=True,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if result.returncode:
        sys.stderr.write(result.stderr)
        raise SystemExit(result.returncode)
    root = ElementTree.fromstring(result.stdout)
    # The vendor xacro hard-codes its 0.46 m/s controller YAML. Replace that
    # plugin parameter with the project-owned 6.0 m/s Gazebo fleet profile
    # before the robot description reaches Gazebo.
    control_file = str(
        get_package_share_directory('sih_amr_fleet') +
        '/config/fleet_fast_control.yaml')
    for plugin in root.iter('plugin'):
        if plugin.get('name') == CONTROL_PLUGIN:
            parameters = plugin.find('parameters')
            if parameters is None:
                raise RuntimeError('gz_ros2_control plugin has no parameters element')
            parameters.text = control_file
    if not keep_sensors:
        for gazebo in list(root.findall('gazebo')):
            if any(plugin.get('filename') == SENSORS_PLUGIN
                   for plugin in gazebo.findall('plugin')):
                root.remove(gazebo)
    if sensor_profile == 'fleet':
        # The vendor Lite description contains eleven cliff/IR GPU lidars and a
        # 30 Hz RGB-D camera in addition to the navigation lidar.  None is
        # consumed by this deterministic no-vision fleet, yet four robots made
        # Gazebo render 44 GPU lidars at 62 Hz plus four cameras.  Keep only the
        # navigation lidar and inexpensive contact sensor.
        for parent in root.iter():
            for sensor in list(parent.findall('sensor')):
                if sensor.get('name') not in FLEET_SENSOR_NAMES:
                    parent.remove(sensor)
                    continue
                if sensor.get('name') == 'rplidar':
                    update_rate = sensor.find('update_rate')
                    if update_rate is not None:
                        update_rate.text = str(lidar_update_rate)
                    visualize = sensor.find('visualize')
                    if visualize is not None:
                        visualize.text = 'false'
    sys.stdout.write(ElementTree.tostring(root, encoding='unicode'))


if __name__ == '__main__':
    main()
