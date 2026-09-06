"""Render a TurtleBot description safe for a shared Gazebo Sensors system."""

import subprocess
import sys
import xml.etree.ElementTree as ElementTree


SENSORS_PLUGIN = 'libgz-sim-sensors-system.so'


def main() -> None:
    """Run xacro, removing the per-model Sensors system plugin.

    The TurtleBot 4 xacro adds Gazebo's ``Sensors`` system to every robot.
    That system is world-scoped; a second instance tries to recreate the whole
    scene and crashes Harmonic's renderer.  The first robot retains it and all
    later robots retain their sensor SDF but omit the duplicate system plugin.
    """
    keep_sensors = False
    arguments = []
    argument_iterator = iter(sys.argv[1:])
    for argument in argument_iterator:
        if argument == '--keep-sensors-system':
            keep_sensors = next(argument_iterator).lower() == 'true'
        else:
            arguments.append(argument)
    result = subprocess.run(
        ['xacro', *arguments], text=True,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if result.returncode:
        sys.stderr.write(result.stderr)
        raise SystemExit(result.returncode)
    root = ElementTree.fromstring(result.stdout)
    if not keep_sensors:
        for gazebo in list(root.findall('gazebo')):
            if any(plugin.get('filename') == SENSORS_PLUGIN
                   for plugin in gazebo.findall('plugin')):
                root.remove(gazebo)
    sys.stdout.write(ElementTree.tostring(root, encoding='unicode'))


if __name__ == '__main__':
    main()
