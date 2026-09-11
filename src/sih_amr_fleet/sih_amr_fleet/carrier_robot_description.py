"""Render a lightweight TurtleBot 4 LiDAR carrier description for Gazebo ray-tracing.

Replaces wheel physics, casters, and ros2_control with a pure rigid carrier
containing base_link, circular collision footprint, visual body, and the exact
RPLIDAR link and GPU sensor mount.
"""

import argparse
import sys


def render_carrier_urdf(namespace: str = 'robot_1', lidar_update_rate: float = 10.0) -> str:
    """Generate URDF string for the kinematic LiDAR carrier."""
    return f"""<?xml version="1.0" ?>
<robot name="turtlebot4">
  <!-- Base Link with authentic TurtleBot visuals and circular collision cylinder -->
  <link name="base_link">
    <!-- Visual body from Create 3 -->
    <visual>
      <origin xyz="0 0 0.0392" rpy="0 0 1.5707963267948966"/>
      <geometry>
        <mesh filename="package://irobot_create_description/meshes/body_visual.dae"/>
      </geometry>
    </visual>
    <!-- Visual front bumper -->
    <visual>
      <origin xyz="0 0 0.0392" rpy="0 0 1.5707963267948966"/>
      <geometry>
        <mesh filename="package://irobot_create_description/meshes/bumper_visual.dae"/>
      </geometry>
    </visual>
    <!-- TurtleBot 4 top shell visual -->
    <visual>
      <origin xyz="0 0 0.08" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://turtlebot4_description/meshes/shell.dae"/>
      </geometry>
    </visual>
    <!-- Simple circular collision envelope (radius 0.17m, height 0.10m) -->
    <collision name="carrier_base_collision">
      <origin xyz="0 0 0.06" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.17" length="0.10"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0.06" rpy="0 0 0"/>
      <mass value="2.5"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.035"/>
    </inertial>
  </link>

  <!-- Gazebo properties for base_link -->
  <gazebo reference="base_link">
    <gravity>false</gravity>
    <visual>
      <material>
        <diffuse>0.12 0.12 0.12 1.0</diffuse>
        <specular>0.4 0.4 0.4 1.0</specular>
        <emissive>0.0 0.0 0.0 1.0</emissive>
      </material>
    </visual>
  </gazebo>

  <!-- Fixed RPLIDAR mount matching physical TurtleBot 4 placement -->
  <joint name="rplidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="rplidar_link"/>
    <origin xyz="0.00393584 0.0 0.13949272" rpy="0 0 1.5707963267948966"/>
  </joint>

  <link name="rplidar_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://turtlebot4_description/meshes/rplidar.dae" scale="1 1 1"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0.0 0.013 -0.019" rpy="0 0 0"/>
      <geometry>
        <box size="0.071 0.1 0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.17"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.00019" ixy="0" ixz="0" iyy="0.00012" iyz="0" izz="0.00021"/>
    </inertial>
  </link>

  <gazebo reference="rplidar_joint">
    <preserveFixedJoint>true</preserveFixedJoint>
  </gazebo>

  <gazebo reference="rplidar_link">
    <gravity>false</gravity>
    <sensor name="rplidar" type="gpu_lidar">
      <update_rate>{lidar_update_rate}</update_rate>
      <visualize>false</visualize>
      <always_on>true</always_on>
      <lidar>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1.0</resolution>
            <min_angle>-3.141592653589793</min_angle>
            <max_angle>3.141592653589793</max_angle>
          </horizontal>
          <vertical>
            <samples>1</samples>
            <resolution>1</resolution>
            <min_angle>0</min_angle>
            <max_angle>0</max_angle>
          </vertical>
        </scan>
        <range>
          <min>0.164</min>
          <max>12.0</max>
          <resolution>0.01</resolution>
        </range>
      </lidar>
    </sensor>
  </gazebo>

  <link name="base_footprint"/>
  <joint name="base_footprint_joint" type="fixed">
    <parent link="base_link"/>
    <child link="base_footprint"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
</robot>
"""


def main():
    parser = argparse.ArgumentParser(description='Render TurtleBot 4 LiDAR carrier URDF')
    parser.add_argument('--namespace', default='robot_1', help='Robot namespace')
    parser.add_argument('--lidar-update-rate', type=float, default=10.0, help='LiDAR update rate in Hz')
    args, _ = parser.parse_known_args()
    urdf = render_carrier_urdf(namespace=args.namespace, lidar_update_rate=args.lidar_update_rate)
    sys.stdout.write(urdf)


if __name__ == '__main__':
    main()
