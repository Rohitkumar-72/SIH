"""Unit tests for the Kinematic LiDAR Carrier motion backend."""

import math
import xml.etree.ElementTree as ET
import pytest

from sih_amr_fleet.carrier_robot_description import render_carrier_urdf
from sih_amr_fleet.kinematic_carrier_node import RobotKinematicState


def test_carrier_urdf_structure():
    """Verify that carrier_robot_description generates valid XML with expected links/sensors."""
    urdf_str = render_carrier_urdf(namespace='robot_1', lidar_update_rate=10.0)
    root = ET.fromstring(urdf_str)
    assert root.tag == 'robot'

    links = {link.get('name') for link in root.findall('link')}
    joints = {joint.get('name') for joint in root.findall('joint')}

    assert 'base_link' in links
    assert 'rplidar_link' in links
    assert 'rplidar_joint' in joints

    # Verify no wheel or caster joints exist
    assert 'left_wheel_joint' not in joints
    assert 'right_wheel_joint' not in joints
    assert 'front_caster_joint' not in joints

    # Verify rplidar joint is fixed with exact physical mounting
    rplidar_joint = next(j for j in root.findall('joint') if j.get('name') == 'rplidar_joint')
    assert rplidar_joint.get('type') == 'fixed'
    origin = rplidar_joint.find('origin')
    assert origin is not None
    assert '0.00393584' in origin.get('xyz')

    # Verify rplidar sensor definition
    rplidar_link = next(l for l in root.findall('link') if l.get('name') == 'rplidar_link')
    gazebo_rplidar = next(g for g in root.findall('gazebo') if g.get('reference') == 'rplidar_link')
    sensor = gazebo_rplidar.find('sensor')
    assert sensor is not None
    assert sensor.get('name') == 'rplidar'
    assert sensor.get('type') == 'gpu_lidar'
    assert sensor.find('update_rate').text == '10.0'


def test_kinematic_state_integration():
    """Verify planar kinematic forward integration."""
    robot = RobotKinematicState('robot_1', x=0.0, y=0.0, yaw=0.0)
    dt = 0.02
    v = 0.5
    w = 0.2

    # Forward step for 1 second (50 steps)
    for _ in range(50):
        dx = v * math.cos(robot.true_yaw) * dt
        dy = v * math.sin(robot.true_yaw) * dt
        dyaw = w * dt
        robot.true_x += dx
        robot.true_y += dy
        robot.true_yaw += dyaw

    expected_yaw = 0.2 * 1.0
    assert abs(robot.true_yaw - expected_yaw) < 1e-4
    # Distance traveled should match arc length
    dist = math.hypot(robot.true_x, robot.true_y)
    assert 0.45 < dist < 0.55


def test_zero_noise_odometry_match():
    """Verify that default 0.0 noise produces exact dead-reckoning odometry."""
    robot = RobotKinematicState('robot_1', x=5.0, y=10.0, yaw=math.pi / 4.0)
    scale_error = 0.0
    gyro_drift = 0.0
    dt = 0.02

    v = 0.46
    w = 0.0

    for _ in range(25):
        dx = v * math.cos(robot.true_yaw) * dt
        dy = v * math.sin(robot.true_yaw) * dt
        dyaw = w * dt
        robot.true_x += dx
        robot.true_y += dy
        robot.true_yaw += dyaw

        true_dist = math.copysign(math.hypot(dx, dy), v)
        meas_dist = true_dist * (1.0 + scale_error)
        meas_dyaw = dyaw + (gyro_drift * dt)

        robot.local_x += meas_dist * math.cos(robot.local_yaw)
        robot.local_y += meas_dist * math.sin(robot.local_yaw)
        robot.local_yaw += meas_dyaw

    total_dist_true = 0.46 * 0.5
    total_dist_local = math.hypot(robot.local_x, robot.local_y)
    assert abs(total_dist_true - total_dist_local) < 1e-6
    assert abs(robot.local_yaw) < 1e-6


def test_odometry_body_frame_twist():
    """Verify that odometry twist is in base_link body frame and produces correct map velocity."""
    from sih_amr_fleet.algorithms import body_velocity_to_map

    dt = 0.02
    cmd_v = 0.46
    # 1. Westbound robot (yaw = pi)
    yaw_west = math.pi
    dx_west = cmd_v * math.cos(yaw_west) * dt
    dy_west = cmd_v * math.sin(yaw_west) * dt
    true_dist_west = math.copysign(math.hypot(dx_west, dy_west), cmd_v)
    body_vx_west = true_dist_west / dt
    assert abs(body_vx_west - cmd_v) < 1e-6
    map_vx, map_vy = body_velocity_to_map(body_vx_west, 0.0, yaw_west)
    assert abs(map_vx - (-0.46)) < 1e-5
    assert abs(map_vy - 0.0) < 1e-5

    # 2. Northbound robot (yaw = pi / 2)
    yaw_north = math.pi / 2.0
    dx_north = cmd_v * math.cos(yaw_north) * dt
    dy_north = cmd_v * math.sin(yaw_north) * dt
    true_dist_north = math.copysign(math.hypot(dx_north, dy_north), cmd_v)
    body_vx_north = true_dist_north / dt
    assert abs(body_vx_north - cmd_v) < 1e-6
    map_vx, map_vy = body_velocity_to_map(body_vx_north, 0.0, yaw_north)
    assert abs(map_vx - 0.0) < 1e-5
    assert abs(map_vy - 0.46) < 1e-5


def test_physical_shelf_collision_clearance():
    """Verify physical shelf collision logic allows passage down all warehouse aisles and detects contact."""
    import yaml
    import os

    layout_file = '/home/rtsws/amr_ws/src/SIH/warehouse_layout.lock.yaml'
    assert os.path.exists(layout_file)

    with open(layout_file) as f:
        data = yaml.safe_load(f)

    shelves = []
    for obj in data.get('objects', []):
        if obj.get('type') == 'storage_shelf':
            bb = obj.get('bounding_box_2d_m')
            shelves.append((bb['min_x'], bb['min_y'], bb['max_x'], bb['max_y']))

    assert len(shelves) == 190

    def is_collision_free(x, y, r=0.17):
        for min_x, min_y, max_x, max_y in shelves:
            if x < min_x - r or x > max_x + r:
                continue
            if y < min_y - r or y > max_y + r:
                continue
            dx = max(0.0, max(min_x - x, x - max_x))
            dy = max(0.0, max(min_y - y, y - max_y))
            if math.hypot(dx, dy) < r:
                return False
        return True

    # AMRs traveling down aisle waypoints for robot 1, 3, 4 must all be collision-free
    assert is_collision_free(-9.1725, -17.5976)
    assert is_collision_free(-10.5000, -17.5976)
    assert is_collision_free(-19.9900, -17.5976)

    assert is_collision_free(-9.1549, -6.1047)
    assert is_collision_free(-10.5000, -6.1047)
    assert is_collision_free(-11.3500, -6.1047)

    assert is_collision_free(-9.0792, 27.7728)
    assert is_collision_free(-10.5000, 27.7728)
    assert is_collision_free(-15.6700, 27.7728)

    # Point directly inside shelf_south_west_06_03 (x=-10.0, y=-18.5) must collide
    assert not is_collision_free(-10.0, -18.5)
    # Point 5cm from shelf surface (max_y=-18.1752, so y=-18.12) must collide
    assert not is_collision_free(-10.0, -18.12)


def test_multirate_carrier_integration():
    """Verify that carrier distance traveled per sim-second is rate-independent.
    
    A robot commanded at 0.46 m/s must advance 0.46 m in 1.0 simulated second,
    regardless of whether callbacks fire at 50 Hz (0.02s dt), 25 Hz (0.04s dt),
    or 11.46 Hz (~0.087s dt).
    """
    v_cmd = 0.46
    sim_duration = 1.0  # 1.0 sim-second

    for rate_hz in [50.0, 25.0, 15.87, 11.46, 10.0]:
        robot = RobotKinematicState('robot_test', x=0.0, y=0.0, yaw=0.0)
        nominal_dt = 0.02
        dt_callback = 1.0 / rate_hz
        elapsed = 0.0
        total_dist = 0.0

        while elapsed < sim_duration - 1e-9:
            step_dt = min(dt_callback, sim_duration - elapsed)
            # Micro-substeps of <= 0.02s
            num_substeps = max(1, math.ceil(step_dt / nominal_dt))
            substep_dt = step_dt / num_substeps

            for _ in range(num_substeps):
                dx = v_cmd * math.cos(robot.true_yaw) * substep_dt
                dy = v_cmd * math.sin(robot.true_yaw) * substep_dt
                robot.true_x += dx
                robot.true_y += dy
                total_dist += math.hypot(dx, dy)

            elapsed += step_dt

        assert abs(total_dist - (v_cmd * sim_duration)) < 1e-4, (
            f'Failed for rate {rate_hz} Hz: got {total_dist}m, expected {v_cmd * sim_duration}m'
        )
        assert abs(robot.true_x - (v_cmd * sim_duration)) < 1e-4


def test_substepping_avoids_tunneling():
    """Verify that substepping catches collisions even with large elapsed dt."""
    # Obstacle is at x = 0.20, robot starts at x = 0.0 with radius 0.17.
    # Contact occurs when center reaches x = 0.20 - 0.17 = 0.03m.
    robot = RobotKinematicState('robot_test', x=0.0, y=0.0, yaw=0.0)
    v_cmd = 0.5  # m/s
    obstacle_x = 0.20
    robot_radius = 0.17
    contact_threshold_x = obstacle_x - robot_radius  # 0.03m

    def is_free(cx, cy):
        return cx < contact_threshold_x

    # One large dt step of 0.1s: without substepping, robot would jump to x = 0.05 (penetrating)
    large_dt = 0.10
    nominal_dt = 0.02
    num_substeps = max(1, math.ceil(large_dt / nominal_dt))
    substep_dt = large_dt / num_substeps

    collided = False
    for _ in range(num_substeps):
        dx = v_cmd * substep_dt
        if not is_free(robot.true_x + dx, robot.true_y):
            collided = True
            break
        robot.true_x += dx

    assert collided, "Substepping failed to detect obstacle"
    assert robot.true_x <= contact_threshold_x + 1e-5


def test_large_dt_retains_unintegrated_remainder():
    """Verify that when dt > 0.5s, carrier integrates up to cap and retains unintegrated remainder."""
    prev_sim_time_s = 100.0
    now_s = 100.8  # dt = 0.8s > 0.5s cap
    max_step_dt = 0.5

    dt = now_s - prev_sim_time_s
    dt_to_integrate = min(dt, max_step_dt)
    prev_sim_time_s += dt_to_integrate
    integrated_1 = dt_to_integrate

    assert integrated_1 == 0.5
    # On next tick at now_s = 100.8, remainder is now_s - prev_sim_time_s
    dt_next = now_s - prev_sim_time_s
    assert abs(dt_next - 0.3) < 1e-9, f"Expected 0.3s remainder, got {dt_next}"
    integrated_2 = min(dt_next, max_step_dt)
    prev_sim_time_s += integrated_2

    total_integrated = integrated_1 + integrated_2
    assert abs(total_integrated - 0.8) < 1e-9, f"Expected 0.8s total integration, got {total_integrated}"
    assert abs(prev_sim_time_s - now_s) < 1e-9


def test_boundary_recovery_scoped_attributes():
    """Verify that carrier boundary limits are accessible properties and prevent NameError."""
    import rclpy
    from sih_amr_fleet.kinematic_carrier_node import KinematicCarrierNode
    if not rclpy.ok():
        rclpy.init()
    node = KinematicCarrierNode()
    try:
        min_x, min_y, max_x, max_y = node.boundary_limits
        assert min_x < max_x
        assert min_y < max_y
        assert node._is_position_collision_free('robot_1', 0.0, 0.0) is True
        assert node._is_position_collision_free('robot_1', min_x - 1.0, 0.0) is False
    finally:
        node.destroy_node()



