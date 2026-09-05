# ROS 2 fleet implementation

This repository now contains a ROS 2 Jazzy overlay workspace at `src/`.  It is deliberately separate from the legacy AWS warehouse package described in `agents.md`; the warehouse remains a Gazebo resource, while this overlay contains the AMR coordination software.

## Control path

```text
Gazebo /odom + /scan
  -> localization + local costmap + blockage detector
  -> peer tracker + CBBA + WHCA* + reservation manager + corridor mutex
  -> path follower -> reciprocal velocity avoidance -> Safety Supervisor
  -> /robot_N/cmd_vel -> Gazebo AMR drive plugin
```

The dashboard bridge only publishes `/fleet/dashboard_telemetry`; it has no command, service, or action that can control a robot.

## Packages

| Package | Purpose |
|---|---|
| `sih_amr_interfaces` | Stable ROS message contracts for fleet state, tasks, reservations, corridor protocol, safety, and telemetry events. |
| `sih_amr_fleet` | Python `rclpy` nodes, launch file, a small deterministic grid map, task scenario, and pure algorithm tests. |

## Nodes per robot

| Node | Inputs | Outputs |
|---|---|---|
| `localization_node` | `odom` | `state`, `/fleet/robot_state` |
| `local_costmap_node` | `scan` | `local_costmap`, `nearest_obstacle_m` |
| `blockage_detector_node` | `state`, `local_costmap` | `/fleet/blockage_observation` |
| `peer_tracker_node` | `/fleet/robot_state` | `peer_tracks` |
| `health_node` | `peer_tracks`, `/fleet/safety_state` | `/fleet/health` |
| `charging_pad_node` | `odom` | `charging/is_docked`, `charging/battery_percent`, `charging/battery_state` |
| `cbba_node` | task announcements, consensus, state | task consensus, `task_assignment` |
| `whca_planner_node` | state, assignment, intents, blockages | `planned_route` |
| `reservation_manager_node` | `planned_route` | `/fleet/trajectory_intent` |
| `corridor_mutex_node` | corridor protocol, health, local clearance | corridor protocol, `corridor_motion_allowed` |
| `path_follower_node` | state, route, corridor permission | `cmd_vel_desired` |
| `orca_node` | desired velocity, peer tracks | `cmd_vel_candidate` |
| `safety_supervisor_node` | scan, state, candidate velocity, E-stop | `/fleet/safety_state`, `entrance_clear`, `cmd_vel` |

`task_scenario_node` publishes reproducible task arrivals only. It is not a fleet manager. `dashboard_bridge_node` is read-only telemetry only.

## Fleet-level message rules

All coordination messages include `robot_id`, a boot-time random `session_id`, monotonically increasing `sequence_no`, `sent_at`, and `valid_until`. Receivers reject expired data and out-of-order state from the same session. A changed session is treated as a new boot instance.

| Topic | QoS | Safety/consistency rule |
|---|---|---|
| `/fleet/robot_state` | reliable current state; local `state` is best-effort latest-only | Fresh state only; tracker grows uncertainty after loss. |
| `/fleet/trajectory_intent` | reliable, bounded, transient-local | Replaced on `plan_id`; expires quickly. |
| `/fleet/corridor_protocol` | reliable, bounded | REQUEST/GRANT/DEFER/ENTER/EXIT events are not state inference. |
| `/fleet/task_consensus` | reliable, bounded | Assignment epoch + owner session + lease reject stale ownership. |
| `/fleet/blockage_observation` | reliable, bounded | Sensor-derived only; expires unless re-observed. |
| `/fleet/health` | reliable low-rate | Used for active participants; not physical occupancy proof. |

## Build in the Ubuntu VM

Copy this repository into `~/amr_ws/src/SIH` (alongside, not inside, `warehouse_world`). Then:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/amr_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select sih_amr_interfaces sih_amr_fleet
source install/setup.bash
pytest src/SIH/src/sih_amr_fleet/test
```

Launch the fleet stack after three Gazebo AMRs exist with matching namespaces and their drive plugins expose `/robot_1/cmd_vel`, `/robot_1/odom`, `/robot_1/scan` (and equivalently for robots 2 and 3):

```bash
ros2 launch sih_amr_fleet fleet.launch.py
```

For a clean warehouse world without an embedded AMR, use this project launch after
starting Gazebo in another terminal:

```bash
ros2 launch sih_amr_fleet spawn_robot_1.launch.py namespace:=robot_1 x:=2.0 y:=2.0 z:=0.05 yaw:=0.0
```

`charging_pad_node` uses the current pad pose (`x=0.513707`, `y=-9.859080`,
`yaw=1.5708`) by default. It only raises its project-owned battery estimate after
the robot has remained aligned, stationary, and inside the docking zone for two
seconds. It rejects stale odometry, so charging stops when Gazebo is paused or
the robot's odometry stream disappears. It deliberately does not overwrite TurtleBot's simulator-owned
`/robot_N/battery_state`; use `/robot_N/charging/battery_state` for the charging
simulation.

The launch does not start `gz sim` or spawn robot models. That keeps it compatible with the current legacy warehouse resource workflow. The next Gazebo task is to add three differential-drive AMR models with LiDAR and then start the existing warehouse command from `agents.md` in a separate terminal.

## Scope and important limitations

- `whca_planner_node` is a working rolling grid planner using peer trajectory reservations. Replace `maps/demo_warehouse.yaml` with an occupancy grid exported from the final warehouse before benchmarking.
- `orca_node` is a lightweight reciprocal velocity-obstacle approximation, not a full external ORCA library. It is intentionally followed by the authoritative sensor-based Safety Supervisor.
- The CBBA node is a bounded one-task CBAA/CBBA stepping stone. It implements task bids, winner claims, leases, epochs, and stale-session protection; queued task bundles and reallocation policy are the next extension.
- `corridor_mutex_node` reads named corridor cells from the map YAML and automatically requests the first corridor on an upcoming route; a test can also inject a namespaced `request_corridor` string. It emits EXIT only after measured local pose enters and then leaves the configured region. A lease timeout must become `SUSPECT_OCCUPIED`/blocked, never physically free.
- ML/DL integration is intentionally absent. Later models can publish a route-cost estimate consumed as a soft CBBA/WHCA* cost, never a `cmd_vel` input.
