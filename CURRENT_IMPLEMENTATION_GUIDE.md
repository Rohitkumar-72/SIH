# Current Fleet Implementation Guide

This is the source of truth for code currently in this repository. It is deliberately stricter than the architecture documents: **implemented** means code exists; **verified** means it ran successfully in the latest test; **planned** must not be presented as complete.

## Status

The ROS overlay builds and 11 pure algorithm/lane tests pass. The latest
headless Gazebo run successfully started the warehouse and four Lite AMRs;
each passed a gate requiring actual odometry and LiDAR samples. A full-stack
run recorded all four robots on `/fleet/robot_state` and showed CBBA converge
to one owner for each announced random task, with only the owner accepting the
first task. The previous robot-1-only status symptom was caused by duplicate
model Sensors plugins plus Fast DDS asynchronous publication overload.

The deterministic baseline is **not accepted as a delivery benchmark yet**.
The task-execution/path-following hand-off still needs an extended successful
pickup/dropoff run, collision metrics, and live network/corridor fault
injection before making that claim. The launcher is a working four-robot
bring-up and allocation test.

The command limit is **6.0 m/s in Gazebo only**. It is not a physical-robot recommendation.

## End-to-end flow

```text
Gazebo odom + LiDAR
  -> localization -> local state + fleet state
  -> costmap -> blockage detector -> shared blockage observations

task generator/scenario -> shared task announcement -> CBBA claims
  -> local task assignment -> task executor -> execution status
  -> WHCA* route -> reservation manager -> shared trajectory intents
  -> corridor mutex -> path follower -> ORCA -> Safety Supervisor
  -> cmd_vel -> Twist-to-TwistStamped adapter -> differential-drive controller

peer tracker -> health; charging pad -> health
dashboard and logger only observe fleet topics.
```

Each robot has a namespace such as `/robot_1`; shared coordination topics start with `/fleet/`. No fleet-manager node exists. Spawn locations are loaded by `scripts/launch_four_amrs.sh` from `~/.config/sih_amr_poses.env`; the canonical checked-in mirror is `warehouse_layout.lock.yaml` at the repository root.

## Nodes and topics

| Node | Inputs | Outputs | Current responsibility |
|---|---|---|---|
| `localization_node` | `/robot_N/odom` | `/robot_N/state`, `/fleet/robot_state`, `amcl_pose` | Transforms simulator odometry to map-relative fleet state. `amcl_pose` is compatibility output, not AMCL. |
| `local_costmap_node` | `scan` | `local_costmap`, `nearest_obstacle_m` | Robot-frame LiDAR occupancy grid. |
| `blockage_detector_node` | local state/costmap | `/fleet/blockage_observation` | Five-frame persistent LiDAR blockage report. |
| `peer_tracker_node` | `/fleet/robot_state` | `peer_tracks` | Per-peer constant-velocity/Kalman-style prediction. |
| `health_node` | peer tracks, safety, charge percent | `/fleet/health`, `status` | Battery, local safety, and communication freshness. |
| `cbba_node` | task announcement/consensus, local state, execution | `/fleet/task_consensus`, local `task_assignment` | Bidding, claim convergence, assignment. |
| `task_execution_node` | assignment, local state | `/fleet/task_execution_status` | Pickup/dropoff and dwell state machine. |
| `whca_planner_node` | state, assignment, execution, intents, blockages | `planned_route`, `path` | Rolling cooperative grid path. |
| `reservation_manager_node` | planned route | `/fleet/trajectory_intent` | Expiring cell/time reservations. |
| `corridor_mutex_node` | route, health, local state, clearance, protocol | `/fleet/corridor_protocol`, `corridor_motion_allowed` | Distributed narrow-corridor permission. |
| `path_follower_node` | state, route, corridor permission | `cmd_vel_desired` | Waypoint velocity controller. |
| `orca_node` | state, desired velocity, peer tracks | `cmd_vel_candidate` | Local reciprocal avoidance approximation. |
| `safety_supervisor_node` | state, scan, candidate velocity, E-stop | `cmd_vel`, `/fleet/safety_state`, `entrance_clear` | Final stop/slow authority. |
| `charging_pad_node` | odometry | `charging/*` | Dock detection and project-owned battery estimate. |
| `dashboard_bridge_node` | fleet state/health/task/corridor/safety | `/fleet/dashboard_telemetry` | Read-only JSON dashboard stream. |
| `data_collection_node` | fleet coordination/safety topics | JSONL file | Passive run recorder. |

`warehouse_map_node` publishes `/map` and `/map_metadata`. `task_scenario_node` publishes a fixed YAML scenario; `random_task_generator_node` publishes random tasks. A run uses one task source.

## Task generation, allocation, and storage

The random generator waits three seconds, then creates tasks every random 12–25 seconds (seed default `42`), up to five active tasks. Pickup and dropoff are distinct points from `warehouse_tasks.py`, which restricts them to known aisle centrelines. A `Task` carries ID, pickup/dropoff poses, priority, dwell times, creation time, and expiry time. It is sent on `/fleet/task_announcement`.

Every `cbba_node` stores two in-memory dictionaries: `tasks` and `winners`. This is local process memory, not a database; restarting a node clears it until new messages arrive. Every 0.5 seconds each robot calculates:

```text
travel_distance = distance(robot, pickup) + distance(pickup, dropoff)
base_bid        = travel_distance / max(nominal_speed_mps, 0.1)
workload        = 120 s × other tasks currently claimed by this robot
priority_bonus  = (priority / 100) × 10 s
bid             = max(1 s, base_bid + workload - priority_bonus)
```

`distance` is currently straight-line Euclidean distance(will change), not actual route length. `nominal_speed_mps` is 6.0 in the Gazebo profile. Lower bids win; robot ID breaks ties deterministically. The workload penalty favours robots without existing work, while higher priority reduces the bid. 

The robot broadcasts a `TaskConsensus` claim on `/fleet/task_consensus`: winner ID/session, bid, epoch, and 2.5 s lease. Receivers keep a higher epoch, or at the same epoch the lowest `(winning_bid, winner_robot_id)`. The winner writes a namespaced `task_assignment` only after a 2.0 s consensus collection window, preventing several robots from executing their own initial claim.

This is a bounded single-task CBAA/CBBA stepping stone, not complete bundle/queue CBBA. The executor protects one active task from being replaced by later assignments. Queued bundles, battery-aware bids, full failure reauction, and durable state storage are planned.

The task executor publishes:

```text
EN_ROUTE_PICKUP -> PICKUP_WAIT -> EN_ROUTE_DROPOFF
-> DROPOFF_WAIT -> COMPLETED
```

Arrival requires <=0.45 m position error and <=0.15 m/s speed. Each pickup/dropoff waits the task’s 2–5 s dwell time. Completion is broadcast for about one second; CBBA and the random generator then remove the task.

## Decentralisation and network behaviour

All robots use the same shared DDS topics and independently calculate bids, routes, and safety decisions. That removes a permanent fleet-manager single point of failure. Fleet headers contain robot ID, boot session ID, sequence number, send time, and validity time. The peer tracker rejects expired and old same-session robot state; a changed session is a fresh boot.

Current network limitations:

- The peer tracker reports `NORMAL` through 0.5 s, `COMM_DEGRADED` through 2.0 s, then `PEER_UNREACHABLE` and increases position uncertainty.
- CBBA observes health leases and may reauction only after an owner was observed healthy and that lease expires.
- DDS reliable delivery is not consensus. Partition/restart/fault-injection tests remain planned.

## WHCA*: path finding, sharing, and blockage

The warehouse uses a 0.5 m occupancy grid. WHCA* searches `(x_cell, y_cell, time_slot)` over a 12-slot rolling horizon. It expands north/south/east/west and wait moves. Its score is:

```text
g = elapsed grid moves
h = Manhattan distance to goal
f = g + h
```

It rejects static shelf cells, observed blockage cells, cells reserved by another robot at the same slot, and opposite-direction edge swaps. If the goal is farther than the window, it returns the reached horizon path and replans every second.

`RoutePlan` is published locally on `planned_route`; a standard `nav_msgs/Path` is also emitted. The reservation manager republishes its cell/time tuples on `/fleet/trajectory_intent` every 0.75 seconds with plan ID, 0.5 s slot duration, priority, and 1.5 s validity. Each planner imports still-valid peer intents into its next search. This provides strategic collision avoidance; it is not a physical guarantee.

LiDAR -> local costmap -> five persistent occupied frames -> `/fleet/blockage_observation`. Valid reports become planner blocked cells and force rerouting if an alternative is found. Entries are removed when their validity lease expires. Blockage-triggered task reallocation is planned.

## Narrow corridors and failure safety

Corridor regions are named map cells in `demo_warehouse.yaml`. When a route reaches a corridor, the mutex node creates a UUID and broadcasts `REQUEST` on `/fleet/corridor_protocol`.

This is a Ricart–Agrawala-style mutual exclusion protocol:

1. Requester timestamps a request with a Lamport clock.
2. Peers compare `(Lamport timestamp, robot ID)`; the earlier request receives `GRANT`, while the other is deferred.
3. Entry requires a grant from every currently healthy peer and a true `entrance_clear` signal.
4. `entrance_clear` comes from the local LiDAR/braking Safety Supervisor, so a network grant cannot override a close obstacle.
5. The requester sends `ENTER`, detects leaving corridor cells from local pose, sends `EXIT`, and releases deferred grants.

Protocol events are `REQUEST`, `GRANT`, `DEFER`, `ENTER`, `EXIT`, `RELEASE`, and `CANCEL`. This is not majority voting: it requires permission from each active peer.

If a robot that announced `ENTER` loses its health lease, the corridor is retained as `SUSPECT_OCCUPIED` and new permits are denied. An `EXIT`, `RELEASE`, or `CANCEL` clears normal occupancy; physical recovery clearance and live fault-injection tests are still required before declaring this policy validated.

## Kalman-style tracking, ORCA, and hard safety

For each peer, the tracker uses:

```text
predict: x += vx × dt; y += vy × dt; variance += process_noise × dt
update:  gain = variance / (variance + measurement_variance)
         estimate moves toward measurement; variance shrinks
```

With delayed messages, the predicted pose advances but variance grows. ORCA uses that uncertainty as radius inflation. The implemented ORCA is a lightweight reciprocal-velocity-obstacle approximation: it projects separation over a 1.5 s horizon, pushes an approaching preferred velocity away when predicted separation is too small, and clips the result to max speed. It is not a full external ORCA library.

The Safety Supervisor is authoritative. It calculates:

```text
braking_distance = speed² / (2 × deceleration) + braking_margin
```

It issues `STOP` and zero velocity for E-stop, localization older than 0.5 s, or LiDAR inside braking distance. It issues `SLOW` in the warning zone and scales linear speed to 40%. It publishes `entrance_clear` for the corridor policy. WHCA*, CBBA, ORCA, dashboard, and future ML cannot override this node.

## Charging pads

Each `charging_pad_node` transforms its local odometry to a configured warehouse pad pose. It sets docked only with fresh odometry, the robot inside the longitudinal/lateral dock bounds, heading error <=0.35 rad, speed <=0.03 m/s, and two seconds of continuous stability. When docked it raises a project-owned battery estimate at 10 percentage points/minute by default.

```text
/robot_N/charging/is_docked       Bool
/robot_N/charging/battery_percent Float32
/robot_N/charging/battery_state   BatteryState
/robot_N/charging/docking_status  String
```

`health_node` consumes charge percent. This node intentionally does not overwrite the TurtleBot simulator battery topic. Battery-aware scheduling and autonomous docking are planned.

## Dashboard, data, and deferred work

The dashboard bridge publishes JSON four times per second with latest poses, health, and last 30 task/corridor/safety events. It has no control output. The data collector writes JSONL run manifests plus state, health, consensus, execution, intents, corridor, safety, and blockage records; write failures are ignored so logging cannot affect motion.

Implemented after the first baseline review: expired LiDAR blockage observations
are now removed from the WHCA* blocked set; CBBA may take over a task whose
owner's health lease has expired; and a corridor whose announced occupant loses
health is retained as `suspect` and denied to new requests. These safeguards
build and pass the unit suite, but still need live fault-injection validation.

Excluded from this phase: learned path selection, ML confidence fallback,
learned CBBA cost, camera image bridge/recording, obstacle classification,
semantic labels, full Nav2/AMCL, and long-duration benchmark acceptance.

## Launcher

```bash
cd ~/amr_ws
colcon build --symlink-install --packages-select sih_amr_interfaces sih_amr_fleet
bash ~/amr_ws/src/SIH/scripts/run_baseline_random_fleet.sh
```

This starts Gazebo, four AMRs, the full node graph, random tasks, and passive telemetry. It is an integration launcher, not proof that delivery, collision, and fault-injection acceptance criteria have been met.
