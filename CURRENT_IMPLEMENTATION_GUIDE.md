# Current Fleet Implementation Guide

This document is the authoritative source of truth for the codebase, architecture, algorithms, and verified performance baselines in this repository.

- **Implemented**: Code exists, builds, and passes all 89 unit/integration tests.
- **Verified**: Validated against live end-to-end simulation runs in Gazebo Harmonic with empirical data logged and analyzed.
- **Standards**: Operational defaults conform to physical hardware limits (0.46 m/s standard speed, strict safety envelope).

---

## 1. Verified Baseline & Validation Results

### 1.1 Executive Summary
The fleet baseline has achieved **verified end-to-end task completion with 100% mission success**. Gazebo Harmonic functions as a high-fidelity **kinematic LiDAR renderer** while closely following the AMRs' internally estimated map positions. Heavy wheel-contact dynamics have been decoupled into a deterministic 50 Hz kinematic simulation backend, eliminating physics instability and wheel slip while preserving high-fidelity ray-traced sensing, dynamic obstacle detection, and strict safety validation.

### 1.2 End-to-End Validation Results (40/40 Tasks Complete)

Two comprehensive consecutive benchmark runs of 20 tasks each (40 tasks total) were executed under the default standard **0.46 m/s** speed profile.

| Measurement Metric | Run 1 (20 tasks) | Run 2 (20 tasks) | Combined Baseline (40 tasks) |
|---|---|---|---|
| **Completed Tasks** | **20 / 20 (100%)** | **20 / 20 (100%)** | **40 / 40 (100%)** |
| **Mean Map ↔ Gazebo Pose Error** | 1.88 cm | 1.81 cm | **1.84 cm** |
| **Maximum Pose Discrepancy** | 24.81 cm | 22.32 cm | **24.81 cm** |
| **Samples within 5 cm Error** | 97.07% | 96.77% | **96.92%** |
| **Samples within 10 cm Error** | 99.62% | 99.67% | **99.64%** |
| **Invalid Localization Samples** | 0 | 0 | **0** |
| **Robot Tilt / Tumble Incidents** | 0 | 0 | **0** |
| **Scan-Stale Safety Stops (Run 2)** | — | 0 | **0** |

### 1.3 Pose Stability & Drift Analysis
1. **Zero Accumulating Drift**: Across all **38,519 recorded pose samples**, every sample remained well within the map's 35 cm operational tolerance.
2. **Quarterly Interval Analysis**: Dividing each run into four temporal quarters, the mean discrepancy remained bounded between **1.68 cm and 2.05 cm** across all quarters.
3. **Terminal Convergence**: Final terminal discrepancies when completing deliveries were at most **1.85 cm in Run 1** and **1.12 cm in Run 2**.
4. **Transient Latency Characteristics**: The occasional peak discrepancies of 20–25 cm occur strictly as transient Gazebo pose-update transport latencies during sharp angular pivots, resolving immediately once linear translation resumes. They represent transport latency, not accumulating divergence.
5. **Sensor Alignment**: The carrier integration operates at **50 Hz** and Gazebo ray-traced LiDAR operates at **5 Hz**. Every AMR passes both odometry and scan readiness gates, ensuring Gazebo LiDAR reflects obstacle returns around the exact pose where each AMR internally believes it is located.

---

## 2. Operating Speeds & Kinematic Envelope

### 2.1 Standard Speed Standard: 0.46 m/s
The fleet standard operating speed is **0.46 m/s** (`physical_max` velocity profile). This matches the physical hardware ceiling of the TurtleBot 4 differential-drive platform.

```python
# Formal Fleet Velocity Profiles (sih_amr_fleet/velocity_profiles.py)
PROFILES = {
    "physical_fidelity": VelocityProfile(nominal=0.31, max=0.46, tier="PHYSICAL_FIDELITY", certified=True),
    "physical_max":      VelocityProfile(nominal=0.46, max=0.46, tier="PHYSICAL_FIDELITY", certified=True), # DEFAULT
    "synthetic_0_75":    VelocityProfile(nominal=0.75, max=0.75, tier="SYNTHETIC_UNCERTIFIED", certified=False),
    "synthetic_1_00":    VelocityProfile(nominal=1.00, max=1.00, tier="SYNTHETIC_UNCERTIFIED", certified=False),
}
```

### 2.2 Physical Footprint & Clearance
- **Planning Footprint**: Circular footprint with diameter **0.56 m** (radius $r = 0.28\text{ m}$).
- **Aisle Clearance**: Narrow storage aisles are 1.1554 m wide. A 0.56 m AMR has 0.2977 m side clearance on both sides, allowing safe passage and in-place turns without wall collisions.

### 2.3 Time-Space Slot Duration
WHCA* derives space-time reservation slot durations dynamically from the tracking speed:
$$\text{slot\_duration} = \frac{\text{cell\_size}}{\text{tracking\_speed}} = \frac{0.5\text{ m}}{0.46\text{ m/s}} \approx 1.087\text{ seconds}$$
This ensures the space-time reservation grid accurately represents physical space occupancy over time.

---

## 3. End-to-End System Architecture

```text
+-----------------------------------------------------------------------------------------------+
|                                    DECENTRALIZED FLEET STACK                                  |
|                                                                                               |
|  [Random Task Generator] --------> [/fleet/task_announcement]                                 |
|                                           |                                                   |
|                                           v                                                   |
|  [cbba_node] <-----------------------> Two-Phase CBBA Consensus (BID / CLAIM Quorum)          |
|        |                                                                                      |
|        v (local task_assignment)                                                              |
|  [task_execution_node] --------------> EN_ROUTE_PICKUP -> PICKUP_WAIT ->                      |
|        |                               EN_ROUTE_DROPOFF -> DROPOFF_WAIT -> COMPLETED          |
|        v                                                                                      |
|  [whca_planner_node] <---------------> Rolling Horizon Space-Time A* (12 slots)               |
|        |                               + Trajectory Intent Reservations                       |
|        v                                                                                      |
|  [corridor_mutex_node] <-------------> Ricart-Agrawala Lamport Mutex (Single-Lane Aisles)     |
|        |                                                                                      |
|        v                                                                                      |
|  [path_follower_node] ---------------> Waypoint Guidance + Deceleration Ramps                 |
|        |                                                                                      |
|        v                                                                                      |
|  [orca_node] ------------------------> Reciprocal Velocity Obstacle Local Avoidance           |
|        |                                                                                      |
|        v                                                                                      |
|  [safety_supervisor_node] -----------> Hard Braking Envelope + LiDAR Directional Guard        |
|        |                                                                                      |
|        v (safety-approved /robot_N/cmd_vel)                                                   |
+--------|--------------------------------------------------------------------------------------+
         |
         v
+-----------------------------------------------------------------------------------------------+
|                              SIMULATION BACKEND & SENSOR RENDERING                            |
|                                                                                               |
|  [kinematic_carrier_node] (50 Hz)                                                             |
|    - Kinematic forward integration of planar equations                                        |
|    - Subdivided swept-footprint circular collision check against shelves & peer AMRs          |
|    - Synthesizes /robot_N/odom (simulated wheel odometry)                                     |
|    - Simulates independent dock contact confirmation (DockProtocol.CONFIRMED)                 |
|    - Batches pose updates via gz.transport13 (/world/default/set_pose_vector)                 |
|                                                                                               |
|  [Gazebo Harmonic Server] (5 Hz LiDAR Ray-Tracing)                                            |
|    - Synchronously renders 2D LiDAR scans from carrier models                                 |
|    - Publishes ROS 2 sensor streams (/robot_N/scan)                                           |
+-----------------------------------------------------------------------------------------------+
```

---

## 4. Node Details & Topic Contracts

| Node | Input Topics | Output Topics | Responsibility |
|---|---|---|---|
| `kinematic_carrier_node` | `/{rid}/cmd_vel`, `/clock` | `/{rid}/odom`, `/fleet/dock_protocol`, `/simulation/true_pose` | 50 Hz deterministic kinematic motion integration, swept-footprint collision gate, Gazebo pose sync. |
| `localization_node` | `/{rid}/odom`, `/fleet/dock_protocol` | `/{rid}/state`, `/fleet/robot_state`, `amcl_pose` | Transforms raw odometry into global `map` frame; anchors to dock on confirmed docking. |
| `local_costmap_node` | `/{rid}/scan` | `local_costmap`, `nearest_obstacle_m` | Robot-frame 2D occupancy grid from ray-traced LiDAR returns. |
| `blockage_detector_node` | `/{rid}/state`, `local_costmap` | `/fleet/blockage_observation` | Detects persistent dynamic blockages; strips static shelves, walls, docks, and peer halos. |
| `peer_tracker_node` | `/fleet/robot_state` | `peer_tracks` | Tracks peer trajectories with constant-velocity prediction and covariance growth under packet loss. |
| `health_node` | `peer_tracks`, `safety_state`, `battery_percent` | `/fleet/health`, `status` | Heartbeat monitoring, communication lease evaluation, and battery health tracking. |
| `cbba_node` | `/fleet/task_announcement`, `/fleet/task_consensus`, local `state` | `/fleet/task_consensus`, `/{rid}/task_assignment` | Two-phase decentralized auction algorithm (BID + CLAIM quorum). |
| `task_execution_node` | `/{rid}/task_assignment`, `/{rid}/state` | `/fleet/task_execution_status` | Manages pickup/dropoff waypoint transitions and 2–5s dwell timers. |
| `whca_planner_node` | `state`, `task_assignment`, `task_execution_status`, `trajectory_intent`, `blockage_observation` | `planned_route`, `path` | 12-slot rolling horizon Space-Time A* with reverse-BFS obstacle heuristic. |
| `reservation_manager_node` | `planned_route` | `/fleet/trajectory_intent` | Broadcasts expiring space-time cell reservations to prevent multi-AMR route conflicts. |
| `corridor_mutex_node` | `planned_route`, `/fleet/corridor_protocol`, `entrance_clear` | `/fleet/corridor_protocol`, `corridor_motion_allowed` | Ricart-Agrawala distributed mutual exclusion for single-lane aisle access. |
| `path_follower_node` | `state`, `planned_route`, `corridor_motion_allowed` | `cmd_vel_desired` | Pure pursuit waypoint follower with heading alignment and deceleration ramps. |
| `orca_node` | `state`, `cmd_vel_desired`, `peer_tracks` | `cmd_vel_candidate` | Reciprocal Velocity Obstacle local collision avoidance approximation. |
| `safety_supervisor_node` | `state`, `scan`, `cmd_vel_candidate`, E-stop | `/{rid}/cmd_vel`, `/fleet/safety_state`, `entrance_clear` | Authoritative hard braking envelope, sensor-staleness guard, and emergency stop. |
| `charging_pad_node` | `/{rid}/odom` | `charging/*` | Dock alignment detection and project-owned battery charge modeling. |
| `docking_coordinator_node` | `/fleet/health`, `battery_percent` | `/fleet/dock_protocol` | Leased dock resource arbitration and recharge dispatching. |
| `data_collection_node` | All `/fleet/*` coordination topics | `fleet_telemetry.jsonl` | Non-blocking passive JSONL run recorder and metrics aggregator. |

---

## 5. Algorithmic Deep Dives

### 5.1 Kinematic LiDAR Carrier Backend (`kinematic_carrier_node.py`)
- **Planar Kinematic Integration (50 Hz)**:
  $$\Delta x = v \cos(\theta) \Delta t, \quad \Delta y = v \sin(\theta) \Delta t, \quad \Delta \theta = \omega \Delta t$$
- **Swept Footprint Collision Gating**: Subdivides motion into $\le 0.02\text{ m}$ sub-steps. Checks distance to physical shelf AABBs, boundary walls, and peer AMR circular footprints ($2 \times r_{\text{robot}}$). If a collision is predicted, translation is clamped to the safe boundary while in-place rotation is permitted.
- **Gazebo Transport Synchronization**: Dispatches batch pose vector protobuf messages (`GzPose_V`) over `gz.transport13` to `/world/{world_name}/set_pose_vector`, positioning lightweight Gazebo carrier models containing 5 Hz GPU LiDAR sensors.
- **Simulated Wheel Odometry**: Generates standard `nav_msgs/Odometry` with configurable Gaussian noise and scale error (set to 0.0 for deterministic verification).
- **Physical Dock Confirmation**: Evaluates proximity to configured dock anchors. When within $0.08\text{ m}$ distance, $0.20\text{ rad}$ heading, and stopped ($v < 0.05\text{ m/s}$), emits `DockProtocol.CONFIRMED`.

### 5.2 Decentralized Task Allocation (CBBA)
- **Bid Calculation**:
  $$\text{travel\_dist} = \|\mathbf{p}_{\text{robot}} - \mathbf{p}_{\text{pickup}}\| + \|\mathbf{p}_{\text{pickup}} - \mathbf{p}_{\text{dropoff}}\|$$
  $$\text{base\_bid} = \frac{\text{travel\_dist}}{\max(v_{\text{nominal}}, 0.1)}, \quad \text{bid} = \max(1.0, \text{base\_bid} - (\text{priority} / 100) \times 10)$$
  (Busy robots advertise $\text{bid} = \infty$).
- **Two-Phase Quorum**:
  1. **Phase 1 (BID)**: Each robot broadcasts its own `TaskConsensus.BID`.
  2. **Phase 2 (CLAIM)**: After a 2.0 s collection window, all replicas compute the minimum `(bid, winner_id)` and broadcast a matching `TaskConsensus.CLAIM`.
  3. **Commit Gate**: The winner assigns the task to its local executor only after receiving matching `CLAIM`s from all $N$ active peers.
- **Decision Immutability**: Bids and claims are frozen for the duration of the auction epoch to prevent asynchronous motion from breaking consensus quorum.

### 5.3 Space-Time Path Planning (WHCA*)
- **State Space**: Searches $(x, y, t)$ over a 12-slot horizon on a 0.5 m grid.
- **Reverse BFS Heuristic**: Precomputes all-pairs obstacle-aware distance to the goal, preventing the rolling horizon from getting trapped in concave shelf dead-ends.
- **Dynamic Intent Reservations**: Active routes publish space-time cell reservations on `/fleet/trajectory_intent`. Peers import valid intents and treat reserved cells as dynamic obstacles.
- **Semantic Blockage Filtering**: Strips static shelf outlines, walls, dock pads, and peer robot footprints from raw LiDAR returns. Only true unmodeled physical blockages trigger global WHCA* replanning.

### 5.4 Distributed Corridor Mutex (Ricart–Agrawala)
- **Resource Protection**: Narrow warehouse aisles (1.1554 m wide) only permit one AMR at a time.
- **Lamport Timestamp Ordering**: Requests are timestamped with logical clocks. Contending requests are ordered by `(timestamp, robot_id)`.
- **Grant Conditions**: Entry is permitted only when `GRANT` is received from every healthy peer AND the local Safety Supervisor confirms `entrance_clear = true`.

### 5.5 Authoritative Safety Supervisor
- **Hard Braking Envelope**:
  $$d_{\text{stop}} = \frac{v^2}{2 a_{\text{decel}}} + d_{\text{margin}}$$
  Where $a_{\text{decel}} = 1.0\text{ m/s}^2$ and $d_{\text{margin}} = 0.15\text{ m}$.
- **Preemptive Stopping**: Forces $v = 0, \omega = 0$ immediately on:
  - Emergency stop trigger.
  - LiDAR or odometry staleness ($> 0.5\text{ s}$).
  - Obstacle detected within the directional braking envelope.
  - Non-finite (NaN/Inf) velocity inputs.
- **Priority**: Operates at 40 Hz and strictly overrides all higher-level nodes (WHCA*, ORCA, path follower).

### 5.6 Docking & Battery Management (`charging_pad_node.py`, `docking_coordinator_node.py`)
- **Dock Arbitration**: Decentralized lease requests via `/fleet/dock_protocol` ordered by `(Lamport timestamp, robot_id, request_id)`. Winning AMRs acquire a renewable 4-second lease on target charging pads.
- **Precision Alignment**: `path_follower_node` aligns geometrically to the dock-centre strip at $\le 0.15\text{ m/s}$.
- **Hardware Anchor Reset**: On verified contact (`DockProtocol.CONFIRMED`), `localization_node` resets its odometry-to-map frame transform to eliminate any accumulated wheel integration error.
- **Battery Charging**: Synthesizes linear battery charging at 10 percentage points per minute while docked.

### 5.7 Telemetry & Dataset Generation (`data_collection_node.py`)
- **Telemetry Schema 0.6**: Records asynchronous JSON Lines with sub-second simulation time (`logged_at`) and host epoch time (`wall_logged_at`), enabling exact computation of achieved Real-Time Factor (RTF).
- **Recorded Event Streams**:
  - `robot_state`: Global map pose, twist, planning cell, and raw simulator odometry.
  - `task_announcement` & `task_consensus`: Bids, winners, epoch numbers, and quorum timestamps.
  - `task_execution`: Exact arrival, dwell, and completion transitions.
  - `trajectory_intent`: Expiring space-time reservations from WHCA*.
  - `corridor_event` & `safety_event`: Mutex grants, defers, directional clearance, and braking reasons.
  - `ros_log`: Persists all ROS 2 warning, error, and fatal messages directly from `/rosout`.
- **Automated CSV Dataset Export**: Converts raw JSONL telemetry into structured ML tabular datasets (`desktop_fleet_dataset.csv`) capturing distance, corridor transits, execution latency, and battery dynamics.

---

## 6. Verification and Diagnostic Tooling

### 6.1 Automated Benchmark Runner (`scripts/run_desktop_data_collection.py`)
Executes automated multi-work-cycle runs with live terminal UI, progress metrics, and dataset export:
```bash
python3 scripts/run_desktop_data_collection.py --cycles 2 --tasks-per-cycle 20 --tracking-speed 0.46
```

### 6.2 Ground-Truth Pose Verification (`scripts/verify_gazebo_pose.py`)
Validates that AMR entities in Gazebo match expected world coordinates and orientation within tolerances:
```bash
python3 scripts/verify_gazebo_pose.py --robot robot_1 --expected-x -1.0 --expected-y -26.0 --expected-yaw 1.5708
```

### 6.3 Test Suite Execution
The pure algorithmic and integration test suite contains 89 unit tests:
```bash
colcon test --packages-select sih_amr_fleet && colcon test-result --verbose
```
All 89 tests pass with zero failures and zero errors.
