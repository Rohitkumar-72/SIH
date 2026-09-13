# Master Forensic Analysis & Fleet Restoration Report

> **Document Status**: Active Master Reference  
> **Workspace**: `/home/rtsws/amr_ws/src/SIH`  
> **Timestamp**: 2026-09-13  
> **Target Task**: Decentralized Autonomous Mobile Robot (AMR) Warehouse Fleet Simulation (`sih_amr_fleet`)

---

## 1. Executive Summary & Context Analysis

### 1.1 The Operational Problem
The decentralized AMR simulation running 4 differential-drive robots (`robot_1` to `robot_4`) in a simulated warehouse (Gazebo Sim Harmonic, ROS 2 Jazzy, 50 Hz physics, RTF ~3.5x–3.8x) exhibited two distinct failure phases:
1. **Initial Global Slowdown (~28–33% speed loss)**:
   The kinematic carrier integrated motion with a static assumption $\Delta t = 1/50\text{ s}$ rather than measuring actual simulation time. Under CPU load or thread contention, callbacks executed at ~11.5 Hz instead of 15.8 Hz, causing the robots to travel ~30% less distance per simulation second.
2. **"Fast Start Then Sudden Fall-Off" (Fleet Gridlocks)**:
   In subsequent runs where other processes were closed, the fleet started moving very rapidly (covering >200 m across the fleet in the first 90–120 seconds), but then abruptly ground to a near-total halt, completing few or no further tasks until the simulation was manually stopped.

---

## 2. Forensic Investigation of Benchmark Runs

### 2.1 Benchmark Run Comparison Matrix

| Metric | Fast Baseline (`192735`) | Run 002437 | Run 012023 ("Fall Off 1") | Run 014605 ("Fall Off 2 - Latest") |
| :--- | :--- | :--- | :--- | :--- |
| **Observation Wall Time** | 1013 s | 812 s (aborted) | 884 s (aborted) | 474 s (aborted) |
| **Sim Time Window** | 0 – 3800 s | 0 – 3050 s | 1500 – 4300 s | 1500 – 3315 s |
| **Tasks Completed** | **20 / 20** | 6 / 20 | 6 / 20 | 6 / 20 (Tasks 1, 2, 3, 4, 5, 6) |
| **Distance Covered (Fleet)** | ~1850 m | ~620 m | ~610 m | **753.1 m** |
| **Initial 120s Motion** | ~250 m | ~180 m | ~287 m | **246.3 m** |
| **Mid-Run Motion (200s window)** | ~380 m | ~45 m (stalled) | 0.8 m (frozen) | 90.5 m (stalled) |
| **Root Failure Mode** | None (baseline) | WHCA Route Infeasibility | Boundary Lock & 1D VO Freeze | Corridor Hold Standoff & ORCA Push |

---

### 2.2 Deep Forensics: Run `desktop_data_collection_20260913_012023`

In this run, the fleet demonstrated the exact "started quickly then fell off" behavior:
- **Phase 1 (Rapid Motion: $t = 1600\text{ s} - 2000\text{ s}$)**:
  - Fleet distance covered was **148.1 m** between 1600–1800s, and **139.2 m** between 1800–2000s.
  - Robots were running at the full target tracking speed ($0.46\text{ m/s}$).
- **Phase 2 (The Perimeter Boundary Lock at $t = 1926.4\text{ s}$)**:
  - `robot_2` was following a route along the west perimeter corridor where waypoints lie at $x = -22.50\text{ m}$.
  - Tracking chatter and differential-drive yaw adjustments caused `robot_2` to drift to $x = -22.56\text{ m}$.
  - In `kinematic_carrier_node.py`, the boundary bounding box was defined as `min_x = -22.55 m`.
  - At $x = -22.56\text{ m}$, `_is_position_collision_free` evaluated to `False`.
  - In the carrier's swept-footprint subdivision:
    ```python
    for s in range(1, substeps + 1):
        fraction = s / substeps
        cx = robot.true_x + fraction * dx
        cy = robot.true_y + fraction * dy
        if not self._is_position_collision_free(robot_id, cx, cy):
            collision = True
            allowed_ratio = max(0.0, (s - 1) / substeps)
            break
    ```
    At $s = 1$, `cx` was already $< -22.55\text{ m}$. The loop evaluated `collision = True`, setting `allowed_ratio = (1 - 1) / substeps = 0.0`.
  - **Result**: `robot_2` was 100% physically locked at $(-22.35, 10.55)$ for the remaining 2374 simulation seconds of the run.
- **Phase 3 (The 1D Velocity Obstacle Hallucination at $t \approx 3200\text{ s} - 3800\text{ s}$)**:
  - `robot_1` was at $(7.08, 2.04)$ traveling West with $v_x = 0.46\text{ m/s}$.
  - `robot_4` was at $(5.84, 0.55)$ traveling East with $v_x = 0.46\text{ m/s}$.
  - Lateral offset between their parallel paths was $\Delta y = 1.49\text{ m}$ (and lateral separation $\Delta x = 1.24\text{ m}$), which provides ample room for two $0.28\text{ m}$ radius robots to pass each other with $> 0.6\text{ m}$ clearance.
  - However, `avoidance_velocity()` in `algorithms.py` used a 1D closing-speed approximation:
    ```python
    closing = ((vx - peer['vx']) * dx + (vy - peer['vy']) * dy) / separation
    predicted_distance = separation - closing * horizon
    if predicted_distance < 2.0 * effective_radius and closing > 0.0:
        push = (2.0 * effective_radius - predicted_distance) / max(horizon, 0.1)
        vx -= push * dx / separation
        vy -= push * dy / separation
    ```
    Because both robots were closing rapidly along the relative distance vector ($\text{closing} \approx 0.90\text{ m/s}$), the 1D equation subtracted $0.90 \times 1.5 = 1.35\text{ m}$ from the 2D Euclidean separation, predicting an imminent head-on collision despite the large lateral miss distance!
  - Both `robot_1` and `robot_4` had their forward velocities pushed backwards. Because reverse motion is clamped (`max(0.0, vx)`), **both robots simultaneously clamped their speeds to $0.00\text{ m/s}$**.
  - `robot_3` approached behind `robot_1` and queued. Total fleet motion dropped to **0.8 m over a 200-second window**.

---

### 2.3 Deep Forensics: Run `desktop_data_collection_20260913_014605` (Latest Run)

After deploying the 2D CPA algorithm and the carrier boundary recovery margin, the user executed a new benchmark run.

#### Positive Outcomes Observed:
1. **Zero Boundary Traps**: `robot_2` smoothly navigated the entire perimeter corridor without any boundary freezing.
2. **Zero Reverse Seesawing**: Not a single negative velocity oscillation event occurred.
3. **High Initial Throughput**:
   - In the first 90 wall seconds (338 simulation seconds), the fleet moved **215.0 meters** across all 4 robots.
   - `rnd_task_001` and `rnd_task_002` completed quickly.
4. **Final Task Yield**: Completed **6 tasks** (`rnd_task_001` through `rnd_task_006`) and covered **753.1 meters** before manual abort at 474 wall seconds.

#### Why Did the Run Still Experience a Severe Mid-Run Stall?
A precise timeline of fleet telemetry reveals a dramatic 180-wall-second stall between Wall 90s and Wall 270s (Simulation seconds 1840s to 2674s):

```
Wall:    0.0s | Sim:    0.0s | Fleet dist:    0.0m | R1:   0.0m | R2:   0.0m | R3:   0.0m | R4:   0.0m
Wall:   30.0s | Sim:  112.5s | Fleet dist:   61.2m | R1:  23.7m | R2:  12.5m | R3:  17.9m | R4:   7.1m
Wall:   60.2s | Sim:  225.9s | Fleet dist:  135.9m | R1:  50.8m | R2:  31.3m | R3:  24.9m | R4:  28.9m
Wall:   90.2s | Sim:  338.9s | Fleet dist:  215.0m | R1:  66.2m | R2:  59.3m | R3:  40.2m | R4:  49.3m
-------------------------------------------------------------------------------------------------------
Wall:  120.3s | Sim:  455.3s | Fleet dist:  246.3m | R1:  78.4m | R2:  78.4m | R3:  40.2m | R4:  49.3m
Wall:  150.3s | Sim:  568.5s | Fleet dist:  267.7m | R1:  78.6m | R2:  99.6m | R3:  40.2m | R4:  49.3m
Wall:  180.4s | Sim:  686.3s | Fleet dist:  286.5m | R1:  78.7m | R2: 118.3m | R3:  40.2m | R4:  49.3m
Wall:  210.4s | Sim:  803.4s | Fleet dist:  303.8m | R1:  78.7m | R2: 135.5m | R3:  40.2m | R4:  49.3m
Wall:  240.4s | Sim:  920.7s | Fleet dist:  322.3m | R1:  78.7m | R2: 154.0m | R3:  40.2m | R4:  49.3m
Wall:  270.5s | Sim: 1034.2s | Fleet dist:  336.8m | R1:  78.8m | R2: 168.5m | R3:  40.2m | R4:  49.3m
-------------------------------------------------------------------------------------------------------
Wall:  300.5s | Sim: 1148.7s | Fleet dist:  397.9m | R1:  94.5m | R2: 188.0m | R3:  45.6m | R4:  69.8m
Wall:  330.6s | Sim: 1264.8s | Fleet dist:  474.6m | R1: 113.3m | R2: 207.2m | R3:  65.4m | R4:  88.7m
Wall:  451.0s | Sim: 1724.3s | Fleet dist:  753.1m | R1: 189.0m | R2: 267.7m | R3: 131.1m | R4: 165.3m
```

During this 700 simulation-second span:
- **`robot_3` moved exactly $0.0\text{ m}$** (frozen at $40.2\text{ m}$).
- **`robot_4` moved exactly $0.0\text{ m}$** (frozen at $49.3\text{ m}$).
- **`robot_1` moved only $12\text{ m}$** before stopping.
- Only `robot_2` was active on the other side of the warehouse.

#### Exact Mechanisms of the Mid-Run Stall:

1. **The 831-Second Corridor Mutex Lock**:
   - At Sim $t = 1842.4\text{ s}$, `robot_3` requested token for narrow corridor `NC-MIDDLE-WEST-01`.
   - At Sim $t = 1843.4\text{ s}$, `corridor_mutex_node` granted the token (`event = 3`).
   - At the exact entrance to that corridor, `robot_3` encountered `robot_1` (`dist = 1.83 m`).
   - Because `robot_1` has higher priority (`'robot_1' < 'robot_3'`), `robot_3` entered ORCA yield state:
     ```
     [1842.3s] [robot_3:ORCA] Escalation: STANDOFF_PERSISTS. Actor=ORCA:robot_3.
     Yielding to robot_1 for 8.2s without clearance. Maintaining stop to prevent collision.
     ```
   - In accordance with safety rules, `robot_3` held its stop: `result.linear.x = 0.0`.
   - However, because `robot_3` was stopped at the entrance, it **never crossed through the corridor**, and therefore **never published corridor release (`event = 4`)**.
   - `robot_3` held `NC-MIDDLE-WEST-01` exclusively locked from **$1843.4\text{ s}$ to $2674.4\text{ s}$ (831 simulation seconds)**.
   - `robot_4` planned a path requiring `NC-MIDDLE-WEST-01` and was stopped in place by `HOLD_CORRIDOR_BLOCKED`.
   - `robot_1` was obstructed in front of `robot_3`.

2. **ORCA Velocity Synthesizing on Desired Zero Speed**:
   - In `orca_node.py`, when a robot desired $v_x = 0.0\text{ m/s}$ (e.g. `PathFollower` turning in place because $|\Delta \theta| > 0.40\text{ rad}$ or holding for a blocked corridor), `preferred` was set to $(0.0, 0.0)$.
   - `avoidance_velocity` computed repulsive vectors from nearby peers and pushed away:
     ```python
     vx -= push * push_dir_x
     vy -= push * push_dir_y
     ```
   - In `orca_node.py`, this resulted in:
     `result.linear.x = vx * cos(theta) + vy * sin(theta)`
     `if self.desired.linear.x >= 0.0: result.linear.x = max(0.0, result.linear.x)`
   - Telemetry captured in `fleet.log`:
     ```
     [robot_1:ORCA] Decision: AVOID_PEER_ADJUSTMENT. Actor=ORCA:robot_1.
     Info: desired_vx=0.00 -> adjusted_vx=0.93, active_peers_tracked=3.
     ```
   - **Consequence**: When `robot_1` was stopped and attempting to rotate in place to clear out of the aisle, ORCA generated a forward translation velocity of $+0.93\text{ m/s}$! This caused the AMR to translate forward while turning, creating erratic positional drift and delaying its exit from `robot_3`'s path.

Once `robot_1` finally wandered away at Sim $2674\text{ s}$, `robot_3` detected clearance, moved through `NC-MIDDLE-WEST-01`, and released the corridor token (`[2674.4s] robot_3 NC-MIDDLE-WEST-01 event=4`). Immediately, the fleet un-froze and rapidly finished tasks 3, 4, 5, and 6.

---

## 3. Comprehensive Inventory of Actions Taken

### 3.1 Code Changes Applied to Date

| # | File | Component | Detailed Modification |
| :--- | :--- | :--- | :--- |
| **1** | [`kinematic_carrier_node.py`](file:///home/rtsws/amr_ws/src/SIH/src/sih_amr_fleet/sih_amr_fleet/kinematic_carrier_node.py) | Simulation-Time Integration | Replaced fixed $\Delta t = 0.02\text{ s}$ with actual elapsed simulation time ($\Delta t = t_{\text{sim}} - t_{\text{prev}}$). Added micro-substepping ($\le 0.02\text{ s}$) with swept-footprint checks to eliminate host-load slowdown and tunneling. |
| **2** | [`kinematic_carrier_node.py`](file:///home/rtsws/amr_ws/src/SIH/src/sih_amr_fleet/sih_amr_fleet/kinematic_carrier_node.py) | Perimeter Margin & Escape | Expanded perimeter boundary margin from $0.05\text{ m}$ to $0.35\text{ m}$ (`min_x = -22.85 m`). Added inward-escape allowance so AMRs slightly outside bounds can step inward back to safety. |
| **3** | [`algorithms.py`](file:///home/rtsws/amr_ws/src/SIH/src/sih_amr_fleet/sih_amr_fleet/algorithms.py) | 2D Velocity Obstacle (CPA) | Replaced 1D Euclidean closing calculation with true 2D Closest Point of Approach ($t_{\text{cpa}} = (\mathbf{p} \cdot \mathbf{v}) / \|\mathbf{v}\|^2$). Parallel lanes with lateral offset $d_{\text{cpa}} \ge 2R$ no longer hallucinate collisions. |
| **4** | [`orca_node.py`](file:///home/rtsws/amr_ws/src/SIH/src/sih_amr_fleet/sih_amr_fleet/orca_node.py) | Geometric Clearance | Removed arbitrary 3.0s timeout and 5.0s cooldown. Yield latch is maintained until peer passes behind ($longitudinal < -r$) or clears laterally ($> 2r$). |
| **5** | [`orca_node.py`](file:///home/rtsws/amr_ws/src/SIH/src/sih_amr_fleet/sih_amr_fleet/orca_node.py) | Radius Normalization | Changed default `robot_radius_m` from $0.35\text{ m}$ to $0.28\text{ m}$ (consistent with physical chassis radius and grid clearance). |
| **6** | [`whca_planner_node.py`](file:///home/rtsws/amr_ws/src/SIH/src/sih_amr_fleet/sih_amr_fleet/whca_planner_node.py) | Entrance Gate Routing | When a goal is inside a mutexed corridor occupied by a peer, planner sets `effective_goal` to the closest entrance cell outside the corridor, avoiding `ROUTE_INFEASIBLE` search failure. |
| **7** | [`path_follower_node.py`](file:///home/rtsws/amr_ws/src/SIH/src/sih_amr_fleet/sih_amr_fleet/path_follower_node.py) | Turn Latching | Added directional turn latching (`_turn_latch`) for $|\text{raw\_error}| > 2.8\text{ rad}$ to eliminate $\pm \pi$ heading limit cycles. |
| **8** | [`scripts/launch_fleet_amrs.sh`](file:///home/rtsws/amr_ws/src/SIH/scripts/launch_fleet_amrs.sh) | Build Environment Clean | Removed obsolete `$SIH_ROOT/install/setup.bash` sourcing that shadowed current workspace builds. |

---

### 3.2 Unit Test Verification Suite

The repository now contains 99 deterministic unit tests covering all algorithm invariants:
- **`test_algorithms.py`**: 75 tests passing (including 2D CPA parallel pass, CBBA auction immutability, WHCA* reservation avoidance, braking safe speeds).
- **`test_kinematic_carrier.py`**: 7 tests passing (multi-rate integration invariance from 10 to 50 Hz, anti-tunneling micro-substeps, body-frame odometry).
- **`test_orca_encounters.py`**: 4 deterministic multi-robot encounter tests passing (head-on priority yielding, standoff escalation without forward collision, lateral offset bypass, three-robot priority hierarchy).
- **`test_runner_metrics.py`**: 13 tests passing.

---

## 4. Remaining Root Causes & Definite Solution Plan

To eliminate the mid-run stalls and achieve 20/20 completed tasks in $\le 1000\text{ s}$, the following two architectural refinements are required:

### 4.1 Suppress ORCA Forward Acceleration on Zero Desired Speed
- **Issue**: When `PathFollower` commands $v_x = 0.0$ (during in-place rotation $\omega_z \ne 0$, or while waiting at a corridor gate), ORCA repulsive forces project onto the robot's heading and push the robot forward at up to $0.93\text{ m/s}$.
- **Fix**: In [`orca_node.py:control()`](file:///home/rtsws/amr_ws/src/SIH/src/sih_amr_fleet/sih_amr_fleet/orca_node.py#L59), ORCA must only **reduce** desired forward velocity, never increase it:
  ```python
  if self.desired.linear.x <= 0.01:
      result.linear.x = 0.0
  else:
      result.linear.x = min(self.desired.linear.x, max(0.0, result.linear.x))
  ```

### 4.2 Corridor Mutex Standoff Release Protocol
- **Issue**: When an AMR holds a corridor token but is stopped outside the entrance yielding to another AMR, it holds the token indefinitely (831 seconds in run 014605), causing fleet-wide gridlock.
- **Fix**: In [`corridor_mutex_node.py`](file:///home/rtsws/amr_ws/src/SIH/src/sih_amr_fleet/sih_amr_fleet/corridor_mutex_node.py) or `task_executor_node.py`:
  - If a robot holding a corridor token has not entered the corridor cells or made forward progress within $10.0\text{ s}$ due to an ORCA yield/standoff, it must yield the corridor token (`RELEASE`) so queued AMRs can proceed or replan.

### 4.3 Retain Valid Route on Transient Infeasibility
- **Issue**: In [`path_follower_node.py:on_route()`](file:///home/rtsws/amr_ws/src/SIH/src/sih_amr_fleet/sih_amr_fleet/path_follower_node.py#L67), setting `self.route = None` on transient `ROUTE_INFEASIBLE` causes the robot to stop dead, preventing it from clearing intersections.
- **Fix**: Retain the existing `self.route` for up to $3.0\text{ s}$ when replanning fails, allowing the robot to continue traversing its current clear segment.

---

## 5. Verification Command Reference

```bash
# 1. Run unit test suite
source /opt/ros/jazzy/setup.bash
source /home/rtsws/amr_ws/install/setup.bash
pytest src/sih_amr_fleet/test/

# 2. Build package
colcon build --packages-select sih_amr_fleet

# 3. Analyze any benchmark run directory
python3 -c "
import glob, json, os
rdir = sorted(glob.glob('/home/rtsws/amr_ws/log/desktop_data_collection_*/desktop_run_*'))[-1]
print('Analyzing:', os.path.basename(rdir))
# Parse run summary and task events
"
```
