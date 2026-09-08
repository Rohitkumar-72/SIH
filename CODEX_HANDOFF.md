# Codex Handoff — Physical-Style Localization, Docking, Safety Recovery, and Long-Run Validation

You are continuing the deterministic ROS 2 / Gazebo AMR fleet baseline in this repository. Do **not** implement ML/DL, learned costs, camera classification, training data pipelines, or vision models in this task.

## First actions

1. Read this file completely.
2. Read `CURRENT_IMPLEMENTATION_GUIDE.md`, `context.md`, and the project-root Markdown files that define current architecture/status. Treat `CURRENT_IMPLEMENTATION_GUIDE.md` as the stricter source of truth when documents conflict.
3. Inspect the current working tree with `git status --short`. It is intentionally dirty: preserve unrelated user changes and do not reset, clean, or overwrite them.
4. Read the implementation files named below before changing behaviour.
5. Build and test before edits to establish a baseline.

## Current verified baseline

- ROS package: `src/sih_amr_fleet`; interfaces: `src/sih_amr_interfaces`.
- Four Lite AMRs launch through `scripts/run_baseline_random_fleet.sh` / `scripts/launch_four_amrs.sh`.
- The launch gate requires real simulated `/robot_N/odom` and `/robot_N/scan` samples, not merely advertised endpoints.
- The map frame is 45 m × 60 m, origin `(-22.5, -30.0)`, resolution `0.5 m`. It is the sole planning frame; its cell conversion is documented in `CURRENT_IMPLEMENTATION_GUIDE.md`.
- In simulation, Gazebo-generated wheel odometry is adapted into `map` by `localization_node`; each robot begins aligned to a fixed dock/spawn anchor.
- `warehouse_layout.lock.yaml` is the checked-in layout lock. Keep it synchronized with `/home/rtsws/amr_ws/src/warehouse_world_custom/worlds/small_warehouse/warehouse_layout.lock.yaml` if that external world path is available and writable.
- `maps/demo_warehouse.yaml` has anchors, a graph, narrow-junction resources, and the 0.5 m grid.
- WHCA* now avoids the exact reserved cell plus its eight adjacent cells at the same time slot (`reservation_buffer_cells: 1`).
- The corridor mutex uses Ricart–Agrawala style messages for constrained narrow lanes/resources. Spacious main junctions use WHCA* reservations, not a one-robot mutex.
- `data_collection_node` is passive and writes JSONL only when `record_data:=true`. The standard random-fleet launcher enables it and writes `fleet_telemetry.jsonl` under `/home/rtsws/amr_ws/log/four_amr_runs/<run-id>/`.
- The package built successfully and `pytest -q src/sih_amr_fleet/test` passed 12 tests after the latest map/buffer change.

Do not claim that an end-to-end delivery benchmark is accepted until a robot completes pickup/dropoff motion, collision/fault cases are exercised, and evidence is present in telemetry.

## Required implementation scope

### 1. Main-corridor and dock guidance strips

Model and document painted guidance strips in the lock YAML and Gazebo world if practical:

- A green centre line in each main corridor.
- A centered red recovery line within each main corridor.
- Red branch lines from the nearby main-corridor recovery line to each dock approach, so an AMR can get close to a dock by line following.
- A dock-centre strip/alignment reference for final parking on each charging pad.
- Preserve the existing green narrow-aisle markings. Make strip IDs, colour, geometry, purpose, and map coordinates explicit in `warehouse_layout.lock.yaml`.

Do not pretend a painted visual is a physical sensor. If Gazebo line perception is not implemented, represent the lines as map/semantic references and clearly mark camera/tape detection as future hardware work.

### 2. Deterministic dock-availability and docking protocol

Implement a distributed, deterministic docking flow. It must not introduce a central fleet manager.

Required behaviour:

1. A restarting AMR, or any AMR needing a dock, publishes a dock request/intent.
2. It determines available docks from fresh, expiring fleet protocol state; simultaneous requests resolve deterministically (Lamport timestamp, then robot ID is acceptable).
3. It reserves/claims one dock before entering its approach branch. Other robots must avoid that pad/approach resource.
4. Near the dock, switch from ordinary path following to a low-speed final docking state that uses the dock-centre strip as its reference. In simulation this can be deterministic geometric alignment to the mapped strip; do not fake visual sensing.
5. The charging-pad node uses simulated odometry to determine pad occupancy/docked condition, then publishes an explicit docking confirmation.
6. Only after that confirmation may `localization_node` reset/correct the robot's map pose to the dock anchor.
7. Add leases, release/cancel paths, failure/timeout handling, and a safe retry/alternate-dock choice. The system must recover from a restart without assuming Gazebo world-pose data exists on real hardware.

You may add project-owned messages in `src/sih_amr_interfaces/msg` if existing messages cannot represent the protocol. Update CMake/package metadata and all documentation if you do.

### 3. Relative map pose and Gazebo odometry telemetry

Extend `data_collection_node` so every robot-state record includes both:

- Map-relative pose/twist used by fleet planning, including cell coordinate when available.
- Raw simulator odometry pose/twist, explicitly labeled `gazebo_odom` or `simulation_odom`.

Do not silently relabel one as the other. Define a clear interface/topic for raw odometry if needed. Retain JSONL compatibility where possible and add a `schema_version` / record field explanation to the guide.

### 4. Collision/crash observability and safe recovery

Add useful passive telemetry for collisions/crashes:

- Event time, involved robot IDs, contact counterpart/entity when available, map pose/cell, raw simulation odometry, current task/route/corridor state, and event reason/source.
- Record Gazebo process crashes and ROS node exits into the run manifest or a structured run-event stream where feasible. A shell launcher can monitor child process status, but never hide a failed process.
- Do not fabricate Gazebo contact data. If a contact sensor/bridge is needed, add it deliberately and document its topic/type.

For a confirmed collision/contact or a safety recovery trigger, implement a conservative recovery state machine:

1. Immediately stop and broadcast a recovery/blocked intent.
2. Recheck local LiDAR, localization freshness, corridor/dock ownership, and peer state.
3. If the reverse path is sensor-clear and not in a protected narrow resource, retreat slowly along the reverse/last-safe path by a bounded **2–4 m** distance (parameterized; default should be conservative).
4. Stop, publish the resulting blockage/recovery event, clear/refresh reservations, and replan.
5. If reverse is not safe, remain stopped and escalate/retry rather than forcing movement.

Never make a collision recovery command bypass the Safety Supervisor, local LiDAR, corridor mutex, or dock reservation.

### 5. Network-loss/intersection hardening

Audit the existing peer tracker, Kalman-style filter, ORCA, Safety Supervisor, WHCA* reservation buffer, and corridor mutex. Much of the desired peer-awareness logic already exists, but verify it instead of assuming it works.

Required policy:

- During loss/degradation of trajectory messages, peer pose prediction must continue using last valid state plus uncertainty growth.
- Approaching a potential intersection/resource conflict under degraded communication must reduce speed or stop based on uncertainty, local sensing, and time-to-conflict.
- The existing one-cell WHCA* same-slot buffer must remain enabled.
- Add an approach-control zone at mouths of narrow aisles/narrow junctions. Robots must not maintain unrestricted straight-line speed immediately before entering; use a mapped approach distance, a reduced speed, and a stop/permission check.
- A narrow resource remains suspect occupied when an `ENTER` owner disappears, until a sensor-backed or explicit recovery procedure clears it.

Do not overstate ORCA: it is a lightweight local reciprocal-avoidance approximation, not a proof that unseen robots cannot collide. The Safety Supervisor remains final authority.

### 6. Tests and long-duration validation

Add focused unit tests for every new deterministic rule, then run headless integration tests.

At minimum verify:

- Four AMRs receive odometry and LiDAR through the launcher.
- Dock choice resolves to one owner when two robots request the same dock.
- Lease expiry/release makes a dock available again.
- Dock confirmation causes a map-anchor correction, not merely an assumed reset.
- Data JSONL has both map and raw simulator odometry fields.
- Collision/recovery events are logged and unsafe reverse is rejected.
- A narrow-junction approach slows/stops without a mutex permit.
- WHCA* keeps its one-cell reservation buffer.

Run headless Gazebo tests; avoid GUI unless visual inspection is essential. Store test log directories and summarize exact evidence. For long soak runs, use the provided launcher with a bounded duration and inspect JSONL/process logs while it runs. If the operator chooses a lower-cost model, `gpt-5.6-luna` with `high` reasoning is an appropriate option for log monitoring and iterative fixes, but do not assume that model selection changes safety requirements.

## Relevant files

- `CURRENT_IMPLEMENTATION_GUIDE.md` — current source of truth and required update target.
- `context.md` — broader project context; update the current-status section after verified work.
- `warehouse_layout.lock.yaml` — root canonical layout lock.
- `src/sih_amr_fleet/maps/demo_warehouse.yaml` — grid, anchors, graph, mutex resources.
- `src/sih_amr_fleet/sih_amr_fleet/localization_node.py`
- `src/sih_amr_fleet/sih_amr_fleet/charging_pad_node.py`
- `src/sih_amr_fleet/sih_amr_fleet/data_collection_node.py`
- `src/sih_amr_fleet/sih_amr_fleet/peer_tracker_node.py`
- `src/sih_amr_fleet/sih_amr_fleet/orca_node.py`
- `src/sih_amr_fleet/sih_amr_fleet/safety_supervisor_node.py`
- `src/sih_amr_fleet/sih_amr_fleet/corridor_mutex_node.py`
- `src/sih_amr_fleet/sih_amr_fleet/whca_planner_node.py`
- `src/sih_amr_fleet/sih_amr_fleet/algorithms.py`
- `src/sih_amr_fleet/launch/fleet.launch.py`
- `src/sih_amr_fleet/launch/spawn_minimal_amr.launch.py`
- `scripts/launch_four_amrs.sh` and `scripts/run_baseline_random_fleet.sh`
- `src/sih_amr_fleet/test/`

## Engineering constraints

- Use `apply_patch` for source edits.
- Preserve existing user changes in the dirty worktree.
- Keep ML, camera classification, and learned policies excluded.
- Never claim a physical sensor feature is implemented merely because its map marking or future protocol exists.
- Keep all coordination decentralized; dashboard/logger nodes must remain passive.
- Keep all movement commands behind ORCA and the Safety Supervisor.
- Update `CURRENT_IMPLEMENTATION_GUIDE.md` and `context.md` with implementation versus verified status. Be explicit about anything still simulated or unverified.
- Do not commit, push, reset, or delete work unless explicitly asked.

## Completion report expected

State exactly what changed, tests run and results, headless run/log paths, collision/recovery evidence, any remaining failures, and the precise commands the user can use to repeat the tests.
