# Current Fleet Implementation Guide

This is the source of truth for code currently in this repository. It is deliberately stricter than the architecture documents: **implemented** means code exists; **verified** means it ran successfully in the latest test; **planned** must not be presented as complete.

## Status

The ROS overlay builds and 24 pure algorithm/lane tests pass. The latest
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

## Coordinate frame, cells, and anchors

The canonical fleet frame is `map`, not a Gazebo-private coordinate system. It
is a 45 m × 60 m rectangle with origin `(-22.5, -30.0)` metres and 0.5 m
cells. Conversion is `cell_x = round((map_x + 22.5)/0.5)` and
`cell_y = round((map_y + 30.0)/0.5)`. WHCA*, blockage reports, trajectory
reservations, map graph nodes, corridors, and junction resources all use this
same frame.

**Current simulation adapter:** Gazebo wheel odometry is transformed into the
`map` frame by `localization_node`; its origin is initialized from that
robot's configured dock/spawn anchor. Charging-pad docking is also evaluated
from simulated odometry. Gazebo is therefore the present source of simulated
motion truth, but no planner consumes a Gazebo world-pose API directly.

**Physical-AMR target:** each dock is a fixed map anchor. On confirmed docking,
the robot resets/corrects its map pose to that anchor. Between anchors it uses
wheel odometry, IMU, LiDAR/local perception, and the map; green tape is the
planned low-speed recovery/line-following aid in main corridors and narrow
aisles. After a restart it must announce `localization_invalid`, move only in
the safest open direction at recovery speed, follow the main-corridor tape to
a dock, and re-anchor. Tape detection and physical dock sensing are specified
in the locked map but are **not implemented yet**. The deterministic simulated
dock lease/final-alignment/recovery state machines are described below; they do
not claim hardware tape or dock sensing.

The map graph is stored in `maps/demo_warehouse.yaml`: charging-pad anchors,
main junctions, two centre narrow junctions, and their resource edges. Main
junctions are spacious multi-robot WHCA* reservation areas. Narrow junctions
and constrained aisles are one-robot resources governed by the existing
Ricart–Agrawala mutex.

## Nodes and topics

| Node | Inputs | Outputs | Current responsibility |
|---|---|---|---|
| `localization_node` | `/robot_N/odom` | `/robot_N/state`, `/fleet/robot_state`, `amcl_pose` | Transforms simulator odometry to map-relative fleet state. `amcl_pose` is compatibility output, not AMCL. |
| `local_costmap_node` | `scan` | `local_costmap`, `nearest_obstacle_m` | Robot-frame LiDAR occupancy grid. |
| `blockage_detector_node` | local/fleet state, local costmap | `/fleet/blockage_observation` | Ten-frame persistent, semantically filtered, short-lease LiDAR blockage report. |
| `peer_tracker_node` | `/fleet/robot_state` | `peer_tracks` | Per-peer constant-velocity/Kalman-style prediction. |
| `health_node` | peer tracks, safety, charge percent | `/fleet/health`, `status` | Battery, local safety, and communication freshness. |
| `cbba_node` | task announcement/consensus, local state, own filtered fleet state, execution | `/fleet/task_consensus`, local `task_assignment` | Bidding, claim convergence, assignment. |
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

The random generator waits for valid state from all four expected AMRs before it
creates tasks every random 12–25 seconds (seed default `42`), up to five active
tasks. DDS CBBA subscription count is logged diagnostically, while the
transient-local task source tolerates late graph discovery. Pickup and dropoff
are distinct points from `warehouse_tasks.py`, which restricts them to known
aisle centrelines. A `Task` carries ID, pickup/dropoff poses, priority, dwell
times, creation time, and expiry time. It is sent on `/fleet/task_announcement`
and reannounced every second until completion or expiry, so a late DDS discovery
cannot silently discard the only copy. This is task-source replay, not central
allocation; every CBBA participant still bids independently.

Every `cbba_node` stores two in-memory dictionaries: `tasks` and `winners`. This is local process memory, not a database; restarting a node clears it until new messages arrive. Every 0.5 seconds each robot calculates:

```text
travel_distance = distance(robot, pickup) + distance(pickup, dropoff)
base_bid        = travel_distance / max(nominal_speed_mps, 0.1)
priority_bonus  = (priority / 100) × 10 s
bid             = infinity when executing another task
                  otherwise max(1 s, base_bid - priority_bonus)
```

`distance` is currently straight-line Euclidean distance (will change), not
actual route length. `nominal_speed_mps` follows the configured route-tracking
speed. Lower bids win; robot ID breaks ties deterministically. A busy robot is
unavailable instead of accumulating assignments its single-task executor cannot
queue, while higher priority reduces the bid.

Each robot first broadcasts its own `TaskConsensus.BID` on
`/fleet/task_consensus`. After a 2.0 s collection window, every replica derives
the same `(winning_bid, winner_robot_id)` minimum and broadcasts a
`TaskConsensus.CLAIM` for that exact winner/session/bid/epoch tuple. The winner
writes its namespaced `task_assignment` only after fresh matching claims from
all four expected participants. Each participant freezes its first bid and its
first complete-set winner decision for the lifetime of that auction epoch;
robot motion or asynchronously learned busy state cannot alter an already
signed tuple. A claim must come from the same boot session as
that participant's current bid, but commit does not depend on cross-writer
sequence ordering: synchronized BID-then-CLAIM timers otherwise make the newest
bid permanently overtake another writer's claim. Live execution status confirms
and refreshes the pre-execution commit. A busy robot advertises an unavailable
bid, and WHCA* rejects an assignment whose task ID differs from its executor's
current task.

Consensus retains the typed shared topic as its public/audit interface. Every
originating CBBA node also sends the identical logical packet as validated JSON
to each peer's independently matched `/<robot>/consensus_inbox`, using reliable
depth-30 protocol QoS. This repairs asymmetric shared-topic writer/reader paths
without introducing an allocator or weakening the four-participant quorum.
Receivers reject expired, malformed, unknown-source, and non-monotonic
same-session packets before updating BID or CLAIM state.

This is a bounded single-task CBAA/CBBA stepping stone, not complete bundle/queue CBBA. The executor protects one active task from being replaced by later assignments; excess work remains pending until a robot becomes available. Queued bundles, battery-aware bids, full failure reauction, and durable state storage are planned.

The task executor publishes:

```text
EN_ROUTE_PICKUP -> PICKUP_WAIT -> EN_ROUTE_DROPOFF
-> DROPOFF_WAIT -> COMPLETED
```

Arrival requires <=0.45 m position error and <=0.15 m/s speed. Each pickup/dropoff waits the task’s 2–5 s dwell time. Completion is broadcast for about one second; CBBA and the random generator then remove the task, and the planner clears its task identity for the next delivery. The announcement TTL is an acceptance deadline: committed work continues to receive lease refreshes until `COMPLETED` instead of being stranded when its original announcement expires.

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
h = obstacle-aware shortest grid distance to goal
f = g + h
```

It rejects static shelf cells, observed blockage cells, cells reserved by another robot at the same slot, and opposite-direction edge swaps. The reverse-BFS distance heuristic is essential for rolling liveness: a route can temporarily increase Manhattan distance to travel around a shelf instead of selecting a full window of WAIT actions at every replan. If the goal is farther than the window, it returns the reached horizon path and replans every second.

`RoutePlan` is published locally on `planned_route`; a standard `nav_msgs/Path` is also emitted. The reservation manager republishes its cell/time tuples on `/fleet/trajectory_intent` every 0.75 seconds with plan ID, speed-derived slot duration, priority, and 1.5 s validity. An infeasible replacement or an expired source route clears the cached plan, so a completed task cannot become an indefinitely refreshed ghost reservation. Each planner imports still-valid peer intents into its next search. This provides strategic collision avoidance; it is not a physical guarantee.

LiDAR -> local costmap -> expected-geometry/robot subtraction -> ten persistent
occupied frames -> `/fleet/blockage_observation`. The detector removes the
one-cell quantization halo around shared static occupancy, the two-cell
envelopes around fresh fleet robot poses and configured dock anchors, and the
two outer grid rows/columns. Known shelves, warehouse walls, docks, and peer
silhouettes therefore cannot become a second obstacle representation on top of
WHCA* reservations, ORCA, and local safety. Valid remaining reports carry a
0.75 s lease, become planner blocked cells, and force rerouting if an
alternative exists. Entries are removed at lease expiry. Local directional
safety continues to consume every scan without this semantic filtering.
Blockage-triggered task reallocation is planned.

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

It issues `STOP` and zero velocity for E-stop, stale localization/LiDAR, a
nonfinite command, or LiDAR inside the commanded travel direction's braking
distance. It issues `SLOW` in the warning zone and scales linear speed to 40%.
It publishes `entrance_clear` for the corridor policy and a separately measured
rear-sector clearance for guarded reverse recovery. WHCA*, CBBA, ORCA,
dashboard, and future ML cannot override this node.

The Gazebo drivetrain retains the requested simulation-only 6.0 m/s ceiling.
A direct `fleet.launch.py` invocation defaults to a real-world-like 1.0 m/s;
the simulation-only `launch_four_amrs.sh` defaults to 4.0 m/s for faster data
collection and accepts `FLEET_TRACKING_SPEED_MPS=1.0` to restore realistic
timing. WHCA* derives each reservation slot as `0.5 m / tracking speed`, so
changing the speed preserves the route's space-time meaning. The follower advances through the received
waypoint sequence locally instead of repeatedly targeting only waypoint 1,
turns in place before translating across a grid-direction change, and ramps
down before the executor's pickup/dropoff arrival region. Inside 0.35 m of the
active task target it commands a stop so the executor can meet both its 0.45 m
position and 0.15 m/s speed gates.

All fleet nodes use `/clock` simulation time. Telemetry schema 0.3 records both
`logged_at` simulation seconds and `wall_logged_at` host epoch seconds, allowing
achieved real-time factor to be calculated from every run. The fleet sensor
profile removes the vendor Lite model's unused RGB-D camera and eleven unused
cliff/IR GPU lidars per robot, keeps the navigation LiDAR plus contact sensor,
and runs the navigation LiDAR at 20 Hz instead of 62 Hz. The full vendor sensor
profile remains opt-in with `SENSOR_PROFILE=full`.

Host-context validation confirmed that the RTX 3070 and NVIDIA stack are
healthy; missing GPU devices in an earlier Codex run were sandbox isolation.
The fleet-profile run still achieved only `0.097` RTF with Gazebo at about one
fully occupied CPU core and GPU utilisation around 36%, so this world is
physics-bound. The same telemetry measured a 3.9997 m/s peak, confirming that
the 4.0 m/s simulation tracking profile is active. Raising only the requested
SDF real-time factor does not accelerate a simulation below its current
ceiling. Evaluate coarser physics and simplified collision profiles separately
after end-to-end task correctness passes.

CBBA assignment uses a two-phase pre-execution commit. Each participant first
publishes its own BID. Once all expected fresh bids are present, every replica
independently selects the same `(bid, robot_id)` minimum and publishes a CLAIM
for the complete winner/session/bid/epoch tuple. An assignment is emitted only
after fresh, identical CLAIMs from all four expected robots. Execution status
then refreshes this committed ownership; it is not relied upon to resolve a
race after local executors have already accepted.

Pre-commit messages are retransmitted until executor confirmation. In
particular, a replica that observes the unanimous CLAIM quorum before the
winner continues publishing its frozen own BID followed by the committed CLAIM.
This prevents the winner's bid cache from expiring during an asymmetric quorum
observation. Matching execution status stops the additional BID traffic.

## Charging pads

Each `charging_pad_node` transforms its local odometry to a configured warehouse pad pose. It sets docked only with fresh odometry, the robot inside the longitudinal/lateral dock bounds, heading error <=0.35 rad, speed <=0.03 m/s, and two seconds of continuous stability. When docked it raises a project-owned battery estimate at 10 percentage points/minute by default.

```text
/robot_N/charging/is_docked       Bool
/robot_N/charging/battery_percent Float32
/robot_N/charging/battery_state   BatteryState
/robot_N/charging/docking_status  String
```

`health_node` consumes charge percent. This node intentionally does not overwrite the TurtleBot simulator battery topic. Battery-aware scheduling is planned; the deterministic simulated dock protocol is described below.

## Docking, recovery, and telemetry (implemented versus verified)

`DockProtocol` is a project-owned, decentralized leased resource contract.
Each AMR runs `docking_coordinator_node`: a dock need produces a REQUEST,
simultaneous claims are ordered by `(Lamport timestamp, robot ID, request ID)`,
and a winning AMR emits CLAIM before its docking target is exposed to the
planner. Claims refresh their four-second lease; RELEASE and CANCEL free a dock
and an unsuccessful contender selects the next unclaimed dock. No fleet manager
decides this.

Near a claimed target, `path_follower_node` changes to deterministic geometric
alignment to the mapped dock-centre strip at at most 0.15 m/s. That desired
velocity still passes through ORCA and the Safety Supervisor. `charging_pad_node`
uses fresh simulated wheel odometry and its existing stationary/alignment test
to emit `DockProtocol.CONFIRMED`. Only then does `localization_node` solve a new
odom-to-map transform so the current raw odometry sample maps exactly to the
configured dock anchor. This deliberately uses no Gazebo world-pose API.

The painted green/red strips in the lock and `warehouse_clean.sdf` are
semantic/map references and simulator visuals, not sensor input. No
camera/tape perception bridge or physical tape sensing is implemented.

The data collector schema is now `0.2.0`. `robot_state.map_pose`/
`map_twist` carry the planning frame and cell; `robot_state.gazebo_odom` carries
the separately labelled, untransformed raw simulator wheel odometry. It also
records dock protocol and recovery/collision event streams. The launcher writes
`run_events.jsonl` start/exit and interface-gate events; it does not hide a
launcher failure.

Only a persistent braking-envelope STOP starts path-follower recovery; startup
localization/scan staleness does not. The follower first holds, then will reverse
at 0.20 m/s for at most 2.0 m only if **rear-sector** LiDAR clearance exceeds
the reverse distance plus margin and the robot is neither in a protected narrow
resource nor in final docking. Travel is measured from odometry instead of
being inferred only from elapsed command time. A five-second retry cooldown
prevents an unsafe reverse from generating an event storm. The resulting events
identify the map pose, global nearest return, rear clearance, and reason. An
unsafe reverse remains stopped and emits `recovery_blocked`; no Gazebo contact
is invented. A contact bridge/sensor has not been added, so `collision_event`
is a passive input contract rather than evidence of a contact.

For narrow-resource mouths, the mutex node computes a 2.0 m approach zone,
requires a permit before entry, and publishes a zero speed cap when permission
is absent or peer tracking is degraded. Peer prediction continues through
message loss while covariance grows. ORCA remains a lightweight approximation,
not an unseen-robot safety proof; the local Safety Supervisor remains final
authority.

**Verified on September 8, 2026:** the overlay build, 27 pure tests, and
`gz sdf -k` passed. The focused suite covers one-owner dock tie resolution,
lease release/expiry, dock-anchor transform math, raw/map telemetry schema
declarations, unsafe reverse rejection, approach stop policy, and the existing
one-cell WHCA* reservation buffer. The headless run at
`/tmp/sih_headless_validation_20260908_02` passed all four actual odometry and
LiDAR interface gates and recorded four robot-state streams with raw
`gazebo_odom` samples. It completed 0 delivery tasks with 4 active robots,
therefore 0 fleet work cycles, with per-robot completions
`robot_1=0, robot_2=0, robot_3=0, robot_4=0`; fairness is not established.
No live task, dock, corridor, safety, or collision evidence was recorded.
The earlier run at `/tmp/sih_headless_validation_20260908_01` exposed the
fixed dock-protocol and nearest-obstacle QoS mismatches. The diagnostic run at
`/tmp/sih_headless_validation_20260908_03` reproduced no task records and
captured Gazebo controller `publish_async_failures_` plus NaN-command warnings.
The Safety Supervisor now rejects nonfinite candidate velocity components and
publishes a stop reason; this guard is unit-tested but not yet live-verified.
After that diagnostic, the local `state` publisher was made an exact
`POSE_QOS` match for its consumers, CBBA gained a filtered
`/fleet/robot_state` fallback plus first-receipt diagnostics, and task sources
gained readiness gating/replay. The Gazebo command boundary was also corrected
to `use_stamped_vel: true`, matching the project-owned `TwistStamped` bridge;
the bridge now supplies finite idle-stop commands and rejects nonfinite input.
These changes and the passive `task_announcement` telemetry record are covered
by the 24-test suite. A new bounded run at
`/tmp/sih_headless_validation_20260908_06` confirmed readiness, announced five
random tasks, and delivered `rnd_task_001` to `robot_2`, but all robots remained
at their spawn poses and the run completed 0 delivery tasks with 4 active
robots: 0 fleet work cycles, `robot_1=0, robot_2=0, robot_3=0, robot_4=0`.
The run still logged controller NaN rejection at
`gazebo_server.log:201,303,412` and did not produce execution, dock, corridor,
or collision records; the JSONL also contained no `task_announcement` or
`task_execution` records. After this diagnostic, all motion-critical nodes
(executor, WHCA*, follower, ORCA, and Safety Supervisor) gained the exact local
`state` input as a resilience path. The supervisor now requires a fresh scan
and calculates the braking envelope only in the commanded forward/reverse scan
sector; it still stops on stale sensors, invalid commands, and unsafe directional
clearance. The stamped command bridge now retains a finite stop for a newly
activated controller. These changes and their focused tests passed in the
26-test suite, but they have not yet been accepted by a new headless run.
The subsequent bounded run `/tmp/sih_headless_validation_20260908_07` again
passed all four real odometry/LiDAR gates and announced five tasks, but no CBBA
subscriber received them (`0/4` discovery count), so it recorded no task,
route, execution, safety, or completion evidence. To remove that single shared
DDS point from task delivery, task sources now broadcast the same announcement
to each robot's namespaced `task_announcement` input while preserving the
shared topic for interoperability. This performs no allocation: every recipient
still bids through CBBA. The executor also emits a local execution-status stream
consumed by its planner/follower and passive telemetry. These changes passed the
27-test suite but have not yet been accepted by a new headless run.
Four-robot motion completion, live dock confirmation,
fault injection, and long-soak evidence remain unverified.

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

Latest verified diagnostics: the four-AMR UDPv4 run passed all real interface
gates and restored passive blockage telemetry, while Fast DDS shared-memory
lock errors were eliminated. `TaskAnnouncement` still had zero matched readers,
so the local resilience channel now carries a strictly validated standard
message payload which CBBA reconstructs into a typed task before bidding. This
does not allocate work centrally. The overlay rebuild, SDF validation, and 29
focused tests passed; end-to-end delivery with that final fallback remains to
be verified in a new headless run.

The next Cyclone run initially failed because this host's Cyclone build defaults
`Discovery/MaxAutoParticipantIndex` to 9, which is too small for the four robot
launchers plus the 60-process fleet graph. The project now supplies
`config/cyclonedds.xml` with the supported maximum participant index (119),
exports it from the headless launcher, installs it with the package, and covers
the wiring with a focused test. After rebuilding, the bounded run
`/tmp/sih_headless_validation_20260908_freshD` selected
`rmw_cyclonedds_cpp`, passed all four real odometry/LiDAR gates, matched all four
task-inbox readers, recorded five de-duplicated task announcements and 20
receipts (four per task), observed all four CBBA participants per task, and
recorded one unique owner per task plus feasible planned routes and trajectory
intents. Robots 1 and 4 moved; robots 2 and 3 repeatedly stopped on measured
directional clearances of about 0.18--0.21 m. The run completed 0 delivery
tasks with 4 active robots: 0 fleet work cycles and
`robot_1=0, robot_2=0, robot_3=0, robot_4=0`; fairness is not established.
Telemetry included distinct non-null `map_pose`, `map_twist`, and
`gazebo_odom` fields. Runtime recovery records included stop, bounded retreat,
and blocked-reverse events, but controlled fault-injection acceptance, pickup,
dropoff, docking, collision-contact sensing, WHCA* runtime-buffer behavior, and
long-soak acceptance remain unverified. The rebuild and focused suite passed 31
tests. Gazebo still emitted the documented controller startup NaN warnings at
`gazebo_server.log:194,295,401`; no fleet process crash occurred in this run.

## Verified pickup/dropoff diagnostic (2026-09-09)

The bounded run `/tmp/sih_debug_short_cycle_v5_20260909` completed one real
Gazebo task from assignment through pickup dwell, dropoff dwell, and
`COMPLETED`. It recorded phases 0/1/2/3/4 in order, zero infeasible planned
routes, and zero ROS errors; the focused suite is 49 passing tests.

The fix aligns the TurtleBot 4 Lite LiDAR's +pi/2 vendor-URDF mount with
`base_link` in local costmap and directional safety consumers. WHCA* also
removes the coarse one-cell self and validated station envelopes from global
LiDAR blockages because peers can observe the executing robot and endpoint
quantization can otherwise mark the start/goal blocked. ORCA and the 40 Hz
directional safety supervisor remain final authority in those near-field
envelopes; distant dynamic blocks remain active.

Telemetry schema 0.6 writes `pipeline_diagnostic` snapshots for assignment,
execution, route, desired command, ORCA command, corridor gate, safety, and
final command. It also records exact route/blockage cells, the claimed winner's
session ID, and subscribes to
`/rosout`, persisting every warning/error/fatal as `ros_log` with node, file,
function, line, and message. The reusable one-cycle scenario is
`scenarios/debug_short_cycle.yaml`. This is lifecycle proof, not yet the full
random-workload/fairness, docking, fault-injection, collision, or soak result.

## Random-workload blockage/consensus correction (pending live validation)

The 20-minute run `/tmp/sih_full_random_workload_20260909_4mps_retry2`
delivered all task packets but completed no task. Offline reconstruction proves
that static WHCA* paths existed. The old blockage detector instead published
the bottom wall, shelf-surface quantization cells, dock hardware, and the other
robots; four 2.0 s leases overlapped into 43--208 global cells and disconnected
the assigned robots from their pickups. Applying the semantic filter above to
the saved blockage/state telemetry restores connectivity in each sampled
failure case without changing the static map.

The same telemetry exposes a separate CBBA liveness violation. An open task's
pose-dependent BID was recalculated every 0.5 s; task 5 alone advertised at
least nine distinct winning float32 values, and busy-state changes could also
replace a finite BID with `1e9`. Since phase 2 intentionally requires exact
winner/session/bid/epoch agreement, participants repeatedly invalidated their
own quorum. Per-epoch BID and CLAIM decisions are now immutable and are merely
lease-refreshed. The data collector records `winner_session_id` so a future run
can distinguish value disagreement from boot-session disagreement.

No Gazebo run was performed after these changes at the user's request. The
offline saved-log audit, two-package rebuild, 52 focused tests, Python/launch
compilation, valid SDF, matching layout locks, and clean `git diff --check` are
the applicable pre-validation evidence; end-to-end throughput and fairness
remain unverified until the delegated run.
