# SIH 2026 PS 26123 — Project Context for Research and AI Agents

## How to use this file

This file is a compact but comprehensive context pack for teammates and AI agents researching or implementing the project. It combines the project architecture, team responsibilities, execution order, data/AI plan, terminology, and the team’s clarified meeting decisions.

The authoritative project documents are:

- `SIH_2026_PS_26123_Decentralized_Multi_AMR_Fleet_Architecture.md`
- `SIH_2026_PS_26123_Team_Roles_and_Execution_Guide.md`
- `SIH_2026_PS_26123_ML_DL_Dataset_and_Start_Plan.md`
- `SIH_2026_PS_26123_Master_Project_Guide.md` (consolidated copy of the above)

The repository currently contains planning/architecture Markdown documents, not a completed implementation. Do not claim that a component is already implemented merely because it is described here.

## 1. Project identity and objective

**Problem:** Coordinate at least three autonomous mobile robots (AMRs) moving goods in a dynamic warehouse. They must move concurrently, resolve conflicts at narrow aisles/intersections, reroute around temporary obstacles, reassign tasks after failures, and remain safe during packet loss or network partitions.

**Proposed platform:** ROS 2 + DDS, Gazebo simulation first, with a path to Raspberry Pi 4 / Jetson-class edge computers and physical AMRs.

**Main design claim:** There is no permanent central fleet controller, broker, or single point of failure. Each AMR runs the same coordination stack, exchanges peer state directly, and retains local safety control when communication or the dashboard fails.

**Success target:** Demonstrate zero inter-robot collisions in the defined benchmark envelope and at least 20% lower mean makespan than a stop-and-wait baseline, while reporting reproducible metrics rather than making an absolute mathematical guarantee.

## 2. Three concerns kept separate

### Efficiency

- WHCA* plans conflict-aware short-horizon routes.
- CBBA allocates and reallocates delivery tasks.
- A small edge-AI model predicts congestion and travel time.

### Coordination

- Robots publish short-horizon trajectory intents and maintain local reservation tables.
- A Ricart–Agrawala-style protocol grants permission for narrow shared spaces.
- Task assignment epochs, leases, session IDs, and sequence numbers reject stale ownership/messages.

### Safety

- ORCA makes local collision-avoidance velocity adjustments.
- A local Safety Supervisor independently checks LiDAR, braking distance, time-to-collision, localization validity, command freshness, and E-stop state.
- Local sensing is the final safety authority. AI and network data can improve efficiency but cannot override an immediate stop.

## 3. End-to-end runtime picture

```text
LiDAR / odometry / IMU / camera
        ↓
Self-localization, dead reckoning, local costmap
        ↓
Peer tracker (one Kalman filter per neighbour) ← DDS peer messages
        ↓
CBBA task allocation + edge congestion/ETA predictor
        ↓
WHCA* space-time planner + local trajectory reservations
        ↓
Distributed corridor mutex (REQUEST / GRANT / ENTER / EXIT)
        ↓
ORCA local collision avoidance
        ↓
Safety Supervisor (local hard veto)
        ↓
Motion controller / cmd_vel / motor driver
```

The dashboard receives read-only telemetry through a bridge and is never in this control path.

## 4. ROS 2 and DDS

### ROS 2

ROS 2 is the robotics framework. A robot is decomposed into nodes, each responsible for one capability. Nodes communicate through topics, services, and actions. Examples in this project include `localization_node`, `whca_planner_node`, `cbba_node`, and `safety_supervisor_node`.

### DDS

DDS (Data Distribution Service) is the middleware commonly underneath ROS 2. It provides peer discovery, publish/subscribe communication, serialization, and configurable Quality of Service (QoS). DDS is transport middleware, not a consensus algorithm and not a fleet-allocation protocol.

### Domains and namespaces

- All AMRs in one coordinating fleet use the same `ROS_DOMAIN_ID`.
- Each AMR uses a unique namespace such as `/robot_1`, `/robot_2`, `/robot_3`.
- Different domains are for separate fleets or simulations, not robots that must coordinate.

### RMW and discovery

ROS 2 accesses DDS through the `rmw` abstraction. One DDS/RMW implementation (for example Cyclone DDS) must be selected, measured, and tuned on the actual simulation/hardware network. Multicast discovery is convenient in simulation; if warehouse Wi-Fi filters multicast, use a supported static-peer or discovery-server configuration. This configuration is not a central fleet controller.

### QoS principles

- Pose: `BEST_EFFORT`, `KEEP_LAST(1)`, volatile; newest state matters most.
- Trajectory intents: reliable, bounded history, state-like durability where appropriate, short lifespan.
- Corridor protocol: reliable, bounded history.
- Task consensus: reliable, bounded history; epochs reject stale data.
- Blockage observations: reliable with finite lifespan/TTL.
- Health: low-rate status with DDS liveliness plus application freshness checks.

Reliable DDS delivery only means transport delivery between matched endpoints. It does not establish agreement; application protocols provide grants, consensus, epochs, and expiry.

## 5. Per-AMR software nodes

| Node | Responsibility |
|---|---|
| `localization_node` | Fuse odometry, IMU and optional AMCL/LiDAR into pose and velocity. |
| `local_costmap_node` | Maintain a local obstacle map from LiDAR. |
| `blockage_detector_node` | Convert persistent sensed obstacles into TTL-bound blockage observations. |
| `peer_tracker_node` | Track peers, timestamps, sessions, sequence numbers, and Kalman uncertainty. |
| `cbba_node` | Decentralised task bids, bundle construction, consensus, and reassignment. |
| `edge_congestion_predictor_node` | Predict short-term traversal time/congestion as a soft planning cost. |
| `whca_planner_node` | Plan rolling space-time routes using reservations and cost estimates. |
| `reservation_manager_node` | Publish/validate short-horizon trajectory intents. |
| `corridor_mutex_node` | Run permission protocol for narrow aisles/intersections. |
| `orca_node` | Produce a locally collision-avoiding candidate velocity. |
| `safety_supervisor_node` | Apply hard sensor-derived stop/slow constraints before motors. |
| `dashboard_bridge_node` | Expose read-only fleet telemetry to the dashboard. |

## 6. State estimation: self versus peers

### Self-localization and dead reckoning

Dead reckoning predicts a robot's own state from previous pose, wheel odometry, velocity, direction, IMU, and elapsed time. It drifts because of wheel slip, noise, and modeling error. LiDAR/map localization can correct the drift.

### Kalman filtering

A Kalman filter repeats two stages:

1. **Prediction:** use the previous estimate and a motion model to predict the next state.
2. **Update/correction:** combine that prediction with a new measurement, weighted by uncertainty.

In this project, `peer_tracker_node` maintains a constant-velocity filter per neighboring robot with state `[x, y, vx, vy]`. When peer messages are delayed, the filter predicts forward and its covariance grows. ORCA and the speed policy use that uncertainty to increase spacing and become conservative.

Kalman filtering and dead reckoning are not the same: dead reckoning is a motion prediction method; a Kalman filter is a prediction-plus-measurement-fusion estimator. A Kalman filter may also be used for self-localization, but the architecture explicitly emphasizes it for peer tracking.

### Communication states

- 0–500 ms: normal.
- 500–2,000 ms: `COMM_DEGRADED`; inflate uncertainty, lower speed, avoid new corridor entry.
- Over 2,000 ms or liveliness lost: `PEER_UNREACHABLE`; treat predicted region conservatively, expire ownership through protocol, and re-auction work when safe.

## 7. Task allocation: CBBA

**CBBA = Consensus-Based Bundle Algorithm.** It answers: **which robot should perform each task?** It does not control motors, emergency braking, or aisle clearance.

### CBBA flow

1. A task is published.
2. Each eligible robot calculates a bid, typically estimated travel time + congestion cost + battery penalty + workload/queue penalty.
3. Each robot inserts attractive tasks into a local ordered bundle.
4. Robots broadcast bids, current winners, bundle state, assignment epoch, and lease information.
5. Conflicting local views are reconciled through repeated consensus updates until allocation converges or a bounded round ends.
6. The winner stores the task locally and WHCA* plans its route.

There is no safe assumption that a robot knows when every bid has arrived. CBBA addresses incomplete information through repeated consensus rather than a simplistic “wait for all bids” rule.

### Stale assignments and failure

Assignments include:

```text
task_id, assignment_epoch, owner_robot_id,
owner_session_id, lease_until
```

If Robot A owns Task 17 at epoch 7, disconnects, and Robot B receives it at epoch 8, Robot A must abandon its old epoch-7 ownership when it reconnects. Reallocation can be triggered by owner failure/unreachability, persistent blockage, low battery/safety infeasibility, or a higher epoch.

If only one active task per robot is needed, CBAA (Consensus-Based Auction Algorithm) is a simpler prototype variant. CBBA is better when robots maintain ordered bundles/queues.

## 8. Planning: A*, WHCA*, and trajectory intents

Normal A* searches a map in `(x, y)` using `f(n)=g(n)+h(n)`. This project uses **WHCA*** (Windowed Hierarchical Cooperative A*), which searches a rolling space-time window `(x, y, time)`.

Each robot publishes a trajectory intent:

```text
robot_id, session_id, plan_id, t0, dt, valid_until,
[(grid_cell, time_slot), ...], priority
```

Every robot builds a local replicated reservation table. A reservation means “this robot expects to occupy this cell during this time slot.” Valid conflicting reservations are treated as occupied by WHCA*. The route is refreshed and replaced by `plan_id` and expiry.

The planned horizon is roughly 10–15 time steps. A cached reverse-Dijkstra distance-to-goal heuristic can speed repeated searches. Replanning occurs after assignment, window advance, reservation conflict, blocked corridor, persistent blockage, route deviation, or degraded communication.

WHCA* provides a strategic route; it does not replace ORCA or the Safety Supervisor. It outputs macro-waypoints or a desired direction, not final wheel commands.

## 9. Narrow-space coordination: Ricart–Agrawala-style mutex

The project’s “Agarwal protocol” reference means **Ricart–Agrawala**, a permission-based distributed mutual-exclusion protocol.

```text
REQUEST(corridor_id, request_id, Lamport timestamp)
    → peers compare (timestamp, robot_id)
    → GRANT or DEFER
    → requester receives required grants
    → local sensing confirms braking envelope is clear
    → ENTER
    → traverse
    → EXIT / RELEASE
```

It is used for one-lane aisles, intersections, lifts, and other predefined corridor resources. Lamport ordering alone is insufficient because delayed messages can give robots different views of outstanding requests.

Corridor network state is distinct from physical occupancy:

```text
FREE → RESERVED → OCCUPIED → FREE
                      └→ lease/communication failure: SUSPECT_OCCUPIED or BLOCKED
```

An expired lease removes network ownership but never proves physical clearance. Only a valid exit or local sensing can clear the corridor.

## 10. ORCA and the Safety Supervisor

### ORCA

**ORCA = Optimal Reciprocal Collision Avoidance.** It is a reactive local-velocity algorithm. It uses the robot’s current state, estimated peer states, peer uncertainty, robot radii, and desired direction from WHCA*. It calculates a preferred safe velocity. ORCA is not a localization system and does not contain a Kalman filter by definition.

### Four safety layers

1. Strategic: WHCA* + trajectory reservations.
2. Critical-space: corridor mutex.
3. Reactive: ORCA.
4. Hard local safety: sensor-driven Safety Supervisor.

### Safety Supervisor

The supervisor sits between motion commands and the motor interface. It independently checks LiDAR/local obstacle distance, braking envelope, time-to-collision, localization validity/age, stale velocity commands, E-stop state, and actuator health. It can veto any planner or ORCA command. A command multiplexer with a high-priority stop/slow channel is a possible implementation, but the architecture requires the supervisor to be authoritative, not a particular multiplexer package.

## 11. Dynamic blockage and rerouting

Blockages must originate from sensing, not a manually invented event:

```text
LiDAR → local costmap → persistence threshold
      → BlockageObservation(polygon/cells, confidence, observed_at, valid_until)
      → peer sharing/fusion → temporary blockage state
      → WHCA* replan → CBBA reallocation only if still infeasible
```

Observations have a TTL and require re-observation, so a moved pallet does not block an aisle forever.

## 12. Edge AI and vision

### ML: congestion and ETA regression

The ML model predicts a number: likely travel time for a route/task. It uses the team’s own Gazebo fleet logs, not a generic public “fleet congestion” dataset. Example features include static path length, nearby robot count, queue length, recent corridor travel time, blockage count, task load, and optional peer freshness. The target is actual travel time.

Recommended progression:

1. Static distance / nominal speed baseline.
2. Distance plus fixed queue penalty.
3. Linear regression, random forest, or gradient-boosted trees.
4. Optional tiny neural network exported to ONNX.

The model publishes a small `RouteCostEstimate` containing route ID, predicted travel time, confidence, model version, and fallback flag. It contributes only a soft WHCA*/CBBA cost. If missing, stale, or low-confidence, use static travel time.

Use split-by-run train/validation/test data to avoid leakage. Start with a few hundred route examples from controlled seeds and measure MAE/RMSE, latency, and ablation improvement.

### DL: object detection

The vision model detects a small class set such as pallet, carton/box, and optionally person. It outputs class, bounding box, and confidence. Use transfer learning with a licensed warehouse dataset, Gazebo camera images matching the intended viewpoint, and a small real test set. Use diverse scene/run splits rather than adjacent video frames.

Vision publishes observations; it never directly commands motion. Example:

```text
Camera → detector → VisionObservation
       → dashboard / blockage-confidence fusion
       → LiDAR confirms physical obstacle
       → Safety Supervisor remains movement authority
```

Recommended metrics are precision, recall, false positives, inference latency, and held-out test performance. Record dataset licences and model versions.

## 13. Dashboard and observability

The dashboard is passive and read-only, potentially fed through `rosbridge_websocket`. It should display:

- robot pose, path, goal, battery, and freshness/connection state;
- trajectory intents and reservation conflicts;
- corridor state (`FREE`, `RESERVED`, `OCCUPIED`, `SUSPECT_OCCUPIED`, `BLOCKED`);
- task owner, assignment epoch, queue, and reallocation events;
- blockage observations and expiry;
- ORCA interventions, safety slowdowns/stops, and E-stop status;
- makespan, task latency, utilization, wait time, planner runtime, and collision count;
- comparison against stop-and-wait and AI-ablation baselines.

The fleet must continue operating if the dashboard is disabled.

## 14. Fault handling and demonstrations

| Fault/demo | Required response |
|---|---|
| Three robots request one corridor | One obtains grants and local clearance; others wait/replan. |
| Robot stops inside corridor | Lease expiry creates `SUSPECT_OCCUPIED`, not `FREE`. |
| Packet loss/delayed poses | Kalman covariance grows, ORCA spacing inflates, speed reduces; LiDAR remains local protection. |
| Network partition | No new shared choke-point commitments; safety over availability. |
| Robot process crash mid-task | Freshness/liveliness loss, ownership lease expiry, CBBA reallocation at a newer epoch. |
| Pallet blocks aisle | Persistent sensed blockage with TTL; WHCA* reroutes. |
| Dashboard closed | Robots continue without operational change. |

## 15. Team roles and ownership

### AI generalist / integration lead

Owns Gazebo warehouse, robot namespaces, ROS 2 launch, core state machines, contracts, WHCA*, trajectory intents, corridor protocol, CBBA integration, and end-to-end scenarios. Learns ROS 2/Gazebo/Nav2 basics, costmaps, `cmd_vel`, A*/WHCA*, ORCA, CBBA, and integration testing.

### Cybersecurity and reliability lead

Owns robot identity and message-validity checks (`robot_id`, `session_id`, sequence numbers, timestamps, expiry), DDS QoS/liveliness and connection-health configuration, network partition/degraded-mode tests, and a lightweight security posture. Ensures stale/replayed messages do not control current state.

### Machine-learning lead

Owns fleet-data logging, simulation dataset generation, travel-time baseline, congestion/ETA model, local inference, confidence/fallback behavior, ablations, and regression evaluation. The model must never command braking or collision avoidance.

### Deep-learning/perception lead

Owns class taxonomy, licensed/public plus Gazebo camera dataset, detector fine-tuning, annotations, held-out evaluation, ROS vision observations, and LiDAR/vision blockage-confidence fusion. Vision is supportive; LiDAR/Safety Supervisor owns safety.

### Full-stack/dashboard lead

Owns read-only telemetry bridge, live fleet dashboard, event/metric logging and replay, benchmark comparison views, and demo controls that cannot control the fleet. Must prove the dashboard can be turned off without stopping robots.

## 16. Recommended build order

### P0 — movement and safety

- Gazebo warehouse with three namespaced AMRs.
- Independent robot control and visibility.
- Localization, LiDAR costmap, Safety Supervisor, E-stop/braking.
- Basic peer pose sharing and ORCA; two robots avoid each other.
- Repeatable three-robot corridor permission demo.

### P1 — coordination and recovery

- WHCA* with trajectory-intent reservation table.
- Peer Kalman filters, freshness states, session IDs, and sequence validation.
- Sensed blockage pipeline, TTL, and rerouting.
- CBBA with assignment epochs and failure/blockage reassignment.

### P2 — evidence and AI

- Passive dashboard and metric logging.
- Stop-and-wait baseline and repeated seeded benchmarks.
- Edge congestion predictor and deterministic-stack ablation.
- Vision detector and blockage classification integration.

### Stretch

Run the same nodes on Raspberry Pi/Jetson-class hardware with LiDAR and controlled Wi-Fi tests.

## 17. Benchmark methodology

Run identical maps, initial positions, task arrivals, robot count, speed limits, and random seeds for:

1. Stop-and-wait baseline.
2. Deterministic decentralised stack: WHCA* + mutex + ORCA + Safety Supervisor.
3. Full stack with congestion-prediction AI.

Use at least 10 seeds per scenario, including 3-robot and 5-robot cases and high-choke-point density. Report mean, median, variance, and worst case for makespan, task latency/completion, collision count, minimum separation, safety stops, corridor wait/lock latency, replans, reassignment success, planner runtime, ORCA rate, network degradation, and AI ablation gain.

## 18. Clarified meeting decisions and discussion

Only a subset of the team joined the meeting. The primary discussion covered the distributed-system architecture: ROS 2, what topics are, how topics carry the project’s state and coordination messages, the leasing policy, and the protocols/algorithms used by the fleet. The architecture sections above contain the formal version of those explanations.

### Charging-point reference for restart localization

The proposed warehouse has a known charging point whose map coordinates are fixed. A robot normally estimates its pose relative to this known map/reference system. If it restarts and does not know its current pose, the team discussed a recovery procedure:

1. Mark the warehouse boundary with a detectable black-tape perimeter.
2. Have the robot move in a safe straight-line direction within an aisle until it detects the warehouse boundary.
3. Follow the boundary/perimeter until it locates the charging point.
4. Use the known charging-point position as a reference to recover the robot’s map-relative pose.

This is a proposed recovery strategy, not yet an implemented or safety-approved behavior. A real implementation must include a low-speed recovery mode, obstacle/people sensing, an emergency stop, and a guaranteed safe direction policy; “move in any direction” is not acceptable on hardware without those constraints.

### Why peer-relative restart localization was rejected

A teammate suggested that a rebooted robot could estimate its position relative to the other robots. The objection was that a robot may restart after continuing to roll or move due to momentum. It would then need to know:

- the exact time at which it lost power or state;
- where each other robot was at that exact time;
- how every other robot moved during the gap; and
- how to reconcile those estimates with delayed messages and uncertainty.

Because those historical states are not reliably available, the meeting rejected peer-relative localization as the primary restart-recovery method. The known charging point/warehouse reference is preferred. Peer messages may still assist localization after a safe reference pose has been recovered, but they must not be treated as a trusted substitute for it.

### Candidate edge-ML runtimes

The meeting suggested **TensorFlow Lite** and **PyTorch Mobile** as possible runtimes for deploying a trained model on edge hardware. The architecture also mentions ONNX. These are candidate deployment formats/runtimes, not three models that must all be used. The team should benchmark one selected runtime on the target Raspberry Pi/Jetson-class hardware and record model version, latency, memory use, and fallback behavior.

### WHCA* baseline followed by learned route selection

The meeting proposed the following experimental route-selection workflow:

1. Use WHCA* as the trusted baseline route planner in Gazebo.
2. Run simulations for a large number of steps/scenarios and collect route, state, decision, congestion, blockage, and outcome data.
3. Train a machine-learning model on those collected examples to predict or select a good route.
4. Compare the learned route selection against WHCA* on held-out simulation runs.
5. Run both in parallel during evaluation/operation: if the ML model has sufficient confidence, its route suggestion can be considered; if confidence is low or the model is unavailable, use WHCA*.

This does not remove WHCA*’s cooperative reservations, corridor mutex, ORCA, or the Safety Supervisor. The ML output remains a route-cost/route-suggestion layer. It cannot directly command braking or override deterministic safety. Any learned route must still pass reservation/conflict checks and local safety checks.

### Fleet data-logging topic and replay record

The meeting proposed a dedicated data-logging topic to collect the project’s telemetry in one stream, including:

- robot positions, velocities, and sensor-derived state;
- task bids, assignment decisions, ownership/epoch changes;
- WHCA* route and trajectory-intent decisions;
- corridor REQUEST/GRANT/ENTER/EXIT/RELEASE events;
- ORCA adjustments and Safety Supervisor slow/stop decisions;
- blockages, reroutes, failures, communication state, and timestamps.

After a Gazebo run, this stream should be recorded/replayed (for example with a ROS bag or an equivalent structured log) so the team can reconstruct what happened and when. The resulting record is the source for debugging, benchmark analysis, and ML dataset construction. A ROS topic by itself is not permanent storage; recording, schema versioning, run IDs, random seeds, and synchronized timestamps are required.

The logging path must not become a control dependency: if logging or the dashboard fails, the fleet must continue operating safely.

## 19. Important boundaries for AI agents researching this project

- Do not replace the decentralised design with a permanent central controller unless the team explicitly changes the architecture.
- Do not confuse DDS reliability with distributed consensus.
- Do not describe WHCA* as merely ordinary A*; its cooperative state includes time and peer reservations.
- Do not say an expired network lease proves a corridor is physically clear.
- Do not make ML or vision responsible for emergency stopping.
- Do not treat ORCA as a Kalman filter or localization system.
- Do not let the dashboard become an operational dependency.
- Do not claim “zero collisions guaranteed” beyond the measured test envelope.
- Preserve static fallbacks when AI models fail.
- Use reproducible seeds, held-out runs, message schemas, timestamps, epochs, leases, and session IDs.

## 20. Useful research questions

When exploring a topic, answer it in the context of this fleet:

1. What inputs and outputs does the component have?
2. Is it strategic planning, coordination, estimation, reactive control, or hard safety?
3. Does it require peer communication, and what happens when that communication fails?
4. What state can become stale, and how are expiry/session/epoch checks applied?
5. What QoS is appropriate and why?
6. What is the safe fallback if the component is missing or uncertain?
7. How will the behavior be demonstrated and measured in Gazebo?

## Bottom line

This is a layered, decentralised, edge-executed multi-AMR system. CBBA chooses task ownership; WHCA* chooses a short-horizon space-time route; Ricart–Agrawala-style permissions protect narrow corridors; Kalman filtering estimates peers under imperfect communication; ORCA adjusts local velocity; and the LiDAR-based Safety Supervisor has the final power to slow or stop the robot. Gazebo, ROS 2, DDS, QoS, localization, costmaps, logging, dashboarding, ML, and DL support those layers without replacing the safety boundary.

## 21. Current implementation status — September 7, 2026

### Verified deterministic-safety update — September 8, 2026

The deterministic baseline now includes a project-owned decentralized dock
lease protocol, mapped dock-centre final-alignment target, charging-pad
confirmation before a dock-anchor map correction, raw-versus-map odometry
telemetry fields, recovery event logging, conservative 2.0 m reverse-recovery
policy, and narrow-resource approach stopping under absent permission or
degraded peer communication. Green/red guidance strips were added to the
canonical layout lock and `warehouse_clean.sdf` as visual/map semantics only;
there is no camera/tape perception claim.

On this host, the overlay built, the 19 pure deterministic tests passed, and
the SDF validator accepted the updated world. A bounded headless launcher run
passed robot 1's real odometry/LiDAR gate and began robot 2, but was interrupted
before all four robots could be verified. A second launch was refused by the
existing overlap guard after it matched the execution sandbox wrapper. Thus the
four-robot gate, live dock confirmation, physical collision/contact telemetry,
and long-duration soak remain unverified—not accepted benchmark evidence.

The repository now includes the static-algorithm baseline additions: the locked-layout map publisher, lane-network task locations, random task generator, task-execution lifecycle, passive JSONL data collector, charging-pad simulation node, four-AMR launcher, and a one-command baseline launcher. The data collector is passive and produces a run manifest plus fleet-state, health, consensus, reservation, corridor, safety, blockage, and execution records when those messages are available.

The current Gazebo profile uses a **6.0 m/s simulation-only maximum speed**, within the requested 5–7 m/s range. This is not a real-robot speed recommendation; physical limits and braking envelopes must be measured and configured separately.

Verification completed on this host:

- `colcon build --symlink-install --packages-select sih_amr_interfaces sih_amr_fleet` succeeds.
- The pure algorithm and lane-network suite passes **11 tests**.
- A headless Gazebo launch successfully started the warehouse, four Lite AMRs, all controller/interface-readiness gates, the full 60-process fleet graph, random task generation, CBBA traffic, and passive JSONL logging.
- Local-state and LiDAR QoS incompatibilities found in the first live run were corrected by using the intended best-effort/volatile local pose/sensor profile. The repeat run had no QoS incompatibility errors.

The static baseline is **not yet accepted as end-to-end complete**. The repeat bounded run showed only `robot_3` publishing live fleet state while the other spawned AMRs did not supply usable odometry/state to their fleet nodes; as a result, CBBA converged incorrectly on `robot_4` for the observed tasks. The task executor now protects an active task from being replaced by later assignments, but the remaining multi-AMR odometry/control publication issue must be resolved and then tested through pickup, dwell, dropoff, completion, route reservations, corridor entry, and safety-stop evidence before long data collection starts.

ML route selection/cost prediction, ML confidence fallback, camera obstacle image collection/classification, semantic labels, and learned CBBA cost are intentionally deferred. The immediate next work is to make the deterministic WHCA* + CBBA + mutex + ORCA + Safety Supervisor stack complete and repeatable; only then should sustained simulation logging be used for ML/DL training data.

### Validation agent record — September 8, 2026

The required build and SDF validation passed, and the focused suite passed 27
tests. The bounded headless run `/tmp/sih_headless_validation_20260908_02`
spawned four Lite AMRs; every robot passed the launcher gate requiring actual
odometry and LiDAR samples. Telemetry contained all four robot-state streams
and non-null raw `gazebo_odom` samples. It completed 0 delivery tasks with 4
active robots: 0 fleet work cycles, with `robot_1=0, robot_2=0, robot_3=0,
robot_4=0`; no fairness conclusion is possible from an all-zero workload.

The first run `/tmp/sih_headless_validation_20260908_01` exposed incompatible
QoS on `/fleet/dock_protocol` and `nearest_obstacle_m`; the smallest fixes
changed localization's dock subscription to `PROTOCOL_QOS` and the path
follower's nearest-obstacle subscription to `POSE_QOS`, with a regression test.
The corrected run had no QoS incompatibility warnings, but still produced no
task-consensus, execution, dock, corridor, safety, or collision records. The
diagnostic run `/tmp/sih_headless_validation_20260908_03` also logged Gazebo
controller `publish_async_failures_` and NaN-command warnings. The Safety
Supervisor now rejects nonfinite candidate velocity components with a stop
reason; this is unit-tested but not yet live-verified. Live dock confirmation,
pickup/dropoff motion, safety recovery, fault injection, collision-contact
sensing, and long-soak acceptance remain unverified.

After that diagnostic, the local pose publisher was made an exact `POSE_QOS`
match for its local consumers; CBBA now also filters its own known-good fleet
state stream and logs first task/local/fleet-state receipt. Task sources wait
for all four valid robot states, log CBBA subscriber count diagnostically, and
use a transient-local announcement stream that replays uncompleted work every
second. The controller configuration now
matches the existing `TwistStamped` adapter (`use_stamped_vel: true`); the
adapter emits finite idle-stop references and rejects nonfinite input. Passive
telemetry now records task announcements. The overlay rebuilt and all 24 focused
tests passed after these changes. The next bounded run,
`/tmp/sih_headless_validation_20260908_06`, confirmed fleet readiness and five
announcements, with `rnd_task_001` accepted by `robot_2`, but it completed 0
delivery tasks with 4 active robots: 0 fleet work cycles and
`robot_1=0, robot_2=0, robot_3=0, robot_4=0`. All robots remained at their spawn
poses; no execution, dock, corridor, or collision telemetry was recorded.
The JSONL also contained no `task_announcement` records even though five
announcements appeared in the generator log and one was accepted by `robot_2`.
Gazebo still logged controller NaN rejections at lines 201, 303, and 412 of its
run log. Pickup/dropoff motion, live dock confirmation, fault injection, and
long-soak evidence remain unverified.

The next corrective change set adds the exact local `state` stream to the task
executor, WHCA* planner, path follower, ORCA, and Safety Supervisor, so an
intermittent shared fleet-state subscription cannot suppress the local motion
chain. The supervisor now requires fresh LiDAR and evaluates braking clearance
only in the commanded forward/reverse scan sector, retaining hard stops for
stale sensors, invalid commands, and insufficient directional clearance. The
stamped command bridge retains a finite stop reference for controller activation.
The overlay rebuilt and the focused suite passed 26 tests after these changes;
no new headless acceptance run has been performed.

The following bounded run `/tmp/sih_headless_validation_20260908_07` passed
all four real odometry/LiDAR gates and announced five tasks, but no CBBA node
received a task announcement; telemetry therefore had only robot-state records
and no motion/route/execution evidence. DDS discovery reported `0/4` shared
task subscribers. The task source now sends each identical announcement to each
robot's namespaced `task_announcement` input as well as the shared audit topic;
this is broadcast delivery, not centralized assignment, and each robot still
performs its own CBBA bid. Executor status is now also emitted and consumed
locally by its planner/follower and passive telemetry. The overlay rebuilt and
27 focused tests passed after this change; it awaits a new headless run.

Run14 isolated Fast DDS shared-memory lock failures (`fastrtps_port7041`) during
the task-path investigation. The launcher now uses synchronous UDPv4 for this
single-host headless baseline and its cleanup signals both process leaders and
groups. Run15 then passed all robot gates and restored blockage telemetry but
still found zero `TaskAnnouncement` readers. The local source therefore now
uses a validated standard-message payload that each CBBA reconstructs locally
before its normal decentralized bid. The overlay rebuilt and 29 focused tests
passed; no end-to-end completion is claimed until that fallback is live-tested.

Validation record, September 8, 2026: Cyclone DDS was installed, but its default
`Discovery/MaxAutoParticipantIndex` of 9 caused the first full graph attempt to
fail with `Failed to find a free participant index for domain 42`; the affected
fleet nodes exited and that run was invalid. A project-local Cyclone XML profile
now raises the supported maximum to 119 and is wired into the launcher and
package install. The fresh post-fix run
`/tmp/sih_headless_validation_20260908_freshD` selected Cyclone, passed all four
real odometry/LiDAR interface gates, kept the full fleet graph alive, matched
all four task inboxes, and produced five task announcements with four receipts
and four CBBA participants each. It recorded one unique owner per task,
feasible planned routes, trajectory intents, task execution status, and actual
movement. It completed 0 delivery tasks with 4 active robots, therefore 0 fleet
work cycles, with `robot_1=0, robot_2=0, robot_3=0, robot_4=0`; fairness is not
established. `map_pose`, `map_twist`, and non-null `gazebo_odom` were present in
state telemetry. Safety telemetry recorded stops, bounded retreats, and
blocked reverse attempts, but deliberate fault-injection recovery, pickup /
dropoff completion, docking, protected-resource tests, collision-contact
sensing, WHCA* runtime-buffer behavior, and long-soak acceptance remain
unverified. Build, SDF validation, and the focused suite passed 31 tests. No
fleet process crashed in the post-fix run; the remaining controller startup NaN
warnings are at `gazebo_server.log:194,295,401`.

Post-run diagnosis of `freshD` found that the remaining zero-completion result
was downstream of CBBA. Robots 2 and 3 translated while rotating away from
their south-facing dock headings and curved into the south wall; their measured
0.18–0.21 m braking stops were valid safety decisions. Robot 1 passed within
0.234 m of its pickup at about 3.9 m/s, failing the executor's stationary-arrival
gate. Robot 4's continuous pose remained physically south of a shelf, but
rounded into occupied planning cell `(52,45)`, after which WHCA* rejected every
route because the start cell was blocked. The path follower also targeted only
`waypoints[1]` between one-second route updates.

The pending-validation correction makes the follower advance through the local
waypoint sequence, turn in place before translating, use a 1.0 m/s cap matching
the 0.5 m / 0.5 s WHCA* reservation model, slow and stop inside the task arrival
region, and use rear-sector clearance plus odometric distance for rate-limited
reverse recovery. WHCA* now snaps a rounded blocked **start** to the nearest
free cardinal cell while still rejecting a blocked goal. LiDAR blockage points
are now rotated from `base_link` into map coordinates and converted from metres
to actual grid indices. These edits passed a Python syntax check only; build,
focused tests, SDF validation, and a new headless acceptance run remain for the
next validation agent. The one-time controller NaN warnings occur immediately
after vendor diff-drive activation, before a subscriber callback can replace
its internally uninitialised reference; no runtime nonfinite command was seen.
The `pal_statistics publish_async_failures_ 172` line was emitted during process
teardown, not during fleet motion. Neither diagnostic is evidence of a runtime
control NaN, though both should continue to be reported separately.

The next bounded run, `/tmp/sih_headless_validation_20260908_takeover_2018`,
passed build, 34 tests, SDF validation, layout-lock equality, Cyclone selection,
all four robot gates, 5 announcements, 20 receipts, and four CBBA participants
per task. All four robots turned before translating, moved at up to 1.0 m/s,
and robot 4 produced no infeasible route. It still completed no task. Telemetry
showed the precise cause: executor and planner task identities diverged. Robot
3's executor pursued task 1 while 210 of its routes targeted task 4; robot 1
executed task 2 before its planner changed to task 5; robot 2 showed the same
task 3/task 5 split. Task 1 also produced active assignments for robots 4, 1,
and 3 across later epochs. The executor's local task lock was therefore not
enough because neither CBBA nor WHCA* treated execution as a commit.

The pending-validation fix makes live execution status sticky ownership in all
CBBA replicas, excludes busy robots from unrelated assignments, ignores
conflicting executor owners/stale consensus, and prevents WHCA* from accepting
an assignment that differs from its executor task. The same run also revealed
an achieved physics real-time factor around 0.12–0.13 during long straight
segments. Hardware inspection found a Ryzen 5 5600X, 16 GiB RAM, and RTX 3070,
but no `/dev/nvidia*`, failed `nvidia-smi`, and Gazebo EGL `driver (null)` / DRI2
errors. The four Lite descriptions were still rendering 48 GPU lidars at 62 Hz
(44 unused cliff/IR units plus four navigation lidars) and four 30 Hz RGB-D
cameras. The fleet sensor profile now retains only four
navigation lidars at 20 Hz plus contact sensing. All fleet nodes use simulation
time; telemetry schema 0.3 adds wall time; the simulation launcher defaults to
4.0 m/s while direct fleet launch remains 1.0 m/s. WHCA* reservation duration
is derived from resolution/speed. These edits have syntax, shell, diff, and
generated-description checks only; a new full validation must establish
completion and measure the post-optimization real-time factor.

The subsequent host-context run
`/tmp/sih_headless_validation_20260908_gpu_2107` established that the GPU
diagnosis above was a Codex sandbox artifact: the host has driver 595.84,
direct NVIDIA OpenGL 4.6, and Gazebo appeared in `nvidia-smi`. Despite that,
achieved RTF was only 0.097 with Gazebo at about 107% CPU and GPU utilisation
around 36%, so this world is CPU/physics-bound. The 4.0 m/s launcher setting did
take effect (telemetry peak approximately 3.9997 m/s), but only about 26.5
simulation seconds elapsed during roughly 271 wall seconds. A target
`real_time_factor > 1` therefore cannot accelerate this setup by itself.

That run also exposed the remaining ownership race. At simulation time 20.0,
robot 4 published the task-1 assignment at epoch 1 while robot 1 independently
published it at epoch 2; both executors accepted before execution status became
visible at 20.1. The old assignment gate proved only that all four participants
had sent some consensus packet, not that their latest winner decisions agreed.
The pending-validation CBBA correction is now a two-phase commit: every robot
publishes its own bid, each replica computes the deterministic `(bid,
robot_id)` minimum from a complete fresh bid set, every robot publishes a CLAIM
acknowledging that exact winner/session/bid/epoch tuple, and assignment is
permitted only after fresh matching CLAIMs from all four expected robots.
Ownership is committed before the local assignment is emitted; execution
status confirms the commit instead of creating it too late. Autonomous
health/lease epoch promotion during an open auction was removed because it was
the source of divergent simultaneous owners. Syntax validation passed; the
full fleet run remains intentionally delegated for independent validation.

Validation run `/tmp/sih_headless_validation_20260908_cbba_fixed_2118` passed
the rebuild, 37 tests, syntax/diff/SDF checks, and all four physical interface
gates. The ownership safety fix worked: robots 1, 2, and 3 logged the same
commit of task 1 to robot 4, with no conflicting-owner error and no duplicate
executor acceptance. It exposed an asymmetric liveness bug, however. The three
early replicas entered their committed branch and stopped publishing BID
records. Robot 4 had not observed the same sequence-fresh quorum in that exact
round; its cached peer bids expired, producing missing-participant warnings at
fleet log lines 125–126, so the sole permitted owner could never assign.

The next full one-cycle run still produced only three local commit messages per
task. Telemetry showed that all four writers were continuously emitting both
BID and CLAIM with identical winner tuples. The remaining liveness failure was
the cross-writer condition that every claim sequence be newer than that
writer's latest bid sequence. Because all nodes run synchronized
BID-then-CLAIM timers, a receiver can always observe one writer's next BID
before its matching CLAIM and chase a permanently moving sequence boundary.
Commit now requires fresh exact claims from all expected robot IDs, the same
winner/session/bid/epoch tuple, and the same source boot session as each current
bid; it does not compare sequence counters belonging to independent writers.
Pre-commit BID/CLAIM refresh continues until execution confirmation.

The same audit found a separate deterministic navigation liveness bug. The
12-cell WHCA* window used Manhattan distance, so at a shelf face the planner
preferred WAIT over the temporarily goal-worsening sideways steps needed to
reach the next aisle. A replay of every dock-to-task route plus 400 seeded
task-to-task routes reproduced 153 rolling-plan loops. WHCA* now computes an
obstacle-aware reverse-BFS distance-to-goal heuristic; the same 1,140-case
diagnostic produced zero rolling-route failures after the change.

Additional pending-validation corrections subtract the exact shared static map
from LiDAR blockage reports (past logs contained thousands of ordinary shelf
echoes falsely injected as dynamic WHCA* blocks), clear the planner's task
identity on completion, treat task TTL as an unclaimed-work acceptance deadline
rather than an in-progress execution deadline, reset stale corridor arming from
the current rolling route, expire cached trajectory intents instead of
refreshing ghost reservations after a task ends, reject an old route when the
follower observes a different executor task, and rotate base-frame odometry
velocity into the map frame before peer prediction. These changes have diagnostic/static evidence but
still require the independently delegated headless acceptance run before any
pickup, dropoff, completion, collision, or work-cycle claim is made.

Validation `/tmp/sih_headless_validation_20260908_correctness_1ms_1mps_2310`
passed 44 tests and all task-delivery/interface gates but again produced three
commits and zero assignments. The shared telemetry subscriber continuously
received all four sources with constant identical claim values, while the
winning CBBA process later logged a missing BID from a source that the other
replicas continued receiving. This establishes an asymmetric DDS
writer-to-reader path, not an auction disagreement. Consensus now keeps the
typed shared topic and additionally sends each packet through independently
matched reliable per-robot standard-message inboxes. The inbox reconstructs
the original logical source/header after strict field, expiry, participant,
event, and finiteness validation. Same-session BID and CLAIM views also reject
non-monotonic sequence updates. This transport correction is pending the next
delegated live validation.

On 2026-09-09, the one-task diagnostic run
`/tmp/sih_debug_short_cycle_v5_20260909` produced the first verified complete
pickup/dropoff lifecycle. The focused suite passed 49 tests. Robot 1 recorded
all phases in order (66 EN_ROUTE_PICKUP, 10 PICKUP_WAIT, 62
EN_ROUTE_DROPOFF, 10 DROPOFF_WAIT, and 10 COMPLETED publications), with one
assignment, zero infeasible routes, and zero ROS errors. The root cause was a
layering/frame combination: TurtleBot 4 Lite mounts `rplidar_link` at +pi/2
from `base_link`, but costmap and directional scan consumers treated scan
angles as base-frame angles; global coarse LiDAR fusion could then quantize
the executing robot or a task endpoint into WHCA*'s blocked set. LiDAR points
are now transformed with the vendor mount pose, safety sectors account for
the sensor yaw, and WHCA* leaves its immediate self footprint and validated
task-station envelope to the 40 Hz ORCA/safety layers while retaining distant
dynamic blocks. Telemetry schema 0.5 adds one-hertz correlated pipeline
diagnostics, exact route/blockage cells, and every `/rosout` warning/error/fatal
with node/source location. This micro-run proves the lifecycle path; it does
not yet establish four-robot fairness, random-workload throughput, docking,
fault injection, contact sensing, or soak acceptance.

The subsequent 20-minute random workload
`/tmp/sih_full_random_workload_20260909_4mps_retry2` produced three assignments,
111 infeasible WHCA* routes, and zero completions. Offline saved-log replay
showed that all sampled static routes were connected, while four robots' 2.0 s
LiDAR leases fused warehouse-wall, shelf-edge, dock/peer, and swept-trail echoes
into 43--208 dynamic cells. The blockage detector now removes the static-map
quantization halo, finite-map boundary, configured docks, and fresh fleet robot
envelopes before a ten-frame persistence gate, and uses a 0.75 s global lease;
the unfiltered local safety path is unchanged. Applying this semantic filter
offline to the recorded failure snapshots restored route availability.

That run also revealed why some four-participant CBBA commits remained live but
never unanimous: the node recalculated pose-dependent bids every round. Task 5
advertised at least nine distinct winning float32 values, and other tasks could
switch from a finite value to `1e9` after busy-state propagation. Exact claim
matching was therefore correctly rejecting a value that participants
themselves kept changing. Each robot now freezes its first BID and first
complete-set CLAIM per task/epoch and only refreshes their leases. Telemetry
schema 0.6 records `winner_session_id` for direct session-level diagnosis.
These corrections have offline-log evidence, a successful two-package rebuild,
52 passing focused tests, successful Python/launch compilation, a valid SDF,
and matching layout locks. No Gazebo run was performed after them at the
user's request.
