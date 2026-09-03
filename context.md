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
