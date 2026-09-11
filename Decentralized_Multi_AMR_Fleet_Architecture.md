# Decentralized Multi-AMR Fleet Coordination

## Production-oriented architecture for SIH 2026 — PS 26123

**Version:** 3.0 (consolidated design)  
**Platform:** ROS 2 + DDS, Gazebo simulation first; portable to Raspberry Pi 4 / Jetson-class AMRs  
**Scope:** 3+ autonomous mobile robots (AMRs) operating in a dynamic warehouse with decentralised coordination, safe shared-space navigation, task reallocation, edge execution, and a live fleet dashboard.

---

## 1. Executive summary

This solution is a **decentralised, safety-first fleet architecture** for multiple AMRs moving goods through a warehouse. Every AMR runs the same coordination stack locally. Robots exchange state and short-horizon intentions directly using ROS 2 / DDS; there is **no permanent central fleet controller, broker, or single point of failure**.

The design intentionally separates three concerns:

1. **Efficiency:** WHCA* plans conflict-aware short-horizon routes, CBBA allocates tasks, and a lightweight edge-AI model estimates congestion and travel time.
2. **Coordination:** trajectory-intent reservations, a distributed corridor mutual-exclusion protocol, and assignment epochs prevent conflicting plans and stale ownership.
3. **Safety:** ORCA reacts to nearby moving robots, while an independent local Safety Supervisor can slow or stop the AMR using onboard sensing. Network data improves throughput; it is never the final safety barrier.

This separation is important for PS 26123. AI helps predict the faster route and improve fleet throughput, while deterministic local controls protect people, robots, and inventory even during packet loss, a robot fault, or a network partition.

### Core claim for the pitch

> The fleet has no permanent central coordinator. Each AMR makes decisions from its own sensors and a locally replicated view of peer intents. If communication degrades, the system deliberately sacrifices speed and new shared-space commitments before it sacrifices safety.

---

## 2. Problem-statement fit

The architecture is designed to demonstrate the capabilities expected by SIH PS 26123:

| PS capability | Evidence in this design |
|---|---|
| Coordinate 3+ AMRs | Every AMR runs the same peer-to-peer ROS 2 stack; Gazebo scenario starts with 3–5 AMRs. |
| Decentralised peer-to-peer operation | DDS peer communication, local replicated state, CBBA, and distributed corridor mutex; no fleet master is required. |
| Resolve conflicts at choke points in real time | Corridor resources use explicit REQUEST / GRANT / ENTER / EXIT coordination plus local clearance sensing. |
| Dynamic rerouting and task reassignment | Persistent blockage observations trigger replanning; CBBA reassignment occurs after assignment expiry or failure. |
| Edge-AI execution | A small on-device congestion/ETA predictor influences route and bid costs; it is non-safety-critical. |
| Fleet monitoring/dashboard | A passive dashboard shows positions, assignments, corridor state, events, safety stops, and benchmark metrics. |
| Zero inter-robot collisions | Four independent layers: reservations, corridor mutex, ORCA, and sensor-based Safety Supervisor. The demo reports measured collisions, not an unjustified mathematical guarantee. |
| Improve time over stop-and-wait | Identical task sets are run against a stop-and-wait baseline across multiple seeds; target is at least 20% lower makespan while maintaining zero collisions. |

---

## 3. Architecture at a glance

```text
                              EACH AMR
┌──────────────────────────────────────────────────────────────────────┐
│ Sensors: LiDAR • odometry/IMU • optional camera • battery            │
│             │                                                        │
│             ▼                                                        │
│ Localization + local costmap ──► Persistent blockage detector        │
│             │                                                        │
│             ▼                                                        │
│ Peer tracker + per-neighbour Kalman filters ◄──── DDS ◄── Other AMRs │
│             │                                                        │
│             ├──► CBBA task allocation                                │
│             ├──► Edge congestion / ETA predictor                     │
│             ▼                                                        │
│ WHCA* planner + local reservation table                              │
│             │             │                                          │
│             │             └── trajectory-intent publication          │
│             ▼                                                        │
│ Distributed corridor mutex (REQUEST / GRANT / ENTER / EXIT)          │
│             ▼                                                        │
│ ORCA local avoidance                                                  │
│             ▼                                                        │
│ Safety Supervisor (LiDAR + braking envelope + TTC + E-stop)          │
│             ▼                                                        │
│ cmd_vel / motor interface                                             │
└──────────────────────────────────────────────────────────────────────┘
                   │
                   └──► Passive dashboard bridge (non-safety-critical)

              DDS peer-to-peer data exchange — no global fleet master
```

---

## 4. ROS 2 / DDS deployment model

### 4.1 Namespaces and domains

All robots belonging to one fleet use the **same `ROS_DOMAIN_ID`** so that they can discover and communicate with each other. Each robot uses a unique namespace, for example `/robot_1`, `/robot_2`, and `/robot_3`, to prevent topic and frame-name collisions.

Different `ROS_DOMAIN_ID` values are appropriate for separate test fleets, classrooms, or parallel simulations—not for robots that must coordinate in the same fleet.

### 4.2 Why ROS 2 / DDS

- DDS provides peer-to-peer data exchange and configurable Quality of Service (QoS), so the team does not have to implement basic transport retries from scratch.
- The same ROS 2 nodes can be run in Gazebo and on edge hardware, reducing simulation-to-hardware changes.
- DDS alone is **not** a distributed consensus system. Application protocols still provide the acknowledgements, epochs, expiry rules, and safety checks needed for correct fleet behaviour.

### 4.3 Discovery caveat and deployment fallback

Multicast discovery is convenient in simulation and on well-managed networks, but warehouse Wi-Fi may filter or degrade multicast. The deployment plan is:

1. Use normal DDS multicast discovery in simulation and controlled LAN tests.
2. Validate the warehouse access points and VLAN settings before the hardware demo.
3. Configure a static peer list / discovery server mechanism supported by the selected DDS deployment if multicast is unreliable.

This preserves decentralised robot-to-robot data exchange. A discovery configuration is not a fleet-control server.

### 4.4 Middleware choice

The team will select and test one ROS 2 RMW/DDS implementation (for example, Cyclone DDS) on the actual simulation and hardware network. The claim is not that one implementation is universally superior; the claim is that QoS and discovery are **measured and tuned for this deployment**.

---

## 5. Per-AMR node architecture

| Node | Responsibility | Typical rate / trigger | Why it exists |
|---|---|---:|---|
| `localization_node` | Fuses odometry, IMU and optional AMCL/LiDAR into pose and velocity. | 20–50 Hz | Provides trustworthy local motion state. |
| `local_costmap_node` | Maintains local obstacle map from LiDAR. | 10–20 Hz | Lets the robot see real obstacles independently of the network. |
| `blockage_detector_node` | Converts persistent local obstacles into expiring fleet blockage observations. | Event-driven | Gives rerouting a real sensing source rather than assuming an event appears. |
| `peer_tracker_node` | Tracks neighbours, timestamps, sessions, sequence numbers, and KF uncertainty. | Receipt + 20 Hz prediction | Supports graceful communication degradation. |
| `cbba_node` | Decentralised task auction and consensus. | Event-driven / periodic consensus | Allocates and reassigns work without a manager robot. |
| `edge_congestion_predictor_node` | Predicts short-term traversal time / congestion cost locally. | On route/bid evaluation | Supplies genuine edge AI without placing ML in the safety path. |
| `whca_planner_node` | Plans a rolling space-time route using reservations and cost estimates. | On assignment/replan trigger | Avoids foreseeable conflicts efficiently. |
| `reservation_manager_node` | Publishes and validates short-horizon trajectory intents. | On new plan / refresh | Makes WHCA* genuinely cooperative. |
| `corridor_mutex_node` | Runs distributed mutual exclusion for narrow aisles/intersections. | Event-driven | Prevents simultaneous entry into critical shared spaces. |
| `orca_node` | Generates locally collision-avoiding candidate velocity. | 20 Hz | Smooth reaction to moving peers. |
| `safety_supervisor_node` | Enforces hard local stop/slow constraints before motors. | 20–50 Hz | Final protection independent of peer messages. |
| `dashboard_bridge_node` | Exposes read-only fleet telemetry to the UI. | Passive | Enables observability with no operational dependency. |

---

## 6. Communication contracts and QoS

Every coordination message includes at least `robot_id`, `session_id`, `sequence_no`, `sent_at`, and `valid_until` where applicable. A new random `session_id` is generated at robot boot. This prevents packets from a previous process instance of the same robot from being mistaken for current state.

| Topic / message | QoS | Payload and use | Rationale |
|---|---|---|---|
| `/robot_i/pose` | `BEST_EFFORT`, `KEEP_LAST(1)`, volatile; short deadline and lifespan | pose, velocity, covariance, timestamp | Only the newest pose matters; stale retransmissions would hurt a congested Wi-Fi link. |
| `/fleet/trajectory_intent` | `RELIABLE`, `KEEP_LAST(1)`, `TRANSIENT_LOCAL`, short lifespan | `plan_id`, `t0`, `dt`, cell/time reservations, priority, valid-until | A late-joining peer receives the current short plan, while obsolete plans expire. |
| `/fleet/corridor_protocol` | `RELIABLE`, bounded `KEEP_LAST(N)` | REQUEST, GRANT, DEFER, ENTER, EXIT, RELEASE | Critical protocol messages need delivery, but unbounded `KEEP_ALL` is avoided on edge devices. |
| `/fleet/task_consensus` | `RELIABLE`, bounded history | CBBA bids, winners, assignment epoch, owner lease | Prevents silent loss while epochs reject stale data. |
| `/fleet/blockage_observation` | `RELIABLE`, bounded history, finite lifespan | polygon/cell, confidence, observed-at, expiry | Shares dynamic blockages without making a one-time observation permanent. |
| `/fleet/health` | `BEST_EFFORT` or reliable low-rate status; liveliness enabled | battery, task progress, safety state | Supports fleet awareness; not a substitute for local obstacle sensing. |

**Important distinction:** DDS `RELIABLE` addresses transport delivery between matched endpoints. It does not establish application-level agreement. The corridor protocol's grants and task allocation's consensus/epochs establish protocol state.

Use DDS **liveliness** alongside application-level timestamp and sequence freshness. Liveliness says a publisher appears alive; freshness says whether the robot is still sharing useful current motion data.

---

## 7. Cooperative planning: WHCA* with trajectory-intent reservations

### 7.1 Why the original design needed a change

Knowing only a neighbour's present pose does not make WHCA* cooperative. A robot must know which cells another robot intends to occupy in the near future. Without that, two robots can independently reserve the same future location.

### 7.2 Revised approach

Each AMR publishes a rolling **trajectory intent** after planning:

```text
TrajectoryIntent {
  robot_id, session_id, plan_id,
  t0, dt, valid_until,
  [ (grid_cell, time_slot), ... ],
  priority
}
```

Every robot maintains a **local replicated reservation table** from peer intents. Its WHCA* search treats valid conflicting reservations as occupied in `(x, y, t)` space. The robot periodically refreshes its intent and replaces old plans by `plan_id` and expiry.

### 7.3 Planning details

- Plan in a rolling horizon of roughly 10–15 time steps; replan before the window is exhausted.
- Use a cached reverse-Dijkstra distance-to-goal heuristic for the static map. Cache it per goal rather than recomputing it every control tick.
- Add predicted congestion/ETA as a soft cost—not a safety constraint—to route selection.
- Trigger replanning on a new assignment, window advance, reservation conflict, blocked corridor, persistent blockage, route deviation, or reduced communication state.
- Output macro-waypoints / a preferred velocity direction. WHCA* does not replace local collision avoidance.

### 7.4 Why this serves PS 26123

Trajectory intentions give a concrete, inspectable mechanism for multi-robot coordination and reduced waiting. They make simultaneous movement through open warehouse space possible while avoiding foreseeable conflicts, supporting the required time improvement over stop-and-wait.

---

## 8. Distributed corridor mutual exclusion

### 8.1 Why naive Lamport tie-breaking is insufficient

Sorting locally visible requests by `(Lamport timestamp, robot_id)` is deterministic only if every robot has already seen the same requests. Network delay can leave two requesters with different inputs, causing both to believe they won. DDS reliability does not remove this race.

### 8.2 Robust corridor protocol

Narrow aisles, one-lane turns, lifts, and intersections are modelled as pre-defined `corridor_id` mutex resources. For a small active fleet, use a Ricart–Agrawala-style permission protocol:

```text
REQUEST(corridor_id, request_id, lamport_ts)
        ↓
Each active conflicting peer compares priority (lamport_ts, robot_id)
        ↓
GRANT(request_id) or DEFER(request_id)
        ↓
Requester receives required grants from current active participants
        ↓
Robot's local sensing confirms entrance is physically clear
        ↓
ENTER(corridor_id, occupancy_epoch)
        ↓
Traverse corridor
        ↓
EXIT(corridor_id, occupancy_epoch) / RELEASE
```

The requester may enter only after protocol permission **and** local sensing confirms a clear braking envelope. Deferred requests are released when the holder exits or cancels.

### 8.3 Lease ownership is not physical occupancy

Corridor state is explicitly separated:

```text
FREE → RESERVED → OCCUPIED → FREE
                    │
                    └─ communication/lease expiry → SUSPECT_OCCUPIED / BLOCKED
```

If a robot is inside a corridor and then fails, lease expiry removes its **network ownership**, but does not assert that the physical corridor is free. The state becomes `SUSPECT_OCCUPIED` / `BLOCKED`. It is cleared only by a valid EXIT or local sensing that confirms the corridor is physically clear.

This is a crucial safety rule:

> A network timeout may change permission; it never proves physical clearance.

### 8.4 Progress and deadlock handling

- A lease is renewed only when measured position shows progress through the resource.
- A non-progressing occupied corridor becomes blocked, triggering reroutes.
- Requests carry priority and expiry; a robot cancels a request if its route changes.
- If a cycle of deferred requests is detected, the lowest-priority requester backs off for a randomized short interval and replans. This improves liveness without a permanent coordinator.

### 8.5 Why this serves PS 26123

This is the explicit real-time conflict-resolution mechanism for choke points. It is safe under reordering and packet delay, avoids the “two robots both won” flaw, and demonstrates mature decentralised systems design.

---

## 9. Four-layer collision-prevention architecture

The project should avoid claiming an absolute collision guarantee from one algorithm. Instead, it demonstrates zero inter-robot collisions under the defined test envelope using four independent layers:

| Layer | Mechanism | Role |
|---|---|---|
| 1. Strategic | WHCA* plus trajectory reservations | Prevents known future space-time conflicts. |
| 2. Critical-space | Distributed corridor mutex | Provides exclusive permission at narrow shared resources. |
| 3. Reactive | ORCA | Generates a safe candidate velocity around nearby moving agents. |
| 4. Hard local safety | Safety Supervisor | Limits / zeros `cmd_vel` from sensor-derived braking and TTC constraints. |

### 9.1 ORCA

ORCA runs around 20 Hz and follows the macro path from WHCA*. If a peer state is predicted rather than fresh, ORCA inflates the peer's effective radius using the tracker covariance, for example `radius + kσ`. Less certain communication therefore produces more conservative spacing.

### 9.2 Safety Supervisor

The `safety_supervisor_node` sits **below ORCA and above the motor interface**. It independently evaluates:

- LiDAR/local obstacle distance;
- stopping/braking envelope based on current speed;
- time-to-collision (TTC);
- localisation validity and age;
- stale/missing velocity command;
- E-stop state and actuator health.

Typical policy:

```text
TTC below critical threshold              → stop
Obstacle inside braking envelope          → stop
Localisation invalid / command stale      → stop or controlled slow-down
Communication degraded                    → lower speed and no new corridor entry
```

The Safety Supervisor may veto a planner or ORCA command. This is intentional: local sensing is authoritative for immediate physical safety.

---

## 10. Peer tracking and degraded-mode behaviour

### 10.1 Per-neighbour Kalman filter

Each AMR tracks every peer using a constant-velocity Kalman filter with state `[x, y, vx, vy]`. On every control tick it predicts state and grows covariance with process noise. On a pose update it corrects using measurement noise based on localisation quality.

The increasing covariance is consumed by ORCA and speed policy; this makes the design a genuine uncertainty-aware tracker, not merely linear extrapolation.

### 10.2 Communication states

| Freshness | State | Fleet response |
|---|---|---|
| 0–500 ms since update | Normal | Use estimate and normal ORCA inflation. |
| 500–2,000 ms | `COMM_DEGRADED` | Inflate uncertainty envelope, reduce speed, avoid new corridor entry, continue using local sensing. |
| >2,000 ms or liveliness lost | `PEER_UNREACHABLE` | Treat predicted region conservatively; expire task ownership through protocol; re-auction unfinished work when safe. |

The disconnected robot may still be moving, so it is never simply assumed static at its last message. LiDAR/local costmap remains the immediate physical observation source.

### 10.3 Network partition policy

The fleet prioritises **safety over availability**:

- Robots may complete low-risk local motion only when their Safety Supervisor permits it.
- Robots do not acquire new corridor permissions or make new shared-space commitments without the required peers.
- Current reservations expire; affected regions are treated conservatively.
- Task consensus pauses across the partition; after reconnection, session IDs and assignment epochs remove stale ownership.

This behaviour may reduce throughput during a partition, but it prevents a network fault from becoming a collision or double-task execution event.

---

## 11. Decentralised task allocation with CBBA

### 11.1 Why replace auctioneer-less CNP

An “everyone sees all bids and independently selects a winner” rule has the same incomplete-information problem as naive corridor locking: no robot knows when it has received every bid. Traditional Contract Net also normally has an initiator/manager role.

The revised design uses **Consensus-Based Bundle Algorithm (CBBA)**, which is built for decentralised multi-vehicle allocation. If the prototype gives each robot only one active job, simplify it to CBAA; with queued pickup/delivery jobs, CBBA is the better fit.

### 11.2 Bid and consensus inputs

Each eligible AMR estimates a bid using:

```text
estimated travel time + congestion cost + battery penalty + queue/load penalty
```

The congestion cost is supplied by the edge predictor; all safety constraints remain deterministic. AMRs broadcast current bundle/winner information and iteratively resolve conflicts until the allocation converges or a bounded round expires.

### 11.3 Assignment epochs and session IDs

Every assignment contains:

```text
task_id
assignment_epoch
owner_robot_id
owner_session_id
lease_until
```

If Robot A loses connectivity and Task 17 is reassigned to Robot B at epoch 8, Robot A's old epoch-7 assignment is invalid on reconnection. It aborts that task rather than duplicate the delivery. This small detail protects against stale messages, robot reboot, and split-brain task ownership.

### 11.4 Reallocation triggers

- owner becomes `PEER_UNREACHABLE` and its ownership lease expires;
- owner reports that a persistent blockage makes the job unreachable;
- battery or safety state makes the current assignment infeasible;
- a higher assignment epoch arrives after consensus.

### 11.5 Why this serves PS 26123

CBBA directly demonstrates decentralised task assignment and automatic reallocation. It improves utilisation and makes the fleet resilient to a mid-run robot failure without introducing a central dispatcher.

---

## 12. Dynamic blockage detection and rerouting

`blockage_event` must originate from sensing, not from a manual assumption. The pipeline is:

```text
LiDAR / local costmap
        ↓
Obstacle persists for threshold duration and intersects usable route/corridor
        ↓
BlockageObservation { polygon/cells, confidence, observed_at, valid_until }
        ↓
Peer sharing and local fusion
        ↓
Temporary fleet blockage state
        ↓
WHCA* replan; task reallocation only if route remains infeasible
```

Observations have a TTL and require re-observation to persist. This prevents “blocked forever” behaviour after a pallet or person moves away. A corridor that is `SUSPECT_OCCUPIED` is also injected as a conservative blockage.

---

## 13. Edge-AI component: congestion and ETA prediction

### Purpose

The architecture uses AI where it is valuable and safe: predicting which route or robot assignment is likely to be faster. It does **not** ask an ML model to make emergency braking or collision decisions.

### `edge_congestion_predictor_node`

Runs locally on each AMR using a compact model such as a small gradient-boosted model or tiny ONNX MLP. Candidate inputs include:

- local corridor queue length and nearby robot density;
- recent corridor traversal time and average speed;
- active / expiring blockage observations;
- time of run or task destination zone;
- communication freshness / uncertainty summary.

Outputs:

- predicted route traversal time;
- congestion penalty for an edge/corridor;
- ETA contribution for task bids.

The output is a **soft cost** added to WHCA* and CBBA bid evaluation. If the model is unavailable, stale, or low-confidence, the system falls back to a static travel-time cost. The safety stack is unchanged.

### Why this serves PS 26123

It supplies measurable, on-device Edge AI without AI theatre. The team can show an ablation: static cost routing versus predictor-assisted routing, with the same safety rules and task set.

---

## 14. Passive fleet dashboard

The dashboard consumes telemetry through a read-only bridge (for example `rosbridge_websocket`) and must not be required for planning, task allocation, or safety. It should show:

- live robot pose, path, goal, battery, and connection/freshness state;
- short-horizon trajectory intents and reservation conflicts;
- corridor state: free, reserved, occupied, suspect occupied, blocked;
- task assignment, assignment epoch, queue, and reallocation events;
- blockage observations and expiry;
- ORCA interventions, safety slow-downs/stops, and E-stop status;
- throughput, makespan, mean task latency, utilisation, wait time, and collision count;
- comparison against the stop-and-wait baseline.

**Resilience demo:** disabling the dashboard does not affect AMR operation. This visibly proves it is monitoring infrastructure, not a hidden fleet controller.

---

## 15. Benchmark methodology and acceptance criteria

### 15.1 Baseline

Implement a simple, understandable baseline: at potential conflict zones, robots use stop-and-wait / first-come-first-served movement rather than reservations, distributed mutex, or congestion-aware routing.

### 15.2 Fair comparison

Run the same warehouse map, initial positions, task arrivals, robot count, speed limits, and random seeds for:

1. stop-and-wait baseline;
2. decentralised deterministic stack (WHCA* + mutex + ORCA + Safety Supervisor);
3. full stack with edge-AI congestion cost.

Use at least 10 simulation seeds per scenario; report mean, median, variance, and worst case. Include 3-robot and 5-robot runs, plus high-choke-point-density scenarios.

### 15.3 Metrics

| Metric | Why it matters to the PS |
|---|---|
| Fleet makespan / total task completion time | Primary proof of the target improvement over stop-and-wait. |
| Task completion rate and mean task latency | Shows operational throughput. |
| Inter-robot collision count | Core safety outcome; target is 0 in all validated runs. |
| Minimum separation and Safety Supervisor stops | Safety evidence beyond a single collision number. |
| Corridor wait time / lock acquisition latency | Shows choke-point conflict resolution quality. |
| Replan and task-reassignment success rate | Demonstrates dynamic adaptation and fault recovery. |
| Planner runtime and ORCA control rate | Shows edge feasibility. |
| Network-degradation outcome | Verifies graceful, conservative operation under loss/partition. |
| AI ablation improvement | Proves the predictor adds measurable value rather than being decorative. |

### 15.4 Target acceptance criteria

- Zero inter-robot collisions in benchmark and fault-injection scenarios.
- At least 20% lower mean makespan than stop-and-wait under the agreed test scenario.
- Sustained local avoidance loop at approximately 20 Hz and practical planner response on target edge hardware.
- Successful safe replan / reallocation after a blockage and a robot failure.
- Dashboard remains passive: fleet continues after it is disabled.

---

## 16. Fault-injection demonstrations

| Demo event | Expected safe response | Subsystems proven |
|---|---|---|
| Three AMRs request one corridor | Only one gains required grants; others wait/replan; local entrance check precedes entry. | Corridor mutex, safety design. |
| Robot stops inside corridor | Lease expires to `SUSPECT_OCCUPIED`, not free; peers route around / wait until sensing confirms clearance. | Occupancy-versus-lease distinction. |
| Packet loss / delayed pose messages | KF covariance grows, ORCA radius inflates, speed reduces; LiDAR remains final local protection. | Degraded-mode handling. |
| Network partition | No new shared choke-point commitment; fleet becomes conservative until peers reconnect. | Safety-over-availability policy. |
| Robot process crash mid-task | Liveliness/freshness loss; ownership lease expires; CBBA reallocates task at a newer epoch. | Self-healing allocation. |
| Pallet blocks aisle | Persistent local detection publishes TTL-bound blockage; WHCA* reroutes; assignment changes only if necessary. | Dynamic rerouting. |
| Dashboard closed | Robots continue their mission unchanged. | No hidden central control. |

---

## 17. Revised build priorities

Build the safety-critical and judge-visible core before optional polish.

| Priority | Deliverable | Definition of done |
|---:|---|---|
| P0 | Gazebo warehouse, 3 namespaced AMRs, ROS 2 networking | Robots are independently controllable and visible. |
| P0 | Localisation, LiDAR costmap, Safety Supervisor | E-stop / braking constraints can stop a robot reliably. |
| P0 | ORCA and basic peer pose sharing | Two AMRs avoid one another locally. |
| P0 | Corridor protocol and physical clearance check | Three-robot choke-point demo is repeatable and safe. |
| P1 | WHCA* plus trajectory-intent reservation table | Robots coordinate short-horizon routes in open space. |
| P1 | Peer KF, freshness states, session IDs | Packet-loss and restart handling works without stale state. |
| P1 | Blockage pipeline and rerouting | Temporary obstacle creates, refreshes, and expires blockage. |
| P1 | CBBA plus assignment epochs | Failure/blockage causes safe task reassignment. |
| P2 | Passive dashboard and metric logging | Demo metrics and protocol state are visible. |
| P2 | Stop-and-wait baseline and repeated benchmarks | The 20% claim is supported by reproducible data. |
| P2 | Edge congestion predictor and ablation | AI contribution is measured against deterministic stack. |
| Stretch | Pi/Jetson deployment, LiDAR and controlled Wi-Fi tests | Same nodes run on a physical AMR. |

---

## 18. Final SIH demo plan

1. **Start state:** show the warehouse map, 3–5 AMRs, live task queue, and dashboard. State clearly that the dashboard is passive.
2. **Normal operation:** dispatch simultaneous jobs. Show AMRs publishing intent, choosing routes, and moving concurrently rather than waiting globally.
3. **Choke point:** route three robots toward one narrow aisle. Visualise REQUEST/GRANT and show one enters after local clearance while the others receive safe alternatives or wait.
4. **Dynamic blockage:** place a virtual pallet in an aisle. Show local sensing, TTL-bound blockage announcement, and replanning.
5. **Failure:** stop one AMR mid-task or inside a corridor. Show conservative corridor state, task ownership expiry, and reassignment—without another AMR entering an uncleared corridor.
6. **Network degradation:** induce packet delay/loss. Show uncertainty state, enlarged safety space, lower speed, and continued sensor-based protection.
7. **Evidence:** display benchmark chart against the stop-and-wait baseline, collision count, task completion time, and AI ablation result.
8. **Resilience close:** disable the dashboard; robots continue. Conclude that no permanent central controller is required.

---

## 19. Why the revisions were made

| Original risk / ambiguity | Revision | Benefit to the problem statement |
|---|---|---|
| Robots were described as using different ROS domains. | All AMRs in one fleet share one `ROS_DOMAIN_ID`; namespaces separate robots. | Ensures peers can actually discover and coordinate. |
| WHCA* had no future occupancy sharing. | Add trajectory-intent messages and local reservation tables. | Makes multi-AMR planning genuinely cooperative and reduces conflicts. |
| Lamport ordering was treated as instant agreement. | Use explicit distributed REQUEST / GRANT / DEFER protocol. | Correctly resolves choke-point races under real network delay. |
| Expired corridor lease was treated as free space. | Separate reservation ownership from physical occupancy; use `SUSPECT_OCCUPIED`. | Prevents entry into a failed/stuck robot. |
| ORCA was the last safety step. | Add sensor-driven Safety Supervisor after ORCA. | Provides a credible path toward zero-collision operation. |
| A missing peer was assumed static after a short timeout. | KF uncertainty, reduced speed, no new shared-space entry, conservative region. | Makes packet loss safe rather than optimistic. |
| CNP had no clear bid-completion / stale-data rule. | Use CBBA/CBAA with task epochs and leases. | Supports decentralised allocation and failure recovery without duplicate work. |
| `robot_id` alone identified protocol participants. | Add a boot-time `session_id` and sequence numbers. | Rejects stale messages after a reboot or reconnection. |
| Critical topics used unbounded history. | Use bounded history, lifespan, durability only where state-like. | Keeps resource use practical on edge hardware. |
| DDS discovery was presented as always effortless. | Add multicast validation and static-peer fallback. | Makes hardware deployment credible. |
| Blockages had no sensing origin or expiry. | Costmap → persistence detector → TTL-bound observation. | Enables realistic dynamic rerouting. |
| “AI” was not explicit. | Add non-safety-critical local congestion/ETA predictor. | Meets Edge-AI intent with measurable impact and no safety compromise. |
| “Decentralised” was phrased as no coordination at all. | State no permanent central coordinator / single point of failure. | Accurate, defensible decentralisation claim. |
| Partition response was undefined. | Explicitly prefer safety over availability. | Demonstrates mature fault-aware fleet behaviour. |

---

## 20. Judge-facing answers

**How is this decentralised?**  
There is no permanent fleet manager or broker that owns planning, task allocation, or safety. Every robot runs the same peer protocols and retains safe local control if other nodes or the dashboard fail.

**How do you prevent two robots from entering a narrow aisle?**  
They use an explicit peer permission protocol, not just message ordering. A robot enters only after receiving required grants and verifying the corridor entrance is clear with local sensing.

**What happens when Wi-Fi fails?**  
We do not assume reliability solves a partition. Robot uncertainty increases, speed reduces, new shared-space commitments pause, and local sensing/Safety Supervisor remains authoritative.

**Where is the AI?**  
Each AMR runs a lightweight congestion/ETA predictor locally. It improves route and task-bid costs. Safety is deliberately deterministic and sensor-gated.

**How do you prove the 20% improvement?**  
We run the same seeded task sets against stop-and-wait and report multiple-run makespan, latency, collisions, and variance. The dashboard shows live and logged evidence.

**Is the dashboard a hidden coordinator?**  
No. It is read-only. We demonstrate this by turning it off during a run while the fleet continues operating.

---

## 21. Final positioning

This is not merely a simulation of robots broadcasting positions. It is a layered fleet system in which:

- **DDS** carries peer data with fit-for-purpose QoS;
- **trajectory reservations** make WHCA* cooperative;
- **distributed mutual exclusion** protects choke points under message delay;
- **CBBA** provides decentralised task allocation and recovery;
- **Kalman-based uncertainty handling** converts communication loss into conservative motion;
- **ORCA plus a local Safety Supervisor** protect the physical world;
- **edge AI** improves throughput without being trusted for safety; and
- **benchmarking and fault injection** turn claims into observable evidence.

That combination directly serves PS 26123: a decentralised, edge-executed, demonstrably safe, adaptive, and measurable multi-AMR coordination solution with a clear path from Gazebo to real warehouse hardware.
