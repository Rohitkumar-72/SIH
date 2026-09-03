# SIH 2026 PS 26123 — Master Project Guide

This master guide consolidates the production architecture, the team execution guide, and the ML/DL dataset start plan.

---

# Part I — Decentralized Multi-AMR Fleet Coordination

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
---

# Part II — SIH 2026 PS 26123 — Team Roles and Execution Guide

## Decentralised Multi-AMR Fleet Coordination

**Team size:** 5  
**Goal:** Build and demonstrate a decentralised fleet of 3+ warehouse AMRs that completes tasks faster than stop-and-wait, resolves choke-point conflicts, reroutes around blockages, reallocates work after failure, and reports results through a dashboard.

This is a practical guide for the five people building the project. It explains what each person owns, why it matters, what to learn, what to build, and how their work connects to everyone else’s.

The reference design is the companion architecture document: [SIH_2026_PS_26123_Decentralized_Multi_AMR_Fleet_Architecture.md](SIH_2026_PS_26123_Decentralized_Multi_AMR_Fleet_Architecture.md).

---

## 1. The simplest picture of the team

Think of the project as a small warehouse where robots must make good decisions and move safely.

| Team member | Primary identity in this project | Main thing they own |
|---|---|---|
| Cybersecurity member | **Trust, reliability, and failure-testing lead** | Secure and trustworthy robot communication; test what happens when the network or robot fails. |
| ML member | **Fleet intelligence lead** | Predict congestion and travel time so robots choose faster routes and better task assignments. |
| DL member | **Visual perception lead** | Detect and classify pallets/obstacles from camera data as an optional extra sensing channel. |
| Full-stack member | **Dashboard and experiment platform lead** | Build the live dashboard, logging, replay, baseline comparison, and clear demo visuals. |
| AI generalist / agent-workflow member | **Autonomy and integration lead** | Build the ROS 2 coordination core and assemble all modules into one working fleet. |

### One important rule

Each person has a **primary owner area**, but no feature is “finished” until it works with the integration lead’s simulation and appears in the dashboard. The final product is one system, not five separate mini-projects.

---

## 2. What the final system must do

Before splitting work, everyone should understand the shared mission.

```text
Tasks arrive
   ↓
Robots decide who should do each task
   ↓
Robots plan routes while sharing short future movement intentions
   ↓
At narrow aisles, robots use a peer permission protocol
   ↓
Robots avoid each other locally and stop if sensors see danger
   ↓
Blockages or failed robots cause rerouting / task reassignment
   ↓
Dashboard shows proof: speed, safety, failures, and recovery
```

The key success criteria are:

- three or more AMRs cooperate without a permanent central coordinator;
- no collisions in the defined test scenarios;
- clear real-time conflict resolution at a narrow corridor;
- automatic safe response to blockage, packet loss, and a failed AMR;
- a measured improvement of at least 20% in task completion time versus stop-and-wait;
- a visible and credible Edge-AI contribution.

---

## 3. Ownership map: who builds what

| System component | Primary owner | Support owner | What “done” looks like |
|---|---|---|---|
| Gazebo warehouse, AMR namespaces, base ROS 2 launch | AI generalist | Full-stack | Three robots start with unique names and publish basic state. |
| WHCA* planner, trajectory intents, reservation table | AI generalist | ML | Robots avoid planned future conflicts in the same map. |
| Corridor REQUEST / GRANT / ENTER / EXIT protocol | AI generalist | Cybersecurity | Exactly one robot enters a single-lane corridor in repeated three-robot tests. |
| ORCA and Safety Supervisor | AI generalist | Cybersecurity | Sensor-based stop/slow rule can override movement command. |
| Peer health, session IDs, message checks, QoS, fault injection | Cybersecurity | AI generalist | Stale/replayed messages are rejected; loss/failure tests behave safely. |
| CBBA task allocation and reassignment | AI generalist | ML | Tasks converge to one owner and reassign safely after failure. |
| Congestion / ETA predictor | ML | Full-stack, AI generalist | Model predicts route cost and improves a measurable metric versus static cost. |
| Camera-based pallet / obstacle recognition | DL | AI generalist | Vision output is published with confidence and is never the sole safety input. |
| Dashboard, logs, replay, results charts | Full-stack | All | Dashboard visualises real system topics and test evidence. |
| Baseline and benchmark report | Full-stack | ML, AI generalist | Same seeded tasks run in baseline and fleet mode with saved metrics. |
| Pitch explanation, demo script, architecture diagrams | All; coordinated by AI generalist | Full-stack | Every team member can explain their own module and its PS value. |

---

## 4. Shared ways of working

### 4.1 A message is a contract

When one person publishes data for another, write down:

- message name and fields;
- which module publishes it;
- which module consumes it;
- update rate / expiry time;
- what a receiver does when it is missing or invalid.

Example:

```text
Message: /fleet/blockage_observation
Publisher: blockage detector (AI generalist) or vision detector (DL member)
Consumer: WHCA* planner and dashboard
Fields: cells/polygon, confidence, source robot, observed_at, valid_until
If missing: planner uses normal static map
If expired: planner removes temporary blockage
```

This habit prevents most integration confusion.

### 4.2 Safety rule for all roles

No ML, DL, dashboard, or network message is allowed to directly command motors. The final `cmd_vel` must pass through the local Safety Supervisor. A feature may suggest “this route is faster” or “there may be a pallet,” but the robot’s local sensors decide if it can move safely.

### 4.3 Definition of done for any feature

A feature is done only when it has all five:

1. a short README explaining what it does;
2. a small repeatable test or simulation scenario;
3. logs or a dashboard view proving it worked;
4. a safe fallback if it crashes or returns no data;
5. a clean handoff note describing inputs and outputs.

### 4.4 Weekly rhythm

- **Start of week:** choose one testable outcome per person.
- **Mid-week:** 15-minute integration check; merge only small working pieces.
- **End of week:** run the same shared scenario and record results.
- **Every member:** explain one issue, one metric, and one next task in plain language.

Do not leave integration for the final week.

---

# 5. Role guide — Cybersecurity and reliability lead

## Your mission

Make sure the robots only act on believable, current messages and behave safely when communication is weak, delayed, or missing. You are not expected to build the navigation algorithm. Your job is to make the system’s communication and failure behaviour **trustworthy and demonstrable**.

## What you will build

### A. Robot identity and message-validity checks

Create a common message wrapper or shared validation helper used by coordination topics. It should check:

```text
robot_id       Which physical/logical robot sent this?
session_id     Which boot/run instance sent this? New random ID at every boot.
sequence_no    Is this message newer than the last one?
sent_at        When was it created?
valid_until    Should this data already be ignored?
```

**Example:** Robot 3 crashes, restarts, and begins publishing again. An old “I own Task 17” message from Robot 3’s previous run arrives late. The new `session_id` tells every other robot: “this message belongs to an old instance; ignore it.”

### B. DDS QoS and connection-health configuration

Work with the integration lead to use the right communication settings:

- pose updates: latest-only, best effort, short lifespan;
- trajectory intent: reliable, current state only, short expiry;
- corridor protocol and task consensus: reliable, bounded queue, explicit protocol acknowledgements;
- liveliness and deadline monitoring for peer health.

Your job is not to say “reliable DDS solves everything.” Your job is to document the difference between transport delivery and agreement between robots.

### C. Safe degraded-mode and network-partition tests

Create repeatable test profiles such as:

| Test | What you simulate | Expected result |
|---|---|---|
| Mild packet loss | Some pose messages missing | Uncertainty grows; robots give more space. |
| Delayed messages | Old positions arrive late | Expired / out-of-order messages are ignored. |
| Peer disconnect | One robot stops publishing | Others slow down, avoid new narrow corridors, and reassign task only after expiry. |
| Partition | Two groups cannot communicate | No new shared-space permission across the partition; robots act conservatively. |
| Robot restart | Same `robot_id`, new session | Old reservations and assignments are rejected. |

### D. Lightweight security posture

Prepare a practical threat model and secure-demo checklist. At minimum cover:

- only approved devices join the test network;
- robot network is separated from public Wi-Fi where possible;
- unnecessary ports/services are disabled;
- dashboard has no motor-control endpoint;
- critical messages are validated for identity, freshness, and format;
- credentials/configuration are never committed to the repository.

If time allows, investigate ROS 2 security / DDS-Security (enclaves, authentication and access control) and apply it in a small, tested scope. Do not let advanced certificates delay the core demo.

## What to learn first

1. ROS 2 topics, namespaces, QoS, deadline, lifespan, and liveliness.
2. What packet loss, latency, reordering, and a network partition mean.
3. The difference between authentication, authorisation, message freshness, and safety.
4. Basics of DDS-Security / SROS2 after the core communication path works.
5. Threat modelling: assets, attackers, failures, mitigations, residual risk.

Useful research questions:

- What QoS combination is appropriate for state messages versus protocol events?
- How should a receiver handle a stale but correctly formatted message?
- What can DDS liveliness detect, and what can it not prove?
- Why must a corridor not become “physically free” just because a lease expired?

## First three tasks

1. Write a one-page threat model and failure matrix for the simulated fleet.
2. Define the shared header fields and implement a unit test for stale / replayed messages.
3. Work with the integration lead to induce packet loss and verify that the fleet enters the expected degraded state.

## Your deliverables

- `security-and-failure-model.md` in simple language;
- shared message validation library / helper;
- QoS configuration table and rationale;
- fault-injection scripts or reproducible scenario steps;
- a one-slide “safe under communication failure” explanation for the pitch.

## How your work helps the PS

It makes decentralisation credible. A decentralised fleet cannot depend on perfect communication, so your work proves that packet loss, delay, restart, and failure result in safer, more conservative behaviour rather than collisions or duplicate tasks.

## Avoid these traps

- Do not block all development waiting for full cryptography / certificates.
- Do not claim that encryption itself prevents collisions; local sensing and the Safety Supervisor do that.
- Do not treat a missed heartbeat as proof that a physical corridor is empty.

---

# 6. Role guide — Machine-learning lead

## Your mission

Build the project’s **useful Edge-AI component**: a small on-device model that predicts congestion or travel time. It should help robots choose faster paths and smarter task assignments. It must be optional: if it fails, the fleet still works using normal static costs.

## What you will build

### A. Fleet-data logger and training dataset

Define one row per route or corridor traversal, for example:

| Feature | Simple meaning |
|---|---|
| corridor ID / route segment | Where is the robot travelling? |
| time / scenario ID | Which run or demand condition is this? |
| number of nearby robots | How crowded is the area? |
| corridor queue length | How many robots are waiting? |
| recent average speed | Is movement already slow? |
| recent traversal time | How long did this segment recently take? |
| blockage active | Is there a temporary obstacle? |
| task destination zone | Where is the robot trying to go? |
| target: actual travel time | What should the model predict? |

Start by logging synthetic data from Gazebo. You do not need real warehouse data to prove the pipeline.

### B. A simple baseline before a fancy model

Build in this order:

1. static map distance only;
2. simple rule: add a penalty when corridor queue is long;
3. linear regression / random forest / gradient-boosted regressor;
4. only then consider a small neural model if it clearly improves validation results.

For this project, a small tree-based model is often easier to train, explain, and deploy than a deep network. “AI” does not need to mean “largest model.”

### C. Model service for the fleet

Expose a small interface:

```text
Input: candidate route features
Output: predicted_time, confidence, model_version
Fallback: static travel-time estimate
```

The integration lead adds `predicted_time` as a **soft cost** in WHCA* and CBBA bidding. You do not command a robot; you estimate which option is likely faster.

**Example:** Two routes both reach the pickup shelf. Route A is shorter on the map but has three robots queued at a narrow aisle. Your model predicts Route A = 45 seconds and Route B = 36 seconds. The planner may select B. The Safety Supervisor remains unchanged.

### D. AI ablation proof

Run the exact same scenarios with:

- static costs only;
- rule-based congestion penalty;
- your trained model.

Report prediction error and fleet outcome. The most valuable result is not “99% model accuracy”; it is “the model reduced average makespan / wait time without increasing safety incidents.”

## What to learn first

1. Regression basics: target, features, train/validation/test split, MAE and RMSE.
2. Time leakage: never train using information that would not be available before the robot chooses its route.
3. Feature engineering for queueing/congestion data.
4. Model calibration and fallback when confidence is low.
5. Small-model deployment: serialised model or ONNX inference on edge hardware.

Useful research questions:

- Which observable features best predict corridor delay?
- Does the model generalise to a new task seed or map layout?
- Does an AI model beat a transparent queue-length rule enough to justify using it?
- What happens when the model has no data for a situation?

## First three tasks

1. Agree on the log schema with the full-stack and integration leads.
2. Collect 20–50 simulated runs and build a static-distance baseline.
3. Train a simple regression model and produce one graph of predicted versus actual route time.

## Your deliverables

- data schema and data-quality checklist;
- training notebook/script and saved reproducible model;
- inference node/API with static-cost fallback;
- comparison chart: baseline, rule-based, and ML-assisted results;
- simple one-minute pitch explanation of the Edge-AI value.

## How your work helps the PS

It gives a clear answer to “where is the Edge AI?” The model runs locally, improves ETA and congestion choices, and is measured through an ablation. This improves the required throughput objective without placing AI in a safety-critical decision.

## Avoid these traps

- Do not train on test runs and then report those same runs as “accuracy.”
- Do not use a model output as an emergency-stop or collision-avoidance decision.
- Do not spend weeks on a deep model before proving a simple baseline.

---

# 7. Role guide — Deep-learning and perception lead

## Your mission

Create an **optional camera-based perception module** that recognises relevant warehouse objects such as pallets, cartons, people, or blocked lanes. It gives the fleet richer awareness, but it must never replace LiDAR or the Safety Supervisor.

This role complements the ML lead. The ML lead predicts operational delay from fleet data; you recognise visual objects in sensor images.

## What you will build

### A. Decide the smallest useful vision use case

Choose one or two classes that strengthen the demo:

- pallet / carton obstructing an aisle;
- person / worker in a warehouse zone;
- pickup/drop shelf marker or package type.

For SIH, **pallet/obstacle classification** is the most directly useful. LiDAR says “something is there”; vision can say “it looks like a pallet / carton” with a confidence score.

### B. Perception pipeline

```text
Camera image
    ↓
Object detector
    ↓
Bounding box + class + confidence
    ↓
Optional depth / map projection
    ↓
VisionObservation message
    ↓
Blockage detector / dashboard
```

Suggested output:

```text
VisionObservation {
  robot_id, session_id, timestamp,
  class_name, confidence,
  image_bbox,
  estimated_map_region (optional),
  valid_until
}
```

### C. Use a proven small model first

Start with a lightweight pretrained detector that can run on the available machine. Fine-tune only if the baseline does not recognise the selected classes well enough. For simulation, create controlled images from Gazebo or use a small labelled image set. For hardware, test under warehouse-like lighting.

The goal is not to invent a new detector. The goal is to integrate a reliable, explainable perception result into the AMR system.

### D. Fusion rule with LiDAR

The safe integration rule is:

```text
LiDAR / costmap detects physical obstacle  → Safety Supervisor can slow/stop
Vision confirms / classifies obstacle      → Helps dashboard and persistent blockage logic
Vision alone sees uncertain obstacle        → Mark as low-confidence observation; do not force unsafe motion
```

**Example:** A camera sees a pallet-shaped object in a corridor with 92% confidence and LiDAR sees an obstacle in the same region for several seconds. The system publishes a high-confidence temporary blockage, and the planner reroutes. If the camera is dark or unavailable, LiDAR-based safety still works.

## What to learn first

1. Image classification versus object detection versus segmentation.
2. Bounding boxes, confidence threshold, false positive, false negative, precision, recall, and mAP.
3. Dataset labelling and train/validation/test split.
4. Lightweight inference and model-size / latency trade-offs.
5. Camera calibration or simple map projection if you need to locate detections in the warehouse grid.

Useful research questions:

- Which object classes can the chosen pretrained model reliably see?
- At what confidence should we display a detection versus create a temporary blockage observation?
- How do lighting and camera angle affect false detections?
- Can the inference loop run fast enough on the target laptop / Jetson?

## First three tasks

1. Select one detection model and make it work on saved test images.
2. Prepare a small, labelled pallet/obstacle evaluation set and measure basic precision/recall.
3. Publish a sample `VisionObservation` and have the dashboard display it.

## Your deliverables

- short model-selection note with speed and accuracy trade-off;
- labelled mini-dataset or reproducible simulated image generator;
- perception node that publishes `VisionObservation`;
- evaluation table with false positives, false negatives, and inference latency;
- demo clip or dashboard screenshot of a detected blockage.

## How your work helps the PS

It strengthens the “Edge AI” and real warehouse awareness story. It makes the demo richer by showing how a robot can identify a physical obstruction, while keeping safety anchored in local LiDAR and braking logic.

## Avoid these traps

- Do not make the project depend on camera detection before the core LiDAR-based blockage path works.
- Do not claim a vision model is perfect; show confidence and tested limitations.
- Do not overlap with the ML lead by building another congestion predictor.

---

# 8. Role guide — Full-stack and dashboard lead

## Your mission

Make the project visible, measurable, and easy for judges to understand. Build the dashboard and the experiment/reporting path that turns robot activity into proof: tasks completed, time saved, safe stops, corridor decisions, and recovery from failures.

The dashboard is **read-only**. It must never be needed for robot coordination or motor control.

## What you will build

### A. Live fleet dashboard

Build a simple web app connected to ROS 2 telemetry through a bridge. It should show the map and the live state of every AMR.

Minimum panels:

| Panel | What the judge sees |
|---|---|
| Warehouse map | Robot position, path, task goal, and temporary blocked cells. |
| Robot cards | Battery, current task, connection state, safety state, last update. |
| Corridor panel | Free / reserved / occupied / suspect occupied / blocked state. |
| Task panel | Task owner, assignment epoch, queue, completion and reassignment events. |
| Safety/event feed | ORCA interventions, Safety Supervisor stops, packet-loss/failure events. |
| Results panel | Makespan, average task time, wait time, collision count, improvement versus baseline. |

**Example:** During the corridor demo, a judge sees Robot 1 marked `OCCUPIED`, Robot 2 waiting with a deferred request, and Robot 3 rerouting. This makes the decentralised protocol understandable without reading code.

### B. Logging and replay

Create a common experiment log format. Every scenario run should save:

- seed, map, robot count, tasks, code/config version;
- start and completion time for every task;
- corridor request/grant/enter/exit times;
- replan, blockage, safety-stop, and reassignment events;
- collision and minimum-separation measurements;
- network impairment profile and model version.

Provide a replay or post-run screen so the team can debug a result after the robots stop moving.

### C. Benchmark and comparison view

Help the team run identical scenarios in baseline and fleet mode. Create clear charts:

- stop-and-wait makespan versus proposed fleet makespan;
- average corridor waiting time;
- total tasks completed;
- collision count and safety interventions;
- static route cost versus ML-assisted route cost.

The visual should use exact labels and units. Never use a vague “faster” claim when a chart can show seconds and percentage improvement.

### D. Demo controls that do not control the fleet

The UI may let the team start a **predefined simulation scenario** or select a replay. Avoid a dashboard button that directly sends movement commands. This preserves the honest claim that the dashboard is passive and the fleet is decentralised.

## What to learn first

1. WebSocket / ROS bridge basics and how to subscribe to, not command, topics.
2. A mapping/canvas library or simple SVG/HTML canvas rendering.
3. Real-time state management: latest message replaces stale state.
4. Logging schema, CSV/JSON, and simple charts.
5. Human-centred dashboard design: show the few states judges need, not every internal variable.

Useful research questions:

- What is the minimum visual set that proves the system’s value in 30 seconds?
- How will a dashboard make `SUSPECT_OCCUPIED` visibly different from `FREE`?
- Which metrics must be logged to make the 20% improvement reproducible?
- How will the UI behave if a topic temporarily disappears?

## First three tasks

1. Draw a static warehouse map with three mock robots and mock task cards.
2. Subscribe to one live pose topic and update a robot marker.
3. Agree on a shared CSV/JSON log schema and make one makespan comparison chart from sample data.

## Your deliverables

- responsive read-only dashboard;
- event timeline and robot/corridor/task visualisations;
- common experiment logger and results exporter;
- benchmark comparison charts;
- a short dashboard walkthrough for the demo presenter.

## How your work helps the PS

The problem asks for fleet visibility and measurable improvement. Your dashboard makes decentralisation, safety decisions, task reallocation, and the 20% benchmark claim understandable and credible to evaluators.

## Avoid these traps

- Do not build a beautiful UI with mock data after live ROS integration is available; use real data early.
- Do not make the UI a hidden task allocator or motor-control service.
- Do not show only positions; show the reason behind waiting, rerouting, and safety stops.

---

# 9. Role guide — AI generalist / agent-workflow and integration lead

## Your mission

You are the **autonomy and integration lead**. You own the actual robot coordination loop and make all modules work together in ROS 2. Your “agent workflow” experience is useful for breaking a complex system into state machines, interfaces, tests, and repeatable workflows—not for putting an LLM in charge of vehicle motion.

Your core responsibility is to keep the project buildable, safe, and integrated. This is the central technical role, but it is not a “central coordinator” inside the fleet.

## What you will build

### A. ROS 2 simulation foundation

- Gazebo warehouse map with racks, open space, and at least one narrow corridor/intersection.
- Three or more AMRs with unique namespaces and frames.
- Shared fleet domain, launch files, and basic pose/command topics.
- A repeatable task scenario generator.

**Example:** Run one launch command and see `/robot_1`, `/robot_2`, and `/robot_3` independently publishing pose and receiving separate goals.

### B. Core coordination state machines

Implement the deterministic parts of the architecture:

1. **Trajectory intent and reservation table:** each robot publishes its short future path; peers reserve those cells locally.
2. **WHCA* planning:** use a rolling horizon, cached distance heuristic, and replan triggers.
3. **Corridor mutual exclusion:** REQUEST / GRANT / DEFER / ENTER / EXIT, with local entrance-clear check.
4. **CBBA task allocation:** assignment convergence, leases, epochs, and reallocation trigger.
5. **Peer tracker:** Kalman prediction/correction, freshness state, and links to safety policy.
6. **ORCA plus Safety Supervisor:** compute a candidate velocity, then apply sensor-derived hard limits before `cmd_vel`.

Build them as small nodes or clearly separated modules. Do not create one giant script.

### C. Integration contracts

You are responsible for agreeing and maintaining interfaces with the other four roles. For example:

| From | To | Contract |
|---|---|---|
| ML predictor | WHCA*/CBBA | predicted ETA and confidence; static fallback if unavailable. |
| DL perception | blockage detector/dashboard | class/confidence/region; LiDAR remains safety authority. |
| Cybersecurity module | all coordination nodes | validation status, peer freshness, session/sequence rules. |
| Core fleet nodes | full-stack dashboard | read-only state/event messages and log schema. |

### D. Integration test scenarios

Maintain small scenarios that prove one behaviour at a time:

- two robots crossing in open space;
- three robots competing for one corridor;
- a blocked aisle causing reroute;
- stopped robot in corridor;
- packet-loss / peer-disconnect state;
- task reassignment;
- comparison against stop-and-wait.

## What to learn first

1. ROS 2 fundamentals: packages, nodes, topics, services/actions, launch files, namespaces, TF.
2. Gazebo / Nav2 basics, occupancy grids, costmaps, and `cmd_vel` control.
3. A* / WHCA* concepts: state space `(x, y, time)`, reservations, heuristic, rolling horizon.
4. ORCA basics and why it is a candidate-velocity layer, not an absolute safety proof.
5. Distributed state machines: request, acknowledgement, timeout, epoch, and cancellation.
6. CBBA at a conceptual level before coding all details.

Useful research questions:

- What exactly is a reservation in space-time and when does it expire?
- Which messages require acknowledgement, and which are state updates?
- What is the safe fallback when a planner / ML service / peer becomes unavailable?
- How can we prove the dashboard does not influence robot operation?

## First three tasks

1. Create the 3-AMR simulation, namespaces, and a shared map with a narrow aisle.
2. Implement a simple stop-and-wait baseline first; it gives the team a working comparison and a motion foundation.
3. Add one corridor protocol test before attempting full WHCA* + CBBA integration.

## Your deliverables

- ROS 2/Gazebo project structure and documented launch command;
- core planning, corridor, allocation, and safety nodes;
- published message definitions and integration notes;
- repeatable simulation scenarios and automated smoke tests;
- architecture diagram and technical explanation for final pitch.

## How your work helps the PS

You deliver the core: decentralised peer coordination, real-time conflict resolution, safe motion, dynamic rerouting, and task reassignment. You also make sure the ML, DL, cybersecurity, and dashboard work become a single useful system.

## Avoid these traps

- Do not use an LLM/agent workflow to choose real-time motor commands. Latency and unpredictability are wrong for this safety path.
- Do not try to implement every advanced algorithm before a small end-to-end baseline moves three robots.
- Do not leave all integration to the end. Merge a working interface from each teammate early.

---

## 10. Collaboration handoffs

Use these exact handoffs to keep work unblocked.

| Handoff | Producer | Consumer | Simple example |
|---|---|---|---|
| `PredictedRouteCost` | ML | Integration lead | “This aisle is estimated to take 42 seconds, confidence 0.76.” |
| `VisionObservation` | DL | Integration lead + dashboard | “Pallet-like object detected near aisle C, confidence 0.92.” |
| `MessageValidation` / `PeerHealth` | Cybersecurity | All coordination nodes | “Robot 2’s last state is stale; do not enter a new corridor.” |
| `FleetState` / `FleetEvent` | Integration lead | Full-stack | “Robot 1 received corridor grant / Robot 3 reassigned Task 7.” |
| `ExperimentResult` | Full-stack | ML + whole team | “Seed 12: baseline 110s, full system 84s, 0 collisions.” |

### Minimal shared vocabulary

Everyone should use these words consistently:

- **Reservation:** a short future claim on a grid cell/time slot; it expires.
- **Corridor permission:** network permission to attempt entering a narrow resource; not proof it is physically empty.
- **Occupied:** physically in a corridor.
- **Suspect occupied:** a robot may still be physically there; treat it as blocked.
- **Assignment epoch:** version number for task ownership; higher epoch wins.
- **Session ID:** unique ID for one run of a robot process; restart means new session.
- **Safety Supervisor:** final local layer that can stop the robot regardless of planned action.

---

## 11. Suggested build sequence for the whole team

This order protects the team from spending too long on features that cannot yet be demonstrated.

### Milestone 1 — Working movement and visibility

- Integration lead: three AMRs move in Gazebo; simple stop-and-wait behaviour.
- Full-stack: static dashboard with live robot poses.
- Cybersecurity: message header / freshness plan.
- ML: log schema and static-distance baseline.
- DL: detector runs on saved sample images.

**Demo checkpoint:** Three robots and their tasks are visible live.

### Milestone 2 — Safe coordination at a choke point

- Integration lead: corridor protocol plus Safety Supervisor.
- Cybersecurity: packet delay / stale-message tests.
- Full-stack: corridor state/event timeline.
- ML: collect initial congestion/traversal data.
- DL: publish one visual observation to the dashboard.

**Demo checkpoint:** Three robots approach one aisle; only one enters, and the UI explains why.

### Milestone 3 — Smarter fleet behaviour

- Integration lead: trajectory reservations, WHCA*, task allocation, rerouting.
- ML: deploy ETA predictor with static fallback.
- DL: combine visual observation with non-safety blockage evidence.
- Cybersecurity: session IDs, partition / failure behaviour.
- Full-stack: task epochs, blockage, and metrics visualisation.

**Demo checkpoint:** Block an aisle and fail a robot; fleet reroutes/reassigns safely.

### Milestone 4 — Evidence and pitch

- Full-stack + ML: repeat seeded benchmark and AI ablation charts.
- Integration + cybersecurity: fault injection and final safe-state tests.
- DL: perception limitations and latency measured honestly.
- Everyone: practise their own 60-second explanation.

**Demo checkpoint:** Baseline comparison, 0-collision test record, and full failure demo work end to end.

---

## 12. Individual weekly self-check

Each member should be able to answer these questions every week:

1. What did my module do in the shared simulation this week?
2. What input does it need, and from whom?
3. What output does it produce, and who consumes it?
4. What happens if my module is unavailable or wrong?
5. Which metric proves it helped the PS solution?
6. Can I explain it to a judge without jargon in 30 seconds?

If the answer to question 1 is “I trained/read/built locally but it is not connected yet,” the next task should be integration, not more isolated work.

---

## 13. Final role summaries for the pitch

| Member | One-sentence explanation |
|---|---|
| Cybersecurity / reliability | “I made robot communication trustworthy and tested the fleet’s safe behaviour during loss, delay, restart, and partition.” |
| ML | “I built the edge model that predicts congestion and ETA so the fleet selects faster routes and better task owners.” |
| DL | “I added optional visual recognition for warehouse obstacles/pallets, while keeping LiDAR and local safety in charge of stopping.” |
| Full stack | “I built the passive dashboard and experiment system that makes fleet decisions, safety events, and benchmark improvements visible.” |
| AI generalist / integration | “I built and integrated the decentralised ROS 2 coordination stack: planning, corridor permissions, task allocation, and local safety.” |

---

## 14. The team’s final standard

The strongest version of this project is not the one with the most buzzwords. It is the one where every claim can be shown live:

- AI improves a measurable routing decision.
- Robots coordinate without a permanent controller.
- A corridor conflict is resolved correctly even with message delay.
- A local safety layer stops danger even if the network is poor.
- A failed robot does not cause a collision or duplicate task.
- The dashboard proves the outcome but is not required for it.

If every member completes their role as described here, the team will have a coherent SIH solution rather than a collection of disconnected features.
---

# Part III — ML and DL Dataset & Start Plan

## SIH 2026 PS 26123 — Decentralised Multi-AMR Fleet

This guide answers the first practical question for the ML and DL members: **“Where do I get data, and what do I build first?”**

The two members should not look for the same dataset, because they are solving different problems.

| Person | Their question | Best data source |
|---|---|---|
| ML member | “Which route / task assignment will take less time?” | **Your own Gazebo fleet logs**: robot positions, queue lengths, blockages, and actual travel times. |
| DL member | “What object is the camera seeing?” | **Public warehouse/pallet images + your own simulated camera images + a small real test set.** |

---

## 1. First: keep the scopes separate

### ML member — fleet congestion prediction

The ML model is a small regression model. Its job is to predict a number:

```text
“If a robot takes this route now, how many seconds will it likely take?”
```

It uses fleet state, not camera pixels. Example inputs are number of robots nearby, queue length at a corridor, recent speed, and active blockage state.

### DL member — visual object detection

The DL model is a camera model. Its job is to identify objects:

```text
“This image region contains a pallet / carton / person.”
```

It outputs a class, a bounding box, and a confidence score. It can help label an observed blockage, but it does not decide how fast a robot moves or when it stops.

---

# 2. ML member: exact starting plan

## 2.1 Do not search for a public “fleet congestion dataset” first

For this project, a public dataset is unlikely to match your map, corridor geometry, robot speeds, task generation, and coordination rules. The correct ML dataset is generated by **your own simulation**.

This is an advantage, not a weakness: you know the ground-truth travel time for every route, and you can make thousands of controlled examples by changing task load and robot placement.

## 2.2 What one ML dataset row looks like

Create one row for every completed route segment or full task trip. Start simple: one row = one traversal from start to goal.

| Column | Example | Why it matters |
|---|---:|---|
| `run_id` | `seed_042` | Lets the team reproduce a result. |
| `robot_id` | `robot_2` | Helpful for debugging; do not let model overfit to ID. |
| `start_zone` / `goal_zone` | `A` / `D` | Describes route context. |
| `static_path_length_m` | `18.4` | Basic map distance. |
| `candidate_corridor_count` | `2` | More critical passages can mean more delay. |
| `mean_nearby_robot_count` | `2.1` | Captures local crowding. |
| `max_corridor_queue_length` | `3` | Strong congestion signal. |
| `recent_corridor_travel_time_s` | `14.6` | Shows whether that aisle is already slow. |
| `active_blockage_count` | `1` | Captures temporary obstruction. |
| `mean_peer_freshness_ms` | `120` | Optional network uncertainty signal. |
| `task_load_count` | `4` | Represents fleet demand. |
| `actual_travel_time_s` **(target)** | `31.8` | Number the model should learn to predict. |

Start with 6–8 features. More fields can be added only after the basic pipeline works.

## 2.3 The first week: build data before models

### Day 1 — agree on the log contract

Meet the integration and full-stack members for 20 minutes. Agree that every task run logs:

```text
run ID, random seed, start time, goal time, route distance,
queue length, nearby robots, blockage state, and actual duration
```

Save one CSV file per run, then combine them later.

### Day 2–3 — generate controlled simulation data

Ask the integration lead to run variations of the same warehouse:

- 3 robots, low task load;
- 3 robots, high task load;
- 5 robots, low/high task load;
- one corridor intentionally congested;
- temporary pallet blockage;
- normal network and mild packet loss.

Each run should use a known random seed. Re-run the same seed if a result looks suspicious.

### Day 4 — make a transparent baseline

Before training, predict travel time using only static map distance:

```text
predicted travel time = static path length / nominal speed
```

Measure MAE (mean absolute error): on average, how many seconds away is this simple estimate from reality?

### Day 5 — make the first model

Train a very small regression model using the simulation CSV:

- first choice: linear regression;
- next choice: random forest or gradient boosting;
- deep neural network only if a simple model clearly cannot improve the result.

The aim is not to impress with architecture complexity. The aim is to show that congestion signals produce better ETA predictions than static distance alone.

## 2.4 How much simulation data is enough?

For a first working model, aim for **a few hundred completed route examples**, not millions. For example, 40–60 simulation runs with multiple tasks per run may be enough to prove the pipeline. Then expand only if validation error is unstable.

Keep test scenarios separate:

```text
Training runs: seeds 1–40
Validation runs: seeds 41–50
Final test runs: seeds 51–60
```

Never mix rows from the same simulation run across training and test. Otherwise the model can accidentally memorise a run instead of learning a useful pattern.

## 2.5 Model choice: recommended order

| Stage | Model | Why |
|---|---|---|
| Baseline | map distance / nominal speed | Must be beaten; easy to explain. |
| Rule baseline | distance + fixed queue penalty | Shows whether ML is better than common sense. |
| First ML model | gradient-boosted trees or random forest | Strong for tabular data, fast, explainable. |
| Optional | tiny neural network exported to ONNX | Only use if it materially improves the test result. |

## 2.6 What the ML module publishes

Keep the interface tiny:

```text
RouteCostEstimate {
  route_id,
  predicted_travel_time_s,
  confidence,
  model_version,
  fallback_used
}
```

The integration lead can add `predicted_travel_time_s` to path cost and CBBA bids. If the model is unavailable or confidence is low, set `fallback_used = true` and use static travel time. The robot must remain fully functional.

## 2.7 ML success criteria

The ML member is successful when they can show all of these:

- the model beats static-distance ETA on unseen simulation seeds;
- model inputs exist at decision time (no future-data leakage);
- prediction inference is quick enough to run locally;
- route/task decisions improve mean wait time or fleet makespan in an ablation;
- the system safely falls back to static cost when the model is absent.

## 2.8 ML research checklist

- regression metrics: MAE, RMSE, residual plots;
- time-series / split-by-run data leakage;
- tabular models: linear regression, random forest, gradient boosting;
- feature importance / SHAP-style explanation, if time permits;
- ONNX or simple local model serialization for edge inference;
- queueing theory basics: why traffic increases delay nonlinearly.

---

# 3. DL member: exact dataset and starting plan

## 3.1 Start with one useful vision problem

Choose a small class list:

```text
1. pallet
2. carton / box
3. person (optional)
```

For the SIH demo, `pallet` and `carton/box` are enough. More labels create more annotation and training work without necessarily improving the fleet demo.

## 3.2 Recommended dataset strategy: three layers

Do not rely on one public dataset alone. Use this order:

| Layer | Source | Purpose |
|---|---|---|
| 1. Public bootstrap | Licensed warehouse/pallet detection dataset | Start training / test the full pipeline quickly. |
| 2. Simulation match | Images from your Gazebo warehouse camera | Teach the model your exact robot viewpoint, aisle, lighting, and objects. |
| 3. Reality check | Small set of phone/robot-camera photos from your intended environment | Check whether the model generalises beyond simulation. |

### Public starting option

Use the [Warehouse Pallet dataset](https://universe.roboflow.com/industrial-inspection/warehouse-pallet) as a bootstrap. It has roughly 1,000 images and labels including boxes, cartons, packages, damaged boxes, and pallets under a CC BY 4.0 licence. Verify the licence and retain attribution in your project notes before using it. It is enough to validate the training pipeline; it is not enough to claim final performance in your warehouse. [Dataset page](https://universe.roboflow.com/industrial-inspection/warehouse-pallet)

For discovery of alternatives, search [Roboflow Universe](https://universe.roboflow.com/) for **pallet**, **warehouse**, **forklift**, and **cardboard box**, then check each dataset’s licence, class definitions, image quality, and viewpoint before downloading. The platform’s documentation specifically advises checking the licence on the dataset’s project page. [Roboflow dataset guidance](https://docs.roboflow.com/universe/find-a-dataset-on-universe)

### A large optional supplement

[Open Images](https://storage.googleapis.com/openimages/web/download_v4.html) can provide additional general-object images and bounding-box labels, but it is broad rather than warehouse-specific. Use it only to supplement classes you need; do not download huge amounts of irrelevant data. [Open Images description](https://storage.googleapis.com/openimages/web/factsfigures_v4.html)

## 3.3 Create your own Gazebo image set

Your model must see images resembling the final demonstration. The integration lead should place an RGB camera on one simulated AMR and provide an image topic. The DL member collects frames while varying:

- pallet/carton position: centre, edge, partly hidden;
- distance and viewing angle;
- lighting / shadows;
- empty aisle and crowded aisle;
- different pallet/carton colours and textures;
- robot motion blur if relevant.

Use simulation ground truth where possible to label images cheaply. If not available, label a small set manually with a tool such as CVAT or Roboflow. Do not label every near-identical video frame; choose diverse frames.

### Minimum realistic dataset target

For a proof-of-concept:

- 300–600 diverse labelled images across the selected classes;
- include at least 20–30% negative images with no target pallet/box;
- reserve 15–20% for validation and 15–20% for final testing;
- split by **scene/run**, not random adjacent video frames.

The exact number is less important than diversity. Fifty nearly identical pallet images are less valuable than twenty views with different distance, lighting, and occlusion.

## 3.4 The first week: make vision visible before training deeply

### Day 1 — define the labels

Write one sentence per class:

```text
pallet: wooden/plastic pallet or stacked pallet blocking/occupying an aisle
carton: movable cardboard box/carton visible in the warehouse
person: human worker; optional class, only if there is time
```

Decide what is *not* labelled. For example, a shelf is not a carton.

### Day 2 — run a pretrained detector on examples

Use a lightweight pretrained detector on 20–30 downloaded or simulated images. Save the annotated outputs. This immediately tells you whether the class choice is practical.

### Day 3–4 — prepare the dataset

- Download one licensed public warehouse/pallet dataset.
- Add your first simulated camera images.
- Check every label format and class name.
- Create train, validation, and test splits by scene.

### Day 5 — train a small detector and measure it

Fine-tune a lightweight detector. Record:

- precision: of boxes predicted, how many were right?
- recall: of real objects, how many were found?
- false positives: what was wrongly called a pallet?
- inference latency: how long per image?

Do not stop at one “good-looking” prediction screenshot.

## 3.5 How vision connects to the AMR fleet

The DL member publishes an observation; it does not create a direct movement command.

```text
Camera → detector → class + confidence + bounding box
                         ↓
                 VisionObservation topic
                         ↓
       dashboard + optional blockage confidence fusion
                         ↓
          LiDAR/local costmap confirms physical clearance
                         ↓
                 Safety Supervisor decides movement
```

**Example:** Vision says “pallet, 0.91 confidence” and LiDAR sees a persistent obstacle in that part of the aisle. The fleet can classify the temporary blockage and reroute. If the camera produces no result, LiDAR still prevents collision. If vision says “pallet” but LiDAR sees clear space, the robot does not emergency stop just because of vision alone.

## 3.6 DL success criteria

The DL member is successful when they can show:

- a documented, licensed dataset source and class taxonomy;
- a model tested on held-out images, not only training images;
- detection result published as a ROS message and shown in dashboard;
- inference time recorded on the intended computer;
- clear limitation statement: vision is supportive; LiDAR/Safety Supervisor remains safety authority.

## 3.7 DL research checklist

- object detection vs classification; why bounding boxes are needed here;
- transfer learning and fine-tuning;
- YOLO-format / COCO-format annotations;
- precision, recall, mAP, confidence threshold, and NMS;
- data augmentation: brightness, blur, scale, occlusion;
- dataset licences and attribution;
- edge inference trade-offs: accuracy, model size, latency.

---

# 4. How ML and DL work together without overlap

| Situation | ML member contributes | DL member contributes |
|---|---|---|
| Corridor is usually slow at 5 PM / under high task load | Predicts the extra route delay from fleet features. | Nothing required. |
| A pallet appears in an aisle | Uses blockage state as one input to revised ETA. | Detects/classifies “pallet” from camera view. |
| Robot must emergency stop | Nothing; model cannot command it. | Nothing; visual result alone cannot command it. |
| Judge asks “where is Edge AI?” | Explains route-cost/ETA predictor and measured ablation. | Shows live object-recognition stream and confidence. |

The integration lead owns the rule connecting them: DL can help create a blockage observation; ML can estimate the delay caused by that blockage; the Safety Supervisor owns physical safety.

---

# 5. Immediate meeting checklist

Schedule one 30-minute meeting between ML, DL, full-stack, and integration leads. Leave with these decisions written down:

1. What exact AMR camera topic / simulated camera will DL consume?
2. What exact CSV fields will the integration lead log for ML?
3. What are the selected vision labels (`pallet`, `carton`, optional `person`)?
4. Which public dataset is licensed and approved for bootstrap use?
5. What is the `VisionObservation` message structure?
6. What is the `RouteCostEstimate` message structure?
7. Where are dataset versions, model versions, and experiment seeds recorded?

After that meeting, both members should be able to work independently for the first week while producing outputs the rest of the team can immediately integrate.

---

# 6. What not to do

- Do not make ML wait for a perfect public dataset; begin logging Gazebo runs immediately.
- Do not make DL train from scratch; use transfer learning and a small, relevant label set.
- Do not use either model to directly control speed, braking, or collision avoidance.
- Do not compare models using data from the same run/scene they trained on.
- Do not download data without checking the licence and noting its source.
- Do not let dataset work delay the 3-AMR safety and corridor demo; both AI modules have safe fallbacks.

---

## Bottom line

**ML starts with a CSV from your own fleet simulation.** Build a simple ETA baseline, then use queue/blockage features to beat it.

**DL starts with a small licensed pallet/warehouse dataset and a pretrained detector.** Add simulated camera images from your exact warehouse, publish detections to ROS, and keep LiDAR in charge of safety.
