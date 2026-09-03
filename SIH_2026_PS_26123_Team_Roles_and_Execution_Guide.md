# SIH 2026 PS 26123 — Team Roles and Execution Guide

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
