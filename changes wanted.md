# Improvements requested for ML-ready warehouse experiments

This is the implementation backlog derived from `Improvements.pdf`, reconciled
with `context.md`, `README.md`, and the current ROS 2/Gazebo fleet code. It is
not an authorization to weaken the decentralized allocation, reservation, ORCA,
or Safety Supervisor boundaries described in `context.md`.

## Incorporated in the locked layout metadata

The following are now represented in
`warehouse_layout.lock.yaml` and should be treated as the stable experiment
catalogue:

- Experiment metadata, warehouse extents, map frame/resolution, and layout
  version.
- Four robot identities, namespaces, dock spawns, model type, and battery
  defaults. Payload capacity is intentionally `null` until a physical payload
  specification is agreed.
- One charging-station record covering four pads.
- Four main-corridor IDs (`MC-*`), four junction IDs (`J-*`), and eight
  narrow-corridor families (`NC-*`) with an unambiguous expandable instance-ID
  system. There are 168 narrow aisle instances in total.
- Current navigation, sensor, logging, benchmark, fault, randomization,
  semantic-label, and DDS communication metadata. Fields that are not yet
  implemented are marked `planned` or `false`, rather than being presented as
  available functionality.

The PDF's pickup/drop station proposal is deliberately omitted: this project
does not use pickup/drop stations. Its AI-configuration proposal (point 10) is
also deliberately deferred, as requested.

## Changes to implement next

### 1. Make corridor IDs runtime resources

`corridor_mutex_node` currently reads generic corridor cells from the planner
map. Update it to read the lock YAML's `MC-*`, `NC-*`, and `J-*` records (or a
generated runtime corridor file) so reservation requests, queue lengths, wait
times, occupancy, and deadlock labels use the stable IDs. Keep the existing
lease/expiry rules: an expired reservation does not prove physical clearance.

### 2. Produce benchmark-grade run records

`data_collection_node` already writes JSONL state, health/battery, trajectory
reservation, task, corridor, and safety events. Extend it with a run manifest:
run ID, Git revision, world-layout version, map version, robot count, random
seed, speed limits, enabled fault/randomization profile, start/end time, and
termination reason. Add a post-run metric calculator for makespan, throughput,
average/worst corridor wait, travel time, energy estimate, replan count,
collisions/minimum separation, safety stops, deadlocks, and task completion.

This directly supports the reproducible multi-seed/ablation protocol already
specified in `context.md`; it must remain passive and never become a control
dependency.

### 3. Add controlled domain randomization

Create a seedable scenario-profile system, separate from the locked baseline
layout. It should randomize only at reset time unless a scenario explicitly
calls for dynamic change:

- lighting and material/visual variation;
- LiDAR/camera noise models;
- wheel slip/friction and battery initial level;
- safe temporary-obstacle spawning in declared free-space spawn zones;
- robot count/start states when non-overlapping and collision-free.

Every sampled value must be written to the run manifest. Never randomize the
locked shelf/dock coordinates in place or spawn an obstacle on a main corridor,
junction, narrow aisle, charging bay, robot footprint, or safety egress route
without an explicit scenario that tests blockage handling.

### 4. Add temporary-obstacle scenarios

Implement a Gazebo-side obstacle spawner with an allow-list of free-space
polygons, collision checking, deterministic IDs, TTL/removal, and a seeded
schedule. Connect observed obstacles to the existing LiDAR -> local costmap ->
blockage observation path; do not inject a planner-only obstacle and call that
a physical test. Include stationary, delayed appearance, and cleared-obstacle
cases.

### 5. Fault-injection harness

Add opt-in, seeded profiles for robot stop/failure, LiDAR dropout/noise,
network delay, and packet loss. The profile must expose start/end events in the
log, preserve message/session/lease validity semantics, and leave the local
Safety Supervisor authoritative. Begin with the PDF's proposed probabilities
(robot 0.01, LiDAR 0.001) only after deciding what the probability interval
means (per run, task, second, or message).

### 6. Complete sensor data collection when required

The current four-AMR baseline bridges LiDAR; camera data remains in Gazebo and
is intentionally not bridged. Before collecting vision data, add a bounded,
namespaced camera bridge and recorder with rate limits, image compression,
storage quotas, synchronized robot pose, semantic labels, and frame IDs. Do
not enable unrestricted four-camera recording: the storage estimate in
`README.md` shows it is impractical.

### 7. Semantic labels and object-level annotation

The layout now provides shelf/corridor label rules. Add a deterministic export
that expands every shelf into a semantic record (storage zone, shelf ID,
bounding box, and any future QR code) and every corridor family into individual
`NC-*` records. Keep label generation tied to the layout version so datasets
cannot silently mix maps.

### 8. Navigation and localization experiment configuration

The current planner is WHCA* with a custom path follower, ORCA, and the Safety
Supervisor - not SmacPlanner/RegulatedPurePursuit. Preserve that fact in run
metadata. The simulator exposes an AMCL-compatible pose topic derived from
transformed Gazebo odometry; a real LiDAR+map AMCL or Nav2 stack remains a
separate validation task and must not be represented as already deployed on a
physical robot.

### 9. Communication experiment profile

Keep DDS as the transport and record heartbeat, reservation timeout, lease
duration, delay/loss profile, and DDS domain in every run. Any retry policy
must be bounded and must not convert the design into a permanent central
controller. The dashboard and logger must remain read-only/non-authoritative.

## Recommended configuration split

Keep `warehouse_layout.lock.yaml` as the immutable geometry and identity source.
Add these runtime-owned, versioned files when their features are implemented:

```text
config/robots.yaml
config/navigation.yaml
config/fleet.yaml
config/logging.yaml
config/benchmark.yaml
config/randomization.yaml
config/faults.yaml
config/vision.yaml
```

ML/RL model-selection configuration is intentionally not added here. It should
be owned by the ML teammate once feature/label contracts and the benchmark
baseline are agreed.
