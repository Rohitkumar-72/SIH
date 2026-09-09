# SIH Fleet Handoff — Deterministic Baseline Recovery

## READ FIRST — current takeover state (2026-09-09)

This section is the authoritative current handoff. Older sections below retain
the investigation history and may describe fixes as pending that have since
been superseded.

### User intent and constraints

The immediate goal is a repeatable four-AMR random workload in which assigned
robots actually reach pickup, dwell, reach dropoff, and publish `COMPLETED`.
The user will delegate the next Gazebo validation to a lower-cost Codex
instance. **Do not run Gazebo or another simulation unless the user explicitly
changes that instruction.** Static inspection, saved-log analysis, compilation,
unit tests, builds, SDF validation, and layout-lock checks are allowed.

Work in `/home/rtsws/amr_ws/src/SIH`. The worktree contains extensive
user-owned changes from several debugging sessions. Preserve them: do not
reset, clean, checkout, discard, or rewrite unrelated files, and do not commit
unless explicitly requested. Maintain decentralized ownership and motion:
CBBA/CBAA allocates work, WHCA* plans, reservations/corridor protocol/ORCA
coordinate motion, and the Safety Supervisor has final velocity authority.
Telemetry and dashboard components must remain passive. Do not add ML/DL or a
central allocator to solve this deterministic baseline.

### Latest failed random-workload evidence

Run directory:
`/tmp/sih_full_random_workload_20260909_4mps_retry2`

The 20-minute bounded run reported:

- build passed, 49 tests passed, SDF valid;
- five task announcements, 20/20 receipts, and 12/12 DDS first-packet gates;
- three assignments: `rnd_task_002 -> robot_4`, `rnd_task_003 -> robot_1`,
  `rnd_task_004 -> robot_3`;
- zero pickup waits, dropoffs, completions, or fleet work cycles;
- every executor remained `EN_ROUTE_PICKUP`;
- 111 WHCA* infeasible routes, no ownership conflicts, no ROS errors/fatals;
- achieved RTF approximately 0.098.

The relevant files are `fleet.log` and `fleet_telemetry.jsonl` inside that run
directory. The earlier bounded diagnostic
`/tmp/sih_debug_short_cycle_v5_20260909` did complete one real task through all
five execution phases, so the executor/pickup/dropoff state machine itself is
capable of completing a carefully chosen route. The random workload exposed
fleet-scale blockage and auction-liveness faults instead.

### Confirmed root cause 1 — false global blockage topology

The old `blockage_detector_node.py` removed only exact static-map cells. Saved
telemetry shows that every robot immediately published cells on the bottom
warehouse boundary, docks/peer silhouettes, and LiDAR surface returns one cell
outside analytically generated shelf occupancy. With four publishers and a
2.0-second lease, moving returns accumulated into swept trails of roughly
43--208 simultaneous dynamic cells.

Concrete reconstruction from the saved JSONL:

- at simulation time 50, robot 4's static route `(58,4) -> (85,4)` was
  connected, but 43 fused dynamic cells disconnected it; robot 4's own shelf
  surface cells at x=63/64 sealed the east passage;
- at time 80, robot 1's route `(35,1) -> (14,13)` was connected statically but
  disconnected by 132 fused cells;
- at time 100, the sampled robot 3 route remained connected but carried 169
  unnecessary dynamic cells and was vulnerable to the same fusion.

Applying the new semantic filter offline to these saved snapshots restored
WHCA* route availability in all three cases. This was an offline replay-style
analysis only; it did not launch ROS or Gazebo.

Implemented correction:

- `algorithms.py` adds `expand_grid_cells` and
  `filter_unexpected_blockages`;
- `blockage_detector_node.py` precomputes a one-cell exclusion halo around
  static occupancy, excludes the two outer grid rows/columns, configured dock
  anchors, and a two-cell envelope around every fresh `/fleet/robot_state`;
- persistence is now 10 LiDAR frames (about 0.5 simulated seconds at 20 Hz),
  replacing 5 frames;
- global observation TTL is now 0.75 seconds, replacing 2.0 seconds;
- the local costmap and directional Safety Supervisor remain unfiltered and
  still react immediately to actual near-field obstacles.

The intended layer ownership is important: known static geometry belongs to
the WHCA* static map; known AMRs belong to trajectory reservations, ORCA, and
Safety Supervisor; only persistent *unmodelled* obstacles belong on the global
blockage topic.

### Confirmed root cause 2 — CBBA invalidated its own exact quorum

CBBA correctly requires four fresh CLAIMs matching the exact
`(winner_robot_id, winner_session_id, winning_bid, epoch)` tuple. However,
`run_round()` recomputed each robot's pose-dependent bid every 0.5 seconds while
the same epoch remained open. Robot motion changed the float32 value, and
asynchronously learned busy state could replace a finite bid with `1e9`.

Saved telemetry proves this was not merely packet loss:

- task 5 advertised at least nine distinct winning bid values around
  `6.882328` through `6.884371`;
- tasks 2 and 3 contained both a finite winner value and `1e9` claims;
- task 1 eventually changed robot 4's local winner from robot 4 to robot 3
  after different replicas committed later work in different orders.

Those changes made the exact-match safety gate continually report
`mismatched_claims`, even though all four writers were alive.

Implemented correction:

- `algorithms.py` adds the tested `freeze_auction_value` helper;
- `cbba_node.py` stores `own_bids` and `own_claims` per task/epoch;
- the first BID is immutable for that epoch and later rounds only refresh its
  lease;
- no CLAIM is signed before the configured 2.0-second settlement window and a
  complete four-robot BID set;
- the first derived complete-set CLAIM is also immutable for the epoch;
- quorum warnings now include every retained source's full claim tuple;
- terminal task cleanup removes the frozen bid/claim state;
- the existing targeted reliable `/<robot>/consensus_inbox` redundancy and
  typed `/fleet/task_consensus` audit topic are retained.

This is still the project's deliberately bounded single-active-task
CBAA/CBBA stepping stone, not complete bundle CBBA with general overlapping
auction revisions. The random generator's 12--25 second spacing should let the
2-second auction commit and execution status propagate before the next task.
If a future test introduces concurrent auction epochs or robot restarts, add an
explicit replicated epoch/reauction protocol rather than mutating a signed
epoch-1 value.

### Diagnostics and telemetry

`data_collection_node.py` now writes schema `0.6.0` and includes
`winner_session_id` in every `task_consensus` record. This distinguishes a
numerical claim disagreement from a winner boot-session disagreement. Existing
schema 0.5 pipeline diagnostics, exact route/blockage cells, and `/rosout`
warning/error/fatal collection remain present.

If CBBA still withholds after the next run, group CLAIM records by task,
source robot, winner robot, winner session, winning bid, and epoch. The full
`claim_values` dictionary in the CBBA warning should identify the divergent
writer directly. If WHCA* is still infeasible, reconstruct active blockage
cells using each record's `valid_until_s`, compare reachability against the
static map, and group retained cells by publishing robot. Do not assume all
infeasibility is dynamic blockage: inspect `reservations`, `start_static`,
`start_dynamic`, `goal_static`, `goal_dynamic`, and dynamic-cell counts in the
WHCA warning.

### Current static verification

After the latest fixes, without running Gazebo:

- `colcon build --symlink-install --packages-select sih_amr_interfaces
  sih_amr_fleet`: passed;
- focused tests: **52 passed**;
- all Python sources and `fleet.launch.py`: compiled successfully;
- warehouse SDF: `Valid`;
- root and external layout-lock SHA-256:
  `d4fdec562dfc1c1cc6b0c66aacef06e7dba459c276456ed5e34f4b172207c93f`;
- `git diff --check`: clean except the pre-existing `package.xml` CRLF warning;
- Gazebo/simulation runs after these changes: **none**, per user instruction.

Files changed for this latest correction:

- `src/sih_amr_fleet/sih_amr_fleet/algorithms.py`
- `src/sih_amr_fleet/sih_amr_fleet/blockage_detector_node.py`
- `src/sih_amr_fleet/sih_amr_fleet/cbba_node.py`
- `src/sih_amr_fleet/sih_amr_fleet/data_collection_node.py`
- `src/sih_amr_fleet/test/test_algorithms.py`
- `CURRENT_IMPLEMENTATION_GUIDE.md`
- `context.md`
- this handoff

### What the next Codex instance should do

1. Read this section, the latest appended portion of `context.md`, current
   source files, and the new validation logs supplied by the user.
2. Do not launch a simulation unless the user explicitly authorizes it.
3. Preserve the dirty worktree and distinguish old user changes from any new
   edits.
4. In the delegated run, first require 20/20 receipts and four immutable BID
   and CLAIM participants per task, then verify exactly one owner/acceptance.
5. Require feasible routes and real progress through `EN_ROUTE_PICKUP ->
   PICKUP_WAIT -> EN_ROUTE_DROPOFF -> DROPOFF_WAIT -> COMPLETED`.
6. Report completed tasks per robot and fleet work cycles (`completions / 4`);
   do not claim fairness from assignment counts alone.
7. Continue to report the known vendor-controller startup NaN warnings and
   teardown `publish_async_failures_` separately unless logs prove they cause a
   runtime failure. Previous evidence showed the controller NaNs occur during
   activation and the async counter during teardown.
8. Do not claim docking, controlled fault injection, collision-contact
   sensing, WHCA* runtime buffer acceptance, or soak success without direct
   telemetry for each criterion.

### Hardware/performance context

The RTX 3070 and NVIDIA driver/OpenGL stack are healthy when Gazebo runs in the
host context. Earlier missing `/dev/nvidia*` visibility was a Codex sandbox
artifact, not a driver fault. Gazebo was nevertheless CPU/physics-bound at
about 107% CPU, 36% GPU utilization, and RTF 0.097--0.12. One Gazebo physics
world does not automatically spread its main update loop over all 12 CPU
cores. The accelerated launcher has reached approximately 4.0 m/s, but raising
requested robot speed or SDF real-time factor cannot overcome a physics-bound
RTF by itself.

Work in `/home/rtsws/amr_ws/src/SIH`. This ROS 2 Jazzy/Gazebo four-AMR task excludes ML/DL, learned policies, and vision. Preserve the dirty worktree; do not reset, clean, overwrite unrelated work, or commit without explicit user approval.

Read this file, `CURRENT_IMPLEMENTATION_GUIDE.md`, `context.md`, `README.md`, `ROS2_FLEET_IMPLEMENTATION.md`, `warehouse_layout.lock.yaml`, and current diffs before editing. Inspect `random_task_generator_node.py`, `cbba_node.py`, `task_execution_node.py`, `whca_planner_node.py`, `path_follower_node.py`, `safety_supervisor_node.py`, `data_collection_node.py`, `common.py`, `launch/fleet.launch.py`, `scripts/launch_four_amrs.sh`, and tests. Keep root and external warehouse locks synchronized. World: `/home/rtsws/amr_ws/src/warehouse_world_custom/worlds/small_warehouse/warehouse_clean.sdf`.

Implemented but not fully live-accepted: deterministic dock protocol/guidance/anchor correction, passive map-vs-Gazebo odometry telemetry, guarded recovery, degraded-peer resource protection, and WHCA* buffer. Movement must remain behind ORCA and Safety Supervisor; dashboard/telemetry must remain passive; no central task allocator.

## Verified state

Latest validation: `/tmp/sih_headless_validation_20260908_freshD`; ROS logs
`/home/rtsws/.ros/log/2026-09-08-19-17-40-484522-rtsws-MS-7C52-9403`.

- Cyclone DDS, all four real odometry/LiDAR gates, and all four task-inbox
  readers passed.
- Five unique announcements produced 20 receipts, four CBBA participants per
  task, and exactly one owner per task.
- Telemetry recorded 950 planned routes, 1,264 trajectory intents, 18,989 task
  execution records, distinct map/raw odometry, and movement by robots 1 and 4.
- Completed tasks remained 0; robots 2 and 3 stopped near the south wall,
  robot 1 crossed pickup too fast, and robot 4 eventually reported infeasible
  routes at a shelf boundary.
- No process crashed. One-time diff-drive activation NaN warnings remained for
  robots 2–4, and `publish_async_failures_ 172` printed during teardown.

The CBBA/task-transport boundary is therefore live. Do not reopen it unless a
new run loses receipts or ownership convergence.

## Pending-validation correction

`freshD` exposed four downstream faults and the working tree now contains fixes:

- `path_follower_node.py` advances across route waypoints, turns in place before
  translating, follows at 1.0 m/s to match 0.5 m / 0.5 s reservations, and
  slows/stops for the executor's arrival gates;
- `whca_star` snaps only a rounded blocked start to nearest free space, fixing
  robot 4's permanent infeasibility at occupied cell `(52,45)`;
- reverse recovery waits for a persistent braking stop, validates rear-sector
  clearance, measures actual retreat, and rate-limits rejected retries;
- blockage observations now transform `base_link` points by robot yaw and emit
  true planning-grid indices instead of rounded map metres.

Only a Python syntax check has run after these edits. Run build, focused tests,
SDF validation, then the full headless acceptance. Specifically verify all four
robots leave their docks without moving south first, task phases reach pickup
wait/dropoff/completed, robot 4 never becomes permanently infeasible at the
first shelf, recovery events do not repeat at control-loop rate, and at least
four completed deliveries establish one fleet work cycle.

## Latest validation and second correction

Run `/tmp/sih_headless_validation_20260908_takeover_2018` passed build, 34 tests,
SDF/lock checks, Cyclone, all four physical interface gates, task transport, and
four-way CBBA participation. All robots moved, initial turn-in-place worked,
maximum speed was 1.0 m/s, and robot 4 never became route-infeasible. It still
completed zero tasks because planners and executors followed different task
IDs after later CBBA epochs. Concrete examples: robot 3 executed task 1 while
210 routes targeted task 4; robot 1 executed task 2 then planned task 5; robot
2 executed task 3 then planned task 5.

The first attempted correction treated execution status as a sticky lock, made
a busy robot unavailable to other tasks, and made WHCA* ignore assignments
that did not match its executor task. The next run proved that execution status
arrived too late to be the initial ownership commit; the busy-robot and planner
guards remain useful downstream protections. The simulator launcher defaults
to 4.0 m/s (direct fleet launch stays at 1.0 m/s), derives reservation slots
from grid resolution/speed, and runs all fleet nodes on simulation time.

Hardware/throughput diagnosis was corrected by host-context validation. The
RTX 3070, driver 595.84, and direct NVIDIA OpenGL 4.6 are healthy, and Gazebo
appeared in `nvidia-smi`; absent GPU devices were specific to the Codex sandbox.
Run `/tmp/sih_headless_validation_20260908_gpu_2107` nevertheless achieved only
0.097 simulation seconds per wall second, with Gazebo around 107% CPU and GPU
utilisation around 36%. The active fleet sensor profile had already removed the
44 unused GPU lidars and four cameras. The remaining bottleneck is CPU/physics,
and merely setting SDF `real_time_factor > 1` cannot exceed available compute.
Telemetry confirms the launcher did reach about 3.9997 m/s, so the accelerated
route-speed wiring is working.

The same run found task 1 assigned to robot 4 at epoch 1 and robot 1 at epoch 2
at the same simulation timestamp; both accepted before execution status could
act as a lock. The pending-validation CBBA implementation now uses unanimous
two-phase ownership: four fresh per-robot BID records, followed by four fresh
CLAIM acknowledgements of the exact same winner/session/bid/epoch tuple. Only
that quorum commits ownership and permits the winner to publish an assignment.
Open-auction health/lease takeovers were removed. Do not describe execution
status as the initial commit boundary; it is now only confirmation/refresh.

The first two-phase validation is
`/tmp/sih_headless_validation_20260908_cbba_fixed_2118`. Build, 37 tests,
syntax/diff/SDF checks, and four interface gates passed. Robots 1–3 unanimously
committed task 1 to robot 4; there was no conflicting-owner error or duplicate
acceptance. Robot 4 did not commit because those early replicas stopped BID
publication, after which its bid cache expired (fleet log lines 125–126).
Pending-validation code now makes a pre-committed replica retransmit its frozen
own BID followed by its identical CLAIM until matching execution status is
observed. This preserves unanimous ownership while allowing the lagging winner
to reconstruct the fresh quorum and publish its assignment.

The following one-cycle validation again produced three commit logs and zero
assignments. Its telemetry contained continuous, identical BID and CLAIM records
from every writer, proving the remaining gate was self-defeating: synchronized
timers publish BID immediately before CLAIM, so requiring each source claim's
sequence to exceed that source's latest observed bid creates a moving boundary.
The current pending-validation code retains fresh four-of-four exact claims and
source-session equality but removes that cross-writer sequence comparison.

A full code/log audit also found the downstream reason the earlier assigned
runs never reached task stations. The 12-step WHCA* search used Manhattan
distance. Where a shelf requires a temporary sideways detour longer than the
window, WAIT remained cheaper, so repeated windows stayed at the same cell.
The failure reproduced in 153 of 1,140 seeded dock/task route replays. WHCA* now
uses a reverse-BFS obstacle-aware distance heuristic; the same replay has zero
failures. Other pending-validation fixes:

- blockage detection derives the exact shared static occupancy and removes
  shelf/wall LiDAR echoes before publishing dynamic blocks;
- completed execution clears WHCA* task identity for the next work cycle;
- announcement TTL expires only uncommitted work, not an active delivery;
- corridor arming is cleared/re-derived from the current rolling route and is
  not switched during an active mutex request;
- infeasible/expired routes clear cached trajectory intents so completed work
  cannot leave indefinitely refreshed ghost reservations, and the follower
  clears a route whose task ID differs from the executor;
- base-frame wheel-odometry velocity is rotated into the map frame before peer
  prediction and telemetry.

Do not run another integration test in this task; the user is delegating that
validation. Build/unit/SDF/live completion evidence for this final correction
is still pending.

Validation `/tmp/sih_headless_validation_20260908_correctness_1ms_1mps_2310`
passed the rebuild, 44 tests, SDF, syntax, all four interfaces, five
announcements, and 20 receipts, but still produced only three local commits for
tasks 1 and 3 and no assignment. Telemetry saw continuous BID and CLAIM traffic
from all four sources with identical constant winner values. Meanwhile the
winning CBBA process logged a participant BID missing from its own cache. The
failure is therefore asymmetric shared-topic delivery to individual CBBA
readers, not winner divergence.

Pending-validation CBBA transport now publishes every consensus packet both on
the typed `/fleet/task_consensus` audit topic and as strictly validated JSON to
each peer's reliable depth-30 `/<robot>/consensus_inbox`. This follows the
independently matched transport pattern that already achieved 20/20 task
receipts. It remains decentralized: every source directly fans out only its own
packet, every receiver still computes the winner, and assignment still requires
four fresh exact claims. Same-session out-of-order BID/CLAIM packets are also
rejected by sequence number. No new build or integration run has been performed
after this correction.

Run headless only using `scripts/launch_four_amrs.sh` with `START_GUI=false`, unique `/tmp` logs, then inspect fleet, Gazebo, robot, run-event, and JSONL logs. Fleet work cycles = completed deliveries / 4; report per-robot fairness. Do not claim delivery, docking, recovery, contact sensing, or soak acceptance until observed. Update `CURRENT_IMPLEMENTATION_GUIDE.md` and `context.md` only with verified results.

Latest verified result (2026-09-09): disregard the earlier instruction not to
run another integration test; the user explicitly requested a bounded Gazebo
diagnostic. `/tmp/sih_debug_short_cycle_v5_20260909` completed one task through
all execution phases. Counts were phase 0=66, 1=10, 2=62, 3=10, 4=10; there
was one assignment, zero infeasible routes, and zero severity-error ROS logs.
The focused suite passed 49 tests.

Root cause and correction: TurtleBot 4 Lite's vendor URDF mounts
`rplidar_link` at +pi/2 yaw, previously omitted by costmap/safety consumers.
After transforming scan points/sectors correctly, coarse global LiDAR fusion
still represented the executing robot (as seen by peers) and occasionally a
validated task endpoint as blocked WHCA* cells. WHCA* now clears its one-cell
self and station envelopes from the coarse blockage layer, leaving those
near-field decisions to ORCA and the 40 Hz directional safety supervisor;
distant blocks remain. Data schema 0.5 now records exact route/blockage data,
correlated pipeline blockers, and all `/rosout` warning/error/fatal messages.
The diagnostic scenario is `scenarios/debug_short_cycle.yaml`. Full random
workload fairness, docking, injected faults, collision contact, and soak are
still unverified and should be the next delegated validation, not inferred
from this micro-run.

## Latest pending-validation correction (random workload)

Do not run Gazebo in this task; the user will delegate the next live test. The
failed run `/tmp/sih_full_random_workload_20260909_4mps_retry2` had 3
assignments, 0 completions, and 111 infeasible WHCA* routes. Saved-telemetry
reconstruction shows the old global blockage layer fused four robots' bottom
wall, dock/peer, shelf-edge, and swept-trail returns into 43--208 cells. These
cells disconnected routes which are connected in the static map.

`blockage_detector_node.py` now filters the static-map one-cell halo, map
boundary, dock anchors, and fresh fleet robot envelopes before persistence;
it requires ten frames and leases reports for 0.75 s instead of 2.0 s. Local
safety remains unfiltered. Offline application of the filter to the saved
failure snapshots restored WHCA* route availability.

CBBA telemetry also proved that an open task's exact signed value was changing:
task 5 emitted at least nine winning bids and other tasks switched between a
finite and unavailable bid as replicas learned busy state. `cbba_node.py` now
freezes each participant's BID and derived CLAIM for the auction epoch, then
refreshes leases without recomputing them. Telemetry schema 0.6 adds
`winner_session_id`. Focused tests cover auction immutability and semantic
blockage filtering. Both packages rebuilt; 52 tests passed; Python/launch
compilation passed; the SDF is valid; layout locks match; and `git diff
--check` is clean apart from the existing package.xml CRLF warning. No Gazebo
run was performed. Live completion/fairness is pending the user's delegated
validation.
