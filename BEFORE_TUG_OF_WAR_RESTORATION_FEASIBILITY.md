# Feasibility Report and AI-Agent Prompt: Restore the “Before Tug of War Fix” Baseline

## Mission

Restore the fleet software to the last verified high-throughput implementation represented by Git commit:

```text
ce79fb78faa583347b346bb8bd354f026e2029bc
Commit subject: before tug of war fix
Commit time: 2026-09-12 19:09:44 +0530
```

Then correct the small number of known defects in that baseline through isolated, evidence-driven patches. Do not carry the later experimental navigation stack back wholesale.

This document is both:

1. A feasibility and risk report for the human operator.
2. A complete implementation prompt for the AI agent assigned to perform the restoration after explicit human approval.

---

## Mandatory governance

Before doing anything, read and obey `/home/rtsws/amr_ws/src/SIH/AGENTS.md` completely.

The following restrictions are absolute:

- Do not run Gazebo, fleet launchers, data-collection runs, benchmarks, or long-running simulations.
- Only the human operator may execute final fleet runs.
- Do not modify code until the human has reviewed and explicitly approved the file-level restoration plan.
- Treat every current tracked modification and untracked file as user-owned data.
- Never use `git reset --hard`, a broad `git checkout -- .`, `git clean`, or any command that could erase the current forensic state.
- Preserve datasets, run artifacts, governance files, handoff documents, and untracked tests.
- Agents may run read-only Git inspection, deterministic unit tests, syntax checks, and approved builds.

---

## Executive feasibility conclusion

Restoration is **highly feasible and is the lowest-risk route back to a productive fleet**.

The reason is unusually favorable: the repository's current `main`/`HEAD` already points to the desired commit `ce79fb7`. The degraded implementation is not a later committed history that must be reverted. It consists of uncommitted tracked changes plus untracked files layered over that commit.

Therefore, no history rewrite is required. The target tree is directly available from Git. Restoration can be prepared in an isolated worktree or performed later with path-scoped `git restore --source=ce79fb7 -- <approved paths>` after all current changes have been preserved.

### Direct performance proof for the target commit

Two consecutive recorded runs identify `ce79fb7` in their manifests and completed all tasks:

| Run | Recorded Git revision | Result | Data-collector wall time |
|---|---|---:|---:|
| `desktop_run_001_20260912_192735` | `ce79fb7` | 20/20 tasks | 1013.14 s |
| `desktop_run_002_20260912_194553` | `ce79fb7` | 20/20 tasks | 901.20 s |

The two runs therefore produced 40/40 successful task completions. The implementation guide reports mean map-to-Gazebo errors of approximately 1.88 cm and 1.81 cm for the same verified baseline pair.

This evidence is much stronger than merely assuming an older commit was better.

### Comparison with the latest experimental run

The run `desktop_run_001_20260913_134106`, executed after another large experimental patch, produced:

- 3 completed tasks in 779.4 wall seconds.
- All three completed tasks were performed by `robot_1`.
- Robots 2, 3, and 4 reached pickup but never completed drop-off.
- `robot_4` accumulated 1284.6 radians of absolute rotation and spent 69.7% of samples rotating.
- Gazebo pose dispatch reported 8,880 failures in 26,462 requests, a 33.56% reported failure rate.
- Mean map-to-Gazebo position errors increased to 0.30-0.61 m depending on robot, with a maximum of 0.88 m.

The current dirty tree is approximately 1,023 insertions and 171 deletions away from `ce79fb7` across 19 tracked files, in addition to untracked tests and documents. Continuing to patch this combined state is substantially riskier than restoring the verified baseline and reintroducing only proven corrections.

---

## Exact scope of the current dirty state

At the time of this report, tracked modifications include:

```text
.gitignore
desktop_fleet_dataset.csv
scripts/launch_fleet_amrs.sh
scripts/launch_four_amrs.sh
scripts/run_desktop_data_collection.py
scripts/run_laptop_data_collection.py
scripts/run_multi_work_cycles.py
src/sih_amr_fleet/config/navigation.yaml
src/sih_amr_fleet/launch/fleet.launch.py
src/sih_amr_fleet/maps/demo_warehouse.yaml
src/sih_amr_fleet/sih_amr_fleet/algorithms.py
src/sih_amr_fleet/sih_amr_fleet/corridor_mutex_node.py
src/sih_amr_fleet/sih_amr_fleet/kinematic_carrier_node.py
src/sih_amr_fleet/sih_amr_fleet/localization_node.py
src/sih_amr_fleet/sih_amr_fleet/orca_node.py
src/sih_amr_fleet/sih_amr_fleet/path_follower_node.py
src/sih_amr_fleet/sih_amr_fleet/whca_planner_node.py
src/sih_amr_fleet/test/test_algorithms.py
src/sih_amr_fleet/test/test_kinematic_carrier.py
```

Known untracked material includes governance files, forensic reports, and new tests. These must not be deleted.

`desktop_fleet_dataset.csv` is collected user data and must not be restored or overwritten as part of a software rollback.

---

## Problems that existed in commit `ce79fb7`

The commit was fast and completed tasks reliably, but it was not defect-free. The restoration must acknowledge these problems rather than claiming the baseline was perfect.

### 1. Main-corridor tug of war

**Observed behavior**

Two AMRs approaching each other in a main corridor can repeatedly advance, stop, rotate, and appear to push one another until one happens to pass or reach a turn. The reaction can also occur when their paths have enough lateral clearance to cross safely.

**Root causes in the baseline**

1. `avoidance_velocity()` uses a one-dimensional predicted-distance approximation:

   ```python
   closing = dot(relative_velocity, separation_direction)
   predicted_distance = separation - closing * horizon
   ```

   This does not calculate the true two-dimensional closest point of approach. It can report a collision for trajectories whose lateral miss distance is already safe.

2. Both robots apply the same reciprocal slowdown without deterministic right-of-way. There is no stable winner/loser decision based on corridor ownership and robot priority.

3. Peer corrections are applied sequentially, so the result can depend on peer iteration order.

4. The algorithm produces a 2-D velocity, but `orca_node.py` projects it back onto the robot's current forward axis and copies the PathFollower angular command unchanged. A differential-drive AMR therefore cannot execute the calculated lateral component.

5. The baseline ORCA code does not guarantee that avoidance only reduces an already-positive requested speed. It can create translation from an intended stop or produce a reverse projection.

6. Uncertainty inflation can dominate the physical footprint. The baseline starts with a 0.35 m radius and adds `2 * sqrt(covariance_trace)` before doubling the result into a collision envelope. This can make robots react to peers much farther away than their physical 0.56-0.70 m combined operating footprint requires.

**Correct repair direction**

- Use true 2-D closest-point-of-approach mathematics for deciding whether trajectories actually intersect.
- Establish right-of-way before local velocity modification: a robot physically inside a protected corridor outranks an outside robot; otherwise use one stable robot-ID tie-breaker.
- ORCA must never accelerate a stopped robot, exceed desired forward speed, or command unverified reverse motion.
- Lateral passing must be represented by a realizable route or heading target. Do not claim that a discarded `vy` component steers a differential-drive robot.
- In 3 m main corridors, represent right-hand directional sublanes in WHCA planning and reservations themselves. Do not post-process waypoints while leaving reservation cells on the old centerline.
- At junctions and lane transitions, taper the lane change over multiple waypoints and preserve the physical corridor boundary.
- If no safe bypass exists, only the lower-priority AMR waits or performs a separately clearance-checked retreat. The winner continues.

### 2. Fixed integration time loses motion under callback load

**Baseline behavior**

The carrier assumes every callback represents exactly `1 / 50 = 0.02` simulation seconds, irrespective of actual elapsed simulation time.

If the callback executes fewer than 50 times per simulation second, commanded motion is under-integrated. Earlier forensics measured a roughly 28-33% speed reduction under load.

**Why the later attempted correction must not be copied blindly**

The later version integrated the entire elapsed simulation interval while holding one stale control command across all microsteps. Collision checking improved, but control feedback did not run between those substeps. Combined with adjacent grid targets and rolling replanning, this produced extreme heading oscillation.

**Correct repair direction**

- Restore the fixed-step baseline first because it has direct 40/40 mission evidence.
- Do not change carrier timing in the same patch as PathFollower, WHCA, or ORCA.
- Build a deterministic controller/carrier replay test before changing time integration.
- Any future elapsed-time implementation must prove equivalent closed-loop behavior for callback intervals from 0.02 to at least 0.15 simulation seconds.
- Use exact or carefully substepped unicycle integration, but do not imply that carrier-only microsteps provide controller feedback.
- Keep this as a later, isolated optimization. It is not required to restore the known successful wall-time baseline.

### 3. Dock-anchor heading inconsistency

**Baseline defect**

The map declares charging-pad headings as `-1.5708` rad, while robots spawn facing `+1.5708` rad. The carrier also accepts an anchor match when the robot is facing either the anchor heading or the heading plus pi.

That combination later produced an instantaneous approximately 180-degree localization correction and trapped robots against the south boundary.

**Proven correction to reapply after restoration**

- Set all dock map-anchor headings to `+1.5708` rad so they match physical spawn orientation.
- Require the true forward anchor heading; remove the `anchor_yaw + pi` match.
- Wrap the published localization yaw to `[-pi, pi]`.
- Verify by static/unit tests that reverse-facing contact cannot confirm a dock.

These corrections were empirically validated in run `125652`: Robots 2 and 3 mean yaw errors fell from approximately 2.97 and 2.94 rad to 0.164 and 0.115 rad.

### 4. Orphaned carrier processes can hijack a later run

**Baseline defect**

`kinematic_carrier_node` was absent from runner cleanup patterns. A detached carrier survived an interrupted run and later moved `robot_1` by 20.624 m from its verified spawn position.

**Proven correction to retain**

- Include `kinematic_carrier_node`, `kinematic_carrier`, and the pose verifier in runner cleanup checks.
- Keep cleanup scoped to the expected ROS/Gazebo processes and DDS domain.
- Prefer graceful termination followed by a bounded forced termination only for verified lingering PIDs.
- Never use an unbounded process-kill pattern.

### 5. Boundary lock has no inward escape

**Baseline defect**

The valid carrier boundary begins at approximately `x = -22.55`. If tracking crosses slightly beyond it, the next candidate point is also invalid and the carrier can assign an allowed movement ratio of zero indefinitely—even when the command would move inward.

**Correct minimal repair**

- Permit a step only when it strictly reduces the current boundary violation and does not introduce a shelf or peer collision.
- Do not solve the problem by expanding the drivable warehouse through the physical wall.
- Add unit tests for outward rejection, inward recovery, shelf rejection, and peer rejection.

### 6. ORCA can move a robot whose desired speed is zero

**Baseline defect**

When PathFollower is turning in place or corridor control requests a hold, peer repulsion can synthesize nonzero translation. A later recorded case produced approximately `0.93 m/s` from a desired `0.00 m/s`.

**Correct minimal repair**

Apply this invariant after avoidance:

```text
if desired_vx <= stop_epsilon: candidate_vx = 0
otherwise: 0 <= candidate_vx <= desired_vx
```

Reverse recovery must remain a separate state with rear-clearance, safety, docking, and protected-corridor checks.

### 7. Corridor token can be held without physical entry

**Baseline defect**

A robot that receives a narrow-corridor token but is stopped outside the entrance can retain the token indefinitely. One observed later run held a corridor for 831 simulation seconds and blocked multiple robots.

**Correct minimal repair**

- Distinguish `token granted`, `ENTER announced`, and `physically inside resource`.
- Start an unentered-token deadline only after grant.
- If the robot has not physically entered or made forward progress within the approved deadline, publish `CANCEL/RELEASE` and re-request later.
- Never time out an owner that is physically inside the single-lane resource.
- Use session/request IDs and ignore stale protocol events.

### 8. A transient infeasible rolling plan erases the valid route immediately

**Baseline defect**

`PathFollowerNode.on_route()` replaces the route with `None` on every infeasible update. A single transient WHCA failure can stop a robot in an intersection and make congestion worse.

**Correct minimal repair**

- Retain the last route only for a short, bounded interval and only for the same task/session.
- Safety stop, corridor denial, expired plan, task change, and final docking constraints override retention immediately.
- Do not retain or republish stale reservations indefinitely.

### 9. Silent Gazebo pose-dispatch failures

**Baseline defect**

The return value from `/set_pose_vector` is ignored and every exception is swallowed. Internal map pose can diverge from the Gazebo carrier that supplies LiDAR rendering.

The latest instrumentation reported 8,880 failures in 26,462 requests. This proves that pose-dispatch reliability must be measured, although the agent must first confirm the exact Gazebo Python return semantics before interpreting each false result.

**Correct repair direction**

- Preserve behavior initially but record request transport success, reply presence, and reply data.
- Log at a throttled rate; do not flood ROS output.
- Correlate commanded pose with subsequently observed Gazebo pose per robot.
- Do not silently advance into a multi-metre split-brain state.
- Do not automatically snap internal localization to Gazebo ground truth; Gazebo is a simulation-only sensor renderer, not the physical localization authority.
- Decide retry, hold, or fault behavior only after the acknowledgement semantics are proven by a focused test.

### 10. Telemetry summary and provenance are unreliable

**Baseline defects**

- `DataCollectionNode.completed_tasks` is initialized but never updated when execution enters `COMPLETED`. Run summaries therefore report zero completions even when task-execution records prove completion.
- `termination_reason` is always written as `NORMAL_SHUTDOWN`, including manually interrupted runs.
- The manifest records only `git rev-parse HEAD`. Different dirty working trees therefore receive the same revision, as happened throughout these experiments.
- The manifest claims pipeline diagnostics exist, while lean telemetry disables them.

**Correct repair direction**

- Add each unique completed task to the completion set on a valid `COMPLETED` transition.
- Record the actual termination reason.
- Record `git_dirty`, a deterministic source/config diff hash, map hash, launch/config hash, and build provenance.
- Either enable pipeline diagnostics for forensic runs or make the manifest accurately state that they were disabled.
- Do not rewrite or discard existing datasets; add quality/provenance labels during offline preparation.

---

## Changes from later experiments that must not be carried over without isolated proof

The following current-tree behavior is unverified or empirically regressive:

1. **Persistent route-progress index across rolling plans**

   Every WHCA route is regenerated from the robot's current cell. Reusing the previous plan's numeric waypoint index in a new route does not preserve geometric progress; index 8 in the new route is not the same location as index 8 in the old route. This can make a robot skip most of each new horizon.

2. **Post-processing waypoints into 0.40 m sublanes while reservations remain unchanged**

   The physical path and the reserved grid path no longer describe exactly the same trajectory. Per-segment right-normal offsets also create discontinuities at corners and junctions.

3. **Bundled actual-time carrier integration plus motion-controller changes**

   Too many variables changed at once. Earlier runs spent 62-87% of samples rotating, compared with about 11-12% in the verified baseline.

4. **Large ORCA/corridor state-machine expansion without fleet-level liveness proof**

   Unit encounter tests did not predict the four-robot failures. Unit success is necessary but not sufficient.

5. **Assuming logged `set_pose_vector` failures are fully understood**

   The counter is useful evidence, but the Gazebo API return contract and subsequent observed pose must be correlated before changing authority or retry behavior.

6. **Restoring generated or collected data from Git**

   Dataset files are user artifacts, not navigation source code. They must remain untouched.

---

## Required restoration strategy

### Phase 0: Preserve and inventory; no behavioral edits

The agent shall first present:

- `git status --short`
- The full list of tracked files differing from `ce79fb7`
- The full list of untracked files
- A diff-stat and file-by-file classification:
  - navigation behavior
  - operational cleanup
  - telemetry/provenance
  - tests
  - governance/documentation
  - collected data

Do not proceed until the human approves the preservation mechanism and exact restore paths.

Preferred approach: prepare an isolated worktree at `ce79fb7` on a new `codex/` restoration branch. This leaves the dirty working tree untouched and makes comparison/recovery straightforward.

If the human instead requests in-place restoration, preserve the complete dirty patch and every untracked file first, then use only path-scoped restore commands. Never reset or clean the repository.

### Phase 1: Reproduce the exact baseline tree in isolation

Restore source, configuration, launch files, and baseline tests from `ce79fb7` in the isolated restoration worktree.

Do not restore or overwrite:

- `desktop_fleet_dataset.csv`
- other collected datasets
- `/home/rtsws/amr_ws/log/**`
- `AGENTS.md`
- `.agents/**`
- forensic/handoff reports
- this restoration report

Run static syntax checks and the baseline deterministic unit tests. Do not run a simulation.

### Phase 2: Reapply only proven, low-coupling corrections

Use separate commits or separately reviewable patches for:

1. Scoped orphan-process cleanup.
2. Correct dock headings, strict forward dock matching, and yaw wrapping.
3. Boundary inward-escape behavior without widening through walls.
4. Telemetry completion counting and dirty-source provenance.
5. Throttled Gazebo dispatch observability with no change of localization authority.

After each patch:

- Run its focused unit tests.
- Run the complete deterministic unit suite.
- Show the exact diff.
- Do not combine it with tug-of-war behavior changes.

The human then performs one seed-1001 fleet run to confirm that baseline throughput remains restored before any collision-avoidance redesign begins.

### Phase 3: Fix tug of war as one isolated feature

Implement the smallest coherent solution with these invariants:

1. True 2-D CPA rejects safe parallel/laterally separated passes.
2. Right-of-way is stable for the complete encounter:
   - physically inside corridor owner first;
   - otherwise stable robot-ID tie-breaker.
3. Winner continues unless emergency collision distance is reached.
4. Loser uses a WHCA-reserved right-hand sublane or a safe holding point.
5. The chosen physical sublane and its reservations refer to the same cells/geometry.
6. No ORCA-generated forward motion from a stop.
7. No ORCA-generated reverse motion.
8. No permanent latch: exit conditions are geometric, session-aware, and bounded for robots not physically inside a mutex resource.
9. The maneuver remains inside the 3 m main corridor with the configured footprint and safety margin.
10. Junction behavior is explicitly tested; do not apply a segment-normal offset blindly through corners.

Do not simultaneously alter carrier time integration or general PathFollower behavior.

### Phase 4: Consider carrier timing only after fleet behavior is stable

Carrier timing is a separate performance project. It requires an offline closed-loop replay that demonstrates comparable arrival time and bounded heading error across different callback intervals.

No actual-time carrier patch should be accepted merely because distance integration is mathematically complete. It must preserve controller stability as well.

---

## Required deterministic tests

Before the human runs Gazebo, the agent must provide focused tests for:

### Baseline preservation

- Existing algorithm, carrier, navigation, CBBA, mutex, and telemetry tests still pass.
- Exact expected dock and spawn headings agree.
- No tracked dataset is modified.

### Tug of war

- Head-on, same centerline.
- Head-on, already laterally separated by one robot width.
- Parallel same-direction following.
- Crossing trajectories with safe CPA miss distance.
- Stationary peer.
- Corridor owner versus outside waiter.
- Simultaneous three- and four-robot junction entry.
- Stable right-of-way despite peer message reordering.
- Encounter completion: both robots eventually pass, not merely avoid collision for a short unit-test window.

### Differential-drive feasibility

- Every commanded lateral maneuver is converted to a realizable route/heading.
- Angular and linear commands remain within configured limits.
- No translation is produced while PathFollower requests a turn-in-place stop.
- Reverse is rejected unless the recovery subsystem proves rear clearance.

### Carrier and Gazebo synchronization

- Request success, rejection, timeout, and exception paths.
- Commanded pose versus subsequently observed pose.
- Bounded logging under persistent failures.
- No use of Gazebo ground truth as physical localization authority.

### Telemetry

- Completion is counted exactly once.
- Manual interruption is not labeled normal completion.
- Dirty trees receive a deterministic provenance identifier distinct from clean `HEAD`.
- The manifest accurately describes whether pipeline diagnostics are enabled.

---

## Human-run acceptance gates

Only the human operator may run these validations.

### Restoration acceptance run

Use the same four-robot seed-1001 workload as the verified baseline. The restored baseline plus low-coupling corrections should satisfy:

- 20/20 tasks complete.
- Wall time no worse than 10% above the slower verified baseline: target at or below approximately 1115 seconds on comparable hardware/load.
- Each robot completes at least one task.
- Fleet turning fraction remains close to the baseline and below 20% per robot over the full run.
- No robot remains commanded forward while translating less than 0.05 m for more than 10 wall seconds without an explicit logged safety/corridor reason.
- Mean map-to-Gazebo position error below 0.05 m.
- Maximum transient map-to-Gazebo error below 0.25 m, with no accumulating divergence.
- No orphan process is present before or after the run.

### Tug-of-war acceptance run

After the isolated encounter fix:

- Two opposing robots select deterministic compatible passing behavior.
- Both complete the encounter without repeated forward/backward direction changes.
- A robot-width lateral separation does not trigger a false head-on stop.
- Neither robot leaves the main-corridor boundary.
- No collision, permanent stop, or corridor-token leak occurs.
- The full seed-1001 fleet result remains within the restored throughput envelope.

If any gate fails, do not layer another patch on top. Revert only that isolated patch, retain its logs, and diagnose before proceeding.

---

## Dataset handling

Existing data is not worthless, but runs must be quality-labeled.

- `192735` and `194553` are strong healthy-baseline candidates.
- `123429` is a dock-heading inversion/fleet-degradation example.
- `125652` is a controller-oscillation, ORCA-veto, and map/Gazebo-divergence example.
- `134106` is a post-lookahead/sublane experimental regression with carrier-dispatch failures and single-robot throughput domination.

Do not mix these classes as equivalent training samples. Add offline labels such as:

```text
software_state_id
git_head
git_dirty
source_diff_hash
run_quality
fault_class
usable_for_nominal_policy_training
usable_for_anomaly_training
```

The failure runs are useful for anomaly detection, deadlock classification, recovery learning, and negative examples. They should be excluded from nominal ETA, throughput, and successful-navigation policy training unless the model explicitly receives the fault/state labels.

---

## Required final deliverables from the implementation agent

Before requesting a human simulation run, provide:

1. Exact restoration branch/worktree and target commit.
2. Preservation record for the original dirty tree and untracked files.
3. File-by-file diff from `ce79fb7`.
4. A statement identifying which later changes were discarded and why.
5. Unit-test and build results.
6. A short human run command/reference without executing it.
7. A metric checklist for comparing the new run with `192735` and `194553`.
8. A rollback instruction for each isolated patch.

Stop and wait for human approval at every governance gate.

---

## Final directive to the assigned AI agent

The goal is not to preserve the maximum amount of recent code. The goal is to recover the empirically verified fleet with the smallest justified delta.

Use commit `ce79fb7` as the behavioral baseline. Preserve user data and forensic evidence. Reapply only proven operational, dock, boundary, and telemetry corrections first. Prove that throughput is restored. Then address tug of war as one isolated, differential-drive-feasible, reservation-consistent feature. Do not run the final simulation yourself.
