# SIH AMR Warehouse Simulation

This repository is the ROS 2 Jazzy fleet overlay for the custom Gazebo Harmonic
warehouse at `~/amr_ws/src/warehouse_world_custom`.

## Current verified baseline

On this bare-metal Ubuntu dual-boot installation, the clean warehouse was started headlessly and four TurtleBot
4 Lite AMRs were inserted sequentially. Each robot passed entity creation,
namespaced controller activation, odometry, LiDAR, and Twist-command gates.
All four `/robot_N/turtlebot4` entities were present simultaneously, and robot 4
was moved successfully through `/robot_4/cmd_vel`.

The launcher avoids Harmonic's second-robot renderer crash by loading Gazebo's
shared Sensors system only with robot 1. Robots 2–4 keep their own camera and
LiDAR definitions, but do not instantiate a duplicate renderer.

## One-time build

```bash
source /opt/ros/jazzy/setup.bash
cd ~/amr_ws
colcon build --symlink-install --packages-select sih_amr_interfaces sih_amr_fleet
source install/setup.bash
```

Create a local pose file from the tracked, verified example:

```bash
mkdir -p ~/.config
cp ~/amr_ws/src/SIH/scripts/sih_amr_poses.env.example \
  ~/.config/sih_amr_poses.env
```

Those defaults place robots 1–4 on charging pads 1–4 respectively in the
south-wall charging bay. Do not change them without first checking clearance
in `warehouse_clean.sdf`.

## One-command four-AMR launch

Install the local aliases once after cloning or pulling this repository:

```bash
alias_source='source "$HOME/amr_ws/src/SIH/scripts/sih_amr_aliases.sh"'
grep -qxF "$alias_source" ~/.bashrc || printf '\n%s\n' "$alias_source" >> ~/.bashrc
source ~/.bashrc
```

For the complete static-algorithm baseline—Gazebo, four AMRs, every fleet
node, reproducible random tasks, and passive telemetry—run:

```bash
bash ~/amr_ws/src/SIH/scripts/run_baseline_random_fleet.sh
```

It uses the same verified pose file as the four-AMR launcher and writes the
JSONL run record beneath `~/amr_ws/log/four_amr_runs/`. Press `Ctrl+C` in that
terminal to stop the entire run cleanly.

Then use one of these commands from a fresh terminal:

```bash
amr4
amr4_standard
amr4_headless
```

`warehouse` starts a managed warehouse-only session. It uses a clean packaged
GUI configuration instead of the mutable `~/.gz` layout. Closing the GUI or
pressing `Ctrl+C` in its terminal shuts down both GUI and server. Do not press
`Ctrl+C` after the prompt has returned: the session has already stopped.

- `amr4` starts the verified Lite baseline, the clock bridge, four sequential
  AMR spawns, and finally attaches the Gazebo GUI.
- `amr4_standard` uses the full TurtleBot 4 Standard body. Its upper sensor
  plate and four tower standoffs are the parts absent from the Lite model.
- `amr4_headless` is the Lite baseline with no GUI, intended for sustained
  automated runs.

The verified renderer is `ogre2` for both the server and GUI. The RTX 3070 was
active during the four-AMR run. Legacy `ogre` remains available as the fallback
through `gzogre` if a future driver or GUI regression requires it.

`amr4` uses the same clean GUI configuration and retries one GUI startup if a
transient Qt / EGL initialization failure occurs. The recurring `libEGL ...
driver (null)` / `failed to create dri2 screen` lines are Mesa's failed probe;
confirm the actual renderer with `glxinfo -B`, which should report the RTX 3070.

The command advances only after each robot has passed its own gate; it never
starts robot N+1 after a failed robot N. It prints a per-run log directory under
`~/amr_ws/log/four_amr_runs/` and the process-group PIDs. The GUI is deliberately
attached only after all four spawn gates have passed.

Do not start a second run while a previous server or AMR launch is running. The
launcher detects that condition and refuses to overlap simulations.

## Manual command

The one-command launcher is preferred. For diagnosis, the underlying command is:

```bash
source ~/.config/sih_amr_poses.env
MODEL=lite START_CHARGING=false START_FLEET=false START_GUI=true \
  bash ~/amr_ws/src/SIH/scripts/launch_four_amrs.sh
```

It performs this ordered workflow:

1. Starts server-only Gazebo with `warehouse_clean.sdf`.
2. Starts the Gazebo-to-ROS `/clock` bridge.
3. Spawns and validates `robot_1`, `robot_2`, `robot_3`, then `robot_4`.
4. Starts the Ogre2 GUI client only after success, then keeps this terminal
   attached. Press `Ctrl+C` in that terminal to shut down the GUI, server,
   clock bridge, and all AMRs together.

Use the following checks while it runs:

```bash
gz model --list | grep 'robot_[1-4]/turtlebot4'
ros2 control list_controllers -c /robot_1/controller_manager
ros2 topic echo --once /robot_4/scan
```

Each robot should show active `joint_state_broadcaster` and
`diffdrive_controller`.

## Fleet interfaces and random warehouse tasks

The fleet layer supplies namespaced `/robot_N/odom`, `/robot_N/cmd_vel`,
`/robot_N/scan`, `/robot_N/tf`, `/robot_N/tf_static`, `/robot_N/amcl_pose`,
`/robot_N/path`, `/robot_N/status`, local `/robot_N/state` (pose and velocity),
the global static `/map`, charging battery/state topics, and fleet health/status
telemetry. The project-specific battery is at
`/robot_N/charging/battery_percent` and
`/robot_N/charging/battery_state`; simulator-owned TurtleBot battery data stays
at `/robot_N/battery_state`.

Enable a repeatable live workload and the non-control JSONL recorder with:

```bash
ros2 launch sih_amr_fleet fleet.launch.py random_tasks:=true record_data:=true \
  data_file:=/tmp/sih_amr_fleet_telemetry.jsonl
```

`random_task_generator_node` chooses both pickup and drop-off only from safe
centreline points in narrow aisles between shelf rows (never shelves, green
main corridors, or the charging bay). CBBA allocates each task; the assigned
AMR pauses for independently sampled 2–5 second durations at pickup and
drop-off. `task_execution_node` publishes the lifecycle on
`/fleet/task_execution_status`, holds the controller during each wait, and
retires completed tasks from all allocators. Use `seed:=...`,
`min_interval_s:=...`, and `max_interval_s:=...` when launching the generator
directly to reproduce or vary an experiment.

`data_collection_node` records robot state/velocity, health and battery,
trajectory reservations, task consensus/execution, corridor events, and safety
decisions to JSON Lines. It is observational only: logging failure cannot
influence robot control.

`warehouse_tasks.py` is also the lane-network source of truth. It defines all
48 narrow storage-lane centrelines: 46 horizontal lanes between shelf rows and
the two widened vertical centre lanes. Random task endpoints lie exactly on
those centrelines. The corridor sweep follows the same network, entering and
exiting each storage lane only through its adjacent main corridor.

`warehouse_map_node` produces `/map` from the locked shelf layout. The
simulation localizer produces AMCL-compatible `/robot_N/amcl_pose` values from
Gazebo odometry transformed into the map frame; it is not a replacement for a
real LiDAR+AMCL localization stack on physical robots. `path` is the standard
`nav_msgs/Path` view of the project `planned_route` contract.

## One-AMR corridor sweep

With the clean warehouse already running, build the overlay and run:

```bash
cd ~/amr_ws
colcon build --symlink-install --packages-select sih_amr_interfaces sih_amr_fleet
bash ~/amr_ws/src/SIH/scripts/run_warehouse_corridor_sweep.sh
```

It spawns `corridor_sweep` at the south-west junction, drives every green main
corridor, then traverses the widened 1.1554 m centre lanes in the middle and
north blocks. The sweep uses odometry for closed-loop waypoint following and
stops on a close LiDAR return or `Ctrl+C`. It does not attempt the remaining
0.4028 m shelf gaps because those are too narrow for an AMR. Use a new
`ROBOT_NAME` or restart the clean world before repeating the test.

## Fleet speed and footprint limits

The simulated drivetrain and corridor-sweep ceiling is **6.0 m/s**. Direct
`fleet.launch.py` route tracking defaults to 1.0 m/s; the simulation-only
`launch_four_amrs.sh` defaults to 4.0 m/s for shorter validation/data runs.
Set `FLEET_TRACKING_SPEED_MPS=1.0` for real-world-like timing. WHCA* derives
its reservation slot duration from the configured tracking speed. These
Gazebo settings must never be reused blindly on physical TurtleBot hardware;
the local Safety Supervisor remains final authority.

The fleet keeps its established 0.56 m diameter planning footprint (0.28 m
radius). The narrow centre lanes are 1.1554 m wide, so a requested 0.95 m
diameter AMR would have only 0.1027 m clearance on either side. It would not
have enough margin to steer or turn safely; the robot footprint therefore has
not been enlarged.

## Lite versus Standard visual model

The verified multi-AMR test used `MODEL=lite` to reduce rendering load. Lite
is a low-profile TurtleBot and intentionally has no Standard tower standoffs or
upper sensor plate. It is not a partially spawned robot. Use
`amr4_standard` when the full upper assembly is required visually.

## Sustained simulation and dataset collection

The host contains a Ryzen 5 5600X (6 cores / 12 threads), 16 GiB RAM, an RTX
3070, and ample NVMe space. Host-context validation confirmed NVIDIA driver
595.84, direct NVIDIA OpenGL 4.6, and Gazebo on the GPU. Missing
`/dev/nvidia*` and `/dev/dri` in an earlier Codex run were sandbox device
isolation, not a host driver fault; no driver change is required.

The default `SENSOR_PROFILE=fleet` removes the Lite model's unused RGB-D camera
and eleven unused cliff/IR GPU lidars per robot, retaining the 20 Hz navigation
LiDAR and contact sensor. Use `SENSOR_PROFILE=full` only when those sensors are
actually needed. Telemetry schema 0.3 records simulation and wall time so every
run can report achieved real-time factor instead of assuming the SDF target was
met.

The host-context headless run
`/tmp/sih_headless_validation_20260908_gpu_2107` measured only `0.097` achieved
real-time factor while Gazebo used about 107% CPU and GPU utilisation was about
36%. This configuration is physics/CPU-bound, not GPU-bound. Its telemetry
also confirms the accelerated route setting is active: peak robot speed was
about 3.9997 m/s. Increasing the SDF `real_time_factor` above one cannot make a
simulation that currently reaches only 0.097 run faster; a later, separately
validated throughput profile must reduce physics work (for example, a coarser
step and simpler collision geometry). Keep correctness validation on the
canonical 1 ms physics profile so timing changes do not hide control defects.

Long runs are feasible, but split them into restartable chunks, record seeds and
scenario metadata, monitor disk and system memory, and disable the GUI. A
single 1,000-hour uninterrupted run is fragile against host sleep, system
updates, driver resets, and disk exhaustion.

Each TurtleBot OAK-D camera is configured at 320×240 and 30 Hz. Four cameras at
full rate produce 432,000 images per simulated hour. Even JPEG-compressed to an
optimistic 30–100 KiB per image, that is about 13–43 GB per simulated hour;
the current free disk cannot hold an unrestricted collection. For an object or
obstacle classifier, capture at 1–5 Hz, JPEG-compress, save labels and poses,
randomize lighting/clutter/robot poses, and use short dataset episodes. For
path planning, store compact state/action/occupancy data rather than camera
frames unless vision is part of the planner.

The cameras are present in Gazebo, but the current minimal four-AMR launch
bridges LiDAR only. Add a namespaced image bridge and a bounded recorder before
attempting visual dataset collection; do not expect `/robot_N/camera/...` ROS
topics or automatic image files from the current baseline.
