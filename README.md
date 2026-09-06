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

Do not change those poses without first checking clearance in
`warehouse_clean.sdf`.

## One-command four-AMR launch

Install the local aliases once after cloning or pulling this repository:

```bash
alias_source='source "$HOME/amr_ws/src/SIH/scripts/sih_amr_aliases.sh"'
grep -qxF "$alias_source" ~/.bashrc || printf '\n%s\n' "$alias_source" >> ~/.bashrc
source ~/.bashrc
```

Then use one of these commands from a fresh terminal:

```bash
amr4
amr4_standard
amr4_headless
```

- `amr4` starts the verified Lite baseline, the clock bridge, four sequential
  AMR spawns, and finally attaches the Gazebo GUI.
- `amr4_standard` uses the full TurtleBot 4 Standard body. Its upper sensor
  plate and four tower standoffs are the parts absent from the Lite model.
- `amr4_headless` is the Lite baseline with no GUI, intended for sustained
  automated runs.

The verified renderer is `ogre2` for both the server and GUI. The RTX 3070 was
active during the four-AMR run. Legacy `ogre` remains available as the fallback
through `gzogre` if a future driver or GUI regression requires it.

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

## Lite versus Standard visual model

The verified multi-AMR test used `MODEL=lite` to reduce rendering load. Lite
is a low-profile TurtleBot and intentionally has no Standard tower standoffs or
upper sensor plate. It is not a partially spawned robot. Use
`amr4_standard` when the full upper assembly is required visually.

## Sustained simulation and dataset collection

This is a bare-metal Ubuntu installation with a Ryzen 5 5600X (6 cores / 12
threads), 16 GiB RAM, and an RTX 3070, so all of that hardware is available to
Gazebo. The NVIDIA 595.84 driver was verified with `nvidia-smi`, and a four-AMR
Ogre2 run used about 1 GiB of RTX memory at a real-time factor of about `0.78`.
Before a long visual run, recheck the stack with `nvidia-smi` and the actual
OpenGL renderer with `glxinfo -B` after a driver or desktop update.

The verified four-lite run with the GUI attached measured a real-time factor of
about `0.78`. At that rate, 1,000 simulated hours takes roughly 1,280 wall-clock
hours (about 53 days). Headless mode should be used for long runs and may be
faster, but measure its real-time factor first; do not assume it will be faster
than real time.

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
