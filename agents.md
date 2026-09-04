# Gazebo / ROS 2 Warehouse Simulation Progress

## Environment

- Host computer: MacBook Air with Apple M5 chip.
- Virtualization: VMware Fusion.
- Simulation OS: Ubuntu ARM64 running inside the VM.
- ROS 2 distribution: Jazzy.
- Gazebo: Gazebo Sim Harmonic, version 8.11.0 (`gz sim`).
- ROS workspace in the VM: `~/amr_ws`.
- Warehouse source directory: `~/amr_ws/src/warehouse_world`.
- Source repository: `aws-robotics/aws-robomaker-small-warehouse-world`.
- Source branch used: `ros2`.
- The AWS repository is an older Gazebo Classic-oriented package; it is not directly compatible with the installed Jazzy/Harmonic `gazebo_ros` build workflow.

## Completed successfully

1. ROS 2 Jazzy and Gazebo Sim Harmonic were installed and confirmed in the Ubuntu VM.
2. The AWS Small Warehouse World repository was cloned successfully into the ROS workspace.
3. The repository models and world files were confirmed to exist, including `model.sdf` and `model.config` files for the warehouse roof, walls, shelves, ground, clutter, lamps, buckets, trash can, and pallet jack.
4. The initial `colcon build` failure was diagnosed correctly: the old package requires `gazebo_ros`, which is a Gazebo Classic dependency and was not available in the Jazzy/Harmonic environment.
5. `COLCON_IGNORE` was added to the old package so it no longer blocks the rest of the workspace build.
6. The warehouse world was launched successfully with modern Gazebo using `gz sim`.
7. Gazebo model-resource paths were corrected. The working runtime path includes both:

   ```bash
   $HOME/amr_ws/src/warehouse_world/models
   $HOME/amr_ws/src/warehouse_world
   ```

   Both are needed because the legacy models use `model://...` and `file://models/...` references.
8. Gazebo Harmonic rejected legacy inertia data for the roof and ground models. Those two models were made static and their legacy inertial blocks were removed. Backup files were created before editing:

   ```text
   model.sdf.backup
   ```

9. The warehouse successfully loaded after the static-model/inertia correction.
10. A `warehouse` shell alias was created to launch the warehouse with the correct resource path.
11. Gazebo’s GUI was used to inspect and manipulate warehouse entities.
12. The world was saved as a modern SDF file named `custom_warehouse.sdf`.
13. The Gazebo GUI showed the custom file in the model inspector’s `Source File Path`, confirming that the custom SDF world was loaded.

## Tested and observed

- The warehouse environment opens in Gazebo Harmonic.
- The roof, ground, walls, shelves, and other legacy warehouse assets render.
- Gazebo can display the entity tree and component inspector.
- Models can be selected and manipulated through the GUI.
- Static environment geometry can remain fixed without invalid-inertia startup failure.
- Saving the GUI configuration and saving the world are separate operations. The custom world was saved as `custom_warehouse.sdf`.

## Known environment limitations

- The VM reported `VMware: No 3D enabled` and `libEGL` warnings. Gazebo still opened, but GUI performance and stability may be reduced. VMware Fusion 3D acceleration should be enabled if available.
- The AWS warehouse package itself is not being built as a normal Jazzy package because it depends on Gazebo Classic’s `gazebo_ros`.
- The current workflow launches the world directly with `gz sim`; it does not yet use a custom ROS 2 launch package for the warehouse.

## Not completed or not yet tested

- No AMR robot model has been added yet.
- No differential-drive controller or `ros2_control` setup has been added.
- No LiDAR, IMU, camera, wheel encoder, or odometry topics have been tested.
- No `ros_gz_bridge` topic bridge has been configured or tested.
- No SLAM Toolbox or Nav2 navigation test has been completed.
- No multi-AMR test has been completed.
- No charging-station model or charging behavior has been implemented.
- The warehouse has not yet been fully redesigned into the intended three-row shelf layout.
- The warehouse walls have not yet been formally extended and validated for AMR clearance.
- GUI-edited object poses should be verified by closing Gazebo and relaunching the saved `custom_warehouse.sdf`.

## Current launch command

```bash
source /opt/ros/jazzy/setup.bash

GZ_SIM_RESOURCE_PATH="$HOME/amr_ws/src/warehouse_world/models:$HOME/amr_ws/src/warehouse_world" \
gz sim -r \
"$HOME/amr_ws/src/warehouse_world/worlds/small_warehouse/custom_warehouse.sdf"
```

The `warehouse` alias should point to the same custom SDF file and resource paths.

## Recommended next development order

1. Make a clean custom warehouse world with three shelf rows, wide AMR aisles, an extended wall, and no unnecessary box clutter.
2. Add a simple static charging dock model.
3. Add a basic differential-drive AMR model.
4. Add and verify `/cmd_vel`, odometry, TF, LiDAR, IMU, and camera topics through `ros_gz_bridge`.
5. Drive the AMR manually with a ROS 2 teleoperation node.
6. Generate a map and test SLAM Toolbox and Nav2.
7. Add charging detection and battery behavior as ROS 2 logic.

---

# Cross-device handoff and current ROS 2 implementation

This section is the current handoff for moving development from the Mac/VM to the Windows PC. It supersedes older “not completed” notes above where they conflict.

## Project repository and workspace

- The Git repository is the SIH root directory.
- The ROS 2 source overlay is now in `src/` in this repository.
- ROS package source names:
  - `src/sih_amr_interfaces` — custom ROS 2 message definitions.
  - `src/sih_amr_fleet` — Python `rclpy` fleet nodes, launch file, map, scenario, and pure algorithm tests.
- The legacy warehouse package is external to this repository in the simulation workspace:

  ```text
  /home/rtsws/amr_ws/src/warehouse_world
  ```

- The current saved warehouse world is:

  ```text
  /home/rtsws/amr_ws/src/warehouse_world/worlds/small_warehouse/custom_warehouse.sdf
  ```

- The warehouse world needs this runtime resource path:

  ```text
  /home/rtsws/amr_ws/src/warehouse_world/models:/home/rtsws/amr_ws/src/warehouse_world
  ```

- The repository contains `ROS2_FLEET_IMPLEMENTATION.md`, which documents package contents, node responsibilities, topics, QoS, build steps, and known limitations.

## ROS 2 fleet code already added

The per-AMR nodes are namespaced as `/robot_1`, `/robot_2`, `/robot_3` and communicate through shared `/fleet/*` topics where coordination is required.

| Node | Main responsibility |
|---|---|
| `localization_node` | Converts simulator `odom` into validated fleet state. |
| `local_costmap_node` | Converts `scan` into a local occupancy grid and nearest-obstacle distance. |
| `blockage_detector_node` | Publishes persistent, TTL-bound LiDAR blockage observations. |
| `peer_tracker_node` | Rejects stale/session-old state and predicts peer motion with uncertainty. |
| `health_node` | Publishes battery/safety/communication health. |
| `cbba_node` | Bounded one-task CBBA/CBAA-style bidding, winner claims, leases, and assignment epochs. |
| `whca_planner_node` | Rolling time-indexed grid A* using peer trajectory reservations. |
| `reservation_manager_node` | Publishes expiring `/fleet/trajectory_intent` messages. |
| `corridor_mutex_node` | Ricart–Agrawala-style REQUEST/GRANT/DEFER/ENTER/EXIT protocol. |
| `path_follower_node` | Simple waypoint-to-velocity controller for the simulator. |
| `orca_node` | Lightweight uncertainty-inflated reciprocal velocity avoidance candidate. |
| `safety_supervisor_node` | Final local LiDAR/braking/E-stop veto before `cmd_vel`. |
| `task_scenario_node` | Reproducible task announcements; not a fleet manager. |
| `dashboard_bridge_node` | Read-only JSON telemetry on `/fleet/dashboard_telemetry`. |

ML and DL are intentionally not implemented in this overlay. Future ML route-cost estimates remain optional soft planning costs and must never command motors.

## Recommended simulator model

Use TurtleBot 4 Standard for the first integration. The official Jazzy branch targets Ubuntu 24.04 + Gazebo Harmonic and its spawn launch includes a compatible simulated standard dock, ROS bridges, LiDAR, camera, IMU-related interfaces, odometry, TF, battery state, and differential-drive control.

Install on Ubuntu Jazzy:

```bash
sudo apt install ros-jazzy-turtlebot4-simulator
```

Do not edit installed files under `/opt/ros`. Add project-specific payload/tray, bumper, battery, or dock behavior in a project-owned description package later.

## New Windows machine recommendation

The selected setup is a Windows PC with Ryzen 6000-series CPU, RTX 3070 8 GB, and 16 GB RAM, running Ubuntu 24.04 in VirtualBox. Keep only Codex/background applications open while simulating.

Suggested VirtualBox settings:

- 6 virtual CPU cores;
- 8 GB guest RAM (6 GB if Windows becomes memory pressured);
- 128 MB video memory;
- Enable 3D acceleration;
- at least 60 GB virtual disk;
- NAT networking is sufficient for a single-machine simulation.

The RTX 3070 is more than adequate, but VirtualBox does not normally pass the physical GPU directly to the guest. If `glxinfo -B` reports `llvmpipe`, Gazebo is software-rendered. Use Gazebo headless mode for tests or correct VirtualBox 3D acceleration before adding multiple robots.

## Windows Ubuntu VM installation sequence

Use Ubuntu 24.04 LTS Desktop. In the VM:

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y curl git build-essential mesa-utils
source /etc/os-release
echo "$PRETTY_NAME"
glxinfo -B
```

Install ROS 2 Jazzy using the official ROS 2 Ubuntu deb instructions, then:

```bash
sudo apt install -y ros-jazzy-desktop ros-dev-tools
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Install the supported Gazebo/ROS pair and control/simulator packages:

```bash
sudo apt install -y ros-jazzy-ros-gz ros-jazzy-gz-ros2-control
sudo apt install -y ros-jazzy-turtlebot4-simulator ros-jazzy-teleop-twist-keyboard
```

Verify before copying the project:

```bash
ros2 doctor --report
gz sim --version
ros2 pkg prefix ros_gz_bridge
gz sim shapes.sdf
```

The `shapes.sdf` GUI test must work before testing the warehouse. If rendering is unstable, use:

```bash
gz sim -s -r /home/rtsws/amr_ws/src/warehouse_world/worlds/small_warehouse/custom_warehouse.sdf
```

## Copy/build this repository on the Windows Ubuntu VM

Clone or copy this Git repository into Linux storage, then build:

```bash
mkdir -p ~/amr_ws/src
cd ~/amr_ws
# copy/clone the SIH repository here so its src/ directory is ~/amr_ws/src/
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

The legacy `warehouse_world` directory must also be copied into `~/amr_ws/src/warehouse_world`. It is not built as a normal Jazzy package because it depends on Gazebo Classic `gazebo_ros`; launch its SDF directly.

## Current correct launch order

Use one terminal per stage and set the same domain in every ROS terminal:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
```

Terminal 1 — start the custom warehouse:

```bash
export GZ_SIM_RESOURCE_PATH=/home/rtsws/amr_ws/src/warehouse_world/models:/home/rtsws/amr_ws/src/warehouse_world
gz sim -r /home/rtsws/amr_ws/src/warehouse_world/worlds/small_warehouse/custom_warehouse.sdf
```

Terminal 2 — spawn one TurtleBot 4 into the already-running world:

```bash
ros2 launch turtlebot4_gz_bringup turtlebot4_spawn.launch.py \
  namespace:=robot_1 model:=standard \
  x:=2.0 y:=2.0 z:=0.05 yaw:=0.0
```

The `turtlebot4_gz.launch.py` command starts the TurtleBot example world; it is not the command for inserting a robot into `custom_warehouse.sdf`. The spawn launch is the correct command after the custom world is running.

Verify:

```bash
gz model --list
ros2 node list | grep robot_1
ros2 topic list | grep '^/robot_1/'
```

Expected interfaces include `/robot_1/cmd_vel`, `/robot_1/odom`, `/robot_1/scan`, `/robot_1/battery_state`, `/robot_1/tf`, and `/robot_1/tf_static`.

Test carefully:

```bash
ros2 topic pub --rate 10 /robot_1/cmd_vel \
  geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

Only after one robot works should robots 2 and 3 be spawned with unique namespaces and clear poses.

## Previous launch failure and recovery

The TurtleBot package was found at `/opt/ros/jazzy`, so installation was successful. The failed run showed many processes exiting with:

```text
Failed to find a free participant index for domain 0
```

This is a Cyclone DDS participant/resource collision, commonly caused by stale or repeated ROS launch processes. Recovery:

1. Press `Ctrl+C` in the launch terminal.
2. Inspect remaining processes:

   ```bash
   pgrep -af 'turtlebot4|parameter_bridge|ros_gz_sim|gz sim|robot_state_publisher'
   ```

3. Terminate only stale PIDs from that failed launch with `kill PID_NUMBER`.
4. Run `ros2 daemon stop`.
5. Use `ROS_DOMAIN_ID=42` consistently for the new run.

The KDL root-link inertia warnings and Qt binding-loop warnings were not the main failure. The previous command also started the bundled TurtleBot world, proven by `/world/warehouse/model/turtlebot4`, rather than the project’s saved custom warehouse.

## Charging station plan

For the first demo, keep the TurtleBot 4 standard dock spawned with each robot. It is already compatible with the TurtleBot model and exposes dock/battery-related interfaces. Later add a project-owned shared charging bay with:

- a static SDF dock visual and collision geometry;
- a marked approach pose;
- a camera-visible fiducial or contact/charging-zone sensor;
- a battery node that increases charge only while correctly docked;
- no direct motor-control authority outside the normal Safety Supervisor path.

## Immediate next checkpoint

On the Windows Ubuntu VM, do not launch the full fleet yet. First collect and record successful output from:

```bash
glxinfo -B
gz sim --version
ros2 doctor --report
gz model --list
ros2 topic list | grep '^/robot_1/'
```

Then validate one TurtleBot in `custom_warehouse.sdf`, then add the other two, then launch `sih_amr_fleet fleet.launch.py`.

## Detailed warehouse-world troubleshooting handoff

This section records the exact warehouse-world failures and fixes so another Codex instance does not repeat the Gazebo Classic workflow.

### Error 1: `colcon build` could not find `gazebo_ros`

The initial build of the cloned AWS package failed with:

```text
CMake Error at CMakeLists.txt:11 (find_package):
Could not find a package configuration file provided by "gazebo_ros"
```

Cause: the AWS package's ROS 2 branch still depends on `gazebo_ros`, which belongs to the Gazebo Classic integration. The active environment uses ROS 2 Jazzy with modern Gazebo Harmonic (`gz sim` 8.11.0), whose integration uses `ros_gz` instead.

Resolution: the legacy package was not built as a normal Jazzy package. This marker was added:

```bash
touch ~/amr_ws/src/warehouse_world/COLCON_IGNORE
```

The world is launched directly with `gz sim`; do not try to fix this by adding the old `gazebo_ros` dependency to the Jazzy/Harmonic project.

### Error 2: mesh URI resolution failures

The first direct launch used only the `models` directory and produced errors like:

```text
uri [file://models/aws_robomaker_warehouse_RoofB_01/meshes/...DAE]
could not be resolved
```

The next attempt used only the package directory and produced errors like:

```text
Unable to find uri[model://aws_robomaker_warehouse_RoofB_01]
```

Cause: the legacy world uses both `model://...` references and internal `file://models/...` mesh references.

Resolution: both directories must be in `GZ_SIM_RESOURCE_PATH`, in this form:

```bash
export GZ_SIM_RESOURCE_PATH="$HOME/amr_ws/src/warehouse_world/models:$HOME/amr_ws/src/warehouse_world"
```

The same two paths are used by the `warehouse` alias.

### Error 3: invalid inertia during world loading

After the resource paths were corrected, Gazebo Harmonic reported:

```text
Error Code 19: Msg: A link named link has invalid inertia.
Error Code 9: Msg: Failed to load a world.
```

The invalid-inertia message appeared twice and was caused by legacy inertial data in the static roof and ground models. The affected files were:

```text
~/amr_ws/src/warehouse_world/models/aws_robomaker_warehouse_RoofB_01/model.sdf
~/amr_ws/src/warehouse_world/models/aws_robomaker_warehouse_GroundB_01/model.sdf
```

The following tag was inserted immediately inside each model, before its `<link>` tag:

```xml
<model name="aws_robomaker_warehouse_RoofB_01">
  <static>true</static>
```

```xml
<model name="aws_robomaker_warehouse_GroundB_01">
  <static>true</static>
```

The old `<inertial>...</inertial>` block was then removed from each of those two `model.sdf` files. Static environment geometry does not need dynamic inertial properties, and Harmonic rejected the legacy values before the world could load.

Backups were made before editing:

```text
~/amr_ws/src/warehouse_world/models/aws_robomaker_warehouse_RoofB_01/model.sdf.backup
~/amr_ws/src/warehouse_world/models/aws_robomaker_warehouse_GroundB_01/model.sdf.backup
```

Do not remove inertial blocks from the AMR, wheels, or any object that should move. The static/inertia change applies only to the roof and ground environment models unless a later error identifies another static asset with the same problem.

### Successful final warehouse launch

The warehouse then loaded successfully with:

```bash
source /opt/ros/jazzy/setup.bash

GZ_SIM_RESOURCE_PATH="$HOME/amr_ws/src/warehouse_world/models:$HOME/amr_ws/src/warehouse_world" \
gz sim -r \
"$HOME/amr_ws/src/warehouse_world/worlds/small_warehouse/custom_warehouse.sdf"
```

The saved custom world was visible in Gazebo's Component Inspector under `Source File Path`, confirming that `custom_warehouse.sdf` was the loaded world.

### GUI and save behavior

`Save client configuration` saves Gazebo's GUI layout, not object positions. To save warehouse edits, use the world-save action and save an SDF file such as:

```text
/home/rtsws/amr_ws/src/warehouse_world/worlds/small_warehouse/custom_warehouse.sdf
```

The `warehouse` alias must load `custom_warehouse.sdf`; if it loads the original `small_warehouse.world`, GUI edits will appear to have been lost.

### Remaining warnings and limitations

The VM printed:

```text
VMware: No 3D enabled
libEGL warning: egl: failed to create dri2 screen
```

These are VMware graphics-acceleration warnings. They did not prevent the warehouse from loading, but they can reduce GUI performance or stability. Enable VMware Fusion 3D acceleration if available.

The following messages were also observed while manipulating entities:

```text
Internal error: A physics entity ptr with an ID ... does not exist.
```

These occurred during GUI manipulation of the legacy world and are not the startup fix. They should be treated as a possible Harmonic/legacy-world GUI limitation until reproduced in a clean saved SDF.
