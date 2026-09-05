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

---

# Latest Windows VM handoff

This section records the current state after the Windows-PC setup session. It supersedes older path examples where they differ.

## VM and rendering

- Host: Windows PC with Ryzen 6000-series CPU, RTX 3070, and 16 GB RAM.
- Guest: Ubuntu 24.04.4 LTS x86_64 in VirtualBox; Linux user is `rtsws`.
- VirtualBox settings: VMSVGA, 128 MB video memory, 4–6 CPUs, about 8 GB RAM when Windows has sufficient memory, and 3D acceleration enabled.
- Guest Additions packages were installed with `virtualbox-guest-utils` and `virtualbox-guest-x11`; this enables clipboard sharing and automatic display resizing. VirtualBox Shared Clipboard is set to Bidirectional.
- Gazebo's default Ogre2 renderer produces a black viewport in this VM. Launch Gazebo with `--render-engine ogre`; `QT_QPA_PLATFORM=xcb` is also useful for the Ubuntu Wayland/Qt issue.

## Installed software

- ROS 2 Jazzy Desktop and `ros-dev-tools` are installed and sourced from `/opt/ros/jazzy/setup.bash`.
- Gazebo Sim Harmonic 8.x is installed through the Jazzy vendor packages, including `ros-jazzy-ros-gz`, `ros-jazzy-ros-gz-bridge`, `ros-jazzy-ros-gz-image`, `ros-jazzy-ros-gz-sim`, and `ros-jazzy-gz-ros2-control`.
- TurtleBot 4 simulator, teleoperation, Nav2, SLAM Toolbox, RViz, ROS 2 control, and controller packages are installed.
- `gz-tools` and `gz-tools2` were not available as apt package names in this setup and are not needed; `gz sim` is already available.
- All ROS terminals must use `export ROS_DOMAIN_ID=42`.

## Warehouse copies and fixes

- The AWS repository was cloned from its `ros2` branch into:
  - `~/amr_ws/src/warehouse_world_original` — preserved reference copy; do not edit.
  - `~/amr_ws/src/warehouse_world_custom` — working copy.
- Both legacy copies have `COLCON_IGNORE`; they are launched directly with `gz sim`, not built with `colcon`, because the package depends on Gazebo Classic `gazebo_ros`.
- The custom copy uses both resource roots:

  ```bash
  export GZ_SIM_RESOURCE_PATH="$HOME/amr_ws/src/warehouse_world_custom/models:$HOME/amr_ws/src/warehouse_world_custom:/opt/ros/jazzy/share"
  ```

  `/opt/ros/jazzy/share` is required after TurtleBot meshes are included because the SDF references `model://turtlebot4_description` and `model://irobot_create_description`.
- In the custom copy only, the roof and ground models were made static and their legacy `<inertial>` blocks removed. Backups named `model.sdf.backup` exist beside the edited files. Do not remove inertia from moving models.
- The edited custom warehouse was saved as:

  ```text
  ~/amr_ws/src/warehouse_world_custom/worlds/small_warehouse/custom_warehouse.sdf
  ```

- Gazebo `Ctrl+S` saves client/GUI configuration, not the world. Save world changes through the top-left menu's `Save world as...`, overwriting the intended SDF, then close and relaunch it to verify persistence.

## Current warehouse alias

The working `warehouse` alias should launch the saved custom SDF with the correct paths and Ogre renderer. The equivalent command is:

```bash
ROS_DOMAIN_ID=42 QT_QPA_PLATFORM=xcb \
GZ_SIM_RESOURCE_PATH="$HOME/amr_ws/src/warehouse_world_custom/models:$HOME/amr_ws/src/warehouse_world_custom:/opt/ros/jazzy/share" \
gz sim -r --render-engine ogre \
"$HOME/amr_ws/src/warehouse_world_custom/worlds/small_warehouse/custom_warehouse.sdf"
```

Running `warehouse` now shows the TurtleBot because the world was saved after a TurtleBot spawn; the robot is therefore embedded in `custom_warehouse.sdf`. Do not spawn another `robot_1` until a clean warehouse-only baseline is restored or the embedded robot is removed.

## TurtleBot integration status

- `turtlebot4_gz_bringup` and `turtlebot4_spawn.launch.py` were found and launched successfully.
- The spawn output reported `Entity creation successful` for the robot and standard dock. Gazebo lists the models as `robot_1/turtlebot4` and `robot_1/standard_dock`.
- ROS topics under `/robot_1/` exist, including `/robot_1/scan`, `/robot_1/odom`, `/robot_1/cmd_vel`, `/robot_1/battery_state`, and TF topics.
- The standalone TurtleBot example world initialized the TurtleBot, bridges, and controllers, but its GUI became unresponsive while downloading Fuel models when run with the default renderer. The example world is not required for future work.
- In the custom warehouse test, `/robot_1/odom` had zero publishers and the controller manager was intermittently unavailable. The `gz_ros2_control` package and shared library are installed under `/opt/ros/jazzy`, so the remaining issue is to inspect the Gazebo server output and ensure the warehouse server, GUI, and spawn launch all use ROS domain 42. Do not add robots 2 or 3 yet.
- A useful low-load arrangement is: Terminal 1 runs `gz sim -s -r` on the custom SDF, Terminal 2 runs `gz sim -g --render-engine ogre`, and Terminal 3 runs the TurtleBot spawn launch. The server must be started after explicitly setting `ROS_DOMAIN_ID=42`.

## Next actions

1. Keep the original warehouse copy untouched and decide whether to remove the embedded TurtleBot from `custom_warehouse.sdf` or restore a warehouse-only saved baseline.
2. Relaunch the clean warehouse, spawn exactly one TurtleBot, and inspect the Gazebo server output for `gz_ros2_control` or `controller_manager` errors.
3. Confirm an active joint-state broadcaster, active diff-drive controller, `/robot_1/odom` publisher, and safe `/robot_1/cmd_vel` movement.
4. Only then continue with warehouse aisle/layout cleanup, the charging dock, and robots 2 and 3.

---

# Latest charging-pad and TurtleBot handoff (September 2026)

This section supersedes older Windows VM notes where they conflict.

## Windows-to-VM SSH access

- The Ubuntu VM is reachable from the Windows host through the forwarded SSH port:

  ```bash
  ssh -p 8322 rtsws@127.0.0.1
  ```

- Files can be copied between the Windows host and the VM with `scp` using the same host and port, for example:

  ```bash
  scp -P 8322 path/to/file rtsws@127.0.0.1:/home/rtsws/amr_ws/src/SIH/
  ```

- SSH keys for GitHub are configured inside the VM. GitHub CLI is available on the Windows host. Prefer the SSH connection above for VM diagnostics and ROS/Gazebo commands; use `scp` when the VM workspace needs a file from this repository.

- For Fast DDS shared-memory lock collisions in multi-robot launches, this Jazzy installation accepts `UDPv4`, not `UDP`:

  ```bash
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  ```

  Set it before starting the warehouse ROS/Gazebo process, the `/clock` bridge, and every robot launch. `UDP` is rejected and causes Fast DDS to fall back to its default transport configuration.

## Current working state

- The Windows VirtualBox Ubuntu VM is running ROS 2 Jazzy and Gazebo Harmonic.
- The active legacy world copy is `~/amr_ws/src/warehouse_world_custom`.
- The intended clean world alias target is:

  ```text
  ~/amr_ws/src/warehouse_world_custom/worlds/small_warehouse/warehouse_clean.sdf
  ```

- Gazebo needs Ogre in this VM: `QT_QPA_PLATFORM=xcb` and `--render-engine ogre`.
- The earlier fatal startup error is fixed:

  ```text
  Failed to load system plugin [libgz_ros2_control-system.so]
  ```

  Fix: include `/opt/ros/jazzy/lib` in `GZ_SIM_SYSTEM_PLUGIN_PATH`.
- `/robot_1/odom` now has one publisher and produces odometry messages.
- Gazebo simulation time is bridged successfully to ROS 2 with `ros_gz_bridge`; this resolves the repeated controller-manager `No clock received` warning.

## Required aliases

Add these to `~/.bashrc`, then reload with `source ~/.bashrc`.

```bash
alias warehouse='source /opt/ros/jazzy/setup.bash && GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/jazzy/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}" ROS_DOMAIN_ID=42 QT_QPA_PLATFORM=xcb GZ_SIM_RESOURCE_PATH="$HOME/amr_ws/src/sih_amr_fleet/models:$HOME/amr_ws/src/warehouse_world_custom/models:$HOME/amr_ws/src/warehouse_world_custom:/opt/ros/jazzy/share" gz sim -r --render-engine ogre "$HOME/amr_ws/src/warehouse_world_custom/worlds/small_warehouse/warehouse_clean.sdf"'

alias bridge_clock='source /opt/ros/jazzy/setup.bash && ROS_DOMAIN_ID=42 ros2 run ros_gz_bridge parameter_bridge "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"'

alias spawn_turtlebot='source /opt/ros/jazzy/setup.bash && source "$HOME/amr_ws/install/setup.bash" && ROS_DOMAIN_ID=42 ros2 launch sih_amr_fleet spawn_robot_1.launch.py namespace:=robot_1 model:=standard x:=2.0 y:=2.0 z:=0.05 yaw:=0.0'

alias start_charging='source /opt/ros/jazzy/setup.bash && source "$HOME/amr_ws/install/setup.bash" && ROS_DOMAIN_ID=42 ros2 run sih_amr_fleet charging_pad_node --ros-args -r __ns:=/robot_1 -p initial_battery_percent:=50.0'
```

Alias purpose:

- `warehouse` — starts Gazebo server and GUI with the correct world, paths, renderer, control-plugin path, and ROS domain.
- `bridge_clock` — starts the Gazebo-to-ROS `/clock` bridge. Leave it running.
- `spawn_turtlebot` — spawns exactly one `robot_1`; run only when Gazebo has no TurtleBot already.
- `start_charging` — starts the project charging detector/battery simulation for `/robot_1`.

Normal one-AMR launch order:

1. Terminal 1: `warehouse`
2. Terminal 2: `bridge_clock`
3. Terminal 3: `spawn_turtlebot` only if Entity Tree and `gz model --list` show no existing TurtleBot.
4. Terminal 4: `start_charging`
5. Terminal 5: inspect topics, e.g. `ros2 topic echo /robot_1/charging/is_docked`.

Never spawn another robot while `robot_1` / `robot_1/turtlebot4` already exists. It can duplicate models inside shelves or other geometry.

## SIH ROS package changes

The repository clone lives at `~/amr_ws/src/SIH`; build from `~/amr_ws` with:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/amr_ws
colcon build --packages-select sih_amr_fleet --symlink-install
source install/setup.bash
```

Key added project files/features:

- `src/sih_amr_fleet/sih_amr_fleet/charging_pad_node.py`
  - subscribes to namespaced `odom`;
  - publishes `charging/is_docked`, `charging/battery_percent`, `charging/battery_state`, and `charging/docking_status`;
  - charges only after the AMR is in the configured pad zone, aligned to the rear stop, stationary, and stable for two seconds;
  - uses a project-owned battery estimate and deliberately does not overwrite TurtleBot's simulator-owned `/robot_N/battery_state`;
  - rejects stale odometry so charging stops when odometry ceases;
  - converts TurtleBot's local odometry frame to warehouse coordinates with `odom_origin_x`, `odom_origin_y`, and `odom_origin_yaw`.
- `src/sih_amr_fleet/launch/spawn_robot_1.launch.py` wraps the official TurtleBot 4 spawn launch.
- `README.md` contains the full build, aliases, launch order, and diagnostics runbook.

### Crucial odometry behavior

TurtleBot odometry starts at `(0, 0, 0)` at its spawn pose. Moving a robot by dragging it in the Gazebo GUI changes the visual/world pose but **does not update wheel odometry**. Therefore GUI placement cannot be used as a real docking test with the default detector.

For production-style tests: spawn the robot at a known world pose, launch the charging node with matching `odom_origin_*` parameters, and drive the robot to the pad with teleoperation or navigation.

The manual visual-placement test was confirmed with:

```bash
ros2 run sih_amr_fleet charging_pad_node \
  --ros-args -r __ns:=/robot_1 \
  -p initial_battery_percent:=50.0 \
  -p odom_origin_x:=0.513707 \
  -p odom_origin_y:=-10.059080 \
  -p odom_origin_yaw:=-1.5708
```

This caused `/robot_1/charging/is_docked` to publish `data: true`; it is only a calibration proof, not the preferred runtime workflow.

Read docking diagnostics with:

```bash
ros2 topic echo /robot_1/charging/docking_status
```

It prints world pose, pad-relative pose, heading error, speed, and which individual dock condition is false.

## Charging-pad world state

Four static custom charging pads now exist and are visually verified in the warehouse:

| Model name | World pose: `x y z roll pitch yaw` |
|---|---|
| `charging_pad_1` | `0.513707 -9.859080 0.02 0 0 1.5708` |
| `charging_pad_2` | `-0.813955 -9.910516 0.02 0 0 1.5708` |
| `charging_pad_3` | `-2.121722 -9.907508 0.02 0 0 1.5708` |
| `charging_pad_4` | `-3.372902 -9.910568 0.02 0 0 1.5708` |

The first pad is the original. The three copied pads were renamed from Gazebo's auto-generated names (`charging_pad_1_1`, etc.) to `charging_pad_2`, `charging_pad_3`, and `charging_pad_4`.

Each pad has a `0.9 x 0.65 x 0.04 m` base, model Z pose `0.02 m` (so its lower surface sits exactly on the floor), a yellow centre stripe, and a rear stop. The models are static; no inertial block is necessary.

## TurtleBot standard dock

`robot_1/standard_dock` is TurtleBot 4's vendor-supplied simulated dock, separate from the black/yellow project charging pads. It supports future TurtleBot-native docking/battery experiments. The current SIH charger uses the custom pads and the project charging node instead.

For now, keep each standard dock in a safe unused area; do not overlap it with a custom pad or leave it in an aisle. Do not delete it until deciding whether TurtleBot-native docking will be used in the final demo.

## Next work

1. Decide distinct safe spawn/parking poses for `robot_2` and `robot_3`; do not guess positions that may overlap shelves.
2. Add `spawn_turtlebot_2` and `spawn_turtlebot_3` aliases/launch arguments with unique namespaces and known spawn poses.
3. Launch one charging node per robot, passing that robot's corresponding pad pose and matching odometry-origin pose:
   - robot 1 → pad 1;
   - robot 2 → pad 2;
   - robot 3 → pad 3;
   - pad 4 remains available as a spare/shared bay.
4. Verify each AMR has `/robot_N/odom`, `/robot_N/cmd_vel`, and only one TurtleBot model in Gazebo before launching the full `fleet.launch.py`.

---

# Latest multi-AMR implementation handoff (September 2026)

The lightweight implementation from the previous Codex instance is now in the
repository:

- `src/sih_amr_fleet/launch/spawn_minimal_amr.launch.py`
- `src/sih_amr_fleet/sih_amr_fleet/twist_stamper_node.py`
- `tools/launch_four_amrs.sh`
- `tools/launch_four_lite.sh`
- `tools/fastdds_udp_only.xml`

The VM copy was rebuilt successfully with:

```bash
cd ~/amr_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select sih_amr_fleet --symlink-install
```

The short command is:

```bash
source ~/.bashrc
launch_four_lite
```

The VM pose file is `~/.config/sih_amr_poses.env` and currently contains:

```text
robot_1 = (2.0, 2.0, 0.0)
robot_2 = (-0.8140, -8.5, 1.5708)
robot_3 = (-0.8140, -7.5, 1.5708)
robot_4 = (-0.8140, -6.5, 1.5708)
```

## Latest verified failure

Run directory:

```text
~/amr_ws/log/four_amr_runs/20260905_182753
```

The minimal launcher successfully inserted the robot body:

```text
[robot_1.create_robot]: Entity creation successful.
```

The failure occurred afterward when the minimal launch tried to activate the
diff-drive controller:

```text
Could not contact service /controller_manager/list_controllers
```

The spawner was looking for the global service, not the expected namespaced
service `/robot_1/controller_manager/list_controllers`. No Gazebo server crash
was reported in this latest run, and no simulation processes remain after the
run.

This is now the primary issue. Do not treat robot insertion alone as a working
AMR: the controller manager, diff-drive controller, odometry, LiDAR, and command
adapter must all pass.

## Next debugging actions

1. Inspect the generated `robot_1/turtlebot4` SDF and the installed TurtleBot
   control configuration to determine how `gz_ros2_control` derives its ROS
   namespace.
2. Compare the minimal launch with the official launch, especially the
   `PushRosNamespace`, `robot_description`, `ros_gz_bridge`, and control-plugin
   parameters.
3. Verify with:

   ```bash
   ros2 node list | grep -E 'controller_manager|robot_1'
   ros2 service list | grep controller_manager
   ros2 control list_controllers -c /robot_1/controller_manager
   ros2 control list_controllers -c /controller_manager
   ```

4. Correct the minimal launch or model/plugin namespace so the controller
   manager is created under `/robot_N/controller_manager`.
5. Do not solve this by blindly changing the spawner to the global service:
   each AMR requires an isolated controller manager and isolated command path.
6. After robot 1 passes, test robot 2, then all four. Keep charging and fleet
   nodes disabled until the four interface gates pass.

## Renderer and DDS notes

The previous Ogre1 server run crashed in the sensor-rendering thread with an
Ogre duplicate scene-node exception for `Warehouse_CeilingLight_003`. The
launcher therefore defaults to `RENDER_ENGINE=ogre2`. Ogre2 kept Gazebo alive in
the later test.

The launcher also sets `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` and uses a project
Fast DDS UDP-only profile. Earlier runs had stale detached launches and shared
memory lock errors; the current launcher isolates process groups and refuses to
start over an existing simulation.

## Model recommendation for the next Codex instance

Use `gpt-5.6-sol` with `high` reasoning for the next implementation/debugging
turn. Luna High is suitable for routine, cost-sensitive work, but this task now
requires reading several ROS launch files, comparing installed vendor launch
behavior, inspecting live VM logs, and making a coordinated launch/control
fix. Official OpenAI model guidance describes GPT-5.6 Sol as the flagship model
for complex professional work and Luna as optimized for cost-sensitive,
high-volume workloads. If Sol is unavailable, use `gpt-5.6-luna` with `high` and
give it this entire handoff plus the latest run directory.
