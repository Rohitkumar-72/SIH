# SIH AMR Warehouse Simulation

This repository contains the ROS 2 Jazzy fleet overlay. The legacy warehouse is a
separate Gazebo resource at `~/amr_ws/src/warehouse_world_custom` and is launched
directly with `gz sim`.

## One-time Ubuntu setup

Clone this repository beside the warehouse source, build it, and source the overlay:

```bash
mkdir -p ~/amr_ws/src
cd ~/amr_ws/src
git clone https://github.com/adityaadep2008/SIH.git SIH

source /opt/ros/jazzy/setup.bash
cd ~/amr_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select sih_amr_interfaces sih_amr_fleet --symlink-install
source install/setup.bash
```

Every ROS terminal below must use ROS domain 42:

```bash
source /opt/ros/jazzy/setup.bash
source ~/amr_ws/install/setup.bash
export ROS_DOMAIN_ID=42
```

## Shell aliases

Add these aliases to `~/.bashrc`, then reload it with `source ~/.bashrc`.

```bash
alias warehouse='source /opt/ros/jazzy/setup.bash && GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/jazzy/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}" ROS_DOMAIN_ID=42 QT_QPA_PLATFORM=xcb GZ_SIM_RESOURCE_PATH="$HOME/amr_ws/src/sih_amr_fleet/models:$HOME/amr_ws/src/warehouse_world_custom/models:$HOME/amr_ws/src/warehouse_world_custom:/opt/ros/jazzy/share" gz sim -r --render-engine ogre "$HOME/amr_ws/src/warehouse_world_custom/worlds/small_warehouse/warehouse_clean.sdf"'
alias bridge_clock='source /opt/ros/jazzy/setup.bash && ROS_DOMAIN_ID=42 ros2 run ros_gz_bridge parameter_bridge "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"'
alias spawn_turtlebot='source /opt/ros/jazzy/setup.bash && source "$HOME/amr_ws/install/setup.bash" && ROS_DOMAIN_ID=42 ros2 launch sih_amr_fleet spawn_robot_1.launch.py namespace:=robot_1 model:=standard x:=2.0 y:=2.0 z:=0.05 yaw:=0.0'
alias start_charging='source /opt/ros/jazzy/setup.bash && source "$HOME/amr_ws/install/setup.bash" && ROS_DOMAIN_ID=42 ros2 run sih_amr_fleet charging_pad_node --ros-args -r __ns:=/robot_1 -p initial_battery_percent:=50.0'
```

Use the aliases as follows:

- `warehouse` — starts the clean warehouse and Gazebo GUI.
- `bridge_clock` — starts the Gazebo-to-ROS simulation clock bridge.
- `spawn_turtlebot` — creates one `robot_1`; run it only when no TurtleBot is already in Gazebo.
- `start_charging` — starts the docking and project-battery detector.

## Start one AMR safely

Use four terminals. Do not spawn a second robot if Gazebo's Entity Tree or
`gz model --list` already shows `robot_1` / `turtlebot4`.

### Terminal 1 — warehouse

```bash
warehouse
```

The errors below must **not** appear:

```text
Failed to load system plugin [libgz_ros2_control-system.so]
```

The warning below means the controller has no ROS `/clock` stream and is using its
time argument instead. It is not the missing-plugin failure, but it should be
resolved by starting the clock bridge below:

```text
No clock received, using time argument instead
```

### Terminal 2 — simulation clock bridge

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42

ros2 run ros_gz_bridge parameter_bridge '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
```

Leave this bridge running. It sends Gazebo simulation time to ROS 2 and should
stop the controller-manager clock warning.

### Terminal 3 — spawn only if there is no robot in the world

```bash
source /opt/ros/jazzy/setup.bash
source ~/amr_ws/install/setup.bash
export ROS_DOMAIN_ID=42

ros2 launch sih_amr_fleet spawn_robot_1.launch.py \
  namespace:=robot_1 model:=standard \
  x:=2.0 y:=2.0 z:=0.05 yaw:=0.0
```

Do not save the world after spawning. The preferred workflow is a clean warehouse
SDF with the charging pad but without a robot, followed by one runtime spawn.

### Terminal 4 — charging detector

```bash
source /opt/ros/jazzy/setup.bash
source ~/amr_ws/install/setup.bash
export ROS_DOMAIN_ID=42

ros2 run sih_amr_fleet charging_pad_node \
  --ros-args -r __ns:=/robot_1 \
  -p initial_battery_percent:=50.0
```

Leave this command running. It normally produces no terminal text.

### Terminal 5 — verify interfaces

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42

ros2 topic list | grep '^/robot_1/'
ros2 topic info /robot_1/odom -v
ros2 topic echo /robot_1/odom --once
```

Use `/robot_1/odom`, not `/odom`. The robot is namespaced, so the global `/odom`
topic does not exist.

When odometry works, observe the dock detector and battery:

```bash
ros2 topic echo /robot_1/charging/is_docked
ros2 topic echo /robot_1/charging/battery_percent
```

`is_docked` is initially `false`. It becomes `true` only when the robot is in the
pad zone, aligned with the rear stop, and stationary for two seconds. The project
battery estimate starts at 50 percent and rises only while docked.

TurtleBot odometry is local to its spawn point. Do not drag the robot onto the pad
in the Gazebo GUI when testing docking: GUI repositioning does not update wheel
odometry. Spawn at a known pose and drive it using `cmd_vel` or teleoperation. The
charging node converts odometry into warehouse coordinates using
`odom_origin_x`, `odom_origin_y`, and `odom_origin_yaw`; these must match the
spawn `x`, `y`, and `yaw` arguments.

For a readable explanation of a false docking result:

```bash
ros2 topic echo /robot_1/charging/docking_status
```

## If `/robot_1/odom` has zero publishers

Check the controller and Gazebo transport topics:

```bash
ros2 node list | grep -E 'robot_1|controller_manager|parameter_bridge|robot_state_publisher'
ros2 control list_controllers -c /robot_1/controller_manager
gz topic -l | grep -Ei 'odom|joint|cmd_vel'
```

If Gazebo logs the missing `libgz_ros2_control-system.so` error, verify that the
plugin exists and that `GZ_SIM_SYSTEM_PLUGIN_PATH` contains its directory:

```bash
find /opt/ros/jazzy -name 'libgz_ros2_control-system.so' -print
echo "$GZ_SIM_SYSTEM_PLUGIN_PATH"
```

Do not run another spawn command until the existing robot has been removed or a
clean warehouse world has been launched; duplicate TurtleBots can overlap shelves
and make the simulation look corrupted.
