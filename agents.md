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
