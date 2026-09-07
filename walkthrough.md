# Walkthrough: Decentralized Multi-AMR ROS 2 Coordination Core

We have implemented, refined, and integrated the complete decentralized ROS 2 coordination core across planning, corridor mutual exclusion, task allocation, safety supervisor, task execution lifecycle, and warehouse map infrastructure for multi-AMR fleet operation.

## Key Changes Made

### 1. Interfaces (`sih_amr_interfaces`)
- **[TaskExecutionStatus.msg](file:///c:/Users/pravin%20sharma/OneDrive/Documents/GitHub/SIH/src/sih_amr_interfaces/msg/TaskExecutionStatus.msg)**: Created message definition for tracking task delivery lifecycle stages (`EN_ROUTE_PICKUP`, `PICKUP_WAIT`, `EN_ROUTE_DROPOFF`, `DROPOFF_WAIT`, `COMPLETED`, `FAILED`) with target poses and remaining dwell times.

### 2. Missing Core Nodes (`sih_amr_fleet`)
- **[warehouse_map_node.py](file:///c:/Users/pravin%20sharma/OneDrive/Documents/GitHub/SIH/src/sih_amr_fleet/sih_amr_fleet/warehouse_map_node.py)**: Loads warehouse layout YAML, expands shelf footprints, and publishes latched `/map` (`nav_msgs/OccupancyGrid`) and `/map_metadata`.
- **[task_execution_node.py](file:///c:/Users/pravin%20sharma/OneDrive/Documents/GitHub/SIH/src/sih_amr_fleet/sih_amr_fleet/task_execution_node.py)**: Manages delivery lifecycle per robot: detects pickup arrival, enforces 2–5s dwell, switches target to dropoff, enforces dropoff dwell, and signals completion to CBBA.
- **[random_task_generator_node.py](file:///c:/Users/pravin%20sharma/OneDrive/Documents/GitHub/SIH/src/sih_amr_fleet/sih_amr_fleet/random_task_generator_node.py)**: Generates safe delivery tasks with coordinates strictly in open aisle centerlines.
- **[data_collection_node.py](file:///c:/Users/pravin%20sharma/OneDrive/Documents/GitHub/SIH/src/sih_amr_fleet/sih_amr_fleet/data_collection_node.py)**: Passive JSONL telemetry recorder logging run manifests, state, consensus, reservations, corridor events, and safety interventions without affecting control.

### 3. Algorithm & Coordination Node Refinements
- **[whca_planner_node.py](file:///c:/Users/pravin%20sharma/OneDrive/Documents/GitHub/SIH/src/sih_amr_fleet/sih_amr_fleet/whca_planner_node.py)**: Integrated dynamic target switching between pickup and dropoff, stationary dwell reservation handling, and rolling space-time reservation fusion.
- **[corridor_mutex_node.py](file:///c:/Users/pravin%20sharma/OneDrive/Documents/GitHub/SIH/src/sih_amr_fleet/sih_amr_fleet/corridor_mutex_node.py)**: Fixed map origin offsets, added 2D corridor cell expansion for bounding endpoint definitions, and enforced Lamport clock synchronization.
- **[cbba_node.py](file:///c:/Users/pravin%20sharma/OneDrive/Documents/GitHub/SIH/src/sih_amr_fleet/sih_amr_fleet/cbba_node.py)**: Added workload queue penalties to balance task bundles across all 4 AMRs, with consensus epochs and lease expiry protection.

### 4. Verification Suite
- **[test/test_algorithms.py](file:///c:/Users/pravin%20sharma/OneDrive/Documents/GitHub/SIH/src/sih_amr_fleet/test/test_algorithms.py)**: 10 comprehensive unit tests for WHCA* space-time search, reservation avoidance, edge swap prevention, static obstacles, Kalman filter uncertainty growth/shrinkage, and ORCA uncertainty inflation.

---

## Verification Results

### Automated Unit Tests
```text
============================= test session starts =============================
platform win32 -- Python 3.14.0, pytest-9.0.2, pluggy-1.6.0
rootdir: C:\Users\pravin sharma\OneDrive\Documents\GitHub\SIH\src\sih_amr_fleet
plugins: anyio-4.12.0
collected 10 items

src\sih_amr_fleet\test\test_algorithms.py ..........                     [100%]

============================= 10 passed in 0.06s ==============================
```

### Python Syntax Validation
```text
All 21 node modules and launch files compiled with zero syntax errors.
```

---

## System Architecture Diagram

```mermaid
flowchart TD
    subgraph SensingAndState["State & Perception (per AMR)"]
        odom["/robot_N/odom"] --> loc["localization_node"]
        scan["/robot_N/scan"] --> costmap["local_costmap_node"]
        loc --> state["/robot_N/state & /fleet/robot_state"]
        costmap --> block["blockage_detector_node"]
        block --> block_topic["/fleet/blockage_observation"]
        state --> peer["peer_tracker_node (Kalman Filter)"]
    end

    subgraph TaskAndPlan["Task Allocation & Rolling Planning"]
        ann["/fleet/task_announcement"] --> cbba["cbba_node"]
        cbba <--> cons["/fleet/task_consensus"]
        cbba --> assign["/robot_N/task_assignment"]
        assign --> exec_node["task_execution_node"]
        exec_node --> exec_status["/fleet/task_execution_status"]
        exec_status --> cbba
        exec_status --> whca["whca_planner_node"]
        state --> whca
        block_topic --> whca
        intent_topic["/fleet/trajectory_intent"] --> whca
        whca --> route["/robot_N/planned_route"]
        route --> res_mgr["reservation_manager_node"]
        res_mgr --> intent_topic
    end

    subgraph CoordAndSafety["Coordination & Safety (Authoritative)"]
        route --> cmutex["corridor_mutex_node (Ricart-Agrawala)"]
        cmutex <--> cproto["/fleet/corridor_protocol"]
        cmutex --> allowed["/robot_N/corridor_motion_allowed"]
        route --> follower["path_follower_node"]
        allowed --> follower
        follower --> desired["cmd_vel_desired"]
        peer --> orca["orca_node"]
        desired --> orca
        orca --> candidate["cmd_vel_candidate"]
        scan --> safety["safety_supervisor_node (Sensor Hard Veto)"]
        candidate --> safety
        safety --> motors["/robot_N/cmd_vel"]
    end

    subgraph GlobalInfra["Global Infrastructure"]
        map_node["warehouse_map_node"] --> global_map["/map & /map_metadata"]
        data_logger["data_collection_node (Passive JSONL)"]
    end
```
