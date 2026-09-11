#!/usr/bin/env python3
"""Multi-Work-Cycle Automated Test Runner for SIH Decentralized AMR Fleet.

Executes sequential work-cycle simulation runs (default: 5 runs, 4 tasks per cycle),
displays real-time terminal progress for each task lifecycle stage (CBBA consensus,
pickup arrival, dwell, dropoff arrival, completion, RTF), optimizes GPU/CPU hardware
utilization, and aggregates logs and benchmark metrics across all runs.
"""

import argparse
import datetime
import json
import os
import queue
import re
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional, Set

# Add package directory to sys.path
SIH_SRC_DIR = Path(__file__).resolve().parent.parent / "src" / "sih_amr_fleet"
if str(SIH_SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SIH_SRC_DIR))

try:
    from sih_amr_fleet.runner_metrics import (
        RollingRTFEstimator, TaskETAEstimator, CampaignETAEstimator,
        format_duration, format_local_finish_time
    )
    from sih_amr_fleet.velocity_profiles import (
        PROFILES as VELOCITY_PROFILES,
        get_profile, resolve_velocity_profile, compute_throughput_break_even
    )
except ImportError:
    from sih_amr_fleet.sih_amr_fleet.runner_metrics import (
        RollingRTFEstimator, TaskETAEstimator, CampaignETAEstimator,
        format_duration, format_local_finish_time
    )
    from sih_amr_fleet.sih_amr_fleet.velocity_profiles import (
        PROFILES as VELOCITY_PROFILES,
        get_profile, resolve_velocity_profile, compute_throughput_break_even
    )

# Terminal Color Styling
C_RESET = "\033[0m"
C_BOLD = "\033[1m"
C_DIM = "\033[2m"
C_RED = "\033[31m"
C_GREEN = "\033[32m"
C_YELLOW = "\033[33m"
C_BLUE = "\033[34m"
C_MAGENTA = "\033[35m"
C_CYAN = "\033[36m"
C_WHITE = "\033[37m"
C_BG_BLUE = "\033[44m"
C_BG_DARK = "\033[100m"

# Regex patterns for live log stream matching
RE_CBBA_BID = re.compile(
    r"\[(?P<robot>robot_\d+):CBBA\]\s*Decision:\s*SUBMIT_BID for task (?P<task>[a-zA-Z0-9_-]+)"
)
RE_CBBA_COMMIT = re.compile(
    r"\[(?P<robot>robot_\d+):CBBA\]\s*Decision:\s*UNANIMOUS_COMMIT for task (?P<task>[a-zA-Z0-9_-]+) -> Winner=(?P<winner>robot_\d+),\s*Bid=(?P<bid>[\d.]+),\s*epoch=(?P<epoch>\d+)\..*Quorum=(?P<quorum>\d+/\d+)"
)
RE_TASK_ACCEPT = re.compile(
    r"\[(?P<robot>robot_\d+):TaskExecutor\]\s*Decision:\s*ACCEPT new task (?P<task>[a-zA-Z0-9_-]+)\..*pickup=\((?P<px>[-\d.]+),\s*(?P<py>[-\d.]+)\),\s*dropoff=\((?P<dx>[-\d.]+),\s*(?P<dy>[-\d.]+)\)"
)
RE_PICKUP_ARRIVED = re.compile(
    r"\[(?P<robot>robot_\d+):TaskExecutor\]\s*Decision:\s*ARRIVED at pickup for task (?P<task>[a-zA-Z0-9_-]+)\..*dwelling (?P<dwell>[-\d.]+)s"
)
RE_PICKUP_DONE = re.compile(
    r"\[(?P<robot>robot_\d+):TaskExecutor\]\s*Decision:\s*PICKUP_DWELL_COMPLETE for task (?P<task>[a-zA-Z0-9_-]+)"
)
RE_DROPOFF_ARRIVED = re.compile(
    r"\[(?P<robot>robot_\d+):TaskExecutor\]\s*Decision:\s*ARRIVED at dropoff for task (?P<task>[a-zA-Z0-9_-]+)\..*dwelling (?P<dwell>[-\d.]+)s"
)
RE_TASK_COMPLETED = re.compile(
    r"\[(?P<robot>robot_\d+):TaskExecutor\]\s*Decision:\s*(?:DROPOFF_DWELL_COMPLETE for task (?P<task>[a-zA-Z0-9_-]+)\..*Transition to COMPLETED|RESET executor after publishing completion for task (?P<task2>[a-zA-Z0-9_-]+))"
)
RE_ROBOT_READY = re.compile(
    r"(?:\[(?P<robot>robot_\d+)\.interface_readiness\]:\s*Interface readiness passed|(?P<robot2>robot_\d+) passed all gates)"
)
RE_SPAWN_START = re.compile(
    r"Starting (?P<robot>robot_\d+) at x=(?P<x>[-\d.]+) y=(?P<y>[-\d.]+) yaw=(?P<yaw>[-\d.]+)"
)
RE_GZ_SERVER = re.compile(
    r"Starting (?:unthrottled )?Gazebo server with (?P<engine>\w+)|Starting server with (?P<engine2>\w+)"
)
RE_FLEET_DONE = re.compile(
    r"All (?P<count>\d+|four) AMRs passed"
)


class TaskState:
    """Tracks the live lifecycle state of a single warehouse task."""
    def __init__(self, task_id: str):
        self.task_id = task_id
        self.assigned_robot: Optional[str] = None
        self.bid: Optional[float] = None
        self.quorum: Optional[str] = None
        self.pickup_coord: Optional[str] = None
        self.dropoff_coord: Optional[str] = None
        self.stage: str = "ANNOUNCED"
        self.start_wall_time: float = time.time()
        self.pickup_time: Optional[float] = None
        self.dropoff_time: Optional[float] = None
        self.completion_time: Optional[float] = None
        self.dwell_time_s: float = 0.0

    @property
    def duration_s(self) -> float:
        if self.completion_time:
            return self.completion_time - self.start_wall_time
        return time.time() - self.start_wall_time


class WorkCycleRun:
    """Manages execution and telemetry monitoring for one work cycle test."""
    def __init__(self, run_index: int, total_runs: int, target_tasks: int, base_dir: Path, timeout_s: int,
                 campaign_estimator: Optional[CampaignETAEstimator] = None, fleet_count: int = 4,
                 launcher_override: Optional[str] = None, scenario: str = "baseline"):
        self.run_index = run_index
        self.total_runs = total_runs
        self.target_tasks = target_tasks
        self.base_dir = base_dir
        self.timeout_s = timeout_s
        self.campaign_estimator = campaign_estimator
        self.fleet_count = fleet_count
        self.launcher_override = launcher_override
        self.scenario = scenario
        self.tracking_speed: float = 0.31
        self.enable_faults: bool = False
        self.enable_spawner: bool = False
        self.enable_vision: bool = False
        self.run_id = f"run_{run_index:02d}_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}"
        self.log_dir = base_dir / self.run_id
        self.log_dir.mkdir(parents=True, exist_ok=True)
        
        self.fleet_log_file = self.log_dir / "fleet.log"
        self.telemetry_file = self.log_dir / "fleet_telemetry.jsonl"
        self.events_file = self.log_dir / "run_events.jsonl"
        
        self.tasks: Dict[str, TaskState] = {}
        self.completed_tasks: List[str] = []
        self.ready_robots: Set[str] = set()
        self.robot_status: Dict[str, str] = {f"robot_{i}": "IDLE" for i in range(1, self.fleet_count + 1)}
        self.start_time: float = 0.0
        self.end_time: float = 0.0
        self.sim_time_s: float = 0.0
        self.process: Optional[subprocess.Popen] = None
        self.status: str = "PENDING"
        self.rolling_rtf_estimator = RollingRTFEstimator(window_seconds=60.0, staleness_threshold_s=5.0)
        self.eta_estimator = TaskETAEstimator(minimum_completed_tasks=3, recent_completion_limit=10, ewma_alpha=0.25)

    @property
    def wall_duration_s(self) -> float:
        if self.end_time > 0 and self.start_time > 0:
            return self.end_time - self.start_time
        elif self.start_time > 0:
            return time.time() - self.start_time
        return 0.0

    @property
    def avg_task_duration_s(self) -> float:
        completed = [t.duration_s for t in self.tasks.values() if t.stage == "COMPLETED"]
        return (sum(completed) / len(completed)) if completed else 0.0

    @property
    def time_per_task_s(self) -> float:
        count = len(self.completed_tasks)
        dur = self.wall_duration_s
        return (dur / count) if count > 0 else 0.0

    def clean_lingering_processes(self, graceful: bool = True):
        """Kills any stale Gazebo, ros2 nodes, or bridges before/after run.
        
        If graceful=True, first sends SIGTERM (pkill -15) so Cyclone DDS participants
        can broadcast disposal messages and unbind ports cleanly before SIGKILL.
        """
        patterns = [
            "gz sim", "gz-sim", "gz_sim", "ign gazebo", "ros_gz_bridge",
            "parameter_bridge", "static_transform_publisher", "data_collection_node",
            "task_execution_node", "cbba_node", "whca_planner_node",
            "path_follower_node", "safety_supervisor_node", "corridor_mutex_node",
            "orca_node", "random_task_generator_node", "warehouse_map_node",
            "spawn_minimal_amr", "turtlebot4_spawn", "twist_stamper",
            "interface_readiness", "localization_node", "robot_state_publisher",
            "controller_manager", "diffdrive_controller"
        ]
        if graceful:
            for pat in patterns:
                try:
                    subprocess.run(["pkill", "-15", "-f", pat], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                except Exception:
                    pass
            time.sleep(1.5)
        for pat in patterns:
            try:
                subprocess.run(["pkill", "-9", "-f", pat], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception:
                pass
        time.sleep(0.5)

    def print_stage_event(self, symbol: str, color: str, tag: str, message: str):
        elapsed = time.time() - self.start_time if self.start_time > 0 else 0.0
        timestamp_str = f"[{elapsed:6.1f}s]"
        if sys.stdout.isatty():
            sys.stdout.write("\r\033[K")
        else:
            sys.stdout.write("\r")
        print(f" {C_DIM}{timestamp_str}{C_RESET} {color}{C_BOLD}{symbol} [{tag:<14}]{C_RESET} {message}")
        sys.stdout.flush()

    def format_robots_status(self) -> str:
        """Formats colored live status string for all fleet AMRs (e.g. R1:IDLE R2:TO_PICKUP)."""
        parts = []
        def r_key(k: str) -> int:
            m = re.search(r"\d+", k)
            return int(m.group(0)) if m else 999

        for rid in sorted(self.robot_status.keys(), key=r_key):
            st = self.robot_status[rid]
            r_label = f"R{r_key(rid)}"
            if st.startswith("TO_PICKUP"):
                c = C_BLUE
            elif st.startswith("DWELL"):
                c = C_YELLOW
            elif st.startswith("TO_DROPOFF"):
                c = C_CYAN
            elif st == "BIDDING":
                c = C_MAGENTA
            elif st == "SPAWNING":
                c = C_YELLOW
            else:
                c = C_GREEN
            parts.append(f"{r_label}:{c}{st}{C_RESET}")
        return " ".join(parts)

    def parse_launcher_line(self, line: str):
        """Parses launcher stdout stream for AMR spawning, Gazebo bringing-up, and gates."""
        line = line.strip()
        if not line:
            return

        m = RE_GZ_SERVER.search(line)
        if m:
            engine = m.group("engine") or m.group("engine2") or "ogre2"
            self.print_stage_event("⚙", C_YELLOW, "GAZEBO SERVER", f"Starting physics server ({engine})...")
            return

        m = RE_SPAWN_START.search(line)
        if m:
            robot = m.group("robot")
            x, y, yaw = m.group("x"), m.group("y"), m.group("yaw")
            self.robot_status[robot] = "SPAWNING"
            self.print_stage_event("⚙", C_BLUE, "SPAWNING AMR", f"Spawning {robot} at ({x}, {y}, yaw={yaw})...")
            return

        m = RE_ROBOT_READY.search(line)
        if m:
            robot = m.group("robot") or m.group("robot2")
            if robot and robot not in self.ready_robots:
                self.ready_robots.add(robot)
                self.robot_status[robot] = "IDLE"
                self.print_stage_event("✔", C_CYAN, "GATE PASSED", f"{robot} interfaces ready ({len(self.ready_robots)}/{self.fleet_count})")
            return

        m = RE_FLEET_DONE.search(line)
        if m:
            self.print_stage_event("🚀", C_GREEN, "FLEET READY", f"All {self.fleet_count} AMRs online! Starting task generation & CBBA auction...")
            return

        if "[GAZEBO POSE OK]" in line:
            self.print_stage_event("⚓", C_GREEN, "POSE VERIFIED", line.split("[GAZEBO POSE OK]")[-1].strip())
            return

        if "[GAZEBO POSE MISMATCH]" in line:
            self.print_stage_event("✖", C_RED, "POSE MISMATCH", line.split("[GAZEBO POSE MISMATCH]")[-1].strip())
            return

        if "[GAZEBO POSE TIMEOUT]" in line:
            self.print_stage_event("✖", C_RED, "POSE TIMEOUT", line.split("[GAZEBO POSE TIMEOUT]")[-1].strip())
            return

        if "Verifying" in line and "Gazebo" in line:
            self.print_stage_event("🔍", C_CYAN, "VERIFYING POSE", line.strip())
            return

        if "ERROR:" in line or "fail" in line.lower():
            self.print_stage_event("✖", C_RED, "ERROR", line)

    def parse_log_line(self, line: str):
        # 1. Robot readiness check
        m = RE_ROBOT_READY.search(line)
        if m:
            robot = m.group("robot") or m.group("robot2")
            if robot and robot not in self.ready_robots:
                self.ready_robots.add(robot)
                self.robot_status[robot] = "IDLE"
                self.print_stage_event("✔", C_CYAN, "ROBOT READY", f"{robot} interfaces passed readiness gate ({len(self.ready_robots)}/{self.fleet_count})")

        # 1b. CBBA Bidding
        m = RE_CBBA_BID.search(line)
        if m:
            robot = m.group("robot")
            if self.robot_status.get(robot, "IDLE") in ("IDLE", "WAIT_BID", "BIDDING"):
                self.robot_status[robot] = "BIDDING"

        # 2. CBBA Consensus / Allocation
        m = RE_CBBA_COMMIT.search(line)
        if m:
            task_id = m.group("task")
            winner = m.group("winner")
            quorum = m.group("quorum")
            bid = float(m.group("bid"))
            if task_id not in self.tasks:
                self.tasks[task_id] = TaskState(task_id)
            t = self.tasks[task_id]
            if t.stage == "ANNOUNCED":
                t.assigned_robot = winner
                t.bid = bid
                t.quorum = quorum
                t.stage = "CBBA_COMMITTED"
                self.print_stage_event("★", C_MAGENTA, "CBBA DONE", f"Task {C_BOLD}{task_id}{C_RESET} assigned to {C_BOLD}{winner}{C_RESET} (Bid: {bid:.2f}, Quorum: {quorum})")

        # 3. Task Accepted by Executor
        m = RE_TASK_ACCEPT.search(line)
        if m:
            task_id = m.group("task")
            robot_id = m.group("robot")
            px, py = m.group("px"), m.group("py")
            dx, dy = m.group("dx"), m.group("dy")
            self.robot_status[robot_id] = f"TO_PICKUP({task_id})"
            for r in self.robot_status:
                if self.robot_status[r] == "BIDDING":
                    self.robot_status[r] = "IDLE"
            t = self.tasks.setdefault(task_id, TaskState(task_id))
            if t.stage in ("ANNOUNCED", "CBBA_COMMITTED"):
                t.assigned_robot = robot_id
                t.pickup_coord = f"({px}, {py})"
                t.dropoff_coord = f"({dx}, {dy})"
                t.stage = "ACCEPTED"
                self.print_stage_event("▶", C_BLUE, "TASK ACCEPT", f"{robot_id} navigating to Pickup {t.pickup_coord} for {task_id}")

        # 4. Pickup Arrived & Dwell
        m = RE_PICKUP_ARRIVED.search(line)
        if m:
            task_id = m.group("task")
            robot_id = m.group("robot")
            dwell = m.group("dwell")
            self.robot_status[robot_id] = f"DWELL_PICKUP({task_id})"
            t = self.tasks.setdefault(task_id, TaskState(task_id))
            if t.stage == "ACCEPTED":
                t.stage = "AT_PICKUP"
                t.pickup_time = time.time()
                self.print_stage_event("⚓", C_YELLOW, "PICKUP ARRIVED", f"{robot_id} reached pickup station for {task_id} (dwelling {dwell}s)")

        m = RE_PICKUP_DONE.search(line)
        if m:
            task_id = m.group("task")
            robot_id = m.group("robot")
            self.robot_status[robot_id] = f"TO_DROPOFF({task_id})"
            t = self.tasks.setdefault(task_id, TaskState(task_id))
            if t.stage in ("ACCEPTED", "AT_PICKUP"):
                t.stage = "EN_ROUTE_DROPOFF"
                self.print_stage_event("➜", C_CYAN, "PICKUP DONE", f"{robot_id} finished loading {task_id}; heading to Dropoff {t.dropoff_coord or ''}")

        # 5. Dropoff Arrived & Dwell
        m = RE_DROPOFF_ARRIVED.search(line)
        if m:
            task_id = m.group("task")
            robot_id = m.group("robot")
            dwell = m.group("dwell")
            self.robot_status[robot_id] = f"DWELL_DROPOFF({task_id})"
            t = self.tasks.setdefault(task_id, TaskState(task_id))
            if t.stage in ("ACCEPTED", "AT_PICKUP", "EN_ROUTE_DROPOFF"):
                t.stage = "AT_DROPOFF"
                t.dropoff_time = time.time()
                self.print_stage_event("⚓", C_YELLOW, "DROPOFF ARRIVED", f"{robot_id} reached dropoff station for {task_id} (dwelling {dwell}s)")

        # 6. Task Completed
        m = RE_TASK_COMPLETED.search(line)
        if m:
            task_id = m.group("task") or m.group("task2")
            robot_id = m.group("robot")
            if robot_id:
                self.robot_status[robot_id] = "IDLE"
            t = self.tasks.setdefault(task_id, TaskState(task_id))
            if task_id not in self.completed_tasks:
                t.stage = "COMPLETED"
                t.completion_time = time.time()
                self.completed_tasks.append(task_id)
                self.eta_estimator.record_completion(time.monotonic())
                count = len(self.completed_tasks)
                pct = int((count / self.target_tasks) * 100)
                bar = "█" * (pct // 2) + "░" * (50 - (pct // 2))
                self.print_stage_event(
                    "✔", C_GREEN, "TASK COMPLETED",
                    f"{C_BOLD}{task_id}{C_RESET} finished by {robot_id or t.assigned_robot} | Duration: {t.duration_s:.1f}s | Progress: [{bar}] {count}/{self.target_tasks} ({pct}%)"
                )

    def compute_rtf(self) -> float:
        """Computes achieved Real-Time Factor from telemetry JSONL if present."""
        if not self.telemetry_file.exists():
            return 0.0
        try:
            first_sim, first_wall = None, None
            last_sim, last_wall = None, None
            with open(self.telemetry_file, "r") as f:
                for line in f:
                    if not line.strip():
                        continue
                    try:
                        record = json.loads(line)
                        st = float(record.get("logged_at", record.get("sim_time_s", 0.0)))
                        wt = float(record.get("wall_logged_at", record.get("wall_time_s", 0.0)))
                        if st <= 0.0 or wt <= 0.0:
                            continue
                        if first_sim is None:
                            first_sim, first_wall = st, wt
                        last_sim, last_wall = st, wt
                    except Exception:
                        continue
            if first_sim is not None and last_sim is not None:
                d_sim = last_sim - first_sim
                d_wall = last_wall - first_wall
                if d_wall > 0:
                    self.sim_time_s = d_sim
                    return d_sim / d_wall
        except Exception:
            pass
        return 0.0

    def execute(self, tracking_speed: float, settle_s: int, seed: int = 42,
                enable_faults: bool = False, enable_spawner: bool = False,
                enable_vision: bool = False, scenario: Optional[str] = None) -> bool:
        """Executes a single work-cycle test run."""
        self.clean_lingering_processes()
        self.start_time = time.time()
        self.rolling_rtf_estimator.reset()
        self.eta_estimator.reset()
        self.tracking_speed = tracking_speed
        self.enable_faults = enable_faults
        self.enable_spawner = enable_spawner
        self.enable_vision = enable_vision
        if scenario:
            self.scenario = scenario
        
        # Setup environment variables optimized for GPU and high RTF
        env = os.environ.copy()
        env["START_GUI"] = "false"
        env["START_FLEET"] = "true"
        env["FLEET_RANDOM_TASKS"] = "true"
        env["FLEET_RECORD_DATA"] = "true"
        env["RENDER_ENGINE"] = "ogre2"
        env["GUI_RENDER_ENGINE"] = "ogre2"
        env["SENSOR_PROFILE"] = "fleet"  # Lean 2D lidar profile
        env["LIDAR_UPDATE_RATE_HZ"] = os.environ.get("LIDAR_UPDATE_RATE_HZ", "10.0")
        env["FLEET_TRACKING_SPEED_MPS"] = str(tracking_speed)
        env["SETTLE_SECONDS"] = str(settle_s)
        env["LOG_DIR"] = str(self.log_dir)
        env["FLEET_DATA_FILE"] = str(self.telemetry_file)
        env["FLEET_RANDOM_SEED"] = str(seed + self.run_index)
        env["FLEET_ENABLE_FAULTS"] = "true" if enable_faults else "false"
        env["FLEET_ENABLE_SPAWNER"] = "true" if enable_spawner else "false"
        env["FLEET_ENABLE_VISION"] = "true" if enable_vision else "false"
        env["FLEET_COUNT"] = str(self.fleet_count)
        
        # GPU Acceleration environment for NVIDIA GeForce RTX 3070 / Linux
        env["__NV_PRIME_RENDER_OFFLOAD"] = "1"
        env["__GLX_VENDOR_LIBRARY_NAME"] = "nvidia"
        env["CUDA_VISIBLE_DEVICES"] = "0"
        
        # Isolate DDS domain ID per run to eliminate participant/endpoint socket collisions
        run_domain_id = (42 + self.run_index) % 100
        env["ROS_DOMAIN_ID"] = str(run_domain_id)

        # Fallback pose file if not configured in home
        script_dir = Path(__file__).resolve().parent
        pose_example = script_dir / "sih_amr_poses.env.example"
        if not Path(os.path.expanduser("~/.config/sih_amr_poses.env")).exists() and pose_example.exists():
            env["SIH_AMR_POSES_FILE"] = str(pose_example)

        if self.launcher_override:
            launch_script = script_dir / self.launcher_override
        elif self.fleet_count > 4 and (script_dir / "launch_fleet_amrs.sh").exists():
            launch_script = script_dir / "launch_fleet_amrs.sh"
        else:
            launch_script = script_dir / "launch_four_amrs.sh"

        if not launch_script.exists():
            print(f"{C_RED}ERROR: launch script not found at {launch_script}{C_RESET}")
            return False

        print(f"\n{C_BG_BLUE}{C_WHITE}{C_BOLD} >>> STARTING TEST RUN {self.run_index}/{self.total_runs}: {self.run_id} <<<{C_RESET}")
        print(f" {C_CYAN}Target Work Cycle:{C_RESET} {self.target_tasks} completed tasks across {self.fleet_count} AMRs")
        print(f" {C_CYAN}Log Directory:{C_RESET}     {self.log_dir}")
        print(f" {C_CYAN}DDS Domain ID:{C_RESET}     {run_domain_id}")
        print(f" {C_CYAN}GPU Acceleration:{C_RESET}  NVIDIA RTX 3070 (ogre2 headless rendering, fleet sensor profile)")
        print(f" {C_CYAN}Max Tracking Speed:{C_RESET}{tracking_speed} m/s | Settle Time: {settle_s}s\n")
        sys.stdout.flush()

        # Start the Gazebo & fleet launcher process in its own process group
        self.process = subprocess.Popen(
            ["bash", str(launch_script)],
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
            text=True,
            bufsize=1
        )

        output_queue = queue.Queue()

        def stream_reader():
            try:
                for line in iter(self.process.stdout.readline, ''):
                    output_queue.put(line)
                self.process.stdout.close()
            except Exception:
                pass

        reader_thread = threading.Thread(target=stream_reader, daemon=True)
        reader_thread.start()

        # Monitor fleet.log and stdout simultaneously
        fleet_log_fd = None
        last_ticker_s = -1
        first_sim_s = None
        
        try:
            while True:
                # Read stdout from launcher script for spawning stages and gates
                while not output_queue.empty():
                    try:
                        lline = output_queue.get_nowait()
                        self.parse_launcher_line(lline)
                    except queue.Empty:
                        break

                now = time.time()
                wall_elapsed = now - self.start_time
                if wall_elapsed > self.timeout_s:
                    if sys.stdout.isatty():
                        sys.stdout.write("\r\033[K")
                    self.print_stage_event("✗", C_RED, "TIMEOUT", f"Run exceeded timeout of {self.timeout_s}s. Terminating.")
                    self.status = "TIMEOUT"
                    break

                # Check if launcher crashed prematurely
                if self.process.poll() is not None:
                    if sys.stdout.isatty():
                        sys.stdout.write("\r\033[K")
                    if len(self.completed_tasks) < self.target_tasks:
                        self.print_stage_event("✗", C_RED, "CRASH", f"Launch script exited prematurely with code {self.process.returncode}")
                        self.status = "FAILED"
                        break

                # Read newly appended lines in fleet.log
                if self.fleet_log_file.exists():
                    if fleet_log_fd is None:
                        fleet_log_fd = open(self.fleet_log_file, "r")
                    
                    lines = fleet_log_fd.readlines()
                    for line in lines:
                        self.parse_log_line(line)

                # Check completion condition: all target tasks completed!
                if len(self.completed_tasks) >= self.target_tasks:
                    if sys.stdout.isatty():
                        sys.stdout.write("\r\033[K")
                    self.end_time = time.time()
                    total_dur = self.end_time - self.start_time
                    rtf = self.compute_rtf()
                    rtf_str = f"{rtf:.2f}x" if rtf > 0 else "N/A"
                    self.print_stage_event(
                        "★", C_GREEN, "CYCLE COMPLETE",
                        f"All {self.target_tasks} work-cycle tasks successfully completed in {total_dur:.1f}s (RTF: {rtf_str})!"
                    )
                    self.status = "PASSED"
                    break

                # Live clock counter ticker
                sec_int = int(wall_elapsed)
                if sec_int != last_ticker_s:
                    last_ticker_s = sec_int
                    mins, secs = divmod(sec_int, 60)
                    tot_m, tot_s = divmod(self.timeout_s, 60)
                    
                    # Quick read of live sim time from telemetry file if available
                    if self.telemetry_file.exists():
                        try:
                            fsize = self.telemetry_file.stat().st_size
                            if fsize > 0:
                                with open(self.telemetry_file, "rb") as tf:
                                    tf.seek(max(0, fsize - 2048))
                                    chunk = tf.read().decode("utf-8", errors="ignore").strip().split("\n")
                                    for entry in reversed(chunk):
                                        if entry.strip():
                                            rec = json.loads(entry)
                                            st = float(rec.get("logged_at", rec.get("sim_time_s", 0.0)))
                                            if st > 0.0:
                                                self.rolling_rtf_estimator.add_sample(time.monotonic(), st)
                                                if first_sim_s is None:
                                                    first_sim_s = st
                                                break
                        except Exception:
                            pass
                    
                    now_mono = time.monotonic()
                    rtf_val, rtf_status, rtf_label = self.rolling_rtf_estimator.get_rolling_rtf(now_mono)
                    rem_tasks = max(0, self.target_tasks - len(self.completed_tasks))
                    eta_val, eta_status, eta_label = self.eta_estimator.get_eta(now_mono, rem_tasks)
                    finish_str = format_local_finish_time(eta_val) if eta_status == "VALID" else eta_label

                    campaign_str = ""
                    if self.campaign_estimator and self.run_index > 1:
                        _, c_str = self.campaign_estimator.get_campaign_eta(self.run_index, eta_val)
                        campaign_str = f" | Campaign ETA: {c_str}"

                    elapsed_fmt = f"{mins:02d}:{secs:02d}"
                    tot_fmt = f"{tot_m:02d}:{tot_s:02d}"
                    tasks_prog = f"Tasks: {len(self.completed_tasks)}/{self.target_tasks}"
                    robots_str = self.format_robots_status()

                    if sys.stdout.isatty():
                        ticker = (
                            f"\r ⏱ {C_CYAN}{C_BOLD}[{elapsed_fmt} / timeout {tot_fmt}]{C_RESET} | "
                            f"{rtf_label} | {tasks_prog} | logical task ETA: {eta_label} | "
                            f"Finish: {finish_str} | {robots_str}{campaign_str} | {C_GREEN}RUNNING{C_RESET}"
                        )
                        sys.stdout.write(ticker)
                        sys.stdout.flush()
                    elif sec_int % 10 == 0:
                        print(
                            f" [{sec_int:5.0f}s] [{elapsed_fmt} / timeout {tot_fmt}] | "
                            f"{rtf_label} | {tasks_prog} | logical task ETA: {eta_label} | "
                            f"Finish: {finish_str} | {robots_str}{campaign_str}"
                        )
                        sys.stdout.flush()

                time.sleep(0.2)

        except KeyboardInterrupt:
            if sys.stdout.isatty():
                sys.stdout.write("\r\033[K")
            self.print_stage_event("!", C_YELLOW, "ABORT", "User interrupted simulation via Ctrl+C")
            self.status = "INTERRUPTED"
            raise
        finally:
            if sys.stdout.isatty():
                sys.stdout.write("\r\033[K")
                sys.stdout.flush()
            if fleet_log_fd is not None:
                fleet_log_fd.close()
            self.teardown()

        # Display run summary with Real-Time Ratio (RTF)
        wall_dur = (self.end_time - self.start_time) if self.end_time > 0 else (time.time() - self.start_time)
        full_run_rtf = self.compute_rtf()
        full_run_str = f"{full_run_rtf:.2f}x" if full_run_rtf > 0 else "N/A"
        _, _, rolling_rtf_str = self.rolling_rtf_estimator.get_rolling_rtf(time.monotonic())
        sim_str = f"{self.sim_time_s:.1f}s" if self.sim_time_s > 0 else "N/A"
        status_color = C_GREEN if self.status == "PASSED" else C_RED
        print(f"\n {C_BOLD}{C_WHITE}[Run {self.run_index} Summary]{C_RESET} Status: {status_color}{self.status}{C_RESET} | Completed Tasks: {len(self.completed_tasks)}/{self.target_tasks} | Wall Time: {wall_dur:.1f}s | Sim Time: {sim_str} | {C_BOLD}{C_CYAN}{rolling_rtf_str}{C_RESET} (full_run_rtf: {full_run_str})\n")

        return self.status == "PASSED"

    def teardown(self):
        """Terminates simulation process tree and cleans up."""
        if self.process and self.process.poll() is None:
            try:
                # Send SIGINT first to process group so launch_four_amrs.sh cleanup() trap runs
                os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
                self.process.wait(timeout=6)
            except Exception:
                try:
                    os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
                    self.process.wait(timeout=4)
                except Exception:
                    try:
                        os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
                    except Exception:
                        pass
        self.clean_lingering_processes(graceful=True)
        self.save_summary()

    def save_summary(self):
        """Saves machine-readable run summary JSON in the log directory."""
        full_run_rtf = self.compute_rtf()
        rolling_rtf_val, _, rolling_rtf_str = self.rolling_rtf_estimator.get_rolling_rtf(time.monotonic())
        summary = {
            "run_id": self.run_id,
            "run_index": self.run_index,
            "scenario": self.scenario,
            "fleet_count": self.fleet_count,
            "tracking_speed_mps": self.tracking_speed,
            "enable_faults": self.enable_faults,
            "enable_spawner": self.enable_spawner,
            "enable_vision": False,
            "status": self.status,
            "wall_duration_s": self.wall_duration_s,
            "avg_task_duration_s": self.avg_task_duration_s,
            "time_per_task_s": self.time_per_task_s,
            "final_rolling_rtf_60s": rolling_rtf_val,
            "final_rolling_rtf_label": rolling_rtf_str,
            "full_run_rtf": full_run_rtf,
            "achieved_rtf": full_run_rtf,
            "target_tasks": self.target_tasks,
            "completed_tasks_count": len(self.completed_tasks),
            "completed_tasks": self.completed_tasks,
            "tasks_detail": {
                tid: {
                    "assigned_robot": t.assigned_robot,
                    "stage": t.stage,
                    "bid": t.bid,
                    "quorum": t.quorum,
                    "pickup_coord": t.pickup_coord,
                    "dropoff_coord": t.dropoff_coord,
                    "duration_s": t.duration_s
                }
                for tid, t in self.tasks.items()
            }
        }
        with open(self.log_dir / "run_summary.json", "w") as f:
            json.dump(summary, f, indent=2)


def generate_master_benchmark_report(runs: List[WorkCycleRun], base_dir: Path):
    """Generates a master Markdown report and summary JSON for the multi-run session."""
    report_file = base_dir / "BENCHMARK_REPORT.md"
    summary_file = base_dir / "benchmark_summary.json"
    
    passed_runs = sum(1 for r in runs if r.status == "PASSED")
    total_runs = len(runs)
    
    lines = [
        "# Multi-Work-Cycle Automated Test Benchmark Report",
        "",
        f"- **Session Date**: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        f"- **Total Test Runs**: {total_runs}",
        f"- **Passed Runs**: {passed_runs} / {total_runs} ({(passed_runs / total_runs * 100):.1f}%)",
        f"- **Tasks Per Cycle**: {runs[0].target_tasks if runs else 4}",
        f"- **Vision Recording**: Disabled (`FLEET_ENABLE_VISION=false`) across all runs",
        "",
        "## Active Runners Notice",
        "- **Primary Runner**: `scripts/run_multi_work_cycles.py`",
        "- **Alternative Runners Discovered**: `scripts/run_desktop_data_collection.py`, `scripts/run_laptop_data_collection.py`",
        "",
        "## Multi-Run Summary Table",
        "",
        "| Run | Run ID | Scenario | AMRs | Speed (m/s) | Status | Tasks | Wall Dur | Avg Task Dur | Time/Task | Final RTF(60s) | Full-Run RTF | Log Path |",
        "| :---: | :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :--- |"
    ]
    
    for r in runs:
        dur = r.wall_duration_s
        full_rtf = r.compute_rtf()
        full_rtf_str = f"{full_rtf:.2f}x" if full_rtf > 0 else "N/A"
        _, _, rolling_str = r.rolling_rtf_estimator.get_rolling_rtf(time.monotonic())
        badge = "✅ PASSED" if r.status == "PASSED" else f"❌ {r.status}"
        lines.append(
            f"| Run {r.run_index} | `{r.run_id}` | {r.scenario} | {r.fleet_count} | {r.tracking_speed:.2f} | {badge} | "
            f"{len(r.completed_tasks)}/{r.target_tasks} | {dur:.1f}s | {r.avg_task_duration_s:.1f}s | {r.time_per_task_s:.1f}s | "
            f"**{rolling_str}** | {full_rtf_str} | `{r.log_dir.name}/` |"
        )
        
    lines.extend([
        "",
        "## Detailed Task Breakdown Per Run",
        ""
    ])
    
    for r in runs:
        lines.append(f"### Run {r.run_index} (`{r.run_id}`)")
        lines.append("")
        lines.append("| Task ID | Robot | Pickup Coord | Dropoff Coord | Bid | Stage | Duration |")
        lines.append("| :--- | :---: | :---: | :---: | :---: | :---: | :---: |")
        for tid, t in r.tasks.items():
            lines.append(f"| `{tid}` | {t.assigned_robot or 'None'} | {t.pickup_coord or 'N/A'} | {t.dropoff_coord or 'N/A'} | {t.bid or 0.0:.2f} | {t.stage} | {t.duration_s:.1f}s |")
        lines.append("")

    with open(report_file, "w") as f:
        f.write("\n".join(lines))

    # Master JSON
    master_summary = {
        "timestamp": datetime.datetime.now().isoformat(),
        "total_runs": total_runs,
        "passed_runs": passed_runs,
        "runs": [
            {
                "run_index": r.run_index,
                "run_id": r.run_id,
                "status": r.status,
                "wall_duration_s": (r.end_time - r.start_time) if r.end_time > 0 else 0.0,
                "final_rolling_rtf_60s": r.rolling_rtf_estimator.get_rolling_rtf(time.monotonic())[0],
                "full_run_rtf": r.compute_rtf(),
                "completed_tasks": r.completed_tasks
            }
            for r in runs
        ]
    }
    with open(summary_file, "w") as f:
        json.dump(master_summary, f, indent=2)

    print(f"\n{C_BG_DARK}{C_WHITE}{C_BOLD} ===================== MULTI-RUN BENCHMARK SUMMARY ===================== {C_RESET}")
    print(f" {C_GREEN if passed_runs == total_runs else C_YELLOW}{C_BOLD}Total Success Rate:{C_RESET} {passed_runs}/{total_runs} runs passed ({(passed_runs / total_runs * 100):.1f}%)")
    print(f"\n {C_BOLD}{'Run':<8} {'Status':<12} {'Tasks':<10} {'Wall Time':<12} {'Sim Time':<12} {'Final RTF(60s)':<22} {'Full-Run RTF':<14}{C_RESET}")
    print(f" {'-'*92}")
    rolling_rtf_values = []
    for r in runs:
        dur = (r.end_time - r.start_time) if r.end_time > 0 else (time.time() - r.start_time)
        full_rtf = r.compute_rtf()
        full_rtf_str = f"{full_rtf:.2f}x" if full_rtf > 0 else "N/A"
        r_val, _, r_label = r.rolling_rtf_estimator.get_rolling_rtf(time.monotonic())
        if r_val and r_val > 0:
            rolling_rtf_values.append(r_val)
        sim_str = f"{r.sim_time_s:.1f}s" if r.sim_time_s > 0 else "N/A"
        status_color = C_GREEN if r.status == 'PASSED' else C_RED
        print(f" {f'Run {r.run_index}':<8} {status_color}{r.status:<12}{C_RESET} {f'{len(r.completed_tasks)}/{r.target_tasks}':<10} {f'{dur:.1f}s':<12} {sim_str:<12} {C_BOLD}{r_label:<22}{C_RESET} {full_rtf_str:<14}")
    print(f" {'-'*92}")
    avg_r_str = f"{sum(rolling_rtf_values)/len(rolling_rtf_values):.2f}x" if rolling_rtf_values else "N/A"
    print(f" {C_BOLD}Average Rolling RTF(60s):{C_RESET} {C_GREEN if rolling_rtf_values and sum(rolling_rtf_values)/len(rolling_rtf_values)>=0.5 else C_CYAN}{C_BOLD}{avg_r_str}{C_RESET}")
    print(f" {C_CYAN}Master Report:{C_RESET}      {report_file}")
    print(f" {C_CYAN}Summary JSON:{C_RESET}       {summary_file}\n")


def main():
    parser = argparse.ArgumentParser(
        description="Run sequential multi-AMR warehouse work-cycle tests with domain randomization, fault injection, and dataset generation."
    )
    parser.add_argument("--runs", type=int, default=5, help="Number of sequential work cycle test runs (default: 5)")
    parser.add_argument("--tasks-per-cycle", type=int, default=4, help="Number of completed tasks required per work cycle (default: 4)")
    parser.add_argument("--fleet-count", type=int, default=int(os.environ.get("FLEET_COUNT", "4")),
                        help="Number of AMRs in the fleet (default: 4 or $FLEET_COUNT)")
    parser.add_argument("--launcher", type=str, default=None, choices=["launch_four_amrs.sh", "launch_fleet_amrs.sh"],
                        help="Launch script to execute (default: launch_four_amrs.sh for <=4 AMRs, launch_fleet_amrs.sh for >4)")
    parser.add_argument("--tracking-speed", type=float, default=0.46, help="Fleet route tracking speed in m/s (default: 0.46 m/s physical_max)")
    parser.add_argument("--velocity-profile", type=str, default="physical_max",
                        help="Velocity profile name (physical_fidelity, physical_max, synthetic_0_75, synthetic_1_00, failure_envelope_2_00, failure_envelope_4_00)")
    parser.add_argument("--settle-seconds", type=int, default=5, help="AMR spawn settle time in seconds (default: 5s)")
    parser.add_argument("--timeout", type=int, default=1200, help="Max timeout per run in seconds (default: 1200s)")
    parser.add_argument("--output-dir", type=str, default=None, help="Root directory for multi-run logs")
    parser.add_argument("--seed", type=int, default=42, help="Base random seed for domain randomization (default: 42)")
    parser.add_argument("--enable-faults", action="store_true", help="Enable seeded fault injection harness")
    parser.add_argument("--enable-spawner", action="store_true", help="Enable dynamic obstacle spawner in safe zones")
    parser.add_argument("--enable-vision", action="store_true", help="Enable bounded camera dataset recorder")
    parser.add_argument("--export-dataset", action="store_true", help="Auto-generate tabular ML dataset CSV on completion")

    args = parser.parse_args()

    # Workspace directory resolution
    script_dir = Path(__file__).resolve().parent
    workspace_dir = script_dir.parent.parent.parent
    if args.output_dir:
        base_dir = Path(args.output_dir).resolve()
    else:
        session_tag = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        base_dir = workspace_dir / "log" / f"multi_work_cycles_{session_tag}"
    
    base_dir.mkdir(parents=True, exist_ok=True)

    # Resolve velocity profile
    if args.tracking_speed != 0.31 and args.velocity_profile == "physical_fidelity":
        profile = resolve_velocity_profile(str(args.tracking_speed))
    else:
        profile = resolve_velocity_profile(args.velocity_profile)
    effective_tracking_speed = profile.nominal_speed_mps

    # Check for other active runner scripts
    discovered_runners = []
    for rname in ["run_desktop_data_collection.py", "run_laptop_data_collection.py"]:
        if (script_dir / rname).exists():
            discovered_runners.append(rname)

    print(f"\n{C_BOLD}{C_CYAN}========================================================================={C_RESET}")
    print(f"{C_BOLD}{C_CYAN}   SIH DECENTRALIZED MULTI-AMR FLEET — WORK-CYCLE & DATASET RUNNER       {C_RESET}")
    print(f"{C_BOLD}{C_CYAN}========================================================================={C_RESET}")
    print(f" • Sequential Runs:    {args.runs}")
    print(f" • Tasks Per Cycle:    {args.tasks_per_cycle} completed tasks")
    print(f" • Fleet Size:         {args.fleet_count} AMRs")
    print(f" • Base Seed:          {args.seed}")
    print(f" • Fault Injection:    {'ENABLED' if args.enable_faults else 'DISABLED'}")
    print(f" • Dynamic Spawner:    {'ENABLED' if args.enable_spawner else 'DISABLED'}")
    print(f" • Vision Recording:   {'ENABLED' if args.enable_vision else 'DISABLED'}")
    print(f" • Velocity Profile:   {profile.name} ({profile.nominal_speed_mps} m/s) [{profile.tier}]")
    print(f"   Status:             {profile.status}")
    print(f"   Break-even RTF:     {profile.break_even_rtf():.3f}x to match nominal 4.0 m/s throughput")
    print(f"   Physics Step Disp:  {profile.displacement_per_step():.4f} m per 0.02s physics step")
    print(f" • Spawn Settle Time:  {args.settle_seconds} s")
    print(f" • Timeout Per Run:    {args.timeout} s")
    print(f" • Root Log Directory: {base_dir}")
    print(f" • GPU Acceleration:   NVIDIA GeForce RTX 3070 (Headless OGRE2, Fleet Profile)")
    if discovered_runners:
        print(f" • Discovered Runners: {', '.join(discovered_runners)} (Active: run_multi_work_cycles.py)")
    print(f"{C_CYAN}-------------------------------------------------------------------------{C_RESET}\n")

    campaign_estimator = CampaignETAEstimator(total_runs=args.runs, cooldown_seconds=5.0)
    runs: List[WorkCycleRun] = []
    current_run: Optional[WorkCycleRun] = None
    interrupted = False
    
    try:
        for i in range(1, args.runs + 1):
            run = WorkCycleRun(
                run_index=i,
                total_runs=args.runs,
                target_tasks=args.tasks_per_cycle,
                base_dir=base_dir,
                timeout_s=args.timeout,
                campaign_estimator=campaign_estimator,
                fleet_count=args.fleet_count,
                launcher_override=args.launcher
            )
            runs.append(run)
            current_run = run
            success = run.execute(
                tracking_speed=effective_tracking_speed,
                settle_s=args.settle_seconds,
                seed=args.seed,
                enable_faults=args.enable_faults,
                enable_spawner=args.enable_spawner,
                enable_vision=args.enable_vision
            )
            if run.end_time > 0 and run.start_time > 0:
                campaign_estimator.record_completed_run(run.end_time - run.start_time)
            current_run = None
            
            # Short cooldown between sequential runs
            if i < args.runs:
                print(f"\n{C_DIM}Cooldown between runs (5s)...{C_RESET}")
                time.sleep(5.0)

    except KeyboardInterrupt:
        interrupted = True
        print(f"\n\n {C_YELLOW}{C_BOLD}[!] Benchmark interrupted by user (Ctrl+C). Performing graceful shutdown...{C_RESET}")
        if current_run:
            current_run.status = "INTERRUPTED"
            current_run.teardown()
        else:
            dummy = WorkCycleRun(1, 1, 4, base_dir, 100, fleet_count=args.fleet_count)
            dummy.clean_lingering_processes(graceful=True)
        print(f" {C_GREEN}✔ All fleet and simulator processes cleanly stopped. DDS state cleared.{C_RESET}\n")
    finally:
        if runs:
            generate_master_benchmark_report(runs, base_dir)
            if args.export_dataset:
                dataset_csv = base_dir / "fleet_congestion_dataset.csv"
                telemetry_files = [str(r.telemetry_file) for r in runs if r.telemetry_file.exists()]
                if telemetry_files:
                    try:
                        cmd = ["python3", str(script_dir / "generate_ml_dataset.py"), *telemetry_files, "-o", str(dataset_csv)]
                        subprocess.run(cmd, check=True)
                        print(f" {C_GREEN}✔ Auto-exported ML Dataset CSV:{C_RESET} {dataset_csv}")
                    except Exception as e:
                        print(f" {C_RED}Failed to auto-export ML dataset: {e}{C_RESET}")
        if interrupted:
            sys.exit(130)


if __name__ == "__main__":
    main()
