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
import re
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Set

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
RE_CBBA_COMMIT = re.compile(
    r"\[(?P<robot>robot_\d):CBBA\]\s*Decision:\s*UNANIMOUS_COMMIT for task (?P<task>[a-zA-Z0-9_-]+) -> Winner=(?P<winner>robot_\d),\s*Bid=(?P<bid>[\d.]+),\s*epoch=(?P<epoch>\d+)\..*Quorum=(?P<quorum>\d+/\d+)"
)
RE_TASK_ACCEPT = re.compile(
    r"\[(?P<robot>robot_\d):TaskExecutor\]\s*Decision:\s*ACCEPT new task (?P<task>[a-zA-Z0-9_-]+)\..*pickup=\((?P<px>[-\d.]+),\s*(?P<py>[-\d.]+)\),\s*dropoff=\((?P<dx>[-\d.]+),\s*(?P<dy>[-\d.]+)\)"
)
RE_PICKUP_ARRIVED = re.compile(
    r"\[(?P<robot>robot_\d):TaskExecutor\]\s*Decision:\s*ARRIVED at pickup for task (?P<task>[a-zA-Z0-9_-]+)\..*dwelling (?P<dwell>[-\d.]+)s"
)
RE_PICKUP_DONE = re.compile(
    r"\[(?P<robot>robot_\d):TaskExecutor\]\s*Decision:\s*PICKUP_DWELL_COMPLETE for task (?P<task>[a-zA-Z0-9_-]+)"
)
RE_DROPOFF_ARRIVED = re.compile(
    r"\[(?P<robot>robot_\d):TaskExecutor\]\s*Decision:\s*ARRIVED at dropoff for task (?P<task>[a-zA-Z0-9_-]+)\..*dwelling (?P<dwell>[-\d.]+)s"
)
RE_TASK_COMPLETED = re.compile(
    r"\[(?P<robot>robot_\d):TaskExecutor\]\s*Decision:\s*(?:DROPOFF_DWELL_COMPLETE for task (?P<task>[a-zA-Z0-9_-]+)\..*Transition to COMPLETED|RESET executor after publishing completion for task (?P<task2>[a-zA-Z0-9_-]+))"
)
RE_ROBOT_READY = re.compile(
    r"(?:\[(?P<robot>robot_\d)\.interface_readiness\]:\s*Interface readiness passed|(?P<robot2>robot_\d) passed all gates)"
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
    def __init__(self, run_index: int, total_runs: int, target_tasks: int, base_dir: Path, timeout_s: int):
        self.run_index = run_index
        self.total_runs = total_runs
        self.target_tasks = target_tasks
        self.base_dir = base_dir
        self.timeout_s = timeout_s
        self.run_id = f"run_{run_index:02d}_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}"
        self.log_dir = base_dir / self.run_id
        self.log_dir.mkdir(parents=True, exist_ok=True)
        
        self.fleet_log_file = self.log_dir / "fleet.log"
        self.telemetry_file = self.log_dir / "fleet_telemetry.jsonl"
        self.events_file = self.log_dir / "run_events.jsonl"
        
        self.tasks: Dict[str, TaskState] = {}
        self.completed_tasks: List[str] = []
        self.ready_robots: Set[str] = set()
        self.start_time: float = 0.0
        self.end_time: float = 0.0
        self.sim_time_s: float = 0.0
        self.process: Optional[subprocess.Popen] = None
        self.status: str = "PENDING"

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
        print(f" {C_DIM}{timestamp_str}{C_RESET} {color}{C_BOLD}{symbol} [{tag:<14}]{C_RESET} {message}")
        sys.stdout.flush()

    def parse_log_line(self, line: str):
        # 1. Robot readiness check
        m = RE_ROBOT_READY.search(line)
        if m:
            robot = m.group("robot") or m.group("robot2")
            if robot and robot not in self.ready_robots:
                self.ready_robots.add(robot)
                self.print_stage_event("✔", C_CYAN, "ROBOT READY", f"{robot} interfaces passed readiness gate ({len(self.ready_robots)}/4)")

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
            t = self.tasks.setdefault(task_id, TaskState(task_id))
            if t.stage == "ACCEPTED":
                t.stage = "AT_PICKUP"
                t.pickup_time = time.time()
                self.print_stage_event("⚓", C_YELLOW, "PICKUP ARRIVED", f"{robot_id} reached pickup station for {task_id} (dwelling {dwell}s)")

        m = RE_PICKUP_DONE.search(line)
        if m:
            task_id = m.group("task")
            robot_id = m.group("robot")
            t = self.tasks.setdefault(task_id, TaskState(task_id))
            if t.stage == "AT_PICKUP":
                t.stage = "EN_ROUTE_DROPOFF"
                self.print_stage_event("➜", C_BLUE, "PICKUP DONE", f"{robot_id} finished loading {task_id}; heading to Dropoff {t.dropoff_coord or ''}")

        # 5. Dropoff Arrived & Dwell
        m = RE_DROPOFF_ARRIVED.search(line)
        if m:
            task_id = m.group("task")
            robot_id = m.group("robot")
            dwell = m.group("dwell")
            t = self.tasks.setdefault(task_id, TaskState(task_id))
            if t.stage == "EN_ROUTE_DROPOFF":
                t.stage = "AT_DROPOFF"
                t.dropoff_time = time.time()
                self.print_stage_event("⚓", C_YELLOW, "DROPOFF ARRIVED", f"{robot_id} reached dropoff station for {task_id} (dwelling {dwell}s)")

        # 6. Task Completed
        m = RE_TASK_COMPLETED.search(line)
        if m:
            task_id = m.group("task") or m.group("task2")
            robot_id = m.group("robot")
            t = self.tasks.setdefault(task_id, TaskState(task_id))
            if task_id not in self.completed_tasks:
                t.stage = "COMPLETED"
                t.completion_time = time.time()
                self.completed_tasks.append(task_id)
                count = len(self.completed_tasks)
                pct = int((count / self.target_tasks) * 100)
                bar = "█" * (count * 4) + "░" * (max(0, self.target_tasks - count) * 4)
                self.print_stage_event(
                    "✔", C_GREEN, "TASK COMPLETED",
                    f"{C_BOLD}{task_id}{C_RESET} finished by {robot_id} | Duration: {t.duration_s:.1f}s | Progress: [{bar}] {count}/{self.target_tasks} ({pct}%)"
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

    def execute(self, tracking_speed: float, settle_s: int) -> bool:
        """Executes a single work-cycle test run."""
        self.clean_lingering_processes()
        self.start_time = time.time()
        
        # Setup environment variables optimized for GPU and high RTF
        env = os.environ.copy()
        env["START_GUI"] = "false"
        env["START_FLEET"] = "true"
        env["FLEET_RANDOM_TASKS"] = "true"
        env["FLEET_RECORD_DATA"] = "true"
        env["RENDER_ENGINE"] = "ogre2"
        env["GUI_RENDER_ENGINE"] = "ogre2"
        env["SENSOR_PROFILE"] = "fleet"  # Lean 2D lidar profile (avoids 44 GPU depth camera rendering pipelines)
        env["LIDAR_UPDATE_RATE_HZ"] = os.environ.get("LIDAR_UPDATE_RATE_HZ", "10.0")
        env["FLEET_TRACKING_SPEED_MPS"] = str(tracking_speed)
        env["SETTLE_SECONDS"] = str(settle_s)
        env["LOG_DIR"] = str(self.log_dir)
        env["FLEET_DATA_FILE"] = str(self.telemetry_file)
        
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

        launch_script = script_dir / "launch_four_amrs.sh"
        if not launch_script.exists():
            print(f"{C_RED}ERROR: launch_four_amrs.sh not found at {launch_script}{C_RESET}")
            return False

        print(f"\n{C_BG_BLUE}{C_WHITE}{C_BOLD} >>> STARTING TEST RUN {self.run_index}/{self.total_runs}: {self.run_id} <<<{C_RESET}")
        print(f" {C_CYAN}Target Work Cycle:{C_RESET} {self.target_tasks} completed tasks across 4 AMRs")
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

        # Monitor fleet.log and stdout simultaneously
        fleet_log_fd = None
        last_ticker_s = -1
        first_sim_s = None
        
        try:
            while True:
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
                    live_rtf_str = ""
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
                                                if first_sim_s is None:
                                                    first_sim_s = st
                                                d_sim = st - first_sim_s
                                                if wall_elapsed > 3.0 and d_sim > 0.0:
                                                    curr_rtf = d_sim / wall_elapsed
                                                    live_rtf_str = f" | Sim: {st:.1f}s ({curr_rtf:.2f}x)"
                                                else:
                                                    live_rtf_str = f" | Sim: {st:.1f}s"
                                                break
                        except Exception:
                            pass
                    
                    tasks_prog = f"Tasks: {len(self.completed_tasks)}/{self.target_tasks} done"
                    assigned_count = len([t for t in self.tasks.values() if t.stage not in ("COMPLETED", "ANNOUNCED")])
                    active_str = f"Active: {assigned_count}" if assigned_count > 0 else "Pending assignment"
                    
                    if sys.stdout.isatty():
                        ticker = f"\r ⏱ {C_CYAN}{C_BOLD}[{mins:02d}:{secs:02d} / {tot_m:02d}:{tot_s:02d}]{C_RESET} ({sec_int}s){live_rtf_str} | {tasks_prog} | {active_str} | {C_GREEN}RUNNING{C_RESET}"
                        sys.stdout.write(ticker)
                        sys.stdout.flush()
                    elif sec_int % 10 == 0:
                        print(f" [{sec_int:5.0f}s] ⏱ [CLOCK] Wall: {mins:02d}:{secs:02d} / {tot_m:02d}:{tot_s:02d}{live_rtf_str} | {tasks_prog}")
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
        rtf = self.compute_rtf()
        rtf_str = f"{rtf:.2f}x" if rtf > 0 else "N/A"
        sim_str = f"{self.sim_time_s:.1f}s" if self.sim_time_s > 0 else "N/A"
        status_color = C_GREEN if self.status == "PASSED" else C_RED
        print(f"\n {C_BOLD}{C_WHITE}[Run {self.run_index} Summary]{C_RESET} Status: {status_color}{self.status}{C_RESET} | Completed Tasks: {len(self.completed_tasks)}/{self.target_tasks} | Wall Time: {wall_dur:.1f}s | Sim Time: {sim_str} | {C_BOLD}{C_CYAN}Real-Time Ratio (RTF): {rtf_str}{C_RESET}\n")

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
        summary = {
            "run_id": self.run_id,
            "run_index": self.run_index,
            "status": self.status,
            "wall_duration_s": (self.end_time - self.start_time) if self.end_time > 0 else (time.time() - self.start_time),
            "achieved_rtf": self.compute_rtf(),
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
        "",
        "## Multi-Run Summary Table",
        "",
        "| Run | Run ID | Status | Completed Tasks | Wall Duration | Achieved RTF | Log Path |",
        "| :---: | :--- | :---: | :---: | :---: | :---: | :--- |"
    ]
    
    for r in runs:
        dur = (r.end_time - r.start_time) if r.end_time > 0 else 0.0
        rtf = r.compute_rtf()
        rtf_str = f"{rtf:.2f}x" if rtf > 0 else "N/A"
        badge = "✅ PASSED" if r.status == "PASSED" else f"❌ {r.status}"
        lines.append(f"| Run {r.run_index} | `{r.run_id}` | {badge} | {len(r.completed_tasks)}/{r.target_tasks} | {dur:.1f}s | **{rtf_str}** | `{r.log_dir.name}/` |")
        
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
                "rtf": r.compute_rtf(),
                "completed_tasks": r.completed_tasks
            }
            for r in runs
        ]
    }
    with open(summary_file, "w") as f:
        json.dump(master_summary, f, indent=2)

    print(f"\n{C_BG_DARK}{C_WHITE}{C_BOLD} ===================== MULTI-RUN BENCHMARK SUMMARY ===================== {C_RESET}")
    print(f" {C_GREEN if passed_runs == total_runs else C_YELLOW}{C_BOLD}Total Success Rate:{C_RESET} {passed_runs}/{total_runs} runs passed ({(passed_runs / total_runs * 100):.1f}%)")
    print(f"\n {C_BOLD}{'Run':<8} {'Status':<12} {'Tasks':<10} {'Wall Time':<12} {'Sim Time':<12} {'Real-Time Ratio (RTF)':<22}{C_RESET}")
    print(f" {'-'*76}")
    rtf_values = []
    for r in runs:
        dur = (r.end_time - r.start_time) if r.end_time > 0 else (time.time() - r.start_time)
        rtf = r.compute_rtf()
        if rtf > 0:
            rtf_values.append(rtf)
        rtf_str = f"{rtf:.2f}x" if rtf > 0 else "N/A"
        sim_str = f"{r.sim_time_s:.1f}s" if r.sim_time_s > 0 else "N/A"
        status_color = C_GREEN if r.status == 'PASSED' else C_RED
        print(f" {f'Run {r.run_index}':<8} {status_color}{r.status:<12}{C_RESET} {f'{len(r.completed_tasks)}/{r.target_tasks}':<10} {f'{dur:.1f}s':<12} {sim_str:<12} {C_BOLD}{rtf_str:<22}{C_RESET}")
    print(f" {'-'*76}")
    avg_rtf_str = f"{sum(rtf_values)/len(rtf_values):.2f}x" if rtf_values else "N/A"
    print(f" {C_BOLD}Average Real-Time Ratio (RTF):{C_RESET} {C_GREEN if rtf_values and sum(rtf_values)/len(rtf_values)>=0.5 else C_CYAN}{C_BOLD}{avg_rtf_str}{C_RESET}")
    print(f" {C_CYAN}Master Report:{C_RESET}      {report_file}")
    print(f" {C_CYAN}Summary JSON:{C_RESET}       {summary_file}\n")


def main():
    parser = argparse.ArgumentParser(
        description="Run 5 sequential multi-AMR warehouse work-cycle tests with GPU acceleration and live stage tracking."
    )
    parser.add_argument("--runs", type=int, default=5, help="Number of sequential work cycle test runs (default: 5)")
    parser.add_argument("--tasks-per-cycle", type=int, default=4, help="Number of completed tasks required per work cycle (default: 4)")
    parser.add_argument("--tracking-speed", type=float, default=4.0, help="Fleet route tracking speed in m/s (default: 4.0 m/s accelerated)")
    parser.add_argument("--settle-seconds", type=int, default=5, help="AMR spawn settle time in seconds (default: 5s)")
    parser.add_argument("--timeout", type=int, default=1200, help="Max timeout per run in seconds (default: 1200s)")
    parser.add_argument("--output-dir", type=str, default=None, help="Root directory for multi-run logs")

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

    print(f"\n{C_BOLD}{C_CYAN}========================================================================={C_RESET}")
    print(f"{C_BOLD}{C_CYAN}   SIH DECENTRALIZED MULTI-AMR FLEET — 5-TEST WORK-CYCLE RUNNER          {C_RESET}")
    print(f"{C_BOLD}{C_CYAN}========================================================================={C_RESET}")
    print(f" • Sequential Runs:    {args.runs}")
    print(f" • Tasks Per Cycle:    {args.tasks_per_cycle} completed tasks")
    print(f" • Tracking Speed:     {args.tracking_speed} m/s")
    print(f" • Spawn Settle Time:  {args.settle_seconds} s")
    print(f" • Timeout Per Run:    {args.timeout} s")
    print(f" • Root Log Directory: {base_dir}")
    print(f" • GPU Acceleration:   NVIDIA GeForce RTX 3070 (Headless OGRE2, Fleet 2D Profile)")
    print(f"{C_CYAN}-------------------------------------------------------------------------{C_RESET}\n")

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
                timeout_s=args.timeout
            )
            runs.append(run)
            current_run = run
            success = run.execute(tracking_speed=args.tracking_speed, settle_s=args.settle_seconds)
            current_run = None
            
            # Short cooldown between sequential runs to allow Cyclone DDS ports to reset
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
            dummy = WorkCycleRun(1, 1, 4, base_dir, 100)
            dummy.clean_lingering_processes(graceful=True)
        print(f" {C_GREEN}✔ All fleet and simulator processes cleanly stopped. DDS state cleared.{C_RESET}\n")
    finally:
        if runs:
            generate_master_benchmark_report(runs, base_dir)
        if interrupted:
            sys.exit(130)


if __name__ == "__main__":
    main()
