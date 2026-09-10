#!/usr/bin/env python3
"""Desktop Automated Data Collection Runner (8 AMRs, 200 Tasks/Run, 12k Total).

Runs sequential work-cycle benchmarks on Desktop (Ryzen 5 5600X + RTX 3070):
- 8 TurtleBot 4 AMRs
- Unthrottled Gazebo physics (50 Hz, 3.5x RTF)
- Consolidated robot agent processes (low CPU footprint)
- DDS Domain IDs cycling in [10, 49]
- Disjoint random seeds in [1000, 1059]
- Auto-exports ML dataset CSV upon completion.
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

    @property
    def duration_s(self) -> float:
        if self.completion_time:
            return self.completion_time - self.start_wall_time
        return time.time() - self.start_wall_time


class DesktopCycleRun:
    def __init__(self, run_index: int, total_runs: int, target_tasks: int, base_dir: Path, timeout_s: int, fleet_count: int = 8):
        self.run_index = run_index
        self.total_runs = total_runs
        self.target_tasks = target_tasks
        self.base_dir = base_dir
        self.timeout_s = timeout_s
        self.fleet_count = fleet_count
        self.run_id = f"desktop_run_{run_index:03d}_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}"
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

    def clean_lingering_processes(self):
        patterns = [
            "gz sim", "gz-sim", "ros_gz_bridge", "parameter_bridge",
            "robot_agent_process", "data_collection_node", "obstacle_spawner_node",
            "warehouse_map_node", "random_task_generator_node", "spawn_minimal_amr"
        ]
        for pat in patterns:
            try:
                subprocess.run(["pkill", "-15", "-f", pat], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception:
                pass
        time.sleep(1.0)
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
        m = RE_ROBOT_READY.search(line)
        if m:
            robot = m.group("robot") or m.group("robot2")
            if robot and robot not in self.ready_robots:
                self.ready_robots.add(robot)
                self.print_stage_event("✔", C_CYAN, "ROBOT READY", f"{robot} interfaces ready ({len(self.ready_robots)}/{self.fleet_count})")

        m = RE_CBBA_COMMIT.search(line)
        if m:
            task_id = m.group("task")
            winner = m.group("winner")
            quorum = m.group("quorum")
            bid = float(m.group("bid"))
            t = self.tasks.setdefault(task_id, TaskState(task_id))
            if t.stage == "ANNOUNCED":
                t.assigned_robot = winner
                t.bid = bid
                t.quorum = quorum
                t.stage = "CBBA_COMMITTED"
                self.print_stage_event("★", C_MAGENTA, "CBBA DONE", f"Task {C_BOLD}{task_id}{C_RESET} assigned to {C_BOLD}{winner}{C_RESET} (Bid: {bid:.2f}, Quorum: {quorum})")

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
            if t.stage in ("ACCEPTED", "AT_PICKUP"):
                t.stage = "EN_ROUTE_DROPOFF"
                self.print_stage_event("➜", C_CYAN, "PICKUP DONE", f"{robot_id} finished loading {task_id}; heading to Dropoff {t.dropoff_coord}")

        m = RE_DROPOFF_ARRIVED.search(line)
        if m:
            task_id = m.group("task")
            robot_id = m.group("robot")
            dwell = m.group("dwell")
            t = self.tasks.setdefault(task_id, TaskState(task_id))
            if t.stage in ("ACCEPTED", "AT_PICKUP", "EN_ROUTE_DROPOFF"):
                t.stage = "AT_DROPOFF"
                t.dropoff_time = time.time()
                self.print_stage_event("⚓", C_YELLOW, "DROPOFF ARRIVED", f"{robot_id} reached dropoff station for {task_id} (dwelling {dwell}s)")

        m = RE_TASK_COMPLETED.search(line)
        if m:
            task_id = m.group("task") or m.group("task2")
            t = self.tasks.setdefault(task_id, TaskState(task_id))
            if t.stage != "COMPLETED":
                t.stage = "COMPLETED"
                t.completion_time = time.time()
                if task_id not in self.completed_tasks:
                    self.completed_tasks.append(task_id)
                    count = len(self.completed_tasks)
                    pct = int((count / self.target_tasks) * 100)
                    bar = "█" * (pct // 2) + "░" * (50 - (pct // 2))
                    self.print_stage_event(
                        "✔", C_GREEN, "TASK COMPLETED",
                        f"{C_BOLD}{task_id}{C_RESET} finished by {t.assigned_robot} | Duration: {t.duration_s:.1f}s | Progress: [{bar}] {count}/{self.target_tasks} ({pct}%)"
                    )

    def execute(self, tracking_speed: float = 4.0, settle_s: int = 3, seed: int = 1000) -> bool:
        self.clean_lingering_processes()
        self.start_time = time.time()
        
        env = os.environ.copy()
        env["START_GUI"] = "false"
        env["START_FLEET"] = "true"
        env["FLEET_RANDOM_TASKS"] = "true"
        env["FLEET_RECORD_DATA"] = "true"
        env["FLEET_COUNT"] = str(self.fleet_count)
        env["RENDER_ENGINE"] = "ogre2"
        env["SENSOR_PROFILE"] = "fleet"
        env["LIDAR_UPDATE_RATE_HZ"] = "10.0"
        env["FLEET_TRACKING_SPEED_MPS"] = str(tracking_speed)
        env["SETTLE_SECONDS"] = str(settle_s)
        env["LOG_DIR"] = str(self.log_dir)
        env["FLEET_DATA_FILE"] = str(self.telemetry_file)
        env["FLEET_RANDOM_SEED"] = str(seed + self.run_index)
        env["FLEET_ENABLE_FAULTS"] = "false"
        env["FLEET_ENABLE_SPAWNER"] = "true"
        env["FLEET_ENABLE_VISION"] = "false"
        env["__NV_PRIME_RENDER_OFFLOAD"] = "1"
        env["__GLX_VENDOR_LIBRARY_NAME"] = "nvidia"
        env["CUDA_VISIBLE_DEVICES"] = "0"
        
        # Desktop domain range [10, 49] - zero collision with laptop [60, 99]
        run_domain_id = 10 + (self.run_index % 40)
        env["ROS_DOMAIN_ID"] = str(run_domain_id)
        env["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"

        script_dir = Path(__file__).resolve().parent
        launch_script = script_dir / "launch_fleet_amrs.sh"
        if not launch_script.exists():
            print(f"{C_RED}ERROR: launch_fleet_amrs.sh not found at {launch_script}{C_RESET}")
            return False

        print(f"\n{C_BG_BLUE}{C_WHITE}{C_BOLD} >>> STARTING DESKTOP TEST RUN {self.run_index}/{self.total_runs}: {self.run_id} <<<{C_RESET}")
        print(f" {C_CYAN}Target Work Cycle:{C_RESET} {self.target_tasks} completed tasks across {self.fleet_count} AMRs")
        print(f" {C_CYAN}Log Directory:{C_RESET}     {self.log_dir}")
        print(f" {C_CYAN}DDS Domain ID:{C_RESET}     {run_domain_id} (Range [10, 49]) | Seed: {seed + self.run_index}")
        print(f" {C_CYAN}Fleet Architecture:{C_RESET}Consolidated Agent Processes, 50 Hz Physics, Lean Telemetry\n")
        sys.stdout.flush()

        self.process = subprocess.Popen(
            ["bash", str(launch_script)],
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
            text=True,
            bufsize=1
        )

        fleet_log_fd = None
        last_ticker_s = -1
        first_sim_s = None
        
        try:
            while self.process.poll() is None:
                if len(self.completed_tasks) >= self.target_tasks:
                    self.status = "PASSED"
                    break

                elapsed = time.time() - self.start_time
                if elapsed > self.timeout_s:
                    self.status = "TIMEOUT"
                    break

                # Tail fleet.log for consensus and executor events
                if fleet_log_fd is None and self.fleet_log_file.exists():
                    fleet_log_fd = open(self.fleet_log_file, "r", encoding="utf-8", errors="ignore")

                if fleet_log_fd is not None:
                    for line in fleet_log_fd:
                        self.parse_log_line(line)

                # Track RTF from telemetry
                if self.telemetry_file.exists() and int(elapsed) % 15 == 0 and int(elapsed) != last_ticker_s:
                    last_ticker_s = int(elapsed)
                    try:
                        with open(self.telemetry_file, "r", encoding="utf-8", errors="ignore") as tf:
                            for tline in tf:
                                if tline.strip():
                                    rec = json.loads(tline)
                                    sim_t = float(rec.get("logged_at", 0.0))
                                    if sim_t > 0.0:
                                        if first_sim_s is None: first_sim_s = sim_t
                                        self.sim_time_s = sim_t
                    except Exception:
                        pass
                    
                    if first_sim_s is not None and self.sim_time_s > first_sim_s:
                        d_sim = self.sim_time_s - first_sim_s
                        rtf = d_sim / max(1.0, elapsed)
                        sys.stdout.write(f"\r {C_DIM}[{elapsed:6.1f}s]{C_RESET} {C_YELLOW}⚡ Active RTF: {rtf:.2f}x | Completed: {len(self.completed_tasks)}/{self.target_tasks}{C_RESET}  ")
                        sys.stdout.flush()

                time.sleep(0.1)

        except KeyboardInterrupt:
            self.status = "ABORTED"
        finally:
            if fleet_log_fd:
                fleet_log_fd.close()
            self.end_time = time.time()
            if self.process and self.process.poll() is None:
                try:
                    os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
                    self.process.wait(timeout=10)
                except Exception:
                    try:
                        os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
                    except Exception:
                        pass
            self.clean_lingering_processes()

        wall_duration = self.end_time - self.start_time
        rtf = (self.sim_time_s / wall_duration) if wall_duration > 0 else 0.0
        print(f"\n\n [Run {self.run_index} Summary] Status: {self.status} | Completed Tasks: {len(self.completed_tasks)}/{self.target_tasks} | Wall Time: {wall_duration:.1f}s | RTF: {rtf:.2f}x\n")
        return self.status == "PASSED"


def main():
    parser = argparse.ArgumentParser(description="Desktop Fleet Data Collection Runner (8 AMRs)")
    parser.add_argument("--runs", type=int, default=60, help="Number of simulation work cycles (default: 60)")
    parser.add_argument("--tasks", type=int, default=200, help="Target tasks per cycle (default: 200)")
    parser.add_argument("--speed", type=float, default=4.0, help="AMR path tracking speed m/s (default: 4.0)")
    parser.add_argument("--seed", type=int, default=1000, help="Base random seed for Desktop (default: 1000)")
    parser.add_argument("--timeout", type=int, default=22000, help="Per-run timeout seconds (default: 22000, 5.5x extended)")
    parser.add_argument("--output-csv", default="desktop_fleet_12k_dataset.csv", help="Combined dataset CSV output name")
    args = parser.parse_args()

    workspace_log = os.environ.get("AMR_WS_LOG_DIR")
    if workspace_log:
        base_dir = Path(workspace_log) / f"desktop_data_collection_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}"
    else:
        base_dir = Path.home() / "amr_ws/log" / f"desktop_data_collection_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}"
    base_dir.mkdir(parents=True, exist_ok=True)

    print(f"\n{C_BOLD}{C_GREEN}======================================================================{C_RESET}")
    print(f"{C_BOLD}{C_GREEN}  SIH DESKTOP AUTOMATED DATA COLLECTION: 8 AMRs, {args.runs} RUNS × {args.tasks} TASKS  {C_RESET}")
    print(f"{C_BOLD}{C_GREEN}  TARGET: {args.runs * args.tasks} DATASET TASKS FOR ML CONGESTION MODEL  {C_RESET}")
    print(f"{C_BOLD}{C_GREEN}======================================================================{C_RESET}\n")

    telemetry_files = []
    passed = 0
    for idx in range(1, args.runs + 1):
        run = DesktopCycleRun(idx, args.runs, args.tasks, base_dir, args.timeout, fleet_count=8)
        success = run.execute(tracking_speed=args.speed, settle_s=3, seed=args.seed)
        if run.telemetry_file.exists():
            telemetry_files.append(str(run.telemetry_file))
        if success:
            passed += 1

    # Automatically generate the combined ML dataset CSV
    print(f"\n{C_BOLD}{C_CYAN}>>> Merging {len(telemetry_files)} telemetry logs into ML Dataset CSV: {args.output_csv} <<<{C_RESET}")
    script_dir = Path(__file__).resolve().parent
    gen_script = script_dir / "generate_ml_dataset.py"
    if gen_script.exists() and telemetry_files:
        subprocess.run(["python3", str(gen_script)] + telemetry_files + ["--output", args.output_csv])
        print(f"\n{C_BOLD}{C_GREEN}✔ Desktop Data Collection Finished: {passed}/{args.runs} runs passed ({passed * args.tasks} tasks).{C_RESET}\n")


if __name__ == "__main__":
    main()
