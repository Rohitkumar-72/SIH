#!/usr/bin/env python3
"""Automated Two-Phase Optimization Runner for SIH Decentralized AMR Fleet.

Workflow:
--------------------------------------------------------------------------------
Phase 1: Speed Profile Benchmark (4 Runs, 6 Tasks/Run, 4 AMRs)
  - Run 1: physical_fidelity (0.31 m/s)
  - Run 2: physical_max (0.46 m/s)
  - Run 3: synthetic_0_75 (0.75 m/s)
  - Run 4: synthetic_1_00 (1.00 m/s)
  -> Evaluates data, average time taken per task, and determines the WINNING SPEED PROFILE.

Phase 2: Fleet Size Scaling Sweep (3 Runs, 15 Tasks/Run, Using Winner Speed Profile)
  - Run 5: 4 AMRs
  - Run 6: 5 AMRs
  - Run 7: 6 AMRs
  -> Evaluates data, average time taken per task, and determines the WINNING FLEET SIZE.
--------------------------------------------------------------------------------

Features:
- Real-time AMR spawning phase progress streaming.
- Live per-robot state machine ticker (R1..R6).
- 60s Rolling RTF & Logical Task ETA.
- Campaign ETA tracking across all 7 runs.
- Generates OPTIMIZATION_REPORT.md and optimization_summary.json.
- Strictly keeps vision recording disabled across all runs.
"""

import argparse
import datetime
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

# Add package directory to sys.path
SCRIPT_DIR = Path(__file__).resolve().parent
SIH_SRC_DIR = SCRIPT_DIR.parent / "src" / "sih_amr_fleet"
if str(SIH_SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SIH_SRC_DIR))
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from sih_amr_fleet.runner_metrics import (
    CampaignETAEstimator,
    RollingRTFEstimator,
    TaskETAEstimator,
    format_duration,
    format_local_finish_time,
)
from sih_amr_fleet.velocity_profiles import (
    PROFILES as VELOCITY_PROFILES,
    get_profile,
    resolve_velocity_profile,
)

# Import WorkCycleRun from run_multi_work_cycles
from run_multi_work_cycles import (
    C_BG_BLUE,
    C_BG_DARK,
    C_BOLD,
    C_CYAN,
    C_DIM,
    C_GREEN,
    C_MAGENTA,
    C_RED,
    C_RESET,
    C_WHITE,
    C_YELLOW,
    WorkCycleRun,
)

# Phase 1: 4 Velocity Profiles to Benchmark
PHASE1_PROFILES = [
    "physical_fidelity",  # 0.31 m/s
    "physical_max",       # 0.46 m/s
    "synthetic_0_75",     # 0.75 m/s
    "synthetic_1_00",     # 1.00 m/s
]

# Phase 2: Fleet Size Scaling
PHASE2_FLEET_SIZES = [4, 5, 6]


def evaluate_phase1_winner(runs: List[WorkCycleRun]) -> Tuple[WorkCycleRun, str, float]:
    """Selects the speed profile yielding the lowest average time taken per task."""
    valid_runs = [r for r in runs if len(r.completed_tasks) > 0]
    if not valid_runs:
        # Fallback to physical_fidelity
        return runs[0], "physical_fidelity", 0.31
    
    # Sort primarily by time_per_task_s (wall_duration / completed_tasks), secondarily by avg_task_duration_s
    winner = min(valid_runs, key=lambda r: (r.time_per_task_s, r.avg_task_duration_s))
    profile_name = getattr(winner, "profile_name", "physical_fidelity")
    speed_mps = winner.tracking_speed
    return winner, profile_name, speed_mps


def evaluate_phase2_winner(runs: List[WorkCycleRun]) -> Tuple[WorkCycleRun, int]:
    """Selects the fleet size setting yielding the lowest average time taken per task."""
    valid_runs = [r for r in runs if len(r.completed_tasks) > 0]
    if not valid_runs:
        return runs[0], runs[0].fleet_count
    
    winner = min(valid_runs, key=lambda r: (r.time_per_task_s, r.avg_task_duration_s))
    return winner, winner.fleet_count


def generate_optimization_report(
    phase1_runs: List[WorkCycleRun],
    phase2_runs: List[WorkCycleRun],
    winner_profile_name: str,
    winner_speed_mps: float,
    winner_fleet_size: int,
    base_dir: Path
):
    """Generates detailed Markdown and JSON reports for the two-phase optimization experiment."""
    report_file = base_dir / "OPTIMIZATION_REPORT.md"
    summary_file = base_dir / "optimization_summary.json"
    
    all_runs = phase1_runs + phase2_runs
    passed_all = sum(1 for r in all_runs if r.status == "PASSED")
    total_all = len(all_runs)

    p1_winner_run = min(phase1_runs, key=lambda r: r.time_per_task_s) if phase1_runs else None
    p2_winner_run = min(phase2_runs, key=lambda r: r.time_per_task_s) if phase2_runs else None

    lines = [
        "# Automated Speed Profile & AMR Fleet Scaling Optimization Report",
        "",
        f"- **Session Date**: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        f"- **Total Benchmark Runs**: {total_all}",
        f"- **Total Passed Runs**: {passed_all} / {total_all} ({(passed_all / max(1, total_all) * 100):.1f}%)",
        f"- **Vision Recording**: Disabled (`FLEET_ENABLE_VISION=false`) across all runs",
        f"- **Winner Speed Profile**: **`{winner_profile_name}`** ({winner_speed_mps:.2f} m/s)",
        f"- **Winner Fleet Scaling**: **{winner_fleet_size} AMRs**",
        "",
        "---",
        "",
        "## Phase 1: Speed Profile Benchmark (4 Runs × 6 Tasks, 4 AMRs)",
        "",
        "Evaluates which velocity profile yields the lowest time taken per completed task.",
        "",
        "| Run | Profile Name | Speed (m/s) | Status | Tasks | Wall Dur | Avg Task Dur | Time/Task | Throughput | Final RTF(60s) | Full RTF |",
        "| :---: | :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |"
    ]

    for r in phase1_runs:
        badge = "✅ PASSED" if r.status == "PASSED" else f"❌ {r.status}"
        pname = getattr(r, "profile_name", "N/A")
        tasks_str = f"{len(r.completed_tasks)}/{r.target_tasks}"
        throughput = (len(r.completed_tasks) / (r.wall_duration_s / 60.0)) if r.wall_duration_s > 0 else 0.0
        full_rtf = r.compute_rtf()
        full_rtf_str = f"{full_rtf:.2f}x" if full_rtf > 0 else "N/A"
        _, _, rolling_str = r.rolling_rtf_estimator.get_rolling_rtf(time.monotonic())
        winner_mark = " 🏆 **WINNER**" if (p1_winner_run and r.run_id == p1_winner_run.run_id) else ""
        lines.append(
            f"| Run {r.run_index} | `{pname}`{winner_mark} | {r.tracking_speed:.2f} | {badge} | {tasks_str} | "
            f"{r.wall_duration_s:.1f}s | {r.avg_task_duration_s:.1f}s | **{r.time_per_task_s:.1f}s** | "
            f"{throughput:.2f} t/min | {rolling_str} | {full_rtf_str} |"
        )

    lines.extend([
        "",
        f"> 🏆 **Phase 1 Winner**: **`{winner_profile_name}`** at **{winner_speed_mps:.2f} m/s** achieved the lowest time per task ({p1_winner_run.time_per_task_s:.1f}s/task)." if p1_winner_run else "",
        "",
        "---",
        "",
        f"## Phase 2: AMR Fleet Scaling Sweep (3 Runs × 15 Tasks, Using `{winner_profile_name}` at {winner_speed_mps:.2f} m/s)",
        "",
        "Evaluates which AMR fleet setting (4, 5, or 6 AMRs) yields the lowest time taken per task.",
        "",
        "| Run | Fleet Size | Speed (m/s) | Status | Tasks | Wall Dur | Avg Task Dur | Time/Task | Throughput | Final RTF(60s) | Full RTF |",
        "| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |"
    ])

    for r in phase2_runs:
        badge = "✅ PASSED" if r.status == "PASSED" else f"❌ {r.status}"
        tasks_str = f"{len(r.completed_tasks)}/{r.target_tasks}"
        throughput = (len(r.completed_tasks) / (r.wall_duration_s / 60.0)) if r.wall_duration_s > 0 else 0.0
        full_rtf = r.compute_rtf()
        full_rtf_str = f"{full_rtf:.2f}x" if full_rtf > 0 else "N/A"
        _, _, rolling_str = r.rolling_rtf_estimator.get_rolling_rtf(time.monotonic())
        winner_mark = " 🏆 **WINNER**" if (p2_winner_run and r.run_id == p2_winner_run.run_id) else ""
        lines.append(
            f"| Run {r.run_index} | **{r.fleet_count} AMRs**{winner_mark} | {r.tracking_speed:.2f} | {badge} | {tasks_str} | "
            f"{r.wall_duration_s:.1f}s | {r.avg_task_duration_s:.1f}s | **{r.time_per_task_s:.1f}s** | "
            f"{throughput:.2f} t/min | {rolling_str} | {full_rtf_str} |"
        )

    lines.extend([
        "",
        f"> 🏆 **Phase 2 Winner**: **{winner_fleet_size} AMRs** achieved the lowest time per task ({p2_winner_run.time_per_task_s:.1f}s/task) with {p2_winner_run.target_tasks} tasks." if p2_winner_run else "",
        "",
        "---",
        "",
        "## Master Optimal Fleet Configuration Recommendation",
        "",
        f"- **Optimal Velocity Profile**: `{winner_profile_name}` ({winner_speed_mps:.2f} m/s)",
        f"- **Optimal Fleet Size**: {winner_fleet_size} AMRs",
        f"- **Phase 1 Benchmark Time/Task**: {p1_winner_run.time_per_task_s:.1f}s" if p1_winner_run else "",
        f"- **Phase 2 Fleet Time/Task**: {p2_winner_run.time_per_task_s:.1f}s" if p2_winner_run else "",
        ""
    ])

    with open(report_file, "w") as f:
        f.write("\n".join(lines))

    # Master JSON serialization
    summary_data = {
        "timestamp": datetime.datetime.now().isoformat(),
        "total_runs": total_all,
        "passed_runs": passed_all,
        "winner_speed_profile": winner_profile_name,
        "winner_speed_mps": winner_speed_mps,
        "winner_fleet_size": winner_fleet_size,
        "phase1_speed_benchmark": [
            {
                "run_index": r.run_index,
                "run_id": r.run_id,
                "profile_name": getattr(r, "profile_name", "N/A"),
                "speed_mps": r.tracking_speed,
                "fleet_count": r.fleet_count,
                "status": r.status,
                "completed_tasks": len(r.completed_tasks),
                "target_tasks": r.target_tasks,
                "wall_duration_s": r.wall_duration_s,
                "avg_task_duration_s": r.avg_task_duration_s,
                "time_per_task_s": r.time_per_task_s,
                "throughput_tasks_per_min": (len(r.completed_tasks) / (r.wall_duration_s / 60.0)) if r.wall_duration_s > 0 else 0.0,
                "full_run_rtf": r.compute_rtf()
            }
            for r in phase1_runs
        ],
        "phase2_fleet_scaling": [
            {
                "run_index": r.run_index,
                "run_id": r.run_id,
                "fleet_count": r.fleet_count,
                "speed_mps": r.tracking_speed,
                "status": r.status,
                "completed_tasks": len(r.completed_tasks),
                "target_tasks": r.target_tasks,
                "wall_duration_s": r.wall_duration_s,
                "avg_task_duration_s": r.avg_task_duration_s,
                "time_per_task_s": r.time_per_task_s,
                "throughput_tasks_per_min": (len(r.completed_tasks) / (r.wall_duration_s / 60.0)) if r.wall_duration_s > 0 else 0.0,
                "full_run_rtf": r.compute_rtf()
            }
            for r in phase2_runs
        ]
    }
    with open(summary_file, "w") as f:
        json.dump(summary_data, f, indent=2)


def main():
    parser = argparse.ArgumentParser(
        description="Run two-phase automated optimization: Phase 1 speed profile benchmark (4 runs, 6 tasks) -> Phase 2 fleet scaling sweep (4, 5, 6 AMRs, 15 tasks)."
    )
    parser.add_argument("--phase1-tasks", type=int, default=6, help="Tasks per run in Phase 1 speed benchmark (default: 6)")
    parser.add_argument("--phase2-tasks", type=int, default=15, help="Tasks per run in Phase 2 fleet scaling sweep (default: 15)")
    parser.add_argument("--timeout", type=int, default=1500, help="Per-run timeout seconds (default: 1500s)")
    parser.add_argument("--settle-seconds", type=int, default=5, help="AMR spawn settle time seconds (default: 5s)")
    parser.add_argument("--seed", type=int, default=42, help="Base random seed (default: 42)")
    parser.add_argument("--override-winner-speed", type=str, default=None,
                        help="Optional override to force a specific speed profile for Phase 2 (e.g. physical_fidelity, physical_max, synthetic_0_75, synthetic_1_00)")
    parser.add_argument("--output-dir", type=str, default=None, help="Root directory for multi-run logs")
    parser.add_argument("--export-dataset", action="store_true", help="Auto-generate tabular ML dataset CSV on completion")

    args = parser.parse_args()

    workspace_dir = SCRIPT_DIR.parent.parent.parent
    if args.output_dir:
        base_dir = Path(args.output_dir).resolve()
    else:
        session_tag = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        base_dir = workspace_dir / "log" / f"speed_fleet_optimization_{session_tag}"
    base_dir.mkdir(parents=True, exist_ok=True)

    print(f"\n{C_BOLD}{C_CYAN}========================================================================={C_RESET}")
    print(f"{C_BOLD}{C_CYAN}   SIH DECENTRALIZED AMR FLEET — SPEED PROFILE & FLEET SCALING RUNNER    {C_RESET}")
    print(f"{C_BOLD}{C_CYAN}========================================================================={C_RESET}")
    print(f" • Phase 1 (Speed Benchmark): 4 runs × {args.phase1_tasks} tasks (Profiles: {', '.join(PHASE1_PROFILES)})")
    print(f" • Phase 2 (Fleet Scaling):   3 runs × {args.phase2_tasks} tasks (Fleet: 4, 5, 6 AMRs using Winner Speed)")
    print(f" • Total Runs:               7 sequential runs")
    print(f" • Spawn Settle Time:        {args.settle_seconds} s")
    print(f" • Timeout Per Run:          {args.timeout} s")
    print(f" • Vision Recording:         DISABLED (FLEET_ENABLE_VISION=false)")
    print(f" • Root Log Directory:       {base_dir}")
    print(f"{C_CYAN}-------------------------------------------------------------------------{C_RESET}\n")

    campaign_estimator = CampaignETAEstimator(total_runs=7, cooldown_seconds=5.0)
    phase1_runs: List[WorkCycleRun] = []
    phase2_runs: List[WorkCycleRun] = []
    current_run: Optional[WorkCycleRun] = None
    interrupted = False
    winner_prof_name: str = PHASE1_PROFILES[0]
    winner_speed_mps: float = 0.31
    winner_fleet_size: int = 4

    try:
        # ---------------------------------------------------------------------
        # PHASE 1: SPEED PROFILE BENCHMARK (Runs 1 to 4, 6 tasks each, 4 AMRs)
        # ---------------------------------------------------------------------
        print(f"\n{C_BG_BLUE}{C_WHITE}{C_BOLD} >>> STARTING PHASE 1: SPEED PROFILE BENCHMARK (4 RUNS × {args.phase1_tasks} TASKS, 4 AMRs) <<<{C_RESET}\n")
        
        for idx, prof_name in enumerate(PHASE1_PROFILES, start=1):
            prof = resolve_velocity_profile(prof_name)
            run = WorkCycleRun(
                run_index=idx,
                total_runs=7,
                target_tasks=args.phase1_tasks,
                base_dir=base_dir,
                timeout_s=args.timeout,
                campaign_estimator=campaign_estimator,
                fleet_count=4,
                scenario=f"speed_profile_{prof_name}"
            )
            run.profile_name = prof_name
            phase1_runs.append(run)
            current_run = run

            print(f"{C_BOLD}{C_YELLOW}>>> Phase 1 Run {idx}/4: Profile '{prof_name}' ({prof.nominal_speed_mps} m/s) [{prof.tier}] <<<{C_RESET}")
            success = run.execute(
                tracking_speed=prof.nominal_speed_mps,
                settle_s=args.settle_seconds,
                seed=args.seed + idx,
                enable_faults=False,
                enable_spawner=False,
                enable_vision=False,
                scenario=f"speed_{prof_name}"
            )
            if run.end_time > 0 and run.start_time > 0:
                campaign_estimator.record_completed_run(run.end_time - run.start_time)
            current_run = None

            print(f"{C_DIM}Cooldown between runs (5s)...{C_RESET}")
            time.sleep(5.0)

        # Evaluate Phase 1 Winner Speed Profile
        if args.override_winner_speed:
            winner_prof = resolve_velocity_profile(args.override_winner_speed)
            winner_prof_name = winner_prof.name
            winner_speed_mps = winner_prof.nominal_speed_mps
            p1_winner_run = phase1_runs[0]
            print(f"\n{C_BOLD}{C_GREEN}>>> Phase 1 Winner Overridden by User: {winner_prof_name} ({winner_speed_mps} m/s) <<<{C_RESET}\n")
        else:
            p1_winner_run, winner_prof_name, winner_speed_mps = evaluate_phase1_winner(phase1_runs)
            print(f"\n{C_BG_DARK}{C_GREEN}{C_BOLD} ======================== PHASE 1 WINNER DECLARED ======================== {C_RESET}")
            print(f" 🏆 {C_BOLD}WINNING SPEED PROFILE:{C_RESET} {C_GREEN}{C_BOLD}{winner_prof_name} ({winner_speed_mps} m/s){C_RESET}")
            print(f" • Average Time Per Task: {C_CYAN}{p1_winner_run.time_per_task_s:.1f}s / task{C_RESET}")
            print(f" • Average Task Duration: {C_CYAN}{p1_winner_run.avg_task_duration_s:.1f}s{C_RESET}")
            print(f" • Wall Duration:         {p1_winner_run.wall_duration_s:.1f}s for {len(p1_winner_run.completed_tasks)} tasks")
            print(f"{C_BG_DARK}{C_GREEN}{C_BOLD} ========================================================================= {C_RESET}\n")

        # ---------------------------------------------------------------------
        # PHASE 2: FLEET SIZE SCALING SWEEP (Runs 5 to 7, 15 tasks each)
        # ---------------------------------------------------------------------
        print(f"\n{C_BG_BLUE}{C_WHITE}{C_BOLD} >>> STARTING PHASE 2: FLEET SCALING SWEEP (3 RUNS × {args.phase2_tasks} TASKS, USING {winner_prof_name} @ {winner_speed_mps} m/s) <<<{C_RESET}\n")

        for i, fleet_size in enumerate(PHASE2_FLEET_SIZES, start=5):
            launcher_name = "launch_fleet_amrs.sh" if fleet_size > 4 else "launch_four_amrs.sh"
            run = WorkCycleRun(
                run_index=i,
                total_runs=7,
                target_tasks=args.phase2_tasks,
                base_dir=base_dir,
                timeout_s=args.timeout,
                campaign_estimator=campaign_estimator,
                fleet_count=fleet_size,
                launcher_override=launcher_name,
                scenario=f"fleet_scale_{fleet_size}_amrs"
            )
            run.profile_name = winner_prof_name
            phase2_runs.append(run)
            current_run = run

            print(f"{C_BOLD}{C_YELLOW}>>> Phase 2 Run {i}/7: Fleet Scaling = {fleet_size} AMRs (Speed: {winner_speed_mps} m/s) <<<{C_RESET}")
            success = run.execute(
                tracking_speed=winner_speed_mps,
                settle_s=args.settle_seconds,
                seed=args.seed + i,
                enable_faults=False,
                enable_spawner=False,
                enable_vision=False,
                scenario=f"fleet_scale_{fleet_size}_amrs"
            )
            if run.end_time > 0 and run.start_time > 0:
                campaign_estimator.record_completed_run(run.end_time - run.start_time)
            current_run = None

            if i < 7:
                print(f"{C_DIM}Cooldown between runs (5s)...{C_RESET}")
                time.sleep(5.0)

        # Evaluate Phase 2 Winner Fleet Size
        p2_winner_run, winner_fleet_size = evaluate_phase2_winner(phase2_runs)
        print(f"\n{C_BG_DARK}{C_GREEN}{C_BOLD} ======================== PHASE 2 WINNER DECLARED ======================== {C_RESET}")
        print(f" 🏆 {C_BOLD}WINNING FLEET CONFIGURATION:{C_RESET} {C_GREEN}{C_BOLD}{winner_fleet_size} AMRs{C_RESET} (using {winner_prof_name} @ {winner_speed_mps} m/s)")
        print(f" • Average Time Per Task: {C_CYAN}{p2_winner_run.time_per_task_s:.1f}s / task{C_RESET}")
        print(f" • Average Task Duration: {C_CYAN}{p2_winner_run.avg_task_duration_s:.1f}s{C_RESET}")
        print(f" • Total Wall Duration:   {p2_winner_run.wall_duration_s:.1f}s for {len(p2_winner_run.completed_tasks)} tasks")
        print(f"{C_BG_DARK}{C_GREEN}{C_BOLD} ========================================================================= {C_RESET}\n")

    except KeyboardInterrupt:
        interrupted = True
        print(f"\n\n {C_YELLOW}{C_BOLD}[!] Benchmark interrupted by user (Ctrl+C). Performing graceful shutdown...{C_RESET}")
        if current_run:
            current_run.status = "INTERRUPTED"
            current_run.teardown()
        else:
            dummy = WorkCycleRun(1, 1, 4, base_dir, 100)
            dummy.clean_lingering_processes(graceful=True)
    finally:
        if phase1_runs or phase2_runs:
            w_prof = winner_prof_name if 'winner_prof_name' in locals() else PHASE1_PROFILES[0]
            w_speed = winner_speed_mps if 'winner_speed_mps' in locals() else 0.31
            w_fleet = winner_fleet_size if 'winner_fleet_size' in locals() else 4
            generate_optimization_report(
                phase1_runs=phase1_runs,
                phase2_runs=phase2_runs,
                winner_profile_name=w_prof,
                winner_speed_mps=w_speed,
                winner_fleet_size=w_fleet,
                base_dir=base_dir
            )
            print(f"\n {C_CYAN}Optimization Markdown Report:{C_RESET} {base_dir / 'OPTIMIZATION_REPORT.md'}")
            print(f" {C_CYAN}Optimization Summary JSON:{C_RESET}    {base_dir / 'optimization_summary.json'}\n")

            if args.export_dataset:
                dataset_csv = base_dir / "optimization_fleet_dataset.csv"
                all_telemetry = [str(r.telemetry_file) for r in (phase1_runs + phase2_runs) if r.telemetry_file.exists()]
                if all_telemetry:
                    try:
                        cmd = ["python3", str(SCRIPT_DIR / "generate_ml_dataset.py"), *all_telemetry, "-o", str(dataset_csv)]
                        subprocess.run(cmd, check=True)
                        print(f" {C_GREEN}✔ Auto-exported ML Dataset CSV:{C_RESET} {dataset_csv}")
                    except Exception as e:
                        print(f" {C_RED}Failed to auto-export ML dataset: {e}{C_RESET}")
        if interrupted:
            sys.exit(130)


if __name__ == "__main__":
    main()
