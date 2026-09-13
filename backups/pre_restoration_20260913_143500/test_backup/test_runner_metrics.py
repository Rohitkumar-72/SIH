"""Unit tests for runner progress metrics and physics timing invariants.

Tests:
1. Pruning samples older than 60 seconds.
2. Correct RTF delta calculation.
3. Simulation-time rollback.
4. Stale telemetry.
5. ETA calibration with fewer than three completions.
6. Median completion-interval ETA.
7. Stalled-task display.
8. Durations exceeding 24 hours.
9. Estimator state resets between sequential runs.
10. Physics timing invariant validation (controller_update_rate_hz <= 1 / max_step_size_s).
"""

import json
import math
import sys
from pathlib import Path
import pytest

src_dir = Path(__file__).resolve().parent.parent
if str(src_dir) not in sys.path:
    sys.path.insert(0, str(src_dir))

from sih_amr_fleet.runner_metrics import (
    CampaignETAEstimator,
    RollingRTFEstimator,
    TaskETAEstimator,
    format_duration,
    format_local_finish_time,
)
from sih_amr_fleet.physics_validator import validate_physics_and_controller_timing
from sih_amr_fleet.velocity_profiles import (
    PROFILES,
    compute_throughput_break_even,
    get_profile,
    resolve_velocity_profile,
)


def test_pruning_samples_older_than_60_seconds():
    """Unit test for pruning samples older than 60 seconds."""
    estimator = RollingRTFEstimator(window_seconds=60.0)
    # Add samples at wall 100, 130, and 165
    estimator.add_sample(wall_monotonic=100.0, sim_time=10.0)
    estimator.add_sample(wall_monotonic=130.0, sim_time=40.0)
    estimator.add_sample(wall_monotonic=165.0, sim_time=75.0)

    # Cutoff at 165.0 is 105.0: sample at 100.0 should be pruned
    estimator.prune(current_wall_monotonic=165.0)
    assert len(estimator.samples) == 2
    assert estimator.samples[0] == (130.0, 40.0)
    assert estimator.samples[1] == (165.0, 75.0)


def test_correct_rtf_delta_calculation():
    """Unit test for correct RTF delta calculation."""
    estimator = RollingRTFEstimator(window_seconds=60.0, staleness_threshold_s=5.0)
    estimator.add_sample(wall_monotonic=100.0, sim_time=10.0)
    estimator.add_sample(wall_monotonic=160.0, sim_time=130.0)

    # Window span: d_wall = 60.0s, d_sim = 120.0s -> RTF = 2.00x
    rtf_val, status, label = estimator.get_rolling_rtf(current_wall_monotonic=160.0)
    assert status == "VALID"
    assert rtf_val == pytest.approx(2.0, rel=1e-3)
    assert "RTF(60s): 2.00x" in label


def test_simulation_time_rollback():
    """Unit test for simulation-time rollback."""
    estimator = RollingRTFEstimator(window_seconds=60.0)
    estimator.add_sample(wall_monotonic=10.0, sim_time=50.0)
    estimator.add_sample(wall_monotonic=12.0, sim_time=54.0)
    assert len(estimator.samples) == 2

    # Simulation time rollback (e.g. world reset from 54.0s to 5.0s)
    estimator.add_sample(wall_monotonic=14.0, sim_time=5.0)
    assert len(estimator.samples) == 1
    assert estimator.samples[0] == (14.0, 5.0)


def test_stale_telemetry():
    """Unit test for stale telemetry."""
    estimator = RollingRTFEstimator(window_seconds=60.0, staleness_threshold_s=5.0)
    estimator.add_sample(wall_monotonic=100.0, sim_time=10.0)
    estimator.add_sample(wall_monotonic=110.0, sim_time=20.0)

    # Query within staleness threshold (gap = 2s <= 5s)
    _, status_fresh, _ = estimator.get_rolling_rtf(current_wall_monotonic=112.0)
    assert status_fresh in ("VALID", "WARMUP")

    # Query beyond staleness threshold (gap = 8s > 5s)
    rtf_val, status_stale, label = estimator.get_rolling_rtf(current_wall_monotonic=118.0)
    assert status_stale == "STALE"
    assert rtf_val is None
    assert "STALE" in label


def test_eta_calibration_with_fewer_than_three_completions():
    """Unit test for ETA calibration with fewer than three completions."""
    estimator = TaskETAEstimator(minimum_completed_tasks=3)

    # 0 completions
    val, status, text = estimator.get_eta(current_wall_monotonic=100.0, remaining_tasks=10)
    assert status == "CALIBRATING"
    assert text == "calibrating"
    assert val is None

    # 1 completion
    estimator.record_completion(wall_monotonic=100.0)
    val, status, text = estimator.get_eta(current_wall_monotonic=110.0, remaining_tasks=9)
    assert status == "CALIBRATING"
    assert text == "calibrating"

    # 2 completions
    estimator.record_completion(wall_monotonic=120.0)
    val, status, text = estimator.get_eta(current_wall_monotonic=125.0, remaining_tasks=8)
    assert status == "CALIBRATING"
    assert text == "calibrating"


def test_median_completion_interval_eta():
    """Unit test for median completion-interval ETA."""
    estimator = TaskETAEstimator(minimum_completed_tasks=3, ewma_alpha=1.0)
    # 4 completions: intervals are (120-100)=20s, (150-120)=30s, (170-150)=20s
    # Median interval = 20.0s. Rate = 1/20 = 0.05 tasks/s.
    estimator.record_completion(wall_monotonic=100.0)
    estimator.record_completion(wall_monotonic=120.0)
    estimator.record_completion(wall_monotonic=150.0)
    estimator.record_completion(wall_monotonic=170.0)

    # Remaining tasks = 5 -> ETA = 5 * 20.0 = 100 seconds (00:01:40)
    val, status, text = estimator.get_eta(current_wall_monotonic=175.0, remaining_tasks=5)
    assert status == "VALID"
    assert val == pytest.approx(100.0)
    assert text == "00:01:40"


def test_stalled_task_display():
    """Unit test for stalled-task display."""
    estimator = TaskETAEstimator(minimum_completed_tasks=3)
    # 3 completions with 10s intervals: t=100, 110, 120. Median interval = 10.0s.
    estimator.record_completion(wall_monotonic=100.0)
    estimator.record_completion(wall_monotonic=110.0)
    estimator.record_completion(wall_monotonic=120.0)

    # Query at t=135 (gap = 15s <= 2 * 10s) -> VALID
    _, status, _ = estimator.get_eta(current_wall_monotonic=135.0, remaining_tasks=5)
    assert status == "VALID"

    # Query at t=145 (gap = 25s > 2 * 10s) -> STALLED
    raw_val, status_stalled, text = estimator.get_eta(current_wall_monotonic=145.0, remaining_tasks=5)
    assert status_stalled == "STALLED"
    assert text == "stalled/recalculating"
    assert raw_val is not None  # Preserves previous raw estimate for diagnostics


def test_durations_exceeding_24_hours():
    """Unit test for durations exceeding 24 hours."""
    # Under 24 hours: HH:MM:SS
    assert format_duration(59.0) == "00:00:59"
    assert format_duration(3665.0) == "01:01:05"
    assert format_duration(86399.0) == "23:59:59"

    # 24 hours or more: DDd HH:MM:SS
    assert format_duration(86400.0) == "1d 00:00:00"
    assert format_duration(90061.0) == "1d 01:01:01"
    assert format_duration(172800.0) == "2d 00:00:00"

    # Invalid / negative values
    assert format_duration(None) == "N/A"
    assert format_duration(-10.0) == "N/A"
    assert format_duration(float("nan")) == "N/A"
    assert format_duration(float("inf")) == "N/A"


def test_estimator_state_resets_between_sequential_runs():
    """Unit test confirming estimator state resets between sequential runs."""
    rtf_est = RollingRTFEstimator()
    eta_est = TaskETAEstimator()

    rtf_est.add_sample(10.0, 5.0)
    rtf_est.add_sample(20.0, 15.0)
    eta_est.record_completion(10.0)
    eta_est.record_completion(20.0)
    eta_est.record_completion(30.0)

    assert len(rtf_est.samples) == 2
    assert len(eta_est.completion_timestamps) == 3

    # Reset
    rtf_est.reset()
    eta_est.reset()

    assert len(rtf_est.samples) == 0
    assert len(eta_est.completion_timestamps) == 0
    assert eta_est.last_raw_estimate is None
    assert eta_est.smoothed_estimate is None


def test_physics_timing_invariant_validation():
    """Unit test confirming controller_update_rate_hz <= 1 / max_step_size_s invariant."""
    # Preferred baseline: 50 Hz controller <= 50 Hz physics (max_step_size = 0.02s)
    ok, msg = validate_physics_and_controller_timing(max_step_size_s=0.02, controller_update_rate_hz=50.0, strict=False)
    assert ok is True
    assert "PASS" in msg

    # High fidelity profile: 100 Hz controller <= 100 Hz physics (max_step_size = 0.01s)
    ok_hf, msg_hf = validate_physics_and_controller_timing(max_step_size_s=0.01, controller_update_rate_hz=100.0, strict=False)
    assert ok_hf is True
    assert "PASS" in msg_hf

    # Timing error violation: 100 Hz controller > 50 Hz physics (max_step_size = 0.02s)
    ok_err, msg_err = validate_physics_and_controller_timing(max_step_size_s=0.02, controller_update_rate_hz=100.0, strict=False)
    assert ok_err is False
    assert "INVARIANT VIOLATION" in msg_err


def test_velocity_profiles_and_break_even_rtf():
    """Unit test for velocity profile resolutions and break-even RTF formulas."""
    p_phys = resolve_velocity_profile("physical_fidelity")
    assert p_phys.nominal_speed_mps == 0.31
    assert p_phys.max_speed_mps == 0.46
    assert p_phys.tier == "PHYSICAL_FIDELITY"
    assert p_phys.certified is True
    # Displacement at 0.31 m/s over 0.02s step = 0.0062 m
    assert p_phys.displacement_per_step(0.02) == pytest.approx(0.0062)

    # Break-even RTF for 0.31 m/s from 4.0 m/s at 1.726 RTF
    # 1.726 * 4.0 / 0.31 = 22.271
    be_rtf = compute_throughput_break_even(0.31, current_speed_mps=4.0, current_rtf=1.726)
    assert be_rtf == pytest.approx(22.271, rel=1e-3)

    # Synthetic profiles
    p_075 = resolve_velocity_profile("synthetic_0_75")
    assert p_075.nominal_speed_mps == 0.75
    assert p_075.certified is False


def test_work_cycle_run_robot_status_and_spawning_events(tmp_path):
    """Test WorkCycleRun robot status tracking, formatting, and launcher line parsing."""
    import importlib.util

    runner_script = Path(__file__).resolve().parent.parent.parent.parent / "scripts" / "run_multi_work_cycles.py"
    spec = importlib.util.spec_from_file_location("run_multi_work_cycles", runner_script)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    run = mod.WorkCycleRun(run_index=1, total_runs=1, target_tasks=4, base_dir=tmp_path, timeout_s=60, fleet_count=4)
    assert len(run.robot_status) == 4
    assert "R1:" in run.format_robots_status()

    # Parse launcher line
    run.parse_launcher_line("Starting robot_1 at x=0.0 y=1.0 yaw=0.0...")
    assert run.robot_status["robot_1"] == "SPAWNING"

    run.parse_launcher_line("robot_1 passed all gates; settling for 5s.")
    assert run.robot_status["robot_1"] == "IDLE"
    assert "robot_1" in run.ready_robots

    # Parse log lines
    run.parse_log_line("[robot_2:CBBA] Decision: SUBMIT_BID for task t_101")
    assert run.robot_status["robot_2"] == "BIDDING"

    run.parse_log_line("[robot_2:TaskExecutor] Decision: ACCEPT new task t_101. pickup=(1.0, 2.0), dropoff=(3.0, 4.0)")
    assert run.robot_status["robot_2"] == "TO_PICKUP(t_101)"

    run.parse_log_line("[robot_2:TaskExecutor] Decision: ARRIVED at pickup for task t_101. dwelling 5.0s")
    assert run.robot_status["robot_2"] == "DWELL_PICKUP(t_101)"

    run.parse_log_line("[robot_2:TaskExecutor] Decision: PICKUP_DWELL_COMPLETE for task t_101")
    assert run.robot_status["robot_2"] == "TO_DROPOFF(t_101)"

    run.parse_log_line("[robot_2:TaskExecutor] Decision: ARRIVED at dropoff for task t_101. dwelling 5.0s")
    assert run.robot_status["robot_2"] == "DWELL_DROPOFF(t_101)"

    run.parse_log_line("[robot_2:TaskExecutor] Decision: DROPOFF_DWELL_COMPLETE for task t_101. Transition to COMPLETED")
    assert run.robot_status["robot_2"] == "IDLE"
    assert "t_101" in run.completed_tasks


def test_optimization_runner_evaluation_and_report_generation(tmp_path):
    """Test evaluating speed profile and fleet scaling winners and generating report."""
    import importlib.util

    opt_script = Path(__file__).resolve().parent.parent.parent.parent / "scripts" / "run_speed_and_fleet_optimization.py"
    spec = importlib.util.spec_from_file_location("run_speed_and_fleet_optimization", opt_script)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    # Mock Phase 1 runs
    p1_runs = []
    speeds = [0.31, 0.46, 0.75, 1.00]
    profiles = ["physical_fidelity", "physical_max", "synthetic_0_75", "synthetic_1_00"]
    # Let physical_max have lowest time per task
    durations = [60.0, 40.0, 45.0, 50.0]

    for idx, (p, s, d) in enumerate(zip(profiles, speeds, durations), start=1):
        r = mod.WorkCycleRun(idx, 7, 6, tmp_path, 100, fleet_count=4)
        r.profile_name = p
        r.tracking_speed = s
        r.status = "PASSED"
        r.completed_tasks = [f"task_{j}" for j in range(6)]
        r.start_time = 100.0
        r.end_time = 100.0 + d
        p1_runs.append(r)

    winner_run, winner_pname, winner_speed = mod.evaluate_phase1_winner(p1_runs)
    assert winner_pname == "physical_max"
    assert winner_speed == 0.46
    assert winner_run.time_per_task_s == pytest.approx(40.0 / 6)

    # Mock Phase 2 runs (4, 5, 6 AMRs)
    p2_runs = []
    fleet_sizes = [4, 5, 6]
    # Let 6 AMRs have lowest time per task
    p2_durations = [120.0, 100.0, 80.0]

    for idx, (f, d) in enumerate(zip(fleet_sizes, p2_durations), start=5):
        r = mod.WorkCycleRun(idx, 7, 15, tmp_path, 100, fleet_count=f)
        r.profile_name = winner_pname
        r.tracking_speed = winner_speed
        r.status = "PASSED"
        r.completed_tasks = [f"task_scale_{j}" for j in range(15)]
        r.start_time = 100.0
        r.end_time = 100.0 + d
        p2_runs.append(r)

    p2_winner_run, winner_fleet = mod.evaluate_phase2_winner(p2_runs)
    assert winner_fleet == 6
    assert p2_winner_run.time_per_task_s == pytest.approx(80.0 / 15)

    # Test report generation
    mod.generate_optimization_report(
        phase1_runs=p1_runs,
        phase2_runs=p2_runs,
        winner_profile_name=winner_pname,
        winner_speed_mps=winner_speed,
        winner_fleet_size=winner_fleet,
        base_dir=tmp_path
    )

    report_md = tmp_path / "OPTIMIZATION_REPORT.md"
    summary_json = tmp_path / "optimization_summary.json"
    assert report_md.exists()
    assert summary_json.exists()

    content = report_md.read_text()
    assert "Winner Speed Profile" in content
    assert "physical_max" in content
    assert "Winner Fleet Scaling" in content
    assert "6 AMRs" in content
    assert "FLEET_ENABLE_VISION=false" in content

    with open(summary_json) as f:
        data = json.load(f)
    assert data["winner_speed_profile"] == "physical_max"
    assert data["winner_fleet_size"] == 6
    assert len(data["phase1_speed_benchmark"]) == 4
    assert len(data["phase2_fleet_scaling"]) == 3


