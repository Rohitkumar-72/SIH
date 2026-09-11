"""Runner progress tracking metrics: rolling RTF and completion-rate ETA estimation.

Provides:
- 60-second rolling RTF with sample pruning, rollback reset, and staleness detection.
- Median completion-interval task ETA estimator with EWMA smoothing and stalled state.
- Formatted duration representation (HH:MM:SS or DDd HH:MM:SS).
- Multi-run Campaign ETA estimation.
"""

import collections
import datetime
import math
import statistics
import time
from typing import Deque, List, Optional, Tuple


def format_duration(seconds: Optional[float]) -> str:
    """Format duration into HH:MM:SS or DDd HH:MM:SS if >= 24 hours."""
    if seconds is None or math.isnan(seconds) or math.isinf(seconds) or seconds < 0:
        return "N/A"
    
    total_seconds = int(round(seconds))
    days, rem = divmod(total_seconds, 86400)
    hours, rem = divmod(rem, 3600)
    minutes, secs = divmod(rem, 60)

    if days > 0:
        return f"{days}d {hours:02d}:{minutes:02d}:{secs:02d}"
    return f"{hours:02d}:{minutes:02d}:{secs:02d}"


def format_local_finish_time(remaining_seconds: Optional[float]) -> str:
    """Calculate and format estimated local wall-clock completion time."""
    if remaining_seconds is None or math.isnan(remaining_seconds) or math.isinf(remaining_seconds) or remaining_seconds < 0:
        return "N/A"
    finish_dt = datetime.datetime.now() + datetime.timedelta(seconds=remaining_seconds)
    # Get local timezone name if available
    tz_name = time.tzname[time.daylight] if time.daylight else time.tzname[0]
    return f"{finish_dt.strftime('%H:%M:%S')} {tz_name}".strip()


class RollingRTFEstimator:
    """Calculates Real-Time Factor only from samples within recent wall-clock window (60s)."""

    def __init__(self, window_seconds: float = 60.0, staleness_threshold_s: float = 5.0):
        self.window_seconds = window_seconds
        self.staleness_threshold_s = staleness_threshold_s
        self.samples: Deque[Tuple[float, float]] = collections.deque()  # (wall_monotonic, sim_time)
        self.last_ingest_wall: float = 0.0

    def reset(self) -> None:
        """Clear all samples and reset state."""
        self.samples.clear()
        self.last_ingest_wall = 0.0

    def add_sample(self, wall_monotonic: float, sim_time: float) -> None:
        """Append sample if valid. Clears on sim rollback; prunes expired samples."""
        if math.isnan(sim_time) or math.isinf(sim_time) or sim_time <= 0:
            return

        # Check for simulation time rollback
        if self.samples:
            last_wall, last_sim = self.samples[-1]
            if sim_time < last_sim:
                # Sim rolled backward (e.g. Gazebo reset) - clear deque and begin new window
                self.samples.clear()
            elif sim_time == last_sim:
                # Same simulation step; ignore redundant sample
                return

        self.samples.append((wall_monotonic, sim_time))
        self.last_ingest_wall = wall_monotonic
        self.prune(wall_monotonic)

    def prune(self, current_wall_monotonic: float) -> None:
        """Remove samples older than current wall monotonic time minus window_seconds."""
        cutoff = current_wall_monotonic - self.window_seconds
        while self.samples and self.samples[0][0] < cutoff:
            self.samples.popleft()

    def get_rolling_rtf(self, current_wall_monotonic: float) -> Tuple[Optional[float], str, str]:
        """Compute rolling RTF over available window.
        
        Returns:
            (rtf_val, status_code, display_text)
            status_code: 'VALID', 'WARMUP', 'STALE', 'EMPTY', 'INVALID'
        """
        self.prune(current_wall_monotonic)

        # Check staleness: if no fresh telemetry within threshold
        if not self.samples:
            return None, "EMPTY", "RTF(60s): N/A"

        if (current_wall_monotonic - self.last_ingest_wall) > self.staleness_threshold_s:
            return None, "STALE", "RTF(60s): STALE"

        if len(self.samples) < 2:
            return None, "WARMUP", "RTF(warmup): calibrating"

        oldest_wall, oldest_sim = self.samples[0]
        latest_wall, latest_sim = self.samples[-1]

        d_wall = latest_wall - oldest_wall
        d_sim = latest_sim - oldest_sim

        if d_wall <= 0 or d_sim < 0:
            return None, "INVALID", "RTF(60s): N/A"

        rtf = d_sim / d_wall

        # Display warmup label until window spans approximately 60 seconds
        if d_wall < (self.window_seconds - 5.0):
            span_int = max(1, int(round(d_wall)))
            label = f"RTF({span_int}s warmup): {rtf:.2f}x"
            return rtf, "WARMUP", label
        else:
            label = f"RTF(60s): {rtf:.2f}x"
            return rtf, "VALID", label


class TaskETAEstimator:
    """Estimates remaining run time from accepted task completion throughput."""

    def __init__(
        self,
        minimum_completed_tasks: int = 3,
        recent_completion_limit: int = 10,
        ewma_alpha: float = 0.25
    ):
        self.minimum_completed_tasks = minimum_completed_tasks
        self.recent_completion_limit = recent_completion_limit
        self.ewma_alpha = ewma_alpha
        self.completion_timestamps: Deque[float] = collections.deque(maxlen=recent_completion_limit)
        self.last_raw_estimate: Optional[float] = None
        self.smoothed_estimate: Optional[float] = None
        self.last_completion_wall: Optional[float] = None

    def reset(self) -> None:
        """Reset estimator state at the start of a run."""
        self.completion_timestamps.clear()
        self.last_raw_estimate = None
        self.smoothed_estimate = None
        self.last_completion_wall = None

    def record_completion(self, wall_monotonic: float) -> None:
        """Record timestamp of an accepted task completion."""
        self.completion_timestamps.append(wall_monotonic)
        self.last_completion_wall = wall_monotonic

    def get_eta(
        self,
        current_wall_monotonic: float,
        remaining_tasks: int
    ) -> Tuple[Optional[float], str, str]:
        """Calculate estimated remaining wall seconds.
        
        Returns:
            (eta_seconds, status_code, display_text)
            status_code: 'COMPLETE', 'CALIBRATING', 'STALLED', 'VALID'
        """
        if remaining_tasks <= 0:
            return 0.0, "COMPLETE", "00:00:00"

        if len(self.completion_timestamps) < self.minimum_completed_tasks:
            return None, "CALIBRATING", "calibrating"

        # Calculate intervals between successive completions
        ts_list = list(self.completion_timestamps)
        intervals = [ts_list[i] - ts_list[i - 1] for i in range(1, len(ts_list))]
        if not intervals:
            return None, "CALIBRATING", "calibrating"

        median_interval = statistics.median(intervals)
        if median_interval <= 0:
            return None, "CALIBRATING", "calibrating"

        # Check for stalled condition: no completion for > 2 * median interval
        time_since_last = current_wall_monotonic - self.completion_timestamps[-1]
        if time_since_last > (2.0 * median_interval):
            # Preserves last estimate separately for diagnostics while displaying stalled
            return self.last_raw_estimate, "STALLED", "stalled/recalculating"

        # Fleet throughput in tasks per wall second
        fleet_rate = 1.0 / median_interval
        raw_eta = remaining_tasks / fleet_rate

        self.last_raw_estimate = raw_eta

        # Smooth successive ETA estimates with small EWMA
        if self.smoothed_estimate is None:
            self.smoothed_estimate = raw_eta
        else:
            self.smoothed_estimate = (
                self.ewma_alpha * raw_eta + (1.0 - self.ewma_alpha) * self.smoothed_estimate
            )

        display_text = format_duration(self.smoothed_estimate)
        return self.smoothed_estimate, "VALID", display_text


class CampaignETAEstimator:
    """Estimates overall campaign completion time across sequential multi-run tests."""

    def __init__(self, total_runs: int, cooldown_seconds: float = 5.0):
        self.total_runs = total_runs
        self.cooldown_seconds = cooldown_seconds
        self.completed_run_durations: List[float] = []

    def record_completed_run(self, wall_duration_s: float) -> None:
        """Record completed run wall duration."""
        if wall_duration_s > 0:
            self.completed_run_durations.append(wall_duration_s)

    def get_campaign_eta(
        self,
        current_run_index: int,
        current_run_eta_s: Optional[float]
    ) -> Tuple[Optional[float], str]:
        """Compute estimated campaign remaining duration.
        
        Returns:
            (campaign_eta_s, display_text)
        """
        remaining_runs_after_current = max(0, self.total_runs - current_run_index)
        
        if not self.completed_run_durations and current_run_eta_s is None:
            return None, "calibrating"

        avg_duration = (
            sum(self.completed_run_durations) / len(self.completed_run_durations)
            if self.completed_run_durations else (current_run_eta_s or 0.0)
        )

        curr_eta = current_run_eta_s if current_run_eta_s is not None else avg_duration
        remaining_time = (
            curr_eta
            + (remaining_runs_after_current * avg_duration)
            + (remaining_runs_after_current * self.cooldown_seconds)
        )
        return remaining_time, format_duration(remaining_time)
