#!/usr/bin/env bash
# One-command execution of 5 sequential decentralized warehouse work-cycle tests.
# Each test completes 4 randomized AMR tasks, shows live terminal stage updates,
# maximizes GPU/CPU throughput, and stores organized logs and benchmark metrics.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONUNBUFFERED=1
chmod +x "$SCRIPT_DIR/run_multi_work_cycles.py"

exec python3 "$SCRIPT_DIR/run_multi_work_cycles.py" \
  --runs 5 \
  --tasks-per-cycle 4 \
  --tracking-speed 4.0 \
  --settle-seconds 5 \
  "$@"
