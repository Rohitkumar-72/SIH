#!/usr/bin/env bash
# One-command, repeatable baseline: Gazebo + four AMRs + full fleet stack,
# random warehouse tasks, and passive JSONL telemetry.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export START_FLEET=true
export FLEET_RANDOM_TASKS=true
export FLEET_RECORD_DATA=true
exec "$SCRIPT_DIR/launch_four_amrs.sh"
