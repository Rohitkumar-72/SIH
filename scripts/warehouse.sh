#!/usr/bin/env bash
# Managed warehouse-only Gazebo session. Closing the GUI or pressing Ctrl+C
# stops the server as well, so no stale world remains on Gazebo Transport.

set -o pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SIH_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE="${AMR_WS:-$(cd "$SIH_ROOT/../.." && pwd)}"
WAREHOUSE_DIR="${WAREHOUSE_DIR:-$WORKSPACE/src/warehouse_world_custom}"
WORLD_FILE="${WORLD_FILE:-$WAREHOUSE_DIR/worlds/small_warehouse/warehouse_clean.sdf}"
GUI_CONFIG="${GZ_GUI_CONFIG:-/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/gui/gui.config}"
RENDER_ENGINE="${RENDER_ENGINE:-ogre2}"
RUN_ID="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${LOG_DIR:-$WORKSPACE/log/warehouse_runs/$RUN_ID}"

[[ -f "$WORLD_FILE" ]] || { echo "ERROR: world not found: $WORLD_FILE" >&2; exit 2; }
[[ -f "$GUI_CONFIG" ]] || { echo "ERROR: GUI config not found: $GUI_CONFIG" >&2; exit 2; }
mkdir -p "$LOG_DIR" || { echo "ERROR: cannot create $LOG_DIR" >&2; exit 1; }
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export GZ_IP="${GZ_IP:-127.0.0.1}"
export QT_QPA_PLATFORM=xcb
export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/jazzy/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_RESOURCE_PATH="$SIH_ROOT/src/sih_amr_fleet/models:$WAREHOUSE_DIR/models:$WAREHOUSE_DIR:/opt/ros/jazzy/share"

SERVER_PID="" GUI_PID="" STARTED_PID=""
start_group() {
  local log_file="$1"
  shift
  setsid nohup "$@" >"$log_file" 2>&1 < /dev/null &
  STARTED_PID=$!
}
cleanup() {
  local status=$? pid
  trap - EXIT INT TERM
  echo 'Stopping warehouse session...'
  for pid in "$GUI_PID" "$SERVER_PID"; do
    [[ -n "$pid" ]] && kill -TERM -- "-$pid" 2>/dev/null || true
  done
  for _ in $(seq 1 10); do
    local alive=false
    for pid in "$GUI_PID" "$SERVER_PID"; do
      [[ -n "$pid" ]] && kill -0 -- "-$pid" 2>/dev/null && alive=true
    done
    [[ "$alive" == false ]] && break
    sleep 1
  done
  for pid in "$GUI_PID" "$SERVER_PID"; do
    [[ -n "$pid" ]] && kill -KILL -- "-$pid" 2>/dev/null || true
  done
  exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
wait_for() {
  local timeout="$2"
  shift 2
  for _ in $(seq 1 "$timeout"); do "$@" && return 0; sleep 1; done
  return 1
}
warehouse_ready() {
  timeout 10s gz model --list 2>/dev/null |
    grep -Eq '^[[:space:]]*-[[:space:]]+charging_pad_1[[:space:]]*$'
}

if running_processes="$(pgrep -af '[g]z sim')"; then
  echo 'ERROR: A Gazebo simulator is already running. Stop it before starting a clean warehouse:' >&2
  echo "$running_processes" >&2
  exit 1
fi
echo "Warehouse logs: $LOG_DIR"
start_group "$LOG_DIR/server.log" gz sim -s -r --render-engine "$RENDER_ENGINE" "$WORLD_FILE"
SERVER_PID="$STARTED_PID"
wait_for server 60 kill -0 "$SERVER_PID" || { tail -n 80 "$LOG_DIR/server.log" >&2; exit 1; }
wait_for world 60 warehouse_ready || { tail -n 80 "$LOG_DIR/server.log" >&2; exit 1; }

# Do not load ~/.gz/sim/8/gui.config: its mutable layout can leave the 3D card
# hidden after an interrupted GUI session. The packaged config is clean.
for attempt in 1 2; do
  start_group "$LOG_DIR/gui_attempt_${attempt}.log" gz sim -g --render-engine "$RENDER_ENGINE" --gui-config "$GUI_CONFIG"
  GUI_PID="$STARTED_PID"
  sleep 4
  kill -0 "$GUI_PID" 2>/dev/null && break
  echo "WARNING: Gazebo GUI exited during attempt $attempt; retrying with the clean GUI config..." >&2
  GUI_PID=""
done
[[ -n "$GUI_PID" ]] || { echo 'ERROR: Gazebo GUI could not start; see logs above.' >&2; exit 1; }
echo 'Warehouse ready. Keep this terminal open; close the GUI or press Ctrl+C to stop both GUI and server.'
wait "$GUI_PID"
