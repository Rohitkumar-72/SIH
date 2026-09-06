#!/usr/bin/env bash
# Start a clean warehouse, then insert four TurtleBot 4 AMRs sequentially.
# Ctrl+C from this terminal stops every process started by this script.

set -o pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SIH_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE="${AMR_WS:-$(cd "$SIH_ROOT/../.." && pwd)}"
POSE_FILE="${SIH_AMR_POSES_FILE:-$HOME/.config/sih_amr_poses.env}"
WAREHOUSE_DIR="${WAREHOUSE_DIR:-$WORKSPACE/src/warehouse_world_custom}"
OVERLAY="${OVERLAY:-$WORKSPACE/install}"
WORLD_FILE="${WORLD_FILE:-$WAREHOUSE_DIR/worlds/small_warehouse/warehouse_clean.sdf}"
WORLD_NAME="${WORLD_NAME:-default}"
MODEL="${MODEL:-lite}"
RENDER_ENGINE="${RENDER_ENGINE:-ogre2}"
GUI_RENDER_ENGINE="${GUI_RENDER_ENGINE:-ogre2}"
GUI_CONFIG="${GZ_GUI_CONFIG:-/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/gui/gui.config}"
START_GUI="${START_GUI:-true}"
START_CHARGING="${START_CHARGING:-false}"
START_FLEET="${START_FLEET:-false}"
SPAWN_WAIT_SECONDS="${SPAWN_WAIT_SECONDS:-180}"
SETTLE_SECONDS="${SETTLE_SECONDS:-20}"
RUN_ID="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${LOG_DIR:-$WORKSPACE/log/four_amr_runs/$RUN_ID}"

[[ -f "$POSE_FILE" ]] || { echo "ERROR: pose file not found: $POSE_FILE" >&2; exit 2; }
source "$POSE_FILE"
for robot in 1 2 3 4; do
  for axis in X Y YAW; do
    variable="ROBOT_${robot}_${axis}"
    [[ -n "${!variable:-}" ]] || { echo "ERROR: set $variable in $POSE_FILE" >&2; exit 2; }
  done
done
[[ -f "$WORLD_FILE" ]] || { echo "ERROR: world not found: $WORLD_FILE" >&2; exit 2; }
[[ -f "$GUI_CONFIG" ]] || { echo "ERROR: GUI config not found: $GUI_CONFIG" >&2; exit 2; }
mkdir -p "$LOG_DIR" || { echo "ERROR: cannot create $LOG_DIR" >&2; exit 1; }

source /opt/ros/jazzy/setup.bash
source "$OVERLAY/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export FASTRTPS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/fastdds_udp_only.xml"
export GZ_IP="${GZ_IP:-127.0.0.1}"
export QT_QPA_PLATFORM=xcb
export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/jazzy/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_RESOURCE_PATH="$SIH_ROOT/src/sih_amr_fleet/models:$WAREHOUSE_DIR/models:$WAREHOUSE_DIR:/opt/ros/jazzy/share"

SERVER_PID="" CLOCK_PID="" GUI_PID="" FLEET_PID="" STARTED_PID=""
declare -a ROBOT_PIDS=() CHARGING_PIDS=()

start_group() {
  local log_file="$1"
  shift
  setsid nohup "$@" >"$log_file" 2>&1 < /dev/null &
  STARTED_PID=$!
}

cleanup() {
  local status=$? pid
  trap - EXIT INT TERM
  echo 'Stopping this four-AMR run...'
  for pid in "$GUI_PID" "$FLEET_PID" "${CHARGING_PIDS[@]}" "${ROBOT_PIDS[@]}" "$CLOCK_PID" "$SERVER_PID"; do
    [[ -n "$pid" ]] && kill -TERM -- "-$pid" 2>/dev/null || true
  done
  for _ in $(seq 1 15); do
    local alive=false
    for pid in "$GUI_PID" "$FLEET_PID" "${CHARGING_PIDS[@]}" "${ROBOT_PIDS[@]}" "$CLOCK_PID" "$SERVER_PID"; do
      [[ -n "$pid" ]] && kill -0 -- "-$pid" 2>/dev/null && alive=true
    done
    [[ "$alive" == false ]] && break
    sleep 1
  done
  for pid in "$GUI_PID" "$FLEET_PID" "${CHARGING_PIDS[@]}" "${ROBOT_PIDS[@]}" "$CLOCK_PID" "$SERVER_PID"; do
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
model_exists() {
  timeout 10s gz model --list 2>/dev/null |
    grep -Eq "^[[:space:]]*-[[:space:]]+$1/turtlebot4[[:space:]]*$"
}
warehouse_ready() {
  timeout 10s gz model --list 2>/dev/null |
    grep -Eq '^[[:space:]]*-[[:space:]]+charging_pad_1[[:space:]]*$'
}
fail() { echo "ERROR: $*" >&2; echo "Logs: $LOG_DIR" >&2; exit 1; }

if pgrep -f '[g]z sim|[r]os_gz_bridge|[s]pawn_minimal_amr|[t]urtlebot4_spawn' >/dev/null; then
  fail 'A Gazebo or AMR launch is already running. Stop it before starting a clean run.'
fi

echo "Run logs: $LOG_DIR"
echo "Starting server with $RENDER_ENGINE..."
start_group "$LOG_DIR/gazebo_server.log" gz sim -s -r --render-engine "$RENDER_ENGINE" "$WORLD_FILE"
SERVER_PID="$STARTED_PID"
wait_for server 60 kill -0 "$SERVER_PID" || fail 'Gazebo server exited during startup'
wait_for warehouse 60 warehouse_ready || fail 'Gazebo did not expose the warehouse models'

start_group "$LOG_DIR/clock_bridge.log" ros2 run ros_gz_bridge parameter_bridge '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
CLOCK_PID="$STARTED_PID"
sleep 3
kill -0 "$CLOCK_PID" 2>/dev/null || fail 'Clock bridge exited'

spawn_robot() {
  local robot="$1" x="$2" y="$3" yaw="$4" keep_sensors="$5" log_file="$LOG_DIR/$1.log"
  echo "Starting $robot at x=$x y=$y yaw=$yaw..."
  start_group "$log_file" ros2 launch sih_amr_fleet spawn_minimal_amr.launch.py \
    namespace:="$robot" model:="$MODEL" world:="$WORLD_NAME" x:="$x" y:="$y" z:=0.05 yaw:="$yaw" \
    spawn_dock:=false keep_sensors_system:="$keep_sensors"
  ROBOT_PIDS+=("$STARTED_PID")
  wait_for entity "$SPAWN_WAIT_SECONDS" model_exists "$robot" || fail "$robot body was not created"
  wait_for controller 120 grep -Fq "[$robot.diffdrive_spawner]: Configured and activated diffdrive_controller" "$log_file" || fail "$robot controller did not activate"
  wait_for interfaces 90 grep -Fq "[$robot.interface_readiness]: Interface readiness passed:" "$log_file" || fail "$robot interfaces are not ready"
  echo "$robot passed all gates; settling for ${SETTLE_SECONDS}s."
  sleep "$SETTLE_SECONDS"
}

spawn_robot robot_1 "$ROBOT_1_X" "$ROBOT_1_Y" "$ROBOT_1_YAW" true
spawn_robot robot_2 "$ROBOT_2_X" "$ROBOT_2_Y" "$ROBOT_2_YAW" false
spawn_robot robot_3 "$ROBOT_3_X" "$ROBOT_3_Y" "$ROBOT_3_YAW" false
spawn_robot robot_4 "$ROBOT_4_X" "$ROBOT_4_Y" "$ROBOT_4_YAW" false

if [[ "$START_FLEET" == true ]]; then
  start_group "$LOG_DIR/fleet.log" ros2 launch sih_amr_fleet fleet.launch.py
  FLEET_PID="$STARTED_PID"
fi
if [[ "$START_GUI" == true ]]; then
  # Always use a known-good config rather than the mutable ~/.gz GUI layout.
  # A short retry recovers from a transient EGL / Qt startup failure.
  for attempt in 1 2; do
    start_group "$LOG_DIR/gazebo_gui_attempt_${attempt}.log" gz sim -g \
      --render-engine "$GUI_RENDER_ENGINE" --gui-config "$GUI_CONFIG"
    GUI_PID="$STARTED_PID"
    sleep 4
    kill -0 "$GUI_PID" 2>/dev/null && break
    echo "WARNING: Gazebo GUI exited during attempt $attempt; retrying..." >&2
    GUI_PID=""
  done
  [[ -n "$GUI_PID" ]] || fail 'Gazebo GUI could not start after two attempts'
fi

echo 'All four AMRs passed. Keep this terminal open; Ctrl+C stops the whole run.'
[[ -n "$GUI_PID" ]] && wait "$GUI_PID" || wait "$SERVER_PID"
