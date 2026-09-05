#!/usr/bin/env bash

# Launch four lightweight TurtleBot 4 AMRs only after each preceding robot has
# passed its Gazebo, controller, odometry, LiDAR, and command-interface gates.
# Run this inside the Ubuntu VM. It never kills an existing simulation: an
# operator must first inspect and stop stale processes deliberately.

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-42}"
MODEL="${MODEL:-lite}"
RENDER_ENGINE="${RENDER_ENGINE:-ogre2}"
WORLD_NAME="${WORLD_NAME:-warehouse}"
SPAWN_WAIT_SECONDS="${SPAWN_WAIT_SECONDS:-180}"
SETTLE_SECONDS="${SETTLE_SECONDS:-20}"
START_CHARGING="${START_CHARGING:-true}"
START_FLEET="${START_FLEET:-false}"
RUN_ID="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${LOG_DIR:-$HOME/amr_ws/log/four_amr_runs/$RUN_ID}"
WAREHOUSE_DIR="${WAREHOUSE_DIR:-$HOME/amr_ws/src/warehouse_world_custom}"
WORLD_FILE="${WORLD_FILE:-$WAREHOUSE_DIR/worlds/small_warehouse/warehouse_clean.sdf}"
OVERLAY="${OVERLAY:-$HOME/amr_ws/install}"

# Safe parking poses are site-specific. Do not replace these required values
# with guesses: inspect warehouse_clean.sdf and supply a clear pose per AMR.
for robot in 1 2 3 4; do
  for axis in X Y YAW; do
    variable="ROBOT_${robot}_${axis}"
    [[ -n "${!variable:-}" ]] || { echo "ERROR: Set ${variable} to a verified clear warehouse pose." >&2; exit 2; }
  done
done

mkdir -p "$LOG_DIR"
source /opt/ros/jazzy/setup.bash
source "$OVERLAY/setup.bash"
set -u -o pipefail
export ROS_DOMAIN_ID="$ROS_DOMAIN_ID_VALUE"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_LOCALHOST_ONLY
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
[[ -f "$SCRIPT_DIR/fastdds_udp_only.xml" ]] && export FASTRTPS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/fastdds_udp_only.xml"
export QT_QPA_PLATFORM=xcb
export DISPLAY="${DISPLAY:-:0}"
if [[ -z "${XAUTHORITY:-}" ]]; then
  XAUTHORITY="$(find "/run/user/$(id -u)" -maxdepth 1 -name '.mutter-Xwaylandauth.*' -type f -print -quit 2>/dev/null || true)"
  export XAUTHORITY
fi
export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/jazzy/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_RESOURCE_PATH="$HOME/amr_ws/src/sih_amr_fleet/models:$WAREHOUSE_DIR/models:$WAREHOUSE_DIR:/opt/ros/jazzy/share"

SERVER_LOG="$LOG_DIR/gazebo_server.log"
CLOCK_LOG="$LOG_DIR/clock_bridge.log"
SERVER_PID=""
CLOCK_PID=""
FLEET_PID=""
declare -a ROBOT_PIDS=()
declare -a CHARGING_PIDS=()

fail() { echo "ERROR: $*" >&2; echo "Logs: $LOG_DIR" >&2; exit 1; }

cleanup_failed_run() {
  local status=$?
  # Do not re-enter this handler when it exits after an INT / TERM request.
  trap - EXIT
  if [[ "$status" -ne 0 ]]; then
    for pid in "$FLEET_PID" "$SERVER_PID" "$CLOCK_PID" "${ROBOT_PIDS[@]}" "${CHARGING_PIDS[@]}"; do
      [[ -n "$pid" ]] && kill -TERM -- "-$pid" 2>/dev/null || true
    done
  fi
  exit "$status"
}
trap cleanup_failed_run EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# Gazebo model listing is an external request and must be bounded so a stalled
# server cannot freeze `wait_for`.
wait_for() {
  local timeout="$2"; shift 2
  for _ in $(seq 1 "$timeout"); do "$@" && return 0; sleep 1; done
  return 1
}
# `gz model --list` presents models as "    - <name>", not as bare names.
# Match the full list entry so robot_1 cannot be mistaken for robot_10, while
# allowing Gazebo's display prefix.  The former bare-line match always timed
# out even after a successful entity insertion.
exact_model_exists() {
  timeout 10s gz model --list 2>/dev/null |
    grep -Eq "^[[:space:]]*-[[:space:]]+$1/turtlebot4[[:space:]]*$"
}
# The controller spawner owns the activation transaction and logs success only
# after the namespaced manager has configured and activated the controller.
# Querying `ros2 control` from another short-lived DDS participant is flaky in
# this VM under load; the following odom gate independently proves the active
# controller is publishing at runtime.
controller_is_active() {
  grep -Fq "[$1.diffdrive_spawner]: Configured and activated diffdrive_controller" "$2"
}
controller_manager_ready() {
  grep -Fq "[$1.controller_manager]: Resource Manager has been successfully initialized. Starting Controller Manager services..." "$SERVER_LOG"
}
interface_ready() {
  grep -Fq "[$1.interface_readiness]: Interface readiness passed:" "$2"
}
start_process_group() { local log_file="$1"; shift; setsid nohup "$@" >"$log_file" 2>&1 < /dev/null & echo "$!"; }

robot_gate() {
  local robot="$1" robot_log="$2"
  echo "Checking $robot interfaces..."
  kill -0 "$SERVER_PID" 2>/dev/null || fail "Gazebo server exited while starting $robot"
  wait_for entity "$SPAWN_WAIT_SECONDS" exact_model_exists "$robot" || fail "$robot body was not created; dock insertion does not count"
  wait_for manager 90 controller_manager_ready "$robot" || fail "$robot controller-manager service is unavailable"
  wait_for diffdrive 120 controller_is_active "$robot" "$robot_log" || fail "$robot diff-drive controller did not become active"
  wait_for interfaces 90 interface_ready "$robot" "$robot_log" || fail "$robot did not produce live odometry, LiDAR, and command-adapter interfaces"
  echo "$robot passed all interface gates. Settling for ${SETTLE_SECONDS}s..."
  sleep "$SETTLE_SECONDS"
}

echo "Run logs: $LOG_DIR"
echo "Model: $MODEL; world: $WORLD_FILE ($WORLD_NAME); renderer: $RENDER_ENGINE"
[[ -f "$WORLD_FILE" ]] || fail "World file not found: $WORLD_FILE"
if pgrep -af 'gz sim|ros_gz_bridge|spawn_minimal_amr|turtlebot4_spawn' | grep -v "$$" >/dev/null 2>&1; then
  fail "A Gazebo or robot launch is already running. Inspect and stop it before a clean run."
fi

echo "Starting server-only Gazebo simulation (rendering disabled for multi-AMR stability)..."
# Ogre scene creation is not needed by the headless baseline and crashes when
# the second TurtleBot is inserted in this VirtualBox guest.  Keep rendering
# out of the server process; a separate GUI may attach after baseline success.
SERVER_PID="$(start_process_group "$SERVER_LOG" gz sim -s -r "$WORLD_FILE")"
wait_for server 60 kill -0 "$SERVER_PID" || { tail -n 120 "$SERVER_LOG" >&2 || true; fail 'Gazebo server exited during startup'; }
wait_for models 60 gz model --list || fail 'Gazebo did not expose the world model list'

echo "Starting simulation clock bridge..."
CLOCK_PID="$(start_process_group "$CLOCK_LOG" ros2 run ros_gz_bridge parameter_bridge '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock')"
sleep 3
kill -0 "$CLOCK_PID" 2>/dev/null || fail "Clock bridge exited; see $CLOCK_LOG"

spawn_robot() {
  local robot="$1" x="$2" y="$3" yaw="$4" log_file="$LOG_DIR/${1}.log" pid
  echo "Starting $robot at x=$x, y=$y, yaw=$yaw..."
  pid="$(start_process_group "$log_file" ros2 launch sih_amr_fleet spawn_minimal_amr.launch.py \
    namespace:="$robot" model:="$MODEL" world:="$WORLD_NAME" x:="$x" y:="$y" z:=0.05 yaw:="$yaw" spawn_dock:=false)"
  ROBOT_PIDS+=("$pid")
  robot_gate "$robot" "$log_file"
}

spawn_robot robot_1 "$ROBOT_1_X" "$ROBOT_1_Y" "$ROBOT_1_YAW"
spawn_robot robot_2 "$ROBOT_2_X" "$ROBOT_2_Y" "$ROBOT_2_YAW"
spawn_robot robot_3 "$ROBOT_3_X" "$ROBOT_3_Y" "$ROBOT_3_YAW"
spawn_robot robot_4 "$ROBOT_4_X" "$ROBOT_4_Y" "$ROBOT_4_YAW"

start_charging() {
  local robot="$1" pad_x="$2" pad_y="$3" pad_yaw="$4" origin_x="$5" origin_y="$6" origin_yaw="$7"
  local log_file="$LOG_DIR/${robot}_charging.log" pid
  pid="$(start_process_group "$log_file" ros2 run sih_amr_fleet charging_pad_node --ros-args \
    -r __ns:=/$robot -p robot_id:=$robot -p pad_x:="$pad_x" -p pad_y:="$pad_y" -p pad_yaw:="$pad_yaw" \
    -p odom_origin_x:="$origin_x" -p odom_origin_y:="$origin_y" -p odom_origin_yaw:="$origin_yaw")"
  CHARGING_PIDS+=("$pid")
  sleep 2; kill -0 "$pid" 2>/dev/null || fail "$robot charging node exited; see $log_file"
}

if [[ "$START_CHARGING" == true ]]; then
  echo 'Starting one charging node per robot...'
  start_charging robot_1 0.513707 -9.859080 1.5708 "$ROBOT_1_X" "$ROBOT_1_Y" "$ROBOT_1_YAW"
  start_charging robot_2 -0.813955 -9.910516 1.5708 "$ROBOT_2_X" "$ROBOT_2_Y" "$ROBOT_2_YAW"
  start_charging robot_3 -2.121722 -9.907508 1.5708 "$ROBOT_3_X" "$ROBOT_3_Y" "$ROBOT_3_YAW"
  start_charging robot_4 -3.372902 -9.910568 1.5708 "$ROBOT_4_X" "$ROBOT_4_Y" "$ROBOT_4_YAW"
fi

if [[ "$START_FLEET" == true ]]; then
  FLEET_PID="$(start_process_group "$LOG_DIR/fleet.log" ros2 launch sih_amr_fleet fleet.launch.py)"
  sleep 3; kill -0 "$FLEET_PID" 2>/dev/null || fail "Fleet stack exited; see $LOG_DIR/fleet.log"
fi

echo 'All four AMRs passed the baseline gates.'
echo "Gazebo PID: $SERVER_PID; clock bridge PID: $CLOCK_PID"
echo "Robot launch PIDs: ${ROBOT_PIDS[*]}"
echo "Charging PIDs: ${CHARGING_PIDS[*]:-not started}"
echo "Logs: $LOG_DIR"
