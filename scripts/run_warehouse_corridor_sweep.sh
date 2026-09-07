#!/usr/bin/env bash
# Spawn a single inspection AMR into an already-running warehouse and sweep it.
# This script does not start or stop Gazebo. Ctrl+C stops the AMR and sends zero
# velocity; restart the clean warehouse before reusing the same robot name.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SIH_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE="${AMR_WS:-$(cd "$SIH_ROOT/../.." && pwd)}"
OVERLAY="${OVERLAY:-$WORKSPACE/install}"
ROBOT_NAME="${ROBOT_NAME:-corridor_sweep}"
WORLD_NAME="${WORLD_NAME:-default}"
MODEL="${MODEL:-lite}"
KEEP_SENSORS_SYSTEM="${KEEP_SENSORS_SYSTEM:-true}"
SPAWN_X="${SPAWN_X:--7.5}"
SPAWN_Y="${SPAWN_Y:--10.0}"
SPAWN_YAW="${SPAWN_YAW:-0.0}"
SWEEP_SPEED_MPS="${SWEEP_SPEED_MPS:-6.0}"
NARROW_SPEED_MPS="${NARROW_SPEED_MPS:-0.85}"
ROBOT_RADIUS_M="${ROBOT_RADIUS_M:-0.35}"
LOG_DIR="${LOG_DIR:-$WORKSPACE/log/corridor_sweep_$(date +%Y%m%d_%H%M%S)}"

# ROS setup scripts intentionally read optional variables that may be unset.
# Keep strict mode for this script, but do not enable nounset while importing
# those upstream setup files.
set +u
source /opt/ros/jazzy/setup.bash
source "$OVERLAY/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
export GZ_IP="${GZ_IP:-127.0.0.1}"
export ROS2CLI_USE_DAEMON=0

mkdir -p "$LOG_DIR"

fail() { echo "ERROR: $*" >&2; exit 1; }
model_exists() {
  timeout 10s gz model --list 2>/dev/null | grep -Eq "^[[:space:]]*-[[:space:]]+$ROBOT_NAME/turtlebot4[[:space:]]*$"
}
clock_available() {
  timeout 5s ros2 topic echo --once /clock >/dev/null 2>&1
}
wait_for() {
  local description="$1" timeout_s="$2"
  shift 2
  for _ in $(seq 1 "$timeout_s"); do "$@" && return 0; sleep 1; done
  fail "timed out waiting for $description; logs: $LOG_DIR"
}

timeout 10s gz model --list 2>/dev/null | grep -q 'charging_pad_1' || \
  fail 'the warehouse is not reachable; start Gazebo/warehouse first'
model_exists && fail "model $ROBOT_NAME/turtlebot4 already exists; reset Gazebo or set ROBOT_NAME"

LAUNCH_PID=""
CLOCK_BRIDGE_PID=""
cleanup() {
  local status=$?
  trap - EXIT INT TERM
  ros2 topic pub --once "/$ROBOT_NAME/cmd_vel" geometry_msgs/msg/Twist \
    '{linear: {x: 0.0}, angular: {z: 0.0}}' >/dev/null 2>&1 || true
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
  if [[ -n "${CLOCK_BRIDGE_PID:-}" ]]; then
    kill -TERM -- "-$CLOCK_BRIDGE_PID" 2>/dev/null || true
    wait "$CLOCK_BRIDGE_PID" 2>/dev/null || true
  fi
  exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

if ! clock_available; then
  echo 'No ROS /clock publisher detected; starting a local Gazebo clock bridge.'
  setsid ros2 run ros_gz_bridge parameter_bridge \
    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' >"$LOG_DIR/clock_bridge.log" 2>&1 &
  CLOCK_BRIDGE_PID=$!
  wait_for 'ROS clock bridge' 30 clock_available
fi

echo "Logs: $LOG_DIR"
echo "Spawning $ROBOT_NAME at ($SPAWN_X, $SPAWN_Y), then starting the corridor sweep..."
setsid ros2 launch sih_amr_fleet spawn_minimal_amr.launch.py \
  namespace:="$ROBOT_NAME" model:="$MODEL" world:="$WORLD_NAME" \
  x:="$SPAWN_X" y:="$SPAWN_Y" z:=0.05 yaw:="$SPAWN_YAW" \
  spawn_dock:=false keep_sensors_system:="$KEEP_SENSORS_SYSTEM" \
  >"$LOG_DIR/spawn.log" 2>&1 &
LAUNCH_PID=$!

wait_for 'AMR entity' 90 model_exists
wait_for 'AMR controller' 120 grep -Fq "[$ROBOT_NAME.diffdrive_spawner]: Configured and activated diffdrive_controller" "$LOG_DIR/spawn.log"
wait_for 'AMR odometry' 60 timeout 8s ros2 topic echo --once "/$ROBOT_NAME/odom"

echo 'Sweeping all main corridors and every YAML-validated storage-aisle centreline.'
echo 'Press Ctrl+C to stop immediately. The AMR sends a zero command on exit.'
ros2 run sih_amr_fleet corridor_sweep_node --ros-args \
  -p robot_name:="$ROBOT_NAME" -p spawn_x:="$SPAWN_X" \
  -p spawn_y:="$SPAWN_Y" -p spawn_yaw:="$SPAWN_YAW" \
  -p speed_mps:="$SWEEP_SPEED_MPS" -p narrow_speed_mps:="$NARROW_SPEED_MPS" \
  -p robot_radius_m:="$ROBOT_RADIUS_M"
