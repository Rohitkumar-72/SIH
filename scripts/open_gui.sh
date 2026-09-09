#!/usr/bin/env bash
# Attach a Gazebo Sim GUI client to an already-running simulation session.
# Closing this GUI window will NOT terminate the background simulation server.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SIH_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE="${AMR_WS:-$(cd "$SIH_ROOT/../.." && pwd)}"
WAREHOUSE_DIR="${WAREHOUSE_DIR:-$WORKSPACE/src/warehouse_world_custom}"
GUI_CONFIG="${GZ_GUI_CONFIG:-/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/gui/gui.config}"
RENDER_ENGINE="${GUI_RENDER_ENGINE:-ogre2}"

export GZ_IP="${GZ_IP:-127.0.0.1}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/jazzy/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_RESOURCE_PATH="$SIH_ROOT/src/sih_amr_fleet/models:$WAREHOUSE_DIR/models:$WAREHOUSE_DIR:/opt/ros/jazzy/share"

echo "Connecting Gazebo GUI to running simulation..."
exec gz sim -g --render-engine "$RENDER_ENGINE" --gui-config "$GUI_CONFIG" "$@"
