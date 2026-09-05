#!/usr/bin/env bash

# Stable one-command entry point for the four-AMR lite baseline.
source "$HOME/.config/sih_amr_poses.env"

export MODEL=lite
export RENDER_ENGINE=ogre2
export START_CHARGING=false
exec "$HOME/amr_ws/launch_four_amrs.sh"
