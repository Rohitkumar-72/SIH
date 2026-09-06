#!/usr/bin/env bash
MODEL=lite RENDER_ENGINE=ogre2 GUI_RENDER_ENGINE=ogre2 \
  exec "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/launch_four_amrs.sh"
