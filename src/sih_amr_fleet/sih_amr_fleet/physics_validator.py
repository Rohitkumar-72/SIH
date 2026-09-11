"""Startup physics and controller timing invariant validator.

Enforces:
    controller_update_rate_hz <= 1.0 / max_step_size_s

Prevents gz_ros2_control error where requested controller period
is faster than simulation period, without overloading CPU by unneeded
physics step increases.
"""

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Optional, Tuple
import yaml


def parse_max_step_size_from_sdf(world_path: str) -> float:
    """Extract max_step_size in seconds from an SDF world file."""
    path = Path(world_path)
    if not path.is_file():
        raise FileNotFoundError(f"World SDF file not found: {world_path}")
    tree = ET.parse(str(path))
    root = tree.getroot()
    for physics in root.iter("physics"):
        step_elem = physics.find("max_step_size")
        if step_elem is not None and step_elem.text:
            return float(step_elem.text.strip())
    # Default Gazebo Harmonic fallback if not explicitly declared in SDF
    return 0.001


def parse_controller_update_rate_from_yaml(config_path: str) -> float:
    """Extract controller_manager update_rate in Hz from controller YAML."""
    path = Path(config_path)
    if not path.is_file():
        raise FileNotFoundError(f"Controller config YAML not found: {config_path}")
    with open(path, "r") as f:
        data = yaml.safe_load(f)

    # Check /**: controller_manager: ros__parameters: update_rate
    # or controller_manager: ros__parameters: update_rate
    def find_update_rate(d):
        if not isinstance(d, dict):
            return None
        if "update_rate" in d:
            return float(d["update_rate"])
        for k, v in d.items():
            res = find_update_rate(v)
            if res is not None:
                return res
        return None

    rate = find_update_rate(data)
    if rate is None:
        raise ValueError(f"Could not find update_rate in controller config: {config_path}")
    return rate


def validate_physics_and_controller_timing(
    max_step_size_s: float,
    controller_update_rate_hz: float,
    strict: bool = True
) -> Tuple[bool, str]:
    """Validate that controller_update_rate_hz <= 1.0 / max_step_size_s."""
    if max_step_size_s <= 0:
        msg = f"Invalid max_step_size_s: {max_step_size_s} (must be positive)"
        if strict:
            raise ValueError(msg)
        return False, msg

    if controller_update_rate_hz <= 0:
        msg = f"Invalid controller_update_rate_hz: {controller_update_rate_hz} (must be positive)"
        if strict:
            raise ValueError(msg)
        return False, msg

    effective_physics_hz = round(1.0 / max_step_size_s, 6)
    controller_period_s = 1.0 / controller_update_rate_hz

    if controller_update_rate_hz > effective_physics_hz + 1e-6:
        msg = (
            f"INVARIANT VIOLATION: Controller update rate ({controller_update_rate_hz:.1f} Hz) "
            f"exceeds effective physics frequency ({effective_physics_hz:.1f} Hz, "
            f"max_step_size={max_step_size_s:.4f}s). "
            f"Controller period ({controller_period_s:.4f}s) is faster than simulation period "
            f"({max_step_size_s:.4f}s), which causes gz_ros2_control timing failures. "
            f"Required: controller_update_rate_hz <= 1 / max_step_size_s."
        )
        if strict:
            raise ValueError(msg)
        return False, msg

    msg = (
        f"PASS: Controller update rate ({controller_update_rate_hz:.1f} Hz) <= "
        f"effective physics frequency ({effective_physics_hz:.1f} Hz, "
        f"max_step_size={max_step_size_s:.4f}s)."
    )
    return True, msg


def validate_files(world_file: str, control_config_file: str) -> bool:
    """Validate timing compatibility between world SDF and controller YAML."""
    step_s = parse_max_step_size_from_sdf(world_file)
    rate_hz = parse_controller_update_rate_from_yaml(control_config_file)
    valid, msg = validate_physics_and_controller_timing(step_s, rate_hz, strict=False)
    if not valid:
        print(f"[PhysicsValidator ERROR] {msg}", file=sys.stderr)
        return False
    print(f"[PhysicsValidator] {msg}")
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate physics and controller timing invariant.")
    parser.add_argument("--world", required=True, help="Path to world SDF file")
    parser.add_argument("--control-config", required=True, help="Path to controller YAML config")
    args = parser.parse_args()

    try:
        ok = validate_files(args.world, args.control_config)
        return 0 if ok else 1
    except Exception as e:
        print(f"[PhysicsValidator FATAL] {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
