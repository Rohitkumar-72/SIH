#!/usr/bin/env python3
"""Spawn a test obstacle (tote/pallet/clutter) in Gazebo at a safe random or specified coordinate."""

import argparse
import os
import random
import subprocess
import sys
import time
from pathlib import Path


# Safe free-space candidate zones in warehouse coordinates
FREE_ZONES = [
    {"name": "South-West Open Floor", "x_range": (-18.0, -12.0), "y_range": (-22.0, -14.0)},
    {"name": "South-East Open Floor", "x_range": (12.0, 18.0), "y_range": (-22.0, -14.0)},
    {"name": "North-West Open Floor", "x_range": (-18.0, -12.0), "y_range": (14.0, 22.0)},
    {"name": "North-East Open Floor", "x_range": (12.0, 18.0), "y_range": (14.0, 22.0)},
    {"name": "Central Crossroad Perimeter", "x_range": (-4.0, 4.0), "y_range": (-4.0, 4.0)},
]

MODEL_PRESETS = {
    "pallet_boxes": "/home/rtsws/amr_ws/src/warehouse_world_custom/models/aws_robomaker_warehouse_ClutteringA_01/model.sdf",
    "clutter_c": "/home/rtsws/amr_ws/src/warehouse_world_custom/models/aws_robomaker_warehouse_ClutteringC_01/model.sdf",
    "clutter_d": "/home/rtsws/amr_ws/src/warehouse_world_custom/models/aws_robomaker_warehouse_ClutteringD_01/model.sdf",
    "bucket": "/home/rtsws/amr_ws/src/warehouse_world_custom/models/aws_robomaker_warehouse_Bucket_01/model.sdf",
    "trash_can": "/home/rtsws/amr_ws/src/warehouse_world_custom/models/aws_robomaker_warehouse_TrashCanC_01/model.sdf",
}


def main():
    parser = argparse.ArgumentParser(description="Spawn a test obstacle in Gazebo.")
    parser.add_argument("-x", type=float, default=None, help="X coordinate in map frame")
    parser.add_argument("-y", type=float, default=None, help="Y coordinate in map frame")
    parser.add_argument("-z", type=float, default=0.05, help="Z coordinate in map frame (default: 0.05)")
    parser.add_argument("--model", choices=list(MODEL_PRESETS.keys()), default="pallet_boxes",
                        help="Obstacle model type to spawn (default: pallet_boxes)")
    parser.add_argument("--name", type=str, default=None, help="Unique entity name in Gazebo")
    args = parser.parse_args()

    # Pick random location if not provided
    if args.x is None or args.y is None:
        zone = random.choice(FREE_ZONES)
        x = round(random.uniform(*zone["x_range"]), 2)
        y = round(random.uniform(*zone["y_range"]), 2)
        zone_name = zone["name"]
    else:
        x, y = args.x, args.y
        zone_name = "User-Specified"

    obs_name = args.name or f"test_obstacle_{int(time.time())}"
    model_file = MODEL_PRESETS[args.model]

    print(f"\n========================================================")
    print(f" Spawning Obstacle into Gazebo:")
    print(f" • Name:     {obs_name}")
    print(f" • Model:    {args.model} ({Path(model_file).name})")
    print(f" • Location: x = {x:.2f} m, y = {y:.2f} m, z = {args.z:.2f} m")
    print(f" • Region:   {zone_name}")
    print(f"========================================================\n")

    cmd = [
        "ros2", "run", "ros_gz_sim", "create",
        "-world", "default",
        "-file", model_file,
        "-name", obs_name,
        "-x", str(x),
        "-y", str(y),
        "-z", str(args.z),
        "-allow_renaming", "true"
    ]

    try:
        res = subprocess.run(cmd, capture_output=True, text=True, check=True)
        print(f"✔ Successfully spawned obstacle '{obs_name}' at ({x:.2f}, {y:.2f})!")
        print(f"Output: {res.stdout.strip()}")
    except subprocess.CalledProcessError as e:
        print(f"✗ Failed to spawn obstacle: {e.stderr.strip()}")
        sys.exit(1)


if __name__ == "__main__":
    main()
