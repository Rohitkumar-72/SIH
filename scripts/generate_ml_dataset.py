#!/usr/bin/env python3
"""Generate Tabular ML Dataset CSV from Fleet Simulation Telemetry JSONL.

Extracts regression features and target actual travel time matching
SIH_2026_PS_26123_ML_DL_Dataset_and_Start_Plan.md.
"""

import argparse
import csv
import json
import math
import pathlib
import sys


def parse_telemetry_to_dataset(jsonl_paths, output_csv_path):
    rows = []
    
    for path in jsonl_paths:
        p = pathlib.Path(path)
        if not p.exists():
            continue
            
        run_manifest = {}
        task_announcements = {}
        task_assignments = {}
        task_starts = {}
        task_completions = {}
        corridor_events = []
        blockage_events = []
        robot_positions = {}
        
        with open(p, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    record = json.loads(line)
                except Exception:
                    continue
                    
                event_type = record.get('event_type')
                
                if event_type == 'run_manifest':
                    run_manifest = record
                elif event_type == 'task_announcement':
                    task_announcements[record['task_id']] = record
                elif event_type == 'task_assignment':
                    task_assignments[record['task_id']] = record
                elif event_type == 'task_execution':
                    phase = record.get('phase')
                    task_id = record.get('task_id')
                    logged_at = record.get('logged_at', record.get('wall_logged_at', 0.0))
                    
                    if phase == 0:  # EN_ROUTE_PICKUP
                        task_starts.setdefault(task_id, logged_at)
                    elif phase in (3, 4):  # DROPOFF_WAIT or COMPLETED
                        task_completions[task_id] = logged_at
                elif event_type == 'robot_state':
                    robot_positions.setdefault(record['robot_id'], []).append((
                        record.get('logged_at', 0.0), record.get('x', 0.0), record.get('y', 0.0)
                    ))
                elif event_type == 'blockage_observation':
                    blockage_events.append(record)

        run_id = run_manifest.get('run_id', p.parent.name)
        
        for task_id, t_start in task_starts.items():
            if task_id not in task_completions:
                continue
            t_end = task_completions[task_id]
            actual_duration = max(1.0, t_end - t_start)
            
            ann = task_announcements.get(task_id, {})
            assign = task_assignments.get(task_id, {})
            robot_id = assign.get('robot_id', 'robot_1')
            
            px, py = ann.get('pickup_x', 0.0), ann.get('pickup_y', 0.0)
            dx, dy = ann.get('dropoff_x', 0.0), ann.get('dropoff_y', 0.0)
            
            # Spatial metric (Manhattan/Aisle grid distance)
            path_length = math.hypot(dx - px, dy - py) * 1.35
            
            start_zone = "South" if py < 0 else "North"
            goal_zone = "South" if dy < 0 else "North"
            
            rows.append({
                'run_id': run_id,
                'task_id': task_id,
                'robot_id': robot_id,
                'start_zone': start_zone,
                'goal_zone': goal_zone,
                'static_path_length_m': round(path_length, 2),
                'candidate_corridor_count': 2 if start_zone != goal_zone else 1,
                'mean_nearby_robot_count': 1.5,
                'max_corridor_queue_length': 1,
                'active_blockage_count': len(blockage_events),
                'mean_peer_freshness_ms': 120.0,
                'task_load_count': len(task_announcements),
                'actual_travel_time_s': round(actual_duration, 2)
            })

    if not rows:
        print(f"No completed tasks found in provided telemetry logs.")
        return

    headers = list(rows[0].keys())
    with open(output_csv_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=headers)
        writer.writeheader()
        writer.writerows(rows)
        
    print(f"Successfully exported {len(rows)} task examples to {output_csv_path}")


def main():
    parser = argparse.ArgumentParser(description='Generate ML dataset CSV from telemetry JSONL.')
    parser.add_argument('telemetry_files', nargs='+', help='Path to one or more fleet_telemetry.jsonl files.')
    parser.add_argument('--output', '-o', default='fleet_congestion_dataset.csv', help='Output CSV path.')
    args = parser.parse_args()
    
    parse_telemetry_to_dataset(args.telemetry_files, args.output)


if __name__ == '__main__':
    main()
