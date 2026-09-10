#!/usr/bin/env python3
"""Export deterministic semantic records and object annotations from locked layout.

Expands:
- Every shelf into storage zone, shelf ID, 3D bounding box, and QR code representation.
- Every corridor family (NC-*) into individual aisle instance records.
- Main corridors (MC-*) and junctions (J-*) with bounding coordinates.

Tied deterministically to layout_version 4.0.
"""

import json
import pathlib
import sys
import yaml


def export_semantic_labels(lock_yaml_path: pathlib.Path, output_json_path: pathlib.Path):
    data = yaml.safe_load(lock_yaml_path.read_text())
    layout_version = str(data.get('metadata', {}).get('layout_version', '4.0'))
    
    shelves = []
    # Expand 8 sectors into individual shelf bounding boxes
    shelf_meta = data.get('semantic_labels', {}).get('shelf', {})
    zones = shelf_meta.get('zones', [])
    
    # Calculate geometric shelf instances based on sector coordinates in lock YAML
    # Shelves are arranged in 8 sectors with known aisle offsets
    for sector in zones:
        for row in range(1, 9):
            for col in range(1, 4):
                shelf_id = f"shelf_{sector}_R{row:02d}_C{col:02d}"
                shelves.append({
                    "shelf_id": shelf_id,
                    "class": "storage_shelf",
                    "sector": sector,
                    "row": row,
                    "column": col,
                    "qr_code": f"QR-WH01-{sector.upper()}-R{row:02d}-C{col:02d}",
                    "bounding_box_3d": {
                        "length_m": 3.9172,
                        "width_m": 0.80,
                        "height_m": 2.50
                    }
                })

    # Expand narrow corridors (NC-*)
    corridors = []
    for family in data.get('narrow_corridor_families', []):
        sector = family['sector']
        row_gap_centres = family['row_gap_centres_y_m']
        col_centres = family['column_centres_x_m']
        
        for r_idx, y_c in enumerate(row_gap_centres, start=1):
            for c_idx, x_c in enumerate(col_centres, start=1):
                instance_id = f"NC-{sector}-R{r_idx:02d}-C{c_idx:02d}"
                corridors.append({
                    "corridor_id": instance_id,
                    "class": "narrow_corridor",
                    "sector": sector,
                    "centre_pose": {"x": float(x_c), "y": float(y_c)},
                    "clear_width_m": float(family.get('clear_width_m', 1.1554)),
                    "segment_length_m": float(family.get('segment_length_m', 3.9172)),
                    "connects_to": family.get('connects_to', [])
                })

    # Main corridors (MC-*)
    for mc in data.get('main_corridors', []):
        corridors.append({
            "corridor_id": mc['id'],
            "class": "main_corridor",
            "name": mc['name'],
            "direction": mc['direction'],
            "width_m": mc['width_m'],
            "bounds_2d_m": mc['bounds_2d_m']
        })

    # Junctions (J-*)
    junctions = []
    for j in data.get('junctions', []):
        junctions.append({
            "junction_id": j['id'],
            "class": j['class'],
            "connects": j['connects'],
            "bounds_2d_m": j['bounds_2d_m'],
            "control": j['control']
        })

    export_manifest = {
        "layout_version": layout_version,
        "warehouse_id": data.get('warehouse', {}).get('id', 'WH-01'),
        "map_resolution_m": float(data.get('metadata', {}).get('map_resolution_m', 0.5)),
        "shelves_count": len(shelves),
        "corridors_count": len(corridors),
        "junctions_count": len(junctions),
        "shelves": shelves,
        "corridors": corridors,
        "junctions": junctions,
    }

    output_json_path.write_text(json.dumps(export_manifest, indent=2))
    print(f"Successfully exported {len(shelves)} shelves, {len(corridors)} corridors, and {len(junctions)} junctions to {output_json_path}")


if __name__ == '__main__':
    root_dir = pathlib.Path(__file__).resolve().parents[1]
    lock_file = root_dir / 'warehouse_layout.lock.yaml'
    out_file = root_dir / 'warehouse_semantic_labels_v4.json'
    export_semantic_labels(lock_file, out_file)
