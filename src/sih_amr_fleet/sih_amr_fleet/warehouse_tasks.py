"""Predefined, shelf-centred lanes for the locked 45 m by 60 m warehouse."""

from dataclasses import dataclass
import math
import os
from pathlib import Path

import yaml

from geometry_msgs.msg import Pose2D


# These are the locked shelf-centre coordinates from warehouse_layout.lock.yaml.
X_ZONES = {
    'west': (-19.99, -15.67, -11.35),
    'center': (-2.5363, 2.5363),
    'east': (11.35, 15.67, 19.99),
}
Y_ZONES = {
    'south': (-28.79, -26.755, -24.72, -22.685, -20.65, -18.615, -16.58, -14.545, -12.51),
    'middle': (-7.1225, -5.0875, -3.0525, -1.0175, 1.0175, 3.0525, 5.0875, 7.1225),
    'north': (12.51, 14.545, 16.58, 18.615, 20.65, 22.685, 24.72, 26.755, 28.79),
}
LAYOUT_FILE = Path(os.environ.get(
    'SIH_WAREHOUSE_LAYOUT',
    '/home/rtsws/amr_ws/src/warehouse_world_custom/worlds/small_warehouse/warehouse_layout.lock.yaml'))


@dataclass(frozen=True)
class Lane:
    """A directed geometric centreline.  Narrow lanes are one-AMR lanes."""
    lane_id: str
    kind: str
    start: tuple[float, float]
    end: tuple[float, float]


MAIN_LANES = (
    Lane('MC-NS-W', 'main', (-7.5, -29.0), (-7.5, 29.0)),
    Lane('MC-NS-E', 'main', (7.5, -29.0), (7.5, 29.0)),
    Lane('MC-EW-S', 'main', (-22.0, -10.0), (22.0, -10.0)),
    Lane('MC-EW-N', 'main', (-22.0, 10.0), (22.0, 10.0)),
)


def _validate_clearance(lanes, robot_radius_m):
    """Validate each generated centreline against locked shelf bounding boxes."""
    data = yaml.safe_load(LAYOUT_FILE.read_text())
    shelves = [item['bounding_box_2d_m'] for item in data['objects']
               if item['type'] == 'storage_shelf']
    required_width = 2.0 * robot_radius_m + 0.10
    for lane in lanes:
        horizontal = abs(lane.start[1] - lane.end[1]) < 1e-6
        axis = 0 if horizontal else 1
        fixed = lane.start[1] if horizontal else lane.start[0]
        begin, end = sorted((lane.start[axis], lane.end[axis]))

        # Check every interval delineated by a shelf edge, rather than one
        # arbitrary midpoint.  An interval with no shelf on either side is an
        # open/main-corridor connection and needs no narrow-aisle constraint.
        edges = {begin, end}
        for box in shelves:
            edges.update((box['min_x'], box['max_x']) if horizontal
                         else (box['min_y'], box['max_y']))
        edges = sorted(value for value in edges if begin <= value <= end)
        checked = 0
        for lower, upper in zip(edges, edges[1:]):
            sample = (lower + upper) / 2.0
            if horizontal:
                below = [box['max_y'] for box in shelves
                         if box['min_x'] <= sample <= box['max_x'] and box['max_y'] < fixed]
                above = [box['min_y'] for box in shelves
                         if box['min_x'] <= sample <= box['max_x'] and box['min_y'] > fixed]
            else:
                below = [box['max_x'] for box in shelves
                         if box['min_y'] <= sample <= box['max_y'] and box['max_x'] < fixed]
                above = [box['min_x'] for box in shelves
                         if box['min_y'] <= sample <= box['max_y'] and box['min_x'] > fixed]
            if not below or not above:
                continue
            checked += 1
            clearance = min(above) - max(below)
            if clearance < required_width:
                raise RuntimeError(
                    f'{lane.lane_id} clearance {clearance:.4f} m is too small for '
                    f'a {2.0 * robot_radius_m:.2f} m robot envelope')
        if not checked:
            raise RuntimeError(f'{lane.lane_id} could not be verified against shelf bounding boxes')


def narrow_lanes(robot_radius_m=0.35):
    """Return every AMR lane centred between adjacent shelf rows.

    West and east storage blocks have horizontal aisle centrelines.  The two
    centre storage blocks additionally have their deliberately widened vertical
    centreline at x=0.  These are the only paths a task or sweep should use
    inside storage; shelf-gap shortcuts are deliberately not represented.
    """
    lanes = []
    for zone_name, rows in Y_ZONES.items():
        for side, main_x, far_x in (('west', -7.5, -21.5), ('east', 7.5, 21.5)):
            for index, (lower, upper) in enumerate(zip(rows, rows[1:]), start=1):
                y = round((lower + upper) / 2.0, 4)
                lanes.append(Lane(
                    f'NC-{zone_name.upper()}-{side.upper()}-{index:02d}',
                    'narrow', (main_x, y), (far_x, y)))
    # The centre/south block is intentionally empty for charging.
    lanes.extend((
        Lane('NC-MIDDLE-CENTRE', 'narrow', (0.0, -7.8), (0.0, 7.8)),
        Lane('NC-NORTH-CENTRE', 'narrow', (0.0, 12.2), (0.0, 29.0)),
    ))
    _validate_clearance(lanes, robot_radius_m)
    return tuple(lanes)


def aisle_points():
    """Return task points exactly on the predefined narrow-lane centrelines.

    The southern centre grid is intentionally omitted because it is the clear
    charging bay.  A point is only emitted where shelves exist on both sides of
    that narrow aisle; no task endpoint falls on a shelf or a green main corridor.
    """
    points = []
    for y_zone, rows in Y_ZONES.items():
        for x_zone, columns in X_ZONES.items():
            if y_zone == 'south' and x_zone == 'center':
                continue
            for lower, upper in zip(rows, rows[1:]):
                aisle_y = (lower + upper) / 2.0
                for shelf_x in columns:
                    points.append(Pose2D(x=shelf_x, y=aisle_y, theta=0.0))
    # The middle and north centre blocks have a vertical lane between their
    # two shelf columns.  Its task positions are also exactly on x=0.
    for y_zone in ('middle', 'north'):
        for shelf_y in Y_ZONES[y_zone]:
            points.append(Pose2D(x=0.0, y=shelf_y, theta=math.pi / 2.0))
    return points
