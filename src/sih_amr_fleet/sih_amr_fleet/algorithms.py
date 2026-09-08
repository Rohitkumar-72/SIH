"""Pure, ROS-independent algorithms used by the coordination nodes."""
import heapq
import math


class DockLeaseTable:
    """Deterministic replicated dock claims; callers transport its events by DDS.

    The smallest ``(Lamport time, robot id, request id)`` wins a simultaneous
    request.  Entries disappear only after their advertised lease expires or
    a RELEASE/CANCEL event arrives.
    """
    def __init__(self):
        self._offers = {}  # dock -> {(robot, request): (lamport, lease)}

    def observe(self, dock_id, robot_id, request_id, lamport, lease_until, event, now):
        self.expire(now)
        key = (robot_id, request_id)
        if event in ('RELEASE', 'CANCEL'):
            self._offers.get(dock_id, {}).pop(key, None)
            return
        if lease_until > now:
            self._offers.setdefault(dock_id, {})[key] = (int(lamport), float(lease_until))

    def expire(self, now):
        for dock_id in list(self._offers):
            self._offers[dock_id] = {
                key: value for key, value in self._offers[dock_id].items()
                if value[1] > now
            }
            if not self._offers[dock_id]:
                del self._offers[dock_id]

    def owner(self, dock_id, now):
        self.expire(now)
        offers = self._offers.get(dock_id, {})
        if not offers:
            return None
        return min(offers, key=lambda key: (offers[key][0], key[0], key[1]))

    def choose(self, dock_ids, robot_id, now):
        self.expire(now)
        for dock_id in sorted(dock_ids):
            owner = self.owner(dock_id, now)
            if owner is None or owner[0] == robot_id:
                return dock_id
        return None


def approach_policy(in_approach_zone, has_permit, communication_degraded, approach_speed_mps):
    """Return a conservative speed cap for a constrained-resource mouth."""
    if not in_approach_zone:
        return math.inf
    if communication_degraded or not has_permit:
        return 0.0
    return max(0.0, float(approach_speed_mps))


def reverse_recovery_allowed(nearest_obstacle_m, reverse_distance_m,
                             margin_m, inside_protected_resource, dock_claimed):
    """A deliberately pessimistic precondition for a reverse recovery move."""
    return (not inside_protected_resource and not dock_claimed
            and nearest_obstacle_m > reverse_distance_m + margin_m)


def finite_command(linear_x, angular_z):
    """Return whether the final velocity command components are finite."""
    return math.isfinite(float(linear_x)) and math.isfinite(float(angular_z))


def directional_scan_minimum(ranges, angle_min, angle_increment, direction,
                             half_angle, range_min=0.0):
    """Return the nearest valid range inside a scan sector, or infinity.

    A stopping envelope is meaningful only in the commanded travel direction.
    Side and rear returns remain available to higher-level observability but
    must not stop a forward-moving AMR merely because it is parked beside a
    dock, shelf, or wall.
    """
    nearest = math.inf
    for index, value in enumerate(ranges):
        if not math.isfinite(value) or value < range_min:
            continue
        angle = angle_min + index * angle_increment
        difference = math.atan2(math.sin(angle - direction), math.cos(angle - direction))
        if abs(difference) <= half_angle:
            nearest = min(nearest, value)
    return nearest


def map_transform_for_anchor(anchor_pose, raw_odom_pose):
    """Return an odom->map transform that maps this raw sample to an anchor."""
    anchor_x, anchor_y, anchor_yaw = anchor_pose
    local_x, local_y, local_yaw = raw_odom_pose
    origin_yaw = anchor_yaw - local_yaw
    cosine, sine = math.cos(origin_yaw), math.sin(origin_yaw)
    return (anchor_x - cosine * local_x + sine * local_y,
            anchor_y - sine * local_x - cosine * local_y, origin_yaw)


def whca_star(start, goal, blocked, reservations, width, height, horizon,
              reservation_buffer_cells=1):
    """Return a 4-connected, time-indexed path or an empty list.

    ``reservations`` contains (x, y, time_slot) held by other robots.  At a
    matching time slot, an AMR also avoids the configured Chebyshev-radius
    buffer around each peer reservation (one cell in every direction by
    default). Edge swaps are rejected to prevent head-on passage through one
    grid edge.
    """
    start = (int(start[0]), int(start[1]))
    goal = (int(goal[0]), int(goal[1]))
    if start in blocked or goal in blocked:
        return []
    queue = [(abs(start[0] - goal[0]) + abs(start[1] - goal[1]), 0, start[0], start[1], 0)]
    parent = {}
    cost = {(start[0], start[1], 0): 0}
    while queue:
        _, g, x, y, t = heapq.heappop(queue)
        state = (x, y, t)
        if g != cost.get(state):
            continue
        if (x, y) == goal or t >= horizon:
            path = [state]
            while state in parent:
                state = parent[state]
                path.append(state)
            return list(reversed(path))
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (0, 0)):
            nx, ny, nt = x + dx, y + dy, t + 1
            candidate = (nx, ny, nt)
            if not (0 <= nx < width and 0 <= ny < height) or (nx, ny) in blocked:
                continue
            reserved_neighbourhood = any(
                (rx, ry, nt) in reservations
                for rx in range(nx - reservation_buffer_cells, nx + reservation_buffer_cells + 1)
                for ry in range(ny - reservation_buffer_cells, ny + reservation_buffer_cells + 1)
            )
            if reserved_neighbourhood or (nx, ny, t) in reservations and (x, y, nt) in reservations:
                continue
            new_g = g + 1
            if new_g < cost.get(candidate, math.inf):
                cost[candidate] = new_g
                parent[candidate] = state
                h = abs(nx - goal[0]) + abs(ny - goal[1])
                heapq.heappush(queue, (new_g + h, new_g, nx, ny, nt))
    return []


class ConstantVelocityTrack:
    """Small diagonal constant-velocity Kalman-style filter for peer tracking."""
    def __init__(self, x, y, vx=0.0, vy=0.0):
        self.x, self.y, self.vx, self.vy = x, y, vx, vy
        self.variance = 0.05

    def predict(self, dt, process_noise=0.3):
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.variance += process_noise * max(dt, 0.0)

    def update(self, x, y, vx, vy, measurement_variance=0.05):
        gain = self.variance / (self.variance + measurement_variance)
        self.x += gain * (x - self.x)
        self.y += gain * (y - self.y)
        self.vx += gain * (vx - self.vx)
        self.vy += gain * (vy - self.vy)
        self.variance *= (1.0 - gain)


def avoidance_velocity(preferred, self_xy, peers, radius, horizon, max_speed):
    """Reciprocal velocity-obstacle approximation with uncertainty inflation."""
    vx, vy = preferred
    for peer in peers:
        dx, dy = peer['x'] - self_xy[0], peer['y'] - self_xy[1]
        separation = math.hypot(dx, dy)
        effective_radius = radius + peer.get('radius_inflation', 0.0)
        if separation < 1e-6:
            vx, vy = 0.0, 0.0
            continue
        closing = ((vx - peer['vx']) * dx + (vy - peer['vy']) * dy) / separation
        predicted_distance = separation - closing * horizon
        if predicted_distance < 2.0 * effective_radius and closing > 0.0:
            push = (2.0 * effective_radius - predicted_distance) / max(horizon, 0.1)
            vx -= push * dx / separation
            vy -= push * dy / separation
    speed = math.hypot(vx, vy)
    if speed > max_speed:
        vx, vy = vx * max_speed / speed, vy * max_speed / speed
    return vx, vy
