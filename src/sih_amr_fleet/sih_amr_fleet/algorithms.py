"""Pure, ROS-independent algorithms used by the coordination nodes."""
import heapq
import math


def whca_star(start, goal, blocked, reservations, width, height, horizon):
    """Return a 4-connected, time-indexed path or an empty list.

    ``reservations`` contains (x, y, time_slot) held by other robots.  Edge swaps
    are also rejected to prevent head-on passage through one grid edge.
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
            if candidate in reservations or (nx, ny, t) in reservations and (x, y, nt) in reservations:
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
