"""Small, simulation-only planner over the current RTAB-Map occupancy grid."""
from collections import deque
import math

import numpy as np


def angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class FreeMap:
    def __init__(self, data, width, height, resolution, origin, clearance=.48):
        if not (width > 0 and height > 0 and resolution > 0
                and all(map(math.isfinite, (*origin, resolution)))):
            raise ValueError('Invalid occupancy grid metadata')
        self.r, self.origin = resolution, origin
        self.cos, self.sin = math.cos(origin[2]), math.sin(origin[2])
        # Unknown, occupied, and off-map cells are unavailable. Inflate by the
        # Scout's simulated circumscribed collision radius plus a small margin.
        cells = np.asarray(data, dtype=np.int8).reshape(height, width)
        free = cells == 0
        radius = math.ceil(clearance / resolution)
        padded = np.pad(free, radius, constant_values=False)
        self.free = free.copy()
        for y in range(-radius, radius + 1):
            for x in range(-radius, radius + 1):
                if math.hypot(x, y) * resolution <= clearance:
                    self.free &= padded[radius+y:radius+y+height, radius+x:radius+x+width]

    def cell(self, xy):
        x, y = xy[0] - self.origin[0], xy[1] - self.origin[1]
        return (math.floor((self.cos*x + self.sin*y) / self.r),
                math.floor((-self.sin*x + self.cos*y) / self.r))

    def point(self, cell):
        x, y = ((v + .5) * self.r for v in cell)
        return (self.origin[0] + self.cos*x - self.sin*y,
                self.origin[1] + self.sin*x + self.cos*y)

    def available(self, c):
        x, y = c
        h, w = self.free.shape
        return 0 <= x < w and 0 <= y < h and self.free[y, x]

    def clear(self, a, b):
        # Sample the segment at a fraction of a cell; also check both cells
        # flanking every diagonal transition to avoid slipping through corners.
        samples = max(1, math.ceil(math.dist(a, b) / (.25 * self.r)))
        prev = self.cell(a)
        for i in range(samples + 1):
            t = i / samples
            cell = self.cell((a[0] + t*(b[0]-a[0]), a[1] + t*(b[1]-a[1])))
            if (not self.available(cell) or (cell[0] != prev[0] and cell[1] != prev[1]
                    and not (self.available((cell[0], prev[1])) and self.available((prev[0], cell[1]))))):
                return False
            prev = cell
        return True

    def route(self, pose, previous_goal):
        start = self.cell(pose)
        if not self.available(start):
            return []
        parent = {start: None}
        queue = deque([(start, 0)])
        best, best_score = None, -math.inf
        while queue:
            c, steps = queue.popleft()
            p = self.point(c)
            separation = math.dist(p, pose[:2])
            if separation >= .40:
                direction = math.atan2(p[1]-pose[1], p[0]-pose[0])
                score = separation + .45 * math.cos(direction-pose[2]) - .3 * (steps*self.r-separation)
                if previous_goal is not None:
                    score += .18 * min(1., math.dist(p, previous_goal))
                if score > best_score:
                    best, best_score = c, score
            if steps*self.r >= 1.65:
                continue
            for dx, dy in ((1, 0), (0, 1), (-1, 0), (0, -1)):
                next_cell = c[0]+dx, c[1]+dy
                if next_cell not in parent and self.available(next_cell):
                    parent[next_cell] = c
                    queue.append((next_cell, steps+1))
        result = []
        while best is not None and best != start:
            result.append(self.point(best))
            best = parent[best]
        return result[::-1]

    def motion_clear(self, pose, speed, turn):
        x, y, heading = pose
        last = (x, y)
        # Check a short arc ahead of commanded motion; a turn in place still
        # requires a clear footprint. The circumscribed mask covers all yaws.
        for _ in range(6):
            heading += turn * .1
            new = x + speed*.2*math.cos(heading), y + speed*.2*math.sin(heading)
            if not self.clear(last, new):
                return False
            x, y = new
            last = new
            heading += turn * .1
        return True


class Roam:
    def __init__(self, speed):
        self.speed = speed
        self.path = []
        self.previous_goal = None
        self.retry = 0.

    @property
    def goal(self):
        return self.path[-1] if self.path else None

    def reset(self):
        self.path = []
        self.retry = 0.

    def step(self, now, pose, grid):
        if grid is None or not grid.available(grid.cell(pose)):
            self.reset()
            return 'wander_blocked', 0., 0.
        if self.path and math.dist(self.goal, pose[:2]) < .14:
            self.previous_goal = self.goal
            self.reset()
        # Route is reused, but every issued segment is rechecked against the
        # latest occupancy grid (including after visual-follow interruptions).
        if not self.path and now >= self.retry:
            self.path = grid.route(pose, self.previous_goal)
            self.retry = now + .6
        while len(self.path) > 1 and math.dist(self.path[0], pose[:2]) < .10:
            self.path.pop(0)
        target = None
        for point in self.path:
            distance = math.dist(pose[:2], point)
            if distance > .55 or not grid.clear(pose[:2], point):
                break
            if distance >= .07:
                target = point
        if target is None:
            self.path = []
            return 'wander_blocked', 0., 0.
        error = angle(math.atan2(target[1]-pose[1], target[0]-pose[0]) - pose[2])
        turn = max(-.35, min(.35, 1.65*error))
        speed = self.speed * math.cos(error) if abs(error) < .22 else 0.
        if not grid.motion_clear(pose, speed, turn):
            self.path = []
            return 'wander_blocked', 0., 0.
        return 'wandering', speed, turn
