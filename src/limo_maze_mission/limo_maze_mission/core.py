"""Conservative occupancy-grid planning and observation bookkeeping.

Inputs are only OccupancyGrid data, estimated poses and detector observations.
No simulator state or expected target count is accepted by this module.
"""
from collections import deque
from dataclasses import dataclass, field
from math import atan2, ceil, cos, hypot, isfinite, pi, sin


def angle_delta(a, b):
    return (a - b + pi) % (2 * pi) - pi


@dataclass(frozen=True)
class Grid:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    data: tuple
    occupied_threshold: int = 65

    def __post_init__(self):
        if (self.width <= 0 or self.height <= 0 or self.resolution <= 0 or
                not all(isfinite(v) for v in (self.resolution, self.origin_x, self.origin_y, self.origin_yaw)) or
                len(self.data) != self.width * self.height):
            raise ValueError('Invalid occupancy-grid geometry')

    def inside(self, p):
        return 0 <= p[0] < self.width and 0 <= p[1] < self.height

    def value(self, p):
        return self.data[p[1] * self.width + p[0]] if self.inside(p) else -1

    def free(self, p):
        # Intermediate occupancy values are conservatively blocked, not free.
        return self.value(p) == 0

    def neighbors(self, p, diagonal=False):
        offsets = ((1, 0), (0, 1), (-1, 0), (0, -1))
        if diagonal:
            offsets += ((1, 1), (1, -1), (-1, 1), (-1, -1))
        return [(p[0] + x, p[1] + y) for x, y in offsets if self.inside((p[0] + x, p[1] + y))]

    def to_world(self, p):
        x, y = (p[0] + .5) * self.resolution, (p[1] + .5) * self.resolution
        c, s = cos(self.origin_yaw), sin(self.origin_yaw)
        return self.origin_x + c*x - s*y, self.origin_y + s*x + c*y

    def to_cell(self, x, y):
        from math import floor
        c, s = cos(self.origin_yaw), sin(self.origin_yaw)
        dx, dy = x - self.origin_x, y - self.origin_y
        return floor((c*dx + s*dy)/self.resolution), floor((-s*dx + c*dy)/self.resolution)

    def component(self, start, allowed=None):
        allowed = allowed if allowed is not None else {p for p in self.cells() if self.free(p)}
        if start not in allowed:
            return {}
        distances, queue = {start: 0}, deque([start])
        while queue:
            p = queue.popleft()
            for q in self.neighbors(p):
                if q in allowed and q not in distances:
                    distances[q] = distances[p] + 1
                    queue.append(q)
        return distances

    def cells(self):
        return ((x, y) for y in range(self.height) for x in range(self.width))

    def safe_cells(self, radius):
        """Footprint disk must lie in known free cells; unknown/outside also block.

        Cell rectangles are considered, not merely their centers. This is a
        conservative raster footprint and intentionally sacrifices narrow gaps.
        """
        reach = ceil(radius / self.resolution + .5)
        spans = []
        for dy in range(-reach, reach+1):
            row = [dx for dx in range(reach+1)
                   if hypot(max(dx-.5, 0), max(abs(dy)-.5, 0))*self.resolution <= radius]
            if row:
                spans.append((dy, max(row)))
        # Row prefix sums preserve the disk/rectangle collision rule while
        # reducing each footprint check from O(radius²) to O(radius).
        blocked = []
        for y in range(self.height):
            prefix = [0]
            for value in self.data[y*self.width:(y+1)*self.width]:
                prefix.append(prefix[-1] + (value != 0))
            blocked.append(prefix)
        safe = set()
        for y in range(self.height):
            for x in range(self.width):
                if self.data[y*self.width+x] != 0:
                    continue
                for dy, half in spans:
                    row_y, left, right = y+dy, x-half, x+half+1
                    if (row_y < 0 or row_y >= self.height or left < 0 or right > self.width or
                            blocked[row_y][right] != blocked[row_y][left]):
                        break
                else:
                    safe.add((x, y))
        return safe

    def frontier(self, p):
        # Grid edges may be expanded by SLAM: outside the current map is unknown.
        return self.free(p) and any(self.value((p[0]+dx, p[1]+dy)) < 0
                                   for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)))

    def line_free(self, a, b):
        """Supercover ray; diagonal corner neighbors prevent corner peeking."""
        x, y = a
        dx, dy = b[0]-x, b[1]-y
        nx, ny = abs(dx), abs(dy)
        sx, sy = (1 if dx > 0 else -1), (1 if dy > 0 else -1)
        ix = iy = 0
        if not self.free(a):
            return False
        while ix < nx or iy < ny:
            decision = (1+2*ix)*ny - (1+2*iy)*nx
            if decision == 0:
                if not self.free((x+sx, y)) or not self.free((x, y+sy)):
                    return False
                x, y, ix, iy = x+sx, y+sy, ix+1, iy+1
            elif decision < 0:
                x, ix = x+sx, ix+1
            else:
                y, iy = y+sy, iy+1
            if not self.free((x, y)):
                return False
        return True


@dataclass(frozen=True)
class Goal:
    x: float
    y: float
    yaw: float
    kind: str
    anchor: tuple


@dataclass
class Failure:
    anchor: tuple
    kind: str
    attempts: int = 0
    last_time: float = 0.0
    reason: str = ''


class FailureBook:
    def __init__(self, radius=.45, max_attempts=2, cooldown=8.0):
        self.radius, self.max_attempts, self.cooldown = radius, max_attempts, cooldown
        self.entries = []

    def match(self, anchor, kind):
        return next((f for f in self.entries if f.kind == kind and hypot(f.anchor[0]-anchor[0], f.anchor[1]-anchor[1]) <= self.radius), None)

    def fail(self, goal, now, reason):
        entry = self.match(goal.anchor, goal.kind)
        if entry is None:
            entry = Failure(goal.anchor, goal.kind)
            self.entries.append(entry)
        entry.attempts += 1
        entry.last_time, entry.reason = now, reason

    def allowed(self, anchor, kind, now):
        entry = self.match(anchor, kind)
        return entry is None or (entry.attempts < self.max_attempts and now-entry.last_time >= self.cooldown)

    def resolve_observed(self, grid, coverage):
        """Only remove failures when new observations actually resolve the anchor."""
        unresolved = []
        for f in self.entries:
            p = grid.to_cell(*f.anchor)
            resolved = (grid.inside(p) and grid.free(p) and
                        (not grid.frontier(p) if f.kind == 'frontier' else coverage.contains(grid.to_world(p), grid.resolution)))
            if not resolved:
                unresolved.append(f)
        self.entries = unresolved

    def has_retry(self):
        return any(f.attempts < self.max_attempts for f in self.entries)


class Coverage:
    """Evidence from processed, valid camera frames; independent of laser map."""
    def __init__(self, quantization=.05):
        self.quantization = quantization
        self.points = set()
        self.frames = 0

    def key(self, point):
        return round(point[0]/self.quantization), round(point[1]/self.quantization)

    def contains(self, point, resolution):
        # Quantized world coordinates survive occupancy-grid origin expansion.
        return self.key(point) in self.points

    def observe(self, grid, camera_xy, yaw, near, far, fov):
        if not all(isfinite(x) for x in (*camera_xy, yaw, near, far, fov)) or not 0 <= near < far or not 0 < fov <= 2*pi:
            raise ValueError('Invalid camera coverage geometry')
        origin = grid.to_cell(*camera_xy)
        for p in grid.cells():
            if not grid.free(p):
                continue
            xy = grid.to_world(p)
            distance = hypot(xy[0]-camera_xy[0], xy[1]-camera_xy[1])
            bearing = atan2(xy[1]-camera_xy[1], xy[0]-camera_xy[0])
            if near <= distance <= far and abs(angle_delta(bearing, yaw)) <= fov/2 and grid.line_free(origin, p):
                self.points.add(self.key(xy))
        self.frames += 1


@dataclass
class Selection:
    state: str
    goal: Goal | None = None
    frontier_count: int = 0
    uncovered_count: int = 0
    reachable_count: int = 0


class Planner:
    def __init__(self, radius=.22, standoff=.55):
        self.radius, self.standoff = radius, standoff

    def select(self, grid, pose, coverage, failures, now, perception_ready):
        start = grid.to_cell(*pose[:2])
        raw = grid.component(start)
        safe = grid.safe_cells(self.radius)
        reachable = grid.component(start, safe)
        if not reachable:
            return Selection('unsafe_or_unlocalized')
        frontiers = {p for p in raw if grid.frontier(p)}
        uncovered = {p for p in raw if not coverage.contains(grid.to_world(p), grid.resolution)}
        failures.resolve_observed(grid, coverage)
        stats = dict(frontier_count=len(frontiers), uncovered_count=len(uncovered), reachable_count=len(raw))
        if not frontiers and not uncovered and not failures.entries and perception_ready:
            return Selection('complete', **stats)
        if not frontiers and not perception_ready:
            return Selection('perception_unverified', **stats)
        kind = 'frontier' if frontiers else 'coverage'
        anchors = frontiers if frontiers else uncovered
        reach = ceil(self.standoff/grid.resolution)
        # Greedy nearby anchor, then shortest known-grid travel within its local
        # candidate set. Do not enumerate all free cells × all standoff offsets.
        # We still inspect every anchor if earlier ones have no feasible goal,
        # so sampling cannot incorrectly produce a completion decision.
        ordered = sorted(anchors, key=lambda p: ((p[0]-start[0])**2 + (p[1]-start[1])**2, p))
        for anchor_cell in ordered:
            anchor = grid.to_world(anchor_cell)
            if not failures.allowed(anchor, kind, now):
                continue
            local = []
            for dy in range(-reach, reach+1):
                for dx in range(-reach, reach+1):
                    p = anchor_cell[0]+dx, anchor_cell[1]+dy
                    if p not in reachable or not grid.line_free(p, anchor_cell):
                        continue
                    xy = grid.to_world(p)
                    distance = hypot(xy[0]-anchor[0], xy[1]-anchor[1])
                    if distance > self.standoff or (kind == 'coverage' and distance < .20):
                        continue
                    local.append((reachable[p]*grid.resolution + .2*distance, p))
            if local:
                cost, p = min(local)
                xy = grid.to_world(p)
                yaw = atan2(anchor[1]-xy[1], anchor[0]-xy[0])
                # Avoid an unchanged-position frontier goal; orienting is still useful.
                return Selection('goal', Goal(*xy, yaw, kind, anchor), **stats)
        if failures.entries:
            return Selection('retry_wait' if failures.has_retry() else 'blocked_frontiers_or_coverage', **stats)
        return Selection('unreachable_frontiers_or_coverage', **stats)


@dataclass
class Target:
    uid: int
    class_id: str
    x: float
    y: float
    z: float
    hits: int
    first_stamp: float
    last_stamp: float
    confirmed: bool = False
    source_id: str = ''


class TargetStore:
    def __init__(self, min_hits=3, merge_radius=.30, window=3.0, min_confidence=.7):
        self.min_hits, self.merge_radius, self.window, self.min_confidence = min_hits, merge_radius, window, min_confidence
        self.targets = []
        self.next_uid = 1

    def add(self, class_id, xyz, stamp, confidence, source_id=''):
        if (not class_id or not all(isfinite(v) for v in (*xyz, stamp, confidence)) or
                not self.min_confidence <= confidence <= 1.0):
            return None
        # Spatial consistency is mandatory even for an ID. A reused ID cannot jump.
        matches = [t for t in self.targets if t.class_id == class_id and
                   (not source_id or not t.source_id or source_id == t.source_id) and
                   hypot(t.x-xyz[0], t.y-xyz[1]) <= self.merge_radius]
        target = min(matches, key=lambda t: hypot(t.x-xyz[0], t.y-xyz[1])) if matches else None
        if target is None:
            target = Target(self.next_uid, str(class_id), *xyz, 1, stamp, stamp, source_id=str(source_id))
            self.next_uid += 1
            self.targets.append(target)
        elif stamp <= target.last_stamp:
            return target  # Replayed message does not count as independent evidence.
        else:
            if not target.confirmed and stamp-target.first_stamp > self.window:
                target.hits, target.first_stamp = 0, stamp
            weight = min(target.hits, 10)
            target.x, target.y, target.z = tuple((old*weight+new)/(weight+1) for old, new in zip((target.x, target.y, target.z), xyz))
            target.hits += 1
            target.last_stamp = stamp
        target.confirmed = target.confirmed or target.hits >= self.min_hits
        return target

    def confirmed(self):
        return [t for t in self.targets if t.confirmed]
