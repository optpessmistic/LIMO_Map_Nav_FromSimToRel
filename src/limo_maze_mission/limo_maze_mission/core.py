"""Conservative occupancy-grid planning and observation bookkeeping.

Inputs are only OccupancyGrid data, estimated poses and detector observations.
No simulator state or expected target count is accepted by this module.
Explicit, independently verified setup priors may create a separate planning
copy; they are not sensor observations or coverage evidence.
"""
from collections import deque
from dataclasses import dataclass, field, replace
from math import atan2, ceil, cos, hypot, isfinite, pi, sin
import hashlib
import json


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

    def with_free_prior(self, center_xy, radius):
        """Return (planning copy, filled count) using a verified free start disk.

        The caller must independently establish this setup prior. It is not a
        lidar observation and must not be credited as observed map/vision
        coverage. Only unknown (-1) cells wholly inside the disk are filled;
        measured values, map geometry and this original Grid remain unchanged.
        No out-of-map area is created. Boundary corners are included (<=).
        """
        try:
            cx, cy = center_xy
            if (any(isinstance(v, bool) for v in (cx, cy, radius)) or
                    not all(isfinite(v) for v in (cx, cy, radius)) or radius <= 0):
                raise ValueError('Invalid prior geometry')
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError('Prior center must contain two finite coordinates and radius must be finite and positive') from exc
        # Rotation preserves a disk. Work in the grid's unrotated local frame,
        # so every cell's four corners can be checked against the same center.
        c, s = cos(self.origin_yaw), sin(self.origin_yaw)
        dx, dy = cx-self.origin_x, cy-self.origin_y
        gx, gy = c*dx+s*dy, -s*dx+c*dy
        values, filled = list(self.data), 0
        for x, y in self.cells():
            index = y*self.width+x
            if values[index] != -1:
                continue
            if all(hypot(px*self.resolution-gx, py*self.resolution-gy) <= radius
                   for px, py in ((x, y), (x+1, y), (x, y+1), (x+1, y+1))):
                values[index], filled = 0, filled+1
        return replace(self, data=tuple(values)), filled

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
    unobserved_count: int = 0


class Planner:
    def __init__(self, radius=.25, standoff=.55, frontier_min_translation_m=.20,
                 goal_arrival_margin_m=.05, frontier_fallback_standoff_m=.8):
        if not isfinite(frontier_min_translation_m) or frontier_min_translation_m <= 0:
            raise ValueError('frontier_min_translation_m must be finite and positive')
        if (any(isinstance(v, bool) or not isinstance(v, (int, float)) or not isfinite(v)
                for v in (standoff, frontier_fallback_standoff_m)) or
                not 0 < standoff <= frontier_fallback_standoff_m <= 1.0):
            raise ValueError('frontier_fallback_standoff_m must satisfy 0 < standoff <= fallback <= 1.0 m')
        self.radius, self.standoff = radius, standoff
        # One bounded wider search, not an expanding/unbounded retry. Equal
        # radii disable it. Only the observation position changes, never the
        # footprint, arrival allowance, known-free route or supercover ray.
        self.frontier_fallback_standoff_m = frontier_fallback_standoff_m
        if (isinstance(goal_arrival_margin_m, bool) or not isfinite(goal_arrival_margin_m)
                or goal_arrival_margin_m <= 0):
            raise ValueError('goal_arrival_margin_m must be finite and positive')
        self.goal_arrival_margin_m = goal_arrival_margin_m
        # Simulation baseline: greater than Nav2's .05 m XY goal tolerance.
        # Verify against the actual goal checker before adapting a real robot.
        self.frontier_min_translation_m = frontier_min_translation_m

    def select(self, grid, pose, coverage, failures, now, perception_ready, observed_grid=None):
        # The optional planning prior permits collision checks in an independently
        # verified start area. It must never become map or camera evidence.
        observed_grid = grid if observed_grid is None else observed_grid
        geometry = ('width', 'height', 'resolution', 'origin_x', 'origin_y', 'origin_yaw')
        if any(getattr(grid, key) != getattr(observed_grid, key) for key in geometry):
            raise ValueError('Planning and observed grids must have identical geometry')
        start = grid.to_cell(*pose[:2])
        connected = grid.component(start)
        safe = grid.safe_cells(self.radius)
        reachable = grid.component(start, safe)
        if not reachable:
            return Selection('unsafe_or_unlocalized')
        observed = {p for p in connected if observed_grid.free(p)}
        unobserved = connected.keys() - observed
        frontiers = {p for p in observed if observed_grid.frontier(p)}
        uncovered = {p for p in observed if not coverage.contains(observed_grid.to_world(p), observed_grid.resolution)}
        failures.resolve_observed(observed_grid, coverage)
        stats = dict(frontier_count=len(frontiers), uncovered_count=len(uncovered),
                     reachable_count=len(connected), unobserved_count=len(unobserved))
        if not frontiers and not uncovered and not unobserved and not failures.entries and perception_ready:
            return Selection('complete', **stats)
        if not frontiers and unobserved:
            return Selection('mapping_evidence_incomplete', **stats)
        if not frontiers and not perception_ready:
            return Selection('perception_unverified', **stats)
        kind = 'frontier' if frontiers else 'coverage'
        anchors = frontiers if frontiers else uncovered
        # Greedy nearby anchor, then shortest known-grid travel within its local
        # candidate set. Do not enumerate all free cells × all standoff offsets.
        # We still inspect every anchor if earlier ones have no feasible goal,
        # so sampling cannot incorrectly produce a completion decision.
        ordered = sorted(anchors, key=lambda p: ((p[0]-start[0])**2 + (p[1]-start[1])**2, p))
        # Keep routes/start checks at R, but leave endpoint room for arrival
        # error plus the worst to_cell center displacement. For a fixed map,
        # any arrival within margin of this center maps to an R-safe cell.
        # This is not a bound on SLAM changes, localization error or the path.
        endpoint_radius = self.radius + self.goal_arrival_margin_m + hypot(grid.resolution, grid.resolution)/2
        endpoints = grid.safe_cells(endpoint_radius)
        search_radii = [self.standoff]
        if kind == 'frontier' and self.frontier_fallback_standoff_m > self.standoff:
            search_radii.append(self.frontier_fallback_standoff_m)
        # Exhaust every anchor at the primary radius before considering the
        # wider ring. Coverage goals retain their original search behavior.
        for search_radius in search_radii:
            reach = ceil(search_radius/grid.resolution)
            for anchor_cell in ordered:
                anchor = grid.to_world(anchor_cell)
                if not failures.allowed(anchor, kind, now):
                    continue
                local = []
                for dy in range(-reach, reach+1):
                    for dx in range(-reach, reach+1):
                        p = anchor_cell[0]+dx, anchor_cell[1]+dy
                        if p not in reachable or p not in endpoints:
                            continue
                        xy = grid.to_world(p)
                        distance = hypot(xy[0]-anchor[0], xy[1]-anchor[1])
                        if (distance > search_radius or
                                (search_radius > self.standoff and distance <= self.standoff) or
                                (kind == 'coverage' and distance < .20)):
                            continue
                        if kind == 'frontier' and (p == anchor_cell or
                                hypot(xy[0]-pose[0], xy[1]-pose[1]) <= self.frontier_min_translation_m):
                            # Use the actual pose, not its cell center; coverage
                            # may still turn in place. No zero-vector heading.
                            continue
                        if not grid.line_free(p, anchor_cell):
                            continue
                        local.append((reachable[p]*grid.resolution + .2*distance, p))
                if local:
                    cost, p = min(local)
                    xy = grid.to_world(p)
                    yaw = atan2(anchor[1]-xy[1], anchor[0]-xy[0])
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
    def __init__(self, min_hits=3, merge_radius=.30, window=3.0, min_confidence=.7,
                 *, max_detections=100, max_tracks=256, max_streams=16):
        if (type(min_hits) is not int or min_hits < 1 or
                not all(isfinite(v) and v > 0 for v in (merge_radius, window)) or
                not isfinite(min_confidence) or not 0 <= min_confidence <= 1 or
                any(type(v) is not int or not 1 <= v <= limit for v, limit in
                    ((max_detections, 100), (max_tracks, 256), (max_streams, 16)))):
            raise ValueError('invalid_target_store_limits')
        self.min_hits, self.merge_radius, self.window, self.min_confidence = min_hits, merge_radius, window, min_confidence
        self.targets = []
        self.next_uid = 1
        self.max_detections, self.max_tracks, self.max_streams = max_detections, max_tracks, max_streams
        self.frame_epoch = None
        self.frame_watermarks = {}

    def add_frame(self, observations, stamp_ns, *, source, sensor_frame, epoch):
        """Atomically associate a single complete detector message, once per frame.

        Each prior track can receive at most one independent observation. The
        bounded minimum-cost assignment first maximizes compatible old-track
        matches, then minimizes normalized squared XY displacement. Source
        instance IDs are compatibility constraints, never a license to jump.
        This is geometric association, not proof of physical object identity.
        """
        def reject(reason):
            return {'accepted': False, 'reason': reason, 'assignments': [], 'ignored': []}
        if (type(stamp_ns) is not int or not 0 <= stamp_ns <= 2**63-1 or
                not all(isinstance(v, str) and 0 < len(v) <= 256 for v in (source, sensor_frame, epoch))):
            return reject('invalid_frame_identity')
        if self.frame_epoch is not None and epoch != self.frame_epoch:
            return reject('map_epoch_changed_requires_new_store')
        if not isinstance(observations, (list, tuple)) or len(observations) > self.max_detections:
            return reject('detection_count_limit')
        stream = source, sensor_frame
        if stream not in self.frame_watermarks and len(self.frame_watermarks) >= self.max_streams:
            return reject('stream_count_limit')
        normalized, ignored = [], []
        try:
            for index, item in enumerate(observations):
                class_id, xyz = item['class_id'], item['position']
                confidence, instance = item['confidence'], item.get('instance_id', '')
                if (not isinstance(class_id, str) or not 0 < len(class_id) <= 128 or
                        not isinstance(instance, str) or len(instance) > 128 or
                        not isinstance(xyz, (list, tuple)) or len(xyz) != 3 or
                        any(type(v) not in (int, float) or not isfinite(v) or abs(v) > 1e6 for v in xyz) or
                        type(confidence) not in (int, float) or not isfinite(confidence) or not 0 <= confidence <= 1):
                    return reject('invalid_observation')
                values = (class_id, instance, *map(float, xyz), float(confidence))
                normalized.append((values, index))
                if confidence < self.min_confidence:
                    ignored.append({'index': index, 'reason': 'below_min_confidence'})
        except (KeyError, TypeError, ValueError, OverflowError):
            return reject('invalid_observation')
        normalized.sort(key=lambda pair: pair[0])
        digest = hashlib.sha256(json.dumps([v for v, _ in normalized],
            separators=(',', ':'), ensure_ascii=False, allow_nan=False).encode()).hexdigest()
        previous = self.frame_watermarks.get(stream)
        if previous is not None and stamp_ns <= previous[0]:
            return reject('duplicate_frame' if stamp_ns == previous[0] and digest == previous[1] else
                          ('conflicting_duplicate_frame' if stamp_ns == previous[0] else 'nonadvancing_frame'))
        selected = [(v, index) for v, index in normalized if v[-1] >= self.min_confidence]
        seen_geometry, seen_ids = set(), set()
        for values, _ in selected:
            geometry = values[:-1]  # Different confidence does not make an independent observation.
            if geometry in seen_geometry:
                return reject('indistinguishable_observations_in_frame')
            seen_geometry.add(geometry)
            if values[1]:
                identity = values[:2]
                if identity in seen_ids:
                    return reject('duplicate_instance_id_in_frame')
                seen_ids.add(identity)
        tracks = sorted(self.targets, key=lambda t: t.uid)
        if len(tracks) > self.max_tracks:
            return reject('track_count_limit')
        count = len(selected)
        # Every real match costs <=1. One dummy costs n+1: cardinality first.
        unmatched_cost = float(count+1)
        impossible_cost = unmatched_cost*(count+1)+1.
        costs, distances = [], []
        for values, _ in selected:
            class_id, instance, x, y, _z, _confidence = values
            row, ds = [], []
            for target in tracks:
                distance = hypot(target.x-x, target.y-y)
                eligible = (target.class_id == class_id and distance <= self.merge_radius and
                            (not instance or not target.source_id or instance == target.source_id))
                row.append((distance/self.merge_radius)**2 if eligible else impossible_cost)
                ds.append(distance if eligible else None)
            costs.append(row+[unmatched_cost]*count)
            distances.append(ds)
        assignment = _minimum_cost_columns(costs)
        new_count = sum(column >= len(tracks) for column in assignment)
        if len(tracks)+new_count > self.max_tracks:
            return reject('track_count_limit')
        # No mutation until input, epoch, replay, matching and capacity checks pass.
        updated = {target.uid: replace(target) for target in tracks}
        next_uid, stamp = self.next_uid, stamp_ns/1e9
        output = []
        for (values, original_index), column, ds in zip(selected, assignment, distances):
            class_id, instance, x, y, z, _confidence = values
            created = column >= len(tracks)
            incremented = True
            if created:
                target = Target(next_uid, class_id, x, y, z, 1, stamp, stamp,
                                confirmed=self.min_hits <= 1, source_id=instance)
                next_uid += 1
                updated[target.uid] = target
                distance = None
            else:
                original = tracks[column]
                target, distance = updated[original.uid], ds[column]
                if stamp <= target.last_stamp:
                    incremented = False  # Preserve per-track cross-stream replay protection.
                else:
                    if not target.confirmed and stamp-target.first_stamp > self.window:
                        target.hits, target.first_stamp = 0, stamp
                    weight = min(target.hits, 10)
                    target.x, target.y, target.z = tuple((old*weight+new)/(weight+1)
                        for old, new in zip((target.x, target.y, target.z), (x, y, z)))
                    target.hits += 1
                    target.last_stamp = stamp
                    target.confirmed = target.confirmed or target.hits >= self.min_hits
            output.append({'index': original_index, 'uid': target.uid, 'created': created,
                           'distance_m': distance, 'hit_incremented': incremented})
        self.targets = sorted(updated.values(), key=lambda target: target.uid)
        self.next_uid, self.frame_epoch = next_uid, epoch
        self.frame_watermarks[stream] = stamp_ns, digest
        return {'accepted': True, 'reason': 'associated', 'assignments': sorted(output, key=lambda a: a['index']),
                'ignored': ignored, 'matched_tracks': count-new_count, 'new_tracks': new_count,
                'algorithm': 'maximum-compatible-cardinality_then_minimum-squared-XY_cost',
                'physical_identity_proven': False}

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


def _minimum_cost_columns(costs):
    """Deterministic rectangular Hungarian assignment, O(n^2*m), n<=100/m<=356.

    Rows are canonical observation order; real columns are track UID order,
    followed by dummies. Equal comparisons keep the first (lowest) column.
    """
    if not costs:
        return []
    n, m = len(costs), len(costs[0])
    if m < n or n > 100 or m > 356 or any(len(row) != m for row in costs):
        raise ValueError('assignment_matrix_out_of_bounds')
    u, v, owner, way = [0.]*(n+1), [0.]*(m+1), [0]*(m+1), [0]*(m+1)
    for row in range(1, n+1):
        owner[0], column = row, 0
        minimum, used = [float('inf')]*(m+1), [False]*(m+1)
        while True:
            used[column] = True
            current_row, delta, next_column = owner[column], float('inf'), 0
            for candidate in range(1, m+1):
                if used[candidate]:
                    continue
                reduced = costs[current_row-1][candidate-1]-u[current_row]-v[candidate]
                if reduced < minimum[candidate]:
                    minimum[candidate], way[candidate] = reduced, column
                if minimum[candidate] < delta:
                    delta, next_column = minimum[candidate], candidate
            for candidate in range(m+1):
                if used[candidate]:
                    u[owner[candidate]] += delta
                    v[candidate] -= delta
                else:
                    minimum[candidate] -= delta
            column = next_column
            if owner[column] == 0:
                break
        while column:
            previous = way[column]
            owner[column] = owner[previous]
            column = previous
    result = [-1]*n
    for column in range(1, m+1):
        if owner[column]:
            result[owner[column]-1] = column-1
    return result
