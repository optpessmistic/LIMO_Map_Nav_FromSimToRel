"""Finite observation actions, not a camera coverage model or a ROS executor.

The caller supplies the RAW observed Grid, never a planning copy with priors.
One action set is frozen from one map snapshot in one epoch. An explicit local
relocation radius may adjust pending station positions after a map update;
anchors, headings and budgets stay fixed. Unsafe views can still be skipped.
Build a new queue only for an
explicitly new mission, not every planner tick. No target/scene truth is used.
"""
from collections import Counter, deque
from copy import deepcopy
from dataclasses import asdict, dataclass, replace
from math import ceil, cos, hypot, isfinite, pi, sin
import hashlib
import json
from . import core
Grid, Goal = core.Grid, core.Goal


@dataclass(frozen=True)
class Policy:
    # All distances are public robot geometry or action sampling choices.
    # station_spacing_m is NOT a sensor range or a coverage guarantee.
    robot_radius_m: float
    arrival_margin_m: float
    station_spacing_m: float
    max_stations: int
    headings_per_station: int
    max_dispatches: int
    max_attempts_per_view: int = 1
    max_grid_cells: int = 250000
    relocation_radius_m: float = 0.0

    def __post_init__(self):
        for name in ('robot_radius_m', 'arrival_margin_m', 'station_spacing_m'):
            value = getattr(self, name)
            if type(value) not in (int, float) or not isfinite(value) or value <= 0:
                raise ValueError(name+' must be finite and positive')
        for name, maximum in (('max_stations', 64), ('headings_per_station', 16),
                              ('max_dispatches', 1024), ('max_attempts_per_view', 8),
                              ('max_grid_cells', 1000000)):
            value = getattr(self, name)
            if type(value) is not int or not 1 <= value <= maximum:
                raise ValueError(name+' exceeds its explicit finite bound')
        radius = self.relocation_radius_m
        if (type(radius) not in (int, float) or not isfinite(radius) or
                not 0 <= radius <= min(.5, self.station_spacing_m/4)):
            raise ValueError('relocation_radius_m must be finite, nonnegative and at most min(0.5, station_spacing_m/4)')


@dataclass(frozen=True)
class View:
    action_id: str
    epoch: str
    station_id: int
    heading_index: int
    goal: Goal


@dataclass(frozen=True)
class Decision:
    state: str
    view: View | None = None
    reason: str | None = None
    dispatch_id: int | None = None
    validation: dict | None = None


def check_epoch(epoch):
    if not isinstance(epoch, str) or not epoch.strip() or epoch != epoch.strip() or len(epoch) > 256:
        raise ValueError('Explicit map epoch required')


def check_inputs(grid, pose, policy):
    if not isinstance(grid, Grid) or grid.width*grid.height > policy.max_grid_cells:
        raise ValueError('Observed Grid is missing or exceeds the cell budget')
    if not isinstance(pose, (tuple, list)) or len(pose) != 3 or any(
            type(v) not in (float, int) or not isfinite(v) for v in pose):
        raise ValueError('Pose must contain finite x, y, base yaw')


def reachable(grid, pose, policy):
    """Existing conservative disk safety + 4-neighbor known-free connectivity."""
    safe = grid.safe_cells(policy.robot_radius_m)
    start = grid.to_cell(*pose[:2])
    if start not in safe:
        return {}
    # The actual current pose need not be its cell center. Inflate by its exact
    # offset to avoid assuming center clearance proves actual-pose clearance.
    center = grid.to_world(start)
    offset = hypot(center[0]-pose[0], center[1]-pose[1])
    if offset > 1e-10 and start not in grid.safe_cells(policy.robot_radius_m+offset):
        return {}
    return grid.component(start, safe)


def endpoint_radius(grid, policy, offset=0.):
    # Same endpoint reserve used by the production Planner. The extra offset
    # supports frozen world goals after a same-epoch grid-origin/resolution update.
    return (policy.robot_radius_m+policy.arrival_margin_m+
            hypot(grid.resolution, grid.resolution)/2+offset)


def station_first_order(grid, pose, distances, views):
    """Reorder exactly the frozen actions using known-safe grid path lengths.

    Visit a nearest remaining station, then its remaining headings with the
    smallest planned yaw change. This is a deterministic greedy route, not a
    globally optimal path or a prediction of Nav2's actual travel/turning time.
    The original reachable component already passed the footprint checks.
    Later dispatches must still revalidate safety on the current raw map.
    """
    by_station = {}
    for action_id, view in views.items():
        by_station.setdefault(view.station_id, []).append(action_id)
    cells = {station: grid.to_cell(views[actions[0]].goal.x, views[actions[0]].goal.y)
             for station, actions in by_station.items()}
    order, remaining = [], set(by_station)
    steps, yaw = distances, pose[2]
    while remaining:
        station = min(remaining, key=lambda s: (steps[cells[s]], s))
        remaining.remove(station)
        actions = list(by_station[station])
        while actions:
            action_id = min(actions, key=lambda a: (
                abs(core.angle_delta(views[a].goal.yaw, yaw)), views[a].heading_index, a))
            actions.remove(action_id)
            order.append(action_id)
            yaw = views[action_id].goal.yaw
        if remaining:
            # Same conservative four-neighbor graph as the production planner;
            # nearby coordinates across a wall may require a long detour.
            steps = grid.component(cells[station], distances)
    return order


class ObservationQueue:
    """At most max_dispatches attempts, independent of contains()/detector claims.

    finish(..., 'inspection_satisfied') is a caller-reported action result, not a
    proof created by this queue. A future adapter must correlate the current goal
    token, epoch and settled processed-frame ledger before reporting that result.
    """
    OUTCOMES = ('inspection_satisfied', 'navigation_failed', 'inspection_failed', 'canceled')

    def __init__(self, observed_grid, pose, epoch, policy):
        check_epoch(epoch)
        check_inputs(observed_grid, pose, policy)
        self.epoch, self.policy = epoch, policy
        self.invalid_reason = None
        self.active_id = None
        self.active_dispatch_id = None
        self.active_view = None
        self.selection_history = []
        self.dispatches = 0
        self.views, self.records, self.pending = {}, {}, deque()
        distances = reachable(observed_grid, pose, policy)
        endpoints = observed_grid.safe_cells(endpoint_radius(observed_grid, policy))
        candidates = {p: observed_grid.to_world(p) for p in distances if p in endpoints}
        self.initial_reason = ('unsafe_or_unlocalized' if not distances else
                               'no_safe_endpoint' if not candidates else None)
        self.planned_from = {'width': observed_grid.width, 'height': observed_grid.height,
                            'resolution': observed_grid.resolution,
                            'origin': (observed_grid.origin_x, observed_grid.origin_y, observed_grid.origin_yaw)}
        stations = []
        # A deterministic spread of known reachable endpoints. Spacing is a
        # sampling/dedup choice only; it is never a camera visibility radius.
        if candidates:
            first = min(candidates, key=lambda p: (distances[p], p))
            stations.append(candidates.pop(first))
        while candidates and len(stations) < min(policy.max_stations, policy.max_dispatches):
            def score(cell):
                xy = candidates[cell]
                distance = min(hypot(xy[0]-s[0], xy[1]-s[1]) for s in stations)
                return distance, -distances[cell], -cell[0], -cell[1]
            chosen = max(candidates, key=score)
            if score(chosen)[0] < policy.station_spacing_m:
                break
            stations.append(candidates.pop(chosen))
        self.stations = tuple(stations)
        self.station_positions = dict(enumerate(stations))
        headings = [core.angle_delta(2*pi*i/policy.headings_per_station, 0.)
                    for i in range(policy.headings_per_station)]
        heading_order = sorted(range(len(headings)), key=lambda i: (abs(core.angle_delta(headings[i], pose[2])), i))
        # Keep the original round-robin selection/IDs, including a truncated
        # max_dispatches set. Only its execution order is optimized below.
        for heading_index in heading_order:
            for station_index, xy in enumerate(stations):
                if len(self.views) >= policy.max_dispatches:
                    break
                action_id = 'observation-%04d' % (len(self.views)+1)
                goal = Goal(*xy, headings[heading_index], 'observation_queue', xy)
                view = View(action_id, epoch, station_index, heading_index, goal)
                self.views[action_id] = view
                self.records[action_id] = {'state': 'pending', 'attempts': 0, 'outcomes': []}
                self.pending.append(action_id)
        self.pending = deque(station_first_order(observed_grid, pose, distances, self.views))

    def invalidate(self, reason):
        if not self.invalid_reason:
            self.invalid_reason = str(reason)
        # Preserve the active action. The adapter must cancel/verify its terminal
        # state; invalidation is never permission to forget an in-flight goal.

    def take_next(self, observed_grid, pose, epoch, *, map_stamp_ns=None):
        check_epoch(epoch)
        if epoch != self.epoch:
            self.invalidate('map_epoch_changed')
        if self.invalid_reason:
            return Decision('invalidated', reason=self.invalid_reason)
        if self.active_id is not None:
            return Decision('active', self.active_view, 'await_current_action_terminal', self.active_dispatch_id)
        if not self.pending:
            return Decision('empty' if not self.views else 'exhausted', reason=self.initial_reason)
        if self.dispatches >= self.policy.max_dispatches:
            return Decision('budget_exhausted', reason='dispatch_budget_consumed')
        check_inputs(observed_grid, pose, self.policy)
        if map_stamp_ns is not None and (type(map_stamp_ns) is not int or map_stamp_ns < 0):
            raise ValueError('map_stamp_ns must be a nonnegative integer or explicitly unavailable')
        distances = reachable(observed_grid, pose, self.policy)
        if not distances:
            return Decision('unsafe_or_unlocalized', reason='current_pose_not_in_observed_safe_component')
        endpoint_cache, footprint_cache = {}, {}
        geometry = {'width': observed_grid.width, 'height': observed_grid.height,
                    'resolution': observed_grid.resolution,
                    'origin': [observed_grid.origin_x, observed_grid.origin_y, observed_grid.origin_yaw]}
        content = json.dumps({**geometry, 'data': observed_grid.data},
                             sort_keys=True, separators=(',', ':'), allow_nan=False).encode('utf-8')
        map_binding = {**geometry, 'stamp_ns': map_stamp_ns,
                       'grid_sha256': hashlib.sha256(content).hexdigest()}

        def check_xy(xy):
            cell = observed_grid.to_cell(*xy)
            center = observed_grid.to_world(cell)
            offset = hypot(center[0]-xy[0], center[1]-xy[1])
            # Round UP so caching does not erode the original safety reserve.
            offset_key = ceil(offset*1e9)/1e9
            if offset_key not in endpoint_cache:
                endpoint_cache[offset_key] = observed_grid.safe_cells(endpoint_radius(observed_grid, self.policy, offset_key))
            return {'xy': list(xy), 'cell': list(cell), 'cell_center_offset_m': offset,
                    'effective_endpoint_radius_m': endpoint_radius(observed_grid, self.policy, offset_key),
                    'reachable': cell in distances, 'path_steps': distances.get(cell),
                    'endpoint_safe': cell in endpoint_cache[offset_key]}, cell, offset_key

        while self.pending:
            action_id = self.pending.popleft()
            view = self.views[action_id]
            anchor = self.stations[view.station_id]
            preferred = self.station_positions[view.station_id]
            original_check, anchor_cell, anchor_offset = check_xy(anchor)
            preferred_check, _, _ = check_xy(preferred)
            validation = {'action_id': action_id, 'station_id': view.station_id,
                'heading_index': view.heading_index, 'epoch': self.epoch,
                'map': map_binding, 'pose': list(pose), 'planned_goal': asdict(view.goal),
                'original_check': original_check, 'preferred_check': preferred_check,
                'relocation_radius_m': self.policy.relocation_radius_m,
                'local_candidate_count': 0, 'selected_check': None,
                'selected_shift_from_anchor_m': None, 'relocation_reason': None}
            selected = preferred if preferred_check['reachable'] and preferred_check['endpoint_safe'] else None
            if selected is None and self.policy.relocation_radius_m > 0:
                # The nominal anchor must still support the actual robot disk.
                # Only the larger arrival reserve may be unavailable. This is
                # a local adjustment, not permission to cross an unknown wall.
                if anchor_offset not in footprint_cache:
                    footprint_cache[anchor_offset] = observed_grid.safe_cells(self.policy.robot_radius_m+anchor_offset)
                anchor_safe = anchor_cell in distances and anchor_cell in footprint_cache[anchor_offset]
                validation['anchor_robot_disk_safe'] = anchor_safe
                if anchor_safe:
                    radius = self.policy.relocation_radius_m
                    nearby = {cell for cell in distances
                              if hypot(*(a-b for a, b in zip(observed_grid.to_world(cell), anchor))) <= radius}
                    local = observed_grid.component(anchor_cell, nearby)
                    if 0. not in endpoint_cache:
                        endpoint_cache[0.] = observed_grid.safe_cells(endpoint_radius(observed_grid, self.policy))
                    candidates = [cell for cell in local if cell in endpoint_cache[0.]]
                    validation['local_candidate_count'] = len(candidates)
                    if candidates:
                        chosen = min(candidates, key=lambda cell: (
                            hypot(*(a-b for a, b in zip(observed_grid.to_world(cell), anchor))),
                            distances[cell], cell))
                        candidate = observed_grid.to_world(chosen)
                        candidate_check, _, _ = check_xy(candidate)
                        # Revalidate exact world coordinates too (including
                        # roundoff in a rotated map); never assume zero offset.
                        if candidate_check['reachable'] and candidate_check['endpoint_safe']:
                            selected = candidate
                            validation['relocation_reason'] = 'preferred_endpoint_unavailable_local_safe_replacement'
                        else:
                            validation['relocation_reason'] = 'candidate_world_revalidation_failed'
                    else:
                        validation['relocation_reason'] = 'no_local_safe_endpoint'
                else:
                    validation['relocation_reason'] = 'anchor_robot_disk_unsafe_or_unreachable'
            elif selected is None:
                validation['relocation_reason'] = 'relocation_disabled'
            if selected is None:
                validation['state'] = 'skipped_unsafe_or_unreachable'
                self.selection_history.append(validation)
                self.records[action_id]['state'] = 'skipped_unsafe_or_unreachable'
                continue
            selected_check, _, _ = check_xy(selected)
            validation['selected_check'] = selected_check
            validation['selected_shift_from_anchor_m'] = hypot(selected[0]-anchor[0], selected[1]-anchor[1])
            validation['state'] = 'dispatch'
            validation['dispatch_id'] = self.dispatches+1
            self.selection_history.append(validation)
            self.station_positions[view.station_id] = tuple(selected)
            # Keep the frozen view and completed records intact. Only this
            # dispatch gets the actual endpoint; its anchor and yaw stay fixed.
            dispatch_view = replace(view, goal=replace(view.goal, x=selected[0], y=selected[1]))
            record = self.records[action_id]
            record['state'] = 'active'
            record['attempts'] += 1
            self.dispatches += 1
            self.active_id = action_id
            self.active_dispatch_id = self.dispatches
            self.active_view = dispatch_view
            return Decision('dispatch', dispatch_view, dispatch_id=self.active_dispatch_id,
                            validation=deepcopy(validation))
        return Decision('exhausted', reason='remaining_views_unsafe_or_unreachable')

    def finish(self, action_id, epoch, outcome, dispatch_id):
        if (epoch != self.epoch or action_id != self.active_id or outcome not in self.OUTCOMES or
                type(dispatch_id) is not int or dispatch_id != self.active_dispatch_id):
            raise ValueError('Result does not match the current action/epoch or has unknown outcome')
        if self.invalid_reason and outcome == 'inspection_satisfied':
            raise ValueError('Invalidated coordinates cannot credit an observation action')
        record = self.records[action_id]
        record['outcomes'].append(outcome)
        self.active_id = None
        self.active_dispatch_id = None
        self.active_view = None
        if self.invalid_reason:
            record['state'] = 'invalidated_terminal'
        elif outcome == 'inspection_satisfied':
            record['state'] = 'completed'
        elif record['attempts'] < self.policy.max_attempts_per_view and self.dispatches < self.policy.max_dispatches:
            record['state'] = 'pending'
            self.pending.append(action_id)
        else:
            record['state'] = 'failed'

    def snapshot(self):
        counts = dict(Counter(record['state'] for record in self.records.values()))
        return {'epoch': self.epoch, 'invalidated': self.invalid_reason is not None,
                'invalid_reason': self.invalid_reason, 'active_action_id': self.active_id,
                'active_dispatch_id': self.active_dispatch_id,
                'cancel_required': self.invalid_reason is not None and self.active_id is not None,
                'planned_stations': len(self.stations), 'planned_views': len(self.views),
                'dispatches': self.dispatches, 'dispatch_budget': self.policy.max_dispatches,
                'relocation_radius_m': self.policy.relocation_radius_m,
                'planned_stations_xy': self.stations,
                'station_dispatch_xy': dict(self.station_positions),
                'selection_history': deepcopy(self.selection_history),
                'pending_views': len(self.pending), 'state_counts': counts,
                'all_planned_actions_completed': bool(self.views) and not self.invalid_reason and
                    counts.get('completed', 0) == len(self.views),
                'visual_coverage_claimed': False, 'mission_success_claimed': False,
                'sampling_scope': 'Frozen known-map snapshot; no newly revealed cells are automatically added.'}
