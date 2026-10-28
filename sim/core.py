"""Unknown-maze exploration with a strict observation/planning boundary.

World holds ground truth. Explorer holds only observations and known grid bounds.
The pose is exact, the scene is static, motion is a one-cell cardinal translation,
and sensing is noiseless. These assumptions deliberately exclude SLAM, camera
recognition, vehicle dynamics, and any claim about real-robot success.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from math import hypot, isfinite
import random

Cell = tuple[int, int]
FREE, WALL, UNKNOWN = ".", "#", "?"


def neighbors(p: Cell, width: int, height: int):
    for dx, dy in ((1, 0), (0, 1), (-1, 0), (0, -1)):
        q = p[0] + dx, p[1] + dy
        if 0 <= q[0] < width and 0 <= q[1] < height:
            yield q


def supercover_line(a: Cell, b: Cell) -> list[Cell]:
    """Cells intersected by a center-to-center ray, including corner neighbors.

    At an exact corner both side cells are included, so sight cannot leak through
    two diagonally touching walls. Endpoint walls themselves remain observable.
    """
    x, y = a
    dx, dy = b[0] - x, b[1] - y
    nx, ny = abs(dx), abs(dy)
    sx, sy = (1 if dx > 0 else -1), (1 if dy > 0 else -1)
    ix = iy = 0
    cells = [(x, y)]
    while ix < nx or iy < ny:
        decision = (1 + 2 * ix) * ny - (1 + 2 * iy) * nx
        if decision == 0:
            cells.extend(((x + sx, y), (x, y + sy)))
            x, y, ix, iy = x + sx, y + sy, ix + 1, iy + 1
        elif decision < 0:
            x, ix = x + sx, ix + 1
        else:
            y, iy = y + sy, iy + 1
        cells.append((x, y))
    return cells


@dataclass(frozen=True)
class TargetObservation:
    """Ideal detector output. IDs become available only upon observation."""
    target_id: str
    position: Cell


@dataclass(frozen=True)
class Observation:
    cells: dict[Cell, str]
    targets: tuple[TargetObservation, ...] = ()
    searched_cells: frozenset[Cell] = frozenset()

    @property
    def target(self) -> Cell | None:
        """Legacy single-target accessor; formal missions use targets."""
        return self.targets[0].position if self.targets else None


class World:
    """Simulator and ideal sensors. Never passed to the planning algorithm."""

    def __init__(self, rows: list[str]):
        if not rows or not rows[0] or len({len(r) for r in rows}) != 1:
            raise ValueError("Maze rows must be non-empty and rectangular")
        if any(ch not in "#. ST" for r in rows for ch in r):
            raise ValueError("Maze characters: # wall, . or space free, S start, T target")
        starts = [(x, y) for y, r in enumerate(rows) for x, v in enumerate(r) if v == "S"]
        targets = [(x, y) for y, r in enumerate(rows) for x, v in enumerate(r) if v == "T"]
        if len(starts) != 1:
            raise ValueError("Exactly one S is required; zero or more T cells are allowed")
        self.rows, self.width, self.height = rows, len(rows[0]), len(rows)
        self.targets = {f"item_{x}_{y}": (x, y) for x, y in targets}
        self.start, self.target = starts[0], targets[0] if targets else None

    def cell(self, p: Cell) -> str:
        return WALL if self.rows[p[1]][p[0]] == WALL else FREE

    def visible(self, a: Cell, b: Cell) -> bool:
        return not any(self.cell(p) == WALL for p in supercover_line(a, b) if p != a and p != b)

    def observe(self, pose: Cell, sensor_range: float, target_range: float) -> Observation:
        seen = {}
        for y in range(self.height):
            for x in range(self.width):
                p = (x, y)
                if hypot(x - pose[0], y - pose[1]) <= sensor_range and self.visible(pose, p):
                    seen[p] = self.cell(p)
        targets = tuple(TargetObservation(target_id, p) for target_id, p in self.targets.items()
                        if p in seen and hypot(p[0] - pose[0], p[1] - pose[1]) <= target_range)
        searched = frozenset(p for p, value in seen.items() if value == FREE and
                             hypot(p[0] - pose[0], p[1] - pose[1]) <= target_range)
        return Observation(seen, targets, searched)

    def move(self, old: Cell, new: Cell) -> Cell:
        if new not in neighbors(old, self.width, self.height) or self.cell(new) == WALL:
            raise ValueError(f"Illegal move {old} -> {new}")
        return new

    def reachable_free(self) -> set[Cell]:
        """Ground truth for offline evaluation only."""
        reached, queue = {self.start}, deque([self.start])
        while queue:
            for q in neighbors(queue.popleft(), self.width, self.height):
                if q not in reached and self.cell(q) == FREE:
                    reached.add(q)
                    queue.append(q)
        return reached


def shortest_path(known: dict[Cell, str], start: Cell, goals: set[Cell], width: int, height: int):
    """BFS over observed FREE cells only. Returns an inclusive path, or None."""
    if known.get(start) != FREE:
        return None
    queue, parent = deque([start]), {start: None}
    while queue:
        p = queue.popleft()
        if p in goals:
            path = [p]
            while parent[path[-1]] is not None:
                path.append(parent[path[-1]])
            return list(reversed(path))
        for q in neighbors(p, width, height):
            if q not in parent and known.get(q) == FREE:
                parent[q] = p
                queue.append(q)
    return None


@dataclass(frozen=True)
class Config:
    sensor_range: float = 3.0
    target_range: float = 2.0
    policy: str = "explore_then_return"
    explore_budget: int = 1000
    max_steps: int = 2000

    def validate(self):
        if not isfinite(self.sensor_range) or self.sensor_range < 1:
            raise ValueError("sensor_range must be >= 1 so cardinal frontier cells are observable")
        if not 1 <= self.target_range <= self.sensor_range:
            raise ValueError("target_range must be between 1 and sensor_range")
        if self.policy not in ("find_then_return", "explore_then_return"):
            raise ValueError("Unknown policy")
        if self.explore_budget < 0 or self.max_steps < 0:
            raise ValueError("Budgets must be non-negative")


class Explorer:
    def __init__(self, width: int, height: int, start: Cell, config: Config):
        config.validate()
        self.width, self.height, self.start, self.config = width, height, start, config
        self.known: dict[Cell, str] = {}
        self.searched: set[Cell] = set()
        self.discovered_targets: dict[str, Cell] = {}
        self.phase = "explore"
        self.return_reason = None
        self.search_complete = False
        self.exploration_moves = 0
        self.status = "running"

    def ingest(self, observation: Observation) -> list[list]:
        delta = []
        for p, value in observation.cells.items():
            if p in self.known and self.known[p] != value:
                raise ValueError("Static-world observation conflict")
            if p not in self.known:
                delta.append([p[0], p[1], value])
            self.known[p] = value
        for target in observation.targets:
            self.discovered_targets[target.target_id] = target.position
        self.searched.update(observation.searched_cells)
        return delta

    @property
    def target(self) -> Cell | None:
        """First observed target for backward-compatible debug mode."""
        return next(iter(self.discovered_targets.values()), None)

    def frontiers(self) -> set[Cell]:
        return {p for p, value in self.known.items() if value == FREE and
                any(q not in self.known for q in neighbors(p, self.width, self.height))}

    def path(self, pose: Cell, goals: set[Cell]):
        return shortest_path(self.known, pose, goals, self.width, self.height)

    def begin_return(self, reason: str):
        self.phase, self.return_reason = "return", reason

    def finish(self):
        if self.search_complete:
            self.status = "found_explored_returned" if self.target is not None else "not_found_explored_returned"
        elif self.return_reason == "target_found":
            self.status = "found_returned"
        else:
            self.status = "search_incomplete_returned"
        self.phase = "done"

    def choose(self, pose: Cell, steps: int) -> Cell | None:
        """No World object, target truth, or evaluation information is accessible."""
        home = self.path(pose, {self.start})
        if home is None:
            self.phase, self.status = "failed", "failed_no_known_return_path"
            return None
        if self.phase == "explore":
            # Geometric coverage is not semantic coverage: a lidar-visible free
            # cell may still lie beyond the ideal target detector's range.
            search_goals = self.frontiers() | {p for p, value in self.known.items()
                                               if value == FREE and p not in self.searched}
            frontier_path = self.path(pose, search_goals)
            if frontier_path is None:
                self.search_complete = True
                self.begin_return("reachable_region_explored")
            elif self.target is not None and self.config.policy == "find_then_return":
                self.begin_return("target_found")
            elif self.exploration_moves >= self.config.explore_budget:
                self.begin_return("exploration_budget")
            elif self.config.max_steps - steps <= len(home) - 1:
                self.begin_return("total_budget_return_reserve")
            elif len(frontier_path) > 1:
                # One outward move can increase shortest return distance by one.
                # Keep enough total budget to get home after that move.
                proposed = frontier_path[1]
                proposed_home = self.path(proposed, {self.start})
                if steps + 1 + len(proposed_home) - 1 > self.config.max_steps:
                    self.begin_return("total_budget_return_reserve")
                else:
                    return proposed
            else:
                self.phase, self.status = "failed", "failed_unobservable_frontier"
                return None
        if self.phase == "return":
            if pose == self.start:
                self.finish()
            elif steps >= self.config.max_steps:
                self.phase, self.status = "failed", "failed_total_budget"
            else:
                return home[1]
        return None


def run_mission(world: World, config: Config) -> dict:
    """Simulator loop; truth-based evaluation is performed only after the run."""
    agent = Explorer(world.width, world.height, world.start, config)
    pose, steps = world.start, 0
    trajectory, frames = [pose], []
    first_detection_step = None
    first_detection_steps: dict[str, int] = {}
    while agent.phase not in ("done", "failed"):
        observation = world.observe(pose, config.sensor_range, config.target_range)
        new_searched = observation.searched_cells - agent.searched
        delta = agent.ingest(observation)
        for target_id in agent.discovered_targets:
            first_detection_steps.setdefault(target_id, steps)
        if agent.target is not None and first_detection_step is None:
            first_detection_step = steps
        action = agent.choose(pose, steps)
        frames.append({"step": steps, "position": list(pose), "phase": agent.phase,
                       "new_cells": delta, "known_cells": len(agent.known),
                       "new_searched_cells": [list(p) for p in sorted(new_searched)],
                       "target_detected": agent.target is not None,
                       "detected_target": list(agent.target) if agent.target else None,
                       "detected_targets": [{"id": target_id, "grid_xy": list(p)}
                                            for target_id, p in agent.discovered_targets.items()],
                       "return_reason": agent.return_reason,
                       "next_position": list(action) if action else None})
        if action is not None:
            phase = agent.phase
            try:
                pose = world.move(pose, action)
            except ValueError:
                agent.phase, agent.status = "failed", "failed_collision"
                break
            steps += 1
            agent.exploration_moves += phase == "explore"
            trajectory.append(pose)
    reachable = world.reachable_free()
    known_free = {p for p, value in agent.known.items() if value == FREE}
    # These fields are offline evaluator outputs, never policy inputs.
    metrics = {
        "status": agent.status,
        "returned_to_start": pose == world.start,
        "target_detected": agent.target is not None,
        "target_exists_ground_truth": world.target is not None,
        "detected_target_count": len(agent.discovered_targets),
        "target_count_ground_truth": len(world.targets),
        "reachable_target_count_ground_truth": sum(p in reachable for p in world.targets.values()),
        "all_reachable_targets_detected_ground_truth": all(target_id in agent.discovered_targets
                                                           for target_id, p in world.targets.items() if p in reachable),
        "all_targets_detected_ground_truth": set(agent.discovered_targets) == set(world.targets),
        "missed_target_count_ground_truth": len(set(world.targets) - set(agent.discovered_targets)),
        "target_reachable_ground_truth": world.target in reachable if world.target else None,
        "search_complete_reachable_region": agent.search_complete,
        "known_map_frontier_count": len(agent.frontiers()),
        "known_free_cells_not_semantically_searched": len(known_free - agent.searched),
        "return_reason": agent.return_reason,
        "steps": steps,
        "exploration_moves": agent.exploration_moves,
        "return_moves": steps - agent.exploration_moves,
        "first_detection_step": first_detection_step,
        "detected_target_map_grid_xy": list(agent.target) if agent.target is not None else None,
        "detected_targets_map_grid_xy": {target_id: list(p) for target_id, p in agent.discovered_targets.items()},
        "known_cells": len(agent.known),
        "total_cells": world.width * world.height,
        "observed_grid_fraction": len(agent.known) / (world.width * world.height),
        "reachable_free_cells_ground_truth": len(reachable),
        "observed_reachable_free_cells": len(known_free & reachable),
        "reachable_free_coverage": len(known_free & reachable) / len(reachable),
        "searched_reachable_free_coverage": len(agent.searched & reachable) / len(reachable),
        "visited_reachable_free_coverage": len(set(trajectory)) / len(reachable),
    }
    return {
        "schema_version": 2,
        "assumptions": ["Exact grid pose and prior rectangular map bounds",
                        "Static walls; noiseless occluded sensing; cardinal point-robot motion",
                        "Target is an ideal semantic detection within target_range, not a camera model",
                        "Target positions and target count are unknown to the agent; IDs arrive only on detection",
                        "No SLAM, Gazebo, robot dynamics, grasping, or physical validation"],
        "config": vars(config), "metrics": metrics,
        "target_annotations": [{"id": target_id, "label": "simulated_special_item", "grid_xy": list(p),
                                 "first_detection_step": first_detection_steps[target_id],
                                 "coordinate_convention": "cell indices; x right, y down; origin upper-left"}
                               for target_id, p in agent.discovered_targets.items()],
        "start": list(world.start), "trajectory": [list(p) for p in trajectory],
        "final_known_map": ["".join(agent.known.get((x, y), UNKNOWN) for x in range(world.width))
                            for y in range(world.height)],
        "frames": frames,
        "offline_ground_truth": {"rows": world.rows, "target": list(world.target) if world.target else None,
                                 "targets": [{"id": target_id, "grid_xy": list(p)} for target_id, p in world.targets.items()]},
    }


def generated_maze(seed: int = 0, width: int = 21, height: int = 17, target: bool = True,
                   target_count: int | None = None) -> World:
    if width < 5 or height < 5 or width % 2 == 0 or height % 2 == 0:
        raise ValueError("Generated width and height must be odd and >= 5")
    rng = random.Random(seed)
    grid = [[WALL] * width for _ in range(height)]
    grid[1][1] = FREE
    stack, seen = [(1, 1)], {(1, 1)}
    while stack:
        x, y = stack[-1]
        choices = [(x + dx, y + dy) for dx, dy in ((2, 0), (0, 2), (-2, 0), (0, -2))
                   if 0 < x + dx < width - 1 and 0 < y + dy < height - 1 and (x + dx, y + dy) not in seen]
        if not choices:
            stack.pop()
            continue
        q = rng.choice(choices)
        grid[(y + q[1]) // 2][(x + q[0]) // 2] = FREE
        grid[q[1]][q[0]] = FREE
        seen.add(q)
        stack.append(q)
    grid[1][1] = "S"
    count = (1 if target else 0) if target_count is None else target_count
    if not isinstance(count, int) or count < 0:
        raise ValueError("target_count must be a non-negative integer")
    if count:
        # World generation may use truth; this data is not supplied to Explorer.
        temp = World(["".join(row) for row in grid])
        distances, queue = {temp.start: 0}, deque([temp.start])
        while queue:
            p = queue.popleft()
            for q in neighbors(p, width, height):
                if temp.cell(q) == FREE and q not in distances:
                    distances[q] = distances[p] + 1
                    queue.append(q)
        candidates = [p for p in distances if p != temp.start]
        if count > len(candidates):
            raise ValueError("target_count exceeds available free cells excluding start")
        # Include one distant target, then spread others across randomly sampled
        # free cells. Generation truth is never supplied to Explorer.
        farthest = max(candidates, key=distances.get)
        selected = [farthest] + rng.sample([p for p in candidates if p != farthest], count - 1)
        for x, y in selected:
            grid[y][x] = "T"
    return World(["".join(row) for row in grid])


SCENARIOS = {
    "demo": ["#############", "#S....#.....#", "#.##..#.###.#", "#.....#...T.#",
             "###.#####.#.#", "#...........#", "#############"],
    "occluded": ["#########", "#S#....T#", "#.#.#####", "#...#####", "#########"],
    "unreachable": ["###########", "#S...#...T#", "#.##.#.##.#", "#....#....#", "###########"],
    "absent": ["#########", "#S......#", "#.###.#.#", "#.....#.#", "#########"],
    "multi_branch": ["#############", "#T....#....T#", "#.###.#.###.#", "#.....S.....#",
                     "#.###.#.###.#", "#.....#....T#", "#############"],
    "multi_unreachable": ["#############", "#S.T..#T...T#", "#.###.#.###.#", "#.....#.....#", "#############"],
}
