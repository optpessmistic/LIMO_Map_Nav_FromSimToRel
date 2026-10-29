"""Run from project root: python -m sim.run --seed 0 --output outputs/simulation/seed0."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from .core import Config, SCENARIOS, World, generated_maze, run_mission
from .render import write_artifacts


def main() -> int:
    parser = argparse.ArgumentParser(description="Idealized unknown-maze mission logic simulation (not SLAM/Gazebo)")
    parser.add_argument("--scenario", choices=["generated", *SCENARIOS], default="generated")
    parser.add_argument("--map", type=Path, help="Custom UTF-8 ASCII map: # . S T; overrides scenario")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--width", type=int, default=21)
    parser.add_argument("--height", type=int, default=17)
    parser.add_argument("--no-target", action="store_true", help="Omit target when generating a maze")
    parser.add_argument("--target-count", type=int, default=3, help="Generator ground truth only; never supplied to the agent")
    parser.add_argument("--policy", choices=["find_then_return", "explore_then_return"], default="explore_then_return")
    parser.add_argument("--sensor-range", type=float, default=3)
    parser.add_argument("--target-range", type=float, default=2)
    parser.add_argument("--explore-budget", type=int, default=1000)
    parser.add_argument("--max-steps", type=int, default=2000, help="Total moves, including return; return reserve is enforced")
    parser.add_argument("--output", type=Path, default=Path("outputs/simulation/latest"))
    args = parser.parse_args()
    try:
        if args.map:
            world = World(args.map.read_text(encoding="utf-8-sig").splitlines())
        elif args.scenario == "generated":
            world = generated_maze(args.seed, args.width, args.height, target_count=0 if args.no_target else args.target_count)
        else:
            if args.no_target:
                parser.error("--no-target applies only to generated mazes")
            world = World(SCENARIOS[args.scenario])
        config = Config(args.sensor_range, args.target_range, args.policy, args.explore_budget, args.max_steps)
        config.validate()
        result = run_mission(world, config)
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    result["scenario"] = {"name": "custom" if args.map else args.scenario, "seed": args.seed,
                          "source_map": str(args.map) if args.map else None}
    paths = write_artifacts(result, args.output)
    print(json.dumps({"metrics": result["metrics"], "artifacts": paths}, indent=2, ensure_ascii=False))
    return 2 if result["metrics"]["status"].startswith("failed_") else 0


if __name__ == "__main__":
    raise SystemExit(main())
