"""Repeatable baseline benchmark and linked HTML index. No external packages."""
from __future__ import annotations

import argparse
import csv
import html
import json
from pathlib import Path

from .core import Config, SCENARIOS, World, generated_maze, run_mission
from .render import write_artifacts


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2, 3, 4])
    parser.add_argument("--output", type=Path, default=Path("outputs/simulation/suite"))
    args = parser.parse_args()
    cases = [(f"seed_{seed}_full", generated_maze(seed, target_count=3), Config(), "found_explored_returned") for seed in args.seeds]
    cases += [
        ("seed_0_find_then_return", generated_maze(0, target_count=3), Config(policy="find_then_return"), "found_returned"),
        ("single_target_full", generated_maze(0, target_count=1), Config(), "found_explored_returned"),
        ("zero_targets_full", generated_maze(0, target_count=0), Config(), "not_found_explored_returned"),
        ("multi_branch_full", World(SCENARIOS["multi_branch"]), Config(), "found_explored_returned"),
        ("multi_unreachable_full", World(SCENARIOS["multi_unreachable"]), Config(), "found_explored_returned"),
        ("absent_full", World(SCENARIOS["absent"]), Config(), "not_found_explored_returned"),
        ("unreachable_full", World(SCENARIOS["unreachable"]), Config(), "not_found_explored_returned"),
        ("occluded_full", World(SCENARIOS["occluded"]), Config(), "found_explored_returned"),
        ("exploration_timeout", generated_maze(0), Config(explore_budget=10), "search_incomplete_returned"),
        ("total_budget_return", generated_maze(0), Config(max_steps=25), "search_incomplete_returned"),
    ]
    summary = []
    for name, world, config, expected in cases:
        result = run_mission(world, config)
        result["scenario"] = {"name": name}
        write_artifacts(result, args.output / name)
        metrics = result["metrics"]
        passed = metrics["status"] == expected and metrics["returned_to_start"] and metrics["steps"] <= config.max_steps
        if config.policy == "explore_then_return" and expected.endswith("explored_returned"):
            passed &= metrics["reachable_free_coverage"] == 1 and metrics["searched_reachable_free_coverage"] == 1
            passed &= metrics["all_reachable_targets_detected_ground_truth"]
        summary.append({"case": name, "passed": passed, **metrics})
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    with (args.output / "summary.csv").open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=summary[0].keys())
        writer.writeheader()
        writer.writerows(summary)
    header = "<tr><th>案例</th><th>状态</th><th>移动步数</th><th>可达空闲格覆盖</th><th>目标搜索覆盖</th><th>发现/可达/真值目标数</th><th>目标坐标</th><th>返航</th><th>检查</th></tr>"
    rows = []
    for m in summary:
        rows.append(f'<tr><td><a href="{m["case"]}/report.html">{html.escape(m["case"])}</a></td><td>{m["status"]}</td><td>{m["steps"]}</td><td>{m["reachable_free_coverage"]:.1%}</td><td>{m["searched_reachable_free_coverage"]:.1%}</td><td>{m["detected_target_count"]}/{m["reachable_target_count_ground_truth"]}/{m["target_count_ground_truth"]}</td><td>{html.escape(str(m["detected_targets_map_grid_xy"]))}</td><td>{m["returned_to_start"]}</td><td>{"PASS" if m["passed"] else "FAIL"}</td></tr>')
    page = '<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>LIMO 仿真批量结果</title><style>body{font:16px/1.6 system-ui;margin:32px;color:#1e293b}table{border-collapse:collapse}td,th{text-align:left;border-bottom:1px solid #cbd5e1;padding:10px}a{color:#1d4ed8}</style><h1>LIMO 二维任务逻辑基线</h1><p>默认：完整探索与目标搜索，标注目标格网坐标，然后返回起点。下表是理想静态格网结果，不是 SLAM、Gazebo 或实车结果。坐标单位为格，原点左上，x 向右、y 向下；物品不拿取。</p><p>无目标和不可达目标均不得报告找到；预算不足只报告搜索未完成。覆盖指标使用真值离线计算，不参与探索决策。</p><table>' + header + "".join(rows) + '</table><p><a href="summary.json">JSON 汇总</a> · <a href="summary.csv">CSV 汇总</a></p></html>'
    (args.output / "index.html").write_text(page, encoding="utf-8")
    print(json.dumps({"cases": len(summary), "passed": sum(m["passed"] for m in summary),
                      "index": str((args.output / "index.html").resolve()),
                      "summary": [{"case": m["case"], "status": m["status"], "steps": m["steps"],
                                   "coverage": m["reachable_free_coverage"], "targets_detected": m["detected_target_count"],
                                   "targets_ground_truth": m["target_count_ground_truth"]} for m in summary]},
                     ensure_ascii=False, indent=2))
    return 0 if all(m["passed"] for m in summary) else 1


if __name__ == "__main__":
    raise SystemExit(main())
