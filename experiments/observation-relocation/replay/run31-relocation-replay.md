# Run31 production queue relocation replay

Replay checks passed: True. No robot/simulation ran.

Both independent queues are rebuilt from 0032; all 12 frozen views and original order must match the archived reconstruction. Five recorded inspection_satisfied outcomes construct history only.

| Variant | Remaining dispatches | Remaining skips | Final state counts |
|---|---:|---:|---|
| default_disabled | 0 | 7 | {'completed': 5, 'skipped_unsafe_or_unreachable': 7} |
| enabled_0_15m | 7 | 0 | {'completed': 5, 'failed': 7} |

The 0.15 m case does not claim seven new successes. Each selected dispatch is deliberately finished canceled to enumerate the next action, while the 0058 map and actual saved pose stay fixed. Only the original five completed records remain completed. All 12 planned action IDs, original anchors, yaw/heading sets and safety/budget parameters remain bound in JSON.

## Hypothetical selected goals

| Action | Station / heading | Selected XY (m) | Shift from frozen anchor (m) |
|---|---|---|---:|
| observation-0003 | 2 / 0 | (0.373133, 0.005738) | 0.042460833 |
| observation-0006 | 2 / 1 | (0.373133, 0.005738) | 0.042460833 |
| observation-0012 | 2 / 2 | (0.373133, 0.005738) | 0.042460833 |
| observation-0011 | 1 / 2 | (3.123133, -0.044262) | 0.057539168 |
| observation-0005 | 1 / 1 | (3.123133, -0.044262) | 0.057539168 |
| observation-0002 | 1 / 0 | (3.123133, -0.044262) | 0.057539168 |
| observation-0008 | 1 / 3 | (3.123133, -0.044262) | 0.057539168 |

Decision validation includes source map geometry/hash/stamp, unchanged current pose, original/preferred/selected endpoint checks and effective safety radii. Default skipped decisions are retained from the production selection history. No world or target truth is read.

Original mission_success and visual_coverage_complete remain false. Offline selection feasibility does not prove Nav2 arrival, observation completion or usable camera range.

Replay: `python -B outputs/setup/run31-relocation-replay.py --output-prefix <fresh-prefix>`. Existing outputs are refused.
