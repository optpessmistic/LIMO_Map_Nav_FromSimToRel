# Navigation recovery validation — 2026-11-03

The mission gains a bounded wall-clock wait after failed navigation before another navigation request. This archive records the code-level validation; it does not claim a completed Gazebo maze mission.

| Check | Recorded result |
|---|---|
| New pure recovery tests | 18/18 passed |
| Existing pure regressions | 168 run: 167 passed, 1 ROS startup test skipped |
| Real DDS, paused ROS clock | 1/1 passed; recovery reaches its wall-clock timeout |
| Separate real ROS startup | 1/1 passed; autonomy disabled |
| Build | `limo_maze_mission`: one package finished |
| Installation | Node and launch bytes equal the six-file source binding where applicable |

The pure regression log retains its original skip. The separate ROS startup result does not replace or edit it. Exact test durations, root completion receipts and limitations are in `results.json`.

`source-binding.json` hashes all six reviewed source/test files without duplicating the full node. The installed launch defaults remain cooldown `0.0` s and recovery timeout `3.0` s. A future run29 is planned to override the cooldown to `1.0` s; this archive contains no run29 result.

Raw logs and the launch-argument receipt are under `validation/`. Original shell wrappers and this archiver are frozen under `runner/` as `.txt`; restore their original filenames before reuse in the repository. They are evidence, not automatically executed. The original verify wrapper also exports the existing run28 annotated map, so its full invocation needs that prior run's files.

`manifest.json` records copied-input hashes and excludes itself. `SHA256SUMS` covers every archived file except itself; verify from this directory with `sha256sum -c SHA256SUMS`. Archive-local `.gitattributes` preserves raw bytes through Git. No source, log or prior experiment is overwritten.
