# Run22: remaining frontier and map-export diagnosis

Run22 remains **full mission false** and **map complete false**, with verified return. Its strict retained-frame audit remains **15 accepted / 15 verified, passed**. This diagnosis uses frozen observed grids and the actual mission core; it does not use world truth or change runtime behavior. Machine-readable geometry, source hashes, parameters, and evidence paths are in `run22-followup-diagnosis.json`.

## Why exploration stopped

Five frontier actions succeeded. The actual selection result is reproducible with the original robot radius 0.25 m, endpoint margin 0.05 m, standoff 0.55 m, and minimum translation 0.20 m. At approximately 0.05 m grid resolution, the endpoint safety radius including the cell diagonal is 0.335355 m.

| Frozen grid | Remaining frontiers | Conservative reachable viewpoints |
| --- | --- | --- |
| `0033-phase-canceling.json`, map stamp 198.625 s | One cluster of 3 cells: (78,32), (79,32), (80,32) | No endpoint-safe reachable cell anywhere has the required clear ray to any anchor. The return decision is explained by the existing safety/visibility rules. |
| `0038-phase-done.json` and valid `final-save.json` grid | One cluster of 3 cells, shifted to row 31 | The first two anchors still have no safe visible viewpoint. The third has 4, all outside the fixed 0.55 m search distance. |

On the final grid, the nearest missed viewpoint is cell **(74,41)**, position **(3.208949, -0.103464) m**, **0.583095 m** from frontier anchor (80,31). It has a known-safe connected route of 66 grid steps, passes the unchanged endpoint radius and conservative visibility checks, and exceeds the minimum translation. This establishes a valid candidate in the planner's grid model, not a tested Nav2 route or proof that one visit will complete the map.

The frontier's neighboring unknown region touches the grid boundary: 518 connected unknown cells at the return trigger, 495 in the final grid. It is therefore unjustified to discard these three cells as insignificant enclosed holes or to declare the environment fully explored. The original conservative rejection is expected from its frozen map, but the later map demonstrates a missed candidate beyond the fixed search distance.

**Smallest future planner change:** add a bounded wider-viewpoint fallback while preserving footprint radius, arrival margin, known-free connectivity, translation, and conservative line-of-sight checks. Increasing the search distance alone would not have changed the original return-trigger snapshot, which had no visible safe endpoint at any distance. Runtime validation is still needed.

## Why PGM/YAML export and the terminal tail failed

`final-save.json` contains a valid fresh OccupancyGrid snapshot: map stamp 271.0 s, capture 271.417 s, map receipt age about 0.556 wall seconds, and fresh TF. `/slam_toolbox/save_map` nevertheless returned **255**. `navigation.log` records `Failed to spin map subscription` approximately **2.005 s** after the spawned map saver began saving. No PGM/YAML was written; `/slam_toolbox/serialize_map` subsequently returned 0 and produced nonempty `.data` and `.posegraph` files.

The local SLAM implementation invokes a new `map_saver_cli` process with transient-local subscription enabled and no timeout override. [Nav2 Jazzy's map-saver source](https://raw.githubusercontent.com/ros-navigation/navigation2/jazzy/nav2_map_server/src/map_saver/map_saver.cpp) corroborates a default 2.0 s subscription timeout and this error before raster-file writing. The fetched upstream branch is corroborating source, not a claim of byte-identical installed code. The failure is consistent with the new subscriber failing to receive/process a map inside its short window; available evidence does not distinguish DDS discovery, delivery, or scheduling. It does not establish that SLAM had no map or that filesystem writing failed. The save helper itself finished in about 16.066 s, within its 45 s budget.

The supervisor calls `save-final` before its independent terminal-tail loop. Its nonzero exit raises immediately, enters cleanup, and bypasses that loop. The remaining observer window spans 246.469–276.436 simulation seconds and includes the preceding return rotation; it is not wholly post-terminal. Its stationarity failure cannot establish a new post-terminal motion fault.

**Smallest future export/supervisor changes:** preserve the export failure, but still collect the original bounded independent terminal tail before cleanup. Export a separately named PGM/YAML from the already captured valid immutable OccupancyGrid, with explicit snapshot provenance, thresholds, origin, and reopened-file validation; do not relabel the failed service call as successful. A single owned CLI invocation with an explicit bounded map-subscription timeout is an alternative for a subsequent run, not evidence that this run exported successfully. Preserve existing overall budgets and ownership checks.

## Scope and execution

The bounded offline core replay completed successfully (session 17656, exit 0); its analysis took 0.1254 s excluding process startup. No ROS/Gazebo node was started, no runtime or production file was edited, and no safety or success criterion was lowered. Only this report and its accompanying JSON were written. Root reports all run22 owned process groups and the writer closed.
