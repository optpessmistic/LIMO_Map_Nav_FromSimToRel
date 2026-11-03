# Run31 station relocation feasibility

0058 observer ROS=278.9 s, map stamp=278.25 s. Unchanged endpoint reserve=0.335355339586 m = robot 0.25 + arrival 0.05 + half-cell diagonal.

| Station | Safe reachable centers within 0.20 m | Minimum shift | Candidate XY (m) | Safe-grid path |
|---|---:|---:|---|---|
| 1 | 3 | 0.057539 m | (3.123133, -0.044262) | 56 steps / 2.800 m |
| 2 | 4 | 0.042461 m | (0.373133, 0.005738) | 0 steps / 0.000 m |

Both original frozen endpoints fail the unchanged endpoint test on this snapshot. The substitutes are actual raw-map cell centers and pass the same production safety checks; their zero origin-offset reserve is geometric recentering, not a reduced robot/arrival margin.

Station 1: remaining heading 2 (-pi) points exactly toward the right-gap representative, 0.30 m from the selected base center. Station 2: remaining heading 0 (0 rad) points toward the left-gap representative, but that point is only 0.05 m away. This does not certify a usable camera or recognition distance. Both planar supercover rays are free on 0058.

Station 2 is already in the current pose cell; its actual pose-to-center distance is 0.022691 m. Zero BFS steps do not prove zero motion or successful settling.

Applicability: a bounded same-station XY replacement has map-geometric candidates without loosening safety or adding headings. It is worth testing with fresh dispatch checks and actual arrival/frame evidence. These outputs do not alter the queue, command motion, credit actions or establish visibility/search completion.

Scope: current core/observation APIs, saved 0058 raw map, archived station reconstruction, and prior frozen map-gap diagnostics only. No world or target truth. Snapshot timing and historical map-pose limits remain; full candidate coordinates, sources and one safe-grid path witness per selected station are retained in JSON.

Replay with a fresh prefix: `python -B outputs/setup/run31-station-relocation-probe.py --output-prefix <new-prefix>`. Existing outputs are refused.
