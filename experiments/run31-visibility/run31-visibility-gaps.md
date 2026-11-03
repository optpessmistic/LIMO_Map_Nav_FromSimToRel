# Run31 retained-depth gaps

Raw free cells: 2584. No nominal_clear at all three heights: 743; 11 four-neighbor components.

The upper-right and upper-left components contain 727/743 gap cells (97.85%). The remaining components are small. This is a discrete diagnostic, not search coverage.

| Component | Cells | Center bounds x / y (m) | Representative XY (m) |
|---|---:|---|---|
| 1 | 442 | [2.023, 3.573] / [-0.444, 0.306] | (2.823, -0.044) |
| 2 | 285 | [-0.327, 1.073] / [-0.344, 0.356] | (0.423, 0.006) |
| 3 | 6 | [2.373, 2.523] / [-1.994, -1.894] | (2.423, -1.944) |
| 4 | 3 | [-0.377, -0.277] / [-1.194, -1.194] | (-0.327, -1.194) |
| 5 | 1 | [3.523, 3.523] / [-1.294, -1.294] | (3.523, -1.294) |
| 6 | 1 | [0.123, 0.123] / [-1.144, -1.144] | (0.123, -1.144) |
| 7 | 1 | [2.923, 2.923] / [-0.494, -0.494] | (2.923, -0.494) |
| 8 | 1 | [2.773, 2.773] / [0.406, 0.406] | (2.773, 0.406) |
| 9 | 1 | [3.173, 3.173] / [0.406, 0.406] | (3.173, 0.406) |
| 10 | 1 | [3.373, 3.373] / [0.406, 0.406] | (3.373, 0.406) |
| 11 | 1 | [3.523, 3.523] / [0.406, 0.406] | (3.523, 0.406) |

## Two principal components

- Component 1, index 3510: each height has 24 behind-camera, 3 outside-image, 6 occluded, 0 invalid, 0 ambiguous and 0 clear frames.
  - Source frame 15: camera XY=(1.729, -1.454) m; optical-forward heading=25.3 deg from map +X toward +Y. Depth stamp=219400000000 ns. Exact identity is in JSON.
  - Source frame 16: camera XY=(1.729, -1.454) m; optical-forward heading=25.3 deg from map +X toward +Y. Depth stamp=219700000000 ns. Exact identity is in JSON.
  - Source frame 17: camera XY=(1.729, -1.454) m; optical-forward heading=25.3 deg from map +X toward +Y. Depth stamp=219800000000 ns. Exact identity is in JSON.
  - Source frame 21: camera XY=(1.666, -1.407) m; optical-forward heading=81.2 deg from map +X toward +Y. Depth stamp=229200000000 ns. Exact identity is in JSON.
  - Source frame 22: camera XY=(1.666, -1.407) m; optical-forward heading=81.2 deg from map +X toward +Y. Depth stamp=229300000000 ns. Exact identity is in JSON.
  - Source frame 23: camera XY=(1.666, -1.407) m; optical-forward heading=81.2 deg from map +X toward +Y. Depth stamp=229400000000 ns. Exact identity is in JSON.
- Component 2, index 3546: each height has 21 behind-camera, 9 outside-image, 3 occluded, 0 invalid, 0 ambiguous and 0 clear frames.
  - Source frame 12: camera XY=(1.378, -1.462) m; optical-forward heading=139.3 deg from map +X toward +Y. Depth stamp=202700000000 ns. Exact identity is in JSON.
  - Source frame 13: camera XY=(1.378, -1.462) m; optical-forward heading=139.3 deg from map +X toward +Y. Depth stamp=202800000000 ns. Exact identity is in JSON.
  - Source frame 14: camera XY=(1.378, -1.462) m; optical-forward heading=139.3 deg from map +X toward +Y. Depth stamp=203000000000 ns. Exact identity is in JSON.

## Interpretation and next observation design

The aggregate occluded label is not an all-frame obstruction claim. Most retained views of the two representative points are behind-camera or outside-image. Optical headings are derived from the quaternion-transformed camera +Z axis, not Euler yaw of an optical-frame quaternion.

Prioritize testing additional view candidates for the right upper corridor, then the left upper corridor. These representative cells describe what lacks evidence; they are not commanded robot poses. Candidate endpoints still need footprint clearance, reachable map paths and camera heading checks. No task strategy is changed here.

Only frozen layers.json.gz and report.json were read. No world or target truth was used. The nominal floor, uncalibrated uncertainty, historical map poses and discrete rays remain limitations. Depth bounds filter measured surfaces; query-point recognition range is not certified. Original mission_success remains false.

Replay with a fresh prefix: `python -B outputs/setup/run31-visibility-gaps.py --output-prefix <new-prefix>`. Existing outputs are refused.
