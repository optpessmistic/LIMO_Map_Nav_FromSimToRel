# Run32 closed outcome review

Original flags: {"status": "blocked", "reason": "map_stale", "mission_success": false, "returned_pose_verified": false, "map_complete": false, "visual_coverage_complete": false, "mission_timed_out": false, "target_definition_verified": false, "coordinates_valid": true}

| Navigation kind | Sent | Success | Failure | Unknown terminal |
|---|---:|---:|---:|---:|
| frontier | 2 | 1 | 1 | 0 |

Queue created=False; completed=None / actual planned=None; skipped=None, failed=None, pending=None, active=None.
Shifted goal sends=0; their recorded inspection completions=0. These are not measured vehicle displacement or whole-scene coverage.

| Action | Station/heading | XY shift (m) | Goal sent | Outcome |
|---|---|---:|---|---|

Targets annotated=1; target recall is not evaluated.
Native sampled home-to-final: {"xy_distance_m": 1.936352088067819, "yaw_delta_rad": -1.8508267422364348}.
Consumer accepted=3; original strict audit passed=False, verified=0 / audit accepted=0.
TF log counts: {"navigation.log": {"extrapolation": 0, "stale_base_tf": 0, "failed_transform": 0, "target_observation_camera_frame_unavailable": 0}, "mission.log": {"extrapolation": 0, "stale_base_tf": 0, "failed_transform": 0, "target_observation_camera_frame_unavailable": 5}}.
Root74954 exit=1; original owned closure=True; native terminal tail=True.
Original orchestration errors: ["Retained-frame offline audit failed"]
Missing stays null/None; neither terminal_observed nor a configured 12-view limit establishes mission success. Full raw flags, per-dispatch selection map/hash and input SHA256 bindings are in the JSON.
