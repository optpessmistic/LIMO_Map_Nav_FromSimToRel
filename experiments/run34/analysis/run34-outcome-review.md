# Run34 closed outcome review

Original flags: {"status": "blocked", "reason": "inspection_wall_timeout_incomplete", "mission_success": false, "returned_pose_verified": false, "map_complete": false, "visual_coverage_complete": false, "mission_timed_out": false, "target_definition_verified": false, "coordinates_valid": true}

Initial survey (diagnostic only): {"phase": "done", "success": true, "reason": "spin_and_odometry_rotation_heading_stop_verified", "motion_started": true}.
Analysis applicability: {"hash_bound_mission_terminal_present": true, "observation_queue_recorded": false, "selection_terminal_and_queue_present": false, "strict_retention_audit_file_present": true, "scope": "Selection audit remains a separate strict check. Missing terminal leaves mission statistics unknown and does not prove the mission process never started. A missing audit file does not establish a pass or successful zero-frame audit."}

| Navigation kind | Sent | Success | Failure | Unknown terminal |
|---|---:|---:|---:|---:|
| frontier | 4 | 4 | 0 | 0 |

Queue created=False; completed=None / actual planned=None; skipped=None, failed=None, pending=None, active=None.
Shifted goal sends=0; their recorded inspection completions=0. These are not measured vehicle displacement or whole-scene coverage.

| Action | Station/heading | XY shift (m) | Goal sent | Outcome |
|---|---|---:|---|---|

Targets annotated=1; target recall is not evaluated.
Native sampled home-to-final: {"xy_distance_m": 1.8242011700240854, "yaw_delta_rad": -2.244837151574595}.
Consumer accepted=9; original strict audit passed=False, verified=0 / audit accepted=0.
TF log counts: {"navigation.log": {"extrapolation": 0, "stale_base_tf": 0, "failed_transform": 0, "target_observation_camera_frame_unavailable": 0}, "mission.log": {"extrapolation": 4, "stale_base_tf": 0, "failed_transform": 0, "target_observation_camera_frame_unavailable": 0}}.
Root22438 exit=1; original owned closure=True; native terminal tail=True.
Original orchestration errors: ["Retained-frame offline audit failed"]
Missing stays null/None; neither terminal_observed nor a configured 12-view limit establishes mission success. Full raw flags, per-dispatch selection map/hash and input SHA256 bindings are in the JSON.
