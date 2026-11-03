# run23: closed evidence archive

This archive preserves the recorded outcome; archive integrity is not full mission acceptance.

```json
{
  "status": "blocked",
  "reason": "operator_sigint_stop_verified",
  "mission_success": false,
  "map_complete": false,
  "visual_coverage_complete": false,
  "returned_pose_verified": false
}
```

Accepted frames: **0**; producer results: **0**. The supervisor did not record a normal terminal result, so no strict retention audit was generated. This is **not a 0/0 audit pass**; no image package or replacement audit is fabricated.

No final-save or snapshot-map export was reached in this failed run. Missing optional evidence remains listed in archive-manifest.json.

Owned groups and writer closure are required. The original global process-scan limitation is preserved; no global absence claim is added. Native/map return measurements have separate coordinate scopes.

From this directory, verify copied artifact identity with `sha256sum -c SHA256SUMS`. Archive identity checks are independent of mission and retention audit outcomes.

The early parameter capture started at 2026-10-31 13:27:02.956911 UTC. Its mission list request was sent at wall +5.467660993 s, approximately 13:27:08.424572 UTC. The mission loaded log timestamp 1793453228.700346545 is 13:27:08.700346 UTC: the request preceded that log by about 0.276 s. mission.log also contains `rclpy failed to send response (timeout)`. This supports a startup/service-response timing hypothesis; the exact DDS cause remains unproven. The capture failed with `parameters_unavailable:limo_maze_mission`, then supervisor SIGINT canceled one frontier goal.

A proposed next run will wait for the Mission loaded log before starting parameter capture; the per-request 6 s, total 17 s and original experiment budgets stay unchanged. This archive does not validate that proposal.

No final snapshot/raster export or independent terminal-tail verification was reached. A confirmed red-surrogate annotation is preserved, but no completed search or return is claimed.

MCAP/posegraph/data remain local; only their paths, sizes and SHA256 are retained. Gzip entries must decode to their recorded source hashes. Runner copies use local paths and are evidence, not a generic launcher. No physical LIMO validation is implied.
