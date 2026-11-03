# run28: closed evidence archive

Archive identity is separate from mission success, image-retention verification and complete visual search.

```json
{
  "status": "blocked",
  "reason": "return_navigation_failed",
  "mission_success": false,
  "map_complete": false,
  "visual_coverage_complete": false,
  "returned_pose_verified": false
}
```

Supervisor terminal recorded: True; orchestration status: terminal_observed.

Accepted frames: 3; original strict audit passed: False; verified count: 0; extracted audit passed: None. Missing audit is null, never a 0/0 pass. A partial-files archive, if present, preserves raw files without a passing audit claim.

Preserved producer fault: None; accepted frames with missing expected files: 0; accepted frames without a stored producer result: 0. Exact IDs and missing paths are in the outcome JSON, retention-audit-status.json and retained-partial-files.json when present. A strict audit can stop at the producer fault before enumerating frames; its original 0/0 counters remain failed and are separate from the mission accepted count.

Captured OccupancyGrid raster passed: True; independent terminal tail verified: True. Snapshot raster and posegraph are separate exports; this does not claim a save_map service repair. See final-save.json, snapshot-map.provenance.json when present, and orchestration.terminal_evidence_collection for exact outcomes. Missing files are listed in archive-manifest.json.

Owned groups and writer closure are required. The original global process-scan limitation remains; no global absence claim is added. Native/map return measurements have separate coordinate scopes.

Runtime production/config revision: e8e72e89914a20c6be28d7a5f98b67d524861a4d. The original preparation, runner, live parameters and installed-check preserve the run profile. validation/ contains available test/build/CI receipts and isolated benchmark summaries; absent receipts remain missing. Writer-pipe receipts are historical baseline evidence; run28-timing-tests logs and verify-run28 scripts document the collector timing change. The collector executes directly from source, with its bytes archived separately. Those source bytes are checked against the required preparation SHA and all available runtime/provenance hashes; collector_source_binding in the manifest records comparisons and unavailable optional hashes. This archive does not execute tests, rebuild, or infer validation success from file presence.

Motion startup parent status: observed_within_budget; requested budget seconds: 60; ordered finite timestamps confirm the parent observation within its recorded budget: True. The outcome JSON startup_timing preserves raw launch, parent wait and collector startup fields plus individual intervals. All recognized timestamps use same-run CLOCK_MONOTONIC, not ROS simulation time. The environment timestamp is the lightweight wrapper entry observation and includes wrapper startup; main_enter excludes interpreter/top-level imports. Parent marker observation is distinct from child loop observation and marker emission. Gate-write samples bracket release; negative, missing or invalid intervals are never clamped to zero or counted as success. Startup timing is diagnostic evidence only, not a motion or mission acceptance test.

Recorded unique writer operations: 3; actual pipe capacities: {'1048576': 3}; stop reasons: {'None': 3}. See outcome JSON writer_transport for each stdin_pipe/input_transfer record, including remaining bytes at stop. Configured writer/request/shutdown budgets as recorded: {'requested_pipe_bytes': 1048576, 'max_bytes_per_poll': 1048576, 'worker_wall_seconds': 5.0, 'request_wall_seconds': 10.0, 'shutdown_wall_seconds': 8.0}. The 5/10/8-second production limits are unchanged by startup timing; archive diagnostics do not extend them. The bounded history may omit older jobs and successful checkpoint workers; owner send times do not isolate child imports or fsync.

TF age rejection events: 0; navigation extrapolation matching log lines: 5. The summary preserves full recorded rejection fields. First-diagnostic and log-line counts are not all failures or independent incidents.

Verify this directory with `sha256sum -c SHA256SUMS`. For a passed retained-frames.tar.gz archive only:

```bash
review_dir="$(mktemp -d -t limo-run28-review.XXXXXX)"
tar -xzf retained-frames.tar.gz -C "$review_dir"
python3 -B ../../tools/audit_frame_retention.py --consumer mission/mission_20261102_233856.json --retention-root "$review_dir/retained"
```

MCAP/posegraph/data stay local with source paths, lengths and SHA256 recorded. Gzip entries decode to source hashes. Runner copies use machine-local paths. No physical LIMO validation or full target recall is implied.
