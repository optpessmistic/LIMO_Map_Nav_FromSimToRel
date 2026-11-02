# run26: closed evidence archive

Archive identity is separate from mission success, image-retention verification and complete visual search.

```json
{
  "status": "blocked",
  "reason": "frame_retention_failed",
  "mission_success": false,
  "map_complete": false,
  "visual_coverage_complete": false,
  "returned_pose_verified": false
}
```

Supervisor terminal recorded: True; orchestration status: terminal_observed.

Accepted frames: 18; original strict audit passed: False; verified count: 0; extracted audit passed: None. Missing audit is null, never a 0/0 pass. A partial-files archive, if present, preserves raw files without a passing audit claim.

Preserved producer fault: writer_wall_timeout; accepted frames with missing expected files: 3; accepted frames without a stored producer result: 3. Exact IDs and missing paths are in retained-partial-files.json and retention-audit-status.json. A strict audit can stop at the producer fault before enumerating frames; its original 0/0 counters remain failed and are separate from the mission accepted count.

Captured OccupancyGrid raster passed: True; independent terminal tail verified: True. Snapshot raster and posegraph are separate exports; this does not claim a save_map service repair. See final-save.json, snapshot-map.provenance.json when present, and orchestration.terminal_evidence_collection for exact outcomes. Missing files are listed in archive-manifest.json.

Owned groups and writer closure are required. The original global process-scan limitation remains; no global absence claim is added. Native/map return measurements have separate coordinate scopes.

Runtime production/config revision is recorded in preparation.json (b23c983). Run26 reused the run25 build; no new build or test execution is claimed. validation/ preserves the original run25 test/build/launch and CI receipts for the same recorded code revision. Run25 failed during recorder startup because epoch.txt contained CRLF; no motion or TF-performance evaluation took place. Run26 uses a fresh ASCII/LF UUID epoch with byte validation before ROS startup. The runner and saved parameters record the simulation-only TF tolerance settings and original budgets.

TF age rejection events: 0; navigation extrapolation matching log lines: 0. The summary preserves full recorded rejection fields. First-diagnostic and log-line counts are not all failures or independent incidents.

Verify this directory with `sha256sum -c SHA256SUMS`. For a passed retained-frames.tar.gz archive only:

```bash
review_dir="$(mktemp -d -t limo-run26-review.XXXXXX)"
tar -xzf retained-frames.tar.gz -C "$review_dir"
python3 -B ../../tools/audit_frame_retention.py --consumer mission/mission_20261101_231156.json --retention-root "$review_dir/retained"
```

MCAP/posegraph/data stay local with source paths, lengths and SHA256 recorded. Gzip entries decode to source hashes. Runner copies use machine-local paths. No physical LIMO validation or full target recall is implied.
