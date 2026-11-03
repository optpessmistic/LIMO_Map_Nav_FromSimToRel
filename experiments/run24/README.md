# run24: closed evidence archive

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

Accepted frames: 0; original strict audit passed: False; verified count: 0; extracted audit passed: None. Missing audit is null, never a 0/0 pass. A partial-files archive, if present, preserves raw files without a passing audit claim.

Captured OccupancyGrid raster passed: True; independent terminal tail verified: True. Snapshot raster and posegraph are separate exports; this does not claim a save_map service repair. See final-save.json, snapshot-map.provenance.json when present, and orchestration.terminal_evidence_collection for exact outcomes. Missing files are listed in archive-manifest.json.

Owned groups and writer closure are required. The original global process-scan limitation remains; no global absence claim is added. Native/map return measurements have separate coordinate scopes.

Production build/test/CI evidence retains its original run23 filenames under validation/, for code 9f682ac; it is not a new build/test run. The runner records the actual run24 startup gate and original budgets.

Verify this directory with `sha256sum -c SHA256SUMS`. For a passed retained-frames.tar.gz archive only:

```bash
review_dir="$(mktemp -d -t limo-run24-review.XXXXXX)"
tar -xzf retained-frames.tar.gz -C "$review_dir"
python3 -B ../../tools/audit_frame_retention.py --consumer mission/mission_20261031_063652.json --retention-root "$review_dir/retained"
```

MCAP/posegraph/data stay local with source paths, lengths and SHA256 recorded. Gzip entries decode to source hashes. Runner copies use machine-local paths. No physical LIMO validation or full target recall is implied.
