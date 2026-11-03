# Independent score promotion review

Reviewed source SHA-256: `score.py` = `4b2646d5f6b6ab99fb286e2f661a6b8121d105bd6206229436ebfa97879758d2`; `test_score.py` = `02e2f74282d6c72ea6290dba1aac0dca46b853cb3015b98b54fbffaaa2c3f468`.

## Actionable finding

**P2 — A closed deferred capture with only its first durable checkpoint aborts the whole batch audit.** `tools/red_calibration/score.py:237-249` reads `raw-checkpoint.json` whenever final `summary.json` is absent. The schema3 producer first persists `sample_*/raw.json`, raw payloads, and `collection-checkpoint.json` before entering detector/PNG work. A process failure or externally bounded termination in this phase can leave exactly that valid early evidence. Even after the outer runner confirms its owned groups are empty, `score_case` raises `FileNotFoundError`, and `score_batch` cannot produce the planned-case denominator or record this case as ran-but-ineligible.

The scorer does not fabricate zero detections here, but loses its report for an expected failure boundary introduced by deferred persistence. Existing score tests exercise missing summary only with the later raw checkpoint present; their synthetic complete captures use schema2.

Minimal fix: when both later reports are absent, accept only a valid early collection checkpoint after the existing owned-group closure gate. Verify its manifest/source/raw artifact references and retain fixed slots 1..6. Derived outputs that are not referenced by this checkpoint are not evidence. Keep processing results and detection counts null, comparison ineligible, and native cleanup/completeness unknown. Preserve batch planned and ran-but-ineligible counts. Do not fall back from a malformed existing later report.

## Independent reproduction

`outputs/setup/test_score_schema3_repro.py` uses the current real pure `FixedWindow`, `CaptureWriter`, `freeze_ready`, `persist_frozen`, and `checkpoint_then_cleanup` implementations with synthetic images and TF; it imports no ROS and launches no subprocess. A deliberate detector exception leaves the early checkpoint boundary. All test inputs are temporary; no live or historical experiment evidence is changed.

Command: `wsl -d Ubuntu-24.04 -- /usr/bin/python3 -B /mnt/c/Users/Admin/.codex/worktrees/0dd9/Personal_Plan/LIMO_Map_Nav_FromSimToRel/outputs/setup/test_score_schema3_repro.py`

Recorded results: `outputs/setup/red-calibration-score-schema3-repro.json`.

- Complete schema3: six counts `[0,0,0,0,0,0]`, integrity/comparison eligible; raw.json and collection-checkpoint artifacts are compatible with the normal audit index.
- Schema3 with third CameraInfo pair absent: counts `[0,0,null,0,0,0]`, five valid input frames, comparison ineligible. Rejected processing is not relabeled as zero or a miss.
- Early checkpoint only: single-case and 12-case planned batch both fail with missing `capture/raw-checkpoint.json`.
- `rclpy` was not imported.

No other blocking promotion problem found in scope. Existing fixed sequence and unrun-case logic retains null placeholders and only computes count relations for eligible whole captures. Hash consistency is explicitly not authenticity certification. A separate authorized ignored repair draft will address the finding; this review does not change production files.
