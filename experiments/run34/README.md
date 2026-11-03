# Run34: lifecycle readiness and incomplete inspection

Runtime commit: `c411fcbe86c1c27ce302593b73fba405a603e002`; epoch: `ea53a5de-b919-4f52-aa28-ea6d8ec571c8`. The actual root session `22438` closed with exit `1` after `602.692668666` wall seconds. The original failure is preserved.

The initial survey completed after the bounded lifecycle-readiness change. All four dispatched frontier navigation actions reported success. The mission then became `blocked` with `inspection_wall_timeout_incomplete`; `mission_success`, `returned_pose_verified`, `map_complete` and `visual_coverage_complete` are all false. The observation queue was never created, so its planned denominator is **null**, not 0/12. One target annotation is recorded; this is not a target-recall result. See [the closed outcome](analysis/run34-outcome-review.json) and [its short report](analysis/run34-outcome-review.md).

The consumer ledger records **9 accepted and 9 stored RGB-D pairs**, with fault `mission_blocked`. The original strict retained-file audit is **failed**, with `accepted_count=0`, `verified_count=0` and `consumer_disabled_or_faulted`. These audit counts do not erase the consumer ledger or establish a successful 0/0 audit. Both raw reports are archived unchanged. Selection/bag analysis was not run for this archive.

Frozen-snapshot raster export and the independent terminal stationary tail passed. Exporting the observed map does not establish complete exploration, and stationary tail evidence does not establish return home. All launched owned groups, the nested recorder and the writer were confirmed closed. Global run-token absence remains false/uncertain because some same-UID process environments were unreadable; no global no-process claim is made.

The independent clock probe (session `34124`, exit 0) sampled 65.001 seconds during navigation startup and recorded one SYSTEM-minus-STEADY offset change of about -3.336 seconds. It does not cover the entire mission or establish a cause for this failure. Original build/install receipts and the earlier failed, unexecuted incremental-build attempt remain separate.

Raw reports, configuration, runtime helpers, logs, numbered map snapshots, map export and the outcome are included. `observer.json.gz` and `initial-motion.json.gz` preserve exact original bytes after decompression (gzip mtime=0). Copying and reopening verified 108 raw/gzip payloads. [external-inputs.json](external-inputs.json) lists actual sizes and SHA256 hashes for 57 excluded files totaling 45,048,526 bytes: one MCAP, two posegraph/data files and 54 retained-frame files. This is a compact evidence archive, not a complete replay/image bundle.

`SHA256SUMS` covers every archive file except itself; verify from this directory with `sha256sum -c SHA256SUMS`. Successful archive verification is not successful mission validation. Work stops after this round as requested; no next experiment is launched by these artifacts.
