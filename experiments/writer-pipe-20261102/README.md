# Isolated writer pipe and filesystem comparison

Two closed, bounded measurements, with no ROS/Gazebo/motion. The source is run26's fifth stored window; missing sixth-window pixels were not reconstructed. Production-before files preserve the installed baseline used by both measurements.

`matrix/` contains default/1 MiB stdin pipe × drvfs/ext4 output. `reverse/` repeats the drvfs pair in reversed order. Original errors, partial results and reopened-file checks remain in each report. All helpers exited. A successful benchmark is not a repaired run26 or a successful maze mission.

Native ext4 raw files remain at the paths recorded by inputs.json; the reports were copied after helper closure. Original source RGB-D is preserved in run26's partial archive. This directory contains reports and hashes, not another duplicate set of raw images.

See ../../docs/writer-pipe-timing.md for measured values, implementation and limits. `sha256sum -c SHA256SUMS` verifies archive bytes only.

`validation/` preserves two failing full local test runs, a passing targeted diagnostic rerun, the final 36-test pass in a standalone Python environment, and the package build log. Original assertions and timeouts were unchanged. A captured readiness failure was still importing standard-library modules before entering the adapter; this does not explain every earlier timing failure.
