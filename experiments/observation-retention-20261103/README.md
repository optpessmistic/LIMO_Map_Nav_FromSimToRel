# Observation routing and retired-window receipt validation

Frozen observation actions are reordered by station and known-safe grid path length. Retired windows may preserve valid storage receipts received before their original deadline, without restoring observation credit or changing the replacement window.

See results.json and original validation logs. The pure suite has 153 passes and 3 ROS skips; the three skipped Action/DDS cases passed separately. Two real retention DDS cases and 14 real-file audit regressions also passed. Mission build and installed-source hashes are retained. The real DDS fixture uses synthetic blank RGB-D, exact synthetic TF and a fixed pose, not robot motion.

dds-originals.tar.gz contains both complete test directories, including six actual accepted RGB-D pairs, metadata, final producer checkpoints, consumer snapshots and cleanup records. Every member was compared byte for byte; its size and SHA256 are in dds-originals-manifest.json. The retired test controls admission/worker order by pausing the producer timer before accepted requests are queued, then restoring it after retirement; it does not fabricate stored receipts or extend the original inspection deadline.

Source snapshots bind these checks to the precommit working files; parent_revision alone does not identify the changed code. Historical run30 remains 33/36 and is not regraded. This archive does not claim full search coverage or mission success.

Verify archive files using sha256sum -c SHA256SUMS; tar members have their own manifest.
