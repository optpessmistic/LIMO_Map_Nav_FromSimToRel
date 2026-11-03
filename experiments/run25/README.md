# run25: startup failure before the autonomous mission

Runtime code `b23c983d724a1c45a92350436991589cad541bd6`; orchestration failed after 155.798 wall seconds. The recorder rejected the map epoch before the survey. No mission result, retained-frame audit, map export or terminal-tail result exists.

`epoch.txt` is preserved byte-for-byte: 38 bytes ending `0D 0A`. Bash `$(cat ...)` removes the LF and leaves the CR. `record_bounded.Config.validate()` rejects the resulting surrounding whitespace before making the recording directory or launching rosbag. The frozen runtime source hash, recorder exit 2 and diagnostic support this prelaunch failure.

All 10 directly recorded owned groups have empty/complete stop checks. The original aggregate `all_owned_groups_empty=false` remains unchanged because the nested recorder supervisor report was never created. The original global token scan also stays false: no readable matching process remained, but two /proc environments were unreadable. This archive does not infer global process absence.

Perception received SIGINT during ros2cli/rclpy import and exited -2. Its traceback and absent retention directory support that detector/writer startup had not been reached; there is no writer checkpoint or exit receipt, so no verified writer shutdown is claimed. The empty mission directory and absent mission/survey/motion process records are preserved as observations, not synthetic successful results.

The TF tolerance experiment was not exercised by autonomous motion. TF age events and accepted-frame counts are unavailable, not fabricated zero-frame successes. Existing navigation extrapolation log-line counts appear in startup-failure-review.json.

Verify copied bytes with `sha256sum -c SHA256SUMS`. Archive verification concerns file identity only. Raw epoch CRLF, errors, false flags, logs and missing-artifact observations are retained. Validation logs document prior tests/builds; they do not turn this failed runtime into a pass. No new ROS processes, tests or simulator were started by the archive builder.
