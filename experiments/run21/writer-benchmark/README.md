# Run21 retained writer: one baseline and one candidate measurement

This folder contains two isolated replays of the three exact RGB-D pairs and requests accepted by the failed run21 mission. It does not change that mission result. No ROS nodes, Gazebo processes, tests, or production edits were made for this comparison.

The baseline uses the installed `OwnedWriter`. The candidate changes only its child launch to `python -S -B` with `PYTHONPATH` restricted to the resolved installed `limo_evidence_protocol` package parent. Both use the same inherited Jazzy/F/C overlay environment, actual 640×480 RGB8/32FC1 bytes, actual request offsets, a 50 ms owner polling interval, 1 MiB transfer ceiling per poll, worker deadline 5 s, request deadline 10 s, and shutdown deadline 8 s. Initial empty-checkpoint persistence happens before each timed three-request workload, as it did before the real mission window. Each variant has an exclusive new output root.

Exactly one baseline attempt followed by exactly one candidate attempt was run. The outer exec session 97297 terminated with exit code 1 because the baseline failed; neither attempt was retried or overwritten.

| Measurement | Baseline | Candidate `-S` |
|---|---:|---:|
| Initial checkpoint worker lifetime | 2.311 s | 0.614 s |
| Job 1 worker lifetime | 4.962 s | 1.555 s |
| Job 2 worker lifetime | 3.563 s | 4.091 s |
| Job 3 worker lifetime | Aborted after 2.078 s | 2.335 s |
| All three durable receipts | Failed: third request timeout | 9.204 s |
| Separate final receipt checkpoint | Not reached | 1.219 s |
| Shutdown elapsed | 2.078 s | 0.823 s |
| All owned helper children exited | Yes | Yes |
| Reopened original bytes and request/receipt/manifest hashes | First two verified, third failed | All three verified |

The candidate's durable completion precedes the original mission's remaining 10.046904 s from its first request by about 0.843 s. This is a replay comparison, not a claim that the candidate would have passed the real mission under its actual load.

The small launch change is worth advancing to a narrowly scoped real-runtime validation. The margin remains small. Ordering, filesystem cache, host scheduling, and absence of concurrent image processing/Gazebo load are uncontrolled; this experiment does not prove repeatable speedup or identify interpreter startup as the sole bottleneck. The second candidate job still took 4.091 s.

Raw per-poll byte counts and timings are in each `report.json`, including constructor duration, input-drained time, terminal-observed time, all checkpoints, and shutdown. Constructor duration measures `Popen`, not interpreter readiness. Input drain includes scheduling and pipe backpressure; there is no child-side startup/import/fsync phase timing. The baseline job 1 sent at most 393216 B in any poll; candidate jobs ranged from 65536 B to the 1 MiB ceiling. The ceiling is not guaranteed throughput.

`inputs.json` records input and imported module hashes. `comparison.json` confirms those three production module hashes stayed unchanged. The candidate's separate `source_bytes_verification` checks all six files by reopening them, exact byte equality for source RGB/depth/producer/CameraInfo/request, and receipt/manifest identity. Manifest timestamps intentionally describe this replay, not the original run. Both shutdown reports confirm the final checkpoint and child exit; the baseline retains its request failure.

The runner and all 11 owned helper subprocesses exited. No ROS/Gazebo processes were started, and there are no benchmark helper sessions to resume.
