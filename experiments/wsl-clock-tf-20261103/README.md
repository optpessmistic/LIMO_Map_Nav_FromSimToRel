# WSL clock and TF evidence — 2026-11-03

Two independent host-clock diagnostics were completed without creating ROS nodes, running Gazebo or changing system time. They were recorded after run29, not during the original navigation failures.

| Concurrent C++ measurement (65.001842 steady seconds) | Rate, SYSTEM_TIME | WallRate, STEADY_TIME |
|---|---:|---:|
| Recorded sleeps | 3039 | 3250 |
| Sleeps longer than 0.1 s | 2 | 0 |
| Maximum sleep | 2.356209149 s | 0.021717746 s |

SYSTEM Rate paused for 1.918409 s and 2.356209 s while the measured realtime-minus-steady offset moved backward by approximately 1.884048 s and 2.337639 s. The concurrent STEADY WallRate observed the same clock shifts and stayed near its 20 ms period. Sleep measurements include scheduler delays; the largest midpoint-clock sampling bracket was 56.881 microseconds. Complete per-sleep CSVs and the build/run logs are retained. External timeout did not trigger. The binary and full ROS/SLAM sources are not copied; the completion receipt includes their relevant build fingerprints.

The separate Python sample lasted 65.052411 monotonic seconds and recorded 6 offset shifts larger than 50 ms; its maximum sampling bracket was 87.357 microseconds. Its full sample JSON and source are included.

The closed run29 raw-bag audit contains 11 Nav2 exception windows with map-to-odom recorder gaps of 0.560–0.793 **ROS seconds**. Recorder stamps use simulation time: these are not independent wall arrival timestamps. A bag gap can reflect publishing, transport or recorder scheduling; it alone does not prove a publisher pause. The bag is referenced by its SHA256 in the audit and is not copied.

The standalone comparison supports investigating SYSTEM-clock pacing as a cause of transform-publication pauses. It does not establish a per-failure causal trace or prove that a patched SLAM runtime solves navigation. Patch validation here covers source applicability and fetch-script guards only; its original unexecuted checks remain listed. The later SLAM overlay build and any new mission are outside this archive.

`manifest.json` binds original paths and raw bytes. Verify archived files from this directory with `sha256sum -c SHA256SUMS`; archive-local `.gitattributes` disables text conversion. No source, prior experiment, system clock or Git state was modified by archiving.
