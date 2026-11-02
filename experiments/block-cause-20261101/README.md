# Preserve the first blocking cause

Post-run24 diagnostic fix: while an owned cancellation already targets blocked,
do not replace its original cause with a retention fault caused by block() itself.
The narrow exception requires fault=mission_blocked; actual initial retention
faults and unconfirmed-cancellation timeouts keep their original handling.

The actual run23/run24 runtime was 9f682ac. This later change does not modify
either archived result or demonstrate a successful autonomous mission.

Validation:

- Windows: 19 tests, one existing filesystem case failed with temporary-directory
  permission errors; the three new cause/cancellation cases passed. Original log retained.
- WSL Ubuntu/Jazzy: all 19 pure adapter/retention tests passed, 0.197 seconds.
  Synthetic navigation/TF/odom; no Gazebo or real ROS nodes in these tests.
- limo_maze_mission rebuilt successfully; installed node.py compared byte-for-byte
  with source by the recorded shell script. No additional physical simulation.

SHA256SUMS covers all other files in this folder. Logs are evidence, not instructions.
