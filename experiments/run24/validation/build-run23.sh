#!/usr/bin/env bash
set -eo pipefail
cd /mnt/c/Users/Admin/.codex/worktrees/0dd9/Personal_Plan/LIMO_Map_Nav_FromSimToRel
for report in run23-terminal-tests.log run23-build.log run23-build-installed.json; do
  if [[ -e "outputs/setup/$report" ]]; then
    printf 'Refuse existing build evidence: %s\n' "$report" >&2
    exit 1
  fi
done
source /opt/ros/jazzy/setup.bash
source /mnt/f/study/Personal_Plan/LIMO_Map_Nav_FromSimToRel/install/local_setup.bash
source /mnt/f/study/Personal_Plan/LIMO_Map_Nav_FromSimToRel/outputs/slam-boundary-ws/install/local_setup.bash
/usr/bin/python3 -B -m unittest tools.test_terminal_evidence -v 2>&1 | tee outputs/setup/run23-terminal-tests.log
colcon build --packages-select limo_maze_mission --allow-overriding limo_maze_mission --executor sequential 2>&1 | tee outputs/setup/run23-build.log
source install/local_setup.bash
/usr/bin/python3 -B outputs/setup/verify-run23-build-installed.py
