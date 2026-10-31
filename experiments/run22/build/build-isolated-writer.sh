#!/usr/bin/env bash
set -eo pipefail
cd /mnt/c/Users/Admin/.codex/worktrees/0dd9/Personal_Plan/LIMO_Map_Nav_FromSimToRel
source /opt/ros/jazzy/setup.bash
source /mnt/f/study/Personal_Plan/LIMO_Map_Nav_FromSimToRel/install/local_setup.bash
source /mnt/f/study/Personal_Plan/LIMO_Map_Nav_FromSimToRel/outputs/slam-boundary-ws/install/local_setup.bash
colcon build --packages-select limo_maze_perception --allow-overriding limo_maze_perception --executor sequential 2>&1 | tee outputs/setup/isolated-writer-build.log
source install/local_setup.bash
/usr/bin/python3 outputs/setup/verify-isolated-writer-installed.py
