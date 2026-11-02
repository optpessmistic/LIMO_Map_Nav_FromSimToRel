#!/usr/bin/env bash
set -eo pipefail
cd /mnt/c/Users/Admin/.codex/worktrees/0dd9/Personal_Plan/LIMO_Map_Nav_FromSimToRel
test ! -e outputs/setup/block-cause-regression-wsl.log
test ! -e outputs/setup/block-cause-build.log
source /opt/ros/jazzy/setup.bash
source /mnt/f/study/Personal_Plan/LIMO_Map_Nav_FromSimToRel/install/local_setup.bash
source /mnt/f/study/Personal_Plan/LIMO_Map_Nav_FromSimToRel/outputs/slam-boundary-ws/install/local_setup.bash
/usr/bin/python3 -B -m unittest discover -s tests -p test_frame_retention_node.py -v 2>&1 | tee outputs/setup/block-cause-regression-wsl.log
colcon build --packages-select limo_maze_mission --allow-overriding limo_maze_mission --executor sequential 2>&1 | tee outputs/setup/block-cause-build.log
cmp src/limo_maze_mission/limo_maze_mission/node.py install/limo_maze_mission/lib/python3.12/site-packages/limo_maze_mission/node.py
