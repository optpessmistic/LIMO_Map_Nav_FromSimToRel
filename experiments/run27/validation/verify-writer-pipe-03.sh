#!/usr/bin/env bash
set -eo pipefail
cd /mnt/c/Users/Admin/.codex/worktrees/0dd9/Personal_Plan/LIMO_Map_Nav_FromSimToRel
test ! -e outputs/setup/writer-pipe-tests-03.log
test ! -e outputs/setup/writer-pipe-build.log
# These tests use stdlib and explicit repository roots, with no ROS imports.
# Do not make their standalone harness search all ROS overlay directories.
env -u PYTHONPATH -u PYTHONHOME -u PYTHONUSERBASE /usr/bin/python3 -S -B outputs/setup/diagnose-writer-pipe-tests.py --all 2>&1 | tee outputs/setup/writer-pipe-tests-03.log
source /opt/ros/jazzy/setup.bash
source /mnt/f/study/Personal_Plan/LIMO_Map_Nav_FromSimToRel/install/local_setup.bash
source /mnt/f/study/Personal_Plan/LIMO_Map_Nav_FromSimToRel/outputs/slam-boundary-ws/install/local_setup.bash
colcon build --packages-select limo_maze_perception --allow-overriding limo_maze_perception --executor sequential 2>&1 | tee outputs/setup/writer-pipe-build.log
cmp src/limo_maze_perception/limo_maze_perception/retention_adapter.py install/limo_maze_perception/lib/python3.12/site-packages/limo_maze_perception/retention_adapter.py
