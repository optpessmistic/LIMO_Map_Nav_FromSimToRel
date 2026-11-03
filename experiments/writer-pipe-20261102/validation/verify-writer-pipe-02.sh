#!/usr/bin/env bash
set -eo pipefail
cd /mnt/c/Users/Admin/.codex/worktrees/0dd9/Personal_Plan/LIMO_Map_Nav_FromSimToRel
test ! -e outputs/setup/writer-pipe-tests-02.log
test ! -e outputs/setup/writer-pipe-build.log
source /opt/ros/jazzy/setup.bash
source /mnt/f/study/Personal_Plan/LIMO_Map_Nav_FromSimToRel/install/local_setup.bash
source /mnt/f/study/Personal_Plan/LIMO_Map_Nav_FromSimToRel/outputs/slam-boundary-ws/install/local_setup.bash
export PYTHONPATH="$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
/usr/bin/python3 -B outputs/setup/diagnose-writer-pipe-tests.py --all 2>&1 | tee outputs/setup/writer-pipe-tests-02.log
colcon build --packages-select limo_maze_perception --allow-overriding limo_maze_perception --executor sequential 2>&1 | tee outputs/setup/writer-pipe-build.log
cmp src/limo_maze_perception/limo_maze_perception/retention_adapter.py install/limo_maze_perception/lib/python3.12/site-packages/limo_maze_perception/retention_adapter.py
