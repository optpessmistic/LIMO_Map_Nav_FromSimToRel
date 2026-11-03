#!/usr/bin/env bash
set -eo pipefail
cd /mnt/c/Users/Admin/.codex/worktrees/0dd9/Personal_Plan/LIMO_Map_Nav_FromSimToRel
test ! -e outputs/setup/run25-tests-02.log
test ! -e outputs/setup/run25-build.log
source /opt/ros/jazzy/setup.bash
source /mnt/f/study/Personal_Plan/LIMO_Map_Nav_FromSimToRel/install/local_setup.bash
source /mnt/f/study/Personal_Plan/LIMO_Map_Nav_FromSimToRel/outputs/slam-boundary-ws/install/local_setup.bash
export PYTHONPATH="$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
/usr/bin/python3 -B -m unittest test_base_tf_age test_tf_experiment_config test_frame_retention_node -v 2>&1 | tee outputs/setup/run25-tests-02.log
colcon build --packages-select limo_maze_mission limo_maze_bringup --allow-overriding limo_maze_mission limo_maze_bringup --executor sequential 2>&1 | tee outputs/setup/run25-build.log
cmp src/limo_maze_mission/limo_maze_mission/node.py install/limo_maze_mission/lib/python3.12/site-packages/limo_maze_mission/node.py
cmp src/limo_maze_mission/launch/mission.launch.py install/limo_maze_mission/share/limo_maze_mission/launch/mission.launch.py
source install/local_setup.bash
ros2 launch limo_maze_mission mission.launch.py --show-args > outputs/setup/run25-launch-args.log
grep -A 1 tf_max_age_s outputs/setup/run25-launch-args.log
