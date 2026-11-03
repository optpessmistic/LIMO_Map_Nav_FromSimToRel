#!/usr/bin/env bash
set -eo pipefail
cd /mnt/c/Users/Admin/.codex/worktrees/0dd9/Personal_Plan/LIMO_Map_Nav_FromSimToRel
test ! -e outputs/setup/run23-ros-startup.log
source /opt/ros/jazzy/setup.bash
source /mnt/f/study/Personal_Plan/LIMO_Map_Nav_FromSimToRel/install/local_setup.bash
source /mnt/f/study/Personal_Plan/LIMO_Map_Nav_FromSimToRel/outputs/slam-boundary-ws/install/local_setup.bash
source install/local_setup.bash
/usr/bin/python3 -B -m unittest tests.test_ros_mission_core.RosStartupTests -v 2>&1 | tee outputs/setup/run23-ros-startup.log
