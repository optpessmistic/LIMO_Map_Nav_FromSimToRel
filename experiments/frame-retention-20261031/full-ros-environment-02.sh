#!/usr/bin/env bash
set -eo pipefail
cd /mnt/c/Users/Admin/.codex/worktrees/0dd9/Personal_Plan/LIMO_Map_Nav_FromSimToRel
source /opt/ros/jazzy/setup.bash
# These fixtures use checkout modules and standard Jazzy messages, not Gazebo
# models or the SLAM underlay. Keep their short process-start tests isolated.
export ROS_DOMAIN_ID=74
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RUN_FRAME_RETENTION_ROS_TEST=1
export RUN_OBSERVATION_ROS_TEST=1
export FRAME_RETENTION_ROS_OUTPUT="$PWD/outputs/setup/$1-artifacts"
/usr/bin/python3 -B outputs/setup/run-frame-retention-check.py "$@"
