#!/usr/bin/env bash
set -eo pipefail
cd /mnt/c/Users/Admin/.codex/worktrees/0dd9/Personal_Plan/LIMO_Map_Nav_FromSimToRel
probe=outputs/setup/rclcpp-rate-comparison-20261103
test ! -e "$probe/run.log"
test ! -e "$probe/results"
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=75 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_LOG_DIR="$PWD/$probe/ros_logs"
# TERM at 84s; escalation 1s later bounds the external allowance to 85s.
/usr/bin/timeout --signal=TERM --kill-after=1s 84s "$probe/binary/rate_probe" "$probe/results" 2>&1 | tee "$probe/run.log"
