#!/usr/bin/env bash
# Source from a fresh WSL shell. Keep this experiment off other robots' DDS domains.
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=73
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=limo_maze_jazzy
project_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ -f "$project_root/install/setup.bash" ]]; then
  source "$project_root/install/setup.bash"
fi
unset project_root
