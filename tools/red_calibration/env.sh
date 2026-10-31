#!/usr/bin/env bash
# Source F model dependencies, then current C bringup. This wrapper starts no
# processes except runner.py; runner requires its separate --execute to run ROS.
set -eo pipefail
cal_repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
if (($# < 1)); then
  printf 'Usage: bash env.sh /path/to/built/runtime [runner options]\n' >&2
  exit 2
fi
cal_runtime=$(cd -- "$1" && pwd)
shift
cd "$cal_repo"
source /opt/ros/jazzy/setup.bash
source "$cal_runtime/install/local_setup.bash"
source "$cal_repo/install/local_setup.bash"
cmp src/limo_sim_bringup/launch/sim.launch.py install/limo_sim_bringup/share/limo_sim_bringup/launch/sim.launch.py
cmp src/limo_sim_bringup/scripts/spawn_once.py install/limo_sim_bringup/lib/limo_sim_bringup/spawn_once.py
cmp src/limo_sim_bringup/scripts/inspect_scene_once.py install/limo_sim_bringup/lib/limo_sim_bringup/inspect_scene_once.py
exec /usr/bin/python3 "$cal_repo/tools/red_calibration/runner.py" "$@"
