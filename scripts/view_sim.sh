#!/usr/bin/env bash
# Start a visualization client only; never launch a simulator or mission.
set -eo pipefail

viewer_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
viewer_runtime="$viewer_root"
viewer_mode=rviz
viewer_frame=map
viewer_dry_run=false

usage() {
  cat <<'EOF'
Usage: bash scripts/view_sim.sh [rviz|gazebo] [options]

Connect a viewer to an already running LIMO simulation. Default: rviz.
  --runtime PATH   Existing built workspace (default: this checkout).
                   Use a WSL path, e.g. /mnt/f/project/LIMO_Map_Nav_FromSimToRel.
  --frame FRAME    RViz fixed frame (default: map; use odom before SLAM starts).
  --dry-run        Print the client command without opening a window.
  -h, --help      Show this help.

The runtime's scripts/env.sh supplies ROS/Gazebo discovery settings and overlay.
The RViz preset comes from this checkout. No server, navigation task or motion
command is started. GUI controls remain interactive; avoid pause/reset/goal tools
while observing an autonomous experiment.
EOF
}

while (($#)); do
  case "$1" in
    rviz|gazebo) viewer_mode="$1"; shift ;;
    --runtime|--frame)
      if (($# < 2)) || [[ -z "$2" || "$2" == --* ]]; then
        printf 'Missing value for %s\n' "$1" >&2
        exit 2
      fi
      if [[ "$1" == --runtime ]]; then viewer_runtime="$2"; else viewer_frame="$2"; fi
      shift 2 ;;
    --dry-run) viewer_dry_run=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ ! -f "$viewer_runtime/scripts/env.sh" || ! -f "$viewer_runtime/install/setup.bash" ]]; then
  printf 'Runtime must contain scripts/env.sh and install/setup.bash: %s\n' "$viewer_runtime" >&2
  printf 'Pass --runtime /path/to/the/built/workspace, or build this checkout first.\n' >&2
  exit 1
fi
viewer_runtime=$(cd -- "$viewer_runtime" && pwd)
# ROS setup scripts do not support nounset reliably; use only errexit/pipefail.
source "$viewer_runtime/scripts/env.sh"

# Both the standalone Gazebo client and RViz need the installed model meshes.
if ! viewer_model_prefix=$(ros2 pkg prefix limo_harmonic_description); then
  printf 'The selected runtime has no limo_harmonic_description model package.\n' >&2
  printf 'Use --runtime with the workspace used to start the simulation.\n' >&2
  exit 1
fi
viewer_model_share="$viewer_model_prefix/share/limo_harmonic_description"
export GZ_SIM_RESOURCE_PATH="$viewer_model_share:$viewer_model_prefix/share${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

if [[ "$viewer_mode" == gazebo ]]; then
  viewer_command=(gz sim -g --force-version 8)
else
  viewer_command=(rviz2 -d "$viewer_root/config/simulation.rviz" -f "$viewer_frame"
                  --ros-args -p use_sim_time:=true)
fi
if ! command -v "${viewer_command[0]}" >/dev/null; then
  printf 'Viewer executable is unavailable: %s\n' "${viewer_command[0]}" >&2
  exit 1
fi

printf 'Viewer: %s | ROS_DOMAIN_ID=%s | GZ_PARTITION=%s\n' \
  "$viewer_mode" "$ROS_DOMAIN_ID" "$GZ_PARTITION"
printf 'Runtime: %s\n' "$viewer_runtime"
if [[ "$viewer_dry_run" == true ]]; then
  printf 'Command:'; printf ' %q' "${viewer_command[@]}"; printf '\n'
  exit 0
fi
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  printf 'No DISPLAY or WAYLAND_DISPLAY. Open this from a WSLg-enabled terminal.\n' >&2
  exit 1
fi
exec "${viewer_command[@]}"
