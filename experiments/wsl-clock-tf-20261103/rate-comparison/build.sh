#!/usr/bin/env bash
set -eo pipefail
cd /mnt/c/Users/Admin/.codex/worktrees/0dd9/Personal_Plan/LIMO_Map_Nav_FromSimToRel
probe=outputs/setup/rclcpp-rate-comparison-20261103
test ! -e "$probe/build.log"
source /opt/ros/jazzy/setup.bash
{
  cmake -S "$probe" -B "$probe/binary" -DCMAKE_BUILD_TYPE=Release
  cmake --build "$probe/binary" --parallel 1
  sha256sum "$probe/rate_probe.cpp" /opt/ros/jazzy/lib/librclcpp.so "$probe/binary/rate_probe"
} 2>&1 | tee "$probe/build.log"
