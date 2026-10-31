#!/usr/bin/env bash
set -eo pipefail
cd /mnt/c/Users/Admin/.codex/worktrees/0dd9/Personal_Plan/LIMO_Map_Nav_FromSimToRel
source /opt/ros/jazzy/setup.bash
source /mnt/f/study/Personal_Plan/LIMO_Map_Nav_FromSimToRel/install/local_setup.bash
source /mnt/f/study/Personal_Plan/LIMO_Map_Nav_FromSimToRel/outputs/slam-boundary-ws/install/local_setup.bash
source /mnt/c/Users/Admin/.codex/worktrees/0dd9/Personal_Plan/LIMO_Map_Nav_FromSimToRel/install/local_setup.bash
exec /usr/bin/python3 -B outputs/setup/run21-writer-benchmark/benchmark.py
