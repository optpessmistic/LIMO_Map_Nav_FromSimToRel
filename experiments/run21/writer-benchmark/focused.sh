#!/usr/bin/env bash
set -eo pipefail
cd /mnt/c/Users/Admin/.codex/worktrees/0dd9/Personal_Plan/LIMO_Map_Nav_FromSimToRel
source /opt/ros/jazzy/setup.bash
exec /usr/bin/python3 -B outputs/setup/run21-writer-benchmark/run-focused.py
