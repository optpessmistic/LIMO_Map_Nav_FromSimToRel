#!/usr/bin/env bash
set -eo pipefail
cd /mnt/c/Users/Admin/.codex/worktrees/0dd9/Personal_Plan/LIMO_Map_Nav_FromSimToRel
test ! -e outputs/setup/run28-timing-tests.log
env -u PYTHONPATH -u PYTHONHOME -u PYTHONUSERBASE /usr/bin/python3 -S -B -m unittest discover -s tools -p test_capture_motion_evidence.py -v 2>&1 | tee outputs/setup/run28-timing-tests.log
