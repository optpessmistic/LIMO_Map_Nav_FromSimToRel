#!/usr/bin/env bash
set -eo pipefail
cd /mnt/c/Users/Admin/.codex/worktrees/0dd9/Personal_Plan/LIMO_Map_Nav_FromSimToRel
source /opt/ros/jazzy/setup.bash
test ! -e outputs/setup/observation-association-build.log
colcon build --packages-select limo_maze_mission 2>&1 | tee outputs/setup/observation-association-build.log
for module in node core inspection observation; do
  cmp "src/limo_maze_mission/limo_maze_mission/$module.py" "install/limo_maze_mission/lib/python3.12/site-packages/limo_maze_mission/$module.py"
done
cmp src/limo_maze_mission/launch/mission.launch.py install/limo_maze_mission/share/limo_maze_mission/launch/mission.launch.py
source install/local_setup.bash
/usr/bin/python3 - <<'PY' | tee outputs/setup/observation-association-installed.json
import hashlib, importlib, json
from pathlib import Path
result = {}
for module in ('node', 'core', 'inspection', 'observation'):
    obj = importlib.import_module('limo_maze_mission.'+module)
    path = Path(obj.__file__).resolve()
    assert '/install/limo_maze_mission/' in str(path), path
    result[module] = {'path': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
print(json.dumps(result, indent=2))
PY
