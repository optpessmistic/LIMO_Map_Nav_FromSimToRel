"""One source-focused writer/lifecycle regression run; no ROS nodes."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

root = Path(__file__).resolve().parents[3]
output = Path(__file__).resolve().parent
log = output/'production-focused-01.log'
report = output/'production-focused-01.json'
if log.exists() or report.exists():
    raise SystemExit('Refuse overwrite or duplicate focused run')
paths = [root/'src/limo_maze_perception/limo_maze_perception/retention_adapter.py',
         root/'tests/test_frame_retention_adapter.py',
         root/'tests/test_frame_retention_owner_races.py',
         root/'tests/test_perception_lifecycle.py']
def hashes():
    return {str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
before = hashes()
command = [sys.executable, '-B', '-m', 'unittest', '-v', 'test_frame_retention_adapter',
           'test_frame_retention_owner_races', 'test_perception_lifecycle']
env = dict(os.environ, PYTHONPATH=str(root/'tests')+os.pathsep+os.environ.get('PYTHONPATH',''))
started = time.monotonic()
with log.open('xb') as stream:
    result = subprocess.run(command, cwd=root, env=env, stdout=stream, stderr=subprocess.STDOUT)
after = hashes()
value = dict(command=command, exit_code=result.returncode, elapsed_wall_s=time.monotonic()-started,
             sources_before=before,sources_after=after,sources_unchanged=before==after,
             scope='source focused owned writer, race, and extracted lifecycle regression; no ROS nodes')
with report.open('x') as stream:
    json.dump(value,stream,indent=2)
    stream.write('\n')
print(json.dumps(value),flush=True)
raise SystemExit(result.returncode if before==after else 99)
