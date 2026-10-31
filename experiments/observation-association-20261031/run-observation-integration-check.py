import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

root = Path(__file__).resolve().parents[2]
label, *arguments = sys.argv[1:]
out = root / 'outputs/setup' / label
if out.with_suffix('.json').exists() or out.with_suffix('.log').exists():
    raise SystemExit('Refusing to replace a previous check')
sources = [p for folder in ('src/limo_maze_mission', 'tests')
           for p in (root / folder).rglob('*.py') if '__pycache__' not in p.parts]
def hashes():
    return {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(sources)}
before = hashes()
start = time.monotonic()
command = [sys.executable, '-B', '-m', 'unittest', *arguments]
with out.with_suffix('.log').open('xb') as log:
    result = subprocess.run(command, cwd=root, stdout=log, stderr=subprocess.STDOUT)
report = dict(command=command, exit_code=result.returncode,
              elapsed_s=time.monotonic()-start, source_before=before,
              source_after=hashes(), environment={key: os.environ.get(key) for key in
              ('ROS_DISTRO', 'ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE',
               'RUN_OBSERVATION_ROS_TEST', 'RUN_INITIAL_SURVEY_ROS_TEST')})
out.with_suffix('.json').write_text(json.dumps(report, indent=2)+'\n')
print(out.with_suffix('.log').read_text()[-18000:], flush=True)
print(json.dumps({key: value for key, value in report.items()
                  if key not in ('source_before', 'source_after')}, indent=2), flush=True)
raise SystemExit(result.returncode if before == report['source_after'] else 99)
