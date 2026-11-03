"""Read-only system/steady clock sampler, independent of ROS or simulator."""
import json
import time
from pathlib import Path

out = Path(__file__).resolve().with_name('run30-wsl-system-steady-clock-probe.json')
if out.exists():
    raise RuntimeError('Refuse to overwrite previous clock probe')
samples = []
began = time.monotonic_ns()
while time.monotonic_ns() - began < 65_000_000_000:
    before = time.monotonic_ns()
    real = time.time_ns()
    after = time.monotonic_ns()
    samples.append({'monotonic_before_ns': before, 'realtime_ns': real,
                    'monotonic_after_ns': after, 'offset_ns': real - (before+after)//2})
    time.sleep(.1)
jumps = []
for i, (a,b) in enumerate(zip(samples, samples[1:]),1):
    shift = b['offset_ns']-a['offset_ns']
    if abs(shift) > 50_000_000:
        jumps.append({'sample':i, 'offset_change_s':shift/1e9,
                      'realtime_step_s':(b['realtime_ns']-a['realtime_ns'])/1e9,
                      'monotonic_step_s':(b['monotonic_before_ns']-a['monotonic_before_ns'])/1e9,
                      'bracket_s':(b['monotonic_after_ns']-b['monotonic_before_ns'])/1e9})
report = {'scope':'Read-only WSL clock sampling during run30 epoch 43dc0537-ffd4-47df-a3a5-d3b7a504bf46, no ROS; offset jumps above 50ms are reported, not automatically assigned a cause.',
          'duration_monotonic_s':(time.monotonic_ns()-began)/1e9,
          'samples':samples, 'jumps':jumps,
          'max_sampling_bracket_s':max(s['monotonic_after_ns']-s['monotonic_before_ns'] for s in samples)/1e9}
with out.open('x') as handle:
    json.dump(report,handle,indent=2)
print(json.dumps({k:v for k,v in report.items() if k!='samples'},indent=2))
