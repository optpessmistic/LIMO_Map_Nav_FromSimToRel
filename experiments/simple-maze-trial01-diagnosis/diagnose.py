"""Read-only summary; never upgrades the failed mission or audits frame bytes."""
import collections
import hashlib
import json
from pathlib import Path
import sys


def diagnose(trial, probe):
    path=trial/'mission/mission_20261111_011942_e4ad8209.json'
    cp_path=trial/'retained/checkpoint.json'
    mission=json.loads(path.read_bytes())
    checkpoint=json.loads(cp_path.read_bytes())
    queue=mission['observation_actions']['queue']
    events=mission['events']
    return {
        'mission_file':str(path),
        'input_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (path,cp_path,probe)},
        'status':mission['status'],'reason':mission['reason'],'task_success':mission['task_success'],
        'map_complete_final':mission['map_complete'],
        'entered_post_mapping_observation':any(e['event']=='observation_queue_frozen' for e in events),
        'navigation_results':dict(collections.Counter(str(e['success']) for e in events if e['event']=='goal_terminal')),
        'observation':{k:queue[k] for k in ('planned_views','dispatches','state_counts')},
        'inspection_resets':dict(collections.Counter(e['reason'] for e in events if e['event']=='inspection_window_reset')),
        'last_inspection':mission['last_inspection_failure'],
        'producer':{'closed':checkpoint['closed'],'fault':checkpoint['fault'],
                    'states':dict(collections.Counter(r['status'] for r in checkpoint['core']['results']))},
        'retention_windows':dict(collections.Counter(w['state'] for w in mission['frame_retention']['windows'])),
        'artifact_status':mission['artifacts']['status'],
        'artifact_map_stamp_ns':mission['artifacts']['map_stamp_ns'],
        'confirmed_targets':mission['targets'],
        'post_failure_live_probe':json.loads(probe.read_bytes())['report'],
        'limits':['The live probe was after failure; it cannot identify the original 5-second map gap.',
                  'Producer stored states are not a completed offline file audit.',
                  'The final map check and all observation actions did not complete.'],
    }


if __name__=='__main__':
    print(json.dumps(diagnose(Path(sys.argv[1]),Path(sys.argv[2])),indent=2))
