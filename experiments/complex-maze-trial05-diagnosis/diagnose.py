"""Read the failed run without changing or retrospectively crediting its evidence."""
import hashlib
import json
from pathlib import Path
import sys


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def diagnose(trial):
    mission_path = trial / 'mission/mission_20261111_004817_d61e4383.json'
    checkpoint_path = trial / 'retained/checkpoint.json'
    mission = json.loads(mission_path.read_bytes())
    checkpoint = json.loads(checkpoint_path.read_bytes())
    verified = []
    for result in checkpoint['core']['results']:
        receipt = result.get('receipt')
        if not receipt:
            continue
        for name, expected in receipt['files'].items():
            path = trial / 'retained' / receipt['directory'] / name
            raw = path.read_bytes()
            verified.append({'path': path.relative_to(trial).as_posix(),
                             'bytes': len(raw), 'sha256': sha(raw),
                             'matches_receipt': len(raw) == expected['bytes'] and sha(raw) == expected['sha256']})
    writers = checkpoint['writer_history']
    return {
        'inputs': {p.relative_to(trial).as_posix(): sha(p.read_bytes()) for p in (mission_path, checkpoint_path)},
        'mission_status': mission['status'], 'mission_reason': mission['reason'],
        'task_success': mission['task_success'], 'map_complete': mission['map_complete'],
        'producer_fault': checkpoint['fault'], 'failed_writer': writers[-1],
        'stored_source_records': sum(r['status'] == 'stored' for r in checkpoint['core']['results']),
        'selected_records': len(checkpoint['core']['results']),
        'verified_file_count': sum(v['matches_receipt'] for v in verified), 'files': verified,
        'consumer_windows': [{'number': w['number'], 'state': w['state'], 'reason': w['reason'],
                              'selected': len(w['accepted']),
                              'stored': sum(s['state'] == 'stored' for s in w['accepted'])}
                             for w in mission['frame_retention']['windows']],
        'scope': 'Read-only file integrity and timing diagnosis; original mission remains failed.',
    }


if __name__ == '__main__':
    print(json.dumps(diagnose(Path(sys.argv[1])), indent=2, ensure_ascii=False))
