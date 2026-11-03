"""Archive the closed run22 fixed samples, verify bytes, and audit extraction.

One-off reproduction helper. Refuses existing outputs; does not start ROS or
change the original mission result. Tar timestamps/owners are normalized;
original file contents, names, request identities and hashes are preserved.
"""
import gzip
import hashlib
import io
import json
from pathlib import Path
import sys
import tarfile
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
SOURCE = ROOT / 'outputs/ros_smoke/run22-isolated-writer/retained'
PAIRS = ('camera_info.json', 'depth.bin', 'manifest.json', 'producer.json', 'request.json', 'rgb.bin')


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


def main():
    outputs = [HERE / name for name in ('retained-frames.tar.gz', 'retained-files.json', 'archive-reopened-audit.json')]
    if any(path.exists() for path in outputs):
        raise RuntimeError('Refuse replacing archive evidence')
    mission_path = next((HERE / 'mission').glob('mission_*.json'))
    mission = json.loads(mission_path.read_text())
    entries = [entry for window in mission['frame_retention']['windows'] for entry in window['accepted']]
    checkpoint = json.loads((SOURCE / 'checkpoint.json').read_text())
    results = {row['retention_id']: row for row in checkpoint['core']['results']}
    selected = [('checkpoint.json', SOURCE / 'checkpoint.json')]
    for entry in entries:
        request = entry['request']
        result = results[request['retention_id']]
        folder = entry['request_sha256']
        if len(folder) != 64 or any(c not in '0123456789abcdef' for c in folder):
            raise RuntimeError('Invalid fixed request directory')
        if result['receipt']['directory'] != folder:
            raise RuntimeError('Receipt/request identity mismatch')
        for name in PAIRS:
            path = SOURCE / folder / name
            if path.is_symlink() or not path.is_file():
                raise RuntimeError('Unsafe or missing source file')
            raw = path.read_bytes()
            expected = result['receipt']['files'][name]
            if len(raw) != expected['bytes'] or digest(raw) != expected['sha256']:
                raise RuntimeError('Original receipt byte mismatch')
            selected.append((folder + '/' + name, path))
    members = []
    with outputs[0].open('xb') as stream, gzip.GzipFile(filename='', mode='wb', fileobj=stream, mtime=0) as packed:
        with tarfile.open(fileobj=packed, mode='w') as archive:
            for name, path in selected:
                raw = path.read_bytes()
                info = tarfile.TarInfo('retained/' + name)
                info.size, info.mode, info.mtime = len(raw), 0o644, 0
                archive.addfile(info, io.BytesIO(raw))
                members.append({'member': info.name, 'source': str(path.relative_to(ROOT)),
                                'bytes': len(raw), 'sha256': digest(raw)})
    by_name = {row['member']: row for row in members}
    if len(by_name) != len(members):
        raise RuntimeError('Duplicate archive member')
    with tarfile.open(outputs[0], 'r:gz') as archive:
        if set(archive.getnames()) != set(by_name):
            raise RuntimeError('Archive member set mismatch')
        for member in archive:
            if not member.isfile():
                raise RuntimeError('Unexpected non-file member')
            raw = archive.extractfile(member).read()
            row = by_name[member.name]
            if len(raw) != row['bytes'] or digest(raw) != row['sha256']:
                raise RuntimeError('Packed bytes mismatch')
        with tempfile.TemporaryDirectory(prefix='limo-run22-audit-') as directory:
            archive.extractall(directory, filter='data')
            sys.dont_write_bytecode = True
            sys.path.insert(0, str(ROOT / 'tools'))
            from audit_frame_retention import audit
            result = audit(mission, Path(directory) / 'retained')
    outputs[2].write_text(json.dumps(result, indent=2) + '\n')
    report = {'schema_version': 1, 'archive': outputs[0].name,
        'archive_bytes': outputs[0].stat().st_size, 'archive_sha256': digest(outputs[0].read_bytes()),
        'accepted_pairs': len(entries), 'member_count': len(members), 'members': members,
        'all_uncompressed_bytes_verified': True, 'extracted_audit_passed': result['passed'],
        'audit_tool_sha256': digest((ROOT / 'tools/audit_frame_retention.py').read_bytes()),
        'scope': 'Fixed accepted original files only; original mission/map-export failure unchanged.'}
    outputs[1].write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({key: value for key, value in report.items() if key != 'members'}, indent=2))
    if not result['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
