"""Owned stdlib-only writer. No ROS imports, Core calls, threads, or descendants."""
import argparse
import base64
import json
import os
from pathlib import Path
import sys

from limo_evidence_protocol.retention import Job, canonical, digest, write_job

MAX_CHECKPOINT = 4 * 1024 * 1024
MAX_REPLY = 32768


def save_checkpoint(root, raw):
    if len(raw) > MAX_CHECKPOINT:
        raise ValueError('checkpoint_budget')
    json.loads(raw)
    pending = root / '.checkpoint.pending'
    flags = os.O_WRONLY | os.O_CREAT | os.O_TRUNC | os.O_NOFOLLOW
    fd = os.open(pending, flags, 0o600)
    with os.fdopen(fd, 'wb') as stream:
        stream.write(raw)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(pending, root / 'checkpoint.json')
    # This is file fsync + atomic rename, not a device power-loss guarantee.


def read_exact(stream, size):
    if type(size) is not int or size < 0:
        raise ValueError('invalid_length')
    data = stream.read(size)
    if len(data) != size:
        raise ValueError('incomplete_owner_transfer')
    return data


def execute(stream, root, max_wire):
    size = int.from_bytes(read_exact(stream, 8), 'big')
    if size > 16384:
        raise ValueError('header_budget')
    header = json.loads(read_exact(stream, size))
    if header['token'] != os.environ.get('LIMO_RETENTION_WRITER_TOKEN'):
        raise ValueError('owner_token_mismatch')
    checkpoint_size = header['checkpoint_bytes']
    if type(checkpoint_size) is not int or not 0 < checkpoint_size <= MAX_CHECKPOINT:
        raise ValueError('checkpoint_budget')
    job_meta = header['job']
    lengths = ([entry[1] for entry in job_meta['files']] +
               [job_meta['manifest_bytes'], job_meta['receipt_bytes']]) if job_meta else []
    if (any(type(n) is not int or n < 0 for n in lengths) or
            8 + size + checkpoint_size + sum(lengths) > max_wire):
        raise ValueError('wire_budget')
    checkpoint = read_exact(stream, checkpoint_size)
    job = None
    if job_meta:
        names = [entry[0] for entry in job_meta['files']]
        if names != ['rgb.bin', 'depth.bin', 'producer.json', 'camera_info.json', 'request.json']:
            raise ValueError('job_file_names')
        directory = digest(job_meta['directory'])
        files = tuple((name, read_exact(stream, size)) for name, size in job_meta['files'])
        manifest = read_exact(stream, job_meta['manifest_bytes'])
        receipt = read_exact(stream, job_meta['receipt_bytes'])
        job = Job(job_meta['request_id'], digest(job_meta['request_sha256']), directory,
                  files, manifest, receipt, job_meta['file_bytes'])
    if stream.read(1):
        raise ValueError('trailing_owner_bytes')
    save_checkpoint(root, checkpoint)
    result = write_job(job, root) if job else None
    return {'ok': True, 'operation_id': header['operation_id'],
            'receipt_b64': base64.b64encode(result).decode() if result is not None else None}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', required=True)
    parser.add_argument('--max-wire-bytes', type=int, required=True)
    args = parser.parse_args()
    root = Path(args.root)
    if not root.is_absolute() or not root.is_dir() or root.is_symlink() or root.resolve() != root:
        raise ValueError('exclusive_absolute_root_required')
    try:
        reply = execute(sys.stdin.buffer, root, args.max_wire_bytes)
    except Exception as exc:
        reply = {'ok': False, 'error': type(exc).__name__ + ':' + str(exc)[:1024]}
    raw = canonical(reply, MAX_REPLY - 1) + b'\n'
    sys.stdout.buffer.write(raw)
    sys.stdout.buffer.flush()
    return 0 if reply['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
