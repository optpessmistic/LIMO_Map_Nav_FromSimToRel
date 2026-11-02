"""Reopen exact accepted-frame files and the final producer checkpoint.

This read-only audit verifies stored bytes and supplied TF identities. It does
not certify scene coverage, target recall, TF accuracy, or physical stationarity.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/limo_evidence_protocol'))
from limo_evidence_protocol.retention import AcceptedWindow, canonical, sample_identity, sha, validate_request

FILES = {'rgb.bin', 'depth.bin', 'producer.json', 'camera_info.json', 'request.json', 'manifest.json'}


def read_json(path, limit=65536):
    if path.is_symlink() or not path.is_file() or path.stat().st_size > limit:
        raise ValueError('missing_unsafe_or_oversize_json:' + path.name)
    with path.open('rb') as stream:
        raw = stream.read(limit + 1)
    if len(raw) > limit:
        raise ValueError('json_budget:' + path.name)
    return json.loads(raw)


def file_digest(path, expected):
    if (type(expected) is not dict or type(expected.get('bytes')) is not int or
            not 0 <= expected['bytes'] <= 2**31 or path.is_symlink() or
            not path.is_file() or path.stat().st_size != expected['bytes']):
        raise ValueError('file_size_or_type:' + path.name)
    digest, count = hashlib.sha256(), 0
    with path.open('rb') as stream:
        while chunk := stream.read(1024 * 1024):
            count += len(chunk)
            if count > expected['bytes']:
                raise ValueError('file_grew:' + path.name)
            digest.update(chunk)
    if count != expected['bytes'] or digest.hexdigest() != expected.get('sha256'):
        raise ValueError('file_hash:' + path.name)


def audit(consumer, root):
    """consumer is Consumer.snapshot(), or a mission result containing it."""
    root = Path(root).absolute()
    errors, frames = [], []
    checkpoint_valid = False
    try:
        if type(consumer) is not dict:
            raise ValueError('consumer_object_required')
        consumer = consumer.get('frame_retention', consumer)
        if type(consumer) is not dict:
            raise ValueError('consumer_object_required')
        if root.is_symlink() or not root.is_dir() or root.resolve() != root:
            raise ValueError('canonical_retention_root_required')
        checkpoint = read_json(root / 'checkpoint.json', 4 * 1024 * 1024)
        core = checkpoint['core']
        if (checkpoint.get('closed') is not True or checkpoint.get('fault') is not None or
                checkpoint.get('checkpoint_failed') is not False or checkpoint.get('writer') is not None or
                core.get('closed') is not True or core.get('writer_pending') is not False or
                core.get('admission', {}).get('closed') is not False):
            raise ValueError('producer_checkpoint_not_cleanly_closed')
        results = core['results']
        by_id = {r['retention_id']: r for r in results}
        if len(by_id) != len(results):
            raise ValueError('duplicate_checkpoint_result')
        if consumer.get('enabled') is not True or consumer.get('fault') is not None:
            raise ValueError('consumer_disabled_or_faulted')
        windows = consumer['windows']
        if not 1 <= len(windows) <= 256:
            raise ValueError('window_count')
        all_ids = set()
        for window in windows:
            accepted = window['accepted']
            if len(accepted) != window['ledger_accepted_count']:
                raise ValueError('accepted_count_mismatch')
            if not accepted:
                continue
            AcceptedWindow.freeze([entry['request'] for entry in accepted])
            if (type(window['binding']) is not dict or set(window['binding']) != {
                    'map_epoch', 'goal_token', 'inspection_request_id', 'after_stamp_ns'} or
                    window['ids'] != [entry['request']['retention_id'] for entry in accepted] or
                    any({key: entry['request'][key] for key in window['binding']} != window['binding']
                        for entry in accepted)):
                raise ValueError('accepted_window_binding_mismatch')
            for entry in accepted:
                req, request_raw = validate_request(entry['request'])
                rid, fingerprint = req['retention_id'], sha(request_raw)
                if rid in all_ids or len(all_ids) >= 4096:
                    raise ValueError('duplicate_or_excess_accepted_id')
                all_ids.add(rid)
                row = {'retention_id': rid, 'request_sha256': fingerprint, 'verified': False}
                frames.append(row)
                try:
                    if req['sample']['map_epoch'] != core['map_epoch'] or req['sample']['detector_session_id'] != core['detector_session_id']:
                        raise ValueError('producer_epoch_or_session_mismatch')
                    if entry['request_sha256'] != fingerprint:
                        raise ValueError('consumer_request_digest_mismatch')
                    result = by_id[rid]
                    if (result.get('status') != 'stored' or result.get('reason') is not None or
                            result.get('writer_owned') is not False or result.get('request_sha256') != fingerprint):
                        raise ValueError('final_result_not_stored')
                    receipt = result['receipt']
                    if (receipt.get('directory') != fingerprint or receipt.get('request_sha256') != fingerprint or
                            receipt.get('file_fsync_completed') is not True or set(receipt['files']) != FILES):
                        raise ValueError('receipt_binding')
                    # Compare the receipt seen by the mission with the final source state.
                    seen = entry.get('result')
                    if (entry.get('state') != 'stored' or not seen or seen.get('status') != 'stored' or
                            seen.get('retention_id') != rid or seen.get('request_sha256') != fingerprint or
                            seen.get('writer_owned') is not False or seen.get('reason') is not None or
                            seen.get('checkpoint_persisted') is not True or seen.get('receipt') != receipt):
                        raise ValueError('consumer_receipt_not_confirmed')
                    directory = root / fingerprint
                    if directory.is_symlink() or directory.resolve() != directory:
                        raise ValueError('unsafe_frame_directory')
                    for name in sorted(FILES):
                        file_digest(directory / name, receipt['files'][name])
                    saved_req = read_json(directory / 'request.json', 16384)
                    if canonical(saved_req) != request_raw:
                        raise ValueError('saved_request_or_exact_tf_mismatch')
                    producer = read_json(directory / 'producer.json')
                    camera = read_json(directory / 'camera_info.json')
                    manifest = read_json(directory / 'manifest.json')
                    if (sample_identity(producer) != req['sample'] or camera != producer['camera_info'] or
                            manifest.get('request_sha256') != fingerprint or manifest.get('sample') != req['sample'] or
                            manifest.get('files') != {k: v for k, v in receipt['files'].items() if k != 'manifest.json'}):
                        raise ValueError('producer_or_manifest_identity')
                    if (sha(canonical({'frame_id': camera['frame_id'], **camera['calibration']})) != req['sample']['calibration_id'] or
                            sha(canonical(producer['detector_config'])) != req['sample']['detector_config_id']):
                        raise ValueError('calibration_or_config_digest')
                    for name in ('rgb', 'depth'):
                        meta, saved = producer[name], receipt['files'][name + '.bin']
                        if (meta['data_sha256'] != saved['sha256'] or meta['data_bytes'] != saved['bytes'] or
                                meta['height'] * meta['step'] != saved['bytes']):
                            raise ValueError('raw_layout_or_hash:' + name)
                    row['verified'] = True
                except (ValueError, TypeError, KeyError, AttributeError, OSError) as exc:
                    row['error'] = str(exc)[:256]
        if all_ids != set(by_id):
            errors.append('accepted_and_producer_request_sets_differ')
        if not all_ids:
            errors.append('no_accepted_frames')
        checkpoint_valid = True
    except (ValueError, TypeError, KeyError, AttributeError, OSError) as exc:
        errors.append(str(exc)[:256])
    verified = sum(row['verified'] for row in frames)
    return {'schema_version': 1, 'passed': bool(checkpoint_valid and frames and verified == len(frames) and not errors),
            'accepted_count': len(frames), 'verified_count': verified, 'errors': errors, 'frames': frames,
            'scope': 'exact_accepted_files_reopened_and_hashed; supplied_TF_identity_only; '
                     'does_not_verify_process_exit_scene_coverage_target_recall_or_motion'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--consumer', type=Path, required=True, help='Saved consumer snapshot or mission result')
    parser.add_argument('--retention-root', type=Path, required=True)
    args = parser.parse_args()
    result = audit(read_json(args.consumer, 256 * 1024 * 1024), args.retention_root)
    print(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
