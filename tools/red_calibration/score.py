#!/usr/bin/env python3
"""Offline fixed-six observation audit. Never imports ROS or detector code.

Truth is read only here, after the runner has recorded closed owned groups.
No target matching, world-position error, accuracy, recall or range estimate.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
import os
from pathlib import Path
import re
import struct
import sys
import zlib

N = 6
MAX_JSON = 2 * 1024**2
MAX_FILE = 16 * 1024**2
MAX_TOTAL = 256 * 1024**2
MAX_PIXELS = 1_048_576
SHA = re.compile(r'[0-9a-f]{64}\Z')
LABEL = re.compile(r'[A-Za-z0-9_-]{1,80}\Z')


def digest(data):
    return hashlib.sha256(data).hexdigest()


def child(root, relative):
    if not isinstance(relative, str) or not relative or '\\' in relative:
        raise ValueError('Invalid relative evidence path')
    part = Path(relative)
    if part.is_absolute() or '..' in part.parts:
        raise ValueError('Evidence path escapes root')
    path = (root / part).resolve()
    if not path.is_relative_to(root.resolve()):
        raise ValueError('Evidence symlink escapes root')
    return path


class Reader:
    def __init__(self):
        self.files, self.total = {}, 0

    def read(self, path, limit=MAX_FILE):
        path = Path(path).resolve()
        size = path.stat().st_size
        if not path.is_file() or not 0 <= size <= limit:
            raise ValueError('Evidence file size limit: ' + str(path))
        data = path.read_bytes()
        if len(data) != size or len(data) > limit:
            raise ValueError('Evidence changed while reading: ' + str(path))
        if str(path) not in self.files:
            self.total += len(data)
        if self.total > MAX_TOTAL:
            raise ValueError('Aggregate evidence byte limit')
        identity = {'bytes': len(data), 'sha256': digest(data)}
        if str(path) in self.files and self.files[str(path)] != identity:
            raise ValueError('Evidence changed during audit: ' + str(path))
        self.files[str(path)] = identity
        return data

    def json(self, path):
        def reject(value):
            raise ValueError('Nonfinite JSON token: ' + value)
        def unique_pairs(pairs):
            result = {}
            for key, value in pairs:
                if key in result:
                    raise ValueError('Duplicate JSON key: '+key)
                result[key] = value
            return result
        value = json.loads(self.read(path, MAX_JSON), parse_constant=reject, object_pairs_hook=unique_pairs)
        if not isinstance(value, dict):
            raise ValueError('Expected JSON object: ' + str(path))
        return value

    def verified(self, path, reference):
        data = self.read(path)
        if (not isinstance(reference, dict) or type(reference.get('bytes')) is not int or
                not isinstance(reference.get('sha256'), str) or not SHA.fullmatch(reference['sha256']) or
                len(data) != reference['bytes'] or digest(data) != reference['sha256']):
            raise ValueError('Hash/size mismatch: ' + str(path))
        return data

    def unchanged(self):
        for path, identity in list(self.files.items()):
            self.verified(Path(path), identity)


def mask_metrics(data):
    """Decode the capture writer's bounded, grayscale filter-0 PNG losslessly."""
    if data[:8] != b'\x89PNG\r\n\x1a\n':
        raise ValueError('Not a PNG mask')
    at, chunks, pixels, width, height = 8, [], None, None, None
    seen_end = False
    while at < len(data):
        if at + 12 > len(data):
            raise ValueError('Truncated PNG')
        length = struct.unpack('!I', data[at:at+4])[0]
        end = at + 12 + length
        if end > len(data):
            raise ValueError('Truncated PNG chunk')
        name, payload = data[at+4:at+8], data[at+8:at+8+length]
        if zlib.crc32(name+payload) & 0xffffffff != struct.unpack('!I', data[at+8+length:end])[0]:
            raise ValueError('PNG CRC mismatch')
        if name == b'IHDR':
            if at != 8 or length != 13 or width is not None:
                raise ValueError('Unexpected PNG header')
            width, height, bits, color, compression, filtering, interlace = struct.unpack('!IIBBBBB', payload)
            if not 0 < width*height <= MAX_PIXELS or (bits,color,compression,filtering,interlace) != (8,0,0,0,0):
                raise ValueError('Unsupported or oversized mask PNG')
        elif name == b'IDAT':
            if width is None:
                raise ValueError('PNG data before header')
            chunks.append(payload)
        elif name == b'IEND':
            if length or end != len(data):
                raise ValueError('Unexpected PNG trailing data')
            seen_end = True
        else:
            raise ValueError('Unexpected capture mask PNG chunk')
        at = end
    if not seen_end or width is None or not chunks:
        raise ValueError('Incomplete mask PNG')
    expected = height*(width+1)
    inflater = zlib.decompressobj()
    raw = inflater.decompress(b''.join(chunks), expected+1)
    if len(raw) != expected or not inflater.eof or inflater.unused_data or inflater.unconsumed_tail:
        raise ValueError('Mask PNG decoded length mismatch')
    count, min_x, min_y, max_x, max_y = 0, width, height, -1, -1
    for y in range(height):
        row = raw[y*(width+1):(y+1)*(width+1)]
        if row[0] != 0:
            raise ValueError('Only capture writer filter-0 PNG is supported')
        for x, value in enumerate(row[1:]):
            if value not in (0, 255):
                raise ValueError('Nonbinary red mask')
            if value:
                count += 1
                min_x, min_y, max_x, max_y = min(min_x,x), min(min_y,y), max(max_x,x), max(max_y,y)
    return {'width': width, 'height': height, 'red_pixels': count,
            'red_fraction': count/(width*height),
            'bbox_xyxy_inclusive': [min_x,min_y,max_x,max_y] if count else None,
            'scope': 'Observed algorithm mask; no ground-truth pixel mask or IoU.'}


def tf_check(sample, manifest):
    expected = {'reference_to_base': (manifest.get('reference_frame'), manifest.get('base_frame'), False),
                'base_to_camera': (manifest.get('base_frame'), sample.get('optical_frame'), True),
                'reference_to_camera': (manifest.get('reference_frame'), sample.get('optical_frame'), False)}
    issues = []
    for key, (target, source, static) in expected.items():
        item = sample.get('transforms', {}).get(key, {})
        stamp = sample.get('stamp_ns')
        if (type(stamp) is not int or stamp <= 0 or not target or not source or
                item.get('target_frame') != target or item.get('source_frame') != source or
                item.get('query_stamp_ns') != stamp or
                type(item.get('returned_stamp_ns')) is not int or
                item['returned_stamp_ns'] not in ((stamp, 0) if static else (stamp,))):
            issues.append(key+':frame_or_stamp')
        values = item.get('translation_xyz_m', []) + item.get('rotation_xyzw', [])
        if (len(item.get('translation_xyz_m', [])) != 3 or len(item.get('rotation_xyzw', [])) != 4 or
                not all(type(v) in (int, float) and math.isfinite(v) for v in values)):
            issues.append(key+':values')
        elif abs(sum(v*v for v in item['rotation_xyzw'])-1) > 1e-3:
            issues.append(key+':quaternion')
    return {'recorded_exact_lookup_consistent': not issues, 'audit_issues': issues,
            'reported_reasons': sample.get('tf_reasons'), 'transforms': sample.get('transforms'),
            'scope': 'Saved lookup values only; raw TF brackets and physical world pose not audited.'}


def ensure_closed(result):
    stops, commands = result.get('stops'), result.get('commands')
    if (result.get('status') not in ('captured', 'failed_or_incomplete') or
            result.get('all_owned_groups_empty') is not True or
            not isinstance(stops, dict) or not stops or not isinstance(commands, dict)):
        raise ValueError('Case is not recorded closed with verified owned cleanup')
    # Commands describe planned jobs, including those never started after a failure.
    if not set(stops).issubset(commands):
        raise ValueError('Cleanup refers to unknown job names')
    for name, stop in stops.items():
        if (stop.get('remaining_check_complete') is not True or stop.get('remaining') != [] or
                stop.get('ownership_errors') or stop.get('error')):
            raise ValueError('Unconfirmed owned cleanup: '+name)


def fixture(reader, manifest_path, case_id, result):
    manifest = reader.json(manifest_path)
    entries = [entry for entry in manifest.get('cases', []) if entry.get('id') == case_id]
    if manifest.get('schema_version') != 1 or len(entries) != 1:
        raise ValueError('Fixture manifest case is missing or ambiguous')
    entry, root = entries[0], manifest_path.parent
    world = child(root, entry.get('world'))
    truth_path = child(root, entry.get('offline_truth'))
    if digest(reader.read(world)) != entry.get('world_sha256'):
        raise ValueError('Fixture world hash mismatch')
    truth_data = reader.read(truth_path, MAX_JSON)
    if digest(truth_data) != entry.get('truth_sha256'):
        raise ValueError('Offline truth hash mismatch')
    truth = reader.json(truth_path)
    if (truth.get('schema_version') != 1 or truth.get('role') != 'simulator_and_offline_evaluator_only' or
            truth.get('case', {}).get('id') != case_id or
            truth.get('world_sha256') != entry['world_sha256'] or
            result.get('world_sha256') != entry['world_sha256']):
        raise ValueError('Case/world/truth identity mismatch')
    count = truth.get('target_count_offline_only')
    if type(count) is not int or not 0 <= count <= 32 or len(truth.get('targets_ground_truth', [])) != count:
        raise ValueError('Invalid declared offline target count')
    return {'case': truth['case'], 'declared_target_count': count,
            'world_sha256': entry['world_sha256'], 'truth_sha256': entry['truth_sha256'],
            'label_scope': truth.get('label_scope'), 'truth_path': str(truth_path),
            'rendered_visibility_verified_by_scorer': False,
            'truth_role': 'Offline control description only; never sent to online strategy.'}


def score_collection_checkpoint(reader, root, result, manifest, control):
    """Describe durable raw inputs, never infer results from orphan derived files."""
    checkpoint = reader.json(root/'collection-checkpoint.json')
    if (checkpoint.get('schema_version') != 1 or
            checkpoint.get('capture_session') != manifest['capture_session'] or
            checkpoint.get('report_phase') != 'raw_saved_before_offline_processing' or
            any(checkpoint.get(key) is not False for key in ('passed', 'processing_complete', 'cleanup_complete'))):
        raise ValueError('Invalid early collection checkpoint identity or phase')
    selected, saved = checkpoint.get('selected_sequences'), checkpoint.get('raw_saved_sequences')
    for sequences in (selected, saved):
        if (not isinstance(sequences, list) or len(sequences) > N or
                any(type(value) is not int or not 1 <= value <= N for value in sequences) or
                sequences != sorted(set(sequences))):
            raise ValueError('Invalid early collection sequence list')
    if selected != list(range(1, len(selected)+1)) or not set(saved).issubset(selected):
        raise ValueError('Early collection changed fixed original sequences')
    if checkpoint.get('not_raw_committed_sequences') != [value for value in selected if value not in saved]:
        raise ValueError('Early collection raw sequence accounting mismatch')
    artifacts = checkpoint.get('artifacts')
    if not isinstance(artifacts, dict) or not 1 <= len(artifacts) <= 300:
        raise ValueError('Invalid early checkpoint artifact index')
    sources = manifest.get('source', {}).get('sources', {})
    if not isinstance(sources, dict) or not 1 <= len(sources) <= 16:
        raise ValueError('Missing or oversized captured source provenance')
    # Derivatives may exist after the checkpoint, but are never read in this path.
    allowed = {'manifest.json'} | {'source/'+Path(name).name for name in sources}
    allowed |= {f'sample_{sequence:03d}/'+name for sequence in saved
                for name in ('raw.json', 'rgb.data', 'depth.data')}
    issues, verified = [], {}
    for name in sorted(allowed & artifacts.keys()):
        try:
            verified[name] = reader.verified(child(root, name), artifacts[name])
        except (OSError, ValueError) as exc:
            issues.append('artifact:'+name+':'+str(exc))
    if 'manifest.json' not in verified:
        issues.append('capture_manifest_not_verified_by_checkpoint')
    for path, reference in sources.items():
        name = 'source/'+Path(path).name
        data = verified.get(name)
        if data is None or digest(data) != reference.get('sha256') or len(data) != reference.get('bytes'):
            issues.append('source_copy_unverified:'+name)
    rows = []
    for sequence in range(1, N+1):
        row = {'sequence': sequence, 'status': 'missing_raw', 'detection_count': None,
               'processing_status': None, 'processing': None, 'detections_optical_frame': None,
               'mask': None, 'input_valid_when_saved': None, 'input_valid_at_collection_end': None,
               'count_relation_to_declared_instances': None, 'audit_issues': []}
        rows.append(row)
        artifact = f'sample_{sequence:03d}/raw.json'
        if sequence not in saved or artifact not in verified:
            row['audit_issues'].append('raw_sample_not_committed_or_verified')
            continue
        sample = reader.json(root/artifact)
        if sample.get('sequence') != sequence or sample.get('processing_status') != 'not_run_yet':
            row['audit_issues'].append('raw_sample_binding_or_phase_mismatch')
        meta, files = sample.get('input_metadata', {}), sample.get('files', {})
        if not isinstance(meta, dict) or not isinstance(files, dict):
            raise ValueError('Invalid raw sample metadata or file references')
        for kind in ('rgb', 'depth'):
            filename = kind+'.data'
            full_name = f'sample_{sequence:03d}/'+filename
            data, image = verified.get(full_name), meta.get(kind) or {}
            if (not isinstance(image, dict) or data is None or
                    files.get(filename) != artifacts.get(full_name) or
                    digest(data) != image.get('data_sha256') or len(data) != image.get('data_bytes')):
                row['audit_issues'].append('raw_payload_unverified:'+kind)
        if not row['audit_issues']:
            row.update(status='raw_only', artifact=artifact, input_metadata=meta,
                       recorded_observation=sample.get('observation'))
    issues.extend('sample_%03d:%s' % (row['sequence'], issue) for row in rows for issue in row['audit_issues'])
    reader.unchanged()
    issues.extend(['only_collection_checkpoint_available', 'offline_processing_results_unavailable',
                   'final_summary_missing_cleanup_unknown', 'fixed_six_collection_not_certified'])
    return {'schema_version': 1, 'case_id': result['case_id'], 'fixed_window_size': N,
            'status': 'incomplete_or_ineligible_evidence', 'evidence_integrity_passed': False,
            'fixed_collection_complete': False, 'eligible_for_control_count_comparison': False,
            'capture_reported_passed': checkpoint['passed'], 'capture_cleanup_complete': None,
            'runner_reported_status': result['status'], 'evidence_phase': 'collection_checkpoint_only',
            'recognition_success': None, 'recognition_success_evaluated': False,
            'control': control, 'frames': rows,
            'summary': {'detection_count_by_sequence': [None]*N, 'detection_count_histogram': {},
                        'missing_or_ambiguous_frames': N, 'input_valid_frames': None,
                        'frames_with_nonzero_detections': None, 'mask_red_pixels_by_sequence': [None]*N,
                        'verified_raw_sequences': [row['sequence'] for row in rows if row['status'] == 'raw_only']},
            'capture_errors': checkpoint.get('errors'),
            'selected_sample_conflicts': checkpoint.get('selected_sample_conflicts'),
            'audit_issues': list(dict.fromkeys(issues)), 'sources': reader.files,
            'scorer_sha256': digest(Path(__file__).read_bytes()),
            'limits': ['Only hash-referenced raw inputs are described; uncommitted derived files are ignored.',
                       'Outer owned-group closure does not certify capture cleanup, processing or collection completeness.',
                       'Null processing results are neither zero detections nor misses; fixed slots 1..6 are retained.',
                       'Hash consistency is not dataset authenticity; no native pose or recognition performance is verified.'],
            'world_position_error_computed': False, 'effective_range_claimed': False,
            'accuracy_claimed': False, 'visual_coverage_claimed': False}


def score_case(case_dir, fixture_manifest):
    case_dir, fixture_manifest = Path(case_dir).resolve(), Path(fixture_manifest).resolve()
    reader = Reader()
    result = reader.json(case_dir/'result.json')
    ensure_closed(result)  # Refuse to read a collection while it is still running.
    case_id = result.get('case_id')
    if not isinstance(case_id, str) or not LABEL.fullmatch(case_id):
        raise ValueError('Invalid case label')
    control = fixture(reader, fixture_manifest, case_id, result)
    root = case_dir/'capture'
    issues, rows = [], []
    capture_manifest = reader.json(root/'manifest.json')
    if (capture_manifest.get('case_id') != case_id or capture_manifest.get('requested_pairs') != N or
            not capture_manifest.get('capture_session')):
        raise ValueError('Capture manifest identity or fixed-six policy mismatch')
    session = capture_manifest['capture_session']
    summary_path = root/'summary.json'
    if (not summary_path.exists() and not (root/'raw-checkpoint.json').exists() and
            (root/'collection-checkpoint.json').is_file()):
        return score_collection_checkpoint(reader, root, result, capture_manifest, control)
    if summary_path.is_file():
        summary = reader.json(summary_path)
        reference = summary.get('raw_checkpoint', {})
        if reference.get('artifact') != 'raw-checkpoint.json':
            raise ValueError('Unexpected checkpoint reference')
        reader.verified(root/'raw-checkpoint.json', reference)
        summary_available = True
    else:
        summary = reader.json(root/'raw-checkpoint.json')
        summary_available = False
        issues.append('final_summary_missing_cleanup_unknown')
    raw = reader.json(root/'raw-checkpoint.json')
    for name, document in (('checkpoint', raw), ('summary', summary)):
        if document.get('case_id') != case_id or document.get('capture_session') != session:
            raise ValueError(name+' identity mismatch')
    for key in ('requested_pairs', 'saved_pairs', 'selected_pairs', 'collection_complete',
                'sampling_complete', 'sampling_passed', 'source_unchanged', 'all_samples_valid',
                'all_samples_valid_when_saved', 'selected_sample_conflicts', 'conflicting_duplicate_count'):
        if raw.get(key) != summary.get(key):
            issues.append('summary_changed_frozen_sampling_field:'+key)
    if any(error not in summary.get('errors', []) for error in raw.get('errors', [])):
        issues.append('summary_removed_checkpoint_error')
    artifacts = raw.get('artifacts_saved_before_cleanup', {})
    if not isinstance(artifacts, dict) or not 1 <= len(artifacts) <= 300:
        raise ValueError('Invalid checkpoint artifact index')
    verified = {}
    for name, reference in artifacts.items():
        try:
            verified[name] = reader.verified(child(root, name), reference)
        except (OSError, ValueError) as exc:
            issues.append('artifact:'+name+':'+str(exc))
    if 'manifest.json' not in verified:
        issues.append('capture_manifest_not_verified_by_checkpoint')
    source_index = capture_manifest.get('source', {}).get('sources', {})
    if not isinstance(source_index, dict) or not 1 <= len(source_index) <= 16:
        raise ValueError('Missing or oversized captured source provenance')
    for source_path, reference in source_index.items():
        copy_name = 'source/'+Path(source_path).name
        data = verified.get(copy_name)
        if data is None or digest(data) != reference.get('sha256') or len(data) != reference.get('bytes'):
            issues.append('source_copy_unverified:'+copy_name)
    samples = summary.get('samples', [])
    if not isinstance(samples, list) or len(samples) > 32:
        raise ValueError('Invalid saved sample list')
    if raw.get('samples') != samples:
        issues.append('summary_checkpoint_sample_list_mismatch')
    by_sequence = {}
    for item in samples:
        sequence = item.get('sequence')
        if type(sequence) is not int or not 1 <= sequence <= N:
            issues.append('unexpected_sample_sequence')
            continue
        by_sequence.setdefault(sequence, []).append(item)
    conflicts = summary.get('selected_sample_conflicts', [])
    conflicted = {item.get('sequence') for item in conflicts}
    previous_stamp = None
    for sequence in range(1, N+1):
        row = {'sequence': sequence, 'status': 'missing', 'detection_count': None,
               'input_valid_when_saved': None, 'input_valid_at_collection_end': False,
               'count_relation_to_declared_instances': None, 'audit_issues': []}
        rows.append(row)
        candidates = by_sequence.get(sequence, [])
        if len(candidates) != 1:
            row['audit_issues'].append('missing_sequence' if not candidates else 'duplicate_sequence')
            continue
        item = candidates[0]
        artifact = f'sample_{sequence:03d}/sample.json'
        if item.get('artifact') != artifact or artifact not in verified:
            row['audit_issues'].append('sample_json_not_verified')
            continue
        sample = reader.json(root/artifact)
        row.update(status='saved', artifact=artifact, stamp_ns=sample.get('stamp_ns'))
        config = sample.get('detector_config')
        config_id = digest(json.dumps(config, sort_keys=True, separators=(',', ':'), allow_nan=False).encode())
        if config != capture_manifest.get('detector_config') or config_id != sample.get('detector_config_id'):
            row['audit_issues'].append('detector_configuration_mismatch')
        if (sample.get('sequence') != sequence or sample.get('case_id') != case_id or
                sample.get('capture_session') != session or sample.get('stamp_ns') != item.get('stamp_ns')):
            row['audit_issues'].append('sample_binding_mismatch')
        stamp = sample.get('stamp_ns')
        if type(stamp) is not int or stamp <= 0 or previous_stamp is not None and stamp <= previous_stamp:
            row['audit_issues'].append('nonadvancing_or_invalid_stamp')
        if type(stamp) is int:
            previous_stamp = stamp
        files = sample.get('files', {})
        for filename, reference in files.items():
            full_name = f'sample_{sequence:03d}/'+filename
            if artifacts.get(full_name) != reference or full_name not in verified:
                row['audit_issues'].append('sample_file_unverified:'+filename)
        meta = sample.get('input_metadata', {})
        for kind in ('rgb', 'depth'):
            name = f'sample_{sequence:03d}/'+kind+'.data'
            data, image = verified.get(name), meta.get(kind) or {}
            if (data is None or digest(data) != image.get('data_sha256') or len(data) != image.get('data_bytes')):
                row['audit_issues'].append('raw_metadata_mismatch:'+kind)
        detections = sample.get('detections')
        processing = sample.get('processing')
        if isinstance(detections, list) and len(detections) <= MAX_PIXELS:
            row['detection_count'] = len(detections)
            if not isinstance(processing, dict) or processing.get('detection_count') != len(detections):
                row['audit_issues'].append('processing_detection_count_mismatch')
        elif detections is not None:
            row['audit_issues'].append('invalid_detections')
        if item.get('detection_count') != row['detection_count']:
            row['audit_issues'].append('summary_detection_count_mismatch')
        row.update(processing_status=sample.get('processing_status'), processing=processing,
                   detections_optical_frame=detections, input_metadata=meta,
                   reported_reasons=sample.get('reasons'), input_reasons=sample.get('input_reasons'),
                   late_conflict=sequence in conflicted, tf=tf_check(sample, capture_manifest))
        mask_name = f'sample_{sequence:03d}/red_mask.png'
        row['mask'] = None
        if mask_name in verified:
            try:
                row['mask'] = mask_metrics(verified[mask_name])
                rgb = meta.get('rgb') or {}
                if (row['mask']['width'], row['mask']['height']) != (rgb.get('width'), rgb.get('height')):
                    row['audit_issues'].append('mask_image_dimensions_mismatch')
            except ValueError as exc:
                row['audit_issues'].append('mask:'+str(exc))
        elif sample.get('processing_status') == 'processed':
            row['audit_issues'].append('processed_mask_missing')
        row['input_valid_when_saved'] = sample.get('frame_valid') is True
        if item.get('frame_valid') != sample.get('frame_valid'):
            row['audit_issues'].append('summary_frame_valid_mismatch')
        row['input_valid_at_collection_end'] = bool(row['input_valid_when_saved'] and
            not sample.get('reasons') and sample.get('processing_status') == 'processed' and
            row['tf']['recorded_exact_lookup_consistent'] and not row['audit_issues'] and sequence not in conflicted)
    fixed_complete = (len(samples) == N and all(row['status'] == 'saved' for row in rows) and
                      summary.get('requested_pairs') == N and summary.get('saved_pairs') == N and
                      summary.get('selected_pairs') == N and summary.get('collection_complete') is True)
    all_valid = all(row['input_valid_at_collection_end'] for row in rows)
    if not fixed_complete:
        issues.append('fixed_six_collection_incomplete')
    if summary.get('all_samples_valid') is True and not all_valid:
        issues.append('summary_all_samples_valid_not_supported')
    issues.extend('sample_%03d:%s' % (row['sequence'], issue) for row in rows for issue in row['audit_issues'])
    if summary.get('conflicting_duplicate_count', 0) or conflicts:
        issues.append('capture_duplicate_conflict')
    reader.unchanged()
    integrity_ok = not issues
    eligible = bool(integrity_ok and fixed_complete and all_valid and summary_available and
                    summary.get('passed') is True and summary.get('cleanup_complete') is True and
                    summary.get('source_unchanged') is True and not summary.get('errors') and
                    result.get('status') == 'captured' and result.get('capture_passed') is True and
                    result.get('exit_codes', {}).get('capture') == 0 and result.get('source_unchanged') is True and
                    not result.get('forced_kill_used') and not result.get('errors'))
    if eligible:
        for row in rows:
            count = row['detection_count']
            if count is not None:
                expected = control['declared_target_count']
                row['count_relation_to_declared_instances'] = 'equal' if count == expected else ('fewer' if count < expected else 'more')
    counts = [row['detection_count'] for row in rows]
    histogram = dict(Counter(str(value) for value in counts if value is not None))
    return {'schema_version': 1, 'case_id': case_id, 'fixed_window_size': N,
            'status': 'audited_control_observations' if eligible else 'incomplete_or_ineligible_evidence',
            'evidence_integrity_passed': integrity_ok, 'fixed_collection_complete': fixed_complete,
            'eligible_for_control_count_comparison': eligible,
            'capture_reported_passed': summary.get('passed'), 'runner_reported_status': result.get('status'),
            'recognition_success': None, 'recognition_success_evaluated': False,
            'control': control, 'frames': rows,
            'summary': {'detection_count_by_sequence': counts, 'detection_count_histogram': histogram,
                        'missing_or_ambiguous_frames': sum(row['status'] != 'saved' for row in rows),
                        'input_valid_frames': sum(row['input_valid_at_collection_end'] for row in rows),
                        'frames_with_nonzero_detections': sum(value is not None and value > 0 for value in counts),
                        'mask_red_pixels_by_sequence': [row['mask']['red_pixels'] if row.get('mask') else None for row in rows]},
            'capture_errors': summary.get('errors'), 'selected_sample_conflicts': conflicts,
            'audit_issues': list(dict.fromkeys(issues)), 'sources': reader.files,
            'scorer_sha256': digest(Path(__file__).read_bytes()),
            'limits': ['Fixed original sequences 1..6, including rejects/missing; no successful-frame replacement.',
                       'Count equality does not prove class identity, object association or localization correctness.',
                       'Mask is saved algorithm output, not a verified ground-truth pixel mask.',
                       'No detector rerun; schema/hash checks do not authenticate an adversarially rewritten dataset.',
                       'Native settled pose not verified; no absolute world-position error is computed.',
                       'Six adjacent frames are not six independent scenes; no accuracy/recall/range/coverage inference.'],
            'world_position_error_computed': False, 'effective_range_claimed': False,
            'accuracy_claimed': False, 'visual_coverage_claimed': False}


def score_batch(batch_dir, fixture_manifest):
    """Keep every planned case, including explicit unrun six-slot placeholders."""
    batch_dir, fixture_manifest = Path(batch_dir).resolve(), Path(fixture_manifest).resolve()
    reader = Reader()
    plan, batch = reader.json(batch_dir/'plan.json'), reader.json(batch_dir/'batch.json')
    planned, recorded = plan.get('cases'), batch.get('cases')
    if (batch.get('status') not in ('capture_batch_complete', 'stopped_incomplete') or
            not isinstance(planned, list) or not 1 <= len(planned) <= 12 or not isinstance(recorded, list)):
        raise ValueError('Batch is not a recorded closed 1..12 case experiment')
    if digest(reader.read(fixture_manifest, MAX_JSON)) != plan.get('manifest_sha256'):
        raise ValueError('Batch fixture manifest hash mismatch')
    ids = [item.get('id') for item in planned]
    if any(not isinstance(value, str) or not LABEL.fullmatch(value) for value in ids) or len(set(ids)) != len(ids):
        raise ValueError('Invalid or duplicate planned case IDs')
    records = {}
    for item in recorded:
        case_id = item.get('case_id')
        if case_id not in ids or case_id in records:
            raise ValueError('Unexpected or duplicate recorded case')
        ensure_closed(item)
        records[case_id] = item
    cases = []
    for entry in planned:
        case_id = entry['id']
        directory = child(batch_dir, case_id)
        if case_id in records:
            if reader.json(directory/'result.json') != records[case_id]:
                raise ValueError('Batch/case terminal report mismatch: '+case_id)
            if records[case_id].get('world_sha256') != entry.get('world_sha256'):
                raise ValueError('Planned/executed world mismatch: '+case_id)
            report = score_case(directory, fixture_manifest)
            report['execution_state'] = 'ran'
        elif directory.exists():
            raise ValueError('Unlisted case artifacts exist; cannot call it not_run: '+case_id)
        else:
            control = fixture(reader, fixture_manifest, case_id, {'world_sha256': entry.get('world_sha256')})
            report = {'case_id': case_id, 'execution_state': 'not_run', 'status': 'not_run',
                      'fixed_window_size': N, 'evidence_integrity_passed': None,
                      'eligible_for_control_count_comparison': False,
                      'recognition_success': None, 'recognition_success_evaluated': False,
                      'control': control,
                      'frames': [{'sequence': sequence, 'status': 'not_run', 'detection_count': None,
                                  'input_valid_at_collection_end': None,
                                  'count_relation_to_declared_instances': None} for sequence in range(1,N+1)],
                      'summary': {'detection_count_by_sequence': [None]*N, 'mask_red_pixels_by_sequence': [None]*N,
                                  'input_valid_frames': None},
                      'interpretation': 'Planned case did not run; no positive/negative observation or zero-detection inference.'}
        cases.append(report)
    reader.unchanged()
    eligible = sum(case['eligible_for_control_count_comparison'] for case in cases)
    ran = sum(case['execution_state'] == 'ran' for case in cases)
    return {'schema_version': 1, 'scope': 'all_planned_cases_in_original_order',
            'status': 'all_planned_captures_eligible' if eligible == len(cases) else 'planned_batch_has_ineligible_or_unrun_cases',
            'batch_reported_status': batch['status'], 'batch_reported_errors': batch.get('errors'),
            'recorded_experiment_completed_all_cases': batch.get('completed_all_cases'),
            'planned_case_count': len(cases), 'ran_case_count': ran, 'not_run_case_count': len(cases)-ran,
            'eligible_case_count': eligible, 'ran_but_ineligible_case_count': ran-eligible,
            'planned_frame_slots': len(cases)*N, 'all_planned_cases_have_eligible_capture': eligible == len(cases),
            'cases': cases, 'sources': reader.files, 'scorer_sha256': digest(Path(__file__).read_bytes()),
            'recognition_success_evaluated': False, 'world_position_error_computed': False,
            'effective_range_claimed': False, 'accuracy_claimed': False,
            'limits': ['Planned denominator is retained; not_run cases are not negatives and have null observations.',
                       'Counts are descriptive controls only, with no identity/recall/accuracy/range estimate.']}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--case-dir', type=Path)
    mode.add_argument('--batch-dir', type=Path)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    output = args.output.resolve()
    input_dir = args.case_dir or args.batch_dir
    if output.exists() or output.is_relative_to(input_dir.resolve()) or output.is_relative_to(args.manifest.resolve().parent):
        parser.error('Output must be new and outside input case/fixture directories')
    try:
        report = (score_case(args.case_dir, args.manifest) if args.case_dir
                  else score_batch(args.batch_dir, args.manifest))
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open('x', encoding='utf-8') as stream:
            json.dump(report, stream, indent=2, ensure_ascii=False, allow_nan=False)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
    except (OSError, ValueError, TypeError, KeyError) as exc:
        print(type(exc).__name__+': '+str(exc), file=sys.stderr)
        return 2
    compact = {'output': str(output), 'status': report['status'], 'recognition_success_evaluated': False}
    if args.case_dir:
        compact['detection_counts'] = report['summary']['detection_count_by_sequence']
        eligible = report['eligible_for_control_count_comparison']
    else:
        compact.update({key: report[key] for key in ('planned_case_count','ran_case_count','not_run_case_count','eligible_case_count')})
        eligible = report['all_planned_cases_have_eligible_capture']
    print(json.dumps(compact))
    return 0 if eligible else 1


if __name__ == '__main__':
    raise SystemExit(main())
