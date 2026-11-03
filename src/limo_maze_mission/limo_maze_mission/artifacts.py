"""Export a raw map and confirmed annotations without ROS or image libraries.

The caller supplies a map and target result from the same coordinate epoch.
An exported snapshot, including an empty target list, is not proof that every
physical target has been found. Partial maps remain explicitly partial.
"""
import hashlib
import json
import math
from pathlib import Path
import shutil
import tempfile
from xml.sax.saxutils import escape

from .core import Grid


def _finite(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def _text(value):
    return (isinstance(value, str) and bool(value.strip()) and
            all(c in '\t\n\r' or 0x20 <= ord(c) <= 0xD7FF or
                0xE000 <= ord(c) <= 0xFFFD or 0x10000 <= ord(c) <= 0x10FFFF
                for c in value))


def _json(value):
    return (json.dumps(value, ensure_ascii=False, allow_nan=False, indent=2) + '\n').encode('utf-8')


def _validated(grid, result, map_stamp_ns):
    if (not isinstance(grid, Grid) or type(grid.width) is not int or type(grid.height) is not int or
            grid.width < 1 or grid.height < 1 or
            not all(_finite(v) for v in (grid.resolution, grid.origin_x,
                                        grid.origin_y, grid.origin_yaw)) or
            grid.resolution <= 0 or len(grid.data) != grid.width * grid.height or
            any(type(v) is not int or not -1 <= v <= 100 for v in grid.data)):
        raise ValueError('Invalid raw occupancy grid')
    if (not isinstance(result, dict) or not _text(result.get('frame_id')) or
            not _text(result.get('map_epoch')) or result.get('coordinates_valid') is not True or
            type(result.get('map_complete')) is not bool or
            not isinstance(result.get('targets'), list)):
        raise ValueError('Map identity and valid coordinates are required')
    if map_stamp_ns is not None and (type(map_stamp_ns) is not int or
                                    not 0 <= map_stamp_ns <= 2**63 - 1):
        raise ValueError('Invalid map timestamp')
    targets, uids = [], set()
    c, s = math.cos(grid.origin_yaw), math.sin(grid.origin_yaw)
    for target in result['targets']:
        if (not isinstance(target, dict) or type(target.get('uid')) is not int or
                target['uid'] < 1 or target['uid'] in uids or
                not _text(target.get('class_id')) or target.get('confirmed') is not True or
                not all(_finite(target.get(key)) for key in ('x', 'y', 'z')) or
                target.get('frame_id', result['frame_id']) != result['frame_id'] or
                target.get('map_epoch', result['map_epoch']) != result['map_epoch'] or
                target.get('coordinates_valid', True) is not True):
            raise ValueError('Invalid or mismatched confirmed target')
        dx, dy = target['x'] - grid.origin_x, target['y'] - grid.origin_y
        px = (c * dx + s * dy) / grid.resolution
        py = (grid.height - (-s * dx + c * dy) / grid.resolution)
        if not (0 <= px < grid.width and 0 < py <= grid.height):
            raise ValueError('Target coordinate is outside the raw map')
        # Also reject NaN or non-JSON metadata rather than writing ambiguous data.
        try:
            clean = json.loads(_json(target))
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError('Invalid target metadata') from exc
        targets.append((clean, px, py))
        uids.add(target['uid'])
    return sorted(targets, key=lambda item: item[0]['uid'])


def _svg(grid, targets, identity):
    size = max(.8, max(grid.width, grid.height) / 100)
    lines = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="-2 {-8*size:g} '
             f'{grid.width+4:g} {grid.height+10*size:g}" width="1000" role="img">',
             '<title>Map with confirmed target annotations</title>',
             '<rect x="-2" y="-1000" width="100000" height="100000" fill="white"/>',
             f'<text x="0" y="{-5*size:g}" font-family="sans-serif" font-size="{1.5*size:g}">'
             f'frame: {escape(identity["frame_id"])}; map complete: '
             f'{str(identity["map_complete"]).lower()}</text>',
             f'<text x="0" y="{-2*size:g}" font-family="sans-serif" font-size="{size:g}">'
             'Confirmed observations only; total physical target count unknown.</text>']
    colors = {0: '#181818', 205: '#b8b8b8', 254: '#fafafa'}
    for row in range(grid.height):
        pixels = [_pixel(v) for v in grid.data[row*grid.width:(row+1)*grid.width]]
        start = 0
        while start < grid.width:
            stop = start + 1
            while stop < grid.width and pixels[stop] == pixels[start]:
                stop += 1
            lines.append(f'<rect x="{start}" y="{grid.height-1-row}" width="{stop-start}" '
                         f'height="1" fill="{colors[pixels[start]]}"/>')
            start = stop
    for target, px, py in targets:
        anchor, offset = ('end', -1.5*size) if px > grid.width*.6 else ('start', 1.5*size)
        label = escape(f'#{target["uid"]} {target["class_id"]} '
                       f'({target["x"]:.3f}, {target["y"]:.3f}) m')
        lines.extend([
            f'<g data-uid="{target["uid"]}" transform="translate({px:.12g} {py:.12g})">',
            f'<circle r="{size:g}" fill="#e8590c" stroke="white" stroke-width="{.2*size:g}"/>',
            f'<text x="{offset:g}" y="{-size:g}" text-anchor="{anchor}" '
            f'font-family="sans-serif" font-size="{size:g}" fill="#9b2c00" '
            f'stroke="white" stroke-width="{.15*size:g}" paint-order="stroke">{label}</text>', '</g>'])
    lines.append('</svg>')
    return ('\n'.join(lines) + '\n').encode('utf-8')


def _pixel(value):
    return 0 if value >= 65 else 254 if value == 0 else 205


def export_bundle(output_dir, stem, grid, result, *, map_stamp_ns=None):
    """Atomically create ``<stem>_map`` and return verified artifact metadata.

    Refuse existing destinations, invalid coordinates and inconsistent optional
    per-target identities. Validation and write errors propagate to the caller;
    no successful bundle is reported until every file has been re-read and
    hashed and the temporary directory has been renamed into place.
    """
    if (not isinstance(stem, str) or not stem or stem in ('.', '..') or
            any(c in stem for c in '/\\:<>"|?*') or stem[-1] in ' .'):
        raise ValueError('stem must be a single safe filename component')
    targets = _validated(grid, result, map_stamp_ns)
    identity = {'frame_id': result['frame_id'], 'map_epoch': result['map_epoch'],
                'coordinates_valid': True, 'map_complete': result['map_complete'],
                'map_stamp_ns': map_stamp_ns}
    map_data = {'schema_version': 1, **identity, 'width': grid.width, 'height': grid.height,
                'resolution': grid.resolution, 'origin_x': grid.origin_x,
                'origin_y': grid.origin_y, 'origin_yaw': grid.origin_yaw,
                'data': list(grid.data), 'source': 'raw_occupancy_grid',
                'image_encoding': {'occupied_min': 65, 'free_value': 0,
                                   'other_values': 'unknown'}}
    target_data = {'schema_version': 1, **identity, 'targets': [t for t, _, _ in targets],
                   'confirmed_target_count': len(targets), 'target_count_prior': None,
                   'all_targets_found': None}
    pixels = bytes(_pixel(grid.data[row*grid.width + col])
                   for row in reversed(range(grid.height)) for col in range(grid.width))
    yaml = ('image: map.pgm\nmode: trinary\n'
            f'resolution: {grid.resolution!r}\n'
            f'origin: [{grid.origin_x!r}, {grid.origin_y!r}, {grid.origin_yaw!r}]\n'
            'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    payloads = {'map.pgm': f'P5\n{grid.width} {grid.height}\n255\n'.encode('ascii') + pixels,
                'map.yaml': yaml.encode('utf-8'), 'map.json': _json(map_data),
                'targets.json': _json(target_data), 'annotated.svg': _svg(grid, targets, identity)}
    output = Path(output_dir).expanduser()
    destination = output / (stem + '_map')
    if destination.exists() or destination.is_symlink():
        raise FileExistsError(str(destination))
    output.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix='.' + stem + '_map-', dir=output))
    try:
        files = {}
        for name, content in payloads.items():
            path = temporary / name
            path.write_bytes(content)
            actual = path.read_bytes()
            expected = hashlib.sha256(content).hexdigest()
            if actual != content or hashlib.sha256(actual).hexdigest() != expected:
                raise OSError('Artifact verification failed: ' + name)
            files[name] = {'sha256': expected, 'bytes': len(actual)}
        manifest = {'schema_version': 1, **identity, 'files': files,
                    'confirmed_target_count': len(targets), 'all_targets_found': None}
        manifest_bytes = _json(manifest)
        (temporary / 'manifest.json').write_bytes(manifest_bytes)
        if (temporary / 'manifest.json').read_bytes() != manifest_bytes:
            raise OSError('Manifest verification failed')
        # Re-check after writing to avoid replacing a destination created meanwhile.
        if destination.exists() or destination.is_symlink():
            raise FileExistsError(str(destination))
        temporary.rename(destination)
    except BaseException:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    return {'status': 'saved', 'directory': str(destination),
            'manifest': str(destination / 'manifest.json'),
            'manifest_sha256': hashlib.sha256(manifest_bytes).hexdigest(), **manifest}
