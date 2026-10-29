#!/usr/bin/env python3
"""Plot confirmed mission targets on a saved ROS map without changing occupancy.

Requires PyYAML and Pillow. Use the map saved from the same live SLAM session as
the mission result; a frame name alone does not establish map-session identity.
"""
import argparse
import base64
import html
from io import BytesIO
import json
from math import cos, isfinite, sin
from pathlib import Path


def world_to_image(x, y, origin, resolution, height):
    if len(origin) != 3 or not all(isfinite(v) for v in [x, y, *origin, resolution, height]) or resolution <= 0 or height <= 0:
        raise ValueError('Map geometry must be finite; resolution must be positive')
    dx, dy, yaw = x-origin[0], y-origin[1], origin[2]
    return ((cos(yaw)*dx + sin(yaw)*dy)/resolution,
            height-(-sin(yaw)*dx + cos(yaw)*dy)/resolution)


def image_contains_world_point(x, y, width, height):
    """Bounds for geometric SVG coordinates after the ROS vertical flip."""
    return 0 <= x < width and 0 < y <= height


def render(map_yaml, result_json, destination, map_epoch=None):
    import yaml
    from PIL import Image
    metadata = yaml.safe_load(Path(map_yaml).read_text(encoding='utf-8'))
    result = json.loads(Path(result_json).read_text(encoding='utf-8'))
    if result.get('frame_id') != 'map' or not result.get('coordinates_valid'):
        raise ValueError('Result must contain valid coordinates in frame map')
    if result.get('map_epoch') is not None and result['map_epoch'] != map_epoch:
        raise ValueError('Supply the matching --map-epoch; refuse ambiguous map-session identity')
    image_path = Path(metadata['image'])
    if not image_path.is_absolute():
        image_path = Path(map_yaml).resolve().parent / image_path
    with Image.open(image_path) as original:
        raster = original.convert('RGB')
        width, height = raster.size
        stream = BytesIO()
        raster.save(stream, format='PNG')
    png = base64.b64encode(stream.getvalue()).decode('ascii')
    resolution, origin = float(metadata['resolution']), list(map(float, metadata['origin']))
    # Validate even when no targets were confirmed.
    world_to_image(0.0, 0.0, origin, resolution, height)
    scale = max(width, height) / 900.0
    font, radius = 14*scale, 5*scale
    margin = 28*scale
    pieces = [f'<svg xmlns="http://www.w3.org/2000/svg" width="1000" viewBox="{-margin} {-margin} {width+2*margin} {height+3*margin}">',
              '<rect x="-100000" y="-100000" width="200000" height="200000" fill="white"/>',
              f'<image x="0" y="0" width="{width}" height="{height}" href="data:image/png;base64,{png}"/>']
    for target in result.get('targets', []):
        if not target.get('confirmed'):
            continue
        x, y = world_to_image(float(target['x']), float(target['y']), origin, resolution, height)
        # ROS map coordinates are half-open from the lower-left origin.
        # SVG y is inverted: the included lower edge is y=height, while y=0
        # is the excluded upper map edge. Values are geometric pixel positions.
        if not image_contains_world_point(x, y, width, height):
            raise ValueError('Target outside saved map; verify SLAM session/origin and saved map extent')
        label = html.escape(f"{target['uid']} {target['class_id']} ({target['x']:.2f}, {target['y']:.2f}) m")
        pieces.extend([f'<circle cx="{x}" cy="{y}" r="{radius}" fill="#dc2626" stroke="white" stroke-width="{scale}"/>',
                       f'<text x="{x+7*scale}" y="{y-7*scale}" font-size="{font}" font-family="sans-serif" fill="#b91c1c" stroke="white" stroke-width="{2*scale}" paint-order="stroke">{label}</text>'])
    caption = html.escape('Confirmed target observations | frame: map | independent overlay | mission success: '+str(result.get('mission_success', False)))
    pieces.append(f'<text x="0" y="{height+margin}" font-size="{font}" font-family="sans-serif" fill="#334155">{caption}</text></svg>')
    destination = Path(destination)
    if destination.exists():
        raise ValueError('Output exists; select a new filename')
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text('\n'.join(pieces), encoding='utf-8')
    return destination


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('map_yaml', type=Path)
    parser.add_argument('mission_json', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--map-epoch')
    args = parser.parse_args()
    try:
        print(render(args.map_yaml, args.mission_json, args.output, args.map_epoch))
    except (ValueError, KeyError, TypeError, OSError) as exc:
        parser.error(str(exc))
