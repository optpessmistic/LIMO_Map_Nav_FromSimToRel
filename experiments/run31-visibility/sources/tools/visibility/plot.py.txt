"""Four-panel offline depth diagnostics with the original occupancy retained."""

from collections import Counter
from io import BytesIO
import json
import math
from numbers import Integral, Real
from pathlib import Path
import textwrap


def plot_layers(data: dict, output: Path) -> None:
    """Write a new PNG containing occupancy and exactly three height layers.

    Cell arrays have OccupancyGrid row order: index = row * width + column.
    Cell edges use the planar map-origin transform; recorded camera positions
    are already in the map frame and are not transformed again. A clear state is
    a diagnostic opportunity, never object detection or search completeness.
    No target truth is read or plotted. The plotting dependencies load only here.
    """
    output = Path(output)
    if output.suffix.lower() != '.png':
        raise ValueError('Visibility plot output must have a .png suffix')
    if output.exists() or output.is_symlink():
        raise FileExistsError(f'Refusing to overwrite visibility plot: {output}')
    try:
        import numpy as np
        from matplotlib import colormaps
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        from matplotlib.collections import PolyCollection
        from matplotlib.colors import BoundaryNorm, ListedColormap
        from matplotlib.figure import Figure
        from matplotlib.lines import Line2D
        from matplotlib.patches import Patch
    except ImportError as error:
        raise RuntimeError(
            'Visibility plotting requires numpy and matplotlib in the current '
            'Python environment; no packages were installed automatically.'
        ) from error

    def finite(value, name):
        try:
            valid = (not isinstance(value, bool) and isinstance(value, Real)
                     and math.isfinite(value))
        except OverflowError:
            valid = False
        if not valid:
            raise ValueError(f'{name} must be finite numeric')
        return float(value)

    def positive_integer(value, name):
        if isinstance(value, bool) or not isinstance(value, Integral) or value <= 0:
            raise ValueError(f'{name} must be a positive integer')
        return int(value)

    def vector(values, size, name):
        if isinstance(values, (str, bytes)) or not hasattr(values, '__len__') or len(values) != size:
            raise ValueError(f'{name} requires {size} coordinates')
        return [finite(value, name) for value in values]

    def ascii_text(value):
        return str(value).encode('ascii', 'backslashreplace').decode('ascii')

    grid = data['map']
    width = positive_integer(grid['width'], 'Map width')
    height = positive_integer(grid['height'], 'Map height')
    resolution = finite(grid['resolution'], 'Map resolution')
    if resolution <= 0:
        raise ValueError('Map resolution must be positive')
    cell_count = width * height
    if len(grid['data']) != cell_count:
        raise ValueError('Map cell count does not match its dimensions')
    if any(isinstance(value, bool) or not isinstance(value, Integral) or
           value < -1 or value > 100 for value in grid['data']):
        raise ValueError('Occupancy values must be integer -1 or 0..100')
    occupancy = np.asarray(grid['data'], dtype=np.int16).reshape(height, width)
    xyz = vector(grid['origin']['xyz'], 3, 'Map origin')
    qx, qy, qz, qw = vector(grid['origin']['quaternion_xyzw'], 4, 'Map quaternion')
    norm = math.hypot(qx, qy, qz, qw)
    if abs(norm - 1.) > 1e-6 or abs(qx) > 1e-8 or abs(qy) > 1e-8:
        raise ValueError('A unit, planar map-origin quaternion is required')
    qz, qw = qz / norm, qw / norm
    yaw = 2. * math.atan2(qz, qw)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    frame = grid['frame_id']
    if not isinstance(frame, str) or not frame.strip():
        raise ValueError('An explicit map frame_id is required')
    frame = ascii_text(frame)
    floor_z = finite(data['floor_z_m'], 'Nominal floor z')
    epsilon = data['uncertainty_bound_m']
    if epsilon is not None:
        epsilon = finite(epsilon, 'Uncalibrated uncertainty bound')
        if epsilon < 0:
            raise ValueError('Uncertainty bound must be nonnegative or None')
    provenance = data['height_reference_provenance']
    if provenance is None or provenance == '' or provenance == {}:
        raise ValueError('Explicit height-reference provenance is required')
    provenance_text = (json.dumps(provenance, ensure_ascii=True, sort_keys=True)
                       if isinstance(provenance, (dict, list)) else ascii_text(provenance))
    provenance_text = ' '.join(provenance_text.split())
    if len(provenance_text) > 190:
        provenance_text = provenance_text[:187] + '...'

    states = ('no_view', 'no_valid_depth', 'occluded', 'mixed_or_surface', 'nominal_clear')
    colors = ('#eeeeee', '#E69F00', '#0072B2', '#CC79A7', '#009E73')
    descriptions = (
        'no_view: no projected sample',
        'no_valid_depth: no valid depth stencil',
        'occluded: measured nearer obstruction',
        'mixed_or_surface: ambiguous depth',
        'nominal_clear: >=1 frame without measured nearer obstruction',
    )
    index_by_state = {state: index for index, state in enumerate(states)}
    if len(data['layers']) != 3:
        raise ValueError('Exactly three height layers are required')
    prepared_layers = []
    for layer in data['layers']:
        above_floor = finite(layer['height_above_floor_m'], 'Height above floor')
        map_z = finite(layer['map_z_m'], 'Layer map z')
        if above_floor < 0 or not math.isclose(map_z, floor_z + above_floor, rel_tol=0., abs_tol=1e-6):
            raise ValueError('Layer map z must equal nominal floor z + nonnegative height')
        if len(layer['states']) != cell_count or any(value not in index_by_state for value in layer['states']):
            raise ValueError('Layer states must match the map and use the five specified states')
        counts = Counter(layer['states'])
        declared = layer['counts']
        if (not isinstance(declared, dict) or any(key not in index_by_state for key in declared) or
                any(isinstance(value, bool) or not isinstance(value, Integral) or value < 0
                    for value in declared.values()) or
                any(declared.get(state, 0) != counts[state] for state in states)):
            raise ValueError('Layer counts must agree with its actual states')
        values = np.fromiter((index_by_state[state] for state in layer['states']),
                             dtype=np.uint8, count=cell_count).reshape(height, width)
        prepared_layers.append((above_floor, map_z, values, counts))
    camera_xy = []
    for entry in data['frames']:
        pose = vector(entry['camera_pose'], 7, 'Recorded camera pose')
        camera_xy.append(pose[:2])

    def transform(local_x, local_y):
        return (xyz[0] + cosine * local_x - sine * local_y,
                xyz[1] + sine * local_x + cosine * local_y)

    local_x, local_y = np.meshgrid(np.arange(width + 1) * resolution,
                                  np.arange(height + 1) * resolution)
    world_x, world_y = transform(local_x, local_y)
    if not (np.isfinite(world_x).all() and np.isfinite(world_y).all()):
        raise ValueError('Map geometry exceeds finite plotting coordinates')
    x_limits = (float(world_x.min()), float(world_x.max()))
    y_limits = (float(world_y.min()), float(world_y.max()))

    # Merge adjacent unknown cells within each row; do not merge across known
    # cells or erase their original occupancy status beneath a colored layer.
    unknown_polygons = []
    for row in range(height):
        transitions = np.diff(np.pad((occupancy[row] == -1).astype(np.int8), (1, 1)))
        for start, stop in zip(np.flatnonzero(transitions == 1), np.flatnonzero(transitions == -1)):
            corners_x = np.array([start, stop, stop, start]) * resolution
            corners_y = np.array([row, row, row + 1, row + 1]) * resolution
            px, py = transform(corners_x, corners_y)
            unknown_polygons.append(np.column_stack((px, py)))
    occupied_rows, occupied_cols = np.nonzero(occupancy > 0)
    occupied_x, occupied_y = transform((occupied_cols + .5) * resolution,
                                      (occupied_rows + .5) * resolution)
    grayscale = colormaps['Greys']
    occupied_colors = grayscale(.35 + .65 * occupancy[occupied_rows, occupied_cols] / 100.)
    cmap = ListedColormap(colors)
    norm = BoundaryNorm(np.arange(len(states) + 1) - .5, len(states))
    fig = Figure(figsize=(14, 11), facecolor='white')
    FigureCanvasAgg(fig)
    axes = fig.subplots(2, 2, sharex=True, sharey=True).ravel()
    fig.subplots_adjust(left=.07, right=.98, bottom=.245, top=.91, wspace=.11, hspace=.24)
    try:
        raw = np.ma.masked_equal(occupancy, -1)
        raw_cmap = grayscale.copy()
        raw_cmap.set_bad('#dddddd')
        axes[0].pcolormesh(world_x, world_y, raw, cmap=raw_cmap, vmin=0, vmax=100,
                           shading='flat', rasterized=True)
        axes[0].set_title('Original occupancy\n0 = free; 100 = occupied; -1 = unknown\n'
                          f'Cells={cell_count}; unknown={int(np.count_nonzero(occupancy == -1))}', fontsize=9)
        for ax, (above_floor, map_z, values, counts) in zip(axes[1:], prepared_layers):
            ax.pcolormesh(world_x, world_y, values, cmap=cmap, norm=norm,
                          shading='flat', rasterized=True)
            ax.scatter(occupied_x, occupied_y, s=2., c=occupied_colors,
                       marker='o', linewidths=0, zorder=4, rasterized=True)
            ax.set_title(
                f'Height above nominal floor: {above_floor:g} m (map z={map_z:g} m)\n'
                f'Cells={cell_count}; clear={counts["nominal_clear"]}; no_view={counts["no_view"]}\n'
                f'no_depth={counts["no_valid_depth"]}; occluded={counts["occluded"]}; '
                f'mixed={counts["mixed_or_surface"]}', fontsize=9)
        for ax in axes:
            if unknown_polygons:
                ax.add_collection(PolyCollection(unknown_polygons, closed=True,
                    facecolors=(.86, .86, .86, .45), edgecolors=(.25, .25, .25, .65),
                    linewidths=0, hatch='////', zorder=3, rasterized=True))
            if camera_xy:
                positions = np.asarray(camera_xy)
                ax.scatter(positions[:, 0], positions[:, 1], s=13, marker='^',
                           facecolors='none', edgecolors='#b2182b', linewidths=.65, zorder=5)
            ax.set(xlim=x_limits, ylim=y_limits, xlabel=f'{frame} x [m]', ylabel=f'{frame} y [m]')
            ax.set_aspect('equal', adjustable='box')
            ax.tick_params(labelsize=8)
        fig.suptitle(f'Depth opportunity diagnostics | {len(camera_xy)} retained camera frames', fontsize=14)
        state_handles = [Patch(facecolor=color, edgecolor='#666666', label=label)
                         for color, label in zip(colors, descriptions)]
        occupancy_handles = [
            Patch(facecolor='#dddddd', edgecolor='#666666', hatch='////',
                  label='Unknown occupancy (hatch; not explored)'),
            Line2D([], [], color='#333333', marker='o', markersize=3, linestyle='none',
                   label='Known occupancy >0 (gray-black dots; no binary threshold)'),
            Line2D([], [], color='#b2182b', marker='^', markerfacecolor='none',
                   markersize=5, linestyle='none', label='Recorded camera positions'),
        ]
        fig.legend(handles=state_handles, loc='lower center', bbox_to_anchor=(.5, .145),
                   ncol=2, fontsize=8, frameon=False)
        fig.legend(handles=occupancy_handles, loc='lower center', bbox_to_anchor=(.5, .116),
                   ncol=3, fontsize=8, frameon=False)
        epsilon_text = ('nominal zero comparison (no uncertainty bound)' if epsilon is None
                        else f'assumed uncertainty bound={epsilon:g} m (uncalibrated)')
        footer = (
            f'Nominal floor z={floor_z:g} m; {epsilon_text}.\n'
            'Historical map poses not reprojected. Discrete depth rays, not detection coverage.\n'
            'nominal_clear is not object identification, whole-object visibility, or search completion.\n'
            'Depth bounds filter measured surfaces; query-point sensor/recognition range is not certified.\n'
            + textwrap.fill('Height reference: ' + provenance_text, width=170)
        )
        fig.text(.07, .018, footer, fontsize=8, va='bottom', ha='left', parse_math=False)
        buffer = BytesIO()
        fig.savefig(buffer, format='png', dpi=160, facecolor='white')
        payload = buffer.getvalue()
    finally:
        fig.clear()
    # Complete rendering before creating the output; exclusive creation also
    # prevents a race from overwriting a file created since the initial check.
    output.parent.mkdir(parents=True, exist_ok=True)
    created = False
    try:
        with output.open('xb') as handle:
            created = True
            handle.write(payload)
    except BaseException:
        if created:
            output.unlink(missing_ok=True)
        raise
