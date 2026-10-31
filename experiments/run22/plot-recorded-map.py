"""Render the captured OccupancyGrid, not the failed save_map export."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm

HERE = Path(__file__).resolve().parent
save = json.loads((HERE / 'final-save.json').read_text())
grid = save['snapshot']['map']
mission = json.loads(next((HERE / 'mission').glob('mission_*.json')).read_text())
if grid['origin']['quaternion_xyzw'] != [0., 0., 0., 1.]:
    raise RuntimeError('Plot requires an unrotated captured grid')
output = HERE / 'recorded-map-and-targets.png'
if output.exists():
    raise RuntimeError('Refuse overwriting the snapshot plot')
data = np.asarray(grid['data']).reshape(grid['height'], grid['width'])
display = np.where(data < 0, 1, np.where(data >= 65, 2, 0))
x, y, _ = grid['origin']['xyz']
r = grid['resolution']
fig, ax = plt.subplots(figsize=(10, 7), layout='constrained')
fig.set_facecolor('#fafaf8')
ax.imshow(display, origin='lower', interpolation='nearest',
          extent=(x, x + grid['width']*r, y, y + grid['height']*r),
          cmap=ListedColormap(['#ffffff', '#c7c9cc', '#252c38']),
          norm=BoundaryNorm([-.5, .5, 1.5, 2.5], 3))
for index, target in enumerate(mission['targets']):
    ax.scatter(target['x'], target['y'], s=140, c='#db562e', marker='*',
               edgecolors='#8c2614', zorder=4, label='Confirmed surrogate' if index == 0 else None)
    offset = (-12, 14) if target['x'] > 2 else (12, 14)
    ax.annotate(f"Target #{target['uid']}", (target['x'], target['y']), xytext=offset,
                textcoords='offset points', ha='right' if offset[0] < 0 else 'left',
                color='#8c2614', bbox=dict(boxstyle='round,pad=.25', fc='white', ec='none', alpha=.9))
home, final = mission['home'], mission['final_pose']
ax.scatter(*home[:2], marker='o', s=100, facecolors='none', edgecolors='#2269b5',
           linewidths=2, zorder=5, label='Recorded home')
if final is not None:
    ax.scatter(*final[:2], marker='x', s=70, c='#158468', linewidths=2, zorder=6, label='Final map pose')
ax.annotate('Home / returned pose', home[:2], xytext=(30, 24), textcoords='offset points',
            arrowprops=dict(arrowstyle='-', color='#2269b5'), color='#2269b5')
ax.set(xlabel='map X (m)', ylabel='map Y (m)', aspect='equal',
       title='run22 | Recorded OccupancyGrid and target annotations')
ax.legend(loc='upper center', bbox_to_anchor=(.5, -.13), ncol=3, frameon=False)
fig.suptitle('15 accepted RGB-D pairs verified; full mapping and visual search remain incomplete',
             fontsize=11, color='#454e5c')
fig.text(.5, -.04, 'Offline snapshot rendering | save_map failed (result 255); this image is not a successful service export',
         ha='center', fontsize=9, color='#454e5c')
fig.savefig(output, dpi=170, bbox_inches='tight')
print(output)
