"""Validate map serialization, coordinate transforms and failed publication."""
import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/limo_maze_mission'))
from limo_maze_mission.artifacts import export_bundle
from limo_maze_mission.core import Grid


class MappingArtifactTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.output = Path(self.temp.name)
        self.grid = Grid(3, 2, .5, 10., 20., math.pi/2, (0, 64, 65, -1, 100, 1))
        x, y = self.grid.to_world((1, 0))
        self.result = {'frame_id': 'map', 'map_epoch': 'epoch-1', 'coordinates_valid': True,
                       'map_complete': True, 'targets': [
                           {'uid': 7, 'class_id': 'red<&>', 'x': x, 'y': y, 'z': .2,
                            'confirmed': True, 'hits': 3}]}

    def export(self, **kwargs):
        return export_bundle(self.output, 'trial', self.grid, self.result,
                             map_stamp_ns=123456789, **kwargs)

    def test_raw_grid_image_conventions_yaml_rotation_and_manifest(self):
        receipt = self.export()
        directory = Path(receipt['directory'])
        self.assertEqual((directory / 'map.pgm').read_bytes(),
                         b'P5\n3 2\n255\n' + bytes([205, 0, 205, 254, 205, 0]))
        raw = json.loads((directory / 'map.json').read_text())
        self.assertEqual(raw['data'], list(self.grid.data))
        self.assertEqual(raw['origin_yaw'], math.pi/2)
        self.assertEqual(raw['map_stamp_ns'], 123456789)
        self.assertIn(f'origin: [10.0, 20.0, {math.pi/2!r}]',
                      (directory / 'map.yaml').read_text())
        manifest = json.loads((directory / 'manifest.json').read_text())
        self.assertEqual(set(manifest['files']),
                         {'map.pgm', 'map.yaml', 'map.json', 'targets.json', 'annotated.svg'})
        for name, record in manifest['files'].items():
            content = (directory / name).read_bytes()
            self.assertEqual(record, {'bytes': len(content), 'sha256': hashlib.sha256(content).hexdigest()})
        self.assertEqual(receipt['manifest_sha256'],
                         hashlib.sha256((directory / 'manifest.json').read_bytes()).hexdigest())

    def test_svg_world_to_rotated_map_and_image_y_flip_with_escaped_label(self):
        directory = Path(self.export()['directory'])
        svg = ET.parse(directory / 'annotated.svg').getroot()
        marker = next(item for item in svg.iter() if item.attrib.get('data-uid') == '7')
        self.assertEqual(marker.attrib['transform'], 'translate(1.5 1.5)')
        label = next(item for item in marker if item.tag.endswith('text')).text
        self.assertIn('#7 red<&>', label)
        self.assertIn('(9.750, 20.750) m', label)
        saved = json.loads((directory / 'targets.json').read_text())
        self.assertEqual(saved['targets'], self.result['targets'])
        self.assertIsNone(saved['all_targets_found'])

    def test_empty_targets_partial_map_do_not_claim_completion(self):
        self.result.update(targets=[], map_complete=False)
        directory = Path(self.export()['directory'])
        saved = json.loads((directory / 'targets.json').read_text())
        self.assertEqual(saved['confirmed_target_count'], 0)
        self.assertFalse(saved['map_complete'])
        self.assertIsNone(saved['all_targets_found'])

    def test_reject_invalid_identity_coordinates_and_duplicate_ids_without_files(self):
        mutations = [lambda r: r.update(coordinates_valid=False),
                     lambda r: r.update(map_epoch=''), lambda r: r.update(frame_id=''),
                     lambda r: r['targets'][0].update(x=float('nan')),
                     lambda r: r['targets'][0].update(y=float('inf')),
                     lambda r: r['targets'][0].update(x=10**1000),
                     lambda r: r['targets'][0].update(z=True),
                     lambda r: r['targets'][0].update(x=1000.),
                     lambda r: r['targets'][0].update(map_epoch='different'),
                     lambda r: r['targets'][0].update(frame_id='odom'),
                     lambda r: r['targets'][0].update(coordinates_valid=False),
                     lambda r: r['targets'][0].update(confirmed=False),
                     lambda r: r['targets'].append(dict(r['targets'][0])),
                     lambda r: r['targets'][0].update(hits=float('nan')),
                     lambda r: r['targets'][0].update(class_id='bad\x00label')]
        original = json.dumps(self.result)
        for mutate in mutations:
            with self.subTest(mutate=mutate):
                self.result = json.loads(original)
                mutate(self.result)
                with self.assertRaises(ValueError):
                    self.export()
                self.assertEqual(list(self.output.iterdir()), [])

    def test_invalid_map_values_stamp_and_stem_rejected(self):
        with self.assertRaises(ValueError):
            export_bundle(self.output, 'trial', None, self.result)
        self.grid = Grid(1, 1, 1., 0., 0., 0., (101,))
        with self.assertRaises(ValueError):
            self.export()
        self.grid = Grid(1, 1, 1., 0., 0., 0., (0,))
        self.result['targets'] = []
        for stamp in (-1, True, float('nan')):
            with self.assertRaises(ValueError):
                export_bundle(self.output, 'trial', self.grid, self.result, map_stamp_ns=stamp)
        for stem in ('../escape', 'a/b', 'a\\b', '.', '..', 'name:stream', 'trailing.'):
            with self.assertRaises(ValueError):
                export_bundle(self.output, stem, self.grid, self.result)
        self.assertEqual(list(self.output.iterdir()), [])

    def test_existing_bundle_never_overwritten(self):
        receipt = self.export()
        manifest = Path(receipt['manifest']).read_bytes()
        self.result['targets'] = []
        with self.assertRaises(FileExistsError):
            self.export()
        self.assertEqual(Path(receipt['manifest']).read_bytes(), manifest)
        self.assertEqual(len(list(self.output.iterdir())), 1)

    def test_failed_write_or_atomic_rename_leaves_no_published_bundle(self):
        for method in ('write_bytes', 'rename'):
            with self.subTest(method=method), patch.object(Path, method, side_effect=OSError('disk failure')):
                with self.assertRaisesRegex(OSError, 'disk failure'):
                    self.export()
            self.assertEqual(list(self.output.iterdir()), [])

    def test_failed_readback_verification_never_publishes_bundle(self):
        read_bytes = Path.read_bytes

        def corrupted(path):
            return b'corrupted' if path.name == 'map.pgm' else read_bytes(path)

        with patch.object(Path, 'read_bytes', corrupted):
            with self.assertRaisesRegex(OSError, 'Artifact verification failed: map.pgm'):
                self.export()
        self.assertEqual(list(self.output.iterdir()), [])


if __name__ == '__main__':
    unittest.main()
