"""Offline contracts for map orientation, unknowns, identity and failure paths."""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import yaml

import export_occupancy_map as exporter


def fixture():
    return {'passed': False, 'errors': ['export_incomplete:/slam_toolbox/save_map'],
            'exports': {'/slam_toolbox/save_map': {'result': 255}},
            'snapshot': {'valid': True, 'errors': [], 'captured_ros_ns': 1235000000,
                'epoch': {'raw': 'fixture-epoch'}, 'map': {
                    'width': 3, 'height': 2, 'resolution': 0.05, 'frame_id': 'map',
                    'stamp_ns': 1230000000, 'map_load_time_ns': 0,
                    'origin': {'xyz': [-1., 2., 0.], 'quaternion_xyzw': [0., 0., 0., 1.], 'valid_geometry': True},
                    'data': [0, -1, 100, 26, 25, 65]}}}


class OccupancyMapExportTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.source = self.root / 'snapshot.json'
        self.prefix = self.root / 'captured-map'
        self.document = fixture()

    def tearDown(self):
        self.temporary.cleanup()

    def save(self, document=None):
        self.source.write_text(json.dumps(document or self.document), encoding='utf-8')

    def export(self, **kwargs):
        self.save()
        return exporter.export_snapshot(self.source, self.prefix, **kwargs)

    def pixels(self):
        return (self.root / 'captured-map.pgm').read_bytes().split(b'\n', 3)[3]

    def test_asymmetric_grid_flips_rows_only_and_preserves_unknown(self):
        report = self.export()
        self.assertEqual((self.root / 'captured-map.pgm').read_bytes(),
                         b'P5\n3 2\n255\n' + bytes([205, 254, 0, 254, 205, 0]))
        self.assertEqual(report['verification']['verified_cells'], 6)
        self.assertEqual(report['verification']['class_counts'], {'-1': 2, '0': 2, '100': 2})
        self.assertEqual(json.loads(self.source.read_text())['snapshot']['map']['data'], [0, -1, 100, 26, 25, 65])

    def test_inclusive_input_thresholds_and_intermediate_cells(self):
        grid = self.document['snapshot']['map']
        grid.update(width=4, height=2, data=[-1, 0, 25, 26, 64, 65, 99, 100])
        self.export()
        self.assertEqual(self.pixels(), bytes([205, 0, 0, 0, 205, 254, 254, 205]))

    def test_custom_input_cuts_round_ties_to_even_but_reload_cuts_are_fixed(self):
        self.document['snapshot']['map'].update(width=4, height=1, data=[24, 25, 65, 66])
        report = self.export(free_thresh=.245, occupied_thresh=.655)
        self.assertEqual(self.pixels(), bytes([254, 205, 205, 0]))
        self.assertEqual(report['input_classification']['free_max_cell_inclusive'], 24)
        self.assertEqual(report['input_classification']['occupied_min_cell_inclusive'], 66)
        self.assertEqual(report['yaml']['free_thresh'], .196)
        self.assertEqual(report['yaml']['occupied_thresh'], .65)

    def test_rounded_coincident_threshold_uses_free_first(self):
        self.document['snapshot']['map'].update(width=2, height=1, data=[25, 26])
        self.export(free_thresh=.251, occupied_thresh=.254)
        self.assertEqual(self.pixels(), bytes([254, 0]))

    def test_standard_yaml_parser_and_nav2_trinary_unknown_reload(self):
        self.prefix = self.root / '地图 [capture]'
        report = self.export()
        metadata = yaml.safe_load((self.root / '地图 [capture].yaml').read_text())
        self.assertEqual(metadata['image'], '地图 [capture].pgm')
        self.assertEqual(metadata, report['yaml'])
        # 205 must lie strictly between the reload thresholds, otherwise an
        # original unknown cell could silently become free on map_server load.
        unknown_probability = 1 - 205/255
        self.assertGreater(unknown_probability, metadata['free_thresh'])
        self.assertLess(unknown_probability, metadata['occupied_thresh'])
        self.assertEqual(metadata['negate'], 0)
        self.assertEqual(metadata['mode'], 'trinary')

    def test_rotated_metric_origin_preserves_lower_left_corner(self):
        self.document['snapshot']['map']['origin'].update(
            xyz=[-1.25, 2.5, 0.], quaternion_xyzw=[0., 0., math.sqrt(.5), math.sqrt(.5)])
        report = self.export()
        x, y, yaw = report['yaml']['origin']
        self.assertEqual([x, y], [-1.25, 2.5])
        self.assertAlmostEqual(yaw, math.pi/2)
        # The first cell's centre is half a cell along both local axes; the
        # exported origin itself must not include that shift.
        centre = (x + .025*math.cos(yaw) - .025*math.sin(yaw),
                  y + .025*math.sin(yaw) + .025*math.cos(yaw))
        self.assertAlmostEqual(centre[0], -1.275)
        self.assertAlmostEqual(centre[1], 2.525)
        self.assertEqual(self.pixels(), bytes([205, 254, 0, 254, 205, 0]))

    def test_double_precision_origin_and_resolution_are_not_rounded(self):
        grid = self.document['snapshot']['map']
        grid['resolution'] = .05000000074505806
        grid['origin']['xyz'] = [-.5160506743998308, -2.1784644387705105, 0.]
        report = self.export()
        metadata = yaml.safe_load((self.root / 'captured-map.yaml').read_text())
        self.assertEqual(metadata['resolution'], grid['resolution'])
        self.assertEqual(metadata['origin'][:2], grid['origin']['xyz'][:2])
        self.assertEqual(report['map_metadata']['origin'], grid['origin'])

    def test_source_hash_stamp_frame_and_original_service_failure_are_preserved(self):
        self.save()
        before = self.source.read_bytes()
        report = exporter.export_snapshot(self.source, self.prefix)
        self.assertEqual(self.source.read_bytes(), before)
        self.assertEqual(report['source_sha256'], hashlib.sha256(before).hexdigest())
        self.assertEqual(report['map_stamp_ns'], 1230000000)
        self.assertEqual(report['frame_id'], 'map')
        self.assertFalse(report['source_report_export_status']['passed'])
        self.assertEqual(report['source_report_export_status']['exports']['/slam_toolbox/save_map']['result'], 255)
        self.assertTrue(report['passed'])
        self.assertEqual(json.loads((self.root / 'captured-map.provenance.json').read_text()), report)

    def test_invalid_snapshot_geometry_and_cells_fail_before_any_output(self):
        mutations = [
            lambda d: d['snapshot'].update(valid=False),
            lambda d: d['snapshot'].update(valid=1),
            lambda d: d['snapshot'].update(errors=['bad']),
            lambda d: d['snapshot']['map'].update(width=True),
            lambda d: d['snapshot']['map'].update(width=0),
            lambda d: d['snapshot']['map'].update(height=2.0),
            lambda d: d['snapshot']['map'].update(width=exporter.MAX_CELLS, height=2),
            lambda d: d['snapshot']['map'].update(resolution=0),
            lambda d: d['snapshot']['map'].update(resolution=-1),
            lambda d: d['snapshot']['map'].update(resolution=float('nan')),
            lambda d: d['snapshot']['map'].update(resolution=10**400),
            lambda d: d['snapshot']['map'].update(frame_id=''),
            lambda d: d['snapshot']['map'].update(frame_id='map\nother'),
            lambda d: d['snapshot']['map'].update(stamp_ns=-1),
            lambda d: d['snapshot']['map'].update(stamp_ns=True),
            lambda d: d['snapshot']['map'].update(data=[0]),
            lambda d: d['snapshot']['map']['data'].__setitem__(0, True),
            lambda d: d['snapshot']['map']['data'].__setitem__(0, 1.5),
            lambda d: d['snapshot']['map']['data'].__setitem__(0, -2),
            lambda d: d['snapshot']['map']['data'].__setitem__(0, 101),
            lambda d: d['snapshot']['map']['origin'].update(valid_geometry=False),
            lambda d: d['snapshot']['map']['origin'].update(xyz=[0., 0., .1]),
            lambda d: d['snapshot']['map']['origin'].update(quaternion_xyzw=[0., 0., 0., 0.]),
            lambda d: d['snapshot']['map']['origin'].update(quaternion_xyzw=[math.sqrt(.5), 0., 0., math.sqrt(.5)]),
            lambda d: d['snapshot']['map']['origin'].update(xyz=[float('inf'), 0., 0.]),
        ]
        for index, mutate in enumerate(mutations):
            with self.subTest(case=index):
                value = fixture()
                mutate(value)
                self.save(value)
                with self.assertRaises(ValueError):
                    exporter.export_snapshot(self.source, self.prefix)
                self.assertFalse(list(self.root.glob('captured-map*')))

    def test_metric_extent_overflow_rejected(self):
        grid = self.document['snapshot']['map']
        grid.update(width=1, height=1, resolution=1e308, data=[0])
        grid['origin']['xyz'] = [1e308, 0., 0.]
        self.save()
        with self.assertRaisesRegex(ValueError, 'world_extent'):
            exporter.export_snapshot(self.source, self.prefix)

    def test_invalid_thresholds_and_duplicate_json_keys_rejected(self):
        self.save()
        for free, occupied in ((-.1, .65), (.65, .65), (.7, .6), (.25, 1.1), (True, .65), (float('nan'), .65)):
            with self.subTest(free=free, occupied=occupied), self.assertRaises(ValueError):
                exporter.export_snapshot(self.source, self.prefix, free, occupied)
        self.source.write_text('{"snapshot": {}, "snapshot": {}}')
        with self.assertRaisesRegex(ValueError, 'duplicate_json_key'):
            exporter.export_snapshot(self.source, self.prefix)
        self.assertFalse(list(self.root.glob('captured-map*')))

    def test_existing_prefix_is_never_overwritten(self):
        self.export()
        before = {p.name: p.read_bytes() for p in self.root.glob('captured-map*')}
        with self.assertRaises(FileExistsError):
            exporter.export_snapshot(self.source, self.prefix)
        self.assertEqual({p.name: p.read_bytes() for p in self.root.glob('captured-map*')}, before)

    def test_partial_foreign_prefix_and_missing_parent_are_rejected(self):
        self.save()
        (self.root / 'captured-map.old').write_bytes(b'keep')
        with self.assertRaises(FileExistsError):
            exporter.export_snapshot(self.source, self.prefix)
        self.assertEqual((self.root / 'captured-map.old').read_bytes(), b'keep')
        with self.assertRaises(ValueError):
            exporter.export_snapshot(self.source, self.root / 'absent' / 'new')

    def test_actual_pgm_corruption_is_detected_by_reopening(self):
        original_write = exporter.write_new
        def corrupted(path, raw):
            original_write(path, raw)
            if path.suffix == '.pgm':
                path.write_bytes(raw[:-1] + bytes([254]))
        self.save()
        with patch.object(exporter, 'write_new', side_effect=corrupted), self.assertRaisesRegex(ValueError, 'roundtrip'):
            exporter.export_snapshot(self.source, self.prefix)
        failure = json.loads((self.root / 'captured-map.failure.json').read_text())
        self.assertFalse(failure['passed'])
        self.assertEqual(failure['stage'], 'verify_readback')
        self.assertFalse((self.root / 'captured-map.provenance.json').exists())

    def test_actual_yaml_type_corruption_and_io_failure_leave_failed_evidence(self):
        original_write = exporter.write_new
        def corrupt_yaml(path, raw):
            original_write(path, raw.replace(b'negate: 0', b'negate: false') if path.suffix == '.yaml' else raw)
        self.save()
        with patch.object(exporter, 'write_new', side_effect=corrupt_yaml), self.assertRaisesRegex(ValueError, 'yaml_readback'):
            exporter.export_snapshot(self.source, self.prefix)
        self.prefix = self.root / 'io-failure'
        def fail_yaml(path, raw):
            if path.suffix == '.yaml':
                raise OSError('controlled write failure')
            original_write(path, raw)
        with patch.object(exporter, 'write_new', side_effect=fail_yaml), self.assertRaises(OSError):
            exporter.export_snapshot(self.source, self.prefix)
        failure = json.loads((self.root / 'io-failure.failure.json').read_text())
        self.assertEqual(failure['stage'], 'write_yaml')
        self.assertTrue((self.root / 'io-failure.pgm').exists())
        self.assertFalse((self.root / 'io-failure.provenance.json').exists())

    def test_cli_reports_success_then_refuses_same_prefix(self):
        self.save()
        command = [sys.executable, '-B', str(Path(exporter.__file__)), '--snapshot', str(self.source),
                   '--output-prefix', str(self.prefix)]
        first = subprocess.run(command, capture_output=True, text=True, timeout=10)
        self.assertEqual(first.returncode, 0, first.stderr)
        self.assertTrue(json.loads(first.stdout)['passed'])
        again = subprocess.run(command, capture_output=True, text=True, timeout=10)
        self.assertEqual(again.returncode, 1)
        self.assertFalse(json.loads(again.stderr)['passed'])


if __name__ == '__main__':
    unittest.main()
