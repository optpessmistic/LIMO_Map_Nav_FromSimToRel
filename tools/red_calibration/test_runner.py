"""Pure planning/budget/stop-policy tests. Never imports ROS or starts a child."""
from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import runner


class RunnerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.output = self.root / 'output'
        entries = []
        for i in range(12):
            world = self.root / ('case_%02d.sdf' % i)
            world.write_text('<sdf version="1.9"><world name="fixture_%d"/></sdf>' % i)
            entries.append({'id': 'case_%02d' % i, 'world': world.name,
                            'world_sha256': runner.digest(world),
                            'offline_truth': 'MUST_NOT_READ_THIS_TRUTH_FILE.json',
                            'truth_sha256': 'irrelevant', 'target_count': 999})
        self.data = {'schema_version': 1, 'cases': entries}
        self.manifest = self.root/'manifest.json'
        self.save_manifest()

    def save_manifest(self):
        self.manifest.write_text(json.dumps(self.data))

    def plan(self, selected=None, all_cases=False, **kwargs):
        return runner.build_plan(self.manifest, selected or ([] if all_cases else ['case_00']),
                                 all_cases, self.output, **kwargs)

    def test_select_one_or_all_without_truth_input(self):
        single, batch = self.plan(), self.plan(all_cases=True)
        self.assertEqual(len(single['cases']), 1)
        self.assertEqual(len(batch['cases']), 12)
        for case in batch['cases']:
            args = case['commands']['capture']
            self.assertEqual(args[args.index('--pairs')+1], '6')
            collection = float(args[args.index('--timeout')+1])
            persistence = float(args[args.index('--save-timeout')+1])
            self.assertGreater(runner.CAPTURE_PROCESS_SECONDS, collection+persistence)
            self.assertNotIn('MUST_NOT_READ_THIS_TRUTH_FILE.json', ' '.join(args))
            self.assertNotIn(case['world'], args)
            self.assertNotIn('--world', args)
            self.assertNotIn('offline_truth', case)
        self.assertFalse(single['stationarity_verified'])
        self.assertFalse(single['motion_authorized'])

    def test_default_cli_is_plan_only_and_creates_nothing(self):
        with patch('runner.subprocess.Popen', side_effect=AssertionError('must not spawn')):
            with redirect_stdout(io.StringIO()) as output:
                code = runner.main(['--manifest', str(self.manifest), '--output', str(self.output),
                                    '--case', 'case_00'])
        self.assertEqual(code, 0)
        self.assertEqual(json.loads(output.getvalue())['status'], 'planned_not_run')
        self.assertFalse(self.output.exists())

    def test_hash_mismatch_or_world_escape_is_rejected(self):
        self.data['cases'][0]['world_sha256'] = '0'*64
        self.save_manifest()
        with self.assertRaisesRegex(ValueError, 'hash mismatch'):
            self.plan()
        other = self.root.parent/'outside-draft-fixture.sdf'
        # No external write needed: an existing absolute runner source is outside manifest root.
        self.data['cases'][0]['world'] = str(Path(runner.__file__).resolve())
        self.save_manifest()
        with self.assertRaisesRegex(ValueError, 'within'):
            self.plan()

    def test_duplicate_unknown_case_and_existing_output_rejected(self):
        for ids in (['case_00', 'case_00'], ['unknown']):
            with self.assertRaises(ValueError):
                self.plan(ids)
        self.output.mkdir()
        with self.assertRaisesRegex(ValueError, 'must not exist'):
            self.plan()

    def test_finite_budget_and_cleanup_reserve(self):
        for value in (float('nan'), float('inf'), 0., 601.):
            with self.assertRaises(ValueError):
                self.plan(case_wall=value)
        self.assertEqual(runner.case_deadline(100., 300., 500.), 332.)
        with self.assertRaises(TimeoutError):
            runner.case_deadline(100., 300., 268.)

    def test_cleanup_uncertainty_is_never_empty(self):
        clean = {'remaining_check_complete': True, 'remaining': [], 'ownership_errors': []}
        self.assertTrue(runner.cleanup_confirmed({'sim': clean}, {'sim': object()}))
        for broken in ({'remaining_check_complete': False},
                       dict(clean, remaining=[{'pid': 5}]), dict(clean, ownership_errors=['unknown']),
                       dict(clean, error='inspection failed')):
            self.assertFalse(runner.cleanup_confirmed({'sim': broken}, {'sim': object()}))
        self.assertFalse(runner.cleanup_confirmed({}, {'sim': object()}))

    def test_uncertain_cleanup_stops_next_case(self):
        calls = []
        def fake(case, plan, deadline):
            calls.append(case['id'])
            return {'status': 'captured', 'all_owned_groups_empty': False}
        result = runner.run_batch(self.plan(all_cases=True), case_runner=fake, clock=lambda: 0.)
        self.assertEqual(calls, ['case_00'])
        self.assertFalse(result['completed_all_cases'])
        self.assertIn('Cleanup unconfirmed', result['errors'][0])

    def test_capture_rejection_stops_without_retrying_selection(self):
        calls = []
        def fake(case, plan, deadline):
            calls.append(case['id'])
            return {'status': 'failed_or_incomplete', 'all_owned_groups_empty': True,
                    'capture_passed': False}
        result = runner.run_batch(self.plan(all_cases=True), case_runner=fake, clock=lambda: 0.)
        self.assertEqual(len(calls), 1)
        self.assertFalse(result['completed_all_cases'])

    def test_budget_includes_previous_case_cleanup_before_next_case(self):
        now = [0.]
        calls = []
        def fake(case, plan, deadline):
            calls.append(case['id'])
            now[0] = 35.  # 200 - 168 cleanup reserve leaves only 32 active seconds.
            return {'status': 'captured', 'all_owned_groups_empty': True}
        result = runner.run_batch(self.plan(all_cases=True, batch_wall=200.),
                                  case_runner=fake, clock=lambda: now[0])
        self.assertEqual(len(calls), 1)
        self.assertFalse(result['completed_all_cases'])
        self.assertIn('Insufficient', result['errors'][0])

    def test_last_case_cleanup_overrun_cannot_report_complete(self):
        now = [0.]
        def fake(case, plan, deadline):
            now[0] = deadline + 1.
            return {'status': 'captured', 'all_owned_groups_empty': True}
        result = runner.run_batch(self.plan(), case_runner=fake, clock=lambda: now[0])
        self.assertFalse(result['completed_all_cases'])
        self.assertIn('Batch wall deadline exceeded', result['errors'][0])

    def test_all_selected_cases_success_is_capture_only_not_truth_score(self):
        result = runner.run_batch(self.plan(all_cases=True),
            case_runner=lambda *args: {'status': 'captured', 'all_owned_groups_empty': True},
            clock=lambda: 0.)
        self.assertTrue(result['completed_all_cases'])
        self.assertEqual(len(result['cases']), 12)
        self.assertFalse(result['truth_scoring_performed'])
        self.assertFalse(result['motion_authorized'])


if __name__ == '__main__':
    unittest.main()
