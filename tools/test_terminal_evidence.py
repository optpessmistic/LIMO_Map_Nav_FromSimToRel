"""Independent terminal evidence must survive a failed map-export operation."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from terminal_evidence import collect_terminal_evidence


class TerminalEvidenceTests(unittest.TestCase):
    def test_failed_export_does_not_skip_second_export_or_stationary_tail(self):
        calls = []

        def failed_capture(deadline):
            self.assertEqual(deadline, 110.)
            calls.append('capture')
            raise RuntimeError('map service returned 255')

        def raster(deadline):
            self.assertEqual(deadline, 110.)
            calls.append('raster')

        def tail(deadline):
            self.assertEqual(deadline, 110.)
            calls.append('tail')
            return True

        report = collect_terminal_evidence([('capture', failed_capture), ('raster', raster)], tail,
                                           deadline_wall=110., monotonic=lambda: 100.)
        self.assertEqual(calls, ['capture', 'raster', 'tail'])
        self.assertTrue(report['tail_verified'])
        self.assertFalse(report['passed'])
        self.assertEqual([item['passed'] for item in report['exports']], [False, True])
        self.assertIn('map service returned 255', report['errors'][0])

    def test_export_success_cannot_substitute_for_verified_terminal_tail(self):
        for value in (False, None, {'stationary': True}, 1):
            with self.subTest(value=value):
                report = collect_terminal_evidence([('capture', lambda end: None)], lambda end: value,
                                                   deadline_wall=110., monotonic=lambda: 100.)
                self.assertTrue(report['exports'][0]['passed'])
                self.assertFalse(report['tail_verified'])
                self.assertFalse(report['passed'])

    def test_original_interrupt_is_not_swallowed_or_followed_by_more_work(self):
        calls = []

        def interrupt(deadline):
            raise KeyboardInterrupt()

        with self.assertRaises(KeyboardInterrupt):
            collect_terminal_evidence([('capture', interrupt)], lambda end: calls.append('tail'),
                                      deadline_wall=110., monotonic=lambda: 100.)
        self.assertEqual(calls, [])

    def test_tail_exception_remains_failure_after_successful_exports(self):
        def failed_tail(deadline):
            raise TimeoutError('original experiment deadline')

        report = collect_terminal_evidence([('capture', lambda end: None)], failed_tail,
                                          deadline_wall=110., monotonic=lambda: 100.)
        self.assertFalse(report['passed'])
        self.assertFalse(report['tail_verified'])
        self.assertIn('original experiment deadline', report['errors'][0])

    def test_expired_budget_never_launches_a_later_export_or_wait(self):
        clock, calls = [100.], []

        def slow_capture(deadline):
            clock[0] = deadline
            raise TimeoutError('capture exhausted remaining budget')

        report = collect_terminal_evidence([
            ('capture', slow_capture), ('raster', lambda end: calls.append('raster'))],
            lambda end: calls.append('tail'), deadline_wall=110., monotonic=lambda: clock[0])
        self.assertEqual(calls, [])
        self.assertFalse(report['passed'])
        self.assertFalse(report['tail_verified'])
        self.assertIn('before export launch', report['exports'][1]['error'])

    def test_late_success_does_not_move_or_reset_deadline(self):
        clock = [100.]

        def late_tail(deadline):
            clock[0] = deadline + .01
            return True

        report = collect_terminal_evidence([], late_tail, deadline_wall=110., monotonic=lambda: clock[0])
        self.assertEqual(report['deadline_wall'], 110.)
        self.assertFalse(report['tail_verified'])
        self.assertFalse(report['passed'])


if __name__ == '__main__':
    unittest.main()
