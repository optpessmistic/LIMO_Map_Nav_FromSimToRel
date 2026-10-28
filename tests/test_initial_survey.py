import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from initial_survey import (Survey, SurveyAbort, angle_delta, authorization, scan_clearance,
                            OwnedSpin, foreign_active, stop_burst, stopped_after)


def ready():
    state = Survey()
    state.on_clock(10, 10)
    state.on_odom(10, 10, 0, 0, 0, 0, 0)
    state.scan = (10, 10, .4, 720)
    return state


class InitialSurveyTests(unittest.TestCase):
    def test_stop_burst_continues_after_callback_and_publish_exceptions(self):
        now, attempts = [0.0], []
        def publish():
            attempts.append(now[0])
            if len(attempts) == 1:
                raise RuntimeError('injected publish failure')
        def spin():
            now[0] += .02
            raise AttributeError('injected callback failure')
        with patch('initial_survey.time.sleep'):
            count, errors = stop_burst(publish, spin, lambda: True, duration=.25, clock=lambda: now[0])
        self.assertGreaterEqual(len(attempts), 4)
        self.assertEqual(count, len(attempts)-1)
        self.assertEqual(len(errors), 2)
        self.assertGreaterEqual(now[0], .25)

    def test_motion_requires_every_explicit_authorization(self):
        authorization(False, None, '0', False)  # Read-only has no motion authorization.
        authorization(True, .30, '73', True)
        for radius, domain, sim in [(None, '73', True), (.29, '73', True),
                                   (math.nan, '73', True), (.3, '0', True), (.3, '73', False)]:
            with self.subTest(radius=radius, domain=domain, sim=sim), self.assertRaises(SurveyAbort):
                authorization(True, radius, domain, sim)

    def test_scan_distances_use_laser_to_base_translation_and_rotation(self):
        ranges = [.3]*20
        front, count = scan_clearance(ranges, 0, .0001, .02, 8, (.103, 0, 0), (0, 0, 0, 1))
        rear, _ = scan_clearance(ranges, 0, .0001, .02, 8, (.103, 0, 0), (0, 0, 1, 0))
        self.assertEqual(count, 20)
        self.assertAlmostEqual(front, .403, places=5)
        self.assertAlmostEqual(rear, .197, places=5)
        self.assertGreater(front, .28)
        self.assertLess(rear, .28)

    def test_unknown_rays_are_not_free_space_or_self_echo_filters(self):
        for ranges in [[math.inf]*20, [math.nan]*20, [-math.inf]*20, [.01]*20]:
            with self.subTest(ranges=ranges[:1]), self.assertRaises(SurveyAbort):
                scan_clearance(ranges, 0, .1, .02, 8, (0, 0, 0), (0, 0, 0, 1))

    def test_obstacle_inside_envelope_plus_margin_stops(self):
        state = ready()
        state.scan = (10, 10, .28, 720)
        with self.assertRaisesRegex(SurveyAbort, 'obstacle'):
            state.start(10)

    def test_stale_clock_scan_and_odometry_rejected(self):
        for source in ('clock', 'scan', 'odom'):
            state = ready()
            sample = getattr(state, source)
            setattr(state, source, (sample[0], 8, *sample[2:]))
            with self.subTest(source=source), self.assertRaisesRegex(SurveyAbort, 'stale'):
                state.check(10)

    def test_paused_clock_does_not_refresh_freshness(self):
        state = ready()
        state.on_clock(10, 10.8)
        with self.assertRaisesRegex(SurveyAbort, 'clock stopped'):
            state.check(10.8)

    def test_clock_rollback_and_jump_rejected(self):
        for stamp in (9.9, 10.6, math.nan):
            with self.subTest(stamp=stamp), self.assertRaises(SurveyAbort):
                ready().on_clock(stamp, 10.1)

    def test_odometry_jump_and_duplicate_stamp_motion_rejected(self):
        for stamp, x, yaw in [(10.1, .02, 0), (10.1, 0, .2), (9.9, 0, 0), (10, .001, 0)]:
            with self.subTest(stamp=stamp, x=x, yaw=yaw), self.assertRaises(SurveyAbort):
                ready().on_odom(stamp, 10.1, x, 0, yaw, 0, 0)

    def test_only_accepted_advancing_odometry_counts_or_refreshes(self):
        state = ready()
        original = state.odom
        self.assertFalse(state.on_odom(10, 10.2, 0, 0, 0, 0, 0))
        self.assertEqual(state.odom, original)
        self.assertTrue(state.on_odom(10.1, 10.2, 0, 0, 0, 0, 0))
        self.assertEqual(state.odom[:2], (10.1, 10.2))

    def test_stop_requires_three_accepted_samples_strictly_after_stage_start(self):
        odom = (10.3, 20, 0, 0, 0, 0, 0)
        self.assertTrue(stopped_after(odom, (10.3, 20), 20.1, 10, [10.1, 10.2, 10.3]))
        for stamps in ([10.2, 10.3], [9.9, 10, 10.3], [9.8, 9.9, 10, 10.1, 10.2]):
            with self.subTest(stamps=stamps):
                self.assertFalse(stopped_after(odom, (10.3, 20), 20.1, 10, stamps))
        self.assertFalse(stopped_after(odom, (10.3, 20), 20.1, 10.3, [10.1, 10.2, 10.3]))

    def test_stop_rejects_stale_clock_odometry_and_motion(self):
        odom, clock, stamps = (10.3, 20, 0, 0, 0, 0, 0), (10.3, 20), [10.1, 10.2, 10.3]
        for sample, tick, wall in ((odom, clock, 21.1),
                                  (odom, (10.3, 19), 20.1),
                                  (odom, (10.9, 20), 20.1),
                                  (odom, (10.1, 20), 20.1),
                                  ((*odom[:5], .006, 0), clock, 20.1),
                                  ((*odom[:6], -.021), clock, 20.1)):
            with self.subTest(sample=sample, tick=tick, wall=wall):
                self.assertFalse(stopped_after(sample, tick, wall, 10, stamps))

    def test_slow_translation_drift_cannot_bypass_total_displacement_limit(self):
        state = ready()
        state.start(10)
        with self.assertRaisesRegex(SurveyAbort, 'Translation'):
            for i in range(1, 10):
                state.on_odom(10+i*.1, 10+i*.2, i*.004, 0, 0, .04, 0)

    def test_odometry_accumulates_full_turn_and_overlap_across_wrap(self):
        state = ready()
        state.start(10)
        yaw = 0.0
        for i in range(1, 321):
            yaw += .02
            state.on_clock(10+i*.05, 10+i*.1)
            state.on_odom(10+i*.05, 10+i*.1, 0, 0, angle_delta(yaw, 0), 0, .4)
        self.assertAlmostEqual(state.yaw_total, 6.4)
        self.assertGreater(state.max_yaw, 2*math.pi+.1)
        with self.assertRaisesRegex(SurveyAbort, 'angular speed'):
            state.on_odom(26.05, 42.1, 0, 0, 0, 0, .46)

    def test_action_status_excludes_only_owned_spin_uuids(self):
        own, other = b'1'*16, b'2'*16
        self.assertFalse(foreign_active([(own, 2)], {own}, '/spin/_action/status'))
        self.assertTrue(foreign_active([(own, 2)], {own}, '/follow_path/_action/status'))
        self.assertTrue(foreign_active([(own, 2), (other, 1)], {own}, '/spin/_action/status'))
        self.assertFalse(foreign_active([(other, 4)], {own}, '/spin/_action/status'))

    def test_late_goal_acceptance_preserves_cancel_intent_and_checks_uuid_ack(self):
        from concurrent.futures import Future
        goal_id = b'1'*16
        send, result, cancel = Future(), Future(), Future()
        handle = SimpleNamespace(accepted=True, get_result_async=lambda: result,
                                 cancel_goal_async=lambda: cancel)
        owned = OwnedSpin(goal_id, send, 6.4)
        owned.cancel_requested = True
        owned.poll()
        self.assertIsNone(owned.cancel_future)
        send.set_result(handle)
        owned.poll()
        self.assertIs(owned.cancel_future, cancel)
        cancel.set_result(SimpleNamespace(return_code=0, goals_canceling=[
            SimpleNamespace(goal_id=SimpleNamespace(uuid=list(goal_id)))]))
        result.set_result(SimpleNamespace(status=5, result=SimpleNamespace(error_code=0)))
        owned.poll()
        self.assertTrue(owned.cancel_ack)
        self.assertEqual(owned.terminal, 5)

    def test_result_future_error_cannot_block_cancel_or_ack_and_is_recorded(self):
        from concurrent.futures import Future
        goal_id, cancel_calls = b'1'*16, []
        send, result, cancel, recovered = Future(), Future(), Future(), Future()
        result.set_exception(RuntimeError('injected result failure'))
        cancel.set_result(SimpleNamespace(return_code=0, goals_canceling=[
            SimpleNamespace(goal_id=SimpleNamespace(uuid=list(goal_id)))]))
        def request_cancel():
            cancel_calls.append(True)
            return cancel
        handle = SimpleNamespace(accepted=True, get_result_async=lambda: result,
                                 cancel_goal_async=request_cancel)
        send.set_result(handle)
        owned = OwnedSpin(goal_id, send, 6.4)
        owned.cancel_requested = True
        with self.assertRaisesRegex(SurveyAbort, 'read result.*injected result failure'):
            owned.poll()
        self.assertEqual(cancel_calls, [True])
        self.assertTrue(owned.cancel_ack)
        self.assertIsNone(owned.terminal)
        self.assertIn('injected result failure', owned.record()['errors'][0])
        # Cleanup can recover terminal evidence, but cannot erase the failed poll.
        recovered.set_result(SimpleNamespace(status=5, result=SimpleNamespace(error_code=0)))
        handle.get_result_async = lambda: recovered
        owned.poll()
        self.assertEqual(owned.terminal, 5)
        self.assertEqual(cancel_calls, [True])
        self.assertIn('injected result failure', owned.record()['errors'][0])

    def test_get_result_request_error_cannot_block_cancellation(self):
        from concurrent.futures import Future
        goal_id, cancel_calls = b'2'*16, []
        send, cancel = Future(), Future()
        def request_result():
            raise RuntimeError('injected get_result_async failure')
        def request_cancel():
            cancel_calls.append(True)
            return cancel
        send.set_result(SimpleNamespace(accepted=True, get_result_async=request_result,
                                        cancel_goal_async=request_cancel))
        owned = OwnedSpin(goal_id, send, 6.4)
        owned.cancel_requested = True
        with self.assertRaisesRegex(SurveyAbort, 'request result.*get_result_async failure'):
            owned.poll()
        self.assertEqual(cancel_calls, [True])
        cancel.set_result(SimpleNamespace(return_code=0, goals_canceling=[
            SimpleNamespace(goal_id=SimpleNamespace(uuid=list(goal_id)))]))
        with self.assertRaises(SurveyAbort):
            owned.poll()
        self.assertTrue(owned.cancel_ack)
        self.assertEqual(cancel_calls, [True])
        self.assertEqual(len(owned.record()['errors']), 1)


if __name__ == '__main__':
    unittest.main()
