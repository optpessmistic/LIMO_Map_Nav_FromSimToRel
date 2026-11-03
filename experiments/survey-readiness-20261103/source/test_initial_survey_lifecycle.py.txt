"""Readiness must recover from a missing response without accepting stale state."""
from concurrent.futures import Future
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from initial_survey import LifecycleReadiness, SurveyAbort


def response(state_id):
    return SimpleNamespace(current_state=SimpleNamespace(id=state_id))


class Client:
    def __init__(self):
        self.ready, self.calls, self.removed = True, [], []
    def service_is_ready(self):
        return self.ready
    def call_async(self, request):
        future = Future()
        self.calls.append(future)
        return future
    def remove_pending_request(self, future):
        self.removed.append(future)


class LifecycleReadinessTests(unittest.TestCase):
    def setUp(self):
        self.client = Client()
        self.gate = LifecycleReadiness({'behavior': self.client}, object)

    def test_missing_reply_retries_and_requires_new_active_response(self):
        self.gate.poll(10)
        with self.assertRaises(SurveyAbort):
            self.gate.require_active(10)
        self.gate.poll(10.99)
        self.assertEqual(len(self.client.calls), 1)
        self.gate.poll(11)
        self.assertEqual(len(self.client.calls), 2)
        self.assertTrue(self.client.calls[0].cancelled())
        self.assertEqual(self.client.removed, self.client.calls[:1])
        self.client.calls[1].set_result(response(3))
        self.gate.poll(11.25)
        self.gate.require_active(11.25)
        detail = self.gate.report(11.25)['nodes']['behavior']
        self.assertEqual((detail['timeouts'], detail['responses']), (1, 1))

    def test_completed_but_expired_response_cannot_appear_fresh(self):
        self.gate.poll(10)
        self.client.calls[0].set_result(response(3))
        self.gate.poll(11.1)
        with self.assertRaises(SurveyAbort):
            self.gate.require_active(11.1)
        self.assertEqual(self.gate.report(11.1)['nodes']['behavior']['responses'], 0)

    def test_freshness_is_bounded_by_request_time_not_late_consumption(self):
        self.gate.poll(10)
        self.client.calls[0].set_result(response(3))
        self.gate.poll(10.9)
        self.gate.require_active(11.99)
        with self.assertRaisesRegex(SurveyAbort, 'stale'):
            self.gate.require_active(12.01)

    def test_inactive_unavailable_and_failed_responses_do_not_authorize_motion(self):
        self.gate.poll(10)
        self.client.calls[0].set_result(response(2))
        self.gate.poll(10.25)
        with self.assertRaisesRegex(SurveyAbort, 'state=2'):
            self.gate.require_active(10.25)
        self.client.calls[1].set_result(response(3))
        self.gate.poll(10.5)
        self.gate.require_active(10.5)
        self.client.ready = False
        self.gate.poll(10.75)
        with self.assertRaisesRegex(SurveyAbort, 'unavailable'):
            self.gate.require_active(10.75)
        self.client.ready = True
        self.client.calls[2].set_exception(RuntimeError('service failure'))
        self.gate.poll(10.9)
        with self.assertRaisesRegex(SurveyAbort, 'no valid response'):
            self.gate.require_active(10.9)
        self.assertIn('service failure', self.gate.report(10.9)['nodes']['behavior']['last_error'])

    def test_one_pending_service_cannot_be_hidden_by_other_active_services(self):
        other = Client()
        gate = LifecycleReadiness({'behavior': self.client, 'collision': other}, object)
        gate.poll(10)
        self.client.calls[0].set_result(response(3))
        gate.poll(10.25)
        with self.assertRaisesRegex(SurveyAbort, 'collision: no valid response'):
            gate.require_active(10.25)


if __name__ == '__main__':
    unittest.main()
