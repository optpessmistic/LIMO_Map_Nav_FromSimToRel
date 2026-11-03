"""Storage evidence for retired windows must never restore observation credit."""
from copy import deepcopy
import json
import unittest

import test_frame_retention_consumer as fixtures


class RetiredReceiptTests(unittest.TestCase):
    def setUp(self):
        self.helper = fixtures.ConsumerTests()

    def retired(self, new_window=True):
        consumer, ledger, _ = self.helper.make()
        self.helper.three(consumer, ledger)
        results = [self.helper.result(consumer, i) for i in range(3)]
        old = consumer.current
        consumer.retire('inspection_window_reset', 100.4)
        if new_window:
            ledger.reset_window(10*fixtures.N + 600_000_000)
            self.assertTrue(consumer.begin_window(ledger, 1, 100.6))
        return consumer, ledger, old, results

    def test_before_deadline_stored_receipts_preserve_retired_evidence_only(self):
        consumer, ledger, old, results = self.retired()
        binding, deadline, closed = deepcopy(old['binding']), old['deadline_wall'], old['closed_wall']
        for result in results:
            decision = consumer.on_result(result, 100.8)
            self.assertTrue(decision['matched'])
            self.assertFalse(decision['credited'])
            self.assertEqual(consumer.slots[result['retention_id']]['state'], 'stored')
            self.assertTrue(decision['retired_storage_confirmed'])
        self.assertEqual((old['state'], old['binding'], old['deadline_wall'], old['closed_wall']),
                         ('retired', binding, deadline, closed))
        self.assertEqual(old['late_results'], 3)
        saved = consumer.snapshot()['windows'][0]['accepted']
        self.assertEqual([item['result'] for item in saved], results)
        self.assertEqual([item['state'] for item in saved], ['stored']*3)
        self.assertEqual([item['stored_receipt_wall'] for item in saved], [100.8]*3)
        self.assertEqual((consumer.current['state'], ledger.accepted_frames, consumer.current['ids']),
                         ('collecting', 0, []))
        self.assertFalse(consumer.current_satisfied(ledger, 1, 100.8))
        self.assertIsNone(consumer.fault)

    def test_retired_current_window_cannot_be_reopened_by_receipts(self):
        consumer, ledger, old, results = self.retired(new_window=False)
        for result in results:
            self.assertFalse(consumer.on_result(result, 100.5)['credited'])
        self.assertEqual(old['state'], 'retired')
        self.assertEqual(ledger.status, 'satisfied')
        self.assertFalse(consumer.current_satisfied(ledger, 1, 100.6))

    def test_original_deadline_is_not_extended_by_a_later_window(self):
        for receipt_wall in (108., 108.1):
            with self.subTest(receipt_wall=receipt_wall):
                consumer, ledger, old, results = self.retired(new_window=False)
                ledger.begin(11*fixtures.N, fixtures.BASE, 107., 8., 'frontier-2')
                self.assertTrue(consumer.begin_window(ledger, 2, 107.))
                decision = consumer.on_result(results[0], receipt_wall)
                self.assertFalse(decision['credited'])
                self.assertEqual(consumer.slots[old['ids'][0]]['state'], 'pending')
                self.assertIsNone(consumer.slots[old['ids'][0]]['result'])
                self.assertEqual(old['state'], 'retired')
                self.assertIsNone(consumer.fault)

    def test_failed_window_and_consumer_fault_are_never_repaired(self):
        consumer, ledger, old, results = self.retired(new_window=False)
        consumer.invalidate('map_epoch_changed')
        self.assertEqual(old['state'], 'failed')
        self.assertFalse(consumer.on_result(results[0], 100.5)['credited'])
        self.assertEqual(consumer.slots[old['ids'][0]]['state'], 'pending')
        self.assertEqual(consumer.fault, 'map_epoch_changed')
        self.assertFalse(consumer.current_satisfied(ledger, 1, 100.6))

    def test_pending_and_unpersisted_receipts_do_not_confirm_storage(self):
        for changes in ({'status': 'queued'}, {'status': 'checkpoint_pending'},
                        {'checkpoint_persisted': False}):
            with self.subTest(changes=changes):
                consumer, _, old, results = self.retired()
                result = dict(results[0], **changes)
                decision = consumer.on_result(result, 100.8)
                self.assertFalse(decision['credited'])
                self.assertFalse(decision.get('retired_storage_confirmed', False))
                self.assertEqual(consumer.slots[old['ids'][0]]['state'], 'pending')

    def test_duplicates_preserve_first_receipt_time_and_conflicts_fail_old_window(self):
        consumer, _, old, results = self.retired()
        consumer.on_result(results[0], 100.8)
        self.assertFalse(consumer.on_result(results[0], 100.9)['credited'])
        slot = consumer.slots[old['ids'][0]]
        self.assertEqual(slot['stored_receipt_wall'], 100.8)
        conflict = deepcopy(results[0])
        conflict['receipt']['files']['rgb.bin']['sha256'] = 'c'*64
        consumer.on_result(conflict, 101.)
        self.assertEqual(old['state'], 'failed')
        self.assertEqual(json.loads(slot['result']), results[0])
        self.assertEqual(consumer.current['state'], 'collecting')
        self.assertIsNone(consumer.fault)

    def test_wrong_binding_and_invalid_stored_receipts_are_rejected(self):
        for changes in ({'request_sha256': 'b'*64}, {'writer_owned': True}, {'receipt': None}):
            with self.subTest(changes=changes):
                consumer, _, old, results = self.retired()
                consumer.on_result(dict(results[0], **changes), 100.8)
                self.assertEqual(old['state'], 'failed')
                self.assertEqual(consumer.slots[old['ids'][0]]['state'], 'pending')
                self.assertEqual(consumer.current['state'], 'collecting')


if __name__ == '__main__':
    unittest.main()
