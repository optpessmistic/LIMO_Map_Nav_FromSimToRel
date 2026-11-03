"""Pure tests against the actual production InspectionLedger; no ROS imports.

Uses installed packages or explicit repository package roots; no outputs imports.
"""
from copy import deepcopy
import json
import unittest
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src/limo_maze_mission'), str(ROOT/'src/limo_evidence_protocol')]

from limo_maze_mission.retention import Consumer, Policy, SLOT_RESERVATION, WINDOW_RESERVATION
from limo_evidence_protocol.retention import canonical, sha
from limo_maze_mission.inspection import InspectionLedger, InspectionPolicy
from test_mission_inspection import frame, BASE, CAMERA, N


class ConsumerTests(unittest.TestCase):
    def make(self, policy=None, required=3, span=200_000_000, source='red_rgbd_sim_baseline', publish=None):
        ledger = InspectionLedger(InspectionPolicy(required_frames=required, min_span_ns=span, allowed_source=source))
        ledger.set_epoch('map-a', 10*N)
        ledger.begin(10*N, BASE, 100., 8., 'frontier-1')
        sent = []
        consumer = Consumer('mission-session-a', required, publish or sent.append, policy or Policy(enabled=True))
        consumer.begin_window(ledger, 1, 100.)
        return consumer, ledger, sent

    def accept(self, consumer, ledger, seq=1, offset=.1, **kwargs):
        record = kwargs.pop('record', frame(10*N+round(offset*N), seq))
        params = dict(now_ns=record['processed_stamp_ns']+10_000_000, wall_now=100.+offset+.03,
                      base_pose=BASE, camera_pose=CAMERA, tf_stamp_ns=record['rgb']['stamp_ns'], goal_token=1)
        params.update(kwargs)
        return consumer.observe(ledger, record, **params)

    def three(self, consumer, ledger):
        for seq, offset in enumerate((.1, .2, .3), 1):
            self.assertTrue(self.accept(consumer, ledger, seq, offset)['accepted'])

    def result(self, consumer, index=0, **kwargs):
        rid = consumer.current['ids'][index]
        digest = consumer.slots[rid]['sha256']
        result = dict(retention_id=rid, request_sha256=digest, status='stored', writer_owned=False,
                      checkpoint_persisted=True,
                      receipt=dict(request_sha256=digest, directory=digest, file_fsync_completed=True,
                                   files={'rgb.bin': {'sha256':'a'*64,'bytes':14400}}))
        result.update(kwargs)
        return result

    def complete(self, consumer, ledger, wall=100.5):
        for index in range(len(consumer.current['ids'])):
            consumer.on_result(self.result(consumer, index), wall)
        self.assertTrue(consumer.current_satisfied(ledger, 1, wall))

    def test_default_off_preserves_1000_frame_policy_and_original_observe(self):
        consumer, ledger, sent = self.make(Policy(), required=1000)
        self.assertTrue(self.accept(consumer, ledger)['accepted'])
        self.assertEqual(ledger.accepted_frames, 1)
        self.assertEqual(sent, [])
        self.assertEqual(consumer.windows, [])

    def test_enabled_rejects_incompatible_config_without_truncation(self):
        for count, policy in ((33, Policy(enabled=True)), (3, Policy(enabled=True,max_samples_per_window=2))):
            with self.assertRaisesRegex(ValueError, 'required_frames_exceeds'):
                self.make(policy, required=count)
        for kwargs in ({'enabled':1},{'max_samples_per_window':33},{'max_total_samples':4097},
                       {'max_windows':257},{'max_metadata_bytes':2**28+1}):
            with self.assertRaises(ValueError):
                Policy(**kwargs)

    def test_freezes_every_real_acceptance_tf_identity_and_ordinal(self):
        consumer, ledger, sent = self.make()
        self.three(consumer, ledger)
        self.assertEqual(ledger.accepted_frames, len(sent))
        requests = list(map(json.loads, sent))
        self.assertEqual([r['accepted_ordinal'] for r in requests], [1,2,3])
        self.assertEqual([r['sample']['sequence'] for r in requests], [1,2,3])
        for r in requests:
            self.assertEqual(r['tf']['stamp_ns'], r['sample']['rgb_stamp_ns'])
            self.assertEqual(r['tf']['base_pose'], list(BASE))
            self.assertEqual(r['tf']['camera_pose'], list(CAMERA))
            self.assertEqual((r['goal_token'],r['inspection_request_id'],r['after_stamp_ns']), (1,'frontier-1',10*N))
        self.assertFalse(consumer.current_satisfied(ledger,1,100.4))
        self.complete(consumer,ledger)

    def test_missing_middle_cannot_be_replaced_by_fourth(self):
        consumer, ledger, sent = self.make()
        self.three(consumer,ledger)
        consumer.on_result(self.result(consumer,0),100.34)
        consumer.on_result(self.result(consumer,2),100.35)
        self.assertFalse(self.accept(consumer,ledger,4,.4)['accepted'])
        unknown=self.result(consumer,1); unknown['retention_id']='fourth-not-selected'
        self.assertFalse(consumer.on_result(unknown,100.44)['matched'])
        self.assertEqual((ledger.accepted_frames,len(sent)),(3,3))
        self.assertFalse(consumer.current_satisfied(ledger,1,100.5))
        consumer.tick(108.)
        self.assertEqual(consumer.fault,'retention_window_deadline')
        consumer.on_result(self.result(consumer,1),108.1)
        self.assertEqual(consumer.current['state'],'failed')

    def test_source_is_policy_replaceable_no_detection_selection(self):
        consumer,ledger,sent=self.make(source='replacement-v2')
        for seq,offset in enumerate((.1,.2,.3),1):
            r=frame(10*N+round(offset*N),seq);r['source']='replacement-v2'
            self.assertTrue(self.accept(consumer,ledger,seq,offset,record=r)['accepted'])
        self.assertTrue(all(json.loads(r)['sample']['source']=='replacement-v2' for r in sent))
        self.complete(consumer,ledger)

    def test_request_and_results_are_detached_from_mutable_inputs(self):
        consumer,ledger,sent=self.make()
        r=frame(); base=list(BASE); camera=list(CAMERA)
        self.assertTrue(self.accept(consumer,ledger,record=r,base_pose=base,camera_pose=camera)['accepted'])
        r['rgb']['data_sha256']='b'*64; base[0]=99;camera[0]=99
        frozen=json.loads(sent[0]);self.assertEqual(frozen['tf']['base_pose'][0],BASE[0])
        self.assertEqual(frozen['sample']['rgb_sha256'],'a'*64)
        result=self.result(consumer);consumer.on_result(result,100.14)
        result['receipt']['files']['rgb.bin']['bytes']=999999
        self.assertEqual(consumer.snapshot()['windows'][0]['accepted'][0]['result']['receipt']['files']['rgb.bin']['bytes'],14400)
        snap=consumer.snapshot();snap['windows'][0]['accepted'][0]['request']['tf']['base_pose'][0]=999
        self.assertEqual(consumer.snapshot()['windows'][0]['accepted'][0]['request']['tf']['base_pose'][0],BASE[0])

    def test_only_checkpointed_stored_receipt_counts(self):
        consumer,ledger,_=self.make();self.three(consumer,ledger)
        for i in range(3):
            consumer.on_result(self.result(consumer,i,checkpoint_persisted=False),100.4)
            consumer.on_result(self.result(consumer,i,status='checkpoint_pending'),100.4)
        self.assertFalse(consumer.current_satisfied(ledger,1,100.4))
        self.complete(consumer,ledger)
        consumer.on_result(self.result(consumer,0,status='queued',checkpoint_persisted=False),100.6)
        self.assertTrue(consumer.current_satisfied(ledger,1,100.6))

    def test_failed_receipt_is_sticky_even_after_later_stored(self):
        consumer,ledger,_=self.make();self.three(consumer,ledger)
        consumer.on_result(self.result(consumer,1,status='failed',checkpoint_persisted=False),100.4)
        consumer.on_result(self.result(consumer,1),100.5)
        self.assertEqual(consumer.fault,'producer_reported_failed')
        self.assertFalse(consumer.current_satisfied(ledger,1,100.5))

    def test_same_id_wrong_request_hash_fails_exact_slot(self):
        consumer,ledger,_=self.make();self.accept(consumer,ledger)
        consumer.on_result(self.result(consumer,request_sha256='b'*64),100.2)
        self.assertEqual(consumer.fault,'retention_result_identity_conflict')
        self.assertEqual(len(consumer.slots),1)

    def test_mutated_stored_receipt_revokes_complete_window(self):
        consumer,ledger,_=self.make();self.three(consumer,ledger);self.complete(consumer,ledger)
        r=self.result(consumer);r['receipt']['files']['rgb.bin']['sha256']='b'*64
        consumer.on_result(r,100.6)
        self.assertEqual(consumer.fault,'invalid_or_conflicting_stored_receipt')
        self.assertFalse(consumer.current_satisfied(ledger,1,100.6))

    def test_invalid_receipt_and_writer_owned_never_credit(self):
        for change in ({'writer_owned':True},{'receipt':{}},{'receipt':None}):
            consumer,ledger,_=self.make();self.accept(consumer,ledger)
            consumer.on_result(self.result(consumer,**change),100.2)
            self.assertEqual(consumer.fault,'invalid_or_conflicting_stored_receipt')

    def test_unknown_and_oversized_results_have_bounded_ignored_counter(self):
        consumer,ledger,_=self.make();self.accept(consumer,ledger)
        for value in (None,[],{}, {'retention_id':'x','request_sha256':'a'*64}, {'data':'x'*20000}):
            self.assertFalse(consumer.on_result(value,100.2)['matched'])
        self.assertEqual(consumer.ignored_results,5)
        self.assertIsNone(consumer.fault)

    def test_reset_preserves_old_fixed_list_and_late_result_cannot_enter_new_window(self):
        consumer,ledger,_=self.make();self.accept(consumer,ledger)
        old_result=self.result(consumer)
        consumer.retire('stationarity_lost',100.2)
        ledger.reset_window(10*N+300_000_000)
        self.assertTrue(consumer.begin_window(ledger,1,100.3))
        self.assertTrue(self.accept(consumer,ledger,2,.4)['accepted'])
        consumer.on_result(old_result,100.5)
        self.assertEqual(consumer.windows[0]['late_results'],1)
        self.assertEqual(len(consumer.windows[0]['ids']),1)
        self.assertEqual(consumer.slots[consumer.current['ids'][0]]['state'],'pending')
        self.assertIsNone(consumer.fault)

    def test_nonempty_same_binding_cannot_be_reused_after_reset(self):
        consumer,ledger,_=self.make();self.accept(consumer,ledger)
        consumer.retire('reset',100.2);ledger.reset_window(10*N)
        self.assertFalse(consumer.begin_window(ledger,1,100.2))
        self.assertEqual(consumer.fault,'nonempty_window_binding_reused')

    def test_unannounced_reset_or_token_change_fails_before_next_acceptance(self):
        for token in (1,2):
            consumer,ledger,_=self.make();self.accept(consumer,ledger)
            if token==1: ledger.reset_window(10*N)
            self.assertFalse(self.accept(consumer,ledger,2,.2,goal_token=token)['accepted'])
            self.assertEqual(consumer.fault,'ledger_window_binding_changed')
            self.assertEqual(len(consumer.slots),1)

    def test_producer_identity_reset_is_retired_without_dropping_prior_acceptance(self):
        consumer,ledger,_=self.make();self.accept(consumer,ledger)
        record=frame(10*N+200_000_000,2,session='replacement-session')
        decision=self.accept(consumer,ledger,2,.2,record=record)
        self.assertEqual(decision['reason'],'producer_identity_changed')
        self.assertEqual((consumer.current['state'],len(consumer.slots),ledger.accepted_frames),('retired',1,0))
        self.assertTrue(consumer.begin_window(ledger,1,100.24))
        self.assertEqual(consumer.current['ids'],[])

    def test_epoch_invalidation_cannot_be_restored_by_late_result(self):
        consumer,ledger,_=self.make();self.accept(consumer,ledger)
        result=self.result(consumer);consumer.invalidate('map_epoch_changed');ledger.set_epoch('map-b',11*N)
        consumer.on_result(result,100.2)
        self.assertEqual(consumer.fault,'map_epoch_changed')
        self.assertFalse(consumer.begin_window(ledger,2,100.3))

    def test_actual_ros_clock_rewind_preserves_audit_and_fails(self):
        consumer,ledger,_=self.make();self.accept(consumer,ledger)
        decision=self.accept(consumer,ledger,2,.2,now_ns=10*N)
        self.assertFalse(decision['accepted'])
        self.assertFalse(ledger.ready)
        self.assertEqual(consumer.fault,'ledger_invalidated')
        self.assertEqual(len(consumer.slots),1)

    def test_wall_clock_rewind_fails_closed(self):
        consumer,ledger,_=self.make();self.accept(consumer,ledger)
        with self.assertRaisesRegex(ValueError,'invalid_monotonic_wall'):
            consumer.tick(99.)
        self.assertEqual(consumer.fault,'invalid_monotonic_wall')

    def test_partial_processing_timeout_remains_uncredited_and_retirable(self):
        consumer,ledger,_=self.make();self.accept(consumer,ledger)
        consumer.tick(108.)
        self.assertEqual(consumer.current['state'],'failed')
        self.assertIsNone(consumer.fault)
        self.assertFalse(consumer.current_satisfied(ledger,1,108.))
        self.assertEqual(len(consumer.current['ids']),1)

    def test_32_actual_acceptances_before_span_fail_without_33rd_or_truncation(self):
        consumer,ledger,sent=self.make(required=3,span=N)
        for seq in range(1,33):
            self.assertTrue(self.accept(consumer,ledger,seq,seq*.001)['accepted'])
        self.assertEqual((ledger.accepted_frames,len(sent),len(consumer.slots)),(32,32,32))
        self.assertEqual(consumer.fault,'accepted_window_limit_before_processing_satisfied')
        self.assertFalse(self.accept(consumer,ledger,33,.04)['accepted'])
        self.assertEqual(ledger.accepted_frames,32)

    def test_required_32_can_satisfy_without_changing_original_count(self):
        consumer,ledger,_=self.make(required=32,span=31_000_000)
        for seq in range(1,33): self.accept(consumer,ledger,seq,seq*.001)
        self.assertEqual(ledger.status,'satisfied')
        self.assertEqual(ledger.accepted_frames,32)
        self.complete(consumer,ledger)

    def test_more_than_required_actual_acceptances_all_remain_selected(self):
        consumer,ledger,sent=self.make()
        for seq,offset in enumerate((.1,.15,.2,.3),1):
            self.assertTrue(self.accept(consumer,ledger,seq,offset)['accepted'])
        self.assertEqual(ledger.accepted_frames,4)
        self.assertEqual(len(sent),4)
        self.assertEqual([json.loads(r)['accepted_ordinal'] for r in sent],[1,2,3,4])
        self.complete(consumer,ledger)

    def test_retire_after_complete_keeps_prior_result_revocable(self):
        consumer,ledger,_=self.make();self.three(consumer,ledger);self.complete(consumer,ledger)
        consumer.retire('stage_finished',100.6)
        self.assertEqual(consumer.current['state'],'complete')
        consumer.on_result(self.result(consumer,status='failed'),100.7)
        self.assertEqual(consumer.fault,'producer_reported_failed')

    def test_original_deadline_cannot_be_extended_while_collecting(self):
        consumer,ledger,_=self.make();self.accept(consumer,ledger)
        ledger.deadline_wall+=10.
        self.assertFalse(self.accept(consumer,ledger,2,.2)['accepted'])
        self.assertEqual(consumer.fault,'ledger_window_binding_changed')

    def test_historical_complete_is_not_current_authority_after_deadline(self):
        consumer,ledger,_=self.make();self.three(consumer,ledger);self.complete(consumer,ledger)
        self.assertFalse(consumer.current_satisfied(ledger,1,108.))
        self.assertEqual(consumer.current['state'],'complete')

    def test_sample_and_metadata_budget_stop_before_next_acceptance(self):
        for policy in (Policy(enabled=True,max_total_samples=1),
                       Policy(enabled=True,max_metadata_bytes=WINDOW_RESERVATION+SLOT_RESERVATION)):
            consumer,ledger,sent=self.make(policy);self.accept(consumer,ledger)
            self.assertFalse(self.accept(consumer,ledger,2,.2)['accepted'])
            self.assertEqual((ledger.accepted_frames,len(sent)),(1,1))
            self.assertEqual(consumer.fault,'retention_sample_budget')

    def test_window_budget_has_no_unbounded_empty_reset_tombstones(self):
        consumer,ledger,_=self.make(Policy(enabled=True,max_windows=2))
        consumer.retire('reset',100.1);ledger.reset_window(10*N)
        self.assertTrue(consumer.begin_window(ledger,1,100.1))
        consumer.retire('reset',100.2);ledger.reset_window(10*N)
        self.assertFalse(consumer.begin_window(ledger,1,100.2))
        self.assertEqual(len(consumer.windows),2)
        self.assertEqual(consumer.fault,'retention_window_budget')

    def test_publish_failure_keeps_the_actual_accepted_slot(self):
        def fail(_): raise RuntimeError('transport unavailable')
        consumer,ledger,_=self.make(publish=fail)
        decision=self.accept(consumer,ledger)
        self.assertTrue(decision['accepted'])
        self.assertEqual((ledger.accepted_frames,len(consumer.slots)),(1,1))
        self.assertEqual(consumer.fault,'retention_request_publish_failed')

    def test_bad_exact_tf_is_never_counted_or_published(self):
        consumer,ledger,sent=self.make()
        self.assertFalse(self.accept(consumer,ledger,tf_stamp_ns=0)['accepted'])
        self.assertEqual((ledger.accepted_frames,len(sent)),(0,0))
        self.assertEqual(consumer.fault,'retention_metadata_unrepresentable')

    def test_rejected_before_barrier_and_duplicate_do_not_allocate_or_fault(self):
        consumer,ledger,sent=self.make()
        self.assertEqual(self.accept(consumer,ledger,offset=0.)['reason'],'sample_before_barrier')
        self.assertTrue(self.accept(consumer,ledger)['accepted'])
        self.assertEqual(self.accept(consumer,ledger,seq=2,offset=.1,wall_now=100.2)['reason'],'nonadvancing_sample')
        self.assertEqual((ledger.accepted_frames,len(sent)),(1,1))
        self.assertIsNone(consumer.fault)

    def test_old_complete_window_late_failure_revokes_run_without_reopening(self):
        consumer,ledger,_=self.make();self.three(consumer,ledger);self.complete(consumer,ledger)
        old=self.result(consumer,1,status='failed',checkpoint_persisted=False)
        ledger.begin(11*N,BASE,101.,8.,'frontier-2')
        self.assertTrue(consumer.begin_window(ledger,2,101.))
        consumer.on_result(old,101.1)
        self.assertEqual(consumer.windows[0]['state'],'failed')
        self.assertEqual(consumer.current['ids'],[])
        self.assertEqual(consumer.fault,'producer_reported_failed')


if __name__=='__main__': unittest.main()
