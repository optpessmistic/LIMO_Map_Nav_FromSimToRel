"""Pure execution of production MissionNode methods with actual retention/Ledger.

Transport, odometry and TF are synthetic; these tests do not initialize ROS.
"""
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as S
import unittest
from unittest.mock import patch
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src/limo_maze_mission'), str(ROOT/'src/limo_evidence_protocol')]
from limo_maze_mission.retention import Consumer, Policy
import test_observation_adapter as t


class RetentionNodeTests(unittest.TestCase):
    def setUp(self):
        self.fixture=t.AdapterTests(); self.fixture.setUp()
        self.namespace=patch.dict(t.adapter.__dict__, RetentionConsumer=Consumer, RetentionPolicy=Policy, uuid=uuid)
        self.namespace.start(); self.addCleanup(self.namespace.stop)

    def mission(self,**kwargs):
        return self.fixture.mission(require_retained_inspection=True,**kwargs)

    def active(self,**kwargs):
        m=self.mission(**kwargs)
        self.fixture.start(m); self.fixture.terminal(m)
        self.assertIsNone(m.frame_retention.current)
        self.fixture.settle(m)
        self.assertEqual(m.frame_retention.current['binding']['after_stamp_ns'],m.inspection_sampling_after_ns)
        return m

    def sample(self,m,seq=1,session='detector-a'):
        t.TIME.advance(.12); self.fixture.odom(m)
        record=t.frame(t.TIME.ns-20_000_000,seq=seq,session=session)
        m.on_frame_processing(S(data=json.dumps(record)))
        return record

    def seal(self,m):
        for seq in range(1,4): self.sample(m,seq)
        self.assertEqual(m.inspection.accepted_frames,3)
        self.assertEqual(m.inspection.status,'satisfied')
        self.assertEqual(m.frame_retention.current['state'],'awaiting_receipts')

    def receipt(self,m,index=0,status='stored'):
        rid=m.frame_retention.current['ids'][index]
        fingerprint=m.frame_retention.slots[rid]['sha256']
        return dict(retention_id=rid,request_sha256=fingerprint,status=status,writer_owned=False,
                    checkpoint_persisted=True,receipt=dict(request_sha256=fingerprint,directory=fingerprint,
                    file_fsync_completed=True,files={}))

    def receipts(self,m):
        for i in range(len(m.frame_retention.current['ids'])):
            m.on_frame_retention_result(S(data=json.dumps(self.receipt(m,i))))

    def test_default_off_does_not_create_consumer_or_change_original_gate(self):
        m=self.fixture.mission()
        self.assertIsNone(m.frame_retention)
        self.fixture.start(m);self.fixture.terminal(m);self.fixture.settle(m)
        self.fixture.processed_window(m)
        self.assertNotEqual(m.phase,'inspecting')
        self.assertNotIn('frame_retention',m.inspection_snapshot())
        large=t.adapter.MissionNode(parameter_overrides={'require_processed_inspection':True,'inspection_required_frames':1000})
        self.assertEqual(large.inspection.policy.required_frames,1000)
        self.assertIsNone(large.frame_retention)

    def test_enabled_requires_processed_and_compatible_limits_at_construction(self):
        for params in ({'require_retained_inspection':True,'require_processed_inspection':False},
                       {'require_retained_inspection':1},
                       {'require_retained_inspection':True,'require_processed_inspection':True,'inspection_required_frames':33}):
            with self.assertRaises(ValueError): t.adapter.MissionNode(parameter_overrides=params)

    def test_exact_accepted_requests_publish_immediately_and_wait_all_receipts(self):
        m=self.active(); initial_sends=len(m.nav.sent)
        self.seal(m)
        messages=m.retention_pub.messages
        self.assertEqual(len(messages),3)
        requests=[json.loads(msg.data) for msg in messages]
        self.assertEqual([r['accepted_ordinal'] for r in requests],[1,2,3])
        self.assertTrue(all(r['tf']['stamp_ns']==r['sample']['rgb_stamp_ns'] for r in requests))
        self.fixture.tick(m)
        self.assertEqual(m.phase,'inspecting')
        self.assertEqual(len(m.nav.sent),initial_sends)
        self.receipts(m)
        self.assertEqual(m.phase,'inspecting')  # receipt callback cannot navigate/transition
        self.fixture.tick(m)
        self.assertNotEqual(m.phase,'inspecting')
        self.assertTrue(any(e['event']=='inspection_complete' for e in m.events))
        self.assertFalse(m.frame_retention.summary()['offline_artifacts_verified'])

    def test_missing_middle_is_not_replaced_and_wall_deadline_blocks(self):
        m=self.active();self.seal(m);deadline=m.inspection_deadline
        for i in (0,2):m.on_frame_retention_result(S(data=json.dumps(self.receipt(m,i))))
        self.sample(m,4)
        self.assertEqual((m.inspection.accepted_frames,len(m.retention_pub.messages)),(3,3))
        t.TIME.advance(deadline-t.TIME.wall+.01,ros=False)
        self.fixture.tick(m)
        self.assertIn(m.phase,('canceling','blocked'))
        self.assertEqual(m.cancel_next,'blocked')
        self.assertEqual(m.frame_retention.fault,'retention_window_deadline')
        m.on_frame_retention_result(S(data=json.dumps(self.receipt(m,1))))
        self.assertEqual(m.frame_retention.current['state'],'failed')

    def test_receipts_do_not_override_current_pose_or_stationarity(self):
        for fault in ('pose','motion'):
            with self.subTest(fault=fault):
                self.fixture.setUp()
                m=self.active();self.seal(m);self.receipts(m)
                if fault=='pose':m.synthetic_pose=(m.goal.x+.1,m.goal.y,m.goal.yaw)
                else:self.fixture.odom(m,moving=True)
                self.fixture.tick(m)
                self.assertEqual(m.phase,'settling')
                self.assertEqual(m.inspection.accepted_frames,0)
                self.assertFalse(any(e['event']=='inspection_complete' for e in m.events))

    def test_reset_preserves_prior_slots_and_late_receipt_does_not_credit_new_window(self):
        m=self.active();self.sample(m)
        old=self.receipt(m); prior=m.frame_retention.current
        t.TIME.advance(.1);self.fixture.odom(m,moving=True);self.fixture.tick(m)
        self.assertEqual(m.phase,'settling')
        self.fixture.settle(m)
        self.assertIsNot(m.frame_retention.current,prior)
        self.sample(m,2)
        m.on_frame_retention_result(S(data=json.dumps(old)))
        self.assertEqual(prior['late_results'],1)
        self.assertEqual(m.frame_retention.summary()['stored_count'],0)
        self.assertEqual(m.frame_retention.summary()['total_selected_count'],2)

    def test_producer_identity_change_restarts_actual_settling_barrier(self):
        m=self.active();self.sample(m)
        before=m.inspection_sampling_after_ns
        self.sample(m,2,session='next-detector')
        self.assertEqual(m.phase,'settling')
        self.assertIsNone(m.inspection_sampling_after_ns)
        self.fixture.settle(m)
        self.assertGreater(m.inspection_sampling_after_ns,before)
        self.sample(m,3,session='next-detector')
        self.assertEqual(m.inspection.accepted_frames,1)
        self.assertEqual(len(m.frame_retention.windows),2)

    def test_epoch_and_clock_faults_keep_fixed_evidence_and_never_reopen(self):
        for fault in ('epoch','clock'):
            with self.subTest(fault=fault):
                self.fixture.setUp();m=self.active();self.sample(m);old=self.receipt(m)
                if fault=='epoch':m.on_epoch(S(data='map-b'))
                else:m.on_inspection_clock_jump();self.fixture.tick(m)
                m.on_frame_retention_result(S(data=json.dumps(old)))
                self.assertIsNotNone(m.frame_retention.fault)
                self.assertEqual(m.cancel_next,'blocked')
                self.assertEqual(len(m.frame_retention.slots),1)

    def test_metadata_capacity_failure_preserves_already_accepted_frame(self):
        m=self.active(retention_max_total_samples=1);self.sample(m)
        self.sample(m,2)
        self.assertEqual(len(m.retention_pub.messages),1)
        self.assertEqual(len(m.frame_retention.slots),1)
        self.assertEqual(m.frame_retention.fault,'retention_sample_budget')
        self.assertEqual(m.cancel_next,'blocked')

    def test_actual_acceptance_limit_before_span_keeps_full_list_when_node_blocks(self):
        m=self.active(retention_max_samples_per_window=3)
        for seq in range(1,4):
            t.TIME.advance(.03);self.fixture.odom(m)
            m.on_frame_processing(S(data=json.dumps(t.frame(t.TIME.ns-20_000_000,seq=seq))))
        window=m.frame_retention.snapshot()['windows'][0]
        self.assertEqual((len(window['accepted']),window['ledger_accepted_count']),(3,3))
        self.assertEqual(m.frame_retention.fault,'accepted_window_limit_before_processing_satisfied')
        self.assertEqual(m.cancel_next,'blocked')

    def test_request_transport_exception_does_not_erase_real_acceptance(self):
        m=self.active()
        with patch.object(m.retention_pub,'publish',side_effect=RuntimeError('transport failed')):
            self.sample(m)
        window=m.frame_retention.snapshot()['windows'][0]
        self.assertEqual((len(window['accepted']),window['ledger_accepted_count']),(1,1))
        self.assertEqual(m.frame_retention.fault,'retention_request_publish_failed')
        self.assertEqual(m.cancel_next,'blocked')

    def test_stage_budget_retirement_preserves_original_return_authority(self):
        m=self.active(observation_stage_timeout_s=1.2);self.sample(m)
        prior=m.frame_retention.current
        while t.TIME.wall<m.observation_deadline_wall:
            t.TIME.advance(.05);self.fixture.odom(m);self.fixture.tick(m)
        self.assertEqual(m.cancel_next,'returning')
        self.assertTrue(m.inspection.ready)
        self.assertIsNone(m.frame_retention.fault)
        self.assertEqual(prior['state'],'retired')
        self.fixture.tick(m)
        self.assertEqual(m.phase,'returning')

    def test_partial_processing_timeout_can_continue_without_crediting_late_old_receipt(self):
        m=self.active(inspection_timeout_s=1.4);self.sample(m)
        old=self.receipt(m); prior=m.frame_retention.current
        while t.TIME.wall<m.inspection_deadline:
            t.TIME.advance(.05);self.fixture.odom(m)
            if t.TIME.wall<m.inspection_deadline:self.fixture.tick(m)
        self.fixture.tick(m)
        self.assertEqual(m.phase,'observation_select')
        self.assertIsNone(m.frame_retention.fault)
        self.assertEqual(len(prior['ids']),1)
        self.fixture.tick(m);self.fixture.terminal(m);self.fixture.settle(m)
        m.on_frame_retention_result(S(data=json.dumps(old)))
        self.assertEqual(m.frame_retention.summary()['selected_count'],0)
        self.assertEqual(m.frame_retention.summary()['stored_count'],0)
        self.assertEqual(prior['late_results'],1)

    def test_mission_budget_return_is_not_changed_to_retention_block(self):
        m=self.active();self.sample(m)
        m.start_return('mission_timeout_incomplete')
        self.assertEqual(m.cancel_next,'returning')
        self.assertIsNone(m.frame_retention.fault)
        self.assertFalse(m.inspection.ready)  # existing fault cancellation semantics preserved
        self.fixture.tick(m)
        self.assertEqual(m.phase,'returning')

    def test_terminal_late_failure_updates_memory_without_new_navigation_or_result_write(self):
        m=self.active();self.seal(m);self.receipts(m)
        m.phase='done';m.result_written=True
        count=len(m.nav.sent); saved=len(m.saved_results)
        m.on_frame_retention_result(S(data=json.dumps(self.receipt(m,status='failed'))))
        self.assertEqual(m.phase,'done')
        self.assertEqual(len(m.nav.sent),count)
        self.assertEqual(len(m.saved_results),saved)
        self.assertEqual(m.frame_retention.fault,'producer_reported_failed')

    def test_final_audit_contains_all_accepted_slots_status_remains_small(self):
        m=self.active();self.seal(m);self.receipts(m)
        m.publish_status()
        state=json.loads(m.status_pub.messages[-1].data)
        self.assertNotIn('accepted',state['inspection']['frame_retention'])
        with tempfile.TemporaryDirectory() as directory:
            m.cfg['output_dir']=directory
            with patch.object(t.TIME,'strftime',return_value='fixture',create=True):
                t.adapter.MissionNode.save_result(m,False)
            result=json.loads(next(Path(directory).glob('mission_*.json')).read_text())
        retained=result['frame_retention']
        self.assertEqual(len(retained['windows'][0]['accepted']),3)
        self.assertIn('offline_artifacts_not_audited',retained['scope'])


if __name__=='__main__': unittest.main()
