"""Opt-in real DDS RGB-D -> accepted Ledger -> owned raw retention regression.

With Jazzy and workspace dependencies sourced, in an otherwise empty domain:
  RUN_FRAME_RETENTION_ROS_TEST=1 ROS_DOMAIN_ID=74 \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST /usr/bin/python3 -B -m unittest \
    discover -s tests -p test_frame_retention_ros.py -v

FRAME_RETENTION_ROS_OUTPUT may name a NEW output directory. Otherwise a
small temporary directory is retained and printed, including on failure. Three
non-red blank images have distinct blue/depth bytes to expose substitution.
Each image is published once; no replacement frames or retries are allowed.
Actual RedRGBDBaseline callbacks receive Image/CameraInfo through DDS; their
metadata reaches the production Consumer/InspectionLedger through DDS, as do
retention requests/results. The real owned worker writes the original bytes.

Uses ROS system time, synthetic exact-time TF and a supplied static pose. This
tests retention, not MissionNode's arrival/odom adapter, physical stationarity,
Gazebo, navigation, scene coverage, recall, or mission success. No Twist or
action client is created. Default discovery skips without starting ROS.
"""
import hashlib
import json
from math import atan2
import os
from pathlib import Path
import struct
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOTS = [ROOT / 'src' / name for name in
                 ('limo_evidence_protocol', 'limo_maze_mission', 'limo_maze_perception')]
for _path in reversed(PACKAGE_ROOTS):
    sys.path.insert(0, str(_path))
sys.path.insert(0, str(ROOT / 'tools'))
try:
    import rclpy
except ImportError:
    rclpy = None

ENABLED = (os.environ.get('RUN_FRAME_RETENTION_ROS_TEST') == '1' and
           os.environ.get('ROS_DOMAIN_ID') == '74' and
           os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') == 'LOCALHOST' and
           rclpy is not None)
N = 1_000_000_000


@unittest.skipUnless(ENABLED, 'Set RUN_FRAME_RETENTION_ROS_TEST=1 with Jazzy, domain 74 / LOCALHOST')
class FrameRetentionDDSTests(unittest.TestCase):
    def setUp(self):
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        self.context = self.executor = self.probe = self.source = self.listener = None
        self.output = self.consumer = self.ledger = self.shutdown_report = None
        self.records, self.decisions, self.requests, self.results, self.inputs = [], [], [], [], []
        self.callback_errors = []
        self.closed_source = False
        self.old_pythonpath = os.environ.get('PYTHONPATH')
        self.addCleanup(self.cleanup_ros)
        # The real child inherits the same explicit checkout modules as this
        # test, rather than accidentally importing a stale installed protocol.
        os.environ['PYTHONPATH'] = os.pathsep.join(
            [str(p) for p in PACKAGE_ROOTS] + ([self.old_pythonpath] if self.old_pythonpath else []))
        destination = os.environ.get('FRAME_RETENTION_ROS_OUTPUT')
        if destination:
            candidate = Path(destination).absolute()
            candidate.mkdir(parents=True, exist_ok=False)
            self.output = candidate.resolve()
        else:
            self.output = Path(tempfile.mkdtemp(prefix='limo-retention-dds-')).resolve()
        self.context = Context()
        rclpy.init(context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.probe = rclpy.create_node('frame_retention_test_probe', context=self.context,
                                       enable_rosout=False)
        self.executor.add_node(self.probe)
        self.wait(lambda: self.probe.get_node_names() == [self.probe.get_name()],
                  'domain 74 must contain only this probe before setup', timeout=3.)

    def save(self, name, value):
        if self.output is not None:
            (self.output / name).write_text(json.dumps(value, indent=2, allow_nan=False), encoding='utf-8')

    def spin(self, seconds=.02):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=min(.01, max(0., end-time.monotonic())))

    def diagnostic(self):
        return {'records': self.records, 'decisions': self.decisions,
                'request_count': len(self.requests), 'result_count': len(self.results),
                'callback_errors': self.callback_errors,
                'ledger': self.ledger.snapshot() if self.ledger else None,
                'consumer': self.consumer.snapshot() if self.consumer else None,
                'producer': self.source.retention.snapshot() if self.source else None}

    def wait(self, condition, label, timeout=3.):
        deadline = time.monotonic() + timeout
        while not condition() and not self.callback_errors and time.monotonic() < deadline:
            self.spin(.005)
        if self.callback_errors or not condition():
            snapshot = self.diagnostic()
            self.save('failure.json', snapshot)
            self.fail(label + ': ' + json.dumps(snapshot, allow_nan=False))

    def close_source(self):
        if self.source is None:
            return
        source = self.source
        self.executor.remove_node(source)
        source.destroy_node()  # Real bounded shutdown flushes and reaps its child.
        self.shutdown_report = source.retention_shutdown
        self.closed_source = True
        self.source = None
        self.save('producer-shutdown.json', self.shutdown_report)
        self.assertIsNotNone(self.shutdown_report)
        self.assertTrue(self.shutdown_report['shutdown']['owned_writer_exit_confirmed'], self.shutdown_report)
        self.assertTrue(self.shutdown_report['shutdown']['final_checkpoint_persisted'], self.shutdown_report)
        self.assertIsNone(self.shutdown_report['writer'])
        self.assertIsNone(self.shutdown_report['fault'])

    def cleanup_ros(self):
        errors = []
        report = {'source_destroyed': self.closed_source, 'executor_shutdown': False,
                  'context_shutdown': False, 'errors': errors}
        # Preserve partial evidence on failure too; this never fills a missing
        # frame or upgrades a partial run to passed.
        try:
            self.save('input-identities.json', self.inputs)
            self.save('dds-metadata.json', {'records': self.records, 'requests': self.requests,
                                           'results': self.results, 'decisions': self.decisions})
            if self.consumer is not None:
                self.save('consumer.json', self.consumer.snapshot())
            if self.ledger is not None:
                self.save('ledger.json', self.ledger.snapshot())
        except Exception as exc:
            errors.append('partial_evidence:' + repr(exc))
        try:
            self.close_source()
        except Exception as exc:
            errors.append('source:' + repr(exc))
        report['source_destroyed'] = self.closed_source
        if self.listener is not None:
            try:
                self.listener.unregister()
            except Exception as exc:
                errors.append('tf_listener:' + repr(exc))
        if self.probe is not None:
            try:
                self.executor.remove_node(self.probe)
                self.probe.destroy_node()
            except Exception as exc:
                errors.append('probe:' + repr(exc))
        if self.executor is not None:
            try:
                report['executor_shutdown'] = self.executor.shutdown(timeout_sec=2.)
            except Exception as exc:
                errors.append('executor:' + repr(exc))
        if self.context is not None:
            try:
                if self.context.ok():
                    self.context.shutdown()
                report['context_shutdown'] = not self.context.ok()
            except Exception as exc:
                errors.append('context:' + repr(exc))
        if self.old_pythonpath is None:
            os.environ.pop('PYTHONPATH', None)
        else:
            os.environ['PYTHONPATH'] = self.old_pythonpath
        report['owned_writer_exit_confirmed'] = bool(self.shutdown_report and
            self.shutdown_report['shutdown']['owned_writer_exit_confirmed'])
        try:
            self.save('cleanup.json', report)
        except Exception as exc:
            errors.append('cleanup_report:' + repr(exc))
        if self.output is not None:
            print('FRAME_RETENTION_DDS_ARTIFACTS ' + str(self.output))
        self.assertEqual(errors, [], 'Cleanup must preserve and report all owned resources')
        if self.context is not None:
            self.assertTrue(report['context_shutdown'])
            self.assertTrue(report['executor_shutdown'])

    def setup_chain(self):
        from geometry_msgs.msg import TransformStamped
        from rclpy.parameter import Parameter
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
        from rclpy.time import Time
        from sensor_msgs.msg import CameraInfo, Image
        from std_msgs.msg import String
        from tf2_ros import Buffer, TransformBroadcaster, TransformListener
        from limo_maze_perception import node as source_module
        from limo_maze_perception import retention_adapter, retention_worker
        from limo_maze_mission import retention as consumer_module, inspection
        from limo_evidence_protocol import retention as protocol
        import audit_frame_retention

        modules = {'source': source_module, 'adapter': retention_adapter, 'worker': retention_worker,
                   'consumer': consumer_module, 'inspection': inspection, 'protocol': protocol,
                   'audit': audit_frame_retention}
        expected = {'source': 'src/limo_maze_perception/limo_maze_perception/node.py',
                    'adapter': 'src/limo_maze_perception/limo_maze_perception/retention_adapter.py',
                    'worker': 'src/limo_maze_perception/limo_maze_perception/retention_worker.py',
                    'consumer': 'src/limo_maze_mission/limo_maze_mission/retention.py',
                    'inspection': 'src/limo_maze_mission/limo_maze_mission/inspection.py',
                    'protocol': 'src/limo_evidence_protocol/limo_evidence_protocol/retention.py',
                    'audit': 'tools/audit_frame_retention.py'}
        sources = {}
        for name, module in modules.items():
            path = Path(module.__file__).resolve()
            self.assertEqual(path, ROOT / expected[name], 'Test must use this checkout')
            sources[name] = {'path': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
        sources['fixture'] = {'path': str(Path(__file__).resolve()),
                              'sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
        self.save('sources.json', sources)
        self.audit = audit_frame_retention.audit
        self.Time, self.Image, self.CameraInfo = Time, Image, CameraInfo
        self.String, self.TransformStamped = String, TransformStamped
        self.epoch, self.token = 'retention-dds-epoch', 1
        prefix = '/retention_test'
        topics = {name: prefix + '/' + name for name in ('rgb', 'depth', 'info', 'evidence', 'requests', 'results')}
        params = {'enabled': True, 'depth_registered_to_color': True, 'use_sim_time': False,
                  'publish_frame_processing': True, 'retention_enabled': True,
                  'retention_output_dir': str(self.output / 'retained'),
                  'color_topic': topics['rgb'], 'depth_topic': topics['depth'],
                  'camera_info_topic': topics['info'], 'frame_processing_topic': topics['evidence'],
                  'retention_request_topic': topics['requests'], 'retention_result_topic': topics['results'],
                  'output_topic': prefix + '/targets'}
        self.source = source_module.RedRGBDBaseline(context=self.context,
            parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
        self.executor.add_node(self.source)
        # Warm only decoding/algorithm imports before opening the timed Ledger.
        # No callback, metadata, cached sample or DDS image is made by warm-up.
        warm_color, warm_depth = Image(), Image()
        for message, encoding, step, raw in (
                (warm_color, 'rgb8', 240, bytes(80*60*3)),
                (warm_depth, '32FC1', 320, struct.pack('<f', 2.) * (80*60))):
            message.width, message.height = 80, 60
            message.encoding, message.step, message.data = encoding, step, raw
        source_module.process_red_frame(
            self.source.bridge.imgmsg_to_cv2(warm_color, desired_encoding='passthrough'),
            self.source.bridge.imgmsg_to_cv2(warm_depth, desired_encoding='passthrough'),
            source_module.Intrinsics(80, 60, 100., 100., 39.5, 29.5), '32FC1')
        reliable = QoSProfile(depth=128, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.rgb_pub = self.probe.create_publisher(Image, topics['rgb'], qos_profile_sensor_data)
        self.depth_pub = self.probe.create_publisher(Image, topics['depth'], qos_profile_sensor_data)
        self.info_pub = self.probe.create_publisher(CameraInfo, topics['info'], qos_profile_sensor_data)
        self.request_pub = self.probe.create_publisher(String, topics['requests'], reliable)
        self.epoch_pub = self.probe.create_publisher(String, '/mission/map_epoch', latched)
        self.evidence_sub = self.probe.create_subscription(String, topics['evidence'], self.on_evidence, 10)
        self.result_sub = self.probe.create_subscription(String, topics['results'], self.on_result, reliable)
        self.tf_buffer = Buffer(node=self.probe)
        self.listener = TransformListener(self.tf_buffer, self.probe, spin_thread=False)
        self.tf_pub = TransformBroadcaster(self.probe)
        self.ledger = inspection.InspectionLedger()
        self.consumer = consumer_module.Consumer('retention-dds-consumer', 3, self.publish_request,
                                                  consumer_module.Policy(enabled=True))
        publishers = (self.rgb_pub, self.depth_pub, self.info_pub, self.request_pub, self.epoch_pub)
        self.wait(lambda: all(p.get_subscription_count() == 1 for p in publishers) and
                  self.source.frame_publisher.get_subscription_count() == 1 and
                  self.source.retention_pub.get_subscription_count() == 1 and
                  self.probe.count_subscribers('/tf') == 1,
                  'all DDS endpoints discovered', timeout=4.)
        self.epoch_pub.publish(String(data=self.epoch))
        self.wait(lambda: self.source.frame_evidence.epoch_ready and self.source.retention.core is not None,
                  'producer epoch received through DDS')
        barrier = self.probe.get_clock().now().nanoseconds
        self.ledger.set_epoch(self.epoch, barrier)
        now = time.monotonic()
        self.ledger.begin(barrier, (1., 1., 0.), now, timeout_seconds=15., request_id='retention-dds-window')
        self.assertTrue(self.consumer.begin_window(self.ledger, self.token, now))

    def publish_request(self, raw):
        self.requests.append(json.loads(raw))
        self.request_pub.publish(self.String(data=raw))

    def on_result(self, message):
        try:
            result = json.loads(message.data)
            self.results.append(result)
            self.consumer.on_result(result, time.monotonic())
        except Exception as exc:
            self.callback_errors.append('result:' + repr(exc))

    def on_evidence(self, message):
        try:
            record = json.loads(message.data)
            self.records.append(record)
            stamp = record['rgb']['stamp_ns']
            at = self.Time(nanoseconds=stamp)
            base = self.tf_buffer.lookup_transform('map', 'base_link', at)
            camera = self.tf_buffer.lookup_transform('map', record['rgb']['frame_id'], at)
            self.assertEqual(base.header.stamp.sec*N + base.header.stamp.nanosec, stamp)
            self.assertEqual(camera.header.stamp.sec*N + camera.header.stamp.nanosec, stamp)
            bp, bq = base.transform.translation, base.transform.rotation
            cp, cq = camera.transform.translation, camera.transform.rotation
            decision = self.consumer.observe(self.ledger, record,
                now_ns=self.probe.get_clock().now().nanoseconds, wall_now=time.monotonic(),
                base_pose=(bp.x, bp.y, atan2(2*(bq.w*bq.z+bq.x*bq.y), 1-2*(bq.y*bq.y+bq.z*bq.z))),
                camera_pose=(cp.x, cp.y, cp.z, cq.x, cq.y, cq.z, cq.w),
                tf_stamp_ns=stamp, goal_token=self.token)
            self.decisions.append(decision)
        except Exception as exc:
            self.callback_errors.append('evidence:' + repr(exc))

    def publish_blank_pair(self, index):
        stamp = self.probe.get_clock().now().nanoseconds
        messages = []
        for child, xyz in (('base_link', (1., 1., 0.)), ('camera_optical_frame', (1.08, 1., .25))):
            tf = self.TransformStamped()
            tf.header.frame_id, tf.child_frame_id = 'map', child
            tf.header.stamp.sec, tf.header.stamp.nanosec = divmod(stamp, N)
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = xyz
            tf.transform.rotation.w = 1.
            messages.append(tf)
        self.tf_pub.sendTransform(messages)
        self.wait(lambda: all(self.tf_buffer.can_transform('map', child, self.Time(nanoseconds=stamp))
                              for child in ('base_link', 'camera_optical_frame')),
                  'exact sample TF consumed before image publication')
        info = self.CameraInfo()
        info.header.frame_id = 'camera_optical_frame'
        info.header.stamp.sec, info.header.stamp.nanosec = divmod(stamp, N)
        info.width, info.height, info.distortion_model = 80, 60, 'plumb_bob'
        info.k = [100., 0., 39.5, 0., 100., 29.5, 0., 0., 1.]
        info.d = [0.] * 5
        info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
        info.p = [100., 0., 39.5, 0., 0., 100., 29.5, 0., 0., 0., 1., 0.]
        self.info_pub.publish(info)
        self.wait(lambda: self.source.info is not None and
                  self.source.info.header.stamp.sec*N + self.source.info.header.stamp.nanosec == stamp,
                  'same-frame CameraInfo consumed before image publication')
        raw_rgb = bytes((0, 0, index + 1)) * (80*60)
        raw_depth = struct.pack('<f', 2. + index*.1) * (80*60)
        color, depth = self.Image(), self.Image()
        for message, encoding, step, raw in ((color, 'rgb8', 240, raw_rgb), (depth, '32FC1', 320, raw_depth)):
            message.header = info.header
            message.width, message.height = 80, 60
            message.encoding, message.step, message.is_bigendian = encoding, step, 0
            message.data = raw
        self.inputs.append({'stamp_ns': stamp,
                            'rgb_sha256': hashlib.sha256(raw_rgb).hexdigest(),
                            'depth_sha256': hashlib.sha256(raw_depth).hexdigest()})
        # Single publish per stream. The synchronizer invokes production on_pair.
        self.rgb_pub.publish(color)
        self.depth_pub.publish(depth)
        self.wait(lambda: len(self.decisions) == index+1, 'fixed input pair processed through DDS')
        self.assertEqual(len(self.records), index+1)
        self.assertTrue(self.decisions[index]['accepted'], self.diagnostic())
        self.assertEqual(self.records[index]['processing']['detection_count'], 0)
        self.assertEqual(self.records[index]['processing']['empty_reason'], 'no_red_candidates')

    def test_exact_three_blank_frames_survive_dds_worker_shutdown_and_offline_audit(self):
        self.setup_chain()
        for index in range(3):
            if index:
                self.spin(.12)  # Fixed timing gives the default Ledger its >=0.2 s sample span.
            self.publish_blank_pair(index)
        self.assertEqual(self.ledger.accepted_frames, 3)
        self.assertEqual(self.ledger.status, 'satisfied')
        self.assertEqual(len(self.requests), 3)
        self.wait(lambda: self.consumer.current_satisfied(self.ledger, self.token, time.monotonic()),
                  'all three DDS receipts must be durable before completion', timeout=10.)
        self.spin(.1)
        self.assertEqual(len(self.records), 3)
        self.assertEqual(self.ledger.accepted_frames, 3)
        snapshot = self.consumer.snapshot()
        accepted = snapshot['windows'][0]['accepted']
        self.assertEqual([e['request']['accepted_ordinal'] for e in accepted], [1, 2, 3])
        self.assertEqual(len({r['sample']['rgb_sha256'] for r in self.requests}), 3)
        self.assertEqual(len({r['sample']['depth_sha256'] for r in self.requests}), 3)
        for original, record, entry in zip(self.inputs, self.records, accepted):
            req = entry['request']
            self.assertEqual(req['sample']['rgb_stamp_ns'], original['stamp_ns'])
            self.assertEqual(req['sample']['depth_stamp_ns'], original['stamp_ns'])
            self.assertEqual(req['tf']['stamp_ns'], original['stamp_ns'])
            self.assertEqual(req['tf']['base_pose'], [1., 1., 0.])
            self.assertEqual(req['tf']['camera_pose'], [1.08, 1., .25, 0., 0., 0., 1.])
            self.assertEqual(req['sample']['rgb_sha256'], original['rgb_sha256'])
            self.assertEqual(req['sample']['depth_sha256'], original['depth_sha256'])
            self.assertEqual(req['sample']['detector_session_id'], record['detector_session_id'])
            self.assertEqual(req['sample']['calibration_id'], record['camera_info']['calibration_id'])
            self.assertEqual(req['inspection_request_id'], self.ledger.request_id)
            self.assertEqual(req['goal_token'], self.token)
            self.assertTrue(entry['result']['checkpoint_persisted'])
        self.save('consumer.json', snapshot)
        self.save('input-identities.json', self.inputs)
        self.save('dds-metadata.json', {'records': self.records, 'requests': self.requests,
                                       'results': self.results, 'decisions': self.decisions})
        self.save('ledger.json', self.ledger.snapshot())
        self.close_source()  # Audit only after the source and all owned writers are closed.
        result = self.audit(snapshot, self.output / 'retained')
        self.save('audit.json', result)
        self.assertTrue(result['passed'], result)
        self.assertEqual(result['accepted_count'], 3)
        self.assertEqual(result['verified_count'], 3)
        self.assertEqual(result['errors'], [])


if __name__ == '__main__':
    unittest.main()
