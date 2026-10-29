"""Processing evidence contracts; optional real callbacks only in isolated domain 74."""
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace as S
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"src/limo_maze_perception"))
from limo_maze_perception.core import DetectorConfig, Intrinsics, process_red_frame
from limo_maze_perception.evidence import FrameEvidence, camera_metadata, image_metadata


def header(ns):
    return S(stamp=S(sec=ns//1_000_000_000, nanosec=ns%1_000_000_000), frame_id="camera_optical_frame")


def fixtures(ns=10_100_000_000, depth_ns=None):
    rgb = np.zeros((60, 80, 3), np.uint8)
    depth = np.full((60, 80), 2, np.float32)
    color = S(header=header(ns), width=80, height=60, encoding="rgb8", step=240, is_bigendian=0, data=rgb.tobytes())
    dep = S(header=header(ns if depth_ns is None else depth_ns), width=80, height=60,
            encoding="32FC1", step=320, is_bigendian=0, data=depth.tobytes())
    info = S(header=header(ns), width=80, height=60, distortion_model="plumb_bob", binning_x=0, binning_y=0,
             k=[100.,0.,39.5,0.,100.,29.5,0.,0.,1.], d=[0.]*5, r=[1.,0.,0.,0.,1.,0.,0.,0.,1.],
             p=[100.,0.,39.5,0.,0.,100.,29.5,0.,0.,0.,1.,0.],
             roi=S(x_offset=0,y_offset=0,height=0,width=0,do_rectify=False))
    return color, dep, info, rgb, depth


def stats(rgb, depth):
    return process_red_frame(rgb, depth, Intrinsics(80,60,100,100,39.5,29.5), "32FC1")[1]


class FrameEvidenceTests(unittest.TestCase):
    def setUp(self):
        self.tracker = FrameEvidence(DetectorConfig())
        self.tracker.on_epoch("epoch-a", 10_000_000_000)

    def record(self, inputs=None, now_ns=10_200_000_000, **flags):
        color, depth, info, rgb, array = inputs or fixtures()
        start = self.tracker.begin(color, depth, info, now_ns,
                                   enabled=flags.get("enabled",True), registered=flags.get("registered",True))
        return self.tracker.finish(start, stats(rgb,array), now_ns+1_000_000)

    def test_valid_empty_and_unlocalized_candidates_are_distinct(self):
        first = self.record()
        self.assertEqual(first["status"], "processed")
        self.assertEqual(first["processing"]["empty_reason"], "no_red_candidates")
        args = fixtures(10_300_000_000)
        args[3][20:40,30:50] = [255,0,0]
        args[4][20:40,30:50] = np.nan
        args[0].data, args[1].data = args[3].tobytes(), args[4].tobytes()
        second = self.record(args, now_ns=10_400_000_000)
        self.assertEqual(second["status"], "processed")
        self.assertEqual(second["processing"]["empty_reason"], "all_candidates_rejected")
        self.assertEqual(second["processing"]["unlocalized_candidate_count"], 1)
        for record in (first,second):
            self.assertNotIn("coverage", record)
            self.assertNotIn("inspection_complete", record)
            json.dumps(record,allow_nan=False)

    def test_all_invalid_depth_and_uncompleted_algorithm_are_not_processed(self):
        args = fixtures(); args[4][:] = np.nan; args[1].data=args[4].tobytes()
        result = self.record(args)
        self.assertEqual(result["reason"], "insufficient_frame_depth")
        self.assertEqual(result["status"], "rejected")
        self.assertIsNone(self.tracker.rgb_watermark)
        attempt = self.tracker.begin(*fixtures()[:3], 10_400_000_000,enabled=True,registered=True)
        self.assertEqual(self.tracker.finish(attempt,None,10_401_000_000)["reason"], "algorithm_not_completed")

    def test_disabled_unregistered_and_missing_epoch_cannot_process(self):
        self.assertEqual(self.record(enabled=False)["reason"], "detector_disabled")
        self.assertEqual(self.record(now_ns=10_300_000_000,registered=False)["reason"], "depth_registration_unconfirmed")
        self.tracker = FrameEvidence(DetectorConfig())
        self.assertEqual(self.record()["reason"], "map_epoch_unavailable")

    def test_both_source_stamps_must_advance_independently(self):
        self.assertEqual(self.record()["status"], "processed")
        for index,(ns,dep) in enumerate(((10_100_000_000,10_100_000_000), (10_120_000_000,10_100_000_000),
                                        (10_090_000_000,10_120_000_000))):
            with self.subTest(rgb=ns,depth=dep):
                self.assertEqual(self.record(fixtures(ns,dep),now_ns=10_400_000_000+index*20_000_000)["reason"],"nonadvancing_sample")
        self.assertEqual(self.record(fixtures(10_500_000_000),now_ns=10_600_000_000)["status"],"processed")

    def test_epoch_barrier_ignores_duplicate_announcements_and_rejects_queued_old_frames(self):
        self.record()
        self.tracker.on_epoch("epoch-a",10_400_000_000)
        self.assertEqual(self.record(fixtures(10_300_000_000),now_ns=10_500_000_000)["status"],"processed")
        self.tracker.on_epoch("epoch-b",11_000_000_000)
        self.assertEqual(self.record(fixtures(10_900_000_000),now_ns=11_100_000_000)["reason"],"sample_before_epoch_barrier")
        self.assertEqual(self.record(fixtures(11_200_000_000),now_ns=11_300_000_000)["status"],"processed")

    def test_clock_rollback_requires_a_new_epoch_not_same_epoch_reannouncement(self):
        self.record()
        self.assertEqual(self.record(fixtures(5_000_000_000),now_ns=5_100_000_000)["reason"],"clock_reset_requires_new_epoch")
        self.tracker.on_epoch("epoch-a",5_200_000_000)
        self.assertEqual(self.record(fixtures(5_300_000_000),now_ns=5_400_000_000)["reason"],"clock_reset_requires_new_epoch")
        self.tracker.on_epoch("epoch-b",5_500_000_000)
        self.assertEqual(self.record(fixtures(5_600_000_000),now_ns=5_700_000_000)["status"],"processed")

    def test_between_frame_clock_reset_stays_locked_after_clock_catches_up(self):
        self.record()
        self.tracker.invalidate_clock()
        self.assertEqual(self.record(fixtures(11_000_000_000),now_ns=11_100_000_000)["reason"],
                         "clock_reset_requires_new_epoch")
        self.tracker.on_epoch("epoch-a",11_200_000_000)
        self.assertFalse(self.tracker.epoch_ready)
        self.tracker.on_epoch("epoch-b",11_300_000_000)
        self.assertEqual(self.record(fixtures(11_400_000_000),now_ns=11_500_000_000)["status"],"processed")

    def test_invalid_metadata_is_finite_bounded_rejection(self):
        reasons = {"missing":"missing_camera_info", "frame":"frame_mismatch", "nan":"invalid_calibration",
                   "size":"dimension_mismatch", "roi":"unsupported_roi_or_binning", "stamp":"invalid_source_stamp",
                   "encoding":"unsupported_encoding", "malformed_encoding":"unsupported_encoding", "stale":"stale_camera_info"}
        for index,(field,reason) in enumerate(reasons.items()):
            with self.subTest(field=field):
                args=list(fixtures())
                if field=="missing":args[2]=None
                elif field=="frame":args[1].header.frame_id="other"
                elif field=="nan":args[2].p[0]=float("nan")
                elif field=="size":args[1].width=79
                elif field=="roi":args[2].roi.do_rectify=True
                elif field=="stamp":args[0].header.stamp.nanosec=-1
                elif field=="encoding":args[0].encoding="mono8"
                elif field=="malformed_encoding":args[0].encoding=[]
                elif field=="stale":args[2].header=header(1_000_000_000)
                result=self.record(args,now_ns=10_200_000_000+index*20_000_000)
                self.assertEqual(result["status"],"rejected")
                self.assertEqual(result["reason"],reason)
                self.assertLess(len(json.dumps(result,allow_nan=False)),6000)

    def test_hashes_include_raw_padding_and_full_calibration_but_not_info_time(self):
        color,depth,info,_,_=fixtures()
        color.step=244;color.data=b"x"*(color.height*color.step);color.is_bigendian=1
        raw=image_metadata(color)
        self.assertEqual(raw["data_sha256"],hashlib.sha256(color.data).hexdigest())
        self.assertEqual((raw["step"],raw["is_bigendian"]),(244,1))
        baseline=camera_metadata(info)
        later=deepcopy(info);later.header=header(10_200_000_000)
        self.assertEqual(camera_metadata(later)["calibration_id"],baseline["calibration_id"])
        for field in ("r","p","roi"):
            changed=deepcopy(info)
            if field=="roi":changed.roi.do_rectify=True
            else:getattr(changed,field)[0]+=.01
            self.assertNotEqual(camera_metadata(changed)["calibration_id"],baseline["calibration_id"])
        self.assertNotEqual(FrameEvidence(DetectorConfig()).session_id,self.tracker.session_id)
        self.assertNotEqual(FrameEvidence(DetectorConfig(min_area_pixels=31)).config_id,self.tracker.config_id)

    def test_numpy_camera_info_matrices_match_ros_generated_message_storage(self):
        args=fixtures()
        for field in ("k","r","p"):
            setattr(args[2],field,np.asarray(getattr(args[2],field),dtype=np.float64))
        result=self.record(args)
        self.assertEqual(result["status"],"processed",result)
        self.assertIs(type(result["camera_info"]["calibration"]["k"][0]),float)


try:
    import rclpy
except ImportError:
    rclpy = None


@unittest.skipUnless(rclpy is not None and os.environ.get("ROS_DOMAIN_ID")=="74" and
                     os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE")=="LOCALHOST",
                     "Real callback test requires ROS installed and explicit domain 74 / LOCALHOST")
class RealPerceptionCallbackTests(unittest.TestCase):
    def test_real_synchronized_messages_publish_evidence_and_keep_legacy_target_contract(self):
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.parameter import Parameter
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        from sensor_msgs.msg import CameraInfo, Image
        from std_msgs.msg import String
        from rosgraph_msgs.msg import Clock
        from limo_maze_perception.node import RedRGBDBaseline
        context=Context();rclpy.init(context=context)
        executor=SingleThreadedExecutor(context=context)
        probe=rclpy.create_node("frame_processing_test_probe",context=context,enable_rosout=False)
        executor.add_node(probe)
        detector=None
        def spin(seconds=.15):
            end=time.monotonic()+seconds
            while time.monotonic()<end:executor.spin_once(timeout_sec=.01)
        try:
            spin(.5)
            self.assertEqual([n for n in probe.get_node_names() if n!=probe.get_name()],[],"Domain must be isolated")
            params={"use_sim_time":True,"enabled":True,"depth_registered_to_color":True,
                    "publish_frame_processing":True,"color_topic":"/frame_test/rgb","depth_topic":"/frame_test/depth",
                    "camera_info_topic":"/frame_test/info","output_topic":"/frame_test/targets",
                    "frame_processing_topic":"/frame_test/evidence"}
            detector=RedRGBDBaseline(context=context,parameter_overrides=[Parameter(k,value=v) for k,v in params.items()])
            executor.add_node(detector)
            evidence=[];targets=[]
            probe.create_subscription(String,"/frame_test/evidence",lambda m:evidence.append(json.loads(m.data)),10)
            probe.create_subscription(String,"/frame_test/targets",lambda m:targets.append(json.loads(m.data)),10)
            pubs={name:probe.create_publisher(kind,"/frame_test/"+name,10) for name,kind in
                  (("rgb",Image),("depth",Image),("info",CameraInfo))}
            clockpub=probe.create_publisher(Clock,"/clock",10)
            epochpub=probe.create_publisher(String,"/mission/map_epoch",QoSProfile(depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
            spin(.5)
            def clock(ns):
                msg=Clock();msg.clock.sec=ns//1_000_000_000;msg.clock.nanosec=ns%1_000_000_000
                clockpub.publish(msg);spin(.1)
            def pair(ns,bad_candidate=False,depth_ns=None):
                color,dep,info,rgb,depth=fixtures(ns,depth_ns)
                if bad_candidate:
                    rgb[20:40,30:50]=[255,0,0];depth[20:40,30:50]=np.nan
                    color.data=rgb.tobytes();dep.data=depth.tobytes()
                actual_info=CameraInfo()
                actual_info.header.frame_id=info.header.frame_id
                actual_info.header.stamp.sec=ns//1_000_000_000;actual_info.header.stamp.nanosec=ns%1_000_000_000
                for field in ("width","height","distortion_model","binning_x","binning_y","k","d","r","p"):
                    setattr(actual_info,field,getattr(info,field))
                pubs["info"].publish(actual_info);spin(.1)
                for name,source in (("rgb",color),("depth",dep)):
                    msg=Image();msg.header.frame_id=source.header.frame_id
                    msg.header.stamp.sec=source.header.stamp.sec;msg.header.stamp.nanosec=source.header.stamp.nanosec
                    for field in ("width","height","encoding","step","is_bigendian","data"):
                        setattr(msg,field,getattr(source,field))
                    pubs[name].publish(msg)
                spin(.25)
            clock(10_000_000_000);epochpub.publish(String(data="callback-epoch-a"));spin(.15)
            clock(10_200_000_000);pair(10_100_000_000)
            self.assertTrue(evidence,"Production on_pair callback must actually publish")
            self.assertEqual(evidence[-1]["status"],"processed",evidence[-1])
            self.assertEqual(evidence[-1]["processing"]["empty_reason"],"no_red_candidates")
            self.assertEqual(set(targets[-1]),{"schema_version","stamp","frame_id","source","detections"})
            clock(10_400_000_000);pair(10_300_000_000,True)
            self.assertEqual(evidence[-1]["processing"]["unlocalized_candidate_count"],1)
            self.assertEqual(evidence[-1]["status"],"processed")
            clock(10_500_000_000);pair(10_320_000_000,depth_ns=10_300_000_000)
            self.assertEqual(evidence[-1]["reason"],"nonadvancing_sample")
            self.assertGreaterEqual(len(targets),3,"Optional evidence gating must not alter legacy targets")
            self.assertFalse(any("coverage" in row for row in evidence))
            # A clock reset between image callbacks must not disappear when time catches up.
            clock(5_000_000_000);clock(11_000_000_000);pair(10_900_000_000)
            self.assertEqual(evidence[-1]["reason"],"clock_reset_requires_new_epoch")
            epochpub.publish(String(data="callback-epoch-a"));spin(.15)
            clock(11_200_000_000);pair(11_100_000_000)
            self.assertEqual(evidence[-1]["reason"],"clock_reset_requires_new_epoch")
            epochpub.publish(String(data="callback-epoch-b"));spin(.15)
            clock(11_400_000_000);pair(11_300_000_000)
            self.assertEqual(evidence[-1]["status"],"processed",evidence[-1])
            # Explicit evidence output does not bypass the existing detector enable switch.
            executor.remove_node(detector);detector.destroy_node();detector=None
            params["enabled"]=False
            detector=RedRGBDBaseline(context=context,parameter_overrides=[Parameter(k,value=v) for k,v in params.items()])
            executor.add_node(detector);spin(.5)
            before_evidence,before_targets=len(evidence),len(targets)
            clock(12_200_000_000);pair(12_100_000_000)
            self.assertGreater(len(evidence),before_evidence,"Disabled production callback must publish a rejection")
            self.assertTrue(all(row["status"]=="rejected" and row["reason"]=="detector_disabled"
                                for row in evidence[before_evidence:]))
            self.assertEqual(len(targets),before_targets)
            # A second real instance with defaults creates no evidence publisher.
            executor.remove_node(detector);detector.destroy_node();detector=None
            plain=RedRGBDBaseline(context=context)
            try:self.assertIsNone(plain.frame_publisher)
            finally:plain.destroy_node()
        finally:
            if detector is not None:executor.remove_node(detector);detector.destroy_node()
            executor.remove_node(probe);probe.destroy_node();executor.shutdown();context.shutdown()


if __name__=="__main__":unittest.main()
