from types import SimpleNamespace as NS
import threading
import numpy as np
import pytest

from scout_visual_follow.node import VisualFollower
from scout_visual_follow.depth_control import DistanceController


def fake_node():
    commands = []
    node = NS(
        target_id=0, last_target_param=0,
        get_parameter=lambda name: NS(value=0),
        bridge=NS(imgmsg_to_cv2=lambda msg, encoding: msg.data,
                  cv2_to_imgmsg=lambda frame, encoding: NS(header=None)),
        controller=DistanceController(), min_depth=.15, max_depth=4., max_angular=.35,
        confidence=.2, imgsz=640, min_box_area=2000., device='',
        depth_min_pixels=20, depth_min_ratio=.3, last_stop_log=0.,
        lock=threading.RLock(), closed=False, input_mode=False, show_window=False,
        pub=NS(publish=commands.append), preview_pub=NS(publish=lambda msg: None),
        targets_pub=NS(publish=lambda msg: None),
        get_clock=lambda: NS(now=lambda: NS(nanoseconds=20_000_000_000)),
        get_logger=lambda: NS(warning=lambda msg: None),
        targets={}, next_id=0, max_miss=8,
    )
    node.assign_ids = lambda detections: VisualFollower.assign_ids(node, detections)
    node.stop = lambda reason='': VisualFollower.stop(node, reason)
    node.report_stop = lambda reason: VisualFollower.report_stop(node, reason)
    node.point_in_base = lambda target, depth, rgb: np.array([depth, 0.25, 0.5])
    node.fresh = lambda packet: True
    rows = np.array([[0, 0, 100, 100, .9, 0]])
    class Model:
        names = {0: 'person'}
        def __call__(self, *args, **kwargs):
            return [NS(boxes=NS(data=NS(cpu=lambda: NS(numpy=lambda: rows))))]
    node.model = Model()
    rgb = NS(data=np.zeros((100, 100, 3), np.uint8),
             header=NS(frame_id='color_optical', stamp=NS(sec=20, nanosec=0)), width=100, height=100)
    depth = NS(data=np.full((100, 100), 2000, np.uint16), encoding='16UC1',
               header=NS(frame_id='color_optical'))
    return node, (rgb, depth, 10.0), commands


def test_grape_profile_keeps_small_boxes_and_passes_inference_parameters():
    node, packet, commands = fake_node()
    node.confidence, node.imgsz, node.device = .1, 800, '0'
    calls = []
    class Model:
        names = {0: 'uva_bbox'}
        def __call__(self, *args, **kwargs):
            calls.append(kwargs)
            rows = np.array([[30, 25, 50, 75, .15, 0]])  # 1000 pixels; valid depth ROI.
            return [NS(boxes=NS(data=NS(cpu=lambda: NS(numpy=lambda: rows))))]
    node.model = Model()
    VisualFollower.process(node, packet)
    assert commands[-1].linear.x == commands[-1].angular.z == 0
    node.min_box_area = 200.
    VisualFollower.process(node, packet)
    assert commands[-1].linear.x > 0
    assert calls[-1]['conf'] == .1 and calls[-1]['imgsz'] == 800 and calls[-1]['device'] == '0'


def test_actual_process_stops_for_missing_invalid_near_or_stale_target():
    node, packet, commands = fake_node()
    VisualFollower.process(node, packet)
    assert commands[-1].linear.x > 0 and commands[-1].angular.z > 0
    packet[1].data[:] = 0
    VisualFollower.process(node, packet)
    assert commands[-1].linear.x == commands[-1].angular.z == 0
    packet[1].data[:] = 800
    VisualFollower.process(node, packet)
    assert commands[-1].linear.x == commands[-1].angular.z == 0
    packet[1].data[:] = 2000
    node.target_id = 999
    VisualFollower.process(node, packet)
    assert commands[-1].linear.x == commands[-1].angular.z == 0
    node.target_id = 0
    freshness = iter([True, False])
    node.fresh = lambda _: next(freshness)
    VisualFollower.process(node, packet)
    assert commands[-1].linear.x == commands[-1].angular.z == 0


def test_unregistered_depth_is_rejected():
    node, packet, _ = fake_node()
    packet[1].header.frame_id = 'depth_optical'
    with pytest.raises(ValueError, match='color optical'):
        VisualFollower.process(node, packet)


def test_sparse_grape_depth_and_diagnostic_reasons_without_replaying_old_depth():
    import json
    node, packet, commands = fake_node()
    node.depth_min_ratio = .15
    diagnostics = []
    node.targets_pub.publish = lambda msg: diagnostics.append(json.loads(msg.data))
    # 25% of the centre has current-frame depth; generic 30% would reject it.
    packet[1].data[:] = 0
    packet[1].data[30:40, 30:70] = 1500
    VisualFollower.process(node, packet)
    assert commands[-1].linear.x > 0
    assert diagnostics[-1]['control_state'] == 'following'
    assert diagnostics[-1]['depth_valid_ratio'] == pytest.approx(.25)
    assert diagnostics[-1]['depth_valid_pixels'] == 400
    assert diagnostics[-1]['processing_ms'] >= 0
    # A complete depth loss must immediately stop, even after a valid frame.
    packet[1].data[:] = 0
    VisualFollower.process(node, packet)
    assert commands[-1].linear.x == commands[-1].angular.z == 0
    assert diagnostics[-1]['control_state'] == 'too_few_depth_pixels'
    assert diagnostics[-1]['selected_depth_m'] is None
    packet[1].data[:] = 800
    VisualFollower.process(node, packet)
    assert diagnostics[-1]['control_state'] == 'distance_hold'
    freshness = iter([True, False])
    node.fresh = lambda _: next(freshness)
    packet[1].data[:] = 2000
    VisualFollower.process(node, packet)
    assert diagnostics[-1]['control_state'] == 'stale_after_inference'
    assert commands[-1].linear.x == commands[-1].angular.z == 0


def test_actual_timestamp_freshness():
    import time
    now = 20.0
    node = NS(timeout=.6, slop=.08, get_clock=lambda: NS(now=lambda: NS(nanoseconds=20_000_000_000)))
    def msg(stamp):
        return NS(header=NS(stamp=NS(sec=int(stamp), nanosec=int((stamp-int(stamp))*1e9))))
    assert VisualFollower.fresh(node, (msg(now-.02), msg(now), time.monotonic()))
    assert not VisualFollower.fresh(node, (msg(now-1), msg(now-1), time.monotonic()))
    assert not VisualFollower.fresh(node, (msg(now), msg(now), time.monotonic()-1))
    assert not VisualFollower.fresh(node, (msg(0), msg(0), time.monotonic()))
