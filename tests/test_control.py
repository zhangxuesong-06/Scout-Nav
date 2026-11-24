import math
import numpy as np
import pytest

from scout_nav_bringup.motion_gate import CommandPolicy
from scout_visual_follow.depth_control import DistanceController, measure_target_depth, target_distance
from scout_visual_follow.follow_parameters import resolve_approach
from scout_visual_follow.geometry import approach_command, transform_point


@pytest.mark.parametrize('encoding,scale,dtype', [('16UC1', 1000, np.uint16), ('32FC1', 1, np.float32)])
def test_depth_units_and_central_background_rejection(encoding, scale, dtype):
    depth = np.full((100, 100), 3.0 * scale, dtype=dtype)
    depth[30:70, 30:70] = 1.5 * scale
    assert target_distance(depth, encoding, (0, 0, 100, 100), (100, 100, 3)) == pytest.approx(1.5)


def test_sparse_invalid_or_unregistered_depth_stops():
    depth = np.zeros((100, 100), np.uint16)
    depth[40:45, 40:45] = 1500
    assert target_distance(depth, '16UC1', (0, 0, 100, 100), (100, 100, 3)) is None
    depth[:] = 1500
    assert target_distance(depth, '16UC1', (0, 0, 100, 100), (50, 100, 3)) is None
    assert target_distance(depth, 'mono16', (0, 0, 100, 100), (100, 100, 3)) is None


def test_grape_depth_ratio_can_be_tuned_but_noise_and_missing_depth_still_stop():
    depth = np.zeros((100, 100), np.uint16)
    roi = depth[40:60, 36:44]
    roi[::4, :] = 1500  # 40 of 160 centre pixels are current valid depth.
    box = (30, 25, 50, 75)
    assert target_distance(depth, '16UC1', box, (100, 100, 3)) is None
    measurement = measure_target_depth(depth, '16UC1', box, (100, 100, 3), min_ratio=.15)
    assert measurement.distance == pytest.approx(1.5)
    assert measurement.valid_pixels == 40 and measurement.roi_pixels == 160
    assert measurement.valid_ratio == pytest.approx(.25) and measurement.reason == ''
    roi[:] = 0
    roi[0, :] = 1500  # Isolated noise cannot satisfy the absolute pixel minimum.
    assert measure_target_depth(depth, '16UC1', box, (100, 100, 3), min_ratio=.15).reason == 'too_few_depth_pixels'
    roi[:] = 0
    assert target_distance(depth, '16UC1', box, (100, 100, 3), min_ratio=.15) is None


@pytest.mark.parametrize('pixels,ratio', [(0, .15), (20, 0), (20, 1.1), (20, float('nan'))])
def test_depth_quality_thresholds_must_not_disable_validation(pixels, ratio):
    with pytest.raises(ValueError):
        target_distance(np.zeros((100, 100)), '16UC1', (0, 0, 100, 100),
                        (100, 100, 3), min_pixels=pixels, min_ratio=ratio)


def test_hysteresis_near_stop_and_invalid_target():
    controller = DistanceController()
    assert controller.speed(1.1) == 0
    assert controller.speed(1.3) > 0
    assert controller.speed(1.1) > 0
    assert controller.speed(1.0) == 0
    assert controller.speed(1.1) == 0
    assert controller.speed(10) == 0.2
    assert controller.speed(float('nan')) == 0


@pytest.mark.parametrize('overrides,parameter', [
    ({'stop_distance': '0.2'}, 'stop_distance'),
    ({'stop_distance': '1.0'}, 'stop_distance'),
    ({'resume_distance': '4.0'}, 'resume_distance'),
    ({'resume_distance': 'nan'}, 'resume_distance'),
    ({'linear_gain': '0'}, 'linear_gain'),
    ({'linear_gain': '-1'}, 'linear_gain'),
    ({'linear_gain': 'inf'}, 'linear_gain'),
    ({'linear_gain': 'fast'}, 'linear_gain'),
])
def test_invalid_approach_settings_are_rejected(overrides, parameter):
    with pytest.raises(ValueError, match=parameter):
        resolve_approach(overrides)


def test_tuned_approach_scales_linear_speed_up_to_unchanged_limit():
    controller = DistanceController(stop=0.8, resume=1.2, gain=0.1, max_speed=0.05)
    assert controller.speed(1.1) == 0.0  # Stationary until the resume threshold.
    assert controller.speed(1.2) == pytest.approx(0.04)
    assert controller.speed(1.8) == 0.05
    assert controller.speed(0.8) == 0.0


def test_tilt_and_camera_offset_transform_optical_target_to_base():
    # R maps optical X-right,Y-down,Z-forward into body X-forward,Y-left,Z-up.
    optical_to_body = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    pitch = math.radians(30)
    mount_pitch = np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0],
                            [-math.sin(pitch), 0, math.cos(pitch)]])
    # Compose body pitch quaternion with the optical convention quaternion.
    p, w = math.sin(pitch / 2), math.cos(pitch / 2)
    q = [-0.5 * w - 0.5 * p, 0.5 * w + 0.5 * p,
         -0.5 * w + 0.5 * p, 0.5 * w - 0.5 * p]
    point = transform_point([0, 0, 2], [0.2, 0, 0.6], q)
    expected = mount_pitch @ optical_to_body @ np.array([0, 0, 2]) + [0.2, 0, 0.6]
    assert point == pytest.approx(expected)
    assert point[0] != pytest.approx(2.0)  # Optical Z is not the base XY distance.


def test_transform_requires_real_finite_rotation():
    with pytest.raises(ValueError):
        transform_point([0, 0, 1], [0, 0, 0], [0, 0, 0, 0])
    with pytest.raises(ValueError):
        transform_point([0, float('nan'), 1], [0, 0, 0], [0, 0, 0, 1])


def test_target_approach_uses_base_xy_and_never_reverses():
    controller = DistanceController(max_speed=0.15)
    forward, angular = approach_command(controller, [2, 0.3, 1.0])
    assert 0 < forward <= 0.15 and angular > 0
    assert approach_command(controller, [0.8, 0, 0.3]) == (0, 0)
    assert approach_command(controller, None) == (0, 0)
    assert approach_command(controller, [-2, 0, 0]) == (0, 0)


def enabled_policy():
    policy = CommandPolicy(mode='navigation')
    policy.depth_time = policy.odom_time = 10.0
    policy.enable(True)
    return policy


def test_gate_is_disabled_by_default_and_sources_do_not_mix():
    policy = CommandPolicy(mode='navigation')
    policy.depth_time = policy.odom_time = 10.0
    policy.receive('navigation', 0.1, 0, 10.0)
    assert policy.output(10.0) == (0, 0)
    policy.enable(True)
    assert policy.output(10.0) == (0, 0)  # Pre-enable command was cleared.
    policy.receive('follow', 0.1, 0.2, 10.0)
    assert policy.output(10.0) == (0, 0)
    policy.receive('navigation', 0.1, 0.2, 10.0)
    assert policy.output(10.0) == (0.1, 0.2)


def test_gate_limits_speed_and_latches_off_on_stale_command():
    policy = enabled_policy()
    policy.receive('navigation', 2.0, -3.0, 10.0)
    assert policy.output(10.1) == (0.15, -0.35)
    policy.depth_time = policy.odom_time = 10.5
    assert policy.output(10.5) == (0, 0)
    assert not policy.enabled
    policy.receive('navigation', 0.1, 0, 10.5)
    assert policy.output(10.5) == (0, 0)


@pytest.mark.parametrize('missing', ['depth_time', 'odom_time'])
def test_gate_camera_or_odom_loss_requires_reenable(missing):
    policy = enabled_policy()
    policy.receive('navigation', 0.1, 0, 10.0)
    setattr(policy, missing, None)
    assert policy.output(10.0) == (0, 0)
    setattr(policy, missing, 10.0)
    assert policy.output(10.0) == (0, 0)


def test_gate_invalid_command_stops_and_no_reverse():
    policy = enabled_policy()
    policy.receive('navigation', -1.0, 0.1, 10.0)
    assert policy.output(10.0) == (0, 0.1)
    policy.receive('navigation', float('nan'), 0.0, 10.0)
    assert not policy.enabled and policy.output(10.0) == (0, 0)


def test_startup_auto_enable_runs_once_and_never_recovers_after_timeout(monkeypatch):
    import threading
    from types import SimpleNamespace as NS
    from scout_nav_bringup import motion_gate
    now, sent = [10.0], []
    monkeypatch.setattr(motion_gate.time, 'monotonic', lambda: now[0])
    policy = CommandPolicy(mode='follow', max_linear=.05, max_angular=.15)
    node = NS(policy=policy, startup_enable_pending=True, lock=threading.RLock(),
              output_pub=NS(publish=sent.append),
              get_logger=lambda: NS(info=lambda _: None, warning=lambda _: None))
    motion_gate.MotionGate.tick(node)
    assert not policy.enabled  # Missing sensors/command cannot enable.
    policy.depth_time = policy.odom_time = now[0]
    policy.receive('follow', .05, .1, now[0])
    motion_gate.MotionGate.tick(node)
    assert policy.enabled and not node.startup_enable_pending
    assert sent[-1].linear.x == 0  # A NEW post-enable command is still required.
    policy.receive('follow', .05, .1, now[0])
    motion_gate.MotionGate.tick(node)
    assert sent[-1].linear.x == .05
    now[0] = 11.0
    motion_gate.MotionGate.tick(node)
    assert not policy.enabled and sent[-1].linear.x == 0
    policy.depth_time = policy.odom_time = now[0]
    policy.receive('follow', .05, .1, now[0])
    motion_gate.MotionGate.tick(node)
    assert not policy.enabled and sent[-1].linear.x == 0
