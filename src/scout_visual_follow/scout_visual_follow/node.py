#!/usr/bin/env python3
"""ROS 2 port of demo/visual_follow; output is a candidate command only."""
from pathlib import Path
import json
import threading
import time

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from .depth_control import DistanceController, measure_target_depth
from .follow_parameters import DEFAULT_APPROACH, DEFAULT_MAX_DEPTH, DEFAULT_MIN_DEPTH, resolve_approach
from .geometry import approach_command, transform_point


class VisualFollower(Node):
    def __init__(self):
        super().__init__('visual_follower', namespace='scout')
        defaults = dict(
            model_path='', color_topic='/nav_camera/color/image_raw',
            depth_topic='/nav_camera/depth/image_raw',
            camera_info_topic='/nav_camera/color/camera_info', base_frame='base_link',
            cmd_vel_topic='/scout/follow_cmd_vel', depth_registered=False,
            **DEFAULT_APPROACH,
            max_linear_speed=0.15, max_angular_speed=0.35,
            image_timeout=0.6, sync_slop=0.08, min_depth=DEFAULT_MIN_DEPTH, max_depth=DEFAULT_MAX_DEPTH,
            show_window=False, target_id=-1,
            auto_target=False, auto_confirm_frames=3, auto_release_frames=3, auto_vertical_margin_px=0,
            confidence=0.2, imgsz=640, min_box_area=2000.0, device='',
            depth_min_valid_pixels=20, depth_min_valid_ratio=0.3,
        )
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value
        path = Path(p('model_path')).expanduser()
        if not path.is_file():
            raise ValueError(f'Model file does not exist: {path}; automatic downloads are disabled')
        if not p('depth_registered'):
            raise ValueError('Enable depth registration in the camera driver first')
        self.timeout, self.slop = float(p('image_timeout')), float(p('sync_slop'))
        self.min_depth, self.max_depth = float(p('min_depth')), float(p('max_depth'))
        self.max_angular = float(p('max_angular_speed'))
        self.confidence, self.imgsz = float(p('confidence')), int(p('imgsz'))
        self.min_box_area, self.device = float(p('min_box_area')), str(p('device'))
        self.depth_min_pixels = int(p('depth_min_valid_pixels'))
        self.depth_min_ratio = float(p('depth_min_valid_ratio'))
        if self.depth_min_pixels < 1 or not 0 < self.depth_min_ratio <= 1:
            raise ValueError('Require depth_min_valid_pixels >= 1 and 0 < depth_min_valid_ratio <= 1')
        if not (0 < self.confidence <= 1 and self.imgsz > 0
                and np.isfinite(self.min_box_area) and self.min_box_area >= 0):
            raise ValueError('Require 0 < confidence <= 1, imgsz > 0 and finite min_box_area >= 0')
        approach = resolve_approach({name: p(name) for name in DEFAULT_APPROACH},
                                    min_depth=self.min_depth, max_depth=self.max_depth)
        self.controller = DistanceController(approach['stop_distance'], approach['resume_distance'],
                                             approach['linear_gain'], p('max_linear_speed'))
        if not (0 < self.slop <= self.timeout and self.max_angular > 0):
            raise ValueError('Invalid image timeout, sync slop or angular speed')
        self.get_logger().info('Approach: ' + ', '.join(f'{name}={value:g}' for name, value in approach.items()))
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError('ultralytics is missing; see ~/scout/README.md vision environment setup') from exc
        self.model = YOLO(str(path))
        self.get_logger().info(f'Model: {path}; classes: {self.model.names}; '
                               f'confidence={self.confidence}, imgsz={self.imgsz}, '
                               f'min_box_area={self.min_box_area}, device={self.device or "auto"}')
        # Complete lazy model/GPU setup before publishing the first command.
        # Otherwise startup zeroes can enable the gate just before warmup blocks
        # command production and immediately latches it off on a timeout.
        self.get_logger().info('Warming up detector before the command publisher starts...')
        self.model(np.zeros((360, 640, 3), dtype=np.uint8), conf=self.confidence,
                   imgsz=self.imgsz, device=self.device or None,
                   iou=0.5, max_det=20, verbose=False)
        self.bridge, self.show_window = CvBridge(), bool(p('show_window'))
        self.base_frame = str(p('base_frame'))
        self.lock, self.done = threading.RLock(), threading.Event()
        self.pending, self.camera_info, self.last_command_time = None, None, None
        self.closed, self.last_error = False, 0.0
        self.last_stop_log = 0.0
        self.targets, self.next_id, self.max_miss = {}, 0, 8
        self.auto_target = bool(p('auto_target'))
        self.auto_confirm, self.auto_release = p('auto_confirm_frames'), p('auto_release_frames')
        self.auto_vertical_margin = p('auto_vertical_margin_px')
        if self.auto_confirm < 1 or self.auto_release < 1 or self.auto_vertical_margin < 0:
            raise ValueError('Automatic selection requires positive frame counts and a nonnegative margin')
        self.auto_candidate, self.auto_hits, self.auto_misses, self.auto_stamp = None, 0, 0, 0
        self.target_id, self.input_mode, self.input_buffer = None, False, ''
        self.last_target_param = None
        self.pub = self.create_publisher(Twist, p('cmd_vel_topic'), 1)
        self.preview_pub = self.create_publisher(Image, '/scout/follow_preview', qos_profile_sensor_data)
        self.targets_pub = self.create_publisher(String, '/scout/follow_targets', 1)
        self.info_sub = self.create_subscription(CameraInfo, p('camera_info_topic'),
                                                 self.receive_info, qos_profile_sensor_data)
        self.rgb_sub = message_filters.Subscriber(self, Image, p('color_topic'),
                                                  qos_profile=qos_profile_sensor_data)
        self.depth_sub = message_filters.Subscriber(self, Image, p('depth_topic'),
                                                    qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=5, slop=self.slop)
        self.sync.registerCallback(self.receive)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.watchdog = threading.Thread(target=self.watch, daemon=True)
        self.watchdog.start()
        selection = ('Automatic target selection is enabled.' if self.auto_target else
                     'Select an ID with target_id parameter, or i / digits / Enter in GUI; c cancels.')
        self.get_logger().info('Preview: /scout/follow_preview; commands: /scout/follow_cmd_vel. '
                               'An external motion gate controls whether the base moves. ' + selection)

    def receive_info(self, msg):
        self.camera_info = msg

    def receive(self, rgb, depth):
        with self.lock:
            self.pending = (rgb, depth, time.monotonic())

    def fresh(self, packet):
        rgb, depth, received = packet
        now = self.get_clock().now().nanoseconds * 1e-9
        stamps = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in (rgb, depth)]
        return (time.monotonic() - received <= self.timeout
                and all(s > 0 and -self.slop <= now - s <= self.timeout for s in stamps)
                and abs(stamps[0] - stamps[1]) <= self.slop)

    def stop(self, reason=''):
        with self.lock:
            self.controller.reset()
            self.pub.publish(Twist())
        if reason:
            self.report_stop(reason)

    def report_stop(self, reason):
        now = time.monotonic()
        if now - self.last_stop_log >= 2.0:
            self.get_logger().warning(f'Follow STOP: {reason}')
            self.last_stop_log = now

    def watch(self):
        # Wall-clock protection still runs while inference blocks or ROS time pauses.
        while not self.done.wait(0.05):
            with self.lock:
                if self.closed:
                    return
                if (self.last_command_time is None or
                        time.monotonic() - self.last_command_time > self.timeout):
                    self.stop('image_or_processing_timeout'
                              if self.last_command_time is not None else '')

    def select_key(self):
        key = cv2.waitKey(1) & 0xff
        if key == 27:
            self.closed = True
            return
        if self.auto_target:
            return
        if key == ord('c'):
            self.set_parameters([Parameter('target_id', value=-1)])
            self.input_mode, self.input_buffer = False, ''
            self.stop()
        elif key == ord('i'):
            self.input_mode, self.input_buffer = True, ''
        elif self.input_mode:
            if key in (10, 13):
                if self.input_buffer:
                    self.set_parameters([Parameter('target_id', value=int(self.input_buffer))])
                self.input_mode = False
            elif key in (8, 127):
                self.input_buffer = self.input_buffer[:-1]
            elif ord('0') <= key <= ord('9'):
                self.input_buffer += chr(key)

    def assign_ids(self, detections):
        pairs = sorted(
            (abs(det['cx'] - obj['cx']) + abs(det['cy'] - obj['cy']), index, tid)
            for index, det in enumerate(detections) for tid, obj in self.targets.items()
            if det['cls'] == obj['cls'])
        used_dets, used_ids = set(), set()
        for distance, index, tid in pairs:
            if distance > 120 or index in used_dets or tid in used_ids:
                continue
            detections[index]['id'] = tid
            self.targets[tid].update(cx=detections[index]['cx'], cy=detections[index]['cy'], miss=0)
            used_dets.add(index)
            used_ids.add(tid)
        for det in detections:
            if 'id' not in det:
                tid = self.next_id
                self.next_id += 1
                det['id'] = tid
                self.targets[tid] = dict(cx=det['cx'], cy=det['cy'], cls=det['cls'], miss=0)
                used_ids.add(tid)
        for tid in list(self.targets):
            if tid not in used_ids:
                self.targets[tid]['miss'] += 1
                if self.targets[tid]['miss'] > self.max_miss:
                    del self.targets[tid]
        return detections

    def point_in_base(self, target, depth, rgb):
        info = self.camera_info
        if (info is None or info.width != rgb.width or info.height != rgb.height
                or info.header.frame_id != rgb.header.frame_id):
            raise ValueError('Need matching color CameraInfo and image frame/size')
        if info.distortion_model not in ('', 'plumb_bob', 'rational_polynomial'):
            raise ValueError(f'Unsupported distortion model: {info.distortion_model}')
        k = np.asarray(info.k, dtype=float).reshape(3, 3)
        if not np.all(np.isfinite(k)) or k[0, 0] <= 0 or k[1, 1] <= 0:
            raise ValueError('Invalid camera intrinsics')
        uv = np.asarray([[[target['cx'], target['cy']]]], dtype=float)
        distortion = np.asarray(info.d, dtype=float)
        ray = cv2.undistortPoints(uv, k, distortion if distortion.size else None)[0, 0]
        optical_point = [float(ray[0]) * depth, float(ray[1]) * depth, depth]
        tf = self.tf_buffer.lookup_transform(self.base_frame, rgb.header.frame_id,
                                             Time.from_msg(rgb.header.stamp),
                                             timeout=Duration(seconds=0.03)).transform
        t, q = tf.translation, tf.rotation
        return transform_point(optical_point, [t.x, t.y, t.z], [q.x, q.y, q.z, q.w])

    def select_automatic(self, detections, measure, stamp):
        """Lock one ID only after consecutive fresh depth-valid frames."""
        if stamp <= self.auto_stamp or stamp - self.auto_stamp > self.timeout:
            self.target_id, self.auto_candidate = None, None
            self.auto_hits, self.auto_misses = 0, 0
        if stamp <= self.auto_stamp:
            self.auto_stamp = stamp
            return None, None
        self.auto_stamp = stamp
        if self.target_id is not None:
            target = next((d for d in detections if d['id'] == self.target_id), None)
            result, point = measure(target)
            if point is not None and point[0] > 0:
                self.auto_misses = 0
            else:
                self.auto_misses += 1
                if self.auto_misses >= self.auto_release:
                    self.target_id, self.auto_candidate = None, None
                    self.auto_hits, self.auto_misses = 0, 0
            return result, point
        # Prefer continuing the candidate's streak over switching to a louder box.
        for det in sorted(detections, key=lambda d: (d['id'] != self.auto_candidate,
                                                     -d['confidence'], d['id'])):
            result, point = measure(det)
            if point is None or point[0] <= 0:
                continue
            self.auto_hits = self.auto_hits + 1 if det['id'] == self.auto_candidate else 1
            self.auto_candidate = det['id']
            if self.auto_hits >= self.auto_confirm:
                self.target_id = det['id']
                return result, point
            return None, None
        self.auto_candidate, self.auto_hits = None, 0
        return None, None

    def process(self, packet):
        started = time.monotonic()
        rgb, depth_msg, received = packet
        if not self.fresh(packet):
            self.stop('stale_rgbd_packet')
            return
        auto = self.auto_target
        if not auto:
            selected = int(self.get_parameter('target_id').value)
            if selected != self.last_target_param:
                self.target_id = selected if selected >= 0 else None
                self.last_target_param = selected
                self.stop()
        frame = self.bridge.imgmsg_to_cv2(rgb, 'bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
        if depth_msg.header.frame_id != rgb.header.frame_id:
            raise ValueError('Registered depth must use the color optical frame')
        detections = []
        for result in self.model(frame, conf=self.confidence, imgsz=self.imgsz,
                                 device=self.device or None, iou=0.5, max_det=20, verbose=False):
            if result.boxes is None:
                continue
            for row in result.boxes.data.cpu().numpy():
                x1, y1, x2, y2, confidence, cls = row[:6]
                if (x2 - x1) * (y2 - y1) < self.min_box_area:
                    continue
                detections.append(dict(cx=int((x1 + x2) / 2), cy=int((y1 + y2) / 2),
                                       cls=int(cls), confidence=float(confidence), box=(x1, y1, x2, y2)))
        self.assign_ids(detections)
        def measure(target):
            if target is None:
                return None, None
            result = measure_target_depth(
                depth, depth_msg.encoding, target['box'], frame.shape, self.min_depth, self.max_depth,
                min_pixels=self.depth_min_pixels, min_ratio=self.depth_min_ratio)
            point = None if result.distance is None else self.point_in_base(target, result.distance, rgb)
            return result, point

        stamp = rgb.header.stamp.sec + rgb.header.stamp.nanosec * 1e-9
        if auto:
            # A vertically clipped box can put its centre on the floor/background.
            margin = self.auto_vertical_margin
            candidates = [d for d in detections if d['box'][1] >= margin
                          and d['box'][3] <= rgb.height - margin] if margin else detections
            measurement, point = self.select_automatic(candidates, measure, stamp)
        else:
            target = next((d for d in detections if d['id'] == self.target_id), None)
            measurement, point = measure(target)
        distance = None if measurement is None else measurement.distance
        state = ('no_selection' if self.target_id is None else
                 'target_missing' if measurement is None else
                 measurement.reason if distance is None else 'following')
        with self.lock:
            if self.closed:
                return
            twist = Twist()
            if self.fresh(packet):
                twist.linear.x, twist.angular.z = approach_command(self.controller, point, self.max_angular)
                if point is not None and point[0] <= 0:
                    state = 'target_behind_base'
                elif point is not None and self.controller.stopped:
                    state = 'distance_hold'
            else:
                self.controller.reset()
                state = 'stale_after_inference'
            self.pub.publish(twist)
            self.last_command_time = received
        for det in detections:
            x1, y1, x2, y2 = map(int, det['box'])
            color = (0, 0, 255) if det['id'] == self.target_id else (0, 255, 0)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, f"ID:{det['id']} {self.model.names[det['cls']]} {det['confidence']:.2f}",
                        (x1, max(15, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        processing_ms = (time.monotonic() - started) * 1000
        image_age_ms = (self.get_clock().now().nanoseconds * 1e-9 - stamp) * 1000
        status = 'FOLLOWING' if state == 'following' else f'STOP: {state}'
        if state not in ('following', 'no_selection', 'distance_hold'):
            detail = '' if measurement is None else (
                f'; depth={measurement.valid_pixels}/{measurement.roi_pixels} '
                f'({measurement.valid_ratio:.0%}, need {self.depth_min_ratio:.0%})')
            self.report_stop(state + detail)
        if point is not None:
            status += f' | XY: {point[0]:.2f}, {point[1]:.2f} m'
        if self.input_mode:
            status += ' | INPUT ID: ' + self.input_buffer
        cv2.putText(frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        quality = '' if measurement is None else (
            f' | depth {measurement.valid_pixels}/{measurement.roi_pixels} '
            f'({measurement.valid_ratio:.0%})')
        cv2.putText(frame, f'age {image_age_ms:.0f} ms | process {processing_ms:.0f} ms' + quality,
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
        preview = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        preview.header = rgb.header
        self.preview_pub.publish(preview)
        self.targets_pub.publish(String(data=json.dumps({
            'targets': [dict(id=d['id'], cx=d['cx'], cy=d['cy'],
                             confidence=d['confidence'], box=[float(v) for v in d['box']])
                        for d in detections],
            'selected_id': self.target_id, 'selected_depth_m': distance,
            'selected_point_base_m': None if point is None else [float(v) for v in point],
            'control_state': state, 'image_age_ms': image_age_ms, 'processing_ms': processing_ms,
            'image_stamp': stamp, 'command': [twist.linear.x, twist.angular.z],
            'depth_valid_pixels': None if measurement is None else measurement.valid_pixels,
            'depth_roi_pixels': None if measurement is None else measurement.roi_pixels,
            'depth_valid_ratio': None if measurement is None else measurement.valid_ratio,
        }, allow_nan=False)))
        if self.show_window:
            cv2.imshow('Scout visual follow', frame)

    def run(self):
        if self.show_window:
            cv2.namedWindow('Scout visual follow')
        try:
            while rclpy.ok() and not self.closed:
                rclpy.spin_once(self, timeout_sec=0.01)
                if self.show_window:
                    self.select_key()
                with self.lock:
                    packet, self.pending = self.pending, None
                if packet is not None:
                    try:
                        self.process(packet)
                    except Exception as exc:
                        self.stop()
                        if time.monotonic() - self.last_error > 2.0:
                            self.get_logger().error(f'Visual follow stopped: {type(exc).__name__}: {exc}')
                            self.last_error = time.monotonic()
        finally:
            with self.lock:
                self.closed = True
                self.done.set()
                if rclpy.ok():
                    self.stop()
            self.watchdog.join(timeout=1.0)
            if self.show_window:
                cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VisualFollower()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
