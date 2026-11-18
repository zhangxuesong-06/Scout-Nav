#!/usr/bin/env python3
"""ROS-independent depth estimation and following policy (distances in metres)."""
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class DepthMeasurement:
    distance: Optional[float] = None
    valid_pixels: int = 0
    roi_pixels: int = 0
    reason: str = ''

    @property
    def valid_ratio(self):
        return self.valid_pixels / self.roi_pixels if self.roi_pixels else 0.0


def measure_target_depth(depth, encoding, box, image_shape, min_depth=0.15,
                         max_depth=10.0, min_pixels=20, min_ratio=0.3):
    if min_pixels < 1 or not 0 < min_ratio <= 1:
        raise ValueError('Require min_pixels >= 1 and 0 < min_ratio <= 1')
    # Pixel lookup is valid only for registered, equally sized RGB/depth images.
    if depth.ndim != 2 or depth.shape != tuple(image_shape[:2]):
        return DepthMeasurement(reason='depth_size_mismatch')
    if encoding == '16UC1':
        scale = 0.001
    elif encoding == '32FC1':
        scale = 1.0
    else:
        return DepthMeasurement(reason='unsupported_depth_encoding')
    x1, y1, x2, y2 = map(float, box)
    if not np.all(np.isfinite([x1, y1, x2, y2])) or x2 <= x1 or y2 <= y1:
        return DepthMeasurement(reason='invalid_target_box')
    # Central 40% of the box reduces background contamination at its edges.
    h, w = depth.shape
    left = max(0, min(w, int(np.floor(x1 + 0.3 * (x2-x1)))))
    right = max(0, min(w, int(np.ceil(x2 - 0.3 * (x2-x1)))))
    top = max(0, min(h, int(np.floor(y1 + 0.3 * (y2-y1)))))
    bottom = max(0, min(h, int(np.ceil(y2 - 0.3 * (y2-y1)))))
    roi = depth[top:bottom, left:right].astype(np.float64) * scale
    valid = roi[np.isfinite(roi) & (roi >= min_depth) & (roi <= max_depth)]
    if valid.size < min_pixels:
        return DepthMeasurement(None, int(valid.size), int(roi.size), 'too_few_depth_pixels')
    if valid.size < roi.size * min_ratio:
        return DepthMeasurement(None, int(valid.size), int(roi.size), 'sparse_depth')
    return DepthMeasurement(float(np.median(valid)), int(valid.size), int(roi.size))


def target_distance(depth, encoding, box, image_shape, min_depth=0.15,
                    max_depth=10.0, min_pixels=20, min_ratio=0.3):
    return measure_target_depth(depth, encoding, box, image_shape, min_depth,
                                max_depth, min_pixels, min_ratio).distance


class DistanceController:
    def __init__(self, stop=1.0, resume=1.2, gain=0.3, max_speed=0.2):
        if not (0 < stop < resume and gain > 0 and max_speed > 0):
            raise ValueError('Require 0 < stop_distance < resume_distance and positive gains/speed')
        self.stop, self.resume = stop, resume
        self.gain, self.max_speed = gain, max_speed
        self.stopped = True

    def reset(self):
        self.stopped = True

    def speed(self, distance):
        if distance is None or not np.isfinite(distance) or distance <= 0:
            self.reset()
            return 0.0
        if distance <= self.stop:
            self.stopped = True
        elif distance >= self.resume:
            self.stopped = False
        if self.stopped:
            return 0.0
        return min(self.max_speed, self.gain * (distance - self.stop))
