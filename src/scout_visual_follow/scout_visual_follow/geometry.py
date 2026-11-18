"""Projection from optical depth to a stationary base frame, in metres."""
import math
import numpy as np


def transform_point(point, translation, quaternion):
    point = np.asarray(point, dtype=float)
    shift = np.asarray(translation, dtype=float)
    quat = np.asarray(quaternion, dtype=float)
    if point.shape != (3,) or shift.shape != (3,) or quat.shape != (4,) or not all(
            np.all(np.isfinite(v)) for v in (point, shift, quat)):
        raise ValueError('Invalid point or camera transform')
    norm = np.linalg.norm(quat)
    if norm < 1e-9:
        raise ValueError('Invalid zero quaternion')
    quat /= norm
    vector, scalar = quat[:3], quat[3]
    return point + 2 * np.cross(vector, np.cross(vector, point) + scalar * point) + shift


def approach_command(controller, point, max_angular=0.35):
    if point is None or not np.all(np.isfinite(point)) or point[0] <= 0:
        controller.reset()
        return 0.0, 0.0
    distance = math.hypot(float(point[0]), float(point[1]))
    speed = controller.speed(distance)
    if controller.stopped:
        return 0.0, 0.0
    yaw = math.atan2(float(point[1]), float(point[0]))
    angular = max(-max_angular, min(max_angular, 0.8 * yaw))
    # Turn towards an off-axis target before moving forwards.
    return speed * max(0.0, 1.0 - abs(yaw) / 0.6), angular
