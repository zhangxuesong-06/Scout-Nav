"""Validate physical configuration; unknown measurements never become zeros."""
import argparse
import math
import os
from pathlib import Path

import yaml


def default_profile():
    return os.environ.get('SCOUT_PROFILE', str(Path.home() / 'scout/config/robot.yaml'))


def read_profile(path):
    with open(Path(path).expanduser(), encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or any(k not in data for k in ('robot', 'base', 'camera')):
        raise ValueError('Profile requires robot, base and camera sections')
    if data['robot'].get('model') != 'scout_mini_skid':
        raise ValueError('This workspace is configured for Scout Mini skid-steer')
    return data


def require_camera(data):
    camera = data['camera']
    if not str(camera.get('serial_number', '')).strip():
        raise ValueError('Set the FIXED rack camera serial_number in config/robot.yaml')
    if camera.get('name') != 'nav_camera':
        raise ValueError('Keep camera name nav_camera; frame/topic names depend on it')


def mount_values(data, allow_approximate=False):
    camera = data['camera']
    if camera.get('mount_locked') is not True:
        raise ValueError('Lock the camera hinge before using a static transform')
    approximate = allow_approximate and camera.get('extrinsics_approximate') is True
    if camera.get('extrinsics_confirmed') is not True and not approximate:
        raise ValueError('Measure xyz_m and rpy_deg, then set extrinsics_confirmed: true')
    values = []
    for key in ('xyz_m', 'rpy_deg'):
        items = camera.get(key)
        if not isinstance(items, list) or len(items) != 3:
            raise ValueError(f'{key} must contain three measured numbers')
        if any(isinstance(v, bool) or not isinstance(v, (float, int))
               or not math.isfinite(v) for v in items):
            raise ValueError(f'{key} contains unknown or invalid measurements')
        values.extend(float(v) for v in items)
    return values[:3], [math.radians(v) for v in values[3:]]


def require_base(data, require_up=True):
    base = data['base']
    port = base.get('can_interface', '')
    if base.get('can_interface_confirmed') is not True or not port:
        raise ValueError('Confirm the Scout-only CAN adapter in config/robot.yaml first')
    path = Path('/sys/class/net') / port
    if '/' in port or not path.is_dir():
        raise ValueError(f'Scout CAN interface does not exist: {port}')
    if (path / 'type').read_text().strip() != '280':  # ARPHRD_CAN
        raise ValueError(f'Not a CAN interface: {port}')
    expected_serial = str(base.get('can_adapter_serial', '')).strip()
    if expected_serial:
        serial_path = (path / 'device').resolve().parent / 'serial'
        if not serial_path.is_file() or serial_path.read_text().strip() != expected_serial:
            raise ValueError(f'{port} USB serial does not match the confirmed Scout adapter')
    flags = int((path / 'flags').read_text().strip(), 16)
    if require_up and not flags & 1:
        raise ValueError(f'{port} is DOWN; configure the correct Scout interface manually')
    return port


def require_footprint(data):
    robot = data['robot']
    points = robot.get('footprint')
    if robot.get('footprint_confirmed') is not True or not isinstance(points, list) or len(points) < 3:
        raise ValueError('Measure the complete robot footprint before starting Nav2')
    for point in points:
        if not isinstance(point, list) or len(point) != 2 or any(
                isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
                for v in point):
            raise ValueError('footprint must be a polygon of finite [x, y] points in metres')
    area2 = sum(a[0] * b[1] - b[0] * a[1]
                for a, b in zip(points, points[1:] + points[:1]))
    if abs(area2) < 0.01:
        raise ValueError('footprint polygon has zero or implausibly small area')
    return points


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--profile', default=default_profile())
    args = parser.parse_args()
    data = read_profile(args.profile)
    checks = [('fixed camera selection', require_camera), ('locked camera transform', mount_values),
              ('Scout CAN adapter', require_base), ('navigation footprint', require_footprint)]
    failed = False
    for label, check in checks:
        try:
            check(data)
            print(f'OK      {label}')
        except (ValueError, OSError) as exc:
            failed = True
            print(f'PENDING {label}: {exc}')
    raise SystemExit(1 if failed else 0)
