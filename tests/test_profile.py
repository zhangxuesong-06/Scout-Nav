from copy import deepcopy
import math
from pathlib import Path
import pytest

from scout_nav_bringup.profile import mount_values, read_profile, require_camera, require_footprint


def profile():
    return read_profile(Path(__file__).parents[1] / 'config/robot.yaml')


def measured():
    data = profile()
    data['camera'].update(extrinsics_confirmed=True, xyz_m=[0.2, 0.0, 0.6], rpy_deg=[0, 20, 0])
    return data


def test_unknown_mount_is_rejected():
    with pytest.raises(ValueError, match='Measure'):
        mount_values(profile())


def test_locked_tilt_is_supported_and_converted_to_radians():
    xyz, rpy = mount_values(measured())
    assert xyz == [0.2, 0.0, 0.6]
    assert rpy == pytest.approx([0, math.radians(20), 0])


@pytest.mark.parametrize('value', [None, True, float('nan'), float('inf')])
def test_unknown_or_invalid_mount_numbers_are_not_zeroed(value):
    data = measured()
    data['camera']['rpy_deg'][1] = value
    with pytest.raises(ValueError):
        mount_values(data)


def test_unlocked_hinge_rejected():
    data = measured()
    data['camera']['mount_locked'] = False
    with pytest.raises(ValueError, match='hinge'):
        mount_values(data)


def test_unknown_serial_is_rejected():
    data = profile()
    data['camera']['serial_number'] = ''
    with pytest.raises(ValueError, match='FIXED'):
        require_camera(data)


def test_approximate_mount_requires_explicit_opt_in():
    data = profile()
    data['camera'].update(extrinsics_confirmed=False, extrinsics_approximate=True,
                          xyz_m=[.2, 0., .710998], rpy_deg=[0., 0., 0.])
    with pytest.raises(ValueError, match='Measure'):
        mount_values(data)
    assert mount_values(data, allow_approximate=True) == ([.2, 0., .710998], [0., 0., 0.])
    data['camera']['xyz_m'][0] = None
    with pytest.raises(ValueError, match='unknown'):
        mount_values(data, allow_approximate=True)


def test_no_navigation_with_unmeasured_footprint():
    with pytest.raises(ValueError, match='footprint'):
        require_footprint(profile())


def test_complete_robot_footprint():
    data = deepcopy(profile())
    data['robot'].update(footprint_confirmed=True,
                         footprint=[[0.4, 0.3], [-0.4, 0.3], [-0.4, -0.3], [0.4, -0.3]])
    assert len(require_footprint(data)) == 4
    data['robot']['footprint'] = [[0, 0], [0, 0], [0, 0]]
    with pytest.raises(ValueError):
        require_footprint(data)
