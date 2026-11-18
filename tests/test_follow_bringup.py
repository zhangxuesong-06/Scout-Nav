"""Inspect the real following launch without starting hardware."""
import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
import pytest


ROOT = Path(__file__).parents[1]


def load_follow(monkeypatch):
    spec = importlib.util.spec_from_file_location(
        'follow_bringup', ROOT / 'src/scout_visual_follow/launch/follow.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda package: f'/fake/{package}')
    monkeypatch.setattr(module, 'require_base', lambda profile: 'can_test')
    ctx = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(ctx)
    return module, ctx


def test_combined_follow_needs_explicit_hardware_start(monkeypatch):
    module, ctx = load_follow(monkeypatch)
    with pytest.raises(ValueError, match='start_hardware'):
        module.start(ctx)


def test_approximate_follow_brings_up_all_five_components_with_low_limits(monkeypatch):
    module, ctx = load_follow(monkeypatch)
    ctx.launch_configurations.update(start_hardware='true', allow_approximate_mount='true')
    actions = module.start(ctx)
    includes = [a for a in actions if isinstance(a, IncludeLaunchDescription)]
    assert len(includes) == 5
    args = [dict(a.launch_arguments) for a in includes]
    camera = next(a for a in args if 'soft_filter_speckle_size' in a)
    assert camera['soft_filter_speckle_size'] == '20'
    preview = next(a for a in args if 'depth_min_valid_ratio' in a)
    assert preview['depth_min_valid_pixels'] == '20'
    assert preview['depth_min_valid_ratio'] == '0.15'
    assert any(a.get('start_hardware') == 'true' for a in args)
    gate = next(a for a in args if a.get('mode') == 'follow')
    assert gate['auto_enable'] == 'false'
    assert gate['max_linear_speed'] == '0.05' and gate['max_angular_speed'] == '0.15'
