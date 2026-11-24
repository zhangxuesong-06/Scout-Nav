"""Inspect the real following launch without starting hardware."""
import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
import pytest


ROOT = Path(__file__).parents[1]


def load_launch(monkeypatch, package, entry):
    spec = importlib.util.spec_from_file_location(entry, ROOT / f'src/{package}/launch/{entry}.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda package: f'/fake/{package}')
    ctx = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(ctx)
    return module, ctx


def load_follow(monkeypatch):
    module, ctx = load_launch(monkeypatch, 'scout_visual_follow', 'follow')
    monkeypatch.setattr(module, 'require_base', lambda profile: 'can_test')
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
    assert [preview[key] for key in ('stop_distance', 'resume_distance', 'linear_gain')] == ['0.5', '1.0', '1.0']
    assert any(a.get('start_hardware') == 'true' for a in args)
    gate = next(a for a in args if a.get('mode') == 'follow')
    assert gate['auto_enable'] == 'false'
    assert gate['max_linear_speed'] == '0.05' and gate['max_angular_speed'] == '0.15'


def test_follow_overrides_reach_preview_node_as_floats(monkeypatch):
    module, ctx = load_follow(monkeypatch)
    ctx.launch_configurations.update(start_hardware='true', allow_approximate_mount='true',
                                     stop_distance='0.8', resume_distance='1.2', linear_gain='0.3')
    preview_args = next(dict(a.launch_arguments) for a in module.start(ctx)
                        if isinstance(a, IncludeLaunchDescription) and 'target_id' in dict(a.launch_arguments))
    preview, preview_ctx = load_launch(monkeypatch, 'scout_visual_follow', 'preview')
    preview_ctx.launch_configurations.update(preview_args)
    monkeypatch.setattr(preview, 'Node', dict)
    params = preview.start(preview_ctx)[0]['parameters'][0]
    assert [params[key] for key in ('stop_distance', 'resume_distance', 'linear_gain')] == [0.8, 1.2, 0.3]
    assert params['max_linear_speed'] == 0.05 and params['max_angular_speed'] == 0.15


def test_follow_rejects_invalid_values_before_hardware_checks(monkeypatch):
    module, ctx = load_follow(monkeypatch)
    ctx.launch_configurations.update(start_hardware='true', profile='/nonexistent', linear_gain='inf')
    with pytest.raises(ValueError, match='linear_gain must be a finite number'):
        module.start(ctx)


def test_mapping_has_own_distances_and_forwards_approach_to_vision(monkeypatch, tmp_path):
    module, ctx = load_launch(monkeypatch, 'scout_sim', 'mapping_mission')
    monkeypatch.setenv('ROS_DOMAIN_ID', '42')
    monkeypatch.setattr(module, 'Node', dict)
    dummy = tmp_path / 'exists'
    dummy.touch()
    ctx.launch_configurations.update(database=str(tmp_path / 'map.db'), model_path=str(dummy),
                                     python_executable=str(dummy))

    def visual_params():
        return next(action['parameters'][0] for action in module.start(ctx)
                    if isinstance(action, dict) and action['package'] == 'scout_visual_follow')

    keys = ('stop_distance', 'resume_distance', 'linear_gain')
    assert [visual_params()[key] for key in keys] == [1.6, 1.8, 1.0]
    ctx.launch_configurations.update(stop_distance='1.5', resume_distance='2', linear_gain='0.4')
    params = visual_params()
    assert [params[key] for key in keys] == [1.5, 2.0, 0.4]
    ctx.launch_configurations['resume_distance'] = '4.0'
    with pytest.raises(ValueError, match='resume_distance < max_depth'):
        module.start(ctx)
