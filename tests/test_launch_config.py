"""Expand launch/configuration without spawning nodes or touching hardware."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

from launch import LaunchContext
from launch.actions import GroupAction, IncludeLaunchDescription, SetLaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.actions import Node, SetRemap
from launch_ros.utilities import evaluate_parameters
import pytest
import yaml


ROOT = Path(__file__).parents[1]


def load_launch(name):
    path = ROOT / 'src/scout_nav_bringup/launch' / f'{name}.launch.py'
    spec = importlib.util.spec_from_file_location('launch_' + name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def context(tmp_path, **values):
    data = yaml.safe_load((ROOT / 'config/robot.yaml').read_text())
    data['camera'].update(serial_number='FIXED_CAMERA_TEST_ONLY', extrinsics_confirmed=True,
                          xyz_m=[0.2, 0.0, 0.6], rpy_deg=[0, 20, 0])
    data['robot'].update(footprint_confirmed=True,
                         footprint=[[.4, .3], [-.4, .3], [-.4, -.3], [.4, -.3]])
    profile = tmp_path / 'profile.yaml'
    profile.write_text(yaml.safe_dump(data))
    ctx = LaunchContext()
    ctx.launch_configurations.update(profile=str(profile), **values)
    return ctx, data


def test_mapping_uses_3d_depth_and_only_one_map_to_odom_publisher(tmp_path, monkeypatch):
    module = load_launch('mapping')
    # RTAB-Map is not installed here yet. Inspect only our generated Node description.
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(tmp_path))
    ctx, data = context(tmp_path, database=str(tmp_path / 'new.db'), localization='false')
    node = module.start(ctx)[0]
    params = evaluate_parameters(ctx, node._Node__parameters)[0]
    assert params['Grid/Sensor'] == '1'
    assert params['Grid/3D'] == 'true'
    assert params['publish_tf'] is True
    assert params['frame_id'] == data['robot']['base_frame']
    assert params['odom_frame_id'] == ''  # Wheel odometry comes in via the odom message.
    assert params['subscribe_scan'] is False


def test_localization_never_silently_creates_a_new_map(tmp_path, monkeypatch):
    module = load_launch('mapping')
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(tmp_path))
    ctx, _ = context(tmp_path, database=str(tmp_path / 'missing.db'), localization='true')
    with pytest.raises(ValueError, match='existing'):
        module.start(ctx)


def test_base_launch_requires_explicit_hardware_start(tmp_path):
    ctx, _ = context(tmp_path, start_hardware='false')
    with pytest.raises(ValueError, match='start_hardware'):
        load_launch('base').start(ctx)


def test_nav2_expansion_preserves_smoother_and_routes_output_to_gate(tmp_path):
    module = load_launch('navigation')
    ctx, data = context(tmp_path)
    group = module.start(ctx)[0]
    children = group.get_sub_entities()
    remap = next(child for child in children if isinstance(child, SetRemap))
    remap.execute(ctx)
    include = next(child for child in children if isinstance(child, IncludeLaunchDescription))
    children = include.execute(ctx)
    for child in children:
        if isinstance(child, SetLaunchConfiguration):
            child.execute(ctx)
    nested = children[-1]
    nav_group = next(entity for entity in nested.entities if isinstance(entity, GroupAction))
    nodes = [entity for entity in nav_group.get_sub_entities() if isinstance(entity, Node)]
    controller = next(node for node in nodes if node.node_package == 'nav2_controller')
    smoother = next(node for node in nodes if node.node_package == 'nav2_velocity_smoother')
    controller._perform_substitutions(ctx)
    smoother._perform_substitutions(ctx)
    controller_rules = dict(reversed(controller.expanded_remapping_rules))
    smoother_rules = dict(reversed(smoother.expanded_remapping_rules))
    assert controller_rules['cmd_vel'] == 'cmd_vel_nav'
    assert smoother_rules['cmd_vel'] == 'cmd_vel_nav'
    assert smoother_rules['cmd_vel_smoothed'] == '/scout/navigation_cmd_vel'
    path = Path(ctx.launch_configurations['params_file'])
    params = yaml.safe_load(path.read_text())
    for name in ('local_costmap', 'global_costmap'):
        values = params[name][name]['ros__parameters']
        assert isinstance(values['footprint'], str)
        assert yaml.safe_load(values['footprint']) == data['robot']['footprint']
        assert values['obstacle_layer']['depth']['data_type'] == 'PointCloud2'
    assert 'amcl' not in params and 'map_server' not in params
    assert params['controller_server']['ros__parameters']['FollowPath']['max_vel_y'] == 0


def test_commissioning_tree_has_no_reverse_or_spin_recoveries():
    tree = ET.parse(ROOT / 'src/scout_nav_bringup/config/navigate_to_pose.xml')
    assert all(element.tag not in ('BackUp', 'Spin') for element in tree.iter())
