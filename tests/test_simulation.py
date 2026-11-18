"""Simulation contracts: preserve scenes and resources, and isolate controls."""
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import pytest
import yaml
from scout_sim.scene import catalog, prepare_scene

ROOT = Path(__file__).parents[1]
SHARE = ROOT / 'src/scout_sim'
MANIFEST = json.loads((SHARE / 'ASSET_MANIFEST.json').read_text())
_spec = importlib.util.spec_from_file_location('migrate_demo_assets', ROOT / 'scripts/migrate_demo_assets.py')
MIGRATION = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(MIGRATION)


def test_all_legacy_worlds_imported_with_originals_preserved():
    originals = ROOT / 'reference/demo_scout/scout_gazebo_sim/worlds'
    paths = {str(p.relative_to(originals).with_suffix('')) for p in originals.rglob('*.world')}
    assert paths == set(catalog(SHARE))
    assert (ROOT / 'reference/COLCON_IGNORE').exists()
    for name, entry in MANIFEST['worlds'].items():
        migrated = ET.parse(SHARE / 'worlds' / entry['file']).getroot().find('world')
        assert migrated is not None
        assert not migrated.findall('state')
        names = [el.get('name') for el in migrated.findall('model')]
        assert len(set(names)) == len(names)
        assert migrated.findall('plugin')
        assert not migrated.findall('.//actor')
        assert not any('libgazebo_ros' in str(p.attrib) or 'ActorCollisions' in str(p.attrib)
                       for p in migrated.findall('.//plugin'))
        if name == 'orchard1':
            assert len(names) == 5
            assert any('closing' in note for note in entry['notes'])


def test_static_world_model_geometry_and_placements_preserved():
    for name, entry in MANIFEST['worlds'].items():
        if name == 'orchard1':
            continue
        old = ET.parse(ROOT / 'reference/demo_scout/scout_gazebo_sim/worlds' / (name + '.world'))
        new = ET.parse(SHARE / 'worlds' / entry['file'])
        original = old.findall('./world/model')
        migrated = new.findall('./world/model')
        states = old.findall('./world/state')
        saved = {m.get('name'): m for m in states[-1].findall('model')} if states else {}
        assert len(original) == len(migrated)
        for a, b in zip(original, migrated):
            assert a.get('name') == b.get('name')
            effective = saved.get(a.get('name'), a)
            assert effective.findtext('pose', a.findtext('pose')) == b.findtext('pose'), (name, a.get('name'))
            # Saved link poses are world-relative; local geometry must stay local.
            assert [(p.tag, p.text) for child in a if child.tag != 'pose'
                    for p in child.iter('pose')] == [
                        (p.tag, p.text) for child in b if child.tag != 'pose'
                        for p in child.iter('pose')]
            # XML indentation may differ; compare actual geometry element values.
            def geometries(el):
                return [[(p.tag, (p.text or '').strip()) for p in geom.iter()]
                        for geom in el.findall('.//geometry')]
            assert geometries(a) == geometries(b)


def test_saved_layout_converter_uses_latest_state_and_preserves_local_frames():
    world = ET.fromstring('''<world name="old_world">
      <model name="barrier"><pose>0 0 0 0 0 0</pose>
        <link name="link"><pose>0.1 0.2 0.3 0 0 0.4</pose></link></model>
      <model name="unsaved"><pose>9 8 7 0 0 0.2</pose></model>
      <include><name>bin</name><uri>model://bin</uri></include>
      <state><sim_time>20 2</sim_time>
        <model name="barrier"><pose>1 2 3 0.1 -0.2 1.56</pose>
          <link name="link"><pose>99 98 97 1 2 3</pose></link></model>
        <model name="bin"><pose>4 5 6 0 0 2</pose></model>
        <model name="deleted_prop"><pose>0 0 0 0 0 0</pose></model></state>
      <state><sim_time>20 1</sim_time>
        <model name="barrier"><pose>0 0 0 0 0 0.5</pose></model></state>
    </world>''')
    result = MIGRATION.bake_saved_model_poses(world)
    assert result == {'sim_time': '20 2', 'model_poses': 2,
                      'unmatched_state_models': ['deleted_prop']}
    assert world.findtext('model[@name="barrier"]/pose') == '1 2 3 0.1 -0.2 1.56'
    assert world.findtext('model[@name="barrier"]/link/pose') == '0.1 0.2 0.3 0 0 0.4'
    assert world.findtext('model[@name="unsaved"]/pose') == '9 8 7 0 0 0.2'
    assert world.findtext('include/pose') == '4 5 6 0 0 2'
    assert world.find('model[@name="deleted_prop"]') is None
    assert not world.findall('state')
    unchanged = ET.tostring(world)
    assert MIGRATION.bake_saved_model_poses(world) is None
    assert ET.tostring(world) == unchanged


@pytest.mark.parametrize('model,yaw', [('jersey_barrier_10', 1.56),
                                        ('jersey_barrier_15', 2.36), ('Dumpster_0', 2.08003)])
def test_playpen_saved_rotations_survive_migration(model, yaw):
    world = ET.parse(SHARE / 'worlds/clearpath_playpen.sdf').find('world')
    pose = world.findtext(f'model[@name="{model}"]/pose')
    assert math.isclose(float(pose.split()[5]), yaw, abs_tol=1e-8)


def test_all_model_and_pbr_resources_resolve_offline():
    for directory in ('worlds', 'models'):
        for path in (SHARE / directory).rglob('*.sdf'):
            for element in ET.parse(path).getroot().iter():
                value = (element.text or '').strip()
                if value.startswith('model://'):
                    assert (SHARE / 'models' / value[8:]).exists(), (path, value)
                assert not value.startswith(('http://', 'https://', 'file://media/'))
    for dae in (SHARE / 'models').rglob('*.dae'):
        assert '<ns0:' not in dae.read_text(errors='replace')
        for element in ET.parse(dae).getroot().iter():
            if element.tag.endswith('}image'):
                for child in element:
                    if child.tag.endswith('}init_from'):
                        assert (dae.parent / child.text).is_file(), (dae, child.text)


def test_all_worlds_pass_native_sdf_validation():
    env = dict(os.environ, SDF_PATH=str(SHARE / 'models'))
    for entry in MANIFEST['worlds'].values():
        result = subprocess.run(['ign', 'sdf', '-k', str(SHARE / 'worlds' / entry['file'])],
                                capture_output=True, text=True, env=env, timeout=30)
        assert result.returncode == 0 and 'Valid.' in result.stdout, result.stderr


def test_tilted_camera_and_four_wheel_drivetrain(tmp_path):
    scene, urdf, _ = prepare_scene(SHARE, tmp_path, camera_pitch_deg=25)
    model = ET.parse(scene).find('.//model[@name="scout_mini"]')
    assert len(model.findall('joint')) == 4
    drive = next(p for p in model.findall('plugin') if p.get('name').endswith('::DiffDrive'))
    assert len(drive.findall('left_joint')) == len(drive.findall('right_joint')) == 2
    assert drive.findtext('topic') == '/scout_sim/drive_cmd'
    sensor = model.find('.//sensor')
    pose = [float(v) for v in sensor.findtext('pose').split()]
    assert math.isclose(pose[4], math.radians(25), abs_tol=1e-5)
    assert sensor.get('type') == 'rgbd_camera'
    robot = ET.fromstring(urdf)
    assert all(j.get('type') == 'fixed' for j in robot.findall('joint')
               if j.get('name', '').startswith('piper_'))
    optical = robot.find('joint[@name="nav_camera_optical_joint"]/origin')
    assert optical is not None
    assert not robot.findall('transmission')


def test_physics_only_mode_never_initializes_renderer(tmp_path):
    scene, _, _ = prepare_scene(SHARE, tmp_path, camera=False)
    world = ET.parse(scene).getroot().find('world')
    assert not world.findall('.//sensor')
    assert not any(p.get('name').endswith('::Sensors') for p in world.findall('plugin'))


def test_navigation_mode_freezes_legacy_props_but_not_robot(tmp_path):
    scene, _, _ = prepare_scene(SHARE, tmp_path, static_environment=True)
    models = ET.parse(scene).getroot().findall('./world/model')
    assert all(m.findtext('static') == 'true' for m in models if m.get('name') != 'scout_mini')
    scout = next(m for m in models if m.get('name') == 'scout_mini')
    assert scout.findtext('static', 'false') == 'false'
    saved = ET.parse(ROOT / 'reference/demo_scout/scout_gazebo_sim/worlds/clearpath_playpen.world')
    poses = {m.get('name'): m.findtext('pose') for m in saved.findall('./world/state/model')}
    assert all(m.findtext('pose') == poses[m.get('name')] for m in models if m.get('name') in poses)


@pytest.mark.parametrize('world,pitch', [('missing', 0), ('clearpath_playpen', float('nan')),
                                         ('clearpath_playpen', 90)])
def test_rejects_invalid_simulation_inputs(tmp_path, world, pitch):
    with pytest.raises(ValueError):
        prepare_scene(SHARE, tmp_path, world=world, camera_pitch_deg=pitch)


def test_bridges_never_publish_to_hardware_command_topic():
    core = yaml.safe_load((SHARE / 'config/bridge.yaml').read_text())
    outputs = [e['ros_topic_name'] for e in core if e['direction'] == 'ROS_TO_GZ']
    assert outputs == ['/scout_sim/drive_cmd']
    camera = yaml.safe_load((SHARE / 'config/camera_bridge.yaml').read_text())
    assert all(e['direction'] == 'GZ_TO_ROS' for e in camera)
    assert all(e['ros_topic_name'].startswith('/nav_camera/') for e in camera)
    cloud = yaml.safe_load((SHARE / 'config/cloud_bridge.yaml').read_text())
    assert cloud[0]['ros_topic_name'] == '/nav_camera/depth/points'
    assert cloud[0]['direction'] == 'GZ_TO_ROS'


def test_launch_rejects_default_robot_domain(monkeypatch):
    path = SHARE / 'launch/simulation.launch.py'
    spec = importlib.util.spec_from_file_location('sim_launch_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.delenv('ROS_DOMAIN_ID', raising=False)
    with pytest.raises(ValueError, match='isolated'):
        module.start(None)
