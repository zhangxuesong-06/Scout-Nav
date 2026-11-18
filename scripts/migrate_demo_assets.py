#!/usr/bin/env python3
"""Copy local assets and convert Classic world materials, without network access.

The source snapshot is kept outside src so colcon cannot discover ROS 1 packages.
Run once for a new workspace; refuses to overwrite existing imported assets.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import re
import shutil
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_world(path):
    text = path.read_text()
    try:
        return ET.fromstring(text), None
    except ET.ParseError:
        if path.name != 'orchard1.world':
            raise
        # The original file contains five complete models but lacks </world>
        # and an <sdf> wrapper. No model or placement is invented.
        return ET.fromstring('<sdf version="1.8">' + text + '</world></sdf>'), 'added sdf wrapper and closing world tag'


def bake_saved_model_poses(world):
    """Restore the saved Classic layout before dropping its runtime state.

    Top-level saved model poses are world-relative. Saved link poses must not
    replace model-relative link/visual/collision transforms. Only update models
    still defined in the world: old snapshots can contain already deleted props.
    """
    states = world.findall('state')
    if not states:
        return None
    state = max(enumerate(states), key=lambda pair: (
        tuple(int(v) for v in pair[1].findtext('sim_time', '-1 0').split()), pair[0]))[1]
    targets = {model.get('name'): model for model in world.findall('model')}
    targets.update({include.findtext('name'): include for include in world.findall('include')
                    if include.findtext('name')})
    restored, unmatched = [], []
    for saved in state.findall('model'):
        name, pose = saved.get('name'), saved.find('pose')
        if name not in targets:
            unmatched.append(name)
            continue
        if pose is None:
            continue
        target = targets[name]
        old_pose = target.find('pose')
        index = list(target).index(old_pose) if old_pose is not None else 0
        if old_pose is not None:
            target.remove(old_pose)
        target.insert(index, copy.deepcopy(pose))
        restored.append(name)
    for snapshot in states:
        world.remove(snapshot)
    return {'sim_time': state.findtext('sim_time'), 'model_poses': len(restored),
            'unmatched_state_models': unmatched}


def saved_layout_note(result):
    return (f'baked {result["model_poses"]} existing model/include poses from Classic saved state '
            f'at sim_time {result["sim_time"]}; removed runtime snapshot')


def material_blocks(text):
    for match in re.finditer(r'(?m)^\s*material\s+(\S+)[^\n{]*\s*\{', text):
        start = match.end()
        level, end = 1, start
        while level and end < len(text):
            level += (text[end] == '{') - (text[end] == '}')
            end += 1
        yield match.group(1), text[start:end-1]


def convert_materials(root, materials):
    changes = 0
    colors = {'Grey': '.5 .5 .5 1', 'Gray': '.5 .5 .5 1',
              'White': '1 1 1 1', 'Black': '.04 .04 .04 1',
              'Red': '.8 .1 .1 1', 'Green': '.1 .6 .1 1',
              'Blue': '.1 .1 .8 1', 'Yellow': '.9 .8 .1 1',
              'Orange': '1 .4 .05 1', 'DarkGrey': '.2 .2 .2 1'}
    for material in root.findall('.//material'):
        script = material.find('script')
        if script is None:
            continue
        name = script.findtext('name', '')
        spec = materials.get(name, {})
        for tag in ('ambient', 'diffuse', 'specular'):
            if material.find(tag) is None:
                value = spec.get(tag)
                if value:
                    ET.SubElement(material, tag).text = value
                elif tag in ('ambient', 'diffuse'):
                    ET.SubElement(material, tag).text = colors.get(name.split('/')[-1], '.65 .65 .65 1')
        if spec.get('texture') and material.find('pbr') is None:
            metal = ET.SubElement(ET.SubElement(material, 'pbr'), 'metal')
            ET.SubElement(metal, 'albedo_map').text = spec['texture']
            ET.SubElement(metal, 'metalness').text = '0'
            ET.SubElement(metal, 'roughness').text = '0.8'
        material.remove(script)
        changes += 1
    return changes


def write_xml(root, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    if root.tag.startswith('{'):
        # Ignition's ColladaLoader expects unprefixed COLLADA element names.
        ET.register_namespace('', root.tag[1:].split('}', 1)[0])
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--demo', type=Path, default=Path.home() / 'demo')
    parser.add_argument('--models', type=Path, default=Path.home() / '.gazebo/models')
    parser.add_argument('--resume', action='store_true', help='Resume a partial import created by this script')
    args = parser.parse_args()
    source = args.demo / 'src/scout'
    package = ROOT / 'src/scout_sim'
    reference = ROOT / 'reference/demo_scout'
    for dest in (reference, package / 'models', package / 'worlds', package / 'meshes'):
        if dest.exists() and not args.resume:
            raise SystemExit(f'Refusing to overwrite imported assets: {dest}')
    paths = sorted((source / 'scout_gazebo_sim/worlds').rglob('*.world'))
    parsed = [(p, *source_world(p)) for p in paths]
    # Recursive model:// closure. Worlds often inline models while still referring
    # to their mesh and material resources by model URI.
    names = {'person_standing'}
    def dependencies(root):
        for el in root.iter():
            if el.text and el.text.strip().startswith('model://'):
                names.add(el.text.strip().split('/')[2])
    for _, root, _ in parsed:
        dependencies(root)
    scanned = set()
    while names - scanned:
        for name in sorted(names - scanned):
            folder = args.models / name
            if not folder.is_dir():
                raise FileNotFoundError(f'Missing local dependency: {folder}')
            for sdf in folder.rglob('*.sdf'):
                dependencies(ET.parse(sdf).getroot())
            scanned.add(name)
    shutil.copytree(source, reference, dirs_exist_ok=args.resume)
    (reference.parent / 'COLCON_IGNORE').touch()
    piper = args.demo / 'src/piper/piper_description'
    piper_ref = reference.parent / 'piper_description_source'
    piper_ref.mkdir(exist_ok=args.resume)
    shutil.copy2(piper / 'piper_description_gazebo.xacro', piper_ref)
    shutil.copy2(piper / 'package.xml', piper_ref)
    for license_file in piper.glob('*LICENSE*'):
        shutil.copy2(license_file, piper_ref)
    manifest = {'source': str(source), 'model_source': str(args.models),
                'source_sha256': {}, 'models': sorted(names), 'worlds': {}, 'missing_source_textures': []}
    for file in reference.rglob('*'):
        if file.is_file():
            manifest['source_sha256'][str(file.relative_to(ROOT))] = sha(file)
    materials = {}
    for name in sorted(names):
        original = args.models / name
        dest = package / 'models' / name
        shutil.copytree(original, dest, dirs_exist_ok=args.resume)
        if name == 'tomato_fruit':
            # The standalone local model lacks mesh resources referenced by the
            # older nested model SDFs. The identical dependency is present here.
            nested = args.models / 'tomato_0/tomato_fruit'
            for folder in ('meshes', 'materials'):
                shutil.copytree(nested / folder, dest / folder, dirs_exist_ok=True)
        for file in original.rglob('*'):
            if file.is_file():
                manifest['source_sha256']['gazebo_models/' + str(file.relative_to(args.models))] = sha(file)
        for script in dest.rglob('*.material'):
            for material_name, block in material_blocks(script.read_text(errors='replace')):
                spec = {}
                for tag in ('ambient', 'diffuse', 'specular'):
                    match = re.search(r'(?m)^\s*' + tag + r'\s+([\d.eE+\- ]+)', block)
                    if match:
                        numbers = match.group(1).split()
                        spec[tag] = ' '.join((numbers + ['1'])[:4])
                match = re.search(r'(?m)^\s*texture\s+(\S+)', block)
                if match:
                    choices = list(dest.rglob(Path(match.group(1)).name))
                    if not choices:
                        manifest['missing_source_textures'].append(f'{name}: {material_name}: {match.group(1)}')
                    else:
                        spec['texture'] = 'model://' + str(choices[0].relative_to(package / 'models'))
                materials[material_name] = spec
    for sdf in sorted((package / 'models').rglob('*.sdf')):
        root = ET.parse(sdf).getroot()
        # New pbr material nodes require SDF 1.7+.
        root.set('version', '1.8')
        convert_materials(root, materials)
        write_xml(root, sdf)
    # Classic searched sibling texture folders for bare DAE image names.
    # Use explicit relative paths so Fortress does not depend on that fallback.
    for dae in (package / 'models').rglob('*.dae'):
        root = ET.parse(dae).getroot()
        changed = False
        model_dir = package / 'models' / dae.relative_to(package / 'models').parts[0]
        for image in root.iter():
            if not image.tag.endswith('}image'):
                continue
            for element in image:
                if not element.tag.endswith('}init_from') or not element.text:
                    continue
                value = element.text.strip()
                if (dae.parent / value).is_file():
                    continue
                matches = list(model_dir.rglob(Path(value).name))
                if not matches:
                    raise FileNotFoundError(f'Missing DAE texture in source: {dae}: {value}')
                import os
                element.text = os.path.relpath(matches[0], dae.parent)
                changed = True
        if changed:
            write_xml(root, dae)
    for path, root, repair in parsed:
        world = root.find('world')
        root.set('version', '1.8')
        world.set('name', 'scout_world')
        notes = []
        if repair:
            notes.append(repair)
        saved_layout = bake_saved_model_poses(world)
        if saved_layout:
            notes.append(saved_layout_note(saved_layout))
        used_names = set()
        for model in world.findall('model'):
            original_name = model.get('name')
            unique = original_name
            index = 1
            while unique in used_names:
                unique = f'{original_name}_{index}'
                index += 1
            if unique != original_name:
                model.set('name', unique)
                notes.append(f'renamed duplicate model {original_name} -> {unique}')
            used_names.add(unique)
        # Classic actor collision plugin is not a Fortress system; use explicit
        # static people at the first waypoint, retaining source trajectories above.
        actors = list(world.findall('actor'))
        for actor in actors:
            pose = actor.findtext('script/trajectory/waypoint/pose', '0 0 0 0 0 0').split()
            pose[2] = '0'
            include = ET.SubElement(world, 'include')
            ET.SubElement(include, 'uri').text = 'model://person_standing'
            ET.SubElement(include, 'name').text = actor.get('name') + '_static'
            ET.SubElement(include, 'pose').text = ' '.join(pose)
            ET.SubElement(include, 'static').text = 'true'
            world.remove(actor)
        if actors:
            notes.append(f'{len(actors)} actors replaced by static people at first waypoints; no walking or skeletal collisions')
        # No old plugin (including actor collision plugins) may be loaded.
        for parent in list(root.iter()):
            for plugin in list(parent.findall('plugin')):
                parent.remove(plugin)
        for physics in list(world.findall('physics')):
            world.remove(physics)
        physics = ET.SubElement(world, 'physics', name='1ms', type='ignored')
        ET.SubElement(physics, 'max_step_size').text = '0.001'
        ET.SubElement(physics, 'real_time_factor').text = '1.0'
        for filename, classname in [('physics', 'Physics'), ('user-commands', 'UserCommands'),
                                    ('scene-broadcaster', 'SceneBroadcaster'), ('sensors', 'Sensors')]:
            plugin = ET.SubElement(world, 'plugin', filename=f'ignition-gazebo-{filename}-system',
                                   name=f'ignition::gazebo::systems::{classname}')
            if filename == 'sensors':
                ET.SubElement(plugin, 'render_engine').text = 'ogre2'
        changes = convert_materials(root, materials)
        key = str(path.relative_to(source / 'scout_gazebo_sim/worlds').with_suffix(''))
        relative = Path(key + ('_static.sdf' if actors else '.sdf'))
        write_xml(root, package / 'worlds' / relative)
        manifest['worlds'][key] = {'file': str(relative), 'source': str(path),
                                   'material_conversions': changes, 'notes': notes}
        if saved_layout:
            manifest['worlds'][key]['saved_layout'] = saved_layout
    meshes = package / 'meshes'
    meshes.mkdir(exist_ok=args.resume)
    for name in ('scout_mini_base_link2.dae', 'wheel.dae'):
        shutil.copy2(source / 'scout_description/meshes' / name, meshes / name)
    passive = ET.Element('robot')
    piper_root = ET.parse(piper / 'piper_description_gazebo.xacro').getroot()
    for child in piper_root:
        if child.tag not in ('link', 'joint'):
            continue
        child = copy.deepcopy(child)
        if child.tag == 'joint':
            child.set('type', 'fixed')
            for tag in ('axis', 'limit', 'dynamics', 'mimic', 'safety_controller'):
                for el in list(child.findall(tag)):
                    child.remove(el)
        for mesh in child.findall('.//mesh'):
            name = Path(mesh.get('filename')).name
            shutil.copy2(piper / 'meshes' / name, meshes / ('piper_' + name))
            mesh.set('filename', '$(arg mesh_root)/piper_' + name)
        # Visual-only passive arm avoids non-convex arm collision computation.
        for collision in list(child.findall('collision')):
            child.remove(collision)
        passive.append(child)
    write_xml(passive, package / 'urdf/piper_passive.urdf.xacro')
    (package / 'ASSET_MANIFEST.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + '\n')
    print(f'Imported {len(paths)} worlds, {len(names)} model dependencies; originals in {reference}')


if __name__ == '__main__':
    main()
