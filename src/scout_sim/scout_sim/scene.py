"""Assemble a local Fortress scene. No hardware nodes or network resources."""
import json
import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import xacro


def catalog(share):
    return json.loads((Path(share) / 'ASSET_MANIFEST.json').read_text())['worlds']


def mapping_course(share, with_grapes):
    """Small rectangular route with repeatable RGB-D features on all sides."""
    scene = ET.parse(share / 'worlds/weston_robot_empty.sdf').getroot()
    world = scene.find('world')
    ambient = ET.SubElement(world, 'scene')
    ET.SubElement(ambient, 'ambient').text = '0.65 0.65 0.65 1'
    ET.SubElement(ambient, 'shadows').text = 'false'

    def box(name, pose, size):
        model = ET.SubElement(world, 'model', name=name)
        ET.SubElement(model, 'static').text = 'true'
        ET.SubElement(model, 'pose').text = ' '.join(map(str, (*pose[:3], 0, 0, pose[3])))
        link = ET.SubElement(model, 'link', name='body')
        for tag in ('collision', 'visual'):
            part = ET.SubElement(link, tag, name=tag)
            geom = ET.SubElement(ET.SubElement(part, 'geometry'), 'box')
            ET.SubElement(geom, 'size').text = ' '.join(map(str, size))
        return link

    def photo(link, filename, x_rotation, offset, size):
        visual = ET.SubElement(link, 'visual', name='textured_face')
        ET.SubElement(visual, 'cast_shadows').text = 'false'
        ET.SubElement(visual, 'pose').text = f'0 {offset} 0 {x_rotation} 0 0'
        plane = ET.SubElement(ET.SubElement(visual, 'geometry'), 'plane')
        ET.SubElement(plane, 'normal').text = '0 0 1'
        ET.SubElement(plane, 'size').text = size
        material = ET.SubElement(visual, 'material')
        ET.SubElement(material, 'ambient').text = '0.85 0.85 0.85 1'
        ET.SubElement(material, 'diffuse').text = '0.85 0.85 0.85 1'
        ET.SubElement(material, 'emissive').text = '1 1 1 1'
        metal = ET.SubElement(ET.SubElement(material, 'pbr'), 'metal')
        uri = 'model://mapping_course/materials/textures/' + filename
        ET.SubElement(metal, 'albedo_map').text = uri
        ET.SubElement(metal, 'emissive_map').text = uri
        ET.SubElement(metal, 'metalness').text = '0'
        ET.SubElement(metal, 'roughness').text = '1'

    # Four distinct textured walls supply repeatable RGB-D features.
    for index, (x, y, yaw) in enumerate(((1.2, -1.45, 0), (4.0, .9, math.pi/2),
                                         (1.2, 3.25, math.pi), (-1.6, .9, -math.pi/2))):
        wall = box(f'wall_{index}', (x, y, .7, yaw), (5.7, .1, 1.4))
        wall.remove(wall.find('visual'))
        photo(wall, f'wall_{index}.png', -math.pi/2, .055, '5.7 1.4')
    box('island', (1.2, .9, .22, 0), (.55, .36, .44))
    if with_grapes:
        fixture = box('grape_fixture', (2.42, 2.95, .68, 0), (.37, .055, .61))
        fixture.remove(fixture.find('visual'))
        photo(fixture, 'grape_fixture.png', math.pi/2, -.029, '.37 .61')
    return scene


def prepare_scene(share, destination, world='clearpath_playpen', camera_pitch_deg=0.0,
                  x=0.0, y=0.0, yaw=0.0, camera=True, static_environment=False,
                  mapping_grapes=False):
    share, destination = Path(share), Path(destination)
    worlds = catalog(share)
    if world != 'mapping_course' and world not in worlds:
        raise ValueError(f'Unknown world {world!r}. Available: {", ".join(worlds)}')
    values = [float(v) for v in (camera_pitch_deg, x, y, yaw)]
    if not all(math.isfinite(v) for v in values) or abs(values[0]) > 80:
        raise ValueError('Spawn pose must be finite and camera pitch within ±80 degrees')
    urdf = xacro.process_file(str(share / 'urdf/scout_mini.urdf.xacro'), mappings={
        'mesh_root': str(share / 'meshes'), 'camera_pitch': str(math.radians(values[0]))
    }).toxml()
    destination.mkdir(parents=True, exist_ok=True)
    urdf_path = destination / 'scout.urdf'
    urdf_path.write_text(urdf)
    converted = subprocess.run(['ign', 'sdf', '-p', str(urdf_path)],
                               capture_output=True, text=True, check=True, timeout=30)
    model = ET.fromstring(converted.stdout).find('model')
    if model is None or len(model.findall('joint')) != 4:
        raise ValueError('URDF conversion must retain exactly four driven wheel joints')
    ET.SubElement(model, 'pose').text = f'{values[1]} {values[2]} 0.08 0 0 {values[3]}'
    scene = (mapping_course(share, mapping_grapes) if world == 'mapping_course'
             else ET.parse(share / 'worlds' / worlds[world]['file']).getroot())
    target = scene.find('world')
    if static_environment:
        # Navigation uses a fixed environment. Legacy movable props have very
        # stiff mesh/contact settings tuned for Classic ODE, not Fortress DART.
        for prop in target.findall('model'):
            static = prop.find('static')
            if static is None:
                static = ET.SubElement(prop, 'static')
            static.text = 'true'
    if not camera:
        for link in model.findall('link'):
            for sensor in list(link.findall('sensor')):
                link.remove(sensor)
        for plugin in list(target.findall('plugin')):
            if plugin.get('name', '').endswith('::Sensors'):
                target.remove(plugin)
    target.append(model)
    ET.indent(scene, space='  ')
    path = destination / 'scene.sdf'
    ET.ElementTree(scene).write(path, encoding='utf-8', xml_declaration=True)
    return path, urdf, worlds.get(world, {}).get('notes', [])
