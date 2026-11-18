"""Assemble a local Fortress scene. No hardware nodes or network resources."""
import json
import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import xacro


def catalog(share):
    return json.loads((Path(share) / 'ASSET_MANIFEST.json').read_text())['worlds']


def prepare_scene(share, destination, world='clearpath_playpen', camera_pitch_deg=0.0,
                  x=0.0, y=0.0, yaw=0.0, camera=True, static_environment=False):
    share, destination = Path(share), Path(destination)
    worlds = catalog(share)
    if world not in worlds:
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
    scene = ET.parse(share / 'worlds' / worlds[world]['file']).getroot()
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
    return path, urdf, worlds[world]['notes']
