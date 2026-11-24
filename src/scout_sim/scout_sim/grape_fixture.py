"""Add or remove named grape photo targets in a running simulation."""
import argparse
import json
import math
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

from .mapping_mission import simulation_domain
from .scene import mapping_course


def manipulate(action, name, x=0., y=0., yaw=0.):
    if not simulation_domain() or not re.fullmatch(r'grape_[A-Za-z0-9_]+', name):
        raise ValueError('Use the simulation environment and a name starting with grape_')
    if action == 'add':
        if not all(map(math.isfinite, (x, y, yaw))):
            raise ValueError('Fixture coordinates must be finite')
        share = Path(get_package_share_directory('scout_sim'))
        model = mapping_course(share, True).find('world/model[@name="grape_fixture"]')
        model.set('name', name)
        model.find('pose').text = f'{x} {y} .68 0 0 {yaw}'
        sdf = ET.Element('sdf', version='1.8')
        sdf.append(model)
        service, kind = 'create', 'EntityFactory'
        request = 'sdf: ' + json.dumps(ET.tostring(sdf, encoding='unicode'))
    elif action == 'remove':
        service, kind = 'remove', 'Entity'
        request = f'name: "{name}" type: MODEL'
    else:
        raise ValueError('Expected add or remove')
    response = subprocess.run(['ign', 'service', '-s', f'/world/scout_world/{service}',
                               '--reqtype', f'ignition.msgs.{kind}', '--reptype', 'ignition.msgs.Boolean',
                               '--timeout', '4000', '--req', request],
                              capture_output=True, text=True, timeout=8, check=True)
    if 'data: true' not in response.stdout:
        raise RuntimeError(response.stdout.strip() or response.stderr.strip() or 'Gazebo rejected request')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='action', required=True)
    add = commands.add_parser('add', help='x/y in metres, yaw in radians')
    add.add_argument('name')
    for key in ('x', 'y', 'yaw'):
        add.add_argument(key, type=float)
    commands.add_parser('remove').add_argument('name')
    args = vars(parser.parse_args())
    try:
        manipulate(**args)
    except (ValueError, RuntimeError, subprocess.SubprocessError) as exc:
        parser.exit(1, f'{exc}\n')
    print(f"{args['action']}: {args['name']}")


if __name__ == '__main__':
    main()
