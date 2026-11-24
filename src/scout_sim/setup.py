from pathlib import Path
from setuptools import find_packages, setup

data = [('share/ament_index/resource_index/packages', ['resource/scout_sim']),
        ('share/scout_sim', ['package.xml', 'ASSET_MANIFEST.json', 'LICENSE'])]
for folder in ('launch', 'urdf', 'meshes', 'models', 'worlds', 'config'):
    for parent in sorted({p.parent for p in Path(folder).rglob('*') if p.is_file()}):
        data.append(('share/scout_sim/' + str(parent),
                     [str(p) for p in sorted(parent.iterdir()) if p.is_file()]))

setup(name='scout_sim', version='0.1.0', packages=find_packages(), data_files=data,
      install_requires=['setuptools'], zip_safe=True,
      maintainer='Scout project maintainers', maintainer_email='maintainers@example.invalid',
      description='Scout Mini Gazebo Fortress simulation', license='BSD-3-Clause',
      entry_points={'console_scripts': ['command_guard = scout_sim.command_guard:main',
                                        'mapping_mission = scout_sim.mapping_mission:main']})
