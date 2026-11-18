from glob import glob
from setuptools import find_packages, setup

setup(
    name='scout_nav_bringup', version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/scout_nav_bringup']),
        ('share/scout_nav_bringup', ['package.xml']),
        ('share/scout_nav_bringup/launch', glob('launch/*.launch.py')),
        ('share/scout_nav_bringup/config', glob('config/*')),
    ],
    install_requires=['setuptools', 'PyYAML'], zip_safe=True,
    maintainer='Scout project maintainers', maintainer_email='maintainers@example.invalid',
    description='Scout Mini RGB-D navigation bringup', license='Apache-2.0',
    entry_points={'console_scripts': [
        'motion_gate = scout_nav_bringup.motion_gate:main',
        'check_profile = scout_nav_bringup.profile:main',
    ]},
)
