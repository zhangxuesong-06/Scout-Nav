from glob import glob
from setuptools import find_packages, setup

setup(
    name='scout_visual_follow', version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/scout_visual_follow']),
        ('share/scout_visual_follow', ['package.xml']),
        ('share/scout_visual_follow/launch', glob('launch/*.launch.py')),
        ('share/scout_visual_follow/models', glob('models/*.pt')),
    ],
    install_requires=['setuptools', 'numpy'], zip_safe=True,
    maintainer='Scout project maintainers', maintainer_email='maintainers@example.invalid',
    description='ROS 2 visual target following port', license='Apache-2.0',
    entry_points={'console_scripts': ['visual_follow = scout_visual_follow.node:main']},
)
