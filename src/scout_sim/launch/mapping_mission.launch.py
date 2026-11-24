"""One isolated Gazebo process tree for visual loop closure and grape following."""
from datetime import datetime
from pathlib import Path
import shlex

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from scout_sim.mapping_mission import simulation_domain
from scout_visual_follow.follow_parameters import DEFAULT_APPROACH, resolve_approach


def start(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    def flag(name):
        if arg(name) not in ('true', 'false'):
            raise ValueError(f'{name} must be true or false')
        return arg(name) == 'true'

    def include(package, launch_file, **values):
        path = Path(get_package_share_directory(package)) / 'launch' / launch_file
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                                        launch_arguments=values.items())

    if not simulation_domain():
        raise ValueError('Use sim_env.bash to isolate the simulation')
    approach = resolve_approach({name: arg(name) for name in DEFAULT_APPROACH})
    get_package_share_directory('rtabmap_slam')
    vision = flag('vision')
    database = Path(arg('database')).expanduser().resolve()
    if database.exists():
        raise ValueError(f'Use a new database for each mapping run: {database}')
    if vision:
        for name in ('python_executable', 'model_path'):
            if not Path(arg(name)).expanduser().is_file():
                raise ValueError(f'{name} not found: {arg(name)}')

    def stop_chain(event, ctx):
        if not ctx.is_shutdown:
            return [EmitEvent(event=Shutdown(reason='A simulation mission process exited'))]
        return []

    actions = [RegisterEventHandler(OnProcessExit(on_exit=stop_chain)),
               include('scout_sim', 'simulation.launch.py', world='mapping_course',
                       x='0', y='0', yaw='0', camera='true', rviz='false',
                       wheel_tf='false',
                       gui=arg('gui'), headless_rendering=arg('headless_rendering'),
                       software_rendering=arg('software_rendering'), mapping_grapes=arg('grapes')),
               include('scout_nav_bringup', 'mapping.launch.py', database=str(database),
                       simulation='true', localization='false', profile=arg('profile'),
                       simulation_odom_topic='/scout_sim/mapping_odom'),
               Node(package='scout_sim', executable='mapping_mission', output='screen',
                    parameters=[{'use_sim_time': True, 'vision_enabled': vision}])]
    if vision:
        actions.append(Node(package='scout_visual_follow', executable='visual_follow', output='screen',
                            prefix=shlex.quote(str(Path(arg('python_executable')).expanduser())),
                            parameters=[{
                                'model_path': str(Path(arg('model_path')).expanduser()),
                                'show_window': flag('show_window'),
                                'device': ParameterValue(arg('device'), value_type=str),
                                'auto_target': True, 'auto_confirm_frames': 3, 'auto_release_frames': 3,
                                'auto_vertical_margin_px': 4,
                                'use_sim_time': True, 'depth_registered': True,
                                'confidence': float(arg('confidence')), 'imgsz': int(arg('imgsz')),
                                'min_box_area': 200., 'depth_min_valid_ratio': .15,
                                **approach,
                                'max_linear_speed': .14, 'max_angular_speed': .35,
                            }]))
    if flag('rviz'):
        view = Path(get_package_share_directory('scout_sim')) / 'config' / 'mapping_mission.rviz'
        actions.append(Node(package='rviz2', executable='rviz2', output='screen',
                            arguments=['-d', str(view)], parameters=[{'use_sim_time': True}]))
    return actions


def generate_launch_description():
    root = Path.home() / 'scout'
    stamp = datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    defaults = {
        'database': str(root / f'maps/sim-{stamp}.db'),
        'profile': str(root / 'config/robot.yaml'),
        'python_executable': str(root / '.venv-yolo-train/bin/python'),
        'model_path': str(root / 'models/grapes/wgisd-grape-cluster-detection-v1.pt'),
        'gui': 'false', 'software_rendering': 'true', 'headless_rendering': 'false',
        'rviz': 'false', 'show_window': 'false', 'vision': 'true',
        'grapes': 'false', 'device': 'cpu', 'confidence': '0.35', 'imgsz': '640',
        'stop_distance': '1.60', 'resume_distance': '1.80', 'linear_gain': str(DEFAULT_APPROACH['linear_gain']),
    }
    return LaunchDescription([*[DeclareLaunchArgument(n, default_value=v) for n, v in defaults.items()],
                              OpaqueFunction(function=start)])
