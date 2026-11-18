"""Bring up the real Scout grape-following chain with one command."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from scout_nav_bringup.profile import default_profile, mount_values, read_profile, require_base, require_camera


def start(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)
    if arg('start_hardware').lower() != 'true':
        raise ValueError('Real following requires start_hardware:=true explicitly')
    profile_path = arg('profile')
    profile = read_profile(profile_path)
    require_camera(profile)
    require_base(profile)
    mount_values(profile, allow_approximate=arg('allow_approximate_mount').lower() == 'true')
    if not Path(arg('model_path')).expanduser().is_file():
        raise ValueError('The selected grape model does not exist')
    nav = Path(get_package_share_directory('scout_nav_bringup')) / 'launch'
    vision = Path(get_package_share_directory('scout_visual_follow')) / 'launch'
    # Resolve dependencies before any hardware process starts.
    get_package_share_directory('scout_base')
    get_package_share_directory('orbbec_camera')

    def include(path, **arguments):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                                        launch_arguments={'profile': profile_path, **arguments}.items())

    def stop_chain(event, context):
        if context.is_shutdown:
            return []
        return [EmitEvent(event=Shutdown(
            reason='A following process exited; stopping the whole chain'))]

    actions = [RegisterEventHandler(OnProcessExit(on_exit=stop_chain))]
    if profile['camera'].get('extrinsics_confirmed') is not True:
        actions.append(LogInfo(msg='FOLLOW TEST: camera position is from simulation; '
                                   'orientation is assumed level, not a measured calibration.'))
    limits = {name: arg(name) for name in ('max_linear_speed', 'max_angular_speed')}
    actions.extend([
        include(nav / 'camera.launch.py', soft_filter_speckle_size=arg('depth_speckle_size')),
        include(nav / 'mount.launch.py', allow_approximate_mount=arg('allow_approximate_mount')),
        include(nav / 'base.launch.py', start_hardware='true'),
        include(vision / 'preview.launch.py', **limits, **{name: arg(name) for name in (
            'model_path', 'python_executable', 'confidence', 'imgsz', 'min_box_area',
            'device', 'show_window', 'target_id', 'depth_min_valid_pixels', 'depth_min_valid_ratio')}),
        include(nav / 'motion_gate.launch.py', mode='follow', auto_enable=arg('auto_enable'), **limits),
    ])
    return actions


def generate_launch_description():
    root = Path.home() / 'scout'
    defaults = dict(profile=default_profile(), start_hardware='false',
                    allow_approximate_mount='false', auto_enable='false',
                    model_path=str(root / 'models/grapes/wgisd-grape-cluster-detection-v1.pt'),
                    python_executable=str(root / '.venv-yolo-train/bin/python'),
                    confidence='0.10', imgsz='800', min_box_area='200', device='0',
                    show_window='true', target_id='-1', depth_speckle_size='20',
                    depth_min_valid_pixels='20', depth_min_valid_ratio='0.15',
                    max_linear_speed='0.05', max_angular_speed='0.15')
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
                             + [OpaqueFunction(function=start)])
