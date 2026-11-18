import os
from pathlib import Path
import shutil
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from scout_sim.scene import prepare_scene


def start(context):
    def value(key):
        return LaunchConfiguration(key).perform(context)
    def boolean(key):
        text = value(key).lower()
        if text not in ('true', 'false'):
            raise ValueError(f'{key} must be true or false')
        return text == 'true'
    domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
    if not 1 <= domain <= 232:
        raise ValueError('Use an isolated simulation domain: source ~/scout/scripts/sim_env.bash')
    share = Path(get_package_share_directory('scout_sim'))
    runtime = Path(tempfile.mkdtemp(prefix='scout_sim_'))
    camera, gui = boolean('camera'), boolean('gui')
    try:
        scene, urdf, notes = prepare_scene(share, runtime, world=value('world'),
            camera_pitch_deg=float(value('camera_pitch_deg')), x=float(value('x')),
            y=float(value('y')), yaw=float(value('yaw')), camera=camera,
            static_environment=boolean('static_environment'))
    except Exception:
        shutil.rmtree(runtime)
        raise
    paths = str(share / 'models')
    env = {'IGN_GAZEBO_RESOURCE_PATH': paths, 'GZ_SIM_RESOURCE_PATH': paths,
           'IGN_PARTITION': os.environ.get('IGN_PARTITION', f'scout_sim_{domain}'),
           'IGN_GAZEBO_LOG_PATH': str(runtime), 'IGN_LOG_PATH': str(runtime),
           'XDG_CACHE_HOME': str(runtime / 'cache')}
    if boolean('software_rendering'):
        env.update(LIBGL_ALWAYS_SOFTWARE='1',
                   __EGL_VENDOR_LIBRARY_FILENAMES='/usr/share/glvnd/egl_vendor.d/50_mesa.json')
    command = ['ign', 'gazebo', '-r', '-v', '3']
    if not gui:
        command += ['-s', '--headless-rendering']
    command.append(str(scene))
    gazebo = ExecuteProcess(cmd=command, output='screen', additional_env=env)
    actions = [SetEnvironmentVariable('IGN_PARTITION', env['IGN_PARTITION']),
        LogInfo(msg=f'Scout simulation DDS domain {domain}, world {value("world")}; camera pitch {value("camera_pitch_deg")} deg'),
        *[LogInfo(msg=note) for note in notes],
        gazebo,
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='scout_sim_state_publisher', output='screen',
             parameters=[{'robot_description': urdf, 'use_sim_time': True}]),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name='scout_sim_bridge', output='screen',
             parameters=[{'config_file': str(share / 'config/bridge.yaml'), 'use_sim_time': True}]),
        Node(package='scout_sim', executable='command_guard', output='screen'),
        RegisterEventHandler(OnProcessExit(target_action=gazebo,
            on_exit=[EmitEvent(event=Shutdown(reason='Gazebo exited'))])),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(
            function=lambda _: shutil.rmtree(runtime, ignore_errors=True))])),
    ]
    if camera:
        actions.append(Node(package='ros_gz_bridge', executable='parameter_bridge',
            name='scout_sim_camera_bridge', output='screen', parameters=[{
                'config_file': str(share / 'config/camera_bridge.yaml'), 'use_sim_time': True,
                'override_frame_id': 'nav_camera_color_optical_frame'}]))
        actions.append(Node(package='ros_gz_bridge', executable='parameter_bridge',
            name='scout_sim_cloud_bridge', output='screen', parameters=[{
                'config_file': str(share / 'config/cloud_bridge.yaml'), 'use_sim_time': True,
                'override_frame_id': 'nav_camera_link'}]))
    if boolean('rviz'):
        actions.append(Node(package='rviz2', executable='rviz2', output='screen',
            arguments=['-d', str(share / 'config/scout_sim.rviz')],
            parameters=[{'use_sim_time': True}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='clearpath_playpen', description='Key in ASSET_MANIFEST.json worlds'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('static_environment', default_value='true',
                              description='Freeze legacy movable props for navigation; Scout remains dynamic'),
        DeclareLaunchArgument('software_rendering', default_value='false'),
        DeclareLaunchArgument('camera_pitch_deg', default_value='0.0', description='Simulation-only downward pitch in degrees'),
        DeclareLaunchArgument('x', default_value='0.0'), DeclareLaunchArgument('y', default_value='-2.0'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw in radians'),
        OpaqueFunction(function=start),
    ])
