import json
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap
from nav2_common.launch import RewrittenYaml
from scout_nav_bringup.profile import default_profile, read_profile, mount_values, require_footprint


def start(context):
    profile = read_profile(LaunchConfiguration('profile').perform(context))
    mount_values(profile)
    footprint = require_footprint(profile)
    share = Path(get_package_share_directory('scout_nav_bringup'))
    params = RewrittenYaml(
        source_file=str(share / 'config/nav2_scout.yaml'),
        root_key='', convert_types=True,
        param_rewrites={
            'footprint': json.dumps(footprint),
            'robot_base_frame': profile['robot']['base_frame'],
            'odom_topic': profile['robot']['odom_topic'],
            'default_nav_to_pose_bt_xml': str(share / 'config/navigate_to_pose.xml'),
        })
    return [GroupAction(actions=[
        SetRemap(src='cmd_vel_smoothed', dst='/scout/navigation_cmd_vel'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            get_package_share_directory('nav2_bringup') + '/launch/navigation_launch.py'),
            launch_arguments={
                'params_file': params, 'use_sim_time': 'false', 'autostart': 'true',
                'use_composition': 'False', 'use_respawn': 'False', 'namespace': '',
            }.items()),
    ])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value=default_profile()), OpaqueFunction(function=start)])
