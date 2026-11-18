from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from scout_nav_bringup.profile import default_profile, read_profile


def start(context):
    profile = read_profile(LaunchConfiguration('profile').perform(context))
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in ('disabled', 'navigation', 'follow'):
        raise ValueError('mode must be disabled, navigation or follow')
    return [Node(package='scout_nav_bringup', executable='motion_gate', output='screen', parameters=[{
        'mode': mode, 'depth_topic': profile['camera']['depth_topic'],
        'odom_topic': profile['robot']['odom_topic'],
        'auto_enable': LaunchConfiguration('auto_enable').perform(context).lower() == 'true',
        'max_linear_speed': float(LaunchConfiguration('max_linear_speed').perform(context)),
        'max_angular_speed': float(LaunchConfiguration('max_angular_speed').perform(context)),
    }])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value=default_profile()),
        DeclareLaunchArgument('mode', default_value='disabled'),
        DeclareLaunchArgument('auto_enable', default_value='false'),
        DeclareLaunchArgument('max_linear_speed', default_value='0.15'),
        DeclareLaunchArgument('max_angular_speed', default_value='0.35'),
        OpaqueFunction(function=start)])
