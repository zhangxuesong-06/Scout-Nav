from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from scout_nav_bringup.profile import default_profile, read_profile, require_base


def start(context):
    if LaunchConfiguration('start_hardware').perform(context).lower() != 'true':
        raise ValueError('Starting the driver enables CAN commanded mode. Set start_hardware:=true explicitly.')
    profile = read_profile(LaunchConfiguration('profile').perform(context))
    port = require_base(profile)
    robot = profile['robot']
    return [Node(package='scout_base', executable='scout_base_node', namespace='scout',
                 name='base', output='screen', parameters=[{
                     'port_name': port, 'is_scout_mini': True, 'is_omni_wheel': False,
                     'simulated_robot': False, 'control_rate': 50, 'use_sim_time': False,
                     'odom_frame': robot['odom_frame'], 'base_frame': robot['base_frame'],
                     'odom_topic_name': robot['odom_topic'],
                 }], remappings=[('/cmd_vel', '/scout/cmd_vel'),
                                 ('/scout_status', '/scout/status'),
                                 ('/rc_status', '/scout/rc_status'),
                                 ('/light_control', '/scout/light_control')])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value=default_profile()),
        DeclareLaunchArgument('start_hardware', default_value='false'), OpaqueFunction(function=start)])
