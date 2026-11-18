from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from scout_nav_bringup.profile import default_profile, read_profile, mount_values


def start(context):
    profile = read_profile(LaunchConfiguration('profile').perform(context))
    approximate = LaunchConfiguration('allow_approximate_mount').perform(context).lower() == 'true'
    xyz, rpy = mount_values(profile, allow_approximate=approximate)
    return [Node(package='tf2_ros', executable='static_transform_publisher',
                 name='nav_camera_mount', output='screen', arguments=[
                     '--x', str(xyz[0]), '--y', str(xyz[1]), '--z', str(xyz[2]),
                     '--roll', str(rpy[0]), '--pitch', str(rpy[1]), '--yaw', str(rpy[2]),
                     '--frame-id', profile['robot']['base_frame'],
                     '--child-frame-id', profile['camera']['name'] + '_link'])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value=default_profile()),
        DeclareLaunchArgument('allow_approximate_mount', default_value='false'),
        OpaqueFunction(function=start)])
