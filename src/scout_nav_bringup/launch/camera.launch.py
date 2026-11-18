from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from scout_nav_bringup.profile import default_profile, read_profile, require_camera


def start(context):
    profile = read_profile(LaunchConfiguration('profile').perform(context))
    require_camera(profile)
    camera = profile['camera']
    path = get_package_share_directory('orbbec_camera') + '/launch/dabai_dcw.launch.py'
    args = {
        'camera_name': 'nav_camera', 'serial_number': str(camera['serial_number']),
        'depth_registration': 'true', 'enable_point_cloud': 'true',
        'enable_colored_point_cloud': 'false', 'publish_tf': 'true',
        'enable_color': 'true', 'enable_depth': 'true', 'enable_ir': 'false',
        'color_width': str(camera['width']), 'color_height': str(camera['height']),
        'depth_width': str(camera['width']), 'depth_height': str(camera['height']),
        'color_fps': str(camera['fps']), 'depth_fps': str(camera['fps']),
        'soft_filter_speckle_size': LaunchConfiguration('soft_filter_speckle_size').perform(context),
    }
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(path), launch_arguments=args.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value=default_profile()),
        DeclareLaunchArgument('soft_filter_speckle_size', default_value='-1'),
        OpaqueFunction(function=start)])
