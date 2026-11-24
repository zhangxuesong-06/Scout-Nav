import shlex

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from scout_nav_bringup.profile import default_profile, read_profile
from scout_visual_follow.follow_parameters import DEFAULT_APPROACH, resolve_approach


def start(context):
    approach = resolve_approach({name: LaunchConfiguration(name).perform(context) for name in DEFAULT_APPROACH})
    profile = read_profile(LaunchConfiguration('profile').perform(context))
    camera = profile['camera']
    python = LaunchConfiguration('python_executable').perform(context)
    options = {'prefix': shlex.quote(python)} if python else {}
    return [Node(package='scout_visual_follow', executable='visual_follow', output='screen', **options, parameters=[{
        'model_path': LaunchConfiguration('model_path').perform(context),
        'show_window': LaunchConfiguration('show_window').perform(context).lower() == 'true',
        'depth_registered': True, 'color_topic': camera['color_topic'],
        'depth_topic': camera['depth_topic'], 'cmd_vel_topic': '/scout/follow_cmd_vel',
        'camera_info_topic': camera['camera_info_topic'],
        'base_frame': profile['robot']['base_frame'],
        'target_id': int(LaunchConfiguration('target_id').perform(context)),
        'confidence': float(LaunchConfiguration('confidence').perform(context)),
        'imgsz': int(LaunchConfiguration('imgsz').perform(context)),
        'min_box_area': float(LaunchConfiguration('min_box_area').perform(context)),
        'depth_min_valid_pixels': int(LaunchConfiguration('depth_min_valid_pixels').perform(context)),
        'depth_min_valid_ratio': float(LaunchConfiguration('depth_min_valid_ratio').perform(context)),
        'device': ParameterValue(LaunchConfiguration('device').perform(context), value_type=str),
        'max_linear_speed': float(LaunchConfiguration('max_linear_speed').perform(context)),
        'max_angular_speed': float(LaunchConfiguration('max_angular_speed').perform(context)),
        **approach,
    }])]


def generate_launch_description():
    model = get_package_share_directory('scout_visual_follow') + '/models/yolov8n.pt'
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value=default_profile()),
        DeclareLaunchArgument('model_path', default_value=model),
        DeclareLaunchArgument('show_window', default_value='false'),
        DeclareLaunchArgument('target_id', default_value='-1'),
        DeclareLaunchArgument('confidence', default_value='0.2'),
        DeclareLaunchArgument('imgsz', default_value='640'),
        DeclareLaunchArgument('min_box_area', default_value='2000'),
        DeclareLaunchArgument('depth_min_valid_pixels', default_value='20'),
        DeclareLaunchArgument('depth_min_valid_ratio', default_value='0.3'),
        DeclareLaunchArgument('device', default_value=''),
        DeclareLaunchArgument('max_linear_speed', default_value='0.15'),
        DeclareLaunchArgument('max_angular_speed', default_value='0.35'),
        *(DeclareLaunchArgument(name, default_value=str(value)) for name, value in DEFAULT_APPROACH.items()),
        DeclareLaunchArgument('python_executable', default_value='',
                              description='Optional Python interpreter for the inference node'),
        OpaqueFunction(function=start)])
