import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from scout_nav_bringup.profile import default_profile, read_profile, mount_values


def start(context):
    profile = read_profile(LaunchConfiguration('profile').perform(context))
    simulation = LaunchConfiguration('simulation', default='false').perform(context).lower() == 'true'
    if simulation:
        if os.environ.get('ROS_DOMAIN_ID') not in {'42', *(str(n) for n in range(190, 200))}:
            raise ValueError('Use sim_env.bash for the isolated simulation domain')
    else:
        mount_values(profile)
    get_package_share_directory('rtabmap_slam')  # Clear error if the system dependency is missing.
    robot, camera = profile['robot'], profile['camera']
    database = Path(LaunchConfiguration('database').perform(context)).expanduser()
    localization = LaunchConfiguration('localization').perform(context).lower() == 'true'
    if localization and not database.is_file():
        raise ValueError(f'Localization requires an existing RTAB-Map database: {database}')
    database.parent.mkdir(parents=True, exist_ok=True)
    # Registered depth is transformed in 3D; a pitched camera is NOT a horizontal LaserScan.
    params = {
        'frame_id': robot['base_frame'], 'map_frame_id': 'map',
        'odom_frame_id': '', 'publish_tf': True,
        'subscribe_depth': True, 'subscribe_rgb': True, 'subscribe_rgbd': False,
        'subscribe_scan': False, 'approx_sync': True, 'approx_sync_max_interval': 0.08,
        'topic_queue_size': 10, 'sync_queue_size': 10,
        'qos_image': 2, 'qos_camera_info': 2, 'qos_odom': 1,
        'database_path': str(database), 'use_sim_time': simulation,
        'Reg/Force3DoF': 'true', 'RGBD/OptimizeFromGraphEnd': 'false',
        'Grid/Sensor': '1', 'Grid/3D': 'true',
        'Grid/CellSize': '0.05', 'Grid/RangeMin': '0.2', 'Grid/RangeMax': '4.0',
        'Grid/MaxGroundHeight': '0.08', 'Grid/MaxObstacleHeight': '1.8',
        'Mem/IncrementalMemory': 'false' if localization else 'true',
        'Mem/InitWMWithAllNodes': 'true' if localization else 'false',
    }
    if simulation:
        params.update({'Rtabmap/DetectionRate': '2.0', 'RGBD/LinearUpdate': '0.12',
                       'RGBD/AngularUpdate': '0.12', 'RGBD/ProximityBySpace': 'false',
                       'Grid/RayTracing': 'true', 'Kp/MaxFeatures': '700',
                       'Mem/DepthCompressionFormat': '.png'})
    return [Node(package='rtabmap_slam', executable='rtabmap', namespace='scout',
                 name='rtabmap', output='screen', parameters=[params], remappings=[
                     ('rgb/image', camera['color_topic']), ('depth/image', camera['depth_topic']),
                     ('rgb/camera_info', camera['camera_info_topic']),
                     ('odom', LaunchConfiguration('simulation_odom_topic',
                                                 default=robot['odom_topic']).perform(context)
                      if simulation else robot['odom_topic']), ('map', '/map'),
                     *([('info', '/scout/rtabmap/info')] if simulation else [])])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value=default_profile()),
        DeclareLaunchArgument('database', default_value=str(Path.home() / 'scout/maps/scout.db')),
        DeclareLaunchArgument('simulation', default_value='false'),
        DeclareLaunchArgument('simulation_odom_topic', default_value='/scout/odom'),
        DeclareLaunchArgument('localization', default_value='false'), OpaqueFunction(function=start)])
