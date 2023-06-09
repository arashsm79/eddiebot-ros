from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace
from launch.conditions import IfCondition, UnlessCondition


ARGUMENTS = [
    DeclareLaunchArgument('model', default_value='eddie_kinect_v1',
                          choices=['eddie_kinect_v1'],
                          description='Eddiebot Model'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('use_rtabmap_viz', default_value='false',
                          choices=['true', 'false'],
                          description='Launch rtabmap_viz'),
    DeclareLaunchArgument('localization', default_value='true',
                          choices=['true', 'false'],
                          description='Localize only, do not change loaded map')

]


def generate_launch_description():

    # Directories
    pkg_eddiebot_nav = get_package_share_directory(
        'eddiebot_nav')

    # Synchronize the depth and rgb images for old rgbd sensors like Kinect 360.
    rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'approx_sync': True},
        ],
        remappings=[
            ('rgb/image', '/kinect_rgbd_camera/image'),
            ('rgb/camera_info', '/kinect_rgbd_camera/camera_info'),
            ('depth/image', '/kinect_rgbd_camera/depth_image'),
            ('rgbd_image', '/kinect_rgbd_camera/rgbd_image'),
            ('rgbd_image/compressed', '/kinect_rgbd_camera/rgbd_image/compressed'),
        ]
    )

    # RGBD odometry using classic feature extraction methods. This is not useful for smooth textures or simple environments.
    rgbd_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'frame_id': 'base_footprint'},
            {'odom_frame_id': 'odom'},
            {'publish_tf': False},
            {'subscribe_rgbd': True},
            {'Vis/InlierDistance': '0.05'}
        ],
        remappings=[
            ('rgbd_image', '/kinect_rgbd_camera/rgbd_image'),
            ('rgbd_image/compressed', '/kinect_rgbd_camera/rgbd_image/compressed'),
            ('odom', '/vis_odom')
        ]
    )

    # extended Kalman filter for combining visual odometry with wheel odometry.
    robot_localization_file_path = PathJoinSubstitution([pkg_eddiebot_nav, 'config', 'ekf.yaml'])
    robot_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            robot_localization_file_path,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('odometry/filtered', '/odom')
        ]
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(robot_localization)
    ld.add_action(rgbd_sync)
    ld.add_action(rgbd_odometry)
    return ld
