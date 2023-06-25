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
    DeclareLaunchArgument('qos', default_value='2',
                          description='QoS used for input sensor topics'),
    DeclareLaunchArgument('localization', default_value='false',
                          choices=['true', 'false'],
                          description='Localize only, do not change loaded map')

]


def generate_launch_description():

    # Directories
    pkg_eddiebot_nav = get_package_share_directory(
        'eddiebot_nav')

    rtabmap_parameters = {
            'use_sim_time': LaunchConfiguration('use_sim_time'),

            # rtabmap ros wrapper parameters
            'frame_id': 'base_footprint',
            'map_frame_id': 'map',
            'subscribe_depth': True,
            'subscribe_scan': True,
            'subscribe_rgbd': False,
            'qos_image': LaunchConfiguration('qos'),
            'qos_scan': LaunchConfiguration('qos'),
            'use_action_for_goal': True,
            'approx_sync': True,
            'queue_size': 30,

            # rtabmap parameters
            'Optimizer/Strategy': '1',

            'RGBD/ProximityBySpace': 'false',
            'Reg/Force3DoF': 'true',
            'Vis/MinInliers': '12',

            'RGBD/AngularUpdate': '0.01',
            'RGBD/LinearUpdate': '0.01',
            'RGBD/OptimizeFromGraphEnd': 'false',

            # 'Grid/FromDepth': 'true',
            # 'Grid/MaxObstacleHeight': '0.7',
            # 'Reg/Strategy': '0'
    }
    rtabmap_remappings = [
            ('rgb/image', '/image_raw'),
            ('rgb/camera_info', '/camera_info'),
            ('depth/image', '/depth/image_raw'),
    ]
    rtabmap_arguments = ['-d', '--ros-args', '--log-level', 'Warn']

    # SLAM mode:
    rtabmap_slam = Node(
            condition=UnlessCondition(LaunchConfiguration('localization')),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[rtabmap_parameters],
            remappings=rtabmap_remappings,
            arguments=rtabmap_arguments
    )

    # Localization mode:
    rtabmap_localization = Node(
           condition=IfCondition(LaunchConfiguration('localization')),
           package='rtabmap_slam', executable='rtabmap', output='screen',
           parameters=[rtabmap_parameters,
                       {
                           'Mem/IncrementalMemory': 'False',
                           'Mem/InitWMWithAllNodes': 'True'
                       }],
           remappings=rtabmap_remappings
    )

    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        output='screen',
        parameters=[rtabmap_parameters],
        remappings=rtabmap_remappings,
        condition=IfCondition(LaunchConfiguration('use_rtabmap_viz'))
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(rtabmap_slam)
    ld.add_action(rtabmap_localization)
    ld.add_action(rtabmap_viz)
    return ld

    #
    # Example of how odometry merging could be done.
    #

    # Synchronize the depth and rgb images for old rgbd sensors like Kinect 360.
    rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'approx_sync': False},
        ],
        remappings=[
            # input
            ('rgb/image', '/image_raw'),
            ('rgb/camera_info', '/camera_info'),
            ('depth/image', '/depth/image_raw'),
            ('depth/camera_info', '/depth/camera_info'),

            # output
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
            {'subscribe_rgbd': False},
            {'approx_sync': True},
            {'Vis/InlierDistance': '0.05'}
        ],
        remappings=rtabmap_remappings + [
            # output
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
    ld.add_action(rgbd_odometry)
    ld.add_action(robot_localization)
    ld.add_action(rgbd_sync)
