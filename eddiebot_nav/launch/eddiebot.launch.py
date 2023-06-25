
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace


ARGUMENTS = [
    DeclareLaunchArgument('model', default_value='eddie_kinect_v1',
                          choices=['eddie_kinect_v1'],
                          description='Eddiebot Model'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='use_sim_time'),

]


def generate_launch_description():

    eddie_odom = Node(
        package="eddiebot_odom",
        executable="eddie_odom",
        name="eddie_odom",
        # remappings=[
        #     ("odom", "/wheel_odom")
        # ]
    )

    eddie_vel_controller = Node(
        package="eddiebot_vel_controller",
        executable="eddie_vel_controller",
        name="eddie_vel_controller",
    )

    # Published topics
    # /camera_info, /depth/camera_info
    # /depth/image_raw, /depth/image_raw/compressed, /depth/image_raw/compressedDepth, /depth/image_raw/theora
    # /image_raw, /image_raw/compressed, /image_raw/compressedDepth, /image_raw/theora
    # /points
    kinect = Node(
        package="kinect_ros2",
        executable="kinect_ros2_node",
        name="kinect_ros2",
    )

    kinect_rgb_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='kinect_depth_static_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'camera_depth_optical_frame', '--child-frame-id', 'kinect_depth'
            ]
    )
    kinect_depth_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='kinect_rgb_static_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'camera_rgb_optical_frame', '--child-frame-id', 'kinect_rgb'
            ]
    )

    # Use RGBD sensor as a LIDAR
    fake_scan1 = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'scan_time': 0.333},
            {'range_min': 0.10},
            {'range_max': 7.0},
        ],
        remappings=[
            ('cloud_in', '/points'),
        ]
    )

    fake_scan2 = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan2',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'range_min': 0.30},
            {'range_max': 10.0},
            {'scan_height': 50}
        ],
        remappings=[
            ('depth', '/depth/image_raw'),
            ('depth_camera_info', '/depth/camera_info'),
        ],
    )
    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(kinect)
    ld.add_action(kinect_rgb_tf)
    ld.add_action(kinect_depth_tf)
    ld.add_action(eddie_odom)
    ld.add_action(eddie_vel_controller)
    ld.add_action(fake_scan2)
    return ld
