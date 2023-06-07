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

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))


def generate_launch_description():

    # Directories
    pkg_eddiebot_description = get_package_share_directory(
        'eddiebot_description')
    pkg_eddiebot_gazebo = get_package_share_directory(
        'eddiebot_gazebo')

    # Paths
    eddiebot_ros_gz_bridge_launch = PathJoinSubstitution([pkg_eddiebot_gazebo, 'launch', 'ros_gz_bridge.launch.py'])
    robot_description_launch = PathJoinSubstitution(
        [pkg_eddiebot_description, 'launch', 'robot_description.launch.py'])

    # Launch configurations
    namespace = LaunchConfiguration('namespace')
    x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

    robot_name = 'eddiebot'

    spawn_robot_group_action = GroupAction([
        PushRosNamespace(namespace),

        # Robot description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_description_launch]),
            launch_arguments=[('model', LaunchConfiguration('model')),
                              ('use_sim_time', LaunchConfiguration('use_sim_time'))]
        ),

        # Spawn Eddiebot
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', robot_name,
                       '-x', x,
                       '-y', y,
                       '-z', z,
                       '-Y', yaw,
                       '-topic', 'robot_description'],
            output='screen'
        ),

        # Use RGBD sensor as a LIDAR
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'scan_time': 0.033},
                {'range_min': 0.45},
                {'range_max': 12.0},
                {'scan_height': 1},
                {'output_frame': 'base_link'},
            ],
            remappings=[
                ('depth_camera_info', '/kinect_rgbd_camera/camera_info'),
                ('depth', '/kinect_rgbd_camera/depth_image')
            ]
        ),

        # ROS GZ bridge
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([eddiebot_ros_gz_bridge_launch]),
            launch_arguments=[
                ('model', LaunchConfiguration('model')),
                ('robot_name', robot_name),
                ('namespace', namespace)]
        ),
    ])

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(spawn_robot_group_action)
    return ld
