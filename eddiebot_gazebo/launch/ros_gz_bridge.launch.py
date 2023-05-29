from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution

from launch_ros.actions import Node

ARGUMENTS = [
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('description', default_value='false',
                              description='Launch eddiebot description'),
        DeclareLaunchArgument('model', default_value='eddie_kinect_v1',
                              choices=['eddie_kinect_v1', 'eddie_kinect_v2'],
                              description='Eddiebot Model'),
        DeclareLaunchArgument('robot_name', default_value='eddiebot',
                              description='Robot name'),
        DeclareLaunchArgument('namespace', default_value='',
                              description='Robot namespace'),
        DeclareLaunchArgument('world', default_value='warehouse',
                              description='World name'),
        ]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    namespace = LaunchConfiguration('namespace')
    world = LaunchConfiguration('world')

    # Define LaunchDescription variable
    jointstate_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='joint_states_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[
            ['/model/eddiebot/joint_states' +
             '@sensor_msgs/msg/JointState@gz.msgs.Model']
        ])
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(jointstate_bridge)
    return ld

    # we don't need the below bridges for now
    # lidar bridge
    # lidar_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     name='lidar_bridge',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time
    #     }],
    #     arguments=[
    #         ['/world/', world,
    #          '/model/', robot_name,
    #          '/lidar/scan' +
    #          '@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan']
    #     ])

    # # Camera sensor bridge
    # kinect_camera_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     name='camera_bridge',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     arguments=[
    #         ['/world/', world,
    #          '/model/', robot_name,
    #          '/kinect_rgbd_camera/image' +
    #          '@sensor_msgs/msg/Image' +
    #          '[gz.msgs.Image'],
    #         ['/world/', world,
    #          '/model/', robot_name,
    #          '/kinect_rgbd_camera/depth_image' +
    #          '@sensor_msgs/msg/Image' +
    #          '[gz.msgs.Image'],
    #         ['/world/', world,
    #          '/model/', robot_name,
    #          '/kinect_rgbd_camera/points' +
    #          '@sensor_msgs/msg/PointCloud2' +
    #          '[gz.msgs.PointCloudPacked'],
    #         ['/world/', world,
    #          '/model/', robot_name,
    #          '/kinect_rgbd_camera/camera_info' +
    #          '@sensor_msgs/msg/CameraInfo' +
    #          '[gz.msgs.CameraInfo'],
    #         ],
    # )

    # cmd_vel_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     name='cmd_vel_bridge',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time
    #     }],
    #     arguments=[
    #         [namespace,
    #          '/cmd_vel' + '@geometry_msgs/msg/Twist' + '[gz.msgs.Twist'],
    #         ['/model/', robot_name, '/cmd_vel' +
    #          '@geometry_msgs/msg/Twist' +
    #          ']gz.msgs.Twist']
    #     ])

    # pose_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     name='pose_bridge',
    #     output='screen',
    #     parameters=[{
    #          'use_sim_time': use_sim_time
    #     }],
    #     arguments=[
    #         ['/model/', robot_name, '/pose' +
    #          '@tf2_msgs/msg/TFMessage' +
    #          '[gz.msgs.Pose_V']
    #     ])

    # # odom to base_link transform bridge
    # odom_base_tf_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     name='odom_base_tf_bridge',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time
    #     }],
    #     arguments=[
    #         ['/model/', robot_name, '/tf' +
    #          '@tf2_msgs/msg/TFMessage' +
    #          '[gz.msgs.Pose_V']
    #     ])
