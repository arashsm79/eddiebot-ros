# Inspired by https://github.com/turtlebot/turtlebot4_desktop/blob/humble/turtlebot4_viz/launch/view_robot.launch.py
# commit: 805279e4846ec6fd7bdf3f9a08034d361cd6668a
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace


ARGUMENTS = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'description',
            default_value='false',
            description='Launch eddiebot description'
            ),
        DeclareLaunchArgument('model', default_value='eddie_kinect_v1',
                              choices=['eddie_kinect_v1'],
                              description='Eddiebot Model'),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Robot namespace'
            ),
        ]


def generate_launch_description():

    pkg_eddiebot_viz = get_package_share_directory('eddiebot_rviz')
    pkg_eddiebot_description = get_package_share_directory('eddiebot_description')

    rviz2_config = PathJoinSubstitution(
        [pkg_eddiebot_viz, 'rviz', 'model.rviz'])
    description_launch = PathJoinSubstitution(
        [pkg_eddiebot_description, 'launch', 'robot_description.launch.py']
    )

    namespace = LaunchConfiguration('namespace')

    rviz = GroupAction([
        PushRosNamespace(namespace),

        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['-d', rviz2_config],
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
             remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
             ],
             output='screen'),

        # Delay launch of robot description to allow Rviz2 to load first.
        # Prevents visual bugs in the model.
        TimerAction(
            period=1.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([description_launch]),
                    launch_arguments=[('model', LaunchConfiguration('model')),
                                      ('use_sim_time', LaunchConfiguration('use_sim_time'))],
                    condition=IfCondition(LaunchConfiguration('description'))
                )])
    ])

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(rviz)
    return ld
