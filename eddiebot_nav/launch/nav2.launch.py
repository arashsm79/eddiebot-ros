from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import PushRosNamespace


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
]


def generate_launch_description():
    pkg_eddiebot_nav = get_package_share_directory('eddiebot_nav')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    nav2_params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution(
            [pkg_eddiebot_nav, 'config', 'nav2.yaml']),
        description='Nav2 parameters')

    namespace_arg = DeclareLaunchArgument(
                        'namespace',
                        default_value='',
                        description='Robot namespace')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    nav2 = GroupAction([
        PushRosNamespace(namespace),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [pkg_nav2_bringup, 'launch', 'navigation_launch.py'])),
            launch_arguments={'use_sim_time': use_sim_time,
                              'use_composition': 'False',
                              'params_file': LaunchConfiguration('params_file')}.items()),
    ])

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(nav2_params_arg)
    ld.add_action(namespace_arg)
    ld.add_action(nav2)
    return ld
