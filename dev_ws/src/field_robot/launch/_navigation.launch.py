import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav = get_package_share_directory('nav2_bringup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value = 'true',
            description = 'Argument for use_sim_time'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value = 'true',
            description = 'Argument for autostart'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value = os.path.join(get_package_share_directory('field_robot'), 'config', 'navigation_parameters.yaml'),
            description = 'Argument for params_file'
        ),
        DeclareLaunchArgument(
            'default_bt_xml_filename',
            default_value = os.path.join(get_package_share_directory('field_robot'), 'config', 'navigation_bt_tree.xml'),
            description = 'Argument for default_bt_xml_filename'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav, 'launch', 'navigation_launch.py')
            ),
        ),
    ])
