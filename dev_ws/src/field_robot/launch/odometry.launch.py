#!/usr/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    world = os.path.join(get_package_share_directory('field_robot'), 'models', 'worlds', 'main.sdf')

    return LaunchDescription([
        IncludeLaunchDescription(
            Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                os.path.join(get_package_share_directory('field_robot'), 'config/odometry.yaml'),
                {'use_sim_time': 'true'}
            ]
        ),
    ])
