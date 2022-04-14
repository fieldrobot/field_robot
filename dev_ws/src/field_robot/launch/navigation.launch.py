#!/usr/bin/python3

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav = get_package_share_directory('nav2_bringup')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
        }.items(),
    )

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(get_package_share_directory('field_robot'), 'config','nav2_params.yaml')),

        # Actual Scripts
        nav2,
    ])
