#!/usr/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    world = os.path.join(get_package_share_directory('field_robot'), 'worlds', 'main.world')

    return LaunchDescription([
        # simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('field_robot'), 'launch', 'simulation.launch.py')
            ),
        ),

        #essential operating services
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('field_robot'), 'launch', 'operating_services.launch.py')
            ),
        ),
    ])
