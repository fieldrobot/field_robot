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
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': world,
                'gui': 'true',
                'pause': 'true',
                'debug': 'true',
            }.items(),
        ),
        
        #Node(
        #    package='field_robot',
        #    executable='robot_spawner.py',
        #    name='robot_spawner'
        #),

        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(
        #        os.path.join(get_package_share_directory('field_robot'), 'launch', 'robot_state_publisher.launch.py')
        #    ),
        #),
    ])
