#!/usr/bin/python3

from distutils.log import debug
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.acions import DeclareLaunchArgument


def generate_launch_description():
    world = DeclareLaunchArgument(
            'world',
            default_value=os.path.join(get_package_share_directory('field_robot'), 'worlds', 'main.world')
        )

    gui = DeclareLaunchArgument(
            'gui',
            default_value='true'
        )

    pause = DeclareLaunchArgument(
            'pause',
            default_value='true'
        )

    debug = DeclareLaunchArgument(
            'debug',
            default_value='debug'
        )

    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'gui': LaunchConfiguration('gui'),
                'pause': LaunchConfiguration('pause'),
                'debug': LaunchConfiguration('debug'),
                'verbose': LaunchConfiguration('debug'),
            }.items(),
        )
    
    robot = Node(
            package='field_robot',
            executable='robot_spawner.py',
            name='robot_spawner'
        )

    return LaunchDescription([
        gazebo,
        robot,
    ])
