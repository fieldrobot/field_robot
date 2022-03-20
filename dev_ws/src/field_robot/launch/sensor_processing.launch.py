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
        # image path finder AI
        Node(
            package='field_robot',
            executable='image_path_finder_ai.py',
            node_namespace='camera_front',
            name='image_path_finder_ai_front',
            parameters=[
                {'image_src' : '/image'},
                {'image_dst' : '/image'},
            ]
        ),
        
        Node(
            package='field_robot',
            executable='image_path_finder_ai.py',
            node_namespace='camera_rear',
            name='image_path_finder_ai_rear',
            parameters=[
                {'image_src' : '/image'},
                {'image_dst' : '/image'},
            ]
        ),
        
        Node(
            package='field_robot',
            executable='image_path_finder_ai.py',
            node_namespace='camera_left',
            name='image_path_finder_ai_left',
            parameters=[
                {'image_src' : '/image'},
                {'image_dst' : '/image'},
            ]
        ),
        
        Node(
            package='field_robot',
            executable='image_path_finder_ai.py',
            node_namespace='camera_right',
            name='image_path_finder_ai_right',
            parameters=[
                {'image_src' : '/image'},
                {'image_dst' : '/image'},
            ]
        ),
    ])
