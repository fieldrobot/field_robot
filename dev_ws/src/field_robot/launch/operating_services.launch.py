#!/usr/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    urdf = os.path.join(
        get_package_share_directory('field_robot'), 'models', 'robot', 'robot.urdf')

    world = os.path.join(get_package_share_directory('field_robot'), 'worlds', 'main.world')

    return LaunchDescription([
        # robot state publisher
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            node_namespace='robot',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf]
        ),

        # odometry
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
