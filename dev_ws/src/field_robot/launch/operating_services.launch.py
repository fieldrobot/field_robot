#!/usr/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    use_sim_time = DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        )

    urdf = DeclareLaunchArgument(
            'urdf',
            default_value=os.path.join(get_package_share_directory('field_robot'), 'models', 'robot', 'robot.urdf'),
        )

    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            node_namespace='robot',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            arguments=[
                LaunchConfiguration('urdf')
            ],
        )

    odometry = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                os.path.join(get_package_share_directory('field_robot'), 'config/odometry.yaml'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ]
        )

    #urdf = os.path.join(
    #    get_package_share_directory('field_robot'), 'models', 'robot', 'robot.urdf')

    return LaunchDescription([
        use_sim_time,
        urdf,
        robot_state_publisher,
        odometry,
    ])
