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
        default_value='true'
    )

    odometry = Node(
        package='robot_localization',
        executable='ekf_node',
        name='odometry_fiter',
        namespace='robot',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory('field_robot'), 'config/ekf.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    return LaunchDescription([
        use_sim_time,
        odometry,
    ])
