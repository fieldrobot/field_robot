#!/usr/bin/python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    namespace_str="/robot"
    pkg_share = get_package_share_directory("field_robot")

    async_slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace=namespace_str,
        parameters=[
            os.path.join(pkg_share, 'config', 'mapping.yaml'),
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ("/scan", "scan"),
            ("/map", "map"),
            ("/map_metadata", "map_metadata"),
            ("/slam_toolbox/scan_visualization", "slam_toolbox/scan_visualization"),
            ("/slam_toolbox/graph_visualization", "slam_toolbox/graph_visualization"),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        async_slam_node,
    ])
