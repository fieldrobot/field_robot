#!/usr/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true'
    )

    urdf_path = os.path.join(get_package_share_directory('field_robot'), 'models', 'robot', 'robot.urdf')
    urdf = DeclareLaunchArgument(
        'urdf',
        default_value=open(urdf_path,'r').read()
    )

    odometry = Node(
        package='robot_localization',
        executable='ekf_node',
        name='odometry_filter',
        namespace='robot',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory('field_robot'), 'config/odometry.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='robot',
        output='screen',
        parameters=[
            {'robot_description': LaunchConfiguration('urdf')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    sensor_processing = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('field_robot'), 'launch', 'sensor_processing.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    '''navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'mapping.launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )'''

    return LaunchDescription([
        # parameters
        use_sim_time,
        urdf,
        # nodes & launch files
        robot_state_publisher,
        odometry,
        #sensor_processing,
    ])
