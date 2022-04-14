#!/usr/bin/python3

from distutils.log import debug
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # Paths
    pkg_share = get_package_share_directory('field_robot')
    urdf_path = os.path.join(pkg_share, 'models', 'robot', 'robot.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'default.rviz')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'world': LaunchConfiguration('world_path'),
            'gui': LaunchConfiguration('gui'),
            'pause': LaunchConfiguration('pause'),
            'debug': LaunchConfiguration('debug'),
            'verbose': LaunchConfiguration('debug'),
        }.items(),
    )

    robot = Node(
        package='field_robot',
        executable='robot_spawner.py',
        namespace='robot',
        name='robot_spawner',
        parameters=[
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

    operating_services = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'operating_services.launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config_path],
    )

    return LaunchDescription([
        # General Parameters
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('urdf', default_value=open(urdf_path, 'r').read()),

        # Gazebo Parameters
        DeclareLaunchArgument('world_path', default_value=os.path.join(pkg_share, 'worlds', 'main.world')),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('pause', default_value='false'),
        DeclareLaunchArgument('debug', default_value='true'),

        # actual launch
        gazebo,
        rviz,
        robot,
        operating_services,
        robot_state_publisher,
    ])

