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
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true'
    )
    
    world = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(get_package_share_directory('field_robot'), 'worlds', 'main.world')
    )

    urdf = DeclareLaunchArgument(
        'urdf',
        default_value=os.path.join(get_package_share_directory('field_robot'), 'models', 'robot', 'robot.urdf'),
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
        default_value='true'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
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
        arguments=[
            LaunchConfiguration('urdf')
        ],
    )

    return LaunchDescription([
        # declare launch configuration
        use_sim_time,
        world,
        gui,
        pause,
        debug,
        urdf,
        # actual launch
        gazebo,
        robot,
        robot_state_publisher,
    ])
