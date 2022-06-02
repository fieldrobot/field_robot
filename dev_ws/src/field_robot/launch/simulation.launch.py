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
    #default_rviz_config_path = os.path.join(get_package_share_directory('field_robot'), 'default.rviz')
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true'
    )
    
    world = DeclareLaunchArgument(
        'world_path',
        default_value=os.path.join(get_package_share_directory('field_robot'), 'worlds', 'main.world')
    )

    urdf_path = os.path.join(get_package_share_directory('field_robot'), 'models', 'robot', 'robot.urdf')
    urdf = DeclareLaunchArgument(
        'urdf',
        default_value=open(urdf_path,'r').read()
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

    operating_services = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('field_robot'), 'launch', 'operating_services.launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'world': LaunchConfiguration('world_path'),
            'gui': LaunchConfiguration('gui'),
            'pause': LaunchConfiguration('pause'),
            'debug': LaunchConfiguration('debug'),
            'verbose': LaunchConfiguration('debug'),
        }.items(),
    )

    return LaunchDescription([
        # General Parameters
        use_sim_time,
        urdf,

        # Gazebo Parameters
        world,
        gui,
        pause,
        debug,

        # actual launch
        gazebo,
        robot,
        operating_services,
    ])

