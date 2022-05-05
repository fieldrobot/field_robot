#!/usr/bin/python3

from http.server import executable
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

    ### DEMO CAMERA SOURCE
    image_path = os.path.join(get_package_share_directory('field_robot'), 'config/test_files/camera_front_ai.png')
    demo_camera_source = Node(
        package='field_robot',
        executable='demo_image_publisher.py',
        namespace='robot/camera_front',
        name='camera_demo',
        parameters=[
            {'image_src': image_path},
            {'sub_topic': '/image_raw'},
            {'image_dst' : '/image_ai'},
        ]
    )

    ### AI PROCESSING
    demo_ai = Node(
        package='field_robot',
        executable='image_path_finder_ai.py',
        namespace='robot/demo_camera',
        name='image_path_finder_ai_demo',
        parameters=[
            {'image_src' : '/image_raw'},
            {'image_dst' : '/image_ai'},
        ]
    )

    front_ai = Node(
        package='field_robot',
        executable='image_path_finder_ai.py',
        namespace='robot/camera_front',
        name='image_path_finder_ai_front',
        parameters=[
            {'image_src' : '/image_raw'},
            {'image_dst' : '/image_ai'},
        ]
    )
    
    rear_ai = Node(
        package='field_robot',
        executable='image_path_finder_ai.py',
        namespace='robot/camera_rear',
        name='image_path_finder_ai_rear',
        parameters=[
           {'image_src' : '/image_raw'},
           {'image_dst' : '/image_ai'},
       ]
    ),
    
    left_ai = Node(
        package='field_robot',
        executable='image_path_finder_ai.py',
        namespace='robot/camera_left',
        name='image_path_finder_ai_left',
        parameters=[
            {'image_src' : '/image_raw'},
            {'image_dst' : '/image_ai'},
        ]
    ),
    
    right_ai = Node(
        package='field_robot',
        executable='image_path_finder_ai.py',
        namespace='robot/camera_right',
        name='image_path_finder_ai_right',
        parameters=[
            {'image_src' : '/image_raw'},
            {'image_dst' : '/image_ai'},
        ]
    ),

    ### POINT CLOUD GENERATOR
    point_cloud_generator_demo = Node(
        package='field_robot',
        executable='point_cloud_generator',
        namespace='robot/demo_camera',
        name='point_cloud_generator_demo',
        parameters=[
            {'image_src' : '/image_ai'},
            {'pc_dst' : '/point_cloud'},
            {'border_image' : '/border_img'},
        ]
    )

    point_cloud_generator_front = Node(
        package='field_robot',
        executable='point_cloud_generator',
        namespace='robot/camera_front',
        name='point_cloud_generator_front',
        parameters=[
            {'image_src' : '/image_ai'},
            {'pc_dst' : '/pc'},
            {'camera_info' : '/camera_info'},
        ]
    )

    point_cloud_generator_rear = Node(
        package='field_robot',
        executable='point_cloud_generator.py',
        namespace='robot/camera_rear',
        name='point_cloud_generator_rear',
        parameters=[
            {'image_src' : '/image_ai'},
            {'pc_dst' : '/pc'},
            {'camera_info' : '/camera_info'},
        ]
    )

    point_cloud_generator_left = Node(
        package='field_robot',
        executable='point_cloud_generator.py',
        namespace='robot/camera_left',
        name='point_cloud_generator_left',
        parameters=[
            {'image_src' : '/image_ai'},
            {'pc_dst' : '/pc'},
            {'camera_info' : '/camera_info'},
        ]
    )

    point_cloud_generator_right = Node(
        package='field_robot',
        executable='point_cloud_generator.py',
        namespace='robot/camera_right',
        name='point_cloud_generator_right',
        parameters=[
            {'image_src' : '/image_ai'},
            {'pc_dst' : '/pc'},
            {'camera_info' : '/camera_info'},
        ]
    )

    ### POINT CLOUD FUSION
    point_cloud_fusion = Node(
        package='field_robot',
        executable='point_cloud_fusion.py',
        namespace='robot',
        name='point_cloud_fusion',
        parameters=[
            {'pc_src_1' : '/camera_front/pc'},
            {'pc_src_2' : '/camera_rear/pc'},
            {'pc_src_3' : '/camera_left/pc'},
            {'pc_src_4' : '/camera_right/pc'},
            {'pc_dst_5' : '/demo_camera/pc'},
            {'dst_pc': '/pc'},
        ]
    )

    ### POINT CLOUD TO LASERSCAN
    point_cloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan',
        namespace='robot',
        name='pointcloud_to_laserscan',
        parameters=[
            
        ],
        remappings=[
            ('scan', 'ls'),
            ('cloud_in', 'pc'),
        ]
    )

    return LaunchDescription([
        use_sim_time,

        # development
        demo_camera_source,

        # image path finder AI    
        #demo_ai,    
        #front_ai,
        #rear_ai,
        #left_ai,
        #right_ai,

        # point cloud generator,
        #point_cloud_generator_demo,
        point_cloud_generator_front,
        #point_cloud_generator_rear,
        #point_cloud_generator_left,
        #point_cloud_generator_right,

        # point cloud fusion,
        #point_cloud_fusion,
    ])
