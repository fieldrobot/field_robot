# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    remappings = [('/robot/tf', '/tf'),
                  ('/robot/tf_static', '/tf_static')]

    ###### GENERAL PARAMETERS ######
    namespace = DeclareLaunchArgument(
            'namespace', default_value='/robot',
            description='Top-level namespace')
    use_sim_time = DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true')

    ###### NAVIGATION BEHAVIOR TREE ######
    compute_goal_in_row_server = Node(
        package='field_robot',
        executable='compute_goal_in_row_server',
        namespace='robot',
        name='compute_goal_in_row_server',
        remappings=remappings,
        parameters=[
            {'action_topic': 'navigation/compute_goal_in_row'},
            {'front_cloud_topic': 'camera_front/pc'},
            {'back_cloud_topic': 'camera_back/pc'},
            {'robot_frame': 'base_footprint'},
        ],
    )

    front_empty_server = Node(
        package='field_robot',
        executable='front_empty_server',
        namespace='robot',
        name='front_empty_server',
        remappings=remappings,
        parameters=[
            {'action_topic': 'navigation/front_empty'},
            {'left_cloud_topic': 'camera_left/pc'},
            {'right_cloud_topic': 'camera_right/pc'},
        ],
    )


    return LaunchDescription([
        ###### general configuration ######
        namespace,
        use_sim_time,

        ###### custom navigation action servers ######
        compute_goal_in_row_server,
        front_empty_server,

    ])
