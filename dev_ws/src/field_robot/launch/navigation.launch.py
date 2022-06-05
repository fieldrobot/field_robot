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
    autostart = LaunchConfiguration('autostart')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')
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

    navigation_bt_params = RewrittenYaml(
            source_file=os.path.join(get_package_share_directory('field_robot'), 'config', 'navigation_parameters.yaml'),
            root_key='robot',
            param_rewrites={
                'xml_file_path': os.path.join(get_package_share_directory('field_robot'), 'config', 'navigation_bt.xml'),
            },
            convert_types=True)

    behavior_tree = Node(
        package='field_robot',
        executable='navigation_bt',
        namespace='robot',
        name='navigation_bt',
        remappings=remappings,
        parameters=[navigation_bt_params],
    )

    ###### CUSTOM NAVIGATION ACTION SERVERS ######
    action_servers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('field_robot'), 'launch', 'navigation_servers.launch.py')),
        launch_arguments={
        }.items(),
    )

    ###### NAV2 CONFIGURATION ######
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('field_robot'), 'launch', 'navigation_nav2.launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            #'world': LaunchConfiguration('namespace'),
        }.items(),
    )


    return LaunchDescription([
        ###### general configuration ######
        namespace,
        use_sim_time,

        ###### custom navigation action servers ######
        action_servers,

        ###### NAV2 ######
        nav2,

        ###### navigation behavior tree ######
        behavior_tree,

    ])
