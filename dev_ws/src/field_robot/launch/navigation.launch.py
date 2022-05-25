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
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    autostart = LaunchConfiguration('autostart')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')

    ###### GENERAL PARAMETERS ######

    namespace = DeclareLaunchArgument(
            'namespace', default_value='/robot',
            description='Top-level namespace')
    use_sim_time = DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true')

    ###### CUSTOM NAVIGATION ACTION SERVERS ######

    ###### NAVIGATION BEHAVIOR TREE ######

    behavior_tree = Node(
        package='field_robot',
        executable='navigation_bt',
        namespace='robot',
        name='navigation_bt',
        parameters=[os.path.join(get_package_share_directory('field_robot'), 'config', 'navigation_params.yaml')],
    )

    ###### NAV2 CONFIGURATION ######

    remappings = [('/robot/tf', '/tf'),
                  ('/robot/tf_static', '/tf_static')]
    
    nav2_autostart = DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack')
    
    nav2_params_file = DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(get_package_share_directory('field_robot'), 'config', 'nav2_parameters.yaml'),
            description='Full path to the ROS2 parameters file to use')
    
    nav2_bt_file = DeclareLaunchArgument(
            'default_bt_xml_filename',
            default_value=os.path.join(get_package_share_directory('field_robot'), "config", 'nav2_bt_config.xml'),
            description='Full path to the behavior tree xml file to use')
    
    nav2_transient_local = DeclareLaunchArgument(
            'map_subscribe_transient_local', default_value='false',
            description='Whether to set the map subscriber QoS to transient local')

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'default_bt_xml_filename': default_bt_xml_filename,
        'autostart': autostart,
        'map_subscribe_transient_local': map_subscribe_transient_local}

    configured_params = RewrittenYaml(
            source_file=LaunchConfiguration('params_file'),
            root_key=LaunchConfiguration('namespace'),
            param_rewrites=param_substitutions,
            convert_types=True)
    
    ###### NAV2 DRIVE NODES ######

    nav2_lifecycle_manager = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                        {'autostart': autostart},
                        {'node_names': [
                            'controller_server',
                            'planner_server',
                            'recoveries_server'
                        ]}],
            remappings=remappings,
            namespace=LaunchConfiguration('namespace'))
    nav2_controller = Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings,
            namespace=LaunchConfiguration('namespace'))
    nav2_planner = Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings,
            namespace=LaunchConfiguration('namespace'))
    nav2_recoveries = Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings,
            namespace=LaunchConfiguration('namespace'))


    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        ###### general configuration ######
        namespace,
        use_sim_time,

        ###### nav2 configuration ######
        nav2_autostart,
        nav2_params_file,
        #nav2_bt_file,
        nav2_transient_local,

        ###### nav2 nodes ######
        nav2_controller,
        nav2_planner,
        nav2_recoveries,
        nav2_lifecycle_manager,

        ###### navigation behavior tree ######
        behavior_tree,

    ])
