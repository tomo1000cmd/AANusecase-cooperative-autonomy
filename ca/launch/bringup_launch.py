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
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace
import time,os
import xml.etree.ElementTree as ET

def get_setting():
    using_sim_time = "false"
    use_case = "lely"
    try:
        bringup_dir = get_package_share_directory('fa')
        path= bringup_dir + '/config/' + 'setting' + '.xml'
            
        tree = ET.parse(path)
        root = tree.getroot()
        
        for data in root:
            onedict = data.attrib   
            using_sim_time = onedict['using_sim_time']
            use_case = onedict['use_case']  
    except BaseException as e:
        print(e) 
    print("[INFO] [SETTING]:the configration for sim time is: "+using_sim_time+", use case is "+ use_case)
    return using_sim_time,use_case


def generate_launch_description():

    # The ID tag of this vehicle. Must match with the namespace in the params file!
    program_id = 'server'

    # Get the launch directory
    bringup_dir = get_package_share_directory('ca')
    launch_dir = os.path.join(bringup_dir, 'launch')
    is_using_time_sim,USE_CASE = get_setting()
    bt_file = 'ca_bt.xml'
    agent_launch = 'agent_launch.py'
    if (USE_CASE == "h2trac"):
        agent_launch = 'agent_launch_h2.py'
        bt_file = 'ca_bt_h2trac.xml'
        print("[INFO] [SETTING]:H2case selected")
    # Create the launch configuration variables
    my_id = LaunchConfiguration('my_id')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    autostart = LaunchConfiguration('autostart')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_my_id_cmd = DeclareLaunchArgument(
        'my_id',
        default_value=program_id,
        description='ID of this server')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='true',
        description='Whether to apply a namespace to the agent stack')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value=is_using_time_sim,
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'params_' + program_id + '.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=os.path.join(
            get_package_share_directory('ca'),
            'behavior_trees', bt_file),
        description='Full path to the behavior tree xml file to use')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the agent stack')

    # Specify the actions
    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=my_id),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, agent_launch)),
            launch_arguments={'my_id': my_id,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'default_bt_xml_filename': default_bt_xml_filename,}.items()),          
    ])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_my_id_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_bt_xml_cmd)

    # Add the actions to launch all of the agent nodes
    ld.add_action(bringup_cmd_group)

    return ld
