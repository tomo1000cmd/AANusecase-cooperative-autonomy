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




from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import Node

import time,os
import xml.etree.ElementTree as ET

from launch.actions import OpaqueFunction

class Role_enum:
    chaserbin = "chaserbin"
    harvester = "harvester"

def launch_setup(context, *args, **kwargs):

    id = LaunchConfiguration('robot_id')
    id_str = str(LaunchConfiguration('robot_id').perform(context))
    role = LaunchConfiguration('role')
    role_str = str(LaunchConfiguration('role').perform(context))
    print(id_str,role_str)

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    

    
    # Declare the launch arguments  
    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Top-level namespace')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='use simulation (Gazebo) clock if true')
    voa= Node(
          package='voa',
          executable='voa_node.py',
          name=id_str + '_voa_node',
          output='screen',
          parameters=[
              {'use_sim_time': use_sim_time},
              {'my_id': id},
          ],
          remappings=[
                      # ('odometry/filtered', 'odometry/global'),
                      # ('/set_pose', '/initialpose_ekf')
                      
                      ])
    if role_str == Role_enum.chaserbin:
      start_rcl= Node(
          package='voa',
          executable='rcl_chaserbin.py',
          name=id_str + '_rcl_chaserbin',
          output='screen',
          parameters=[
              {'use_sim_time': use_sim_time},
              {'my_id': id},
          ],
          remappings=[
                      # ('odometry/filtered', 'odometry/global'),
                      # ('/set_pose', '/initialpose_ekf')
                      
                      ])
    elif role_str == Role_enum.harvester:
      start_rcl= Node(
          package='voa',
          executable='rcl_harvester.py',
          name=id_str + '_rcl_harvester',
          output='screen',
          parameters=[
              {'use_sim_time': use_sim_time},
              {'my_id': id},
          ],
          remappings=[
                      # ('odometry/filtered', 'odometry/global'),
                      # ('/set_pose', '/initialpose_ekf')
                      
                      ])
    

    return [
        declare_namespace_cmd,
        declare_use_sim_time_cmd,
        start_rcl,
        voa,
            ]

    
def generate_launch_description():



    declare_id_cmd = DeclareLaunchArgument(
        'robot_id',
        default_value='',
        description='ID of this Vehicle')
    
    declare_robot_role_cmd = DeclareLaunchArgument(
        'role',
        default_value='chaserbin',
        description='ID of this Vehicle')
 
    return LaunchDescription(
        [
            declare_id_cmd,
            declare_robot_role_cmd,
           
        ]
        + [OpaqueFunction(function=launch_setup)]
    )
 
