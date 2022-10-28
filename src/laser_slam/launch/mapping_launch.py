#!/usr/bin/python3
# Copyright 2020, EAIBOT
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

from http.server import executable
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo
from nav2_common.launch import RewrittenYaml

import lifecycle_msgs.msg
import os


def generate_launch_description():
    share_dir = get_package_share_directory('laser_slam')
    mapping_parameter_file = LaunchConfiguration('mapping_params_file')
    namespace = LaunchConfiguration('namespace',default='')
    imu_topic = LaunchConfiguration('imu_topic')
    param_substitutions = {
        'imu_topic': imu_topic
   }


    mapping_configured_params = RewrittenYaml(
            source_file=mapping_parameter_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True)
    
    rew = DeclareLaunchArgument(
            'imu_topic', default_value='/camera/imu',
            description='Use simulation (Gazebo) clock if true')


    mapping_params_declare = DeclareLaunchArgument('mapping_params_file',
                                           default_value=os.path.join(
                                               share_dir, 'param', 'mapping_node.yaml'),
                                           description='FPath to the ROS2 parameters file to use.')

    driver_node = LifecycleNode(package='laser_slam',
                                executable='mapping',
                                name='map_builder',
                                output='screen',
                                emulate_tty=True,
                                namespace=namespace,
                                #parameters=[parameter_file]
                                parameters=[mapping_configured_params],
                                #namespace=get_namespace()
                                )

    return LaunchDescription([
        rew,
        mapping_params_declare,
        driver_node,
    ])