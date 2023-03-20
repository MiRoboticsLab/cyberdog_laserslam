#!/usr/bin/python3
#
# Copyright (c) 2023 Xiaomi Corporation
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
from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml

import os


def generate_launch_description():
    share_dir = get_package_share_directory('laser_slam')
    localization_parameter_file = LaunchConfiguration('localization_params_file')
    namespace = LaunchConfiguration('namespace', default='')
    imu_topic = LaunchConfiguration('imu_topic')
    param_substitutions = {
        'imu_topic': imu_topic
    }

    localization_configured_params = RewrittenYaml(
            source_file=localization_parameter_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True)

    localization_params_declare = DeclareLaunchArgument('localization_params_file',
                                                        default_value=os.path.join(
                                                         share_dir, 'param', 'localization.yaml'),
                                                        description='Path to Ros Parameter')

    rew_loc = DeclareLaunchArgument(
            'imu_topic', default_value='/camera/imu',
            description='Use simulation (Gazebo) clock if true')

    driver_node = LifecycleNode(package='laser_slam',
                                executable='localization',
                                name='localization_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[localization_configured_params],
                                namespace=namespace,
                                )

    return LaunchDescription([
        rew_loc,
        localization_params_declare,
        driver_node,
    ])
