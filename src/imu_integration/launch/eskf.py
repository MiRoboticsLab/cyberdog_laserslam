from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo

import lifecycle_msgs.msg
import os


def generate_launch_description():
    share_dir = get_package_share_directory('imu_integration')
    parameter_file = LaunchConfiguration('params_file')
    node_name = 'imu_integration_node'

    params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                               share_dir, 'params', 'eskf_midpoint.yaml'),
                                           description='FPath to the ROS2 parameters file to use.')

    imu_integration_node = LifecycleNode(package='imu_integration',
                                node_executable='imu_integration_node',
                                node_name='imu_integration_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[parameter_file],
                                node_namespace='/',
                                )

    return LaunchDescription([
        params_declare,
        imu_integration_node,
    ])
