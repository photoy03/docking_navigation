#!/usr/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_dir = get_package_share_directory('stella_ahrs')
    config_file = os.path.join(config_dir, 'config', 'config.yaml')

    driver_node = Node(
        package='stella_ahrs',
        executable='stella_ahrs_node',
        name='stella_ahrs_node',
        namespace='/',
        parameters=[config_file],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        driver_node,
    ])
