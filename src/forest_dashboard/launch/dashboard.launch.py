#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    dashboard_node = Node(
        package='forest_dashboard',
        executable='dashboard_server',
        name='hercules_dashboard_server',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        dashboard_node
    ])
