"""
Perception-to-Navigation Bridge Launch File.
Bridges topic remappings between physical/simulated perception pipelines and autonomous_ugv_nav.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    return LaunchDescription([
        declare_sim_time,
    ])
