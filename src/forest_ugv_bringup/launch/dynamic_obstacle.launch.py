#!/usr/bin/env python3

from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_bringup = get_package_share_directory('forest_ugv_bringup')

    mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(pkg_bringup, 'launch', 'mission.launch.py')
        ),
        launch_arguments={
            'scenario': 'forest_dynamic_obstacle',
            'gps_enabled': 'false',
            'rviz': 'true'
        }.items()
    )

    return LaunchDescription([
        mission,
    ])
